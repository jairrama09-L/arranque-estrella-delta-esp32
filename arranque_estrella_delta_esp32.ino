#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <math.h>

const char* WIFI_SSID="NOMBRE_WIFI";
const char* WIFI_PASSWORD="CONTRASEÑA_WIFI";

const char* MQTT_SERVER="MQTT_SERVER.s1.eu.hivemq.cloud";
const uint16_t MQTT_PORT=8883;
const char* MQTT_USER="MQTT_USER";
const char* MQTT_PASSWORD="MQTT_PASSWORD";

const char* TOPIC_CMD="jair/motor/cmd";
const char* TOPIC_SETPOINT="jair/motor/setpoint";
const char* TOPIC_ESTADO="jair/motor/estado";
const char* TOPIC_ONLINE="jair/motor/online";
const char* TOPIC_ALARMA="jair/motor/alarma";
const char* TOPIC_CORRIENTE="jair/motor/corriente";
const char* TOPIC_TELEMETRIA="jair/motor/telemetria";

const uint8_t PIN_K1=25;
const uint8_t PIN_K2=26;
const uint8_t PIN_K3=27;
const uint8_t PIN_LED=2;
const uint8_t PIN_SENSOR=34;

const uint8_t SSR_ON=LOW;
const uint8_t SSR_OFF=HIGH;

unsigned long tiempoEstrellaMs=4000;

const unsigned long TIEMPO_MUERTO_MS=300;
const unsigned long INTERVALO_TELEMETRIA_MS=1000;
const unsigned long INTERVALO_MQTT_MS=5000;
const unsigned long INTERVALO_WIFI_MS=10000;
const unsigned long TIMEOUT_COMUNICACION_MS=10000;
const unsigned long VENTANA_RMS_MS=400;

const float SENSIBILIDAD_ACS712=0.066;
const float RELACION_DIVISOR=20.0/(10.0+20.0);
const float UMBRAL_RUIDO_A=0.20;

const float CORRIENTE_NOMINAL_A=0.0;
const float MULTIPLICADOR_ADVERTENCIA=1.20;
const float MULTIPLICADOR_DISPARO=1.50;
const unsigned long TIEMPO_SOBRECORRIENTE_MS=3000;

float corrienteRms=0.0;
float voltajeCeroACS=2.5;
int ultimaLecturaADC=0;

double sumaCuadrados=0.0;
unsigned long cantidadMuestras=0;
unsigned long inicioVentanaRms=0;

enum EstadoMotor{
  DETENIDO,
  ESTRELLA,
  TIEMPO_MUERTO,
  DELTA,
  FALLO
};

EstadoMotor estadoActual=DETENIDO;

unsigned long instanteEstado=0;
unsigned long ultimaTelemetria=0;
unsigned long ultimoIntentoMQTT=0;
unsigned long ultimoIntentoWiFi=0;
unsigned long instantePerdidaComunicacion=0;
unsigned long instanteLed=0;
unsigned long instanteSobrecorriente=0;

bool emergenciaActiva=false;
bool estadoLed=false;
bool estadoK1=false;
bool estadoK2=false;
bool estadoK3=false;
bool sobrecorrienteTemporizando=false;
bool advertenciaCorrienteActiva=false;

WiFiClientSecure clienteSeguro;
PubSubClient mqtt(clienteSeguro);

const char* nombreEstado(EstadoMotor estado){
  switch(estado){
    case DETENIDO:
      return "DETENIDO";
    case ESTRELLA:
      return "ESTRELLA";
    case TIEMPO_MUERTO:
      return "TIEMPO_MUERTO";
    case DELTA:
      return "DELTA";
    case FALLO:
      return "FALLO";
    default:
      return "DESCONOCIDO";
  }
}

bool motorActivo(){
  return estadoActual==ESTRELLA||
         estadoActual==TIEMPO_MUERTO||
         estadoActual==DELTA;
}

void escribirSalida(uint8_t pin,bool activar){
  digitalWrite(pin,activar?SSR_ON:SSR_OFF);
}

void apagarTodo(){
  estadoK1=false;
  estadoK2=false;
  estadoK3=false;

  escribirSalida(PIN_K1,false);
  escribirSalida(PIN_K2,false);
  escribirSalida(PIN_K3,false);
}

bool aplicarSalidas(bool linea,bool delta,bool estrella){
  if(delta&&estrella){
    apagarTodo();
    return false;
  }

  estadoK1=linea;
  estadoK2=delta;
  estadoK3=estrella;

  escribirSalida(PIN_K1,linea);
  escribirSalida(PIN_K2,delta);
  escribirSalida(PIN_K3,estrella);

  return true;
}

void publicarAlarma(const char* alarma){
  if(mqtt.connected()){
    mqtt.publish(TOPIC_ALARMA,alarma,true);
  }
}

void publicarEstado(){
  if(mqtt.connected()){
    mqtt.publish(
      TOPIC_ESTADO,
      nombreEstado(estadoActual),
      true
    );
  }
}

void mostrarEstado(){
  Serial.println();

  Serial.print("ESTADO: ");
  Serial.println(nombreEstado(estadoActual));

  Serial.print("K1 Linea: ");
  Serial.println(estadoK1?"ON":"OFF");

  Serial.print("K2 Delta: ");
  Serial.println(estadoK2?"ON":"OFF");

  Serial.print("K3 Estrella: ");
  Serial.println(estadoK3?"ON":"OFF");

  Serial.print("Tiempo estrella: ");
  Serial.print(tiempoEstrellaMs/1000UL);
  Serial.println(" s");

  Serial.print("Corriente RMS: ");
  Serial.print(corrienteRms,2);
  Serial.println(" A");
}

void cambiarEstado(EstadoMotor nuevoEstado){
  estadoActual=nuevoEstado;
  instanteEstado=millis();

  bool configuracionValida=true;

  switch(estadoActual){
    case DETENIDO:
      configuracionValida=
        aplicarSalidas(false,false,false);
      break;

    case ESTRELLA:
      configuracionValida=
        aplicarSalidas(true,false,true);
      break;

    case TIEMPO_MUERTO:
      configuracionValida=
        aplicarSalidas(true,false,false);
      break;

    case DELTA:
      configuracionValida=
        aplicarSalidas(true,true,false);
      break;

    case FALLO:
      configuracionValida=
        aplicarSalidas(false,false,false);
      break;
  }

  if(!configuracionValida){
    emergenciaActiva=true;
    estadoActual=FALLO;
    instanteEstado=millis();

    apagarTodo();
    publicarAlarma("INTERLOCK_K2_K3");
  }

  mostrarEstado();
  publicarEstado();
}

void iniciarMotor(){
  if(emergenciaActiva||estadoActual==FALLO){
    Serial.println("START rechazado: fallo activo.");
    publicarAlarma("START_RECHAZADO_FALLO");
    return;
  }

  if(estadoActual!=DETENIDO){
    Serial.println("START rechazado: motor no detenido.");
    return;
  }

  if(WiFi.status()!=WL_CONNECTED||!mqtt.connected()){
    Serial.println("START rechazado: sin comunicacion MQTT.");
    publicarAlarma("START_RECHAZADO_SIN_MQTT");
    return;
  }

  Serial.println("START aceptado.");

  sobrecorrienteTemporizando=false;
  advertenciaCorrienteActiva=false;
  instanteSobrecorriente=0;

  publicarAlarma("SIN_ALARMA");
  cambiarEstado(ESTRELLA);
}

void detenerMotor(){
  Serial.println("STOP recibido.");

  emergenciaActiva=false;
  sobrecorrienteTemporizando=false;
  advertenciaCorrienteActiva=false;
  instanteSobrecorriente=0;

  publicarAlarma("SIN_ALARMA");
  cambiarEstado(DETENIDO);
}

void activarFallo(const char* motivo){
  Serial.print("FALLO: ");
  Serial.println(motivo);

  emergenciaActiva=true;
  sobrecorrienteTemporizando=false;
  advertenciaCorrienteActiva=false;
  instanteSobrecorriente=0;

  cambiarEstado(FALLO);
  publicarAlarma(motivo);
}

void rearmarSistema(){
  if(estadoActual!=FALLO){
    Serial.println("RESET rechazado: no hay fallo.");
    return;
  }

  if(
    CORRIENTE_NOMINAL_A>0.0&&
    corrienteRms>CORRIENTE_NOMINAL_A
  ){
    Serial.println("RESET rechazado: corriente elevada.");
    publicarAlarma("RESET_RECHAZADO_CORRIENTE");
    return;
  }

  if(WiFi.status()!=WL_CONNECTED||!mqtt.connected()){
    Serial.println("RESET rechazado: sin comunicacion.");
    return;
  }

  emergenciaActiva=false;
  sobrecorrienteTemporizando=false;
  advertenciaCorrienteActiva=false;
  instanteSobrecorriente=0;

  publicarAlarma("SIN_ALARMA");
  cambiarEstado(DETENIDO);

  Serial.println("Sistema rearmado.");
}

void ejecutarFSM(){
  unsigned long ahora=millis();

  switch(estadoActual){
    case DETENIDO:
      break;

    case ESTRELLA:
      if(ahora-instanteEstado>=tiempoEstrellaMs){
        cambiarEstado(TIEMPO_MUERTO);
      }
      break;

    case TIEMPO_MUERTO:
      if(ahora-instanteEstado>=TIEMPO_MUERTO_MS){
        cambiarEstado(DELTA);
      }
      break;

    case DELTA:
      break;

    case FALLO:
      break;
  }
}

void reiniciarVentanaRMS(){
  sumaCuadrados=0.0;
  cantidadMuestras=0;
  inicioVentanaRms=millis();
}

void calibrarCeroACS712(){
  apagarTodo();
  corrienteRms=0.0;

  Serial.println();
  Serial.println("Calibrando cero del ACS712-30A.");
  Serial.println("El motor debe estar apagado y sin corriente.");

  const int numeroMuestras=1000;
  double sumaVoltajes=0.0;

  delay(1000);

  for(int i=0;i<numeroMuestras;i++){
    uint32_t milivoltios=
      analogReadMilliVolts(PIN_SENSOR);

    float voltajeGPIO=
      milivoltios/1000.0;

    float voltajeACS=
      voltajeGPIO/RELACION_DIVISOR;

    sumaVoltajes+=voltajeACS;

    delay(2);
  }

  voltajeCeroACS=
    sumaVoltajes/(double)numeroMuestras;

  Serial.print("Voltaje de cero calibrado: ");
  Serial.print(voltajeCeroACS,4);
  Serial.println(" V");

  reiniciarVentanaRMS();
}

void actualizarCorriente(){
  unsigned long ahora=millis();

  ultimaLecturaADC=
    analogRead(PIN_SENSOR);

  uint32_t milivoltios=
    analogReadMilliVolts(PIN_SENSOR);

  float voltajeGPIO=
    milivoltios/1000.0;

  float voltajeACS=
    voltajeGPIO/RELACION_DIVISOR;

  float componenteAC=
    voltajeACS-voltajeCeroACS;

  sumaCuadrados+=
    (double)componenteAC*
    (double)componenteAC;

  cantidadMuestras++;

  if(inicioVentanaRms==0){
    inicioVentanaRms=ahora;
  }

  if(ahora-inicioVentanaRms>=VENTANA_RMS_MS){
    if(cantidadMuestras>0){
      double promedioCuadrados=
        sumaCuadrados/(double)cantidadMuestras;

      float voltajeRms=
        sqrt(promedioCuadrados);

      corrienteRms=
        voltajeRms/SENSIBILIDAD_ACS712;

      if(corrienteRms<UMBRAL_RUIDO_A){
        corrienteRms=0.0;
      }
    }

    reiniciarVentanaRMS();
  }
}

void verificarSobrecorriente(){
  if(
    !motorActivo()||
    CORRIENTE_NOMINAL_A<=0.0
  ){
    sobrecorrienteTemporizando=false;
    advertenciaCorrienteActiva=false;
    instanteSobrecorriente=0;
    return;
  }

  float umbralAdvertencia=
    CORRIENTE_NOMINAL_A*
    MULTIPLICADOR_ADVERTENCIA;

  float umbralDisparo=
    CORRIENTE_NOMINAL_A*
    MULTIPLICADOR_DISPARO;

  if(corrienteRms>=umbralDisparo){
    advertenciaCorrienteActiva=false;

    if(!sobrecorrienteTemporizando){
      sobrecorrienteTemporizando=true;
      instanteSobrecorriente=millis();

      Serial.println("Sobrecorriente detectada.");

      publicarAlarma(
        "SOBRECORRIENTE_TEMPORIZANDO"
      );
    }

    if(
      millis()-instanteSobrecorriente>=
      TIEMPO_SOBRECORRIENTE_MS
    ){
      activarFallo("FALLO_SOBRECORRIENTE");
    }

    return;
  }

  sobrecorrienteTemporizando=false;
  instanteSobrecorriente=0;

  if(corrienteRms>=umbralAdvertencia){
    if(!advertenciaCorrienteActiva){
      advertenciaCorrienteActiva=true;
      publicarAlarma("ADVERTENCIA_CORRIENTE");
    }
  }else{
    if(advertenciaCorrienteActiva){
      advertenciaCorrienteActiva=false;
      publicarAlarma("SIN_ALARMA");
    }
  }
}

String leerPayload(
  byte* payload,
  unsigned int length
){
  String mensaje="";

  for(unsigned int i=0;i<length;i++){
    mensaje+=(char)payload[i];
  }

  mensaje.trim();
  mensaje.toUpperCase();

  return mensaje;
}

void callbackMQTT(
  char* topic,
  byte* payload,
  unsigned int length
){
  String mensaje=
    leerPayload(payload,length);

  Serial.print("MQTT [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(mensaje);

  if(String(topic)==TOPIC_CMD){
    if(mensaje=="START"){
      iniciarMotor();
    }
    else if(mensaje=="STOP"){
      detenerMotor();
    }
    else if(mensaje=="EMERGENCY"){
      activarFallo("PARADA_REMOTA");
    }
    else if(mensaje=="RESET"){
      rearmarSistema();
    }
    else if(mensaje=="CALIBRATE"){
      if(estadoActual==DETENIDO){
        calibrarCeroACS712();
        publicarAlarma("ACS712_CALIBRADO");
      }else{
        publicarAlarma("CALIBRACION_RECHAZADA");
      }
    }
    else{
      publicarAlarma("COMANDO_INVALIDO");
    }
  }
  else if(String(topic)==TOPIC_SETPOINT){
    int segundos=mensaje.toInt();

    if(segundos>=2&&segundos<=10){
      tiempoEstrellaMs=
        (unsigned long)segundos*1000UL;

      Serial.print("Tiempo estrella actualizado: ");
      Serial.print(segundos);
      Serial.println(" s");
    }else{
      publicarAlarma("SETPOINT_INVALIDO");
    }
  }
}

void iniciarWiFi(){
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);

  WiFi.begin(
    WIFI_SSID,
    WIFI_PASSWORD
  );

  Serial.print("Conectando a WiFi");

  unsigned long inicio=millis();

  while(
    WiFi.status()!=WL_CONNECTED&&
    millis()-inicio<20000
  ){
    delay(500);
    Serial.print(".");
  }

  if(WiFi.status()==WL_CONNECTED){
    Serial.println();
    Serial.println("WiFi conectado.");

    Serial.print("Direccion IP: ");
    Serial.println(WiFi.localIP());
  }else{
    Serial.println();
    Serial.println("No fue posible conectar al WiFi.");
  }
}

void mantenerWiFi(){
  if(WiFi.status()==WL_CONNECTED){
    return;
  }

  unsigned long ahora=millis();

  if(
    ahora-ultimoIntentoWiFi<
    INTERVALO_WIFI_MS
  ){
    return;
  }

  ultimoIntentoWiFi=ahora;

  Serial.println("Intentando reconectar WiFi.");

  WiFi.disconnect();

  WiFi.begin(
    WIFI_SSID,
    WIFI_PASSWORD
  );
}

void conectarMQTT(){
  if(
    mqtt.connected()||
    WiFi.status()!=WL_CONNECTED
  ){
    return;
  }

  unsigned long ahora=millis();

  if(
    ahora-ultimoIntentoMQTT<
    INTERVALO_MQTT_MS
  ){
    return;
  }

  ultimoIntentoMQTT=ahora;

  String clientId="ESP32-Jair-";
  uint64_t chipId=ESP.getEfuseMac();

  clientId+=String(
    (uint32_t)(chipId>>32),
    HEX
  );

  clientId+=String(
    (uint32_t)chipId,
    HEX
  );

  Serial.print("Conectando a HiveMQ... ");

  bool conectado=mqtt.connect(
    clientId.c_str(),
    MQTT_USER,
    MQTT_PASSWORD,
    TOPIC_ONLINE,
    1,
    true,
    "OFFLINE"
  );

  if(conectado){
    Serial.println("MQTT conectado.");

    mqtt.subscribe(TOPIC_CMD,1);
    mqtt.subscribe(TOPIC_SETPOINT,1);

    mqtt.publish(
      TOPIC_ONLINE,
      "ONLINE",
      true
    );

    publicarEstado();

    publicarAlarma(
      emergenciaActiva?
      "FALLO_ACTIVO":
      "SIN_ALARMA"
    );

    instantePerdidaComunicacion=0;

    Serial.println(
      "Suscrito a comandos y setpoint."
    );
  }else{
    Serial.print("Fallo MQTT. Codigo: ");
    Serial.println(mqtt.state());
  }
}

void verificarComunicacion(){
  bool comunicacionCorrecta=
    WiFi.status()==WL_CONNECTED&&
    mqtt.connected();

  if(comunicacionCorrecta){
    instantePerdidaComunicacion=0;
    return;
  }

  if(!motorActivo()){
    instantePerdidaComunicacion=0;
    return;
  }

  if(instantePerdidaComunicacion==0){
    instantePerdidaComunicacion=millis();
  }

  if(
    millis()-instantePerdidaComunicacion>=
    TIMEOUT_COMUNICACION_MS
  ){
    instantePerdidaComunicacion=0;
    activarFallo("PERDIDA_COMUNICACION");
  }
}

void publicarTelemetria(){
  if(!mqtt.connected()){
    return;
  }

  unsigned long ahora=millis();

  if(
    ahora-ultimaTelemetria<
    INTERVALO_TELEMETRIA_MS
  ){
    return;
  }

  ultimaTelemetria=ahora;

  char corrienteTexto[20];

  snprintf(
    corrienteTexto,
    sizeof(corrienteTexto),
    "%.2f",
    corrienteRms
  );

  mqtt.publish(
    TOPIC_CORRIENTE,
    corrienteTexto,
    false
  );

  char mensaje[450];

  snprintf(
    mensaje,
    sizeof(mensaje),
    "{\"estado\":\"%s\","
    "\"rampa\":%lu,"
    "\"tiempo_muerto_ms\":%lu,"
    "\"corriente\":%.2f,"
    "\"corriente_nominal\":%.2f,"
    "\"sensor\":\"ACS712_30A\","
    "\"adc\":%d,"
    "\"voltaje_cero\":%.4f,"
    "\"k1\":%s,"
    "\"k2\":%s,"
    "\"k3\":%s,"
    "\"emergencia\":%s,"
    "\"sobrecorriente\":%s}",
    nombreEstado(estadoActual),
    tiempoEstrellaMs/1000UL,
    TIEMPO_MUERTO_MS,
    corrienteRms,
    CORRIENTE_NOMINAL_A,
    ultimaLecturaADC,
    voltajeCeroACS,
    estadoK1?"true":"false",
    estadoK2?"true":"false",
    estadoK3?"true":"false",
    emergenciaActiva?"true":"false",
    sobrecorrienteTemporizando?
      "true":"false"
  );

  mqtt.publish(
    TOPIC_TELEMETRIA,
    mensaje,
    false
  );
}

void procesarSerial(){
  if(!Serial.available()){
    return;
  }

  char comando=Serial.read();

  if(comando=='S'||comando=='s'){
    iniciarMotor();
  }
  else if(comando=='P'||comando=='p'){
    detenerMotor();
  }
  else if(comando=='E'||comando=='e'){
    activarFallo("PARADA_SERIAL");
  }
  else if(comando=='R'||comando=='r'){
    rearmarSistema();
  }
  else if(comando=='C'||comando=='c'){
    if(estadoActual==DETENIDO){
      calibrarCeroACS712();
      publicarAlarma("ACS712_CALIBRADO");
    }else{
      Serial.println(
        "Calibracion rechazada: motor no detenido."
      );
    }
  }
}

void actualizarLed(){
  unsigned long ahora=millis();
  unsigned long intervalo=1000;

  if(estadoActual==ESTRELLA){
    intervalo=250;
  }
  else if(estadoActual==TIEMPO_MUERTO){
    intervalo=100;
  }
  else if(estadoActual==DELTA){
    intervalo=500;
  }
  else if(estadoActual==FALLO){
    intervalo=100;
  }

  if(ahora-instanteLed>=intervalo){
    instanteLed=ahora;
    estadoLed=!estadoLed;

    digitalWrite(
      PIN_LED,
      estadoLed
    );
  }
}

void setup(){
  digitalWrite(PIN_K1,SSR_OFF);
  digitalWrite(PIN_K2,SSR_OFF);
  digitalWrite(PIN_K3,SSR_OFF);

  pinMode(PIN_K1,OUTPUT);
  pinMode(PIN_K2,OUTPUT);
  pinMode(PIN_K3,OUTPUT);
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_SENSOR,INPUT);

  apagarTodo();
  digitalWrite(PIN_LED,LOW);

  Serial.begin(115200);
  delay(1000);

  analogReadResolution(12);

  analogSetPinAttenuation(
    PIN_SENSOR,
    ADC_11db
  );

  clienteSeguro.setInsecure();

  mqtt.setServer(
    MQTT_SERVER,
    MQTT_PORT
  );

  mqtt.setCallback(callbackMQTT);
  mqtt.setBufferSize(512);
  mqtt.setKeepAlive(60);

  estadoActual=DETENIDO;
  emergenciaActiva=false;
  instanteEstado=millis();

  Serial.println();
  Serial.println(
    "CONTROL ESTRELLA-DELTA CON ACS712-30A"
  );

  calibrarCeroACS712();
  mostrarEstado();
  iniciarWiFi();
}

void loop(){
  mantenerWiFi();
  conectarMQTT();

  if(mqtt.connected()){
    mqtt.loop();
  }

  procesarSerial();
  ejecutarFSM();
  actualizarCorriente();
  verificarSobrecorriente();
  verificarComunicacion();
  publicarTelemetria();
  actualizarLed();
}
