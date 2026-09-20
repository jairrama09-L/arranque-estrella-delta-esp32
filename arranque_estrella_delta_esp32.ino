/* =============================================================================
 *  ARRANQUE ESTRELLA-DELTA CON ACCIONAMIENTO ESTATICO Y SUPERVISION MQTT
 * -----------------------------------------------------------------------------
 *  Universidad del Magdalena - Facultad de Ingenieria
 *  Programa de Ingenieria Electronica - Electronica de Potencia (Grupo 4)
 *  Docente: Ing. Jordan Dallan Guillot Fula, PhD (c)
 *
 *  Autores:  Andres Esteban Perez Barbosa  - 2023119062
 *            Jair David Rangel Martinez    - 2023119055
 *
 *  Plataforma: ESP32 DevKit V1 (Arduino core para ESP32)
 *  Libreria adicional: PubSubClient (Nick O'Leary)
 * -----------------------------------------------------------------------------
 *  QUE HACE ESTE FIRMWARE
 *
 *  Sustituye la logica cableada (pulsadores, contactos de autoenclavamiento y
 *  temporizador analogico Autonics AT8N) de un arranque estrella-delta por una
 *  maquina de estados finitos no bloqueante. Los contactores de potencia
 *  Siemens K1, K2 y K3 se conservan y se gobiernan por medio de tres reles de
 *  estado solido (SSR) que aislan galvanicamente la etapa de 3,3 V del circuito
 *  de mando en 110/220 V AC.
 *
 *  Secuencia:  DETENIDO -> ESTRELLA -> TIEMPO_MUERTO -> DELTA
 * -----------------------------------------------------------------------------
 *  CRITERIOS DE SEGURIDAD APLICADOS  (ver seccion VII del informe)
 *
 *  S1. Estado seguro en el arranque del microcontrolador. Las salidas se
 *      escriben en OFF ANTES de configurarse como salidas. Como durante el
 *      reset los GPIO quedan en alta impedancia y el modulo SSR es activo en
 *      nivel BAJO, se montan ademas tres resistencias de 10 kohm de pull-up a
 *      3,3 V en las entradas de los SSR: el pull-up externo, no el firmware,
 *      es lo que garantiza que ningun contactor cierre durante el arranque.
 *
 *  S2. Enclavamiento por software. Una unica funcion escribe las salidas y
 *      rechaza cualquier combinacion en la que K2 y K3 esten activos a la vez;
 *      si se solicita, apaga todo y fuerza el estado FALLO.
 *
 *  S3. Tiempo muerto deterministico de 300 ms. Presupuesto del peor caso:
 *      corte del SSR en el paso por cero (<= 8,3 ms a 60 Hz) + retardo de
 *      apertura del contactor 3RT2026 (4...16 ms) + tiempo de arco (10 ms)
 *      = 34,3 ms. El margen aplicado es de 8,7 veces ese valor.
 *
 *  S4. Paro de emergencia FISICO. La seta NC corta la alimentacion de las tres
 *      bobinas por hardware, sin pasar por el firmware ni por la red Wi-Fi.
 *      Su contacto auxiliar se lee en GPIO 33: si el lazo se abre (seta pulsada
 *      o cable roto) el firmware va a FALLO. Un paro remoto por Wi-Fi NO es una
 *      parada de seguridad y no sustituye a la seta.
 *
 *  S5. Failsafe de comunicaciones. Si se pierde Wi-Fi o MQTT durante mas de
 *      10 s con el motor en marcha, el firmware local va a FALLO.
 *
 *  S6. Proteccion contra mensajes MQTT retenidos. Un mensaje "START" publicado
 *      como retenido se reentregaria en cada reconexion y arrancaria el motor
 *      sin que nadie lo ordene. Por eso los comandos recibidos en los primeros
 *      3 s despues de suscribirse se descartan.
 *
 *  S7. Bloqueo de rearranque. Tras una parada no se acepta un nuevo START
 *      durante 5 s, para no reconectar sobre un motor que aun gira con flujo
 *      remanente.
 *
 *  S8. Proteccion de sobrecarga por imagen termica I2t, inhibida durante el
 *      arranque para no disparar con la corriente de irrupcion.
 *
 *  S9. Credenciales fuera del codigo fuente (credenciales.h + .gitignore) y
 *      validacion opcional del certificado del broker.
 *
 *  S10. La FSM no usa delay(): todas las temporizaciones son por millis(), de
 *       modo que STOP, EMERGENCY y el paro fisico se atienden en cualquier
 *       instante de la secuencia.
 * -----------------------------------------------------------------------------
 *  MAPA DE CONEXIONES
 *
 *    GPIO 25  -> entrada de control SSR1  -> bobina A1-A2 de K1 (linea)
 *    GPIO 26  -> entrada de control SSR2  -> bobina A1-A2 de K2 (delta)
 *    GPIO 27  -> entrada de control SSR3  -> bobina A1-A2 de K3 (estrella)
 *    GPIO 33  <- contacto auxiliar NC de la seta de emergencia (a GND)
 *    GPIO 34  <- salida del ACS712-30A a traves del divisor 10k/20k
 *    GPIO 2   -> LED indicador de estado
 *
 *  GPIO 25, 26 y 27 no son pines de arranque (strapping), por lo que su nivel
 *  no altera el modo de booteo del ESP32. GPIO 34 es solo entrada y no dispone
 *  de pull-up interno, lo cual es correcto para una entrada analogica.
 * =============================================================================
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <math.h>

/* Datos privados: copie credenciales_ejemplo.h como credenciales.h y editelo.
 * credenciales.h esta listado en .gitignore y no debe subirse al repositorio. */
#include "credenciales.h"

/* ============================ TOPICOS MQTT ================================ */
/* Se construyen a partir del prefijo definido en credenciales.h.            */
#define TOPIC_CMD         MQTT_PREFIJO "/cmd"          /* HMI  -> ESP32 */
#define TOPIC_SETPOINT    MQTT_PREFIJO "/setpoint"     /* HMI  -> ESP32 */
#define TOPIC_NOMINAL     MQTT_PREFIJO "/nominal"      /* HMI  -> ESP32 */
#define TOPIC_ESTADO      MQTT_PREFIJO "/estado"       /* ESP32 -> HMI  */
#define TOPIC_ONLINE      MQTT_PREFIJO "/online"       /* ESP32 -> HMI  */
#define TOPIC_ALARMA      MQTT_PREFIJO "/alarma"       /* ESP32 -> HMI  */
#define TOPIC_CORRIENTE   MQTT_PREFIJO "/corriente"    /* ESP32 -> HMI  */
#define TOPIC_TELEMETRIA  MQTT_PREFIJO "/telemetria"   /* ESP32 -> HMI  */

/* ============================ PINES ======================================= */
const uint8_t PIN_K1     = 25;
const uint8_t PIN_K2     = 26;
const uint8_t PIN_K3     = 27;
const uint8_t PIN_PARO   = 33;   /* contacto NC de la seta, a GND            */
const uint8_t PIN_SENSOR = 34;   /* ADC1_CH6, solo entrada                   */
const uint8_t PIN_LED    = 2;

/* El modulo SSR empleado es activo en nivel BAJO. Si se cambia por uno activo
 * en ALTO basta con invertir estas dos constantes; el resto del codigo no
 * conoce la polaridad.                                                       */
const uint8_t SSR_ON  = LOW;
const uint8_t SSR_OFF = HIGH;

/* ============================ TEMPORIZACION =============================== */
/* Rampa de estrella. Valor por defecto 4 s; la HMI lo ajusta entre 2 y 10 s. */
unsigned long tiempoEstrellaMs = 4000;
const unsigned long RAMPA_MIN_MS = 2000;
const unsigned long RAMPA_MAX_MS = 10000;

/* Tiempo muerto: ver nota S3 de la cabecera. No se deja configurable desde la
 * HMI a proposito, porque es un parametro de seguridad, no de proceso.       */
const unsigned long TIEMPO_MUERTO_MS = 300;

const unsigned long INTERVALO_TELEMETRIA_MS   = 1000;
const unsigned long INTERVALO_MQTT_MS         = 5000;  /* reintento de conexion */
const unsigned long INTERVALO_WIFI_MS         = 10000; /* reintento de Wi-Fi    */
const unsigned long TIMEOUT_COMUNICACION_MS   = 10000; /* failsafe S5           */
const unsigned long VENTANA_RETENIDOS_MS      = 3000;  /* guarda S6            */
const unsigned long BLOQUEO_REARRANQUE_MS     = 5000;  /* guarda S7            */
const unsigned long REBOTE_PARO_MS            = 50;    /* antirrebote de la seta */

/* ============================ MEDICION DE CORRIENTE ======================= */
const unsigned long PERIODO_MUESTREO_US = 500;  /* 2 kHz: 33 muestras por ciclo
                                                 * de 60 Hz. Muestreo a periodo
                                                 * fijo por micros(), no "una
                                                 * muestra por vuelta de loop",
                                                 * para que el RMS no dependa de
                                                 * la carga de trabajo.        */
const unsigned long VENTANA_RMS_MS = 400;       /* 24 ciclos a 60 Hz          */

const float SENSIBILIDAD_ACS712 = 0.066;        /* V/A, version de 30 A       */
const float RELACION_DIVISOR    = 20.0 / (10.0 + 20.0);  /* 0,667            */
const float UMBRAL_RUIDO_A      = 0.20;         /* por debajo se reporta 0    */

const int MUESTRAS_CALIBRACION = 1000;          /* cero del sensor            */

/* ============================ PROTECCIONES ================================ */
/* Corriente nominal del motor. Mientras valga 0 A todas las protecciones de
 * sobrecarga quedan inhibidas, porque disparar contra un valor inventado es
 * peor que no proteger. Se configura en caliente publicando el valor en el
 * topico .../nominal, o con el comando serial 'N'.                           */
float corrienteNominalA = 0.0;

/* Imagen termica I2t (modelo de primer orden).
 *   theta[k+1] = theta[k] + (dt/tau) * ( (I/In)^2 - theta[k] )
 * theta = 1,0 equivale al calentamiento de regimen a corriente nominal.      */
const float         THETA_DISPARO      = 1.20;
const float         THETA_ADVERTENCIA  = 0.90;
const float         THETA_REARME       = 0.60;   /* memoria termica           */
const unsigned long TAU_TERMICA_S      = 300;    /* constante de enfriamiento */

/* Respaldo instantaneo por cortocircuito. */
const float         FACTOR_CORTOCIRCUITO   = 8.0;
/* El valor eficaz se refresca cada VENTANA_RMS_MS (400 ms), de modo que este
 * temporizador no puede resolver por debajo de una ventana: el tiempo real de
 * disparo queda en el orden de 0,4 a 0,6 s. Es un respaldo del firmware, no
 * una proteccion de cortocircuito: esa la da el mini breaker.               */
const unsigned long TIEMPO_CORTOCIRCUITO_MS = 200;

/* Las protecciones se inhiben mientras dura la irrupcion de arranque. */
const unsigned long INHIBICION_ARRANQUE_MS = 500;  /* tras cerrar delta       */

/* ============================ ESTADO GLOBAL =============================== */
enum EstadoMotor { DETENIDO, ESTRELLA, TIEMPO_MUERTO, DELTA, FALLO };
EstadoMotor estadoActual = DETENIDO;

enum EstadoCalibracion { CAL_INACTIVA, CAL_MUESTREANDO };
EstadoCalibracion calibracion = CAL_INACTIVA;

/* Marcas de tiempo */
unsigned long instanteEstado             = 0;
unsigned long instanteParada             = 0;
unsigned long instanteSuscripcion        = 0;
unsigned long instantePerdidaComunicacion = 0;
unsigned long instanteCortocircuito      = 0;
unsigned long ultimaTelemetria           = 0;
unsigned long ultimoIntentoMQTT          = 0;
unsigned long ultimoIntentoWiFi          = 0;
unsigned long instanteLed                = 0;
unsigned long ultimaMuestraUs            = 0;
unsigned long inicioVentanaRms           = 0;
unsigned long ultimoCambioParo           = 0;
unsigned long instanteTermica            = 0;

/* Salidas y banderas */
bool estadoK1 = false, estadoK2 = false, estadoK3 = false;
bool emergenciaActiva      = false;
bool paroFisicoActivo      = false;
bool lecturaParoAnterior   = false;
bool advertenciaTermica    = false;
bool cortocircuitoTemporizando = false;
bool estadoLed = false;

/* Medicion */
float  corrienteRms   = 0.0;
float  voltajeCeroACS = 2.5;
float  thetaTermica   = 0.0;
double sumaCuadrados  = 0.0;
double sumaCalibracion = 0.0;
unsigned long cantidadMuestras = 0;
unsigned long muestrasCalibracion = 0;
unsigned long muestrasPorSegundo = 0;

char ultimaAlarma[32] = "SIN_ALARMA";

WiFiClientSecure clienteSeguro;
PubSubClient mqtt(clienteSeguro);

/* ========================================================================== */
/*  UTILIDADES                                                                */
/* ========================================================================== */

const char* nombreEstado(EstadoMotor estado) {
  switch (estado) {
    case DETENIDO:      return "DETENIDO";
    case ESTRELLA:      return "ESTRELLA";
    case TIEMPO_MUERTO: return "TIEMPO_MUERTO";
    case DELTA:         return "DELTA";
    case FALLO:         return "FALLO";
    default:            return "DESCONOCIDO";
  }
}

/* Estados en los que hay potencia aplicada al motor. */
bool motorActivo() {
  return estadoActual == ESTRELLA ||
         estadoActual == TIEMPO_MUERTO ||
         estadoActual == DELTA;
}

void publicarAlarma(const char* alarma) {
  snprintf(ultimaAlarma, sizeof(ultimaAlarma), "%s", alarma);
  if (mqtt.connected()) {
    mqtt.publish(TOPIC_ALARMA, alarma, true);   /* retenida: estado persistente */
  }
  Serial.print("ALARMA: ");
  Serial.println(alarma);
}

void publicarEstado() {
  if (mqtt.connected()) {
    mqtt.publish(TOPIC_ESTADO, nombreEstado(estadoActual), true);
  }
}

/* ========================================================================== */
/*  CONTROL DE SALIDAS  (S1 y S2)                                             */
/* ========================================================================== */

void escribirSalida(uint8_t pin, bool activar) {
  digitalWrite(pin, activar ? SSR_ON : SSR_OFF);
}

/* Apagado incondicional. Es la unica operacion que nunca puede fallar ni
 * depender de una condicion previa: se llama en el arranque, en FALLO y en
 * cualquier situacion dudosa.                                               */
void apagarTodo() {
  estadoK1 = estadoK2 = estadoK3 = false;
  escribirSalida(PIN_K1, false);
  escribirSalida(PIN_K2, false);
  escribirSalida(PIN_K3, false);
}

/* Unico punto del programa que energiza contactores. Devuelve false si la
 * combinacion solicitada es insegura (K2 y K3 simultaneos), en cuyo caso deja
 * todo apagado y quien la llamo debe ir a FALLO.                            */
bool aplicarSalidas(bool linea, bool delta, bool estrella) {
  if (delta && estrella) {          /* enclavamiento: cortocircuito trifasico */
    apagarTodo();
    return false;
  }

  /* El orden importa: primero se desactiva lo que debe abrir y despues se
   * activa lo que debe cerrar.                                              */
  if (!delta)    escribirSalida(PIN_K2, false);
  if (!estrella) escribirSalida(PIN_K3, false);
  escribirSalida(PIN_K1, linea);
  if (delta)     escribirSalida(PIN_K2, true);
  if (estrella)  escribirSalida(PIN_K3, true);

  estadoK1 = linea;
  estadoK2 = delta;
  estadoK3 = estrella;
  return true;
}

void mostrarEstado() {
  Serial.println();
  Serial.print("ESTADO: ");        Serial.println(nombreEstado(estadoActual));
  Serial.print("  K1 linea: ");    Serial.println(estadoK1 ? "ON" : "OFF");
  Serial.print("  K2 delta: ");    Serial.println(estadoK2 ? "ON" : "OFF");
  Serial.print("  K3 estrella: "); Serial.println(estadoK3 ? "ON" : "OFF");
  Serial.print("  Rampa: ");       Serial.print(tiempoEstrellaMs / 1000.0, 1);
  Serial.println(" s");
  Serial.print("  Corriente RMS: ");
  Serial.print(corrienteRms, 2);   Serial.println(" A");
  Serial.print("  Imagen termica: ");
  Serial.print(thetaTermica, 3);   Serial.println(" p.u.");
}

/* ========================================================================== */
/*  MAQUINA DE ESTADOS                                                        */
/* ========================================================================== */

void cambiarEstado(EstadoMotor nuevoEstado) {
  /* Si se abandona un estado con potencia, se marca el instante de parada
   * para el bloqueo de rearranque (S7).                                     */
  if (motorActivo() && (nuevoEstado == DETENIDO || nuevoEstado == FALLO)) {
    instanteParada = millis();
  }

  estadoActual   = nuevoEstado;
  instanteEstado = millis();

  bool combinacionValida = true;
  switch (estadoActual) {
    case DETENIDO:      combinacionValida = aplicarSalidas(false, false, false); break;
    case ESTRELLA:      combinacionValida = aplicarSalidas(true,  false, true);  break;
    case TIEMPO_MUERTO: combinacionValida = aplicarSalidas(true,  false, false); break;
    case DELTA:         combinacionValida = aplicarSalidas(true,  true,  false); break;
    case FALLO:         combinacionValida = aplicarSalidas(false, false, false); break;
  }

  if (!combinacionValida) {
    emergenciaActiva = true;
    estadoActual     = FALLO;
    instanteEstado   = millis();
    apagarTodo();
    publicarAlarma("ENCLAVAMIENTO_K2_K3");
  }

  mostrarEstado();
  publicarEstado();
}

void activarFallo(const char* motivo) {
  apagarTodo();                 /* primero se abre, despues se informa */
  emergenciaActiva = true;
  cortocircuitoTemporizando = false;
  cambiarEstado(FALLO);
  publicarAlarma(motivo);
}

void iniciarMotor() {
  if (paroFisicoActivo) {
    publicarAlarma("START_RECHAZADO_PARO_FISICO");
    return;
  }
  if (emergenciaActiva || estadoActual == FALLO) {
    publicarAlarma("START_RECHAZADO_FALLO");
    return;
  }
  if (estadoActual != DETENIDO) {
    Serial.println("START rechazado: el motor no esta detenido.");
    return;
  }
  if (calibracion != CAL_INACTIVA) {
    publicarAlarma("START_RECHAZADO_CALIBRANDO");
    return;
  }
  /* S7: no reconectar sobre un motor que todavia gira. */
  if (instanteParada != 0 && millis() - instanteParada < BLOQUEO_REARRANQUE_MS) {
    publicarAlarma("START_RECHAZADO_REARRANQUE");
    return;
  }
  /* Memoria termica: no rearrancar con el modelo termico todavia caliente. */
  if (corrienteNominalA > 0.0 && thetaTermica > THETA_REARME) {
    publicarAlarma("START_RECHAZADO_TERMICO");
    return;
  }
  /* Se exige enlace MQTT para arrancar: sin supervision remota no se opera.
   * El paro, en cambio, funciona siempre (local y remoto).                  */
  if (WiFi.status() != WL_CONNECTED || !mqtt.connected()) {
    Serial.println("START rechazado: sin comunicacion MQTT.");
    return;
  }

  Serial.println("START aceptado.");
  cortocircuitoTemporizando = false;
  advertenciaTermica = false;
  publicarAlarma("SIN_ALARMA");
  cambiarEstado(ESTRELLA);
}

void detenerMotor() {
  Serial.println("STOP recibido.");
  emergenciaActiva = false;
  cortocircuitoTemporizando = false;
  cambiarEstado(DETENIDO);
  publicarAlarma("SIN_ALARMA");
}

void rearmarSistema() {
  if (estadoActual != FALLO) {
    Serial.println("RESET rechazado: no hay ningun fallo activo.");
    return;
  }
  if (paroFisicoActivo) {
    publicarAlarma("RESET_RECHAZADO_PARO_FISICO");
    return;
  }
  if (corrienteRms > UMBRAL_RUIDO_A) {
    publicarAlarma("RESET_RECHAZADO_CORRIENTE");
    return;
  }
  if (corrienteNominalA > 0.0 && thetaTermica > THETA_REARME) {
    publicarAlarma("RESET_RECHAZADO_TERMICO");
    return;
  }

  emergenciaActiva = false;
  cortocircuitoTemporizando = false;
  cambiarEstado(DETENIDO);
  publicarAlarma("SIN_ALARMA");
  Serial.println("Sistema rearmado.");
}

/* La FSM propiamente dicha: solo transiciones por tiempo. Las transiciones por
 * evento (STOP, EMERGENCY, fallos) las provocan las funciones anteriores.    */
void ejecutarFSM() {
  unsigned long ahora = millis();

  switch (estadoActual) {
    case ESTRELLA:
      if (ahora - instanteEstado >= tiempoEstrellaMs) {
        cambiarEstado(TIEMPO_MUERTO);
      }
      break;

    case TIEMPO_MUERTO:
      if (ahora - instanteEstado >= TIEMPO_MUERTO_MS) {
        cambiarEstado(DELTA);
      }
      break;

    case DETENIDO:
    case DELTA:
    case FALLO:
    default:
      break;   /* estados estables: se abandonan solo por evento */
  }
}

/* ========================================================================== */
/*  PARO DE EMERGENCIA FISICO  (S4)                                           */
/* ========================================================================== */
/* El contacto NC de la seta une GPIO 33 con GND. Con el pull-up interno:
 *    seta liberada  -> contacto cerrado -> LOW  -> operacion normal
 *    seta pulsada   -> contacto abierto -> HIGH -> emergencia
 *    cable cortado  -> entrada flotante -> HIGH -> emergencia (falla segura)  */
void verificarParoFisico() {
  bool lectura = (digitalRead(PIN_PARO) == HIGH);

  if (lectura != lecturaParoAnterior) {
    lecturaParoAnterior = lectura;
    ultimoCambioParo = millis();
    return;
  }
  if (millis() - ultimoCambioParo < REBOTE_PARO_MS) return;

  if (lectura && !paroFisicoActivo) {
    paroFisicoActivo = true;
    activarFallo("PARO_FISICO_ACCIONADO");
  } else if (!lectura && paroFisicoActivo) {
    paroFisicoActivo = false;
    Serial.println("Seta de emergencia liberada. Se requiere RESET.");
    publicarAlarma("PARO_FISICO_LIBERADO_ESPERA_RESET");
  }
}

/* ========================================================================== */
/*  MEDICION DE CORRIENTE                                                     */
/* ========================================================================== */

float leerVoltajeACS() {
  /* analogReadMilliVolts aplica la curva de calibracion de fabrica del ADC,
   * que es bastante mas lineal que analogRead() en bruto.                    */
  float voltajeGPIO = analogReadMilliVolts(PIN_SENSOR) / 1000.0;
  return voltajeGPIO / RELACION_DIVISOR;   /* se deshace el divisor 10k/20k  */
}

void reiniciarVentanaRMS() {
  sumaCuadrados   = 0.0;
  cantidadMuestras = 0;
  inicioVentanaRms = millis();
}

/* Calibracion del cero del sensor. Es NO BLOQUEANTE: acumula muestras en el
 * loop principal, de modo que MQTT y el paro de emergencia siguen atendidos
 * mientras dura. Solo se permite con el motor detenido.                      */
void iniciarCalibracion() {
  if (estadoActual != DETENIDO) {
    publicarAlarma("CALIBRACION_RECHAZADA");
    return;
  }
  apagarTodo();
  sumaCalibracion = 0.0;
  muestrasCalibracion = 0;
  corrienteRms = 0.0;
  calibracion = CAL_MUESTREANDO;
  Serial.println("Calibrando el cero del ACS712 (motor detenido, sin corriente)...");
}

void procesarCalibracion() {
  sumaCalibracion += leerVoltajeACS();
  muestrasCalibracion++;

  if (muestrasCalibracion >= (unsigned long)MUESTRAS_CALIBRACION) {
    voltajeCeroACS = sumaCalibracion / (double)muestrasCalibracion;
    calibracion = CAL_INACTIVA;
    reiniciarVentanaRMS();
    Serial.print("Voltaje de cero calibrado: ");
    Serial.print(voltajeCeroACS, 4);
    Serial.println(" V");
    publicarAlarma("ACS712_CALIBRADO");
  }
}

/* Muestreo a periodo fijo y calculo del valor eficaz de la componente alterna. */
void actualizarCorriente() {
  unsigned long ahoraUs = micros();
  if (ahoraUs - ultimaMuestraUs < PERIODO_MUESTREO_US) return;
  ultimaMuestraUs = ahoraUs;

  if (calibracion == CAL_MUESTREANDO) {
    procesarCalibracion();
    return;
  }

  float componenteAC = leerVoltajeACS() - voltajeCeroACS;
  sumaCuadrados += (double)componenteAC * (double)componenteAC;
  cantidadMuestras++;

  unsigned long ahora = millis();
  if (inicioVentanaRms == 0) inicioVentanaRms = ahora;

  if (ahora - inicioVentanaRms >= VENTANA_RMS_MS) {
    if (cantidadMuestras > 0) {
      float voltajeRms = sqrt(sumaCuadrados / (double)cantidadMuestras);
      corrienteRms = voltajeRms / SENSIBILIDAD_ACS712;

      /* Por debajo del umbral la lectura es ruido del ADC, no corriente. */
      if (corrienteRms < UMBRAL_RUIDO_A) corrienteRms = 0.0;

      /* Se publica la frecuencia de muestreo real: sirve para justificar en el
       * informe cuantas muestras por ciclo sustentan el RMS.                 */
      muestrasPorSegundo = (cantidadMuestras * 1000UL) / (ahora - inicioVentanaRms);
    }
    reiniciarVentanaRMS();
  }
}

/* ========================================================================== */
/*  PROTECCIONES  (S8)                                                        */
/* ========================================================================== */

/* Durante la irrupcion de arranque la corriente es legitimamente varias veces
 * la nominal. Disparar ahi seria un falso positivo, no una proteccion.       */
bool proteccionInhibida() {
  if (!motorActivo())            return true;
  if (corrienteNominalA <= 0.0)  return true;   /* sin In no hay referencia   */
  if (estadoActual == ESTRELLA)  return true;
  if (estadoActual == TIEMPO_MUERTO) return true;
  if (estadoActual == DELTA &&
      millis() - instanteEstado < INHIBICION_ARRANQUE_MS) return true;
  return false;
}

void actualizarImagenTermica() {
  unsigned long ahora = millis();
  if (instanteTermica == 0) { instanteTermica = ahora; return; }

  float dt = (ahora - instanteTermica) / 1000.0;   /* segundos */
  if (dt < 0.1) return;
  instanteTermica = ahora;

  if (corrienteNominalA <= 0.0) { thetaTermica = 0.0; return; }

  float relacion = corrienteRms / corrienteNominalA;
  /* Modelo de primer orden: calienta con I^2 y enfria con la constante tau. */
  thetaTermica += (dt / (float)TAU_TERMICA_S) * (relacion * relacion - thetaTermica);
  if (thetaTermica < 0.0) thetaTermica = 0.0;

  if (proteccionInhibida()) return;

  if (thetaTermica >= THETA_DISPARO) {
    activarFallo("FALLO_SOBRECARGA_I2T");
    return;
  }
  if (thetaTermica >= THETA_ADVERTENCIA && !advertenciaTermica) {
    advertenciaTermica = true;
    publicarAlarma("ADVERTENCIA_TERMICA");
  } else if (thetaTermica < THETA_ADVERTENCIA && advertenciaTermica) {
    advertenciaTermica = false;
    publicarAlarma("SIN_ALARMA");
  }
}

/* Respaldo rapido: una corriente muy superior a la nominal sostenida durante
 * 100 ms no es sobrecarga, es un defecto. No espera a la imagen termica.     */
void verificarCortocircuito() {
  if (proteccionInhibida()) {
    cortocircuitoTemporizando = false;
    return;
  }

  if (corrienteRms >= FACTOR_CORTOCIRCUITO * corrienteNominalA) {
    if (!cortocircuitoTemporizando) {
      cortocircuitoTemporizando = true;
      instanteCortocircuito = millis();
    } else if (millis() - instanteCortocircuito >= TIEMPO_CORTOCIRCUITO_MS) {
      activarFallo("FALLO_CORTOCIRCUITO");
    }
  } else {
    cortocircuitoTemporizando = false;
  }
}

/* ========================================================================== */
/*  COMUNICACIONES                                                            */
/* ========================================================================== */

void procesarComando(const String& mensaje) {
  if      (mensaje == "START")     iniciarMotor();
  else if (mensaje == "STOP")      detenerMotor();
  else if (mensaje == "EMERGENCY") activarFallo("PARADA_REMOTA");
  else if (mensaje == "RESET")     rearmarSistema();
  else if (mensaje == "CALIBRATE") iniciarCalibracion();
  else                             publicarAlarma("COMANDO_INVALIDO");
}

void callbackMQTT(char* topic, byte* payload, unsigned int length) {
  /* Se limita la longitud: un payload arbitrariamente largo no debe poder
   * agotar la memoria del ESP32.                                            */
  if (length > 32) length = 32;

  String mensaje;
  mensaje.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++) mensaje += (char)payload[i];
  mensaje.trim();
  mensaje.toUpperCase();

  Serial.print("MQTT [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(mensaje);

  String t(topic);

  if (t == TOPIC_CMD) {
    /* S6: descarta comandos retenidos reentregados al reconectar. */
    if (millis() - instanteSuscripcion < VENTANA_RETENIDOS_MS) {
      Serial.println("Comando ignorado: llego dentro de la ventana de reconexion.");
      publicarAlarma("COMANDO_IGNORADO_RECONEXION");
      return;
    }
    procesarComando(mensaje);
  }
  else if (t == TOPIC_SETPOINT) {
    long segundos = mensaje.toInt();
    unsigned long valorMs = (unsigned long)segundos * 1000UL;
    if (valorMs >= RAMPA_MIN_MS && valorMs <= RAMPA_MAX_MS) {
      tiempoEstrellaMs = valorMs;
      Serial.print("Rampa de estrella actualizada: ");
      Serial.print(segundos);
      Serial.println(" s");
    } else {
      publicarAlarma("SETPOINT_INVALIDO");
    }
  }
  else if (t == TOPIC_NOMINAL) {
    float valor = mensaje.toFloat();
    /* Cota superior coherente con el fondo de escala del ACS712-30A. */
    if (valor >= 0.0 && valor <= 30.0) {
      corrienteNominalA = valor;
      thetaTermica = 0.0;
      Serial.print("Corriente nominal configurada: ");
      Serial.print(corrienteNominalA, 2);
      Serial.println(" A");
    } else {
      publicarAlarma("NOMINAL_INVALIDA");
    }
  }
}

void iniciarWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  ultimoIntentoWiFi = millis();
  Serial.println("Conectando a Wi-Fi en segundo plano...");
}

/* Reconexion no bloqueante: nunca se espera dentro de un while. */
void mantenerWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  unsigned long ahora = millis();
  if (ahora - ultimoIntentoWiFi < INTERVALO_WIFI_MS) return;
  ultimoIntentoWiFi = ahora;

  Serial.println("Reintentando la conexion Wi-Fi...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void conectarMQTT() {
  if (mqtt.connected() || WiFi.status() != WL_CONNECTED) return;

  unsigned long ahora = millis();
  if (ahora - ultimoIntentoMQTT < INTERVALO_MQTT_MS) return;
  ultimoIntentoMQTT = ahora;

  String clientId = "ESP32-EstrellaDelta-";
  uint64_t chipId = ESP.getEfuseMac();
  clientId += String((uint32_t)(chipId >> 32), HEX);
  clientId += String((uint32_t)chipId, HEX);

  Serial.print("Conectando al broker MQTT... ");

  /* Testamento (last will): si el ESP32 se cae, el broker publica OFFLINE y
   * la HMI se entera sin necesidad de sondeo.                               */
  bool conectado = mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD,
                                TOPIC_ONLINE, 1, true, "OFFLINE");

  if (!conectado) {
    Serial.print("fallo, codigo ");
    Serial.println(mqtt.state());
    return;
  }

  Serial.println("conectado.");
  mqtt.subscribe(TOPIC_CMD, 1);
  mqtt.subscribe(TOPIC_SETPOINT, 1);
  mqtt.subscribe(TOPIC_NOMINAL, 1);
  instanteSuscripcion = millis();          /* arranca la ventana de guarda S6 */

  mqtt.publish(TOPIC_ONLINE, "ONLINE", true);
  publicarEstado();
  /* Se reenvia la ultima alarma conocida sin pasar por publicarAlarma(), que
   * copiaria el buffer sobre si mismo.                                      */
  if (emergenciaActiva) {
    publicarAlarma("FALLO_ACTIVO");
  } else {
    mqtt.publish(TOPIC_ALARMA, ultimaAlarma, true);
  }
  instantePerdidaComunicacion = 0;
}

/* Failsafe S5: el control local decide por si mismo si se queda sin supervision. */
void verificarComunicacion() {
  bool enlaceCorrecto = (WiFi.status() == WL_CONNECTED) && mqtt.connected();

  if (enlaceCorrecto || !motorActivo()) {
    instantePerdidaComunicacion = 0;
    return;
  }
  if (instantePerdidaComunicacion == 0) {
    instantePerdidaComunicacion = millis();
    Serial.println("Enlace perdido con el motor en marcha. Cuenta atras del failsafe.");
    return;
  }
  if (millis() - instantePerdidaComunicacion >= TIMEOUT_COMUNICACION_MS) {
    instantePerdidaComunicacion = 0;
    activarFallo("PERDIDA_COMUNICACION");
  }
}

void publicarTelemetria() {
  if (!mqtt.connected()) return;

  unsigned long ahora = millis();
  if (ahora - ultimaTelemetria < INTERVALO_TELEMETRIA_MS) return;
  ultimaTelemetria = ahora;

  char corrienteTexto[16];
  snprintf(corrienteTexto, sizeof(corrienteTexto), "%.2f", corrienteRms);
  mqtt.publish(TOPIC_CORRIENTE, corrienteTexto, false);

  char mensaje[512];
  snprintf(mensaje, sizeof(mensaje),
    "{\"estado\":\"%s\",\"rampa_s\":%.1f,\"t_muerto_ms\":%lu,"
    "\"corriente\":%.2f,\"corriente_nominal\":%.2f,\"imagen_termica\":%.3f,"
    "\"muestras_s\":%lu,\"voltaje_cero\":%.4f,"
    "\"k1\":%s,\"k2\":%s,\"k3\":%s,"
    "\"paro_fisico\":%s,\"emergencia\":%s,\"rssi\":%d,\"alarma\":\"%s\"}",
    nombreEstado(estadoActual),
    tiempoEstrellaMs / 1000.0,
    TIEMPO_MUERTO_MS,
    corrienteRms,
    corrienteNominalA,
    thetaTermica,
    muestrasPorSegundo,
    voltajeCeroACS,
    estadoK1 ? "true" : "false",
    estadoK2 ? "true" : "false",
    estadoK3 ? "true" : "false",
    paroFisicoActivo ? "true" : "false",
    emergenciaActiva ? "true" : "false",
    WiFi.RSSI(),
    ultimaAlarma);

  mqtt.publish(TOPIC_TELEMETRIA, mensaje, false);
}

/* ========================================================================== */
/*  CONSOLA SERIAL  (respaldo local, funciona sin red)                        */
/* ========================================================================== */

void procesarSerial() {
  if (!Serial.available()) return;

  char comando = Serial.read();
  switch (comando) {
    case 'S': case 's': iniciarMotor();                 break;
    case 'P': case 'p': detenerMotor();                 break;
    case 'E': case 'e': activarFallo("PARADA_SERIAL");  break;
    case 'R': case 'r': rearmarSistema();               break;
    case 'C': case 'c': iniciarCalibracion();           break;
    case 'I': case 'i': mostrarEstado();                break;
    case 'N': case 'n': {
      /* Ejemplo:  N3.3  configura la corriente nominal en 3,3 A. */
      float valor = Serial.parseFloat();
      if (valor >= 0.0 && valor <= 30.0) {
        corrienteNominalA = valor;
        thetaTermica = 0.0;
        Serial.print("Corriente nominal configurada: ");
        Serial.print(corrienteNominalA, 2);
        Serial.println(" A");
      }
      break;
    }
    default: break;
  }
}

/* ========================================================================== */
/*  INDICADOR LUMINOSO                                                        */
/* ========================================================================== */

void actualizarLed() {
  unsigned long intervalo = 1000;          /* DETENIDO: parpadeo lento       */
  if      (estadoActual == ESTRELLA)      intervalo = 250;
  else if (estadoActual == TIEMPO_MUERTO) intervalo = 100;
  else if (estadoActual == DELTA)         intervalo = 500;
  else if (estadoActual == FALLO)         intervalo = 80;   /* muy rapido    */

  unsigned long ahora = millis();
  if (ahora - instanteLed >= intervalo) {
    instanteLed = ahora;
    estadoLed = !estadoLed;
    digitalWrite(PIN_LED, estadoLed);
  }
}

/* ========================================================================== */
/*  SETUP                                                                     */
/* ========================================================================== */

void setup() {
  /* S1: el nivel se fija ANTES de habilitar la salida, para que el pin no pase
   * por un estado activo al configurarse. El pull-up externo de 10 kohm cubre
   * el intervalo previo, mientras el GPIO todavia esta en alta impedancia.   */
  digitalWrite(PIN_K1, SSR_OFF);
  digitalWrite(PIN_K2, SSR_OFF);
  digitalWrite(PIN_K3, SSR_OFF);
  pinMode(PIN_K1, OUTPUT);
  pinMode(PIN_K2, OUTPUT);
  pinMode(PIN_K3, OUTPUT);
  apagarTodo();

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  pinMode(PIN_PARO, INPUT_PULLUP);
  pinMode(PIN_SENSOR, INPUT);

  Serial.begin(115200);
  /* Sin esto, Serial.parseFloat() bloquearia el loop hasta 1 s. */
  Serial.setTimeout(50);

  analogReadResolution(12);                         /* 0...4095              */
  analogSetPinAttenuation(PIN_SENSOR, ADC_11db);    /* fondo de escala ~3,3 V */

  Serial.println();
  Serial.println("=======================================================");
  Serial.println(" ARRANQUE ESTRELLA-DELTA  |  ESP32 + SSR + ACS712-30A");
  Serial.println(" Comandos: S start  P stop  E emergencia  R rearme");
  Serial.println("           C calibrar  I info  N<valor> corriente nominal");
  Serial.println("=======================================================");

  /* TLS: con certificado raiz se autentica el broker; sin el, solo se cifra. */
#ifdef MQTT_CA_CERT
  clienteSeguro.setCACert(MQTT_CA_CERT);
  Serial.println("TLS: validando el certificado del broker.");
#else
  clienteSeguro.setInsecure();
  Serial.println("TLS: AVISO - sin certificado raiz. El trafico va cifrado,");
  Serial.println("     pero no se verifica la identidad del broker.");
  Serial.println("     Defina MQTT_CA_CERT en credenciales.h para corregirlo.");
#endif

  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(callbackMQTT);
  mqtt.setBufferSize(768);
  mqtt.setKeepAlive(30);

  /* Estado inicial conocido y seguro. */
  estadoActual     = DETENIDO;
  emergenciaActiva = false;
  instanteEstado   = millis();
  instanteTermica  = millis();
  lecturaParoAnterior = (digitalRead(PIN_PARO) == HIGH);
  paroFisicoActivo    = lecturaParoAnterior;
  reiniciarVentanaRMS();

  if (paroFisicoActivo) {
    Serial.println("AVISO: la seta de emergencia esta accionada o su lazo abierto.");
  }

  iniciarCalibracion();   /* no bloqueante: termina dentro del loop */
  iniciarWiFi();          /* no bloqueante: la FSM ya esta operativa */
}

/* ========================================================================== */
/*  LOOP                                                                      */
/* ========================================================================== */

void loop() {
  /* Orden deliberado: primero lo que puede abrir contactores (paro fisico),
   * despues la FSM, y de ultimo lo accesorio (telemetria, LED).             */
  verificarParoFisico();

  mantenerWiFi();
  conectarMQTT();
  if (mqtt.connected()) mqtt.loop();

  procesarSerial();
  ejecutarFSM();

  actualizarCorriente();
  actualizarImagenTermica();
  verificarCortocircuito();

  verificarComunicacion();
  publicarTelemetria();
  actualizarLed();
}
