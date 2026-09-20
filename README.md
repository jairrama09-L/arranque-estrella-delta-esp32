# arranque-estrella-delta-esp32
Control IoT de arranque estrella-delta con ESP32, SSR, MQTT y ACS712.
# Arranque estrella-delta con ESP32, MQTT y ACS712

## Descripción del proyecto

Este proyecto implementa la modernización de un sistema de arranque estrella-delta para un motor trifásico mediante una ESP32 DevKit V1, un módulo de relés de estado sólido de cuatro canales y una interfaz móvil basada en MQTT.

La lógica convencional de temporización y enclavamiento se reemplazó por una máquina de estados finitos no bloqueante. El sistema controla tres contactores:

- K1: contactor de línea.
- K2: contactor de delta.
- K3: contactor de estrella.

La ESP32 ejecuta la transición automática desde estrella hacia delta, incorpora un tiempo muerto de seguridad y permite supervisar y controlar el sistema desde un teléfono móvil.

También se integró un sensor ACS712-30A para medir la corriente de una fase del motor y publicar el valor RMS mediante MQTT. El firmware incluye funciones para calibración del sensor, telemetría, alarmas y protección temporizada por sobrecorriente.

## Integrantes

- Andrés Esteban Pérez Barbosa
  - Código: 2023119062
  - Correo: aeperezb@unimagdalena.edu.co

- Jair David Rangel Martínez
  - Código: 2023119055
  - Correo: jdrangelm@unimagdalena.edu.co

### Información académica

- Asignatura: Electrónica de Potencia
- Grupo: 4
- Docente: Jordan Dallan Guillot Fula
- Universidad del Magdalena
- Fecha de realización: 17 de septiembre de 2026

## Componentes utilizados

- ESP32 DevKit V1.
- Módulo de relés de estado sólido de cuatro canales.
- Tres contactores electromecánicos.
- K1 como contactor de línea.
- K2 como contactor de delta.
- K3 como contactor de estrella.
- Motor trifásico Siemens de 1 HP y dos polos.
- Sensor de corriente ACS712-30A.
- Resistencias de 10 kΩ y 20 kΩ para acondicionamiento de la salida del ACS712.
- Teléfono móvil con IoT MQTT Panel.
- Broker MQTT HiveMQ Cloud.
- Fuente de 5 V para el circuito de control.
- Conductores y borneras del banco de entrenamiento.
- Pinza amperimétrica para la validación de corriente.
- Computador con Arduino IDE.

## Secuencia estrella-delta

La máquina de estados finitos utiliza cinco estados:

### 1. DETENIDO

Todos los contactores permanecen apagados.

```text
K1 = OFF
K2 = OFF
K3 = OFF

### 2. ESTRELLA

Al recibir el comando `START`, se activan el contactor de línea K1 y el contactor de estrella K3.

```text
K1 = ON
K2 = OFF
K3 = ON

3. TIEMPO MUERTO

Cuando finaliza el tiempo de funcionamiento en estrella, el contactor K3 se desactiva y K1 permanece activado.

Plain Text
K1 = ON
K2 = OFF
K3 = OFF

El tiempo muerto programado es de 300 ms. Durante este intervalo, K2 y K3 permanecen desactivados para evitar una conexión simultánea entre estrella y delta.

4. DELTA

Después de finalizar el tiempo muerto, se activa el contactor de delta K2.


K1 = ON
K2 = ON
K3 = OFF
``

El sistema permanece en delta hasta recibir un comando STOP, EMERGENCY o hasta detectar una condición de fallo.

5. FALLO

Ante una parada de emergencia, pérdida prolongada de comunicación, activación simultánea no permitida o sobrecorriente, se desactivan todas las salidas.


K1 = OFF
K2 = OFF
K3 = OFF

El sistema permanece bloqueado hasta recibir un comando RESET válido.

Asignación de GPIO

La asignación de pines utilizada en la ESP32 es la siguiente:

GPIO 25 → CH1 del SSR → K1, contactor de línea
GPIO 26 → CH2 del SSR → K2, contactor de delta
GPIO 27 → CH3 del SSR → K3, contactor de estrella
GPIO 34 ← Salida acondicionada del ACS712-30A
GPIO 2 → LED interno de diagnóstico

El módulo SSR funciona con lógica activa en nivel bajo:

LOW → Canal SSR activado
HIGH → Canal SSR desactivado

La alimentación del lado lógico se distribuye de la siguiente forma:

ESP32 5V/VIN → DC+ del módulo SSR
ESP32 GND → DC- del módulo SSR
ESP32 5V/VIN → VCC del ACS712
ESP32 GND → GND del ACS712

La salida del ACS712 se acondiciona antes de conectarse al GPIO 34:

ACS712 OUT → resistencia de 10 kΩ → GPIO 34
GPIO 34 → resistencia de 20 kΩ → GND
Tópicos MQTT
Comandos del motor
jair/motor/cmd
 

Mensajes utilizados:


START → Inicia la secuencia estrella-delta
STOP → Ejecuta una parada normal
EMERGENCY → Activa la parada de emergencia
RESET → Rearma el sistema después de un fallo
CALIBRATE → Calibra el cero del ACS712 con el motor detenido
``
Ajuste del tiempo de estrella
jair/motor/setpoint

Este tópico permite configurar el tiempo de funcionamiento en estrella entre 2 y 10 segundos.

Estado de la máquina
jair/motor/estado

Estados publicados:

DETENIDO
ESTRELLA
TIEMPO_MUERTO
DELTA
FALLO
Estado de conexión
jair/motor/online

Mensajes publicados:

ONLINE
OFFLINE

El mensaje OFFLINE se configura como última voluntad MQTT para indicar una desconexión inesperada de la ESP32.

Alarmas
jair/motor/alarma

Mensajes posibles:

SIN_ALARMA
PARADA_REMOTA
PERDIDA_COMUNICACION
SOBRECORRIENTE_TEMPORIZANDO
FALLO_SOBRECORRIENTE
INTERLOCK_K2_K3
START_RECHAZADO_FALLO
START_RECHAZADO_SIN_MQTT
RESET_RECHAZADO_CORRIENTE
SETPOINT_INVALIDO
ACS712_CALIBRADO
 
Corriente RMS
jair/motor/corriente

Este tópico publica la corriente RMS medida mediante el ACS712-30A, expresada en amperios.

Telemetría
jair/motor/telemetria

La telemetría se publica en formato JSON e incluye:

Estado actual de la FSM.
Tiempo configurado en estrella.
Tiempo muerto.
Corriente RMS.
Corriente nominal configurada.
Lectura del convertidor ADC.
Voltaje de cero del ACS712.
Estado de K1, K2 y K3.
Estado de emergencia.
Estado de la temporización por sobrecorriente.

Ejemplo:

JSON
{
"estado": "DELTA",
"rampa": 5,
"tiempo_muerto_ms": 300,
"corriente": 1.82,
"corriente_nominal": 0.0,
"sensor": "ACS712_30A",
"adc": 2048,
"voltaje_cero": 2.5,
"k1": true,
"k2": true,
"k3": false,
"emergencia": false,
"sobrecorriente": false
}
Dashboard móvil

El dashboard fue desarrollado en IoT MQTT Panel y contiene:

Indicador del estado de la FSM.
Indicador de conexión de la ESP32.
Slider para ajustar el tiempo de estrella.
Botón ARRANCAR.
Botón DETENER.
Botón EMERGENCIA.
Botón REARMAR.
Panel de alarmas.
Medidor de corriente de fase.
Panel de telemetría.

El dashboard permite controlar y supervisar el sistema desde uno o varios teléfonos conectados al mismo broker MQTT. Cada dispositivo debe utilizar un Client ID diferente.

Medición de corriente

El sensor ACS712-30A se utiliza para medir la corriente de una fase del motor.

El firmware realiza las siguientes operaciones:

Calibra automáticamente el voltaje de cero con el motor detenido.
Muestrea la salida analógica del sensor.
Elimina el componente de continua de la señal.
Calcula el valor cuadrático medio.
Convierte el voltaje RMS en corriente RMS.
Publica el resultado mediante MQTT.
Compara la corriente con los umbrales de advertencia y disparo.

La sensibilidad nominal configurada es:

66 mV/A

La señal del ACS712 se conecta al GPIO 34 mediante un divisor resistivo de 10 kΩ y 20 kΩ para limitar el voltaje aplicado a la entrada analógica de la ESP32.

Seguridad implementada

El firmware incorpora las siguientes medidas de seguridad:

Estado inicial con todos los contactores desactivados.
Enclavamiento lógico entre K2 y K3.
Tiempo muerto de 300 ms.
Parada normal mediante STOP.
Parada de emergencia mediante EMERGENCY.
Bloqueo del arranque mientras exista un fallo.
Rearme controlado mediante RESET.
Detección de pérdida de comunicación.
Publicación de estados ONLINE y OFFLINE.
Advertencia por sobrecorriente.
Disparo temporizado por sobrecorriente.
Rechazo del rearme mientras exista una corriente elevada.

Las conexiones de potencia y corriente alterna deben realizarse con el sistema desenergizado y bajo supervisión del laboratorio.
```text Completa README con FSM, MQTT y estado del proyecto
