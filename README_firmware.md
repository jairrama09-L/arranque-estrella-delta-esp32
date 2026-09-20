# Arranque estrella-delta con accionamiento estático, ESP32 y supervisión MQTT

Práctica de laboratorio N.º 1 de Electrónica de Potencia, Universidad del Magdalena.
Sustituye el mando cableado y el temporizador Autonics AT8N de un arrancador
estrella-delta por una máquina de estados no bloqueante en un ESP32, conservando
los contactores de potencia Siemens K1, K2 y K3.

**Autores:** Andrés Esteban Pérez Barbosa · Jair David Rangel Martínez
**Docente:** Ing. Jordan Dallan Guillot Fula, PhD (c)

---

## Contenido del repositorio

| Archivo | Descripción |
|---|---|
| `arranque_estrella_delta_esp32.ino` | Firmware completo y comentado |
| `credenciales_ejemplo.h` | Plantilla de credenciales (copiar como `credenciales.h`) |
| `.gitignore` | Impide publicar `credenciales.h` |

## Puesta en marcha

1. Copie `credenciales_ejemplo.h` como `credenciales.h` y complete la red Wi-Fi,
   el host del broker, el usuario y la contraseña.
2. Instale la librería **PubSubClient** desde el gestor de librerías de Arduino.
3. Seleccione la tarjeta *ESP32 Dev Module* y cargue el sketch.
4. Con el motor detenido, envíe `C` por el monitor serial (115200 baudios) para
   calibrar el cero del ACS712.
5. Publique la corriente nominal medida del motor en `<prefijo>/nominal`
   (o use `N3.3` por serial). Mientras ese valor sea 0 A, las protecciones de
   sobrecarga quedan inhibidas a propósito.

## Conexiones

| Señal | Pin | Destino |
|---|---|---|
| K1 línea | GPIO 25 | Entrada de control del SSR1 |
| K2 delta | GPIO 26 | Entrada de control del SSR2 |
| K3 estrella | GPIO 27 | Entrada de control del SSR3 |
| Paro de emergencia | GPIO 33 | Contacto auxiliar NC de la seta, a GND |
| Corriente | GPIO 34 | ACS712-30A a través del divisor 10 k / 20 k |
| LED de estado | GPIO 2 | Indicador integrado |

**Obligatorio en el montaje:** tres resistencias de 10 kΩ de pull-up desde
GPIO 25, 26 y 27 a 3,3 V. Durante el reset del ESP32 los GPIO quedan en alta
impedancia y el módulo SSR es activo en nivel bajo; sin esos pull-up un contactor
puede cerrar solo al energizar o reiniciar la tarjeta.

**Recomendado:** enclavamiento mecánico además del de software, cableando el
contacto auxiliar NC de K2 en serie con la bobina de K3 y viceversa, y un
varistor o red snubber RC en paralelo con cada bobina A1–A2.

## Tópicos MQTT

Todos cuelgan del prefijo definido en `credenciales.h` (por defecto `jair/motor`).

| Tópico | Sentido | Contenido |
|---|---|---|
| `.../cmd` | HMI → ESP32 | `START`, `STOP`, `EMERGENCY`, `RESET`, `CALIBRATE` |
| `.../setpoint` | HMI → ESP32 | Tiempo de estrella en segundos (2 a 10) |
| `.../nominal` | HMI → ESP32 | Corriente nominal del motor en amperios |
| `.../estado` | ESP32 → HMI | Estado de la FSM (retenido) |
| `.../online` | ESP32 → HMI | `ONLINE` / `OFFLINE` por testamento (retenido) |
| `.../alarma` | ESP32 → HMI | Última alarma (retenido) |
| `.../corriente` | ESP32 → HMI | Corriente eficaz en amperios |
| `.../telemetria` | ESP32 → HMI | JSON con todas las variables |

> Los comandos deben publicarse **sin** la bandera *retained*. Un `START`
> retenido se reentregaría en cada reconexión. El firmware se protege
> descartando los comandos que llegan en los primeros 3 s tras suscribirse,
> pero la HMI debe configurarse correctamente igual.

## Criterios de seguridad implementados

- Estado seguro en el arranque y en cualquier fallo: las tres salidas se apagan.
- Enclavamiento por software: K2 y K3 nunca se activan a la vez.
- Tiempo muerto determinístico de 300 ms (peor caso calculado: 34,3 ms).
- Paro de emergencia físico con contacto NC realimentado a GPIO 33; falla segura
  ante rotura de cable.
- Failsafe: pérdida de Wi-Fi o MQTT durante más de 10 s en marcha → FALLO.
- Bloqueo de rearranque de 5 s tras una parada.
- Imagen térmica I²t inhibida durante la irrupción de arranque.
- Credenciales fuera del código fuente y validación opcional del certificado TLS.

## Aviso

El paro de emergencia remoto por Wi-Fi **no** es una parada de seguridad y no
sustituye a la seta cableada en serie con las bobinas.
