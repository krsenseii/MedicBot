# MedicBot - Controlador de estanteria (ESP-IDF 5.5)

Proyecto inicial para controlar **tres servomotores independientes** desde un ESP32 mediante el periférico MCPWM de ESP-IDF 5.5. No utiliza multiplexor.

## Estructura

```text
MedicBot_Estanteria_ESP_IDF55/
├── CMakeLists.txt
├── README.md
└── main/
    ├── CMakeLists.txt
    ├── main.c
    ├── servo_control.c
    └── servo_control.h
```

## Configuración inicial

En `main/servo_control.c` se pueden cambiar los GPIO y los ángulos:

```c
#define SERVO_1_GPIO 18
#define SERVO_2_GPIO 19
#define SERVO_3_GPIO 21

#define SERVO_CLOSED_ANGLE (-30)
#define SERVO_OPEN_ANGLE   (30)
```

Los tres canales utilizan inicialmente:

- GPIO 18: servo 1.
- GPIO 19: servo 2.
- GPIO 21: servo 3.
- Frecuencia PWM: 50 Hz.
- Resolución del temporizador: 1 MHz (1 tick = 1 microsegundo).
- Rango de pulso de partida: 500-2500 us, igual que el ejemplo MCPWM original de Espressif.
- Posiciones iniciales: -30 grados (cerrado) y +30 grados (abierto).

Los ángulos son valores iniciales de prueba, no posiciones mecánicas universales. Ajusta el sentido y los límites de cada servo según el montaje. Si un servo gira al revés, intercambia sus ángulos de abierto y cerrado en la configuración.

## Funciones disponibles

La API está en `main/servo_control.h`:

```c
ESP_ERROR_CHECK(inicializar_servos());

ESP_ERROR_CHECK(abrir_puerta(1));
ESP_ERROR_CHECK(cerrar_puerta(1));

ESP_ERROR_CHECK(mover_servo(2, 15));  // Servo 2 a 15 grados
```

La numeración empieza en 1. `abrir_puerta(n)` y `cerrar_puerta(n)` actúan sobre el canal de servo n. En el diseño mecánico de doble compuerta, un compartimento puede necesitar dos servos (retenedor y compuerta inferior); en ese caso, la secuencia del compartimento deberá coordinar dos canales.

## Demostración al arrancar

Por defecto, `main/main.c` ejecuta una demostración: cierra los tres servos y después abre/cierra cada uno una vez.

Para desactivar el movimiento automático al arrancar, cambia:

```c
#define EJECUTAR_DEMO_AL_ARRANQUE 1
```

a:

```c
#define EJECUTAR_DEMO_AL_ARRANQUE 0
```

Así se podrán accionar más adelante desde una tarea de comunicación ESP-NOW/Wi-Fi.

## Conexión eléctrica

Cada servo tiene tres cables:

| Cable del servo | Conexión |
|---|---|
| Señal | GPIO asignado |
| VCC | Fuente externa regulada de 5 V (o la tensión indicada por el fabricante) |
| GND | GND de la fuente externa |

**Obligatorio:** conectar el GND de la fuente externa con el GND del ESP32. No alimentar los servos desde un GPIO ni desde el pin de 3,3 V de la placa. Dimensionar la fuente para la corriente de arranque/bloqueo de los tres servos. Es recomendable probar primero un servo cada vez, sin carga mecánica, y comprobar que la fuente no cae de tensión.

Comprueba que los GPIO seleccionados están disponibles en tu modelo concreto de placa. En un ESP32 clásico, GPIO 18, 19 y 21 suelen estar disponibles, pero otras placas o periféricos conectados pueden ocuparlos.

## Compilar y cargar con ESP-IDF 5.5

Abre una terminal ESP-IDF 5.5 y entra en esta carpeta:

```bash
cd MedicBot_Estanteria_ESP_IDF55
idf.py set-target esp32
idf.py build
```

Para cargar el firmware y abrir el monitor:

```bash
idf.py -p COMx flash monitor
```

Sustituye `COMx` por el puerto de tu placa (por ejemplo `COM5` en Windows o `/dev/ttyUSB0` en Linux).

Para limpiar y recompilar desde cero:

```bash
idf.py fullclean
idf.py build
```

## Nota sobre la API MCPWM

Se crea un temporizador y un operador MCPWM por servo, con comparador y generador independientes. Cada canal genera una señal de 50 Hz y puede cambiar de posición sin alterar los otros dos.
