/*
 * Base de control de compuertas con servos - ESP32 clásico / ESP-IDF 6.1
 * Basado en el ejemplo MCPWM de Espressif aportado por el usuario.
 *
 * Esta versión usa un operador MCPWM por servo y comparte un temporizador
 * de 50 Hz por cada grupo MCPWM. En el ESP32 clásico hay dos grupos y tres
 * operadores por grupo: esta base contempla hasta seis servos.
 */

#include <stdint.h>
#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"

static const char *TAG = "estanteria_servos";

// -----------------------------------------------------------------------------
// Ajustes generales de los servos
// Consulta la ficha técnica de tus servos y calibra estos valores.
// -----------------------------------------------------------------------------
#define SERVO_MIN_PULSEWIDTH_US       500
#define SERVO_MAX_PULSEWIDTH_US       2500
#define SERVO_MIN_DEGREE              (-90)
#define SERVO_MAX_DEGREE              90

#define SERVO_TIMEBASE_RESOLUTION_HZ  1000000  // 1 tick = 1 us
#define SERVO_TIMEBASE_PERIOD         20000    // 20 ms = 50 Hz

// ESP32 clásico: 2 grupos MCPWM, con 3 operadores por grupo.
#define SERVO_GROUP_COUNT             2
#define SERVO_OPERATORS_PER_GROUP     3
#define NUM_PUERTAS                   6

#if NUM_PUERTAS > (SERVO_GROUP_COUNT * SERVO_OPERATORS_PER_GROUP)
#error "Esta base admite hasta 6 servos MCPWM en el ESP32 clasico"
#endif

// -----------------------------------------------------------------------------
// Configuración de cada puerta
//
// Los ángulos están expresados en el rango -90..90 del ejemplo de Espressif.
// Ajusta los ángulos de apertura/cierre para que la compuerta no fuerce
// mecánicamente el servo ni la estructura.
// -----------------------------------------------------------------------------
typedef struct {
    gpio_num_t gpio;
    int angulo_cerrado;
    int angulo_abierto;

    // Recursos MCPWM asociados a este servo.
    mcpwm_oper_handle_t operador;
    mcpwm_cmpr_handle_t comparador;
    mcpwm_gen_handle_t generador;
} servo_puerta_t;

// Un temporizador independiente para cada grupo MCPWM.
static mcpwm_timer_handle_t timers[SERVO_GROUP_COUNT] = { NULL };

// El índice 0 del array corresponde a la puerta número 1.
// GPIO de ejemplo para una ESP32 DevKit común: cámbialos según tu placa.
static servo_puerta_t servos[NUM_PUERTAS] = {
    { .gpio = GPIO_NUM_18, .angulo_cerrado = -45, .angulo_abierto =  45 },
    { .gpio = GPIO_NUM_19, .angulo_cerrado = -45, .angulo_abierto =  45 },
    { .gpio = GPIO_NUM_21, .angulo_cerrado = -45, .angulo_abierto =  45 },
    { .gpio = GPIO_NUM_22, .angulo_cerrado = -45, .angulo_abierto =  45 },
    { .gpio = GPIO_NUM_23, .angulo_cerrado = -45, .angulo_abierto =  45 },
    { .gpio = GPIO_NUM_25, .angulo_cerrado = -45, .angulo_abierto =  45 },
};

// Convierte un ángulo a ticks MCPWM. Con resolución de 1 MHz, un tick equivale
// a un microsegundo, igual que en el ejemplo original.
static uint32_t angulo_a_ticks(int angulo)
{
    return (uint32_t)(
        (angulo - SERVO_MIN_DEGREE) *
        (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) /
        (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) +
        SERVO_MIN_PULSEWIDTH_US
    );
}

// Crea un temporizador MCPWM a 50 Hz para el grupo indicado.
static esp_err_t crear_timer_grupo(int grupo)
{
    mcpwm_timer_config_t timer_config = {
        .group_id = grupo,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };

    return mcpwm_new_timer(&timer_config, &timers[grupo]);
}

// Crea el canal MCPWM que controla un servo concreto.
static esp_err_t configurar_servo(int indice)
{
    servo_puerta_t *servo = &servos[indice];

    // Cada grupo dispone de tres operadores. Los índices 0..2 usan el grupo 0
    // y los índices 3..5 usan el grupo 1.
    const int grupo = indice / SERVO_OPERATORS_PER_GROUP;

    mcpwm_operator_config_t operator_config = {
        .group_id = grupo,
    };
    ESP_RETURN_ON_ERROR(
        mcpwm_new_operator(&operator_config, &servo->operador),
        TAG, "No se pudo crear el operador MCPWM"
    );

    ESP_RETURN_ON_ERROR(
        mcpwm_operator_connect_timer(servo->operador, timers[grupo]),
        TAG, "No se pudo conectar el operador al timer"
    );

    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_RETURN_ON_ERROR(
        mcpwm_new_comparator(servo->operador, &comparator_config, &servo->comparador),
        TAG, "No se pudo crear el comparador"
    );

    // Al arrancar, cada compuerta se coloca en su posición de cierre.
    ESP_RETURN_ON_ERROR(
        mcpwm_comparator_set_compare_value(
            servo->comparador,
            angulo_a_ticks(servo->angulo_cerrado)
        ),
        TAG, "No se pudo fijar la posición inicial del servo"
    );

    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = servo->gpio,
    };
    ESP_RETURN_ON_ERROR(
        mcpwm_new_generator(servo->operador, &generator_config, &servo->generador),
        TAG, "No se pudo crear el generador PWM"
    );

    // Inicio del pulso al comienzo del periodo.
    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_timer_event(
            servo->generador,
            MCPWM_GEN_TIMER_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                MCPWM_TIMER_EVENT_EMPTY,
                MCPWM_GEN_ACTION_HIGH
            )
        ),
        TAG, "No se pudo configurar el flanco inicial"
    );

    // Fin del pulso cuando el contador alcanza el valor del comparador.
    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_compare_event(
            servo->generador,
            MCPWM_GEN_COMPARE_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                servo->comparador,
                MCPWM_GEN_ACTION_LOW
            )
        ),
        TAG, "No se pudo configurar el flanco final"
    );

    ESP_LOGI(TAG, "Puerta %d configurada en GPIO %d (grupo MCPWM %d)",
             indice + 1, servo->gpio, grupo);

    return ESP_OK;
}

// Inicializa todos los recursos MCPWM y arranca los temporizadores.
static esp_err_t inicializar_servos(void)
{
    esp_err_t err;

    // 1. Crear un timer por grupo.
    for (int grupo = 0; grupo < SERVO_GROUP_COUNT; grupo++) {
        err = crear_timer_grupo(grupo);
        if (err != ESP_OK) {
            return err;
        }
    }

    // 2. Crear un operador/canal independiente por puerta.
    for (int i = 0; i < NUM_PUERTAS; i++) {
        err = configurar_servo(i);
        if (err != ESP_OK) {
            return err;
        }
    }

    // 3. Habilitar y arrancar los timers cuando todos los canales estén listos.
    for (int grupo = 0; grupo < SERVO_GROUP_COUNT; grupo++) {
        err = mcpwm_timer_enable(timers[grupo]);
        if (err != ESP_OK) {
            return err;
        }

        err = mcpwm_timer_start_stop(timers[grupo], MCPWM_TIMER_START_NO_STOP);
        if (err != ESP_OK) {
            return err;
        }
    }

    ESP_LOGI(TAG, "Control de %d puertas inicializado", NUM_PUERTAS);
    return ESP_OK;
}

// Mueve el servo indicado a un ángulo absoluto dentro del rango permitido.
static esp_err_t mover_servo(int indice, int angulo)
{
    if (indice < 0 || indice >= NUM_PUERTAS) {
        return ESP_ERR_INVALID_ARG;
    }

    if (angulo < SERVO_MIN_DEGREE || angulo > SERVO_MAX_DEGREE) {
        ESP_LOGE(TAG, "Ángulo fuera de rango: %d", angulo);
        return ESP_ERR_INVALID_ARG;
    }

    if (servos[indice].comparador == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    return mcpwm_comparator_set_compare_value(
        servos[indice].comparador,
        angulo_a_ticks(angulo)
    );
}

// -----------------------------------------------------------------------------
// API para la lógica de la estantería
// La numeración es desde 1: abrir_puerta(1) controla servos[0].
// -----------------------------------------------------------------------------
esp_err_t abrir_puerta(int numero_puerta)
{
    if (numero_puerta < 1 || numero_puerta > NUM_PUERTAS) {
        ESP_LOGE(TAG, "Número de puerta no válido: %d", numero_puerta);
        return ESP_ERR_INVALID_ARG;
    }

    const int indice = numero_puerta - 1;
    ESP_LOGI(TAG, "Abriendo puerta %d", numero_puerta);
    return mover_servo(indice, servos[indice].angulo_abierto);
}

esp_err_t cerrar_puerta(int numero_puerta)
{
    if (numero_puerta < 1 || numero_puerta > NUM_PUERTAS) {
        ESP_LOGE(TAG, "Número de puerta no válido: %d", numero_puerta);
        return ESP_ERR_INVALID_ARG;
    }

    const int indice = numero_puerta - 1;
    ESP_LOGI(TAG, "Cerrando puerta %d", numero_puerta);
    return mover_servo(indice, servos[indice].angulo_cerrado);
}

// Ejemplo mínimo de uso. Aquí se abre la puerta 3, se espera y se cierra.
// En el proyecto final, estas llamadas se harán desde la lógica de inventario.
void app_main(void)
{
    ESP_ERROR_CHECK(inicializar_servos());

    ESP_ERROR_CHECK(abrir_puerta(3));
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_ERROR_CHECK(cerrar_puerta(3));

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
