#include "servo_control.h"

#include <stdbool.h>
#include <inttypes.h>

#include "driver/mcpwm_prelude.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "servo_control";

/*
 * Configuracion PWM para servos RC:
 * - 50 Hz -> periodo de 20 ms.
 * - Resolucion de 1 MHz -> cada tick equivale a 1 us.
 *
 * Se conserva el rango del ejemplo original de Espressif (500-2500 us).
 * Para las primeras pruebas se usan angulos centrales (-30 y +30 grados),
 * evitando llevar el servo directamente a sus topes mecanicos.
 * Ajusta estos valores segun la ficha tecnica y el mecanismo real.
 */
#define SERVO_MIN_PULSEWIDTH_US       500U
#define SERVO_MAX_PULSEWIDTH_US       2500U
#define SERVO_MIN_DEGREE              (-90)
#define SERVO_MAX_DEGREE              (90)

#define SERVO_TIMEBASE_RESOLUTION_HZ  1000000U
#define SERVO_TIMEBASE_PERIOD_TICKS   20000U
#define MCPWM_GROUP_ID                0

#define SERVO_1_GPIO                  18
#define SERVO_2_GPIO                  19
#define SERVO_3_GPIO                  21

#define SERVO_CLOSED_ANGLE            (-30)
#define SERVO_OPEN_ANGLE              (30)

typedef struct {
    int gpio_num;
    int angulo_cerrado;
    int angulo_abierto;
    const char *nombre;

    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t oper;
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t generator;
} servo_channel_t;

/*
 * Un temporizador y un operador MCPWM por servo.
 * En un ESP32 clasico, el grupo MCPWM dispone de los recursos necesarios
 * para estos tres canales independientes.
 */
static servo_channel_t s_servos[NUM_SERVOS] = {
    {
        .gpio_num = SERVO_1_GPIO,
        .angulo_cerrado = SERVO_CLOSED_ANGLE,
        .angulo_abierto = SERVO_OPEN_ANGLE,
        .nombre = "Servo 1",
    },
    {
        .gpio_num = SERVO_2_GPIO,
        .angulo_cerrado = SERVO_CLOSED_ANGLE,
        .angulo_abierto = SERVO_OPEN_ANGLE,
        .nombre = "Servo 2",
    },
    {
        .gpio_num = SERVO_3_GPIO,
        .angulo_cerrado = SERVO_CLOSED_ANGLE,
        .angulo_abierto = SERVO_OPEN_ANGLE,
        .nombre = "Servo 3",
    },
};

static bool s_servos_inicializados = false;

static uint32_t angulo_a_pulso_us(int angulo)
{
    const int32_t rango_angulo = SERVO_MAX_DEGREE - SERVO_MIN_DEGREE;
    const int32_t rango_pulso = (int32_t)SERVO_MAX_PULSEWIDTH_US -
                                (int32_t)SERVO_MIN_PULSEWIDTH_US;

    return (uint32_t)(((int32_t)(angulo - SERVO_MIN_DEGREE) * rango_pulso) /
                      rango_angulo + (int32_t)SERVO_MIN_PULSEWIDTH_US);
}

static esp_err_t inicializar_canal(servo_channel_t *servo)
{
    ESP_RETURN_ON_FALSE(servo != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "Puntero de servo nulo");

    mcpwm_timer_config_t timer_config = {
        .group_id = MCPWM_GROUP_ID,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD_TICKS,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_timer(&timer_config, &servo->timer), TAG,
                        "No se pudo crear el temporizador MCPWM de %s", servo->nombre);

    mcpwm_operator_config_t operator_config = {
        .group_id = MCPWM_GROUP_ID,
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_operator(&operator_config, &servo->oper), TAG,
                        "No se pudo crear el operador MCPWM de %s", servo->nombre);

    ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(servo->oper, servo->timer), TAG,
                        "No se pudo conectar el timer de %s", servo->nombre);

    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_comparator(servo->oper, &comparator_config,
                                              &servo->comparator), TAG,
                        "No se pudo crear el comparador de %s", servo->nombre);

    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = servo->gpio_num,
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_generator(servo->oper, &generator_config,
                                             &servo->generator), TAG,
                        "No se pudo crear el generador de %s en GPIO %d",
                        servo->nombre, servo->gpio_num);

    /* Arranca en la posicion configurada como cerrada. */
    uint32_t pulso_inicial = angulo_a_pulso_us(servo->angulo_cerrado);
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(servo->comparator,
                                                            pulso_inicial), TAG,
                        "No se pudo fijar la posicion inicial de %s", servo->nombre);

    /* Inicio del pulso al comienzo del periodo y fin al alcanzar el comparador. */
    ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_timer_event(
                            servo->generator,
                            MCPWM_GEN_TIMER_EVENT_ACTION(
                                MCPWM_TIMER_DIRECTION_UP,
                                MCPWM_TIMER_EVENT_EMPTY,
                                MCPWM_GEN_ACTION_HIGH)), TAG,
                        "No se pudo configurar el inicio PWM de %s", servo->nombre);

    ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_compare_event(
                            servo->generator,
                            MCPWM_GEN_COMPARE_EVENT_ACTION(
                                MCPWM_TIMER_DIRECTION_UP,
                                servo->comparator,
                                MCPWM_GEN_ACTION_LOW)), TAG,
                        "No se pudo configurar el fin PWM de %s", servo->nombre);

    ESP_RETURN_ON_ERROR(mcpwm_timer_enable(servo->timer), TAG,
                        "No se pudo habilitar el timer de %s", servo->nombre);
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(servo->timer,
                                                MCPWM_TIMER_START_NO_STOP), TAG,
                        "No se pudo arrancar el timer de %s", servo->nombre);

    ESP_LOGI(TAG, "%s inicializado en GPIO %d; posicion cerrada: %d grados (%" PRIu32 " us)",
             servo->nombre, servo->gpio_num, servo->angulo_cerrado, pulso_inicial);

    return ESP_OK;
}

esp_err_t inicializar_servos(void)
{
    if (s_servos_inicializados) {
        ESP_LOGW(TAG, "Los servos ya estaban inicializados");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Inicializando %d servos con MCPWM", NUM_SERVOS);

    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        esp_err_t err = inicializar_canal(&s_servos[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Fallo al inicializar el servo %u", (unsigned)(i + 1));
            return err;
        }
    }

    s_servos_inicializados = true;
    ESP_LOGI(TAG, "Todos los servos estan listos");
    return ESP_OK;
}

esp_err_t mover_servo(uint8_t numero_servo, int angulo)
{
    ESP_RETURN_ON_FALSE(s_servos_inicializados, ESP_ERR_INVALID_STATE, TAG,
                        "Primero debes llamar a inicializar_servos()");
    ESP_RETURN_ON_FALSE(numero_servo >= 1 && numero_servo <= NUM_SERVOS,
                        ESP_ERR_INVALID_ARG, TAG,
                        "Numero de servo invalido: %u (usa 1..%d)",
                        (unsigned)numero_servo, NUM_SERVOS);
    ESP_RETURN_ON_FALSE(angulo >= SERVO_MIN_DEGREE && angulo <= SERVO_MAX_DEGREE,
                        ESP_ERR_INVALID_ARG, TAG,
                        "Angulo fuera de rango: %d (usa %d..%d)",
                        angulo, SERVO_MIN_DEGREE, SERVO_MAX_DEGREE);

    servo_channel_t *servo = &s_servos[numero_servo - 1];
    uint32_t pulso_us = angulo_a_pulso_us(angulo);

    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(servo->comparator,
                                                            pulso_us), TAG,
                        "No se pudo actualizar el PWM de %s", servo->nombre);

    ESP_LOGI(TAG, "%s (GPIO %d) -> %d grados (%" PRIu32 " us)",
             servo->nombre, servo->gpio_num, angulo, pulso_us);
    return ESP_OK;
}

esp_err_t abrir_puerta(uint8_t numero_servo)
{
    ESP_RETURN_ON_FALSE(numero_servo >= 1 && numero_servo <= NUM_SERVOS,
                        ESP_ERR_INVALID_ARG, TAG,
                        "Numero de servo/puerta invalido: %u",
                        (unsigned)numero_servo);

    return mover_servo(numero_servo,
                       s_servos[numero_servo - 1].angulo_abierto);
}

esp_err_t cerrar_puerta(uint8_t numero_servo)
{
    ESP_RETURN_ON_FALSE(numero_servo >= 1 && numero_servo <= NUM_SERVOS,
                        ESP_ERR_INVALID_ARG, TAG,
                        "Numero de servo/puerta invalido: %u",
                        (unsigned)numero_servo);

    return mover_servo(numero_servo,
                       s_servos[numero_servo - 1].angulo_cerrado);
}
