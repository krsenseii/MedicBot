#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "servo_control.h"

static const char *TAG = "medicbot_estanteria";

/*
 * 1: realiza una prueba al arrancar: abre y cierra cada servo una vez.
 * 0: deja todos los servos en posicion cerrada y espera a que otras tareas
 *    (por ejemplo, comunicacion ESP-NOW) los controlen.
 */
#define EJECUTAR_DEMO_AL_ARRANQUE 1
#define TIEMPO_MOVIMIENTO_MS       800
#define TIEMPO_ENTRE_SERVOS_MS     300

void app_main(void)
{
    ESP_LOGI(TAG, "Arrancando controlador de estanteria MedicBot");

    ESP_ERROR_CHECK(inicializar_servos());

    /* Asegura que todos comienzan en la posicion de cierre configurada. */
    for (uint8_t servo = 1; servo <= NUM_SERVOS; servo++) {
        ESP_ERROR_CHECK(cerrar_puerta(servo));
    }

    vTaskDelay(pdMS_TO_TICKS(500));

#if EJECUTAR_DEMO_AL_ARRANQUE
    ESP_LOGI(TAG, "Iniciando demostracion de los tres servos");

    for (uint8_t servo = 1; servo <= NUM_SERVOS; servo++) {
        ESP_LOGI(TAG, "Probando servo %u", (unsigned)servo);

        ESP_ERROR_CHECK(abrir_puerta(servo));
        vTaskDelay(pdMS_TO_TICKS(TIEMPO_MOVIMIENTO_MS));

        ESP_ERROR_CHECK(cerrar_puerta(servo));
        vTaskDelay(pdMS_TO_TICKS(TIEMPO_MOVIMIENTO_MS + TIEMPO_ENTRE_SERVOS_MS));
    }

    ESP_LOGI(TAG, "Demostracion terminada. Servos en posicion cerrada.");
#else
    ESP_LOGI(TAG, "Demo desactivada; esperando ordenes de control");
#endif

    /*
     * El firmware queda en espera. En una siguiente iteracion del proyecto
     * aqui se podra iniciar una tarea de comunicacion ESP-NOW/Wi-Fi.
     */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
