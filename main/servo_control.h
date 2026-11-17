#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_SERVOS 3

/**
 * Inicializa los tres canales MCPWM y coloca cada servo en su posicion cerrada.
 * Debe llamarse una sola vez desde app_main().
 */
esp_err_t inicializar_servos(void);

/**
 * Mueve un servo a un angulo entre -90 y +90 grados.
 * numero_servo usa numeracion humana: 1, 2 o 3.
 */
esp_err_t mover_servo(uint8_t numero_servo, int angulo);

/**
 * Lleva el servo indicado a su angulo configurado como abierto.
 * En esta base, cada numero identifica un canal/actuador, no un compartimento
 * completo con dos servos.
 */
esp_err_t abrir_puerta(uint8_t numero_servo);

/** Lleva el servo indicado a su angulo configurado como cerrado. */
esp_err_t cerrar_puerta(uint8_t numero_servo);

#ifdef __cplusplus
}
#endif
