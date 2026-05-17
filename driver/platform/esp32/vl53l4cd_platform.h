/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2026 Reptaide.
 *
 * Redistributed under the terms of the BSD 3-Clause License.
 * See LICENSE in the root of this repository for the full license text.
 */

#ifndef VL53L4CD_PLATFORM_H
#define VL53L4CD_PLATFORM_H

#include "driver/i2c_types.h"
#include "vl53l4cd_core.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Inizializza l'interfaccia hardware del dispositivo.
     *
     * @param[in] device                Dispositivo VL53L4CD.
     * @param[in] bus_handle            Handle del bus I2C.
     * @param[in] i2c_address           Indirizzo I2C del dispositivo.
     * @param[in] i2c_scl_speed         Velocità di clock del dispositivo sul bus I2C.
     * @param[in] i2c_timeout           Valore del timeout I2C del dispositivo
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     * @retval VL53L4CD_ERR_FAIL        Errore nella configurazione dell'ISR o del pin.
     */
    vl53l4cd_err_t vl53l4cd_init_hal(vl53l4cd_t *device,
        i2c_master_bus_handle_t bus_handle,
        const uint8_t i2c_address,
        const uint32_t i2c_scl_speed,
        const int32_t i2c_timeout);

    /**
     * @brief Configura il pin di alimentazione del dispositivo.
     *
     * @param[in] device                Dispositivo VL53L4CD.
     * @param[in] xshut_pin             Valore del pin di alimentazione.
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     * @retval VL53L4CD_ERR_FAIL        Errore nella configurazione dell'ISR o del pin.
     */
    vl53l4cd_err_t vl53l4cd_hal_setup_xshut(vl53l4cd_t *device, const uint8_t xshut_pin);

    /**
     * @brief Configura il pin di interrupt e registra la ISR.
     *
     * @param[in] device                Dispositivo VL53L4CD.
     * @param[in] int_pin               Valore del pin di interrupt.
     * @retval MCP23017_ERR_OK          Successo.
     * @retval MCP23017_ERR_INVALID_ARG Parametri non validi.
     * @retval MCP23017_ERR_FAIL        Errore nella configurazione dell'ISR o del pin.
     */
    vl53l4cd_err_t vl53l4cd_hal_setup_int(vl53l4cd_t *device, const uint8_t int_pin);

#ifdef __cplusplus
}
#endif

#endif // VL53L4CD_PLATFORM_H