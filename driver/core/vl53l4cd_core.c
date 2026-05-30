/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2021 STMicroelectronics.
 * Portions Copyright (c) 2026 Reptaide.
 *
 * This file is derived from the VL53L4CD ULD driver published by
 * STMicroelectronics (STSW-IMG026, https://github.com/stm32duino/VL53L4CD)
 * and is redistributed under the terms of the BSD 3-Clause License.
 * See LICENSE in the root of this repository for the full license text.
 */

#include "vl53l4cd_core.h"

/**
 * @brief Macro per interrompere l'esecuzione in caso una funzione restituisca un errore.
 */
#define VL53L4CD_ERROR_CHECK(func)                                                                 \
    do                                                                                             \
    {                                                                                              \
        vl53l4cd_err_t __err = (func);                                                             \
        if (__err != VL53L4CD_ERR_OK)                                                              \
            return __err;                                                                          \
    } while (0)

/**
 * @brief Definizione dei registri.
 */
static const uint16_t REG_SOFT_RESET = 0x0000;
static const uint16_t REG_I2C_SLAVE_DEVICE_ADDRESS = 0x0001;
static const uint16_t REG_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND = 0x0008;
static const uint16_t REG_XTALK_PLANE_OFFSET_KCPS = 0x0016;
static const uint16_t REG_XTALK_X_PLANE_GRADIENT_KCPS = 0x0018;
static const uint16_t REG_XTALK_Y_PLANE_GRADIENT_KCPS = 0x001A;
static const uint16_t REG_RANGE_OFFSET_MM = 0x001E;
static const uint16_t REG_INNER_OFFSET_MM = 0x0020;
static const uint16_t REG_OUTER_OFFSET_MM = 0x0022;
static const uint16_t REG_I2C_FAST_MODE_PLUS = 0x002D;
static const uint16_t REG_GPIO_HV_MUX_CTRL = 0x0030;
static const uint16_t REG_GPIO_TIO_HV_STATUS = 0x0031;
static const uint16_t REG_SYSTEM_INTERRUPT = 0x0046;
static const uint16_t REG_RANGE_CONFIG_A = 0x005E;
static const uint16_t REG_RANGE_CONFIG_B = 0x0061;
static const uint16_t REG_RANGE_CONFIG_SIGMA_THRESH = 0x0064;
static const uint16_t REG_MIN_COUNT_RATE_RTN_LIMIT_MCPS = 0x0066;
static const uint16_t REG_INTERMEASUREMENT_MS = 0x006C;
static const uint16_t REG_THRESH_HIGH = 0x0072;
static const uint16_t REG_THRESH_LOW = 0x0074;
static const uint16_t REG_SYSTEM_INTERRUPT_CLEAR = 0x0086;
static const uint16_t REG_SYSTEM_START = 0x0087;
static const uint16_t REG_RESULT_RANGE_STATUS = 0x0089;
static const uint16_t REG_RESULT_SPAD_NB = 0x008C;
static const uint16_t REG_RESULT_SIGNAL_RATE = 0x008E;
static const uint16_t REG_RESULT_AMBIENT_RATE = 0x0090;
static const uint16_t REG_RESULT_SIGMA = 0x0092;
static const uint16_t REG_RESULT_DISTANCE = 0x0096;
static const uint16_t REG_RESULT_OSC_CALIBRATE_VAL = 0x00DE;
static const uint16_t REG_FIRMWARE_SYSTEM_STATUS = 0x00E5;
static const uint16_t REG_IDENTIFICATION_MODEL_ID = 0x010F;

// Questo è l'indirizzo univoco per il sensore VL53L4CD
static const uint16_t VL53L4CD_SENSOR_ID = 0xEBAA;

/**
 * @brief Valori di default dei registri durante lo stato power of reset.
 */
static const uint8_t REG_POR_CONFIG[] = {
    0x12, // 0x2D : Imposta i bit 2 e 5 a uno per la modalità fast plus (1MHz I2C), altrimenti non
          // toccare
    0x00, // 0x2E : Bit 0 = Impostalo a zero se I2C è pulled-up a 1.8V, oppure impostalo a uno
          // (pulled-up a AVDD)
    0x00, // 0x2F : Bit 0 = Impostalo a zero se è pulled-up a 1.8V, oppure impostalo a uno
          // (pulled-up a AVDD)
    0x11, // 0x30 : Imposta il bit 4 a zero per un interrupt active-high e uno per active-low (i bit
          // 3:0 devono essere 0x01)
    0x02, // 0x31 : Bit 1 = Interrupt a seconda della polarità
    0x00, // 0x32 : Non modificabile dall'utente
    0x02, // 0x33 : Non modificabile dall'utente
    0x08, // 0x34 : Non modificabile dall'utente
    0x00, // 0x35 : Non modificabile dall'utente
    0x08, // 0x36 : Non modificabile dall'utente
    0x10, // 0x37 : Non modificabile dall'utente
    0x01, // 0x38 : Non modificabile dall'utente
    0x01, // 0x39 : Non modificabile dall'utente
    0x00, // 0x3A : Non modificabile dall'utente
    0x00, // 0x3B : Non modificabile dall'utente
    0x00, // 0x3C : Non modificabile dall'utente
    0x00, // 0x3D : Non modificabile dall'utente
    0xFF, // 0x3E : Non modificabile dall'utente
    0x00, // 0x3F : Non modificabile dall'utente
    0x0F, // 0x40 : Non modificabile dall'utente
    0x00, // 0x41 : Non modificabile dall'utente
    0x00, // 0x42 : Non modificabile dall'utente
    0x00, // 0x43 : Non modificabile dall'utente
    0x00, // 0x44 : Non modificabile dall'utente
    0x00, // 0x45 : Non modificabile dall'utente
    0x20, // 0x46 : Configurazione interrrupt 0=level low, 1=level high, 2=fuori intervallo,
          // 3->dentro intervallo, 0x20=nuovo campione pronto
    0x0B, // 0x47 : Non modificabile dall'utente
    0x00, // 0x48 : Non modificabile dall'utente
    0x00, // 0x49 : Non modificabile dall'utente
    0x02, // 0x4A : Non modificabile dall'utente
    0x14, // 0x4B : Non modificabile dall'utente
    0x21, // 0x4C : Non modificabile dall'utente
    0x00, // 0x4D : Non modificabile dall'utente
    0x00, // 0x4E : Non modificabile dall'utente
    0x05, // 0x4F : Non modificabile dall'utente
    0x00, // 0x50 : Non modificabile dall'utente
    0x00, // 0x51 : Non modificabile dall'utente
    0x00, // 0x52 : Non modificabile dall'utente
    0x00, // 0x53 : Non modificabile dall'utente
    0xC8, // 0x54 : Non modificabile dall'utente
    0x00, // 0x55 : Non modificabile dall'utente
    0x00, // 0x56 : Non modificabile dall'utente
    0x38, // 0x57 : Non modificabile dall'utente
    0xFF, // 0x58 : Non modificabile dall'utente
    0x01, // 0x59 : Non modificabile dall'utente
    0x00, // 0x5A : Non modificabile dall'utente
    0x08, // 0x5B : Non modificabile dall'utente
    0x00, // 0x5C : Non modificabile dall'utente
    0x00, // 0x5D : Non modificabile dall'utente
    0x01, // 0x5E : Non modificabile dall'utente
    0xCC, // 0x5F : Non modificabile dall'utente
    0x07, // 0x60 : Non modificabile dall'utente
    0x01, // 0x61 : Non modificabile dall'utente
    0xF1, // 0x62 : Non modificabile dall'utente
    0x05, // 0x63 : Non modificabile dall'utente
    0x00, // 0x64 : Soglia sigma MSB
    0xA0, // 0x65 : Soglia sigma LSB
    0x00, // 0x66 : Conteggio minimo MSB
    0x80, // 0x67 : Conteggio minimo LSB
    0x08, // 0x68 : Non modificabile dall'utente
    0x38, // 0x69 : Non modificabile dall'utente
    0x00, // 0x6A : Non modificabile dall'utente
    0x00, // 0x6B : Non modificabile dall'utente
    0x00, // 0x6C : Periodo di intermisurazione MSB
    0x00, // 0x6D : Periodo di intermisurazione
    0x0F, // 0x6E : Periodo di intermisurazione
    0x89, // 0x6F : Periodo di intermisurazione LSB
    0x00, // 0x70 : Non modificabile dall'utente
    0x00, // 0x71 : Non modificabile dall'utente
    0x00, // 0x72 : Soglia alta distanza MSB
    0x00, // 0x73 : Soglia alta distanza LSB
    0x00, // 0x74 : Soglia bassa distanza MSB
    0x00, // 0x75 : Soglia bassa distanza LSB
    0x00, // 0x76 : Non modificabile dall'utente
    0x01, // 0x77 : Non modificabile dall'utente
    0x07, // 0x78 : Non modificabile dall'utente
    0x05, // 0x79 : Non modificabile dall'utente
    0x06, // 0x7A : Non modificabile dall'utente
    0x06, // 0x7B : Non modificabile dall'utente
    0x00, // 0x7C : Non modificabile dall'utente
    0x00, // 0x7D : Non modificabile dall'utente
    0x02, // 0x7E : Non modificabile dall'utente
    0xC7, // 0x7F : Non modificabile dall'utente
    0xFF, // 0x80 : Non modificabile dall'utente
    0x9B, // 0x81 : Non modificabile dall'utente
    0x00, // 0x82 : Non modificabile dall'utente
    0x00, // 0x83 : Non modificabile dall'utente
    0x00, // 0x84 : Non modificabile dall'utente
    0x01, // 0x85 : Non modificabile dall'utente
    0x00, // 0x86 : Elimina l'interrupt
    0x00  // 0x87 : Avvio della misurazione. Se si vuole un avvio automatico, dopo l'init, inserire
          // 0x40 nel registro 0x87
};

/**
 * @brief Legge un byte (8 bit) da un registro del dispositivo, tramite il protocollo I2C.
 *
 * @param[in]  device               Dispositivo VL53L4CD.
 * @param[in]  reg                  Indirizzo del registro.
 * @param[out] data                 Dati da leggere.
 * @retval VL53L4CD_ERR_OK          Successo.
 * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
 */
static vl53l4cd_err_t read_register_8(vl53l4cd_t *device, const uint16_t reg, uint8_t *data)
{
    // Verifica i parametri
    if (!device || !data)
        return VL53L4CD_ERR_INVALID_ARG;

    VL53L4CD_ERROR_CHECK(device->platform->i2c_read(device, reg, 1, data));

    return VL53L4CD_ERR_OK;
}

/**
 * @brief Scrive un byte (8 bit) in un registro del dispositivo, tramite il protocollo I2C.
 *
 * @param[in] device                Dispositivo VL53L4CD.
 * @param[in] reg                   Indirizzo del registro.
 * @param[in] data                  Dati da scrivere.
 * @retval VL53L4CD_ERR_OK          Successo.
 * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
 */
static vl53l4cd_err_t write_register_8(vl53l4cd_t *device, const uint16_t reg, uint8_t data)
{
    // Verifica il parametro
    if (!device)
        return VL53L4CD_ERR_INVALID_ARG;

    VL53L4CD_ERROR_CHECK(device->platform->i2c_write(device, reg, 1, &data));

    return VL53L4CD_ERR_OK;
}

/**
 * @brief Legge 2 byte (16 bit) da un registro del dispositivo, tramite il protocollo I2C.
 *
 * @param[in]  device               Dispositivo VL53L4CD.
 * @param[in]  reg                  Indirizzo del registro.
 * @param[out] data                 Dati da leggere.
 * @retval VL53L4CD_ERR_OK          Successo.
 * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
 */
static vl53l4cd_err_t read_register_16(vl53l4cd_t *device, const uint16_t reg, uint16_t *data)
{
    // Verifica i parametri
    if (!device || !data)
        return VL53L4CD_ERR_INVALID_ARG;

    uint8_t buffer[2];
    VL53L4CD_ERROR_CHECK(device->platform->i2c_read(device, reg, 2, buffer));

    // Combina i byte per ricostruire il dato (big-endian: MSB first)
    *data = ((uint16_t)buffer[0] << 8) | ((uint16_t)buffer[1]);

    return VL53L4CD_ERR_OK;
}

/**
 * @brief Scrive 2 byte (16 bit) in un registro del dispositivo, tramite il protocollo I2C.
 *
 * @param[in] device                Dispositivo VL53L4CD.
 * @param[in] reg                   Indirizzo del registro.
 * @param[in] data                  Dati da scrivere.
 * @retval VL53L4CD_ERR_OK          Successo.
 * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
 */
static vl53l4cd_err_t write_register_16(vl53l4cd_t *device, const uint16_t reg, uint16_t data)
{
    // Verifica il parametro
    if (!device)
        return VL53L4CD_ERR_INVALID_ARG;

    // Definisce il buffer (big-endian: MSB first)
    uint8_t buffer[2];
    buffer[0] = (uint8_t)(data >> 8);   // MSB
    buffer[1] = (uint8_t)(data & 0xFF); // LSB

    VL53L4CD_ERROR_CHECK(device->platform->i2c_write(device, reg, 2, buffer));

    return VL53L4CD_ERR_OK;
}

/**
 * @brief Legge 4 byte (32 bit) da un registro del dispositivo, tramite il protocollo I2C.
 *
 * @param[in]  device               Dispositivo VL53L4CD.
 * @param[in]  reg                  Indirizzo del registro.
 * @param[out] data                 Dati da leggere.
 * @retval VL53L4CD_ERR_OK          Successo.
 * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
 */
static vl53l4cd_err_t read_register_32(vl53l4cd_t *device, const uint16_t reg, uint32_t *data)
{
    // Verifica i parametri
    if (!device || !data)
        return VL53L4CD_ERR_INVALID_ARG;

    uint8_t buffer[4];
    VL53L4CD_ERROR_CHECK(device->platform->i2c_read(device, reg, 4, buffer));

    // Combina i byte per ricostruire il dato (big-endian: MSB first)
    *data = ((uint32_t)buffer[0] << 24) | ((uint32_t)buffer[1] << 16) | ((uint32_t)buffer[2] << 8) |
        (uint32_t)buffer[3];

    return VL53L4CD_ERR_OK;
}

/**
 * @brief Scrive 4 byte (32 bit) in un registro del dispositivo, tramite il protocollo I2C.
 *
 * @param[in] device                Dispositivo VL53L4CD.
 * @param[in] reg                   Indirizzo del registro.
 * @param[in] data                  Dati da scrivere.
 * @retval VL53L4CD_ERR_OK          Successo.
 * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
 */
static vl53l4cd_err_t write_register_32(vl53l4cd_t *device, const uint16_t reg, uint32_t data)
{
    // Verifica il parametro
    if (!device)
        return VL53L4CD_ERR_INVALID_ARG;

    // Definisce il buffer (big-endian: MSB first)
    uint8_t buffer[4];
    buffer[0] = (uint8_t)(data >> 24);
    buffer[1] = (uint8_t)(data >> 16);
    buffer[2] = (uint8_t)(data >> 8);
    buffer[3] = (uint8_t)(data & 0xFF);

    VL53L4CD_ERROR_CHECK(device->platform->i2c_write(device, reg, 4, buffer));

    return VL53L4CD_ERR_OK;
}

/**
 * @brief Attende l'avvio del dispositivo o lo scadere del timeout.
 *
 * @param[in] device                Dispositivo VL53L4CD.
 * @param[in] timeout               Valore di timeout in millisecondi.
 * @retval VL53L4CD_ERR_OK          Successo.
 * @retval VL53L4CD_ERR_TIMEOUT     Timeout terminato.
 * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
 */
static vl53l4cd_err_t wait_for_boot(vl53l4cd_t *device, uint16_t timeout)
{
    // Verifica il parametro
    if (!device)
        return VL53L4CD_ERR_INVALID_ARG;

    uint8_t state = 0;

    do
    {
        // Ottiene lo stato di boot
        vl53l4cd_get_boot_state(device, &state);

        // Boot avvenuto correttamente
        if (state == 0x03)
            return VL53L4CD_ERR_OK;

        // Attende 1 ms
        device->platform->delay_ms(1);

        // Decrementa il timeout
        timeout--;

    } while (timeout > 0);

    return VL53L4CD_ERR_TIMEOUT;
}

/**
 * @brief Attende l'elaborazione dei dati o lo scadere del timeout.
 *
 * @param[in] device                Dispositivo VL53L4CD.
 * @param[in] timeout               Valore di timeout in millisecondi.
 * @retval VL53L4CD_ERR_OK          Successo.
 * @retval VL53L4CD_ERR_TIMEOUT     Timeout terminato.
 * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
 */
static vl53l4cd_err_t wait_for_data_ready(vl53l4cd_t *device, uint16_t timeout)
{
    // Verifica il parametro
    if (!device)
        return VL53L4CD_ERR_INVALID_ARG;

    uint8_t state = 0;

    do
    {
        // Verifica se il dato è pronto per essere letto
        vl53l4cd_check_for_data_ready(device, &state);

        // Dato pronto
        if (state == 1)
            return VL53L4CD_ERR_OK;

        // Attende 1 ms
        device->platform->delay_ms(1);

        // Decrementa il timeout
        timeout--;

    } while (timeout > 0);

    return VL53L4CD_ERR_TIMEOUT;
}

vl53l4cd_err_t vl53l4cd_init_core(vl53l4cd_t *device)
{
    vl53l4cd_err_t status = VL53L4CD_ERR_OK;

    // Verifica i parametri
    if (!device || !device->platform->delay_ms)
        return VL53L4CD_ERR_INVALID_ARG;

    // Attende l'avvio del sensore (timeout 1000 ms)
    status = wait_for_boot(device, 1000);

    if (status != VL53L4CD_ERR_OK)
        return status;

    // =========================================================
    // Verifica se l'ID del dispositivo coincide con il VL53L4CD
    // =========================================================

    uint16_t sensor_id = 0;
    VL53L4CD_ERROR_CHECK(vl53l4cd_get_device_id(device, &sensor_id));

    if (sensor_id != VL53L4CD_SENSOR_ID)
        return VL53L4CD_ERR_FAIL;

    // ==================================
    // Configura i parametri dei registri
    // ==================================

    /* VL53L4CD_ERROR_CHECK(
        device->platform->i2c_write(device->i2c_device_handle, 0x002D, 91, REG_POR_CONFIG)); */

    // Carica la configurazione di default
    uint8_t address = 0;
    for (address = 0x2D; address <= 0x87; address++)
        VL53L4CD_ERROR_CHECK(write_register_8(device, address, REG_POR_CONFIG[address - 0x2D]));

    // ==================================
    // Calibrazione very-high-voltage VHV
    // ==================================
    VL53L4CD_ERROR_CHECK(write_register_8(device, REG_SYSTEM_START, 0x40));

    // Attende il dato da leggere (timeout 1000 ms)
    status = wait_for_data_ready(device, 1000);

    if (status != VL53L4CD_ERR_OK)
        return status;

    // Elimina gli interrupt pendenti
    VL53L4CD_ERROR_CHECK(vl53l4cd_clear_interrupt(device));

    // Disabilita l'acquisizione dei dati
    VL53L4CD_ERROR_CHECK(vl53l4cd_stop_ranging(device));

    // ====================
    // Configurazioni varie
    // ====================

    VL53L4CD_ERROR_CHECK(write_register_8(device, REG_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09));

    VL53L4CD_ERROR_CHECK(write_register_8(device, 0x0B, 0x00));

    VL53L4CD_ERROR_CHECK(write_register_16(device, 0x0024, 0x0500));

    VL53L4CD_ERROR_CHECK(vl53l4cd_set_range_timing(device, 50, 0));

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_power_on(vl53l4cd_t *device)
{
    // Verifica i parametri
    if (!device || !device->platform->xshut_set_level || !device->platform->delay_ms)
        return VL53L4CD_ERR_INVALID_ARG;

    // Imposta il pin xshut su HIGH (abilita l'alimentazione)
    device->platform->xshut_set_level(device->xshut_pin, 1);
    device->platform->delay_ms(10);

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_power_off(vl53l4cd_t *device)
{
    // Verifica i parametri
    if (!device || !device->platform->xshut_set_level || !device->platform->delay_ms)
        return VL53L4CD_ERR_INVALID_ARG;

    // Imposta il pin xshut su LOW (disabilita l'alimentazione)
    device->platform->xshut_set_level(device->xshut_pin, 0);
    device->platform->delay_ms(10);

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_power_reset(vl53l4cd_t *device)
{
    // Verifica i parametri
    if (!device || !device->platform->xshut_set_level || !device->platform->delay_ms)
        return VL53L4CD_ERR_INVALID_ARG;

    // Imposta il pin xshut su LOW (disabilita l'alimentazione)
    vl53l4cd_power_off(device);

    // Imposta il pin xshut su HIGH (abilita l'alimentazione)
    vl53l4cd_power_on(device);

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_get_boot_state(vl53l4cd_t *device, uint8_t *state)
{
    // Verifica i parametri
    if (!device || !state)
        return VL53L4CD_ERR_INVALID_ARG;

    uint8_t temp = 0;
    VL53L4CD_ERROR_CHECK(read_register_8(device, REG_FIRMWARE_SYSTEM_STATUS, &temp));

    *state = temp;

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_get_device_id(vl53l4cd_t *device, uint16_t *value)
{
    // Verifica i parametri
    if (!device || !value)
        return VL53L4CD_ERR_INVALID_ARG;

    uint16_t temp = 0;
    VL53L4CD_ERROR_CHECK(read_register_16(device, REG_IDENTIFICATION_MODEL_ID, &temp));

    *value = temp;

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_set_i2c_address(vl53l4cd_t *device, const uint8_t address)
{
    // Verifica i parametri
    if (!device || !device->platform->update_address || address < 0x0B || address > 0x77)
        return VL53L4CD_ERR_INVALID_ARG;

    // Attende il corretto avvio del sensore
    VL53L4CD_ERROR_CHECK(wait_for_boot(device, 1000));

    // Cambia l'indirizzo I2C del sensore
    VL53L4CD_ERROR_CHECK(write_register_8(device, REG_I2C_SLAVE_DEVICE_ADDRESS, address));

    // Chiama la funzione per aggiornare gli handle I2C
    VL53L4CD_ERROR_CHECK(device->platform->update_address(device, address));

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_start_ranging(vl53l4cd_t *device)
{
    // Verifica il parametro
    if (!device)
        return VL53L4CD_ERR_INVALID_ARG;

    uint32_t temp = 0;
    VL53L4CD_ERROR_CHECK(read_register_32(device, REG_INTERMEASUREMENT_MS, &temp));

    // Sensore in modalità continua
    if (temp == 0)
        VL53L4CD_ERROR_CHECK(write_register_8(device, REG_SYSTEM_START, 0x21));

    // Sensore in modalità autonoma
    else
        VL53L4CD_ERROR_CHECK(write_register_8(device, REG_SYSTEM_START, 0x40));

    // Elimina un eventuale interrupt pendente
    vl53l4cd_clear_interrupt(device);

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_stop_ranging(vl53l4cd_t *device)
{
    // Verifica il parametro
    if (!device)
        return VL53L4CD_ERR_INVALID_ARG;

    VL53L4CD_ERROR_CHECK(write_register_8(device, REG_SYSTEM_START, 0x00));

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_get_result(vl53l4cd_t *device, vl53l4cd_result_t *result)
{
    // Verifica i parametri
    if (!device || !result)
        return VL53L4CD_ERR_INVALID_ARG;

    uint8_t status_rtn[24] = {255, 255, 255, 5,   2,   4,   1,  7, 3,   0,   255, 255,
                              9,   13,  255, 255, 255, 255, 10, 6, 255, 255, 11,  12};

    // Legge tutti i byte in modo consecutivo (0x89=range_status, 0x97=distance_lsb)
    // Questo approccio è più veloce che leggere singolarmente i registri
    // 0x89 : Registro RANGE_STATUS buffer[0]
    // 0x8A : Riservato
    // 0x8B : Riservato
    // 0x8C : Registro SPAD_NB (MSB) buffer[3]
    // 0x8D : Registro SPAD_NB (LSB) buffer[4]
    // 0x8E : Registro SIGNAL_RATE (MSB) buffer[5]
    // 0x8F : Registro SIGNAL_RATE (LSB) buffer[6]
    // 0x90 : Registro AMBIENT_RATE (MSB) buffer[7]
    // 0x91 : Registro AMBIENT_RATE (LSB) buffer[8]
    // 0x92 : Registro SIGMA (MSB) buffer[9]
    // 0x93 : Registro SIGMA (LSB) buffer[10]
    // 0x94 : Riservato
    // 0x95 : Riservato
    // 0x96 : Registro DISTANCE (MSB) buffer[13]
    // 0x97 : Registro DISTANCE (LSB) buffer[14]

    uint8_t buffer[15];
    VL53L4CD_ERROR_CHECK(device->platform->i2c_read(device, REG_RESULT_RANGE_STATUS, 15, buffer));

    uint8_t status = buffer[0] & 0x1F;
    if (status < 24)
        status = status_rtn[status];
    result->range_status = status;

    // Anche se il campo "number_of_spad" è a 16 bit, questo viene poi
    // diviso per 256, il che equivale a uno shift a destra di 8 bit.
    // In altre parole, la parte LSB di questo campo non viene utilizzata.
    result->number_of_spad = buffer[3];

    if (result->number_of_spad == 0)
    {
        result->range_status = 255;
        return VL53L4CD_ERR_FAIL;
    }

    result->signal_rate_kcps = ((uint16_t)buffer[5] << 8 | (uint16_t)buffer[6]) * 8;
    result->ambient_rate_kcps = ((uint16_t)buffer[7] << 8 | (uint16_t)buffer[8]) * 8;
    result->sigma_mm = ((uint16_t)buffer[9] << 8 | (uint16_t)buffer[10]) / 4;
    result->distance_mm = (uint16_t)buffer[13] << 8 | (uint16_t)buffer[14];
    result->signal_per_spad_kcps = result->signal_rate_kcps / result->number_of_spad;
    result->ambient_per_spad_kcps = result->ambient_rate_kcps / result->number_of_spad;

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_get_range_timing(
    vl53l4cd_t *device,
    uint32_t *timing_budget_ms,
    uint32_t *inter_measurement_ms
)
{
    // Verifica i parametri
    if (!device || !timing_budget_ms || !inter_measurement_ms)
        return VL53L4CD_ERR_INVALID_ARG;

    uint16_t osc_frequency = 1, range_config_macrop_high = 0, clock_pll = 1;
    uint32_t temp = 0, ls_byte = 0, ms_byte = 0, macro_period_us = 0;
    float clock_pll_factor = 1.065f;

    // ===================================
    // Ottiene il valore inter-measurement
    // ===================================

    VL53L4CD_ERROR_CHECK(read_register_32(device, REG_INTERMEASUREMENT_MS, &temp));

    VL53L4CD_ERROR_CHECK(read_register_16(device, REG_RESULT_OSC_CALIBRATE_VAL, &clock_pll));

    clock_pll &= 0x03FF;
    clock_pll_factor *= (float)clock_pll;
    clock_pll = (uint16_t)clock_pll_factor;
    *inter_measurement_ms = (uint16_t)(temp / clock_pll);

    // ===============================
    // Ottiene il valore timing-budget
    // ===============================

    VL53L4CD_ERROR_CHECK(read_register_16(device, 0x0006, &osc_frequency));

    VL53L4CD_ERROR_CHECK(read_register_16(device, REG_RANGE_CONFIG_A, &range_config_macrop_high));

    macro_period_us = (2304 * (0x40000000 / osc_frequency)) >> 6;
    ls_byte = (range_config_macrop_high & 0x00FF) << 4;
    ms_byte = (range_config_macrop_high & 0xFF00) >> 8;
    ms_byte = 0x04 - (ms_byte - 1) - 1;

    macro_period_us *= 16;
    *timing_budget_ms =
        (((ls_byte + 1) * (macro_period_us >> 6)) - ((macro_period_us >> 6) >> 1)) >> 12;

    if (ms_byte < 12)
        *timing_budget_ms >>= (uint8_t)ms_byte;

    // Modalità continua
    if (temp == 0)
    {
        *timing_budget_ms += 2500;
    }

    // Modalità autonoma
    else
    {
        *timing_budget_ms *= 2;
        *timing_budget_ms += 4300;
    }

    *timing_budget_ms /= 1000;

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_set_range_timing(
    vl53l4cd_t *device,
    const uint32_t timing_budget_ms,
    const uint32_t inter_measurement_ms
)
{
    // Verifica i parametri
    if (!device || timing_budget_ms < 10 || timing_budget_ms > 200)
        return VL53L4CD_ERR_INVALID_ARG;

    uint16_t clock_pll = 0, osc_frequency = 0, ms_byte = 0;
    uint32_t macro_period_us = 0, timing_budget_us = 0, ls_byte = 0, temp = 0;
    float inter_measurement_factor = 1.055f;

    VL53L4CD_ERROR_CHECK(read_register_16(device, 0x0006, &osc_frequency));

    if (osc_frequency != 0)
    {
        timing_budget_us = timing_budget_ms * 1000;

        macro_period_us =
            (uint32_t)((uint32_t)2304 * ((uint32_t)0x40000000 / (uint32_t)osc_frequency)) >> 6;
    } else
    {
        return VL53L4CD_ERR_INVALID_ARG;
    }

    // Il sensore è in modalità continua
    if (inter_measurement_ms == 0)
    {
        VL53L4CD_ERROR_CHECK(write_register_32(device, REG_INTERMEASUREMENT_MS, 0x00));
        timing_budget_us -= 2500;
    }

    // Il sensore è in modalità autonoma con risparmio energetico
    else if (inter_measurement_ms > timing_budget_ms)
    {
        VL53L4CD_ERROR_CHECK(read_register_16(device, REG_RESULT_OSC_CALIBRATE_VAL, &clock_pll));

        clock_pll &= 0x3FF;
        inter_measurement_factor *= (float)inter_measurement_ms * (float)clock_pll;

        VL53L4CD_ERROR_CHECK(
            write_register_32(device, REG_INTERMEASUREMENT_MS, (uint32_t)inter_measurement_factor)
        );

        timing_budget_us -= 4300;
        timing_budget_us /= 2;
    } else
    {
        return VL53L4CD_ERR_INVALID_ARG;
    }

    ms_byte = 0;
    timing_budget_us <<= 12;
    temp = macro_period_us * 16;
    ls_byte = ((timing_budget_us + ((temp >> 6) >> 1)) / (temp >> 6)) - 1;

    while ((ls_byte & 0xFFFFFF00U) > 0)
    {
        ls_byte >>= 1;
        ms_byte++;
    }
    ms_byte = (ms_byte << 8) + (ls_byte & 0xFF);

    VL53L4CD_ERROR_CHECK(write_register_16(device, REG_RANGE_CONFIG_A, ms_byte));

    ms_byte = 0;
    temp = macro_period_us * 12;
    ls_byte = ((timing_budget_us + ((temp >> 6) >> 1)) / (temp >> 6)) - 1;

    while ((ls_byte & 0xFFFFFF00U) > 0)
    {
        ls_byte >>= 1;
        ms_byte++;
    }

    ms_byte = (ms_byte << 8) + (ls_byte & 0xFF);
    VL53L4CD_ERROR_CHECK(write_register_16(device, REG_RANGE_CONFIG_B, ms_byte));

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_get_offset(vl53l4cd_t *device, int16_t *offset_mm)
{
    // Verifica i parametri
    if (!device || !offset_mm)
        return VL53L4CD_ERR_INVALID_ARG;

    uint16_t temp = 0;
    VL53L4CD_ERROR_CHECK(read_register_16(device, REG_RANGE_OFFSET_MM, &temp));

    temp <<= 3;
    temp >>= 5;
    *offset_mm = (int16_t)(temp);

    if (*offset_mm > 1024)
        *offset_mm -= 2048;

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_set_offset(vl53l4cd_t *device, const int16_t offset_mm)
{
    // Verifica i parametri
    if (!device || offset_mm < -1024 || offset_mm > 1023)
        return VL53L4CD_ERR_INVALID_ARG;

    uint16_t temp = (uint16_t)((uint16_t)offset_mm * 4);

    VL53L4CD_ERROR_CHECK(read_register_16(device, REG_RANGE_OFFSET_MM, &temp));

    VL53L4CD_ERROR_CHECK(write_register_16(device, REG_INNER_OFFSET_MM, 0x0000));

    VL53L4CD_ERROR_CHECK(write_register_16(device, REG_OUTER_OFFSET_MM, 0x0000));

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_get_xtalk(vl53l4cd_t *device, uint16_t *xtalk_kcps)
{
    // Verifica i parametri
    if (!device || !xtalk_kcps)
        return VL53L4CD_ERR_INVALID_ARG;

    uint16_t temp = 0;
    VL53L4CD_ERROR_CHECK(read_register_16(device, REG_XTALK_PLANE_OFFSET_KCPS, &temp));

    *xtalk_kcps = (uint16_t)(temp * 1000) >> 9;

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_set_xtalk(vl53l4cd_t *device, uint16_t xtalk_kcps)
{
    // Verifica i parametri
    if (!device || xtalk_kcps > 128)
        return VL53L4CD_ERR_INVALID_ARG;

    xtalk_kcps = (xtalk_kcps << 9) / 1000;

    VL53L4CD_ERROR_CHECK(write_register_16(device, REG_XTALK_X_PLANE_GRADIENT_KCPS, 0x0000));

    VL53L4CD_ERROR_CHECK(write_register_16(device, REG_XTALK_Y_PLANE_GRADIENT_KCPS, 0x0000));

    VL53L4CD_ERROR_CHECK(write_register_16(device, REG_XTALK_PLANE_OFFSET_KCPS, xtalk_kcps));

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_get_detection_thresholds(
    vl53l4cd_t *device,
    uint16_t *low_dist_mm,
    uint16_t *high_dist_mm,
    uint8_t *window
)
{
    // Verifica i parametri
    if (!device || !low_dist_mm || !high_dist_mm || !window)
        return VL53L4CD_ERR_INVALID_ARG;

    uint8_t temp_8 = 0;
    uint16_t temp_16 = 0;

    VL53L4CD_ERROR_CHECK(read_register_16(device, REG_THRESH_LOW, &temp_16));

    *low_dist_mm = temp_16;

    VL53L4CD_ERROR_CHECK(read_register_16(device, REG_THRESH_HIGH, &temp_16));

    *high_dist_mm = temp_16;

    VL53L4CD_ERROR_CHECK(read_register_8(device, REG_SYSTEM_INTERRUPT, &temp_8));

    *window = temp_8 & 0x07;

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_set_detection_thresholds(
    vl53l4cd_t *device,
    const uint16_t low_dist_mm,
    const uint16_t high_dist_mm,
    const uint8_t window
)
{
    // Verifica i parametri
    if (!device || high_dist_mm > 1200 || low_dist_mm > high_dist_mm || window > 3)
        return VL53L4CD_ERR_INVALID_ARG;

    VL53L4CD_ERROR_CHECK(write_register_16(device, REG_THRESH_LOW, low_dist_mm));

    VL53L4CD_ERROR_CHECK(write_register_16(device, REG_THRESH_HIGH, high_dist_mm));

    VL53L4CD_ERROR_CHECK(write_register_8(device, REG_SYSTEM_INTERRUPT, window));

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_get_signal_threshold(vl53l4cd_t *device, uint16_t *signal_kcps)
{
    // Verifica i parametri
    if (!device || !signal_kcps)
        return VL53L4CD_ERR_INVALID_ARG;

    uint16_t temp = 0;
    VL53L4CD_ERROR_CHECK(read_register_16(device, REG_MIN_COUNT_RATE_RTN_LIMIT_MCPS, &temp));

    *signal_kcps = temp << 3;

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_set_signal_threshold(vl53l4cd_t *device, uint16_t signal_kcps)
{
    // Verifica i parametri
    if (!device || signal_kcps > 16383)
        return VL53L4CD_ERR_INVALID_ARG;

    signal_kcps >>= 3;

    VL53L4CD_ERROR_CHECK(write_register_16(device, REG_MIN_COUNT_RATE_RTN_LIMIT_MCPS, signal_kcps));

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_get_sigma_threshold(vl53l4cd_t *device, uint16_t *sigma_mm)
{
    // Verifica i parametri
    if (!device || !sigma_mm)
        return VL53L4CD_ERR_INVALID_ARG;

    uint16_t temp = 0;
    VL53L4CD_ERROR_CHECK(read_register_16(device, REG_RANGE_CONFIG_SIGMA_THRESH, &temp));

    *sigma_mm = temp >> 2;

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_set_sigma_threshold(vl53l4cd_t *device, uint16_t sigma_mm)
{
    // Verifica i parametri
    if (!device || sigma_mm > 16383)
        return VL53L4CD_ERR_INVALID_ARG;

    sigma_mm <<= 2;

    VL53L4CD_ERROR_CHECK(write_register_16(device, REG_RANGE_CONFIG_SIGMA_THRESH, sigma_mm));

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_get_interrupt_polarity(vl53l4cd_t *device, uint8_t *polarity)
{
    // Verifica i parametri
    if (!device || !polarity)
        return VL53L4CD_ERR_INVALID_ARG;

    uint8_t temp = 0;
    VL53L4CD_ERROR_CHECK(read_register_8(device, REG_GPIO_HV_MUX_CTRL, &temp));

    // Ottiene il solo valore del bit 4 (0x10 = 0b00010000)
    temp &= 0x10;

    *polarity = !(temp >> 4);

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_set_interrupt_polarity(vl53l4cd_t *device, const uint8_t polarity)
{
    // Verifica i parametri
    if (!device || polarity > 1)
        return VL53L4CD_ERR_INVALID_ARG;

    uint8_t temp = 0;
    VL53L4CD_ERROR_CHECK(read_register_8(device, REG_GPIO_HV_MUX_CTRL, &temp));

    // Imposta a zero il bit 4 (0xEF = 0b11101111)
    temp &= 0xEF;

    // Il seguente codice esegue:
    //  - Ottiene il solo valore del bit 0
    //  - Inverte uno con zero e viceversa
    //  - Sposta a sinistra di quattro posizioni il valore
    //  - Assegna il valore alla variabile temp
    temp |= (!(polarity & 0x01)) << 4;

    VL53L4CD_ERROR_CHECK(write_register_8(device, REG_GPIO_HV_MUX_CTRL, temp));

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_check_for_data_ready(vl53l4cd_t *device, uint8_t *value)
{
    // Verifica i parametri
    if (!device || !value)
        return VL53L4CD_ERR_INVALID_ARG;

    uint8_t temp = 0, int_pol = 0;

    VL53L4CD_ERROR_CHECK(vl53l4cd_get_interrupt_polarity(device, &int_pol));

    VL53L4CD_ERROR_CHECK(read_register_8(device, REG_GPIO_TIO_HV_STATUS, &temp));

    *value = ((temp & 0x01) == int_pol) ? 1 : 0;

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_clear_interrupt(vl53l4cd_t *device)
{
    // Verifica il parametro
    if (!device)
        return VL53L4CD_ERR_INVALID_ARG;

    VL53L4CD_ERROR_CHECK(write_register_8(device, REG_SYSTEM_INTERRUPT_CLEAR, 0x01));

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_set_isr_handler(
    vl53l4cd_t *device,
    vl53l4cd_isr_handler_t handler,
    void *context
)
{
    // Verifica i parametri
    if (!device || !handler)
        return VL53L4CD_ERR_INVALID_ARG;

    device->isr_handler = handler;
    device->context = context;

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_calibrate_offset(
    vl53l4cd_t *device,
    const int16_t target_dist_mm,
    const uint8_t nb_samples,
    int16_t *measured_offset_mm
)
{
    // Verifica i parametri
    if (!device || !measured_offset_mm || target_dist_mm < 50 || target_dist_mm > 1000 ||
        nb_samples < 5)
        return VL53L4CD_ERR_INVALID_ARG;

    uint16_t tmp_off;
    int16_t avg_distance = 0;
    vl53l4cd_result_t results;

    VL53L4CD_ERROR_CHECK(write_register_16(device, REG_RANGE_OFFSET_MM, 0x0000));

    VL53L4CD_ERROR_CHECK(write_register_16(device, REG_INNER_OFFSET_MM, 0x0000));

    VL53L4CD_ERROR_CHECK(write_register_16(device, REG_OUTER_OFFSET_MM, 0x0000));

    // ==================================================
    // Esegue 10 misurazioni per preriscaldare il sensore
    // ==================================================

    // Avvia l'acquisizione dei dati
    VL53L4CD_ERROR_CHECK(vl53l4cd_start_ranging(device));

    for (size_t i = 0; i < 10; i++)
    {
        // Attende il dato da leggere (timeout 5000 ms)
        VL53L4CD_ERROR_CHECK(wait_for_data_ready(device, 5000));

        // Legge i risultati del sensore
        VL53L4CD_ERROR_CHECK(vl53l4cd_get_result(device, &results));

        // Elimina l'interrupt
        VL53L4CD_ERROR_CHECK(vl53l4cd_clear_interrupt(device));
    }

    // Termina l'acquisizione dei dati
    VL53L4CD_ERROR_CHECK(vl53l4cd_stop_ranging(device));

    // ==================================
    // Esegue la calibrazione dell'offset
    // ==================================

    // Avvia l'acquisizione dei dati
    VL53L4CD_ERROR_CHECK(vl53l4cd_start_ranging(device));

    for (size_t i = 0; i < nb_samples; i++)
    {
        // Attende il dato da leggere (timeout 5000 ms)
        VL53L4CD_ERROR_CHECK(wait_for_data_ready(device, 5000));

        // Legge i risultati del sensore
        VL53L4CD_ERROR_CHECK(vl53l4cd_get_result(device, &results));

        // Elimina l'interrupt
        VL53L4CD_ERROR_CHECK(vl53l4cd_clear_interrupt(device));

        avg_distance += (int16_t)results.distance_mm;
    }

    // Termina l'acquisizione dei dati
    VL53L4CD_ERROR_CHECK(vl53l4cd_stop_ranging(device));

    avg_distance /= nb_samples;
    *measured_offset_mm = target_dist_mm - avg_distance;
    tmp_off = (uint16_t)*measured_offset_mm * 4;

    VL53L4CD_ERROR_CHECK(write_register_16(device, REG_RANGE_OFFSET_MM, tmp_off));

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_calibrate_xtalk(
    vl53l4cd_t *device,
    const int16_t target_dist_mm,
    const uint8_t nb_samples,
    uint16_t *measured_xtalk_kcps
)
{
    // Verifica i parametri
    if (!device || !measured_xtalk_kcps || target_dist_mm < 50 || target_dist_mm > 1000 ||
        nb_samples < 5)
        return VL53L4CD_ERR_INVALID_ARG;

    uint16_t cal_xtalk = 0;
    float average_signal = 0.0f;
    float average_distance = 0.0f;
    float average_spad_nb = 0.0f;
    float tmp_xtalk;
    vl53l4cd_result_t results;

    // ==================================================
    // Esegue 10 misurazioni per preriscaldare il sensore
    // ==================================================

    // Avvia l'acquisizione dei dati
    VL53L4CD_ERROR_CHECK(vl53l4cd_start_ranging(device));

    for (size_t i = 0; i < 10; i++)
    {
        // Attende il dato da leggere (timeout 5000 ms)
        VL53L4CD_ERROR_CHECK(wait_for_data_ready(device, 5000));

        // Legge i risultati del sensore
        VL53L4CD_ERROR_CHECK(vl53l4cd_get_result(device, &results));

        // Elimina l'interrupt
        VL53L4CD_ERROR_CHECK(vl53l4cd_clear_interrupt(device));
    }

    // Termina l'acquisizione dei dati
    VL53L4CD_ERROR_CHECK(vl53l4cd_stop_ranging(device));

    // =================================
    // Esegue la calibrazione dell'xtalk
    // =================================

    // Avvia l'acquisizione dei dati
    VL53L4CD_ERROR_CHECK(vl53l4cd_start_ranging(device));

    for (size_t i = 0; i < nb_samples; i++)
    {
        // Attende il dato da leggere (timeout 5000 ms)
        VL53L4CD_ERROR_CHECK(wait_for_data_ready(device, 5000));

        // Legge i risultati del sensore
        VL53L4CD_ERROR_CHECK(vl53l4cd_get_result(device, &results));

        // Elimina l'interrupt
        VL53L4CD_ERROR_CHECK(vl53l4cd_clear_interrupt(device));

        average_distance += (float)results.distance_mm;
        average_spad_nb += (float)results.number_of_spad;
        average_signal += (float)results.signal_rate_kcps;
    }

    // Termina l'acquisizione dei dati
    VL53L4CD_ERROR_CHECK(vl53l4cd_stop_ranging(device));

    average_distance /= (float)nb_samples;
    average_spad_nb /= (float)nb_samples;
    average_signal /= (float)nb_samples;

    tmp_xtalk = 512.0f * (average_signal * (1.0f - (average_distance / (float)target_dist_mm))) /
        average_spad_nb;
    cal_xtalk = (uint16_t)tmp_xtalk;
    *measured_xtalk_kcps = (uint16_t)(cal_xtalk * 1000) >> 9;

    VL53L4CD_ERROR_CHECK(write_register_16(device, REG_XTALK_PLANE_OFFSET_KCPS, cal_xtalk));

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_start_temperature_update(vl53l4cd_t *device)
{
    // Verifica il parametro
    if (!device)
        return VL53L4CD_ERR_INVALID_ARG;

    VL53L4CD_ERROR_CHECK(write_register_8(device, REG_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x81));

    VL53L4CD_ERROR_CHECK(write_register_8(device, 0x0B, 0x92));

    VL53L4CD_ERROR_CHECK(write_register_8(device, REG_SYSTEM_START, 0x40));

    // Attende il dato da leggere (timeout 1000 ms)
    VL53L4CD_ERROR_CHECK(wait_for_data_ready(device, 1000));

    // Elimina l'interrupt
    VL53L4CD_ERROR_CHECK(vl53l4cd_clear_interrupt(device));

    // Termina l'acquisizione dei dati
    VL53L4CD_ERROR_CHECK(vl53l4cd_stop_ranging(device));

    VL53L4CD_ERROR_CHECK(write_register_8(device, REG_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09));

    VL53L4CD_ERROR_CHECK(write_register_8(device, 0x0B, 0));

    return VL53L4CD_ERR_OK;
}
