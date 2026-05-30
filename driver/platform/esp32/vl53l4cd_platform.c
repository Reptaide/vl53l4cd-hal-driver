/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2026 Reptaide.
 *
 * Redistributed under the terms of the BSD 3-Clause License.
 * See LICENSE in the root of this repository for the full license text.
 */

#include "vl53l4cd_platform.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vl53l4cd_core.h"

#include <string.h>

/**
 * @brief Questa è la funzione che viene chiamata dopo un evento di interrupt.
 *
 * @param[in] arg Puntatore generico contenente varie informazioni.
 */
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    vl53l4cd_t *device = (vl53l4cd_t *)arg;

    if (!device || !device->isr_handler)
        return;

    device->isr_handler(device, device->context);
}

/**
 * @brief Implementa la logica per leggere il buffer I2C tramite ESP32.
 *
 * @param[in]  handle               Dispositivo VL53L4CD.
 * @param[in]  reg                  Indirizzo del registro.
 * @param[in]  length               Dimensione del buffer.
 * @param[out] data                 Buffer dei dati da ricevere.
 * @retval VL53L4CD_ERR_OK          Successo.
 * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
 * @retval VL53L4CD_ERR_FAIL        Errore durante la ricezione dei dati.
 * @retval VL53L4CD_ERR_TIMEOUT     Timeout raggiunto.
 */
static vl53l4cd_err_t i2c_read_register(
    void *handle,
    const uint16_t reg,
    const size_t length,
    uint8_t *data
)
{
    // Verifica i parametri
    if (!handle || !data || length == 0)
        return VL53L4CD_ERR_INVALID_ARG;

    esp_err_t status = ESP_OK;

    // Ottiene la struttura del dispositivo
    vl53l4cd_t *device = (vl53l4cd_t *)handle;

    // Definisce il buffer di trasmissione (big-endian: MSB first)
    uint8_t tx_buffer[2] = {
        (uint8_t)(reg >> 8),  // Registro MSB
        (uint8_t)(reg & 0xFF) // Registro LSB
    };

    // Invia il comando di scrittura e lettura nel bus I2C
    status = i2c_master_transmit_receive(
        (i2c_master_dev_handle_t)device->i2c_device_handle,
        tx_buffer,
        2,
        data,
        length,
        device->i2c_timeout
    );

    if (status == ESP_ERR_TIMEOUT)
        return VL53L4CD_ERR_TIMEOUT;

    else if (status != ESP_OK)
        return VL53L4CD_ERR_FAIL;

    return VL53L4CD_ERR_OK;
}

/**
 * @brief Implementa la logica per scrivere il buffer I2C tramite ESP32.
 *
 * @param[in] handle                Dispositivo VL53L4CD.
 * @param[in] reg                   Indirizzo del registro.
 * @param[in] length                Dimensione del buffer.
 * @param[in] data                  Buffer dei dati da inviare.
 * @retval VL53L4CD_ERR_OK          Successo.
 * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
 * @retval VL53L4CD_ERR_FAIL        Errore durante la ricezione dei dati.
 * @retval VL53L4CD_ERR_TIMEOUT     Timeout raggiunto.
 */
static vl53l4cd_err_t i2c_write_register(
    void *handle,
    const uint16_t reg,
    const size_t length,
    const uint8_t *data
)
{
    // Verifica i parametri
    if (!handle || !data || length == 0 || length > 32)
        return VL53L4CD_ERR_INVALID_ARG;

    // Ottiene la struttura del dispositivo
    vl53l4cd_t *device = (vl53l4cd_t *)handle;

    // Alloca un buffer di trasmissione di 34 byte: 2 per il registro + 32 per i dati
    uint8_t tx_buffer[34];

    // Popola il buffer con il registro (big-endian: MSB first)
    tx_buffer[0] = (uint8_t)(reg >> 8);
    tx_buffer[1] = (uint8_t)(reg & 0xFF);

    // Copia i dati nel buffer
    memcpy(tx_buffer + 2, data, length);

    esp_err_t status = i2c_master_transmit(
        (i2c_master_dev_handle_t)device->i2c_device_handle,
        tx_buffer,
        length + 2,
        device->i2c_timeout
    );

    if (status == ESP_ERR_TIMEOUT)
        return VL53L4CD_ERR_TIMEOUT;

    else if (status != ESP_OK)
        return VL53L4CD_ERR_FAIL;

    return VL53L4CD_ERR_OK;
}

/**
 * @brief Implementa la logica per aggiornare a runtime l'indirizzo I2C tramite ESP32.
 *
 * @param[in] handle                Dispositivo VL53L4CD.
 * @param[in] address               Nuovo indirizzo I2C.
 * @retval VL53L4CD_ERR_OK          Successo.
 * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
 * @retval VL53L4CD_ERR_FAIL        Errore durante la ricezione dei dati.
 */
static vl53l4cd_err_t update_i2c_address(void *handle, const uint8_t address)
{
    // Verifica il parametro
    if (!handle)
        return VL53L4CD_ERR_INVALID_ARG;

    esp_err_t status = ESP_OK;

    // Ottiene la struttura del dispositivo
    vl53l4cd_t *device = (vl53l4cd_t *)handle;

    // Rimuove il dispositivo dal bus I2C
    status = i2c_master_bus_rm_device(device->i2c_device_handle);

    if (status != ESP_OK)
        return VL53L4CD_ERR_FAIL;

    // Aggiorna la configurazione I2C con il nuovo indirizzo
    i2c_device_config_t i2c_device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = device->i2c_scl_speed,
    };

    // Aggiunge il sensore con il nuovo indirizzo al bus I2C
    i2c_master_dev_handle_t new_device_handle;
    status =
        i2c_master_bus_add_device(device->i2c_bus_handle, &i2c_device_config, &new_device_handle);

    if (status != ESP_OK)
        return VL53L4CD_ERR_FAIL;

    // Aggiorna l'handle e l'indirizzo I2C del dispositivo
    device->i2c_device_handle = (void *)new_device_handle;
    device->i2c_address = address;

    return VL53L4CD_ERR_OK;
}

/**
 * @brief Implementa la logica per impostare lo stato logico di un pin tramite ESP32.
 *
 * @param[in] pin                   Dispositivo VL53L4CD.
 * @param[in] value                 Nuovo indirizzo I2C.
 * @retval VL53L4CD_ERR_OK          Successo.
 * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
 * @retval VL53L4CD_ERR_FAIL        Errore durante la ricezione dei dati.
 */
static vl53l4cd_err_t set_gpio_state(const uint8_t pin, const uint8_t value)
{
    // Verifica i parametri
    if (pin > GPIO_NUM_MAX || value > 1)
        return VL53L4CD_ERR_INVALID_ARG;

    // Imposta il livello logico del pin
    esp_err_t status = gpio_set_level((gpio_num_t)pin, value);

    if (status != ESP_OK)
        return VL53L4CD_ERR_INVALID_ARG;

    return VL53L4CD_ERR_OK;
}

/**
 * @brief Applica un delay bloccante in millisecondi.
 *
 * @param[in] ms Valore del ritardo in ms.
 */
static void delay_ms(const uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

static const vl53l4cd_platform_t vl53l4cd_platform = {
    .i2c_read = i2c_read_register,
    .i2c_write = i2c_write_register,
    .update_address = update_i2c_address,
    .xshut_set_level = set_gpio_state,
    .delay_ms = delay_ms,
};

vl53l4cd_err_t vl53l4cd_init_hal(
    vl53l4cd_t *device,
    i2c_master_bus_handle_t bus_handle,
    const uint8_t i2c_address,
    const uint32_t i2c_scl_speed,
    const int32_t i2c_timeout
)
{
    // Verifica i parametri
    if (!device || !bus_handle)
        return VL53L4CD_ERR_INVALID_ARG;

    if (i2c_address < 0x08 || i2c_address > 0x77)
        return VL53L4CD_ERR_INVALID_ARG;

    esp_err_t status = ESP_OK;
    i2c_master_dev_handle_t device_handle;

    // Configura l'handle I2C del dispositivo
    i2c_device_config_t i2c_device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_address,
        .scl_speed_hz = i2c_scl_speed,
    };

    // Inizializza il dispositivo sul bus I2C
    status = i2c_master_bus_add_device(bus_handle, &i2c_device_config, &device_handle);

    if (status != ESP_OK)
        return VL53L4CD_ERR_FAIL;

    // Inizializza i campi restanti del dispositivo
    device->i2c_bus_handle = (void *)bus_handle;
    device->i2c_device_handle = (void *)device_handle;
    device->i2c_address = i2c_address;
    device->i2c_scl_speed = i2c_scl_speed;
    device->i2c_timeout = i2c_timeout;
    device->xshut_pin = GPIO_NUM_NC;
    device->int_pin = GPIO_NUM_NC;
    device->context = NULL;
    device->isr_handler = NULL;
    device->platform = &vl53l4cd_platform;

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_hal_setup_xshut(vl53l4cd_t *device, const uint8_t xshut_pin)
{
    // Verifica i parametri
    if (!device || xshut_pin > GPIO_NUM_MAX)
        return VL53L4CD_ERR_INVALID_ARG;

    esp_err_t status = ESP_OK;

    // Configurazione del pin XSHUT
    gpio_config_t xshut_pin_config = {
        .intr_type = GPIO_INTR_DISABLE,        // Disabilita l'interrupt sul GPIO
        .mode = GPIO_MODE_OUTPUT,              // Configura il pin come output
        .pin_bit_mask = (1ULL << xshut_pin),   // Valore del pin da configurare
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disabilita il pull-down interno sul pin
        .pull_up_en = GPIO_PULLUP_ENABLE,      // Disabilita il pull-up interno sul pin
    };

    // Applica la configurazione al GPIO
    status = gpio_config(&xshut_pin_config);

    if (status != ESP_OK)
        return VL53L4CD_ERR_FAIL;

    // Aggiorna i campi del dispositivo
    device->xshut_pin = xshut_pin;

    return VL53L4CD_ERR_OK;
}

vl53l4cd_err_t vl53l4cd_hal_setup_int(vl53l4cd_t *device, const uint8_t int_pin)
{
    // Verifica i parametri
    if (!device || int_pin > GPIO_NUM_MAX)
        return VL53L4CD_ERR_INVALID_ARG;

    esp_err_t status = ESP_OK;

    // Configurazione del pin INT
    gpio_config_t int_pin_config = {
        .intr_type = GPIO_INTR_NEGEDGE,        // Imposta l'interrupt active-low
        .mode = GPIO_MODE_INPUT,               // Configura il pin come input
        .pin_bit_mask = (1ULL << int_pin),     // Valore del pin da configurare
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disabilita il pull-down interno sul pin
        .pull_up_en = GPIO_PULLUP_ENABLE,      // Abilita il pull-up interno sul pin
    };

    // Applica la configurazione al GPIO
    status = gpio_config(&int_pin_config);

    if (status != ESP_OK)
        return VL53L4CD_ERR_FAIL;

    // Aggiunge l'ISR handler al GPIO
    status = gpio_isr_handler_add(int_pin, gpio_isr_handler, device);

    if (status != ESP_OK)
        return VL53L4CD_ERR_FAIL;

    // Aggiorna i campi del dispositivo
    device->int_pin = int_pin;

    return VL53L4CD_ERR_OK;
}
