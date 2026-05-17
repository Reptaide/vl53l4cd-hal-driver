/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2026 Reptaide.
 *
 * Redistributed under the terms of the BSD 3-Clause License.
 * See LICENSE in the root of this repository for the full license text.
 */

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vl53l4cd_core.h"
#include "vl53l4cd_platform.h"

#include <stdint.h>
#include <stdio.h>

#define I2C_DEFAULT_ADDRESS 0x29 // Indirizzo I2C di default
#define I2C_PORT I2C_NUM_0       // Numero porta I2C
#define I2C_PIN_SCL 10           // SCL
#define I2C_PIN_SDA 11           // SDA
#define I2C_SCL_SPEED_HZ 100000  // 100 kHz
#define I2C_TIMEOUT 200          // Timeout I2C in ms
#define XSHUT_PIN 14             // Alimentazione
#define POLLING_PERIOD_MS 10     // Periodo di polling

// Configurazione della finestra di rilevamento.
#define DETECTION_LOW_MM 100  // Soglia inferiore in mm
#define DETECTION_HIGH_MM 500 // Soglia superiore in mm
#define DETECTION_WINDOW 3    // 0: sotto low, 1: sopra high, 2: fuori range, 3; dentro range

// Definisce il tag per il debugging
static const char *TAG = "main";

/**
 * @brief Inizializza e configura il bus I2C.
 *
 * @param[in] i2c_bus_handle Handle del bus I2C.
 */
void i2c_bus_init(i2c_master_bus_handle_t *i2c_bus_handle)
{
    // Configura il bus I2C
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_PIN_SDA,
        .scl_io_num = I2C_PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };

    // Inizializza il bus I2C
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, i2c_bus_handle));

    ESP_LOGI(TAG, "I2C bus initialized successfully");
}

void app_main(void)
{
    // Handler del bus I2C
    i2c_master_bus_handle_t i2c_bus_handle = NULL;

    // Inizializza il bus I2C
    i2c_bus_init(&i2c_bus_handle);

    // Definisce e inizializza a zero il dispositivo
    vl53l4cd_t device = {0};

    // Inizializza il platform per comunicare con l'hardware
    vl53l4cd_init_hal(&device, i2c_bus_handle, I2C_DEFAULT_ADDRESS, I2C_SCL_SPEED_HZ, I2C_TIMEOUT);

    // Configura il pin di alimentazione
    vl53l4cd_hal_setup_xshut(&device, XSHUT_PIN);

    // Accende il dispositivo
    vl53l4cd_power_on(&device);

    // Inizializza il core del dispositivo
    vl53l4cd_init_core(&device);

    // Ottiene l'ID del dispositivo
    uint16_t sensor_id = 0;
    vl53l4cd_get_device_id(&device, &sensor_id);
    ESP_LOGI(TAG, "Device ID: 0x%04X", sensor_id);

    // Configura la modalità di rilevamento specificata dalla window
    vl53l4cd_set_detection_thresholds(
        &device, DETECTION_LOW_MM, DETECTION_HIGH_MM, DETECTION_WINDOW);

    ESP_LOGI(TAG,
        "Detection range: [%u, %u] mm | Mode: %u",
        DETECTION_LOW_MM,
        DETECTION_HIGH_MM,
        DETECTION_WINDOW);

    // Avvia le misurazioni
    vl53l4cd_start_ranging(&device);

    uint8_t data_ready = 0;
    vl53l4cd_result_t result;

    for (;;)
    {
        // Verifica quando sono pronti nuovi dati
        vl53l4cd_check_for_data_ready(&device, &data_ready);

        // Continua se i dati non sono pronti
        if (!data_ready)
            continue;

        vl53l4cd_err_t status = vl53l4cd_get_result(&device, &result);

        if (status == VL53L4CD_ERR_OK && result.range_status == 0)
            ESP_LOGI(TAG,
                "[0x%02X] Distance: %6u mm | Signal: %6u",
                device.i2c_address,
                result.distance_mm,
                result.signal_per_spad_kcps);
        else
            ESP_LOGD(TAG, "[0x%02X] Status: %u", device.i2c_address, result.range_status);

        // Elimina l'interrupt pendente per riarmare il flag data-ready
        vl53l4cd_clear_interrupt(&device);

        // Delay temporale in millisecondi
        vTaskDelay(pdMS_TO_TICKS(POLLING_PERIOD_MS));
    }
}
