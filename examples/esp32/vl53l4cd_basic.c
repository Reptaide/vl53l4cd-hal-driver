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

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>

#define I2C_DEFAULT_ADDRESS 0x29 // Indirizzo I2C di default
#define I2C_PORT I2C_NUM_0       // Numero porta I2C
#define I2C_PIN_SCL 10           // SCL
#define I2C_PIN_SDA 11           // SDA
#define I2C_SCL_SPEED_HZ 100000  // 100 kHz
#define I2C_TIMEOUT 200          // Timeout I2C in ms
#define XSHUT_PIN 14             // // Alimentazione

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

    // Ottiene lo stato di boot del dispositivo
    uint8_t boot_state = 0;
    vl53l4cd_get_boot_state(&device, &boot_state);
    ESP_LOGI(TAG, "Boot state: 0x%02X (expected: 0x03)", boot_state);

    // Inizializza il core del dispositivo
    vl53l4cd_init_core(&device);

    // Ottiene l'ID del dispositivo
    uint16_t sensor_id = 0;
    vl53l4cd_get_device_id(&device, &sensor_id);
    ESP_LOGI(TAG, "Device ID: 0x%04X (expected: 0xEBAA)", sensor_id);

    // Ottine la durata della misurazione e la pausa tra misurazioni in modalità autonoma.
    uint32_t timing_budget_ms = 0;
    uint32_t inter_measurement_ms = 0;
    vl53l4cd_get_range_timing(&device, &timing_budget_ms, &inter_measurement_ms);
    ESP_LOGI(TAG,
        "Timing budget: %u ms | Inter-measurement: %u ms",
        timing_budget_ms,
        inter_measurement_ms);

    // Ottiene la polarità del pin di interrupt
    uint8_t int_polarity = 0;
    vl53l4cd_get_interrupt_polarity(&device, &int_polarity);
    ESP_LOGI(TAG, "Interrupt polarity: %u (0: active-low, 1: active-high)", int_polarity);

    // Ottiene l'offset di correzione applicato alla distanza
    int16_t offset_mm = 0;
    vl53l4cd_get_offset(&device, &offset_mm);
    ESP_LOGI(TAG, "Offset: %d mm", offset_mm);

    // Ottiene il valore di correzione per la protezione in vetro davanti al sensore
    uint16_t xtalk_kcps = 0;
    vl53l4cd_get_xtalk(&device, &xtalk_kcps);
    ESP_LOGI(TAG, "Xtalk: %u kcps", xtalk_kcps);

    // Ottiene le soglie di qualità della misurazione (segnale minimo / rumore massimo)
    uint16_t signal_threshold_kcps = 0;
    uint16_t sigma_threshold_mm = 0;
    vl53l4cd_get_signal_threshold(&device, &signal_threshold_kcps);
    vl53l4cd_get_sigma_threshold(&device, &sigma_threshold_mm);
    ESP_LOGI(TAG,
        "Signal threshold: %u kcps | Sigma threshold: %u mm",
        signal_threshold_kcps,
        sigma_threshold_mm);

    // Avvia le misurazioni
    vl53l4cd_start_ranging(&device);

    // Attende il primo flag data-ready
    uint8_t data_ready = 0;
    while (!data_ready)
    {
        // Verifica quando sono pronti nuovi dati
        vl53l4cd_check_for_data_ready(&device, &data_ready);

        // Delay temporale in millisecondi
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // Lettura del risultato
    vl53l4cd_result_t result;
    vl53l4cd_get_result(&device, &result);

    ESP_LOGI(TAG,
        "Result: Distance: %u mm | Status: %u | Signal: %u kcps/SPAD | "
        "Ambient: %u kcps/SPAD | Sigma: %u mm | SPAD: %u",
        result.distance_mm,
        result.range_status,
        result.signal_per_spad_kcps,
        result.ambient_per_spad_kcps,
        result.sigma_mm,
        result.number_of_spad);

    // Elimina l'interrupt pendente e termina il ranging
    vl53l4cd_clear_interrupt(&device);

    // Termina le misurazioni
    vl53l4cd_stop_ranging(&device);

    // Rimuove il dispositivo VL53L4CD dal bus I2C
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(device.i2c_device_handle));
    ESP_LOGI(TAG, "VL53L4CD device removed successfully");

    // Libera il bus I2C
    ESP_ERROR_CHECK(i2c_del_master_bus(i2c_bus_handle));
    ESP_LOGI(TAG, "I2C bus successfully freed");
}
