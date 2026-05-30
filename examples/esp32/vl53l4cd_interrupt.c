/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2026 Reptaide.
 *
 * Redistributed under the terms of the BSD 3-Clause License.
 * See LICENSE in the root of this repository for the full license text.
 */

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vl53l4cd_core.h"
#include "vl53l4cd_platform.h"

#define I2C_DEFAULT_ADDRESS 0x29 // Indirizzo I2C di default all'avvio
#define I2C_PORT I2C_NUM_0       // Numero porta I2C
#define I2C_PIN_SCL 10           // SCL
#define I2C_PIN_SDA 11           // SDA
#define I2C_SCL_SPEED_HZ 100000  // 100 kHz
#define I2C_TIMEOUT 200          // Timeout I2C in ms
#define INT_PIN 13               // Interrupt
#define XSHUT_PIN 14             // Alimentazione

// Definisce il tag per il debugging
static const char *TAG = "main";
static TaskHandle_t vl53l4cd_task_handle = NULL;

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

/**
 * @brief Task di elaborazione asincrono fuori dal contesto di interrupt che legge i risultati del
 * dispositivo che ha generato l'interrupt.
 *
 * @param[in] arg Argomento passato al task.
 */
void vl53l4cd_task(void *arg)
{
    vl53l4cd_err_t status = VL53L4CD_ERR_OK;

    vl53l4cd_t *device = (vl53l4cd_t *)arg;
    vl53l4cd_result_t result;

    for (;;)
    {
        // Attende la notifica dall'ISR
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Ottiene i risultati della misurazione
        status = vl53l4cd_get_result(device, &result);

        if (status == VL53L4CD_ERR_OK && result.range_status == 0)
            ESP_LOGI(
                TAG,
                "[%p] Distance: %6u, Signal: %6u",
                device,
                result.distance_mm,
                result.signal_per_spad_kcps
            );
        else
            ESP_LOGD(TAG, "[%p] Status: %u", device, result.range_status);

        // Elimina l'interrupt pendente
        vl53l4cd_clear_interrupt(device);
    }
}

/**
 * @brief ISR callback applicativa utilizzata per notificare un task FreeRTOS. Viene
 * richiamata dal driver al verificarsi di un interrupt hardware del sensore VL53L4CD.
 *
 * @param[in] device   Dispositivo VL53L4CD che ha generato l'interrupt.
 * @param[in] context  Contesto utente contenente il TaskHandle_t del task.
 */
static void IRAM_ATTR vl53l4cd_isr_handler(vl53l4cd_t *device, void *context)
{
    BaseType_t high_task_wakeup = pdFALSE;

    TaskHandle_t task_handle = (TaskHandle_t)context;

    vTaskNotifyGiveFromISR(task_handle, &high_task_wakeup);

    portYIELD_FROM_ISR(high_task_wakeup);
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

    // Installa nella IRAM il servizio di interrupt GPIO globale
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));

    // Crea il task per gestire l'evento del sensore
    xTaskCreate(vl53l4cd_task, "vl53l4cd_task", 4096, &device, 5, &vl53l4cd_task_handle);

    // Registra la ISR callback e associa il task da notificare all'interrupt
    vl53l4cd_set_isr_handler(&device, vl53l4cd_isr_handler, (void *)vl53l4cd_task_handle);

    // Configura il pin di interrupt
    vl53l4cd_hal_setup_int(&device, INT_PIN);

    // Avvia le misurazioni
    vl53l4cd_start_ranging(&device);

    for (;;)
        vTaskDelay(pdMS_TO_TICKS(1000));
}
