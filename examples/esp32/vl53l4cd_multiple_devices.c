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
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "vl53l4cd_core.h"
#include "vl53l4cd_platform.h"

#define I2C_DEFAULT_ADDRESS 0x29 // Indirizzo I2C di default all'avvio
#define I2C_PORT I2C_NUM_0       // Numero porta I2C
#define I2C_PIN_SCL 10           // SCL
#define I2C_PIN_SDA 11           // SDA
#define I2C_SCL_SPEED_HZ 100000  // 100 kHz
#define I2C_TIMEOUT 200          // Timeout I2C in ms
#define VL53L4CD_NUM_SENSOR 2    // Numero dei dispositivi presenti

// Definisce il tag per il debugging
static const char *TAG = "main";

// Coda condivisa per i dati dei sensori
static QueueHandle_t vl53l4cd_sensor_queue = NULL;

// Definisce gli indirizzi I2C dei dispositivi
static const uint8_t I2C_ADDRESSES[VL53L4CD_NUM_SENSOR] = {0x29, 0x30};

// Definisce i pin di alimentazione dei dispositivi
static const uint8_t XSHUT_PINS[VL53L4CD_NUM_SENSOR] = {14, 12};

// Definisce i pin di interrupt dei dispositivi
static const uint8_t INT_PINS[VL53L4CD_NUM_SENSOR] = {13, 25};

// Array dei dispositivi
static vl53l4cd_t devices[VL53L4CD_NUM_SENSOR];

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

    vl53l4cd_t *device = NULL;
    vl53l4cd_result_t result;

    for (;;)
    {
        // Attende che l'ISR inserisca un dispositivo nella coda
        if (xQueueReceive(vl53l4cd_sensor_queue, &device, portMAX_DELAY) != pdTRUE)
            continue;

        // Ottiene i risultati della misurazione
        status = vl53l4cd_get_result(device, &result);

        if (status == VL53L4CD_ERR_OK && result.range_status == 0)
            ESP_LOGI(TAG,
                "[%p] Distance: %6u, Signal: %6u",
                device,
                result.distance_mm,
                result.signal_per_spad_kcps);
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

    // Recupera l'handle della coda dal contesto
    QueueHandle_t queue = (QueueHandle_t)context;

    // Inserisce il puntatore al dispositivo nella coda.
    if (queue)
        xQueueSendFromISR(queue, &device, &high_task_wakeup);

    portYIELD_FROM_ISR(high_task_wakeup);
}

/**
 * @brief Inizializza N dispositivi VL53L4CD sullo stesso bus I2C, assegnando un
 * indirizzo univoco a ciascuno tramite XSHUT, e avviando le misurazioni via ISR.
 *
 * @param[in] i2c_bus_handle Handle del bus I2C.
 */
void vl53l4cd_init_multiple(i2c_master_bus_handle_t i2c_bus_handle)
{
    vl53l4cd_err_t status = VL53L4CD_ERR_OK;

    // Itera i dispositivi presenti
    for (size_t i = 0; i < VL53L4CD_NUM_SENSOR; i++)
    {
        // Inizializza a zero il dispositivo
        devices[i] = (vl53l4cd_t){0};

        // Inizializza il platform per comunicare con l'hardware
        vl53l4cd_init_hal(
            &devices[i], i2c_bus_handle, I2C_DEFAULT_ADDRESS, I2C_SCL_SPEED_HZ, I2C_TIMEOUT);

        // Configura il pin di alimentazione
        vl53l4cd_hal_setup_xshut(&devices[i], XSHUT_PINS[i]);

        // Spegne il dispositivo
        vl53l4cd_power_off(&devices[i]);
    }

    // Itera i dispositivi presenti
    for (size_t i = 0; i < VL53L4CD_NUM_SENSOR; i++)
    {
        // Accende il dispositivo
        vl53l4cd_power_on(&devices[i]);

        // Inizializza il core del dispositivo
        status = vl53l4cd_init_core(&devices[i]);

        if (status != VL53L4CD_ERR_OK)
        {
            ESP_LOGE(TAG, "Device %u: init_core failed", i);
            continue;
        }

        // Ottiene l'ID del dispositivo
        uint16_t sensor_id = 0;
        vl53l4cd_get_device_id(&devices[i], &sensor_id);
        ESP_LOGI(TAG, "Device %u: [ID=0x%04X]", i, sensor_id);

        // Cambia l'indirizzo I2C se necessario (volatile: si perde al power-cycle)
        if (I2C_ADDRESSES[i] != I2C_DEFAULT_ADDRESS)
        {
            status = vl53l4cd_set_i2c_address(&devices[i], I2C_ADDRESSES[i]);

            if (status != VL53L4CD_ERR_OK)
            {
                ESP_LOGE(TAG, "Device %u: I2C address change failed", i, I2C_ADDRESSES[i]);
                continue;
            }

            ESP_LOGI(TAG, "Device %u: New I2C address 0x%02X", i, I2C_ADDRESSES[i]);
        }

        // Registra la ISR callback e associa il task da notificare all'interrupt
        vl53l4cd_set_isr_handler(&devices[i], vl53l4cd_isr_handler, (void *)vl53l4cd_sensor_queue);

        // Configura il pin di interrupt
        vl53l4cd_hal_setup_int(&devices[i], INT_PINS[i]);

        // Avvio del ranging (start_ranging include già il clear_interrupt)
        vl53l4cd_start_ranging(&devices[i]);

        ESP_LOGI(TAG,
            "Device %u: started at 0x%02X (INT_PIN %u)",
            i,
            devices[i].i2c_address,
            INT_PINS[i]);
    }
}

void app_main(void)
{
    // Handler del bus I2C
    i2c_master_bus_handle_t i2c_bus_handle = NULL;

    // Inizializza il bus I2C
    i2c_bus_init(&i2c_bus_handle);

    // Installa nella IRAM il servizio di interrupt GPIO globale
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));

    // Crea la coda per accogliere gli interrupt
    vl53l4cd_sensor_queue = xQueueCreate(VL53L4CD_NUM_SENSOR * 4, sizeof(vl53l4cd_t *));

    // Crea il task per gestire l'evento del sensore
    xTaskCreate(vl53l4cd_task, "vl53l4cd_task", 4096, NULL, 5, NULL);

    // Inizializza tutti i dispositivi
    vl53l4cd_init_multiple(i2c_bus_handle);

    for (;;)
        vTaskDelay(pdMS_TO_TICKS(1000));
}
