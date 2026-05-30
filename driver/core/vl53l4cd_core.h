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

#ifndef VL53L4CD_CORE_H
#define VL53L4CD_CORE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct vl53l4cd_t vl53l4cd_t;

    typedef enum {
        VL53L4CD_ERR_OK = 0,
        VL53L4CD_ERR_INVALID_ARG,
        VL53L4CD_ERR_FAIL,
        VL53L4CD_ERR_TIMEOUT,
    } vl53l4cd_err_t;

    typedef enum {
        VL53L4CD_LOG_ERROR = 0,
        VL53L4CD_LOG_WARNING,
        VL53L4CD_LOG_INFO,
        VL53L4CD_LOG_DEBUG,
        VL53L4CD_LOG_VERBOSE,
    } vl53l4cd_log_t;

    /**
     * @brief Il seguente codice definisce un modello (template) di firma per le seguenti funzioni.
     * L'implementazione di queste funzioni spetta allo sviluppatore in base al tipo di platform.
     */
    typedef vl53l4cd_err_t (*vl53l4cd_read_register_t)(
        void *handle,
        const uint16_t reg,
        const size_t length,
        uint8_t *data
    );
    typedef vl53l4cd_err_t (*vl53l4cd_write_register_t)(
        void *handle,
        const uint16_t reg,
        const size_t length,
        const uint8_t *data
    );
    typedef vl53l4cd_err_t (*vl53l4cd_update_address_t)(void *handle, const uint8_t address);
    typedef vl53l4cd_err_t (*vl53l4cd_gpio_set_level_t)(const uint8_t pin, const uint8_t level);
    typedef void (*vl53l4cd_time_delay_t)(const uint32_t ms);
    typedef void (*vl53l4cd_isr_handler_t)(vl53l4cd_t *device, void *context);

    /**
     * @brief La struttura contiene le funzioni per comunicare con l'hardware.
     * Suddividendo la comunicazione hardware dai dati specifici per ogni sensore,
     * è possibile risparmiare memoria RAM nel caso in cui si debbano utilizzare
     * più di un sensore.
     */
    typedef struct {
        vl53l4cd_read_register_t i2c_read;
        vl53l4cd_write_register_t i2c_write;
        vl53l4cd_update_address_t update_address;
        vl53l4cd_gpio_set_level_t xshut_set_level;
        vl53l4cd_time_delay_t delay_ms;
    } vl53l4cd_platform_t;

    /**
     * @brief La struttura contiene i dati relativi ad ogni sensore.
     */
    struct vl53l4cd_t {
        void *i2c_bus_handle;                // Handler del bus I2C
        void *i2c_device_handle;             // Handler dispositivo I2C
        uint8_t i2c_address;                 // Indirizzo I2C del dispositivo
        uint32_t i2c_scl_speed;              // Velocità clock I2C
        uint16_t i2c_timeout;                // Timeout I2C del dispositivo
        int8_t xshut_pin;                    // Pin per l'alimentazione
        int8_t int_pin;                      // Pin di interrupt
        void *context;                       // Puntatore generico a servizio dell'applicazione
        vl53l4cd_isr_handler_t isr_handler;  // Funzione chiamata all'arrivo di un interrupt
        const vl53l4cd_platform_t *platform; // Puntatore alla struttura platform
    };

    /**
     * @brief La struttura contiene i risulati delle letture del sensore.
     */
    typedef struct {
        uint8_t range_status;           // Stato della misurazione (0 = dati validi)
        uint8_t number_of_spad;         // Numero degli SPAD attivati (la parte LSB è esclusa)
        uint16_t distance_mm;           // Distanza misurata in mm
        uint16_t ambient_rate_kcps;     // Rumore ambientale in kcps
        uint16_t ambient_per_spad_kcps; // Rumore ambientale in kcps/SPAD
        uint16_t signal_rate_kcps;      // Segnale del target misurato in kcps
        uint16_t signal_per_spad_kcps;  // Segnale del target misurato in kcps/SPAD
        uint16_t sigma_mm;              // Misura stimata della deviazione standard in mm
    } vl53l4cd_result_t;

    /**
     * @brief Inizializza il core del dispositivo VL53L4CD.
     *
     * @param[in] device                Dispositivo VL53L4CD.
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_init_core(vl53l4cd_t *device);

    /**
     * @brief Dà alimentazione al dispositivo tramite il pin power. Dopo questa chiamata è
     * necessario inizializzare nuovamente il dispositivo nel caso in cui fosse spento
     * precedentemente.
     *
     * @param[in] device                Dispositivo VL53L4CD.
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_power_on(vl53l4cd_t *device);

    /**
     * @brief Toglie alimentazione al dispositivo tramite il pin power.
     *
     * @param[in] device                Dispositivo VL53L4CD.
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_power_off(vl53l4cd_t *device);

    /**
     * @brief Riavvia il dispositivo tramite un power reset. Dopo la chiamata di
     * questa funzione è necessario inizializzare nuovamente il dispositivo.
     *
     * @param[in] device                Dispositivo VL53L4CD.
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_power_reset(vl53l4cd_t *device);

    /**
     * @brief Ottiene lo stato di boot del dispositivo.
     *
     * @param[in]  device               Dispositivo VL53L4CD.
     * @param[out] state                Stato di boot (3: successo, 0: altrimenti).
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_get_boot_state(vl53l4cd_t *device, uint8_t *state);

    /**
     * @brief Imposta un nuovo indirizzo I2C (7 bit) per il dispositivo. In questo modo è possibile
     * utilizzare più di un sensore sullo stesso bus I2C.
     *
     * @param[in] device                Dispositivo VL53L4CD.
     * @param[in] address               Indirizzo I2C del dispositivo.
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_set_i2c_address(vl53l4cd_t *device, const uint8_t address);

    /**
     * @brief Ottiene l'ID del dispositivo. Per il VL53L4CD è 0xEBAA.
     *
     * @param[in]  device               Dispositivo VL53L4CD.
     * @param[out] value                ID del sensore.
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_get_device_id(vl53l4cd_t *device, uint16_t *value);

    /**
     * @brief Avvia il processo di misurazione continua della distanza.
     *
     * @param[in] device                Dispositivo VL53L4CD.
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_start_ranging(vl53l4cd_t *device);

    /**
     * @brief Termina il processo di misurazione della distanza.
     *
     * @param[in] device                Dispositivo VL53L4CD.
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_stop_ranging(vl53l4cd_t *device);

    /**
     * @brief Ottiene l'intervallo di temporizzazione, il quale è composto da:
     * - TimingBudget, rappresenta la temporizzazione durante l'attivazione di VCSEL;
     * - InterMeasurement, rappresenta l'intervallo tra due misurazioni.
     * Il dispositivo ha diverse modalità di intervallo, consultare il manuale utente.
     *
     * @param[in]  device               Dispositivo VL53L4CD.
     * @param[out] timing_budget_ms     Valore del timing-budget in ms.
     * @param[out] inter_measurement_ms Valore del inter-measurement in ms.
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_get_range_timing(
        vl53l4cd_t *device,
        uint32_t *timing_budget_ms,
        uint32_t *inter_measurement_ms
    );

    /**
     * @brief Imposta l'intervallo di temporizzazione, il quale è composto da:
     * - TimingBudget, rappresenta la temporizzazione durante l'attivazione di VCSEL;
     * - InterMeasurement, rappresenta l'intervallo tra due misurazioni.
     * Il dispositivo ha diverse modalità di intervallo, consultare il manuale utente.
     *
     * @param[in] device                Dispositivo VL53L4CD.
     * @param[in] timing_budget_ms      Imposta il timing-budget in ms (10, 200), 50: default.
     * @param[in] inter_measurement_ms  Imposta l'inter-measurement in ms. Se zero, il periodo di
     * intervallo è definito dal timing-budget. Altrimenti inter-measurement deve essere > del
     * timing-budget. Dopo che il timing-budget è trascorso, il dispositivo va in modalità risparmio
     * energetico finché inter-measurement non è trascorso.
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_set_range_timing(
        vl53l4cd_t *device,
        const uint32_t timing_budget_ms,
        const uint32_t inter_measurement_ms
    );

    /**
     * @brief Ottiene i dati risultanti dalla misurazione.
     *
     * @param[in]  device               Dispositivo VL53L4CD.
     * @param[out] result               Struttura contenente i dati della misurazione.
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_get_result(vl53l4cd_t *device, vl53l4cd_result_t *result);

    /**
     * @brief Ottiene l'offset di correzione della distanza in mm. Indica la differenza in mm tra
     * la distanza reale e quella misurata.
     *
     * @param[in]  device               Dispositivo VL53L4CD.
     * @param[out] offset_mm            Valore di offset in mm (-1024, 1023).
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_get_offset(vl53l4cd_t *device, int16_t *offset_mm);

    /**
     * @brief Imposta l'offset di correzione della distanza in mm. Indica la differenza in mm tra
     * la distanza reale e quella misurata.
     *
     * @param[in] device                Dispositivo VL53L4CD.
     * @param[in] offset_mm             Valore di offset in mm (-1024, 1023).
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_set_offset(vl53l4cd_t *device, const int16_t offset_mm);

    /**
     * @brief Ottiene il valore Xtalk in kcps. Rappresenta la correzione da applicare al sensore
     * quando una protezione in vetro è posta sopra il sensore.
     *
     * @param[in]  device               Dispositivo VL53L4CD.
     * @param[out] xtalk_kcps           Valore di Xtalk in kcps.
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_get_xtalk(vl53l4cd_t *device, uint16_t *xtalk_kcps);

    /**
     * @brief Imposta il valore Xtalk in kcps. Rappresenta la correzione da applicare al sensore
     * quando una protezione in vetro è posta sopra il sensore.
     *
     * @param[in] device                Dispositivo VL53L4CD.
     * @param[in] xtalk_kcps            Imposta l'Xtalk in kcps (0, 128), 0: default.
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_set_xtalk(vl53l4cd_t *device, uint16_t xtalk_kcps);

    /**
     * @brief Ottiene la soglia inferiore e superiore di rilevamento. Possono generare un interrupt
     * sul PIN 7 (GPIO1) al raggiungimento una condizione.
     *
     * @param[in]  device               Dispositivo VL53L4CD.
     * @param[out] low_dist_mm          Valore della soglia inferiore in mm.
     * @param[out] high_dist_mm         Valore della soglia superiore in mm.
     * @param[out] window               Metodo di interrupt (0=sotto la soglia inferiore;
     * 1=sopra la soglia superiore; 2=fuori dall'intervallo; 3=dento l'intervallo).
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_get_detection_thresholds(
        vl53l4cd_t *device,
        uint16_t *low_dist_mm,
        uint16_t *high_dist_mm,
        uint8_t *window
    );

    /**
     * @brief Imposta la soglia inferiore e superiore di rilevamento. Possono generare un interrupt
     * sul PIN 7 (GPIO1) al raggiungimento una condizione.
     *
     * @param[in] device                Dispositivo VL53L4CD.
     * @param[in] low_dist_mm           Valore della soglia inferiore in mm.
     * @param[in] high_dist_mm          Valore della soglia superiore in mm.
     * @param[in] window                Metodo di interrupt (0=sotto la soglia inferiore;
     * 1=sopra la soglia superiore; 2=fuori dall'intervallo; 3=dento l'intervallo).
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_set_detection_thresholds(
        vl53l4cd_t *device,
        const uint16_t low_dist_mm,
        const uint16_t high_dist_mm,
        const uint8_t window
    );

    /**
     * @brief Ottiene la soglia del segnale in kcps. Se il target ha un segnale inferiore alla
     * soglia impostata, il campo status del risultato avrà valore 2.
     *
     * @param[in]  device               Dispositivo VL53L4CD.
     * @param[out] signal_kcps          Valore del segnale in kcps.
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_get_signal_threshold(vl53l4cd_t *device, uint16_t *signal_kcps);

    /**
     * @brief Imposta la soglia del segnale in kcps. Se il target ha un segnale inferiore alla
     * soglia impostata, il campo status del risultato avrà valore 2.
     *
     * @param[in] device                Dispositivo VL53L4CD.
     * @param[in] signal_kcps           Imposta la soglia del segnale in kcps (0, 16384), 1024:
     * default.
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_set_signal_threshold(vl53l4cd_t *device, uint16_t signal_kcps);

    /**
     * @brief Ottiene la soglia sigma in mm. Corrisponde alla deviazione standard dell'impulso
     * restituito. Se il valore supera la soglia, il campo status del risultato avrà valore 1.
     *
     * @param[in]  device               Dispositivo VL53L4CD.
     * @param[out] sigma_mm             Valore della soglia sigma in mm (0, 16383), 15: default.
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_get_sigma_threshold(vl53l4cd_t *device, uint16_t *sigma_mm);

    /**
     * @brief Imposta la soglia sigma in mm. Corrisponde alla deviazione standard dell'impulso
     * restituito. Se il valore supera la soglia, il campo status del risultato avrà valore 1.
     *
     * @param[in] device                Dispositivo VL53L4CD.
     * @param[in] sigma_mm              Valore della soglia sigma in mm (0, 16383), 15: default.
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_set_sigma_threshold(vl53l4cd_t *device, uint16_t sigma_mm);

    /**
     * @brief Ottiene la polarità del pin di interrupt DRDY.
     *
     * @param[in]  device               Dispositivo VL53L4CD.
     * @param[out] polarity             Valore della polarità (0: active-low, 1: active-high).
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_get_interrupt_polarity(vl53l4cd_t *device, uint8_t *polarity);

    /**
     * @brief Imposta la polarità del pin di interrupt DRDY.
     *
     * @param[in] device                Dispositivo VL53L4CD.
     * @param[in] polarity              Imposta la polarità (0: active-low, 1: active-high).
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_set_interrupt_polarity(vl53l4cd_t *device, const uint8_t polarity);

    /**
     * @brief Verifica la disponibilità di nuove misurazioni interrogando il registro.
     *
     * @param[in]  device               Dispositivo VL53L4CD.
     * @param[out] value                Flag dati pronti (1: pronti, 0: altrimenti).
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_check_for_data_ready(vl53l4cd_t *device, uint8_t *value);

    /**
     * @brief Elimina l'interrupt pendente.
     *
     * @param[in] device                Dispositivo VL53L4CD.
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_clear_interrupt(vl53l4cd_t *device);

    /**
     * @brief Configura la funzione chiamata al verificarsi di un interrupt hardware.
     *
     * @param[in] device                Dispositivo VL53L4CD.
     * @param[in] handler               Funzione invocata dalla ISR.
     * @param[in] context               Contesto passato alla callback ISR. NULL se inutilizzato.
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_set_isr_handler(
        vl53l4cd_t *device,
        vl53l4cd_isr_handler_t handler,
        void *context
    );

    /**
     * @brief Esegue una calibrazione dell'offset. Il quale corrisponde alla differenza in
     * millimetri tra la distanza reale e quella misurata. Si consiglia di eseguire la procedura su
     * un target grigio rifrettente al 17% lontano 100 mm dal dispositivo. Il valore di offset
     * restituito viene programmato all'interno del dispositivo.
     *
     * @param[in]  device               Dispositivo VL53L4CD.
     * @param[in]  target_dist_mm       Valore della distanza reale in mm del target.
     * @param[in]  nb_samples           Numero dei campioni (5, 255), 10: default.
     * @param[out] measured_offset_mm   Valore dell'offset ottenuto dalla calibrazione.
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_calibrate_offset(
        vl53l4cd_t *device,
        const int16_t target_dist_mm,
        const uint8_t nb_samples,
        int16_t *p_measured_offset_mm
    );

    /**
     * @brief Esegue una calibrazione Xtalk. Il quale rappresenta la correzione da applicare al
     * sensore quando una protezione in vetro è posta sopra il sensore. Il valore di Xtalk
     * restituito viene programmato all'interno del dispositivo.
     *
     * @param[in]  device               Dispositivo VL53L4CD.
     * @param[in]  target_dist_mm       Valore della distanza reale in mm del target.
     * @param[in]  nb_samples           Numero dei campioni (5, 255), 10: default.
     * @param[out] measured_xtalk_kcps  Valore dell'Xtalk ottenuto dalla calibrazione.
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_calibrate_xtalk(
        vl53l4cd_t *device,
        const int16_t target_dist_mm,
        const uint8_t nb_samples,
        uint16_t *measured_xtalk_kcps
    );

    /**
     * @brief Esegue una calibrazione del dispositivo. Va fatta quando la temperatura cambia di
     * almeno 8 gradi Celsisus. Per chiamare questa funzione il dispositivo non deve eseguire alcuna
     * misurazione.
     *
     * @param[in] device                Dispositivo VL53L4CD.
     * @retval VL53L4CD_ERR_OK          Successo.
     * @retval VL53L4CD_ERR_INVALID_ARG Parametri non validi.
     */
    vl53l4cd_err_t vl53l4cd_start_temperature_update(vl53l4cd_t *device);

#ifdef __cplusplus
}
#endif

#endif // VL53L4CD_CORE_H
