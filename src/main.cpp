#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include "config.h"
#include "display/display.h"
#include "display/ui.h"
#include "display/aircraft_layer.h"
#include "display/map_background.h"
#include "touch/cst816s.h"
#include "model/flight.h"
#include "network/wifi_manager.h"
#include "network/ntp.h"
#include "network/flight_data.h"

// Pinos físicos de botão comuns em placas ESP32-C3 / ESP32-2424S012
#define BTN_BOOT_C3 9
#define BTN_BOOT_ALT 0

// Definition of active location variables declared in config.h
float g_home_lat = RADAR_LOCATIONS[0].lat;
float g_home_lon = RADAR_LOCATIONS[0].lon;
const char* g_location_name = RADAR_LOCATIONS[0].name;
static size_t s_current_loc_index = 0;
static volatile bool s_force_map_refetch = false;

FlightData g_flights[MAX_FLIGHTS];
size_t g_flight_count = 0;
SemaphoreHandle_t g_flights_mutex = nullptr;
static TaskHandle_t s_display_task_handle = nullptr;
static TaskHandle_t s_network_task_handle = nullptr;

static const char *reset_reason_to_str(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_UNKNOWN:   return "UNKNOWN";
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_EXT:       return "EXT";
        case ESP_RST_SW:        return "SW";
        case ESP_RST_PANIC:     return "PANIC";
        case ESP_RST_INT_WDT:   return "INT_WDT";
        case ESP_RST_TASK_WDT:  return "TASK_WDT";
        case ESP_RST_WDT:       return "WDT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "OTHER";
    }
}

// Function to cycle locations (Santos -> Congonhas -> Guarulhos -> Viracopos)
static void switch_next_location() {
    s_current_loc_index = (s_current_loc_index + 1) % RADAR_LOCATIONS_COUNT;
    g_home_lat = RADAR_LOCATIONS[s_current_loc_index].lat;
    g_home_lon = RADAR_LOCATIONS[s_current_loc_index].lon;
    g_location_name = RADAR_LOCATIONS[s_current_loc_index].name;

    Serial.printf("[location] Switched to %s (%.4f, %.4f)\n", g_location_name, g_home_lat, g_home_lon);
    ui_set_network_status(g_location_name);
    s_force_map_refetch = true;
}

// ---- On-demand route request ----
static portMUX_TYPE s_route_req_mux = portMUX_INITIALIZER_UNLOCKED;
static char s_route_req_icao24[9]    = {0};
static char s_route_req_callsign[9]  = {0};
static volatile bool s_route_req_pending = false;

static void process_route_request_once() {
    if (!s_route_req_pending) {
        return;
    }

    char req_icao24[9]   = {0};
    char req_callsign[9] = {0};
    taskENTER_CRITICAL(&s_route_req_mux);
    strncpy(req_icao24,   s_route_req_icao24,   sizeof(req_icao24)   - 1);
    strncpy(req_callsign, s_route_req_callsign, sizeof(req_callsign) - 1);
    s_route_req_pending = false;
    taskEXIT_CRITICAL(&s_route_req_mux);

    char dep[5] = {0};
    char arr[5] = {0};
    char cs_iata[9] = {0};
    const bool route_ok = flight_data_fetch_route(req_icao24, req_callsign, dep, sizeof(dep), arr, sizeof(arr),
                                                  cs_iata, sizeof(cs_iata));

    if (dep[0] != '\0' && arr[0] != '\0') {
        if (xSemaphoreTake(g_flights_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            for (size_t i = 0; i < g_flight_count; ++i) {
                if (strncmp(g_flights[i].icao24, req_icao24, 8) == 0) {
                    strncpy(g_flights[i].departure, dep, sizeof(g_flights[i].departure) - 1);
                    g_flights[i].departure[sizeof(g_flights[i].departure) - 1] = '\0';
                    strncpy(g_flights[i].arrival, arr, sizeof(g_flights[i].arrival) - 1);
                    g_flights[i].arrival[sizeof(g_flights[i].arrival) - 1] = '\0';
                    strncpy(g_flights[i].callsign_iata, cs_iata, sizeof(g_flights[i].callsign_iata) - 1);
                    g_flights[i].callsign_iata[sizeof(g_flights[i].callsign_iata) - 1] = '\0';
                    break;
                }
            }
            xSemaphoreGive(g_flights_mutex);
        }
    }
}

void set_route_request(const char *icao24, const char *callsign) {
    taskENTER_CRITICAL(&s_route_req_mux);
    strncpy(s_route_req_icao24,   icao24,   sizeof(s_route_req_icao24)   - 1);
    strncpy(s_route_req_callsign, callsign, sizeof(s_route_req_callsign) - 1);
    s_route_req_icao24[sizeof(s_route_req_icao24)   - 1] = '\0';
    s_route_req_callsign[sizeof(s_route_req_callsign) - 1] = '\0';
    s_route_req_pending = true;
    taskEXIT_CRITICAL(&s_route_req_mux);
}

// ============================================================
// Display task — Botões Físicos para Troca + Touch para Aeronaves
// ============================================================
static void display_task(void * /*pvParameters*/) {
    display_init();
    ui_init();
    aircraft_layer_init();
    const bool touch_ok = cst816s_init();

    // Configura os pinos dos botões físicos com Pull-Up interno
    pinMode(BTN_BOOT_C3, INPUT_PULLUP);
    pinMode(BTN_BOOT_ALT, INPUT_PULLUP);

    Serial.println("[display] LVGL running");
    Serial.printf("[display] touch=%s | Buttons GPIO 9/0 ready\n", touch_ok ? "ok" : "offline");

    uint32_t last_btn_ms = 0;
    bool last_btn_state1 = HIGH;
    bool last_btn_state2 = HIGH;

    for (;;) {
        map_background_try_install();

        // 1. Leitura dos Botões Físicos (GPIO 9 e GPIO 0)
        bool btn1 = digitalRead(BTN_BOOT_C3);
        bool btn2 = digitalRead(BTN_BOOT_ALT);
        const uint32_t now = millis();

        if ((btn1 == LOW && last_btn_state1 == HIGH) || (btn2 == LOW && last_btn_state2 == HIGH)) {
            if (now - last_btn_ms > 400) {
                last_btn_ms = now;
                Serial.printf("[button] Pressionado (GPIO9=%d, GPIO0=%d) -> Mudando cidade!\n", btn1, btn2);
                switch_next_location();
            }
        }
        last_btn_state1 = btn1;
        last_btn_state2 = btn2;

        // 2. Toque na tela -> Seleciona a aeronave para ver detalhes
        int16_t touch_x = 0, touch_y = 0;
        if (cst816s_poll_tap(&touch_x, &touch_y)) {
            Serial.printf("[touch] Toque na tela em (%d, %d) -> Selecionando aeronave\n", touch_x, touch_y);
            aircraft_layer_handle_tap(touch_x, touch_y);
        }

        lv_timer_handler();           // Processa renderização e menus LVGL
        aircraft_layer_update();      // Atualiza aviões no radar
        vTaskDelay(pdMS_TO_TICKS(20)); // ~50 fps
    }
}

// ============================================================
// Network task — WiFi + NTP + ADS-B polling
// ============================================================
static void network_task(void * /*pvParameters*/) {
    static FlightData tmp_flights[MAX_FLIGHTS];
    static bool map_fetched_this_boot = false;

    for (;;) {
        if (!wifi_is_connected()) {
            ui_set_network_status("CONNECTING...");
            if (!wifi_connect_blocking()) {
                ui_set_network_status("WIFI RETRY");
                vTaskDelay(pdMS_TO_TICKS(3000));
                continue;
            }
            ui_set_network_status("WIFI ONLINE");
        }

        if (!map_fetched_this_boot || s_force_map_refetch) {
            ui_set_network_status("FETCH MAP");
            if (map_background_fetch_once()) {
                map_fetched_this_boot = true;
                s_force_map_refetch = false;
                Serial.printf("[map] Mapa carregado para %s\n", g_location_name);
            } else {
                Serial.println("[map] Falha no mapa, mantendo radar ativo");
                s_force_map_refetch = false;
            }
        }

        if (!ntp_is_synced()) {
            ui_set_network_status("SYNC TIME...");
            if (!ntp_sync_blocking(10000)) {
                ui_set_network_status("NTP RETRY");
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
        }

        process_route_request_once();

        size_t fetched_count = 0;
        ui_set_network_status("UPDATING...");
        bool ok = flight_data_fetch_flights(tmp_flights, MAX_FLIGHTS, &fetched_count);
        
        if (ok) {
            if (xSemaphoreTake(g_flights_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                g_flight_count = fetched_count;
                for (size_t i = 0; i < fetched_count; ++i) {
                    g_flights[i] = tmp_flights[i];
                }
                xSemaphoreGive(g_flights_mutex);
            }
            ui_set_network_status(g_location_name);
        } else {
            ui_set_network_status("API RETRY");
        }

        process_route_request_once();

        uint32_t slept_ms = 0;
        while (slept_ms < OPENSKY_FETCH_PERIOD_MS) {
            if (s_force_map_refetch) {
                break; // Interrompe a espera se o botão for pressionado
            }
            process_route_request_once();
            constexpr uint32_t SLICE_MS = 200;
            vTaskDelay(pdMS_TO_TICKS(SLICE_MS));
            slept_ms += SLICE_MS;
        }
    }
}

// ============================================================
// Arduino entry points
// ============================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(200);

    Serial.println("[boot] ESP32 Radar — M2");
    const esp_reset_reason_t reason = esp_reset_reason();
    Serial.printf("[boot] reset_reason=%s (%d)\n", reset_reason_to_str(reason), (int)reason);
    Serial.printf("[boot] Free heap: %lu bytes\n", (unsigned long)esp_get_free_heap_size());

    g_flights_mutex = xSemaphoreCreateMutex();
    if (g_flights_mutex == nullptr) {
        Serial.println("[boot] ERROR: failed to create flights mutex");
    }

    BaseType_t rc_display = xTaskCreatePinnedToCore(
        display_task,
        "display",
        TASK_DISPLAY_STACK,
        nullptr,
        TASK_DISPLAY_PRIO,
        &s_display_task_handle,
        TASK_DISPLAY_CORE
    );

    if (rc_display != pdPASS) {
        Serial.printf("[boot] ERROR: display task create failed (rc=%ld)\n", (long)rc_display);
    }

    BaseType_t rc_network = xTaskCreatePinnedToCore(
        network_task,
        "network",
        TASK_NETWORK_STACK,
        nullptr,
        TASK_NETWORK_PRIO,
        &s_network_task_handle,
        TASK_NETWORK_CORE
    );

    if (rc_network != pdPASS) {
        Serial.printf("[boot] ERROR: network task create failed (rc=%ld)\n", (long)rc_network);
    }
}

void loop() {
    static uint32_t last_report = 0;
    if (millis() - last_report >= 10000) {
        last_report = millis();
        const uint32_t free_heap = (uint32_t)esp_get_free_heap_size();
        const uint32_t min_heap = (uint32_t)esp_get_minimum_free_heap_size();
        const uint32_t largest_block = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
        const UBaseType_t display_hwm = (s_display_task_handle != nullptr) ? uxTaskGetStackHighWaterMark(s_display_task_handle) : 0;
        const UBaseType_t network_hwm = (s_network_task_handle != nullptr) ? uxTaskGetStackHighWaterMark(s_network_task_handle) : 0;

        Serial.printf("[diag] heap_free=%lu min_ever=%lu largest_block=%lu stack_hwm_display_words=%lu stack_hwm_network_words=%lu\n",
                      (unsigned long)free_heap,
                      (unsigned long)min_heap,
                      (unsigned long)largest_block,
                      (unsigned long)display_hwm,
                      (unsigned long)network_hwm);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
}
