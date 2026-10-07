#include "aircraft_layer.h"
#include "../config.h"
#include "../model/flight.h"
#include "ui.h"

#include <Arduino.h>
#include <lvgl.h>
#include <math.h>

extern FlightData g_flights[MAX_FLIGHTS];
extern size_t g_flight_count;
extern SemaphoreHandle_t g_flights_mutex;

constexpr int16_t CANVAS_WIDTH = 240;
constexpr int16_t CANVAS_HEIGHT = 240;
constexpr float CENTER_X = 120.0f;
constexpr float CENTER_Y = 120.0f;

// Container principal para as aeronaves
static lv_obj_t *s_aircraft_layer = nullptr;

// Pool de objetos nativos do LVGL para redesenhar os aviões sem estourar a RAM
static lv_obj_t *s_flight_dots[MAX_FLIGHTS] = {nullptr};
static lv_obj_t *s_flight_labels[MAX_FLIGHTS] = {nullptr};

// Converte latitude/longitude para coordenadas X, Y no ecrã (240x240)
static void latlon_to_screen(float lat, float lon, int16_t *out_x, int16_t *out_y) {
    const float lat_diff = lat - g_home_lat;
    const float lon_diff = lon - g_home_lon;

    const float dy_km = lat_diff * 111.0f;
    const float dx_km = lon_diff * (111.0f * cosf(g_home_lat * 0.01745329252f));

    const float px_per_km = (CANVAS_WIDTH / 2.0f) / RADAR_RADIUS_KM;

    *out_x = (int16_t)roundf(CENTER_X + (dx_km * px_per_km));
    *out_y = (int16_t)roundf(CENTER_Y - (dy_km * px_per_km));
}

void aircraft_layer_init() {
    if (s_aircraft_layer != nullptr) return;

    // Cria um container transparente em cima do radar
    s_aircraft_layer = lv_obj_create(lv_scr_act());
    lv_obj_set_size(s_aircraft_layer, CANVAS_WIDTH, CANVAS_HEIGHT);
    lv_obj_center(s_aircraft_layer);
    lv_obj_set_style_bg_opa(s_aircraft_layer, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_aircraft_layer, 0, 0);
    lv_obj_clear_flag(s_aircraft_layer, LV_OBJ_FLAG_SCROLLABLE);

    // Inicializa a pool de pontos e labels leves
    for (size_t i = 0; i < MAX_FLIGHTS; ++i) {
        s_flight_dots[i] = lv_obj_create(s_aircraft_layer);
        lv_obj_set_size(s_flight_dots[i], 8, 8);
        lv_obj_set_style_radius(s_flight_dots[i], LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(s_flight_dots[i], 0, 0);
        lv_obj_add_flag(s_flight_dots[i], LV_OBJ_FLAG_HIDDEN);

        s_flight_labels[i] = lv_label_create(s_aircraft_layer);
        lv_obj_add_flag(s_flight_labels[i], LV_OBJ_FLAG_HIDDEN);
    }

    Serial.println("[aircraft] Camada de avioes ultra-leve inicializada com sucesso!");
}

void aircraft_layer_update() {
    if (s_aircraft_layer == nullptr || g_flights_mutex == nullptr) return;

    if (xSemaphoreTake(g_flights_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (size_t i = 0; i < MAX_FLIGHTS; ++i) {
            if (i < g_flight_count) {
                const FlightData &f = g_flights[i];

                int16_t x = 0, y = 0;
                latlon_to_screen(f.lat, f.lon, &x, &y);

                // Verifica se está dentro dos limites da tela redonda
                if (x >= 10 && x <= CANVAS_WIDTH - 10 && y >= 10 && y <= CANVAS_HEIGHT - 10) {
                    // DIFERENCIAÇÃO DE PORTE
                    const bool is_large = (f.speed_mps > 120.0f || f.altitude_m > 3000.0f);

                    // Ajusta tamanho e cor do ponto
                    int size = is_large ? 12 : 7;
                    lv_color_t color = is_large ? lv_color_hex(0xFFB000) : lv_color_hex(0x00FFCD);

                    lv_obj_set_size(s_flight_dots[i], size, size);
                    lv_obj_set_style_bg_color(s_flight_dots[i], color, 0);
                    lv_obj_set_style_bg_opa(s_flight_dots[i], LV_OPA_COVER, 0);
                    lv_obj_align(s_flight_dots[i], LV_ALIGN_TOP_LEFT, x - (size / 2), y - (size / 2));
                    lv_obj_clear_flag(s_flight_dots[i], LV_OBJ_FLAG_HIDDEN);

                    // Ajusta o Callsign do avião
                    if (f.callsign[0] != '\0') {
                        lv_label_set_text(s_flight_labels[i], f.callsign);
                        lv_obj_set_style_text_color(s_flight_labels[i], color, 0);
                        lv_obj_align(s_flight_labels[i], LV_ALIGN_TOP_LEFT, x - 15, y + (size / 2) + 2);
                        lv_obj_clear_flag(s_flight_labels[i], LV_OBJ_FLAG_HIDDEN);
                    } else {
                        lv_obj_add_flag(s_flight_labels[i], LV_OBJ_FLAG_HIDDEN);
                    }
                    continue;
                }
            }

            // Oculta pontos não utilizados ou fora da tela
            lv_obj_add_flag(s_flight_dots[i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(s_flight_labels[i], LV_OBJ_FLAG_HIDDEN);
        }
        xSemaphoreGive(g_flights_mutex);
    }
}

void aircraft_layer_handle_tap(int16_t touch_x, int16_t touch_y) {
    if (g_flights_mutex == nullptr) return;

    if (xSemaphoreTake(g_flights_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        float min_dist_sq = 999999.0f;
        int best_idx = -1;

        for (size_t i = 0; i < g_flight_count; ++i) {
            int16_t ax = 0, ay = 0;
            latlon_to_screen(g_flights[i].lat, g_flights[i].lon, &ax, &ay);

            float dx = (float)(touch_x - ax);
            float dy = (float)(touch_y - ay);
            float dist_sq = (dx * dx) + (dy * dy);

            if (dist_sq < min_dist_sq && dist_sq < 900.0f) {
                min_dist_sq = dist_sq;
                best_idx = (int)i;
            }
        }

        if (best_idx >= 0) {
            const FlightData &f = g_flights[best_idx];
            Serial.printf("[tap] Selecionado: %s (Alt: %.0fm, Vel: %.0fkm/h)\n",
                          f.callsign, f.altitude_m, f.speed_mps * 3.6f);
        }

        xSemaphoreGive(g_flights_mutex);
    }
}
