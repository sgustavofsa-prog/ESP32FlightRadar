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

static lv_obj_t *s_aircraft_canvas = nullptr;
static lv_color_t *s_canvas_buf = nullptr;

constexpr int16_t CANVAS_WIDTH = 240;
constexpr int16_t CANVAS_HEIGHT = 240;
constexpr float CENTER_X = 120.0f;
constexpr float CENTER_Y = 120.0f;

// Converte latitude/longitude para coordenadas X, Y no ecrã (240x240)
static void latlon_to_screen(float lat, float lon, int16_t *out_x, int16_t *out_y) {
    const float lat_diff = lat - g_home_lat;
    const float lon_diff = lon - g_home_lon;

    const float dy_km = lat_diff * 111.0f;
    const float dx_km = lon_diff * (111.0f * cosf(g_home_lat * 0.01745329252f));

    const float px_per_km = (CANVAS_WIDTH / 2.0f) / RADAR_RADIUS_KM;

    *out_x = (int16_t)roundf(CENTER_X + (dx_km * px_per_km));
    *out_y = (int16_t)roundf(CENTER_Y - (dy_km * px_per_km)); // Inverte Y no ecrã
}

// Desenha o símbolo do avião no canvas conforme a orientação (heading) e dimensão
static void draw_aircraft_icon(int16_t x, int16_t y, float heading_deg, int size, lv_color_t color) {
    if (x < 5 || x >= CANVAS_WIDTH - 5 || y < 5 || y >= CANVAS_HEIGHT - 5) {
        return; // Fora dos limites visíveis do ecrã redondo
    }

    const float rad = (heading_deg - 90.0f) * 0.01745329252f;
    const float cos_a = cosf(rad);
    const float sin_a = sinf(rad);

    // Desenha o triângulo/vetor apontado na direção da aeronave
    int16_t x1 = x + (int16_t)(size * cos_a);
    int16_t y1 = y + (int16_t)(size * sin_a);

    int16_t x2 = x + (int16_t)((size / 2.0f) * cosf(rad + 2.5f));
    int16_t y2 = y + (int16_t)((size / 2.0f) * sinf(rad + 2.5f));

    int16_t x3 = x + (int16_t)((size / 2.0f) * cosf(rad - 2.5f));
    int16_t y3 = y + (int16_t)((size / 2.0f) * sinf(rad - 2.5f));

    lv_point_t poly[3] = {
        {x1, y1},
        {x2, y2},
        {x3, y3}
    };

    lv_draw_line_dsc_t line_dsc;
    lv_draw_line_dsc_init(&line_dsc);
    lv_draw_line_dsc_t *dsc = &line_dsc;
    dsc->color = color;
    dsc->width = (size > 10) ? 2 : 1;

    // Linhas de contorno da aeronave
    lv_canvas_draw_line(s_aircraft_canvas, poly, 2, dsc);
    poly[0] = poly[1]; poly[1] = poly[2];
    lv_canvas_draw_line(s_aircraft_canvas, poly, 2, dsc);
    poly[0] = poly[2]; poly[1] = {x1, y1};
    lv_canvas_draw_line(s_aircraft_canvas, poly, 2, dsc);
}

void aircraft_layer_init() {
    if (s_aircraft_canvas != nullptr) return;

    s_canvas_buf = (lv_color_t *)malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(CANVAS_WIDTH, CANVAS_HEIGHT));
    if (!s_canvas_buf) {
        Serial.println("[aircraft] Falha ao alocar buffer do canvas");
        return;
    }

    s_aircraft_canvas = lv_canvas_create(lv_scr_act());
    lv_canvas_set_buffer(s_aircraft_canvas, s_canvas_buf, CANVAS_WIDTH, CANVAS_HEIGHT, LV_IMG_CF_TRUE_COLOR);
    lv_obj_center(s_aircraft_canvas);
}

void aircraft_layer_update() {
    if (s_aircraft_canvas == nullptr) return;

    // Limpa o canvas para o fundo transparente/preto
    lv_canvas_fill_bg(s_aircraft_canvas, lv_color_black(), LV_OPA_TRANSP);

    if (g_flights_mutex == nullptr) return;

    if (xSemaphoreTake(g_flights_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (size_t i = 0; i < g_flight_count; ++i) {
            const FlightData &f = g_flights[i];

            int16_t x = 0, y = 0;
            latlon_to_screen(f.lat, f.lon, &x, &y);

            // DIFERENCIAÇÃO DE PORTE:
            // Aviões grandes / comerciais: velocidade > 120 m/s (~233 nós) ou altitude > 3000 metros
            const bool is_large = (f.speed_mps > 120.0f || f.altitude_m > 3000.0f);

            if (is_large) {
                // AVIÃO GRANDE: Ícone de 14px em Amarelo/Laranja Brilhante
                draw_aircraft_icon(x, y, f.heading_deg, 14, lv_color_hex(0xFFB000));
            } else {
                // AVIÃO PEQUENO / LEVE: Ícone de 8px em Verde/Ciano
                draw_aircraft_icon(x, y, f.heading_deg, 8, lv_color_hex(0x00FFCD));
            }

            // Exibe o Callsign reduzido logo abaixo do ponto do avião
            if (f.callsign[0] != '\0') {
                lv_draw_label_dsc_t label_dsc;
                lv_draw_label_dsc_init(&label_dsc);
                label_dsc.color = is_large ? lv_color_hex(0xFFFFFF) : lv_color_hex(0x00FFCD);
                label_dsc.font = LV_FONT_DEFAULT; // Usa a fonte ativa padrão do projeto
                lv_canvas_draw_text(s_aircraft_canvas, x - 15, y + 6, 40, &label_dsc, f.callsign);
            }
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

            if (dist_sq < min_dist_sq && dist_sq < 900.0f) { // Raio de toque de ~30px
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
