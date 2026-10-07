#include "aircraft_layer.h"
#include "../config.h"
#include "../model/flight.h"
#include <lvgl.h>
#include <math.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <freertos/semphr.h>

extern void set_route_request(const char *icao24, const char *callsign);

extern FlightData g_flights[MAX_FLIGHTS];
extern size_t g_flight_count;
extern SemaphoreHandle_t g_flights_mutex;

namespace {

constexpr int CENTER_X = 120;
constexpr int CENTER_Y = 120;
constexpr int OUTER_RADIUS = 110;
constexpr int MAX_AIRCRAFT_OBJECTS = 32;

struct AircraftObject {
    lv_obj_t *line_l;
    lv_obj_t *line_r;
    lv_obj_t *line_b;
    lv_point_t pts[3];
    lv_obj_t *label;
    char label_text[16];
    char icao24[9];
    char callsign[9];
    char type_code[9];
    char kind[9];
    char departure[5];
    char arrival[5];
    char callsign_iata[9];
    float altitude_m;
    float speed_mps;
    float heading_deg;
    int16_t screen_x;
    int16_t screen_y;
    uint32_t last_update_time_ms;
    uint32_t first_seen_time_ms;
};

static AircraftObject s_aircraft[MAX_AIRCRAFT_OBJECTS];
static int s_aircraft_count = 0;
static int s_selected_slot = -1;
static uint32_t s_selected_at_ms = 0;
static bool s_selected_waiting_route = false;
static bool s_selected_route_na = false;

static lv_obj_t *s_detail_panel = nullptr;
static lv_obj_t *s_detail_title = nullptr;
static lv_obj_t *s_detail_iata = nullptr;
static lv_obj_t *s_detail_line1 = nullptr;
static lv_obj_t *s_detail_line2 = nullptr;
static char s_detail_title_cache[24] = {0};
static char s_detail_iata_cache[9] = {0};
static char s_detail_line1_cache[48] = {0};
static char s_detail_line2_cache[64] = {0};

constexpr uint32_t DETAIL_AUTO_CLOSE_MS = 7000;
constexpr uint32_t DETAIL_ROUTE_WAIT_TIMEOUT_MS = 15000;
constexpr int TOUCH_SELECT_RADIUS_PX = 24;

inline float deg_to_rad(float deg) {
    return deg * 0.01745329252f;
}

static float distance_haversine(float lat1, float lon1, float lat2, float lon2) {
    const float R = 6371.0f;
    const float dLat = (lat2 - lat1) * 0.01745329252f;
    const float dLon = (lon2 - lon1) * 0.01745329252f;
    const float a = sinf(dLat * 0.5f) * sinf(dLat * 0.5f) +
                    cosf(lat1 * 0.01745329252f) * cosf(lat2 * 0.01745329252f) *
                    sinf(dLon * 0.5f) * sinf(dLon * 0.5f);
    const float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
    return R * c;
}

static lv_point_t polar_to_screen(float angle_deg, int radius) {
    float plot_deg = angle_deg - 90.0f;
    float rad = deg_to_rad(plot_deg);

    lv_point_t p;
    p.x = (lv_coord_t)(CENTER_X + (int)lroundf(cosf(rad) * radius));
    p.y = (lv_coord_t)(CENTER_Y + (int)lroundf(sinf(rad) * radius));
    return p;
}

static lv_point_t flight_to_screen(const FlightData &f) {
    const float lat_off = f.lat - g_home_lat;
    const float lon_off = f.lon - g_home_lon;

    const float lat_km = lat_off * 111.0f;
    const float lon_km = lon_off * 111.0f * cosf(g_home_lat * 0.01745329252f);

    const float r_km = sqrtf(lat_km * lat_km + lon_km * lon_km);
    const float r_px = (r_km / RADAR_RADIUS_KM) * OUTER_RADIUS;

    float angle_deg = atan2f(lon_km, lat_km) * 57.29577951f;
    if (angle_deg < 0.0f) {
        angle_deg += 360.0f;
    }

    return polar_to_screen(angle_deg, (int)r_px);
}

static int find_aircraft_slot(const char *icao24) {
    for (int i = 0; i < s_aircraft_count; ++i) {
        if (strncmp(s_aircraft[i].icao24, icao24, sizeof(s_aircraft[i].icao24)) == 0) {
            return i;
        }
    }
    return -1;
}

static bool obj_valid(lv_obj_t *obj) {
    return (obj != nullptr) && lv_obj_is_valid(obj);
}

static void delete_aircraft_graphics(AircraftObject &a) {
    if (obj_valid(a.line_l)) lv_obj_del(a.line_l);
    if (obj_valid(a.line_r)) lv_obj_del(a.line_r);
    if (obj_valid(a.line_b)) lv_obj_del(a.line_b);
    if (obj_valid(a.label))  lv_obj_del(a.label);

    a.line_l = nullptr;
    a.line_r = nullptr;
    a.line_b = nullptr;
    a.label  = nullptr;
}

static void copy_trimmed(char *dst, size_t dst_size, const char *src) {
    if (dst == nullptr || dst_size == 0) return;
    if (src == nullptr) { dst[0] = '\0'; return; }
    strncpy(dst, src, dst_size - 1);
    dst[dst_size - 1] = '\0';

    size_t start = 0;
    while (dst[start] == ' ' || dst[start] == '\n' || dst[start] == '\r' || dst[start] == '\t') ++start;
    if (start > 0) memmove(dst, dst + start, strlen(dst + start) + 1);

    size_t len = strlen(dst);
    while (len > 0) {
        const char c = dst[len - 1];
        if (c == ' ' || c == '\n' || c == '\r' || c == '\t') {
            dst[len - 1] = '\0';
            --len;
        } else break;
    }
}

static void set_label_if_changed(lv_obj_t *label, char *cache, size_t cache_size, const char *text) {
    if (!obj_valid(label) || cache == nullptr || cache_size == 0 || text == nullptr) return;
    if (strncmp(cache, text, cache_size) == 0) return;
    strncpy(cache, text, cache_size - 1);
    cache[cache_size - 1] = '\0';
    lv_label_set_text(label, cache);
}

static void detail_panel_hide() {
    if (s_detail_panel != nullptr) lv_obj_add_flag(s_detail_panel, LV_OBJ_FLAG_HIDDEN);
    s_selected_slot = -1;
    s_selected_waiting_route = false;
    s_selected_route_na = false;
}

static void detail_panel_show_for_slot(int slot, uint32_t now_ms) {
    if (slot < 0 || slot >= s_aircraft_count || s_detail_panel == nullptr) return;

    const AircraftObject &a = s_aircraft[slot];

    char title_buf[24];
    if (a.callsign[0] != '\0') snprintf(title_buf, sizeof(title_buf), "%s", a.callsign);
    else snprintf(title_buf, sizeof(title_buf), "%s", a.icao24);

    char line1[48] = {0};
    const bool has_route = (a.departure[0] != '\0' && strncmp(a.departure, "----", 4) != 0
                         && a.arrival[0] != '\0' && strncmp(a.arrival, "----", 4) != 0);
    if (has_route) snprintf(line1, sizeof(line1), "DEP %s  ARR %s", a.departure, a.arrival);
    else snprintf(line1, sizeof(line1), "Retrieving...");

    char line2[64];
    snprintf(line2, sizeof(line2), "ALT %.0fm  SPD %.0fm/s", a.altitude_m, a.speed_mps);

    set_label_if_changed(s_detail_title, s_detail_title_cache, sizeof(s_detail_title_cache), title_buf);
    set_label_if_changed(s_detail_iata,  s_detail_iata_cache,  sizeof(s_detail_iata_cache),  a.callsign_iata[0] ? a.callsign_iata : "");
    set_label_if_changed(s_detail_line1, s_detail_line1_cache, sizeof(s_detail_line1_cache), line1);
    set_label_if_changed(s_detail_line2, s_detail_line2_cache, sizeof(s_detail_line2_cache), line2);
    lv_obj_clear_flag(s_detail_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(s_detail_panel);

    s_selected_slot = slot;
    s_selected_waiting_route = !has_route;
    s_selected_route_na = false;
    s_selected_at_ms = now_ms;
}

static int find_nearest_aircraft_slot(int16_t x, int16_t y, int max_radius_px) {
    int best_slot = -1;
    int best_d2 = max_radius_px * max_radius_px;

    for (int i = 0; i < s_aircraft_count; ++i) {
        const int dx = (int)s_aircraft[i].screen_x - (int)x;
        const int dy = (int)s_aircraft[i].screen_y - (int)y;
        const int d2 = dx * dx + dy * dy;
        if (d2 <= best_d2) {
            best_d2 = d2;
            best_slot = i;
        }
    }
    return best_slot;
}

} // namespace

void aircraft_layer_init() {
    for (int i = 0; i < MAX_AIRCRAFT_OBJECTS; ++i) {
        s_aircraft[i].line_l = nullptr;
        s_aircraft[i].line_r = nullptr;
        s_aircraft[i].line_b = nullptr;
        s_aircraft[i].label  = nullptr;
        s_aircraft[i].label_text[0] = '\0';
        s_aircraft[i].icao24[0] = '\0';
        s_aircraft[i].callsign[0] = '\0';
        s_aircraft[i].type_code[0] = '\0';
        s_aircraft[i].kind[0] = '\0';
        s_aircraft[i].departure[0] = '\0';
        s_aircraft[i].arrival[0] = '\0';
        s_aircraft[i].callsign_iata[0] = '\0';
        s_aircraft[i].altitude_m = 0.0f;
        s_aircraft[i].speed_mps = 0.0f;
        s_aircraft[i].heading_deg = 0.0f;
        s_aircraft[i].screen_x = -1000;
        s_aircraft[i].screen_y = -1000;
    }
    s_aircraft_count = 0;
    s_selected_slot = -1;

    lv_obj_t *scr = lv_scr_act();
    s_detail_panel = lv_obj_create(scr);
    lv_obj_set_size(s_detail_panel, 180, 54);
    lv_obj_align(s_detail_panel, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_radius(s_detail_panel, 6, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_detail_panel, lv_color_hex(0x001A0A), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_detail_panel, LV_OPA_80, LV_PART_MAIN);
    lv_obj_set_style_border_color(s_detail_panel, lv_color_hex(0x00AA44), LV_PART_MAIN);
    lv_obj_set_style_border_opa(s_detail_panel, LV_OPA_90, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_detail_panel, 1, LV_PART_MAIN);
    lv_obj_set_style_pad_all(s_detail_panel, 4, LV_PART_MAIN);
    lv_obj_clear_flag(s_detail_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_detail_panel, LV_OBJ_FLAG_HIDDEN);

    s_detail_title = lv_label_create(s_detail_panel);
    lv_obj_set_style_text_color(s_detail_title, lv_color_hex(0x00FF66), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_detail_title, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(s_detail_title, LV_ALIGN_TOP_LEFT, 0, 0);

    s_detail_iata = lv_label_create(s_detail_panel);
    lv_label_set_text(s_detail_iata, "");
    lv_obj_set_style_text_color(s_detail_iata, lv_color_hex(0x00FF66), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_detail_iata, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(s_detail_iata, LV_ALIGN_TOP_RIGHT, 0, 0);

    s_detail_line1 = lv_label_create(s_detail_panel);
    lv_obj_set_style_text_color(s_detail_line1, lv_color_hex(0x00AA44), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_detail_line1, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_align_to(s_detail_line1, s_detail_title, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 1);

    s_detail_line2 = lv_label_create(s_detail_panel);
    lv_obj_set_style_text_color(s_detail_line2, lv_color_hex(0x00AA44), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_detail_line2, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_align_to(s_detail_line2, s_detail_line1, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);
}

void aircraft_layer_handle_tap(int16_t x, int16_t y) {
    const uint32_t now_ms = lv_tick_get();
    int slot = find_nearest_aircraft_slot(x, y, TOUCH_SELECT_RADIUS_PX);
    if (slot < 0) {
        detail_panel_hide();
        return;
    }
    detail_panel_show_for_slot(slot, now_ms);
    if (s_selected_waiting_route) {
        set_route_request(s_aircraft[slot].icao24, s_aircraft[slot].callsign);
    }
}

void aircraft_layer_update() {
    if (xSemaphoreTake(g_flights_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }

    const uint32_t now_ms = lv_tick_get();
    const lv_color_t plane_color = lv_color_hex(0x00FF66);
    const lv_color_t label_color = lv_color_hex(0x00AA44);

    int visible_count = 0;
    for (size_t i = 0; i < g_flight_count && visible_count < MAX_AIRCRAFT_OBJECTS; ++i) {
        const FlightData &f = g_flights[i];

        const float dist_km = distance_haversine(g_home_lat, g_home_lon, f.lat, f.lon);
        if (dist_km > RADAR_RADIUS_KM) continue;

        int slot = find_aircraft_slot(f.icao24);
        bool is_new = (slot < 0);

        if (is_new) {
            slot = s_aircraft_count;
            if (s_aircraft_count >= MAX_AIRCRAFT_OBJECTS) {
                xSemaphoreGive(g_flights_mutex);
                return;
            }
            s_aircraft_count++;

            lv_obj_t *scr = lv_scr_act();
            s_aircraft[slot].line_l = lv_line_create(scr);
            s_aircraft[slot].line_r = lv_line_create(scr);
            s_aircraft[slot].line_b = lv_line_create(scr);

            lv_obj_set_style_line_color(s_aircraft[slot].line_l, plane_color, LV_PART_MAIN);
            lv_obj_set_style_line_width(s_aircraft[slot].line_l, 2, LV_PART_MAIN);

            lv_obj_set_style_line_color(s_aircraft[slot].line_r, plane_color, LV_PART_MAIN);
            lv_obj_set_style_line_width(s_aircraft[slot].line_r, 2, LV_PART_MAIN);

            lv_obj_set_style_line_color(s_aircraft[slot].line_b, plane_color, LV_PART_MAIN);
            lv_obj_set_style_line_width(s_aircraft[slot].line_b, 1, LV_PART_MAIN);

            s_aircraft[slot].label = lv_label_create(scr);
            lv_obj_set_style_text_color(s_aircraft[slot].label, label_color, LV_PART_MAIN);
            lv_obj_set_style_text_font(s_aircraft[slot].label, &lv_font_montserrat_12, LV_PART_MAIN);
            s_aircraft[slot].label_text[0] = '\0';

            strncpy(s_aircraft[slot].icao24, f.icao24, sizeof(s_aircraft[slot].icao24) - 1);
            s_aircraft[slot].icao24[sizeof(s_aircraft[slot].icao24) - 1] = '\0';
            s_aircraft[slot].first_seen_time_ms = now_ms;
        }

        copy_trimmed(s_aircraft[slot].callsign, sizeof(s_aircraft[slot].callsign), f.callsign);
        copy_trimmed(s_aircraft[slot].type_code, sizeof(s_aircraft[slot].type_code), f.type_code);
        copy_trimmed(s_aircraft[slot].kind, sizeof(s_aircraft[slot].kind), f.kind);
        copy_trimmed(s_aircraft[slot].departure, sizeof(s_aircraft[slot].departure), f.departure);
        copy_trimmed(s_aircraft[slot].arrival, sizeof(s_aircraft[slot].arrival), f.arrival);
        copy_trimmed(s_aircraft[slot].callsign_iata, sizeof(s_aircraft[slot].callsign_iata), f.callsign_iata);

        if (s_aircraft[slot].departure[0] == '\0') copy_trimmed(s_aircraft[slot].departure, sizeof(s_aircraft[slot].departure), "----");
        if (s_aircraft[slot].arrival[0] == '\0') copy_trimmed(s_aircraft[slot].arrival, sizeof(s_aircraft[slot].arrival), "----");

        s_aircraft[slot].altitude_m = f.altitude_m;
        s_aircraft[slot].speed_mps = f.speed_mps;
        s_aircraft[slot].heading_deg = f.heading_deg;

        lv_point_t pos = flight_to_screen(f);
        s_aircraft[slot].screen_x = (int16_t)pos.x;
        s_aircraft[slot].screen_y = (int16_t)pos.y;

        // Desenha a silhueta em forma de avião (triângulo orientado na direção do voo)
        const float rad = deg_to_rad(f.heading_deg - 90.0f);
        const float size = 10.0f;

        // Vértice frontal (bico)
        s_aircraft[slot].pts[0].x = pos.x + (int16_t)lroundf(cosf(rad) * size);
        s_aircraft[slot].pts[0].y = pos.y + (int16_t)lroundf(sinf(rad) * size);

        // Asa esquerda
        s_aircraft[slot].pts[1].x = pos.x + (int16_t)lroundf(cosf(rad + 2.4f) * (size * 0.7f));
        s_aircraft[slot].pts[1].y = pos.y + (int16_t)lroundf(sinf(rad + 2.4f) * (size * 0.7f));

        // Asa direita
        s_aircraft[slot].pts[2].x = pos.x + (int16_t)lroundf(cosf(rad - 2.4f) * (size * 0.7f));
        s_aircraft[slot].pts[2].y = pos.y + (int16_t)lroundf(sinf(rad - 2.4f) * (size * 0.7f));

        // Atualiza os vetores de linha do avião no LVGL
        static lv_point_t line_l_pts[2];
        static lv_point_t line_r_pts[2];
        static lv_point_t line_b_pts[2];

        line_l_pts[0] = s_aircraft[slot].pts[0]; line_l_pts[1] = s_aircraft[slot].pts[1];
        line_r_pts[0] = s_aircraft[slot].pts[0]; line_r_pts[1] = s_aircraft[slot].pts[2];
        line_b_pts[0] = s_aircraft[slot].pts[1]; line_b_pts[1] = s_aircraft[slot].pts[2];

        if (obj_valid(s_aircraft[slot].line_l)) lv_line_set_points(s_aircraft[slot].line_l, line_l_pts, 2);
        if (obj_valid(s_aircraft[slot].line_r)) lv_line_set_points(s_aircraft[slot].line_r, line_r_pts, 2);
        if (obj_valid(s_aircraft[slot].line_b)) lv_line_set_points(s_aircraft[slot].line_b, line_b_pts, 2);

        char label_buf[16];
        const char *callsign = f.callsign;
        if (callsign == nullptr || callsign[0] == '\0') callsign = f.icao24;
        snprintf(label_buf, sizeof(label_buf), "%.6s", callsign);

        set_label_if_changed(s_aircraft[slot].label, s_aircraft[slot].label_text, sizeof(s_aircraft[slot].label_text), label_buf);
        if (obj_valid(s_aircraft[slot].label)) {
            lv_obj_set_pos(s_aircraft[slot].label, pos.x + 8, pos.y - 6);
        }

        s_aircraft[slot].last_update_time_ms = now_ms;
        visible_count++;
    }

    int slot_idx = 0;
    while (slot_idx < s_aircraft_count) {
        const uint32_t age_ms = now_ms - s_aircraft[slot_idx].last_update_time_ms;
        if (age_ms > 60000) {
            delete_aircraft_graphics(s_aircraft[slot_idx]);
            if (slot_idx == s_selected_slot) detail_panel_hide();

            if (slot_idx < s_aircraft_count - 1) {
                memmove(&s_aircraft[slot_idx], &s_aircraft[slot_idx + 1],
                        (s_aircraft_count - slot_idx - 1) * sizeof(s_aircraft[0]));
                if (s_selected_slot > slot_idx) s_selected_slot--;
            }
            s_aircraft_count--;
        } else {
            slot_idx++;
        }
    }

    if (s_selected_slot >= 0 && !s_selected_waiting_route && (now_ms - s_selected_at_ms) > DETAIL_AUTO_CLOSE_MS) {
        detail_panel_hide();
    } else if (s_selected_slot >= 0) {
        const AircraftObject &a = s_aircraft[s_selected_slot];
        char line1[48] = {0};
        const bool has_route = (a.departure[0] != '\0' && strncmp(a.departure, "----", 4) != 0
                             && a.arrival[0] != '\0' && strncmp(a.arrival, "----", 4) != 0);
        if (has_route) {
            snprintf(line1, sizeof(line1), "DEP %s  ARR %s", a.departure, a.arrival);
            if (s_selected_waiting_route) {
                s_selected_waiting_route = false;
                s_selected_route_na = false;
                s_selected_at_ms = now_ms;
            }
        } else if (s_selected_waiting_route) {
            const uint32_t wait_ms = now_ms - s_selected_at_ms;
            if (wait_ms >= DETAIL_ROUTE_WAIT_TIMEOUT_MS) {
                s_selected_waiting_route = false;
                s_selected_route_na = true;
                s_selected_at_ms = now_ms;
                snprintf(line1, sizeof(line1), "N/A");
            } else {
                snprintf(line1, sizeof(line1), "Retrieving...");
            }
        } else if (s_selected_route_na) {
            snprintf(line1, sizeof(line1), "N/A");
        }
        set_label_if_changed(s_detail_line1, s_detail_line1_cache, sizeof(s_detail_line1_cache), line1);
        set_label_if_changed(s_detail_iata,  s_detail_iata_cache,  sizeof(s_detail_iata_cache),  a.callsign_iata[0] ? a.callsign_iata : "");

        char line2[64];
        snprintf(line2, sizeof(line2), "ALT %.0fm  SPD %.0fm/s", a.altitude_m, a.speed_mps);
        set_label_if_changed(s_detail_line2, s_detail_line2_cache, sizeof(s_detail_line2_cache), line2);
    }

    xSemaphoreGive(g_flights_mutex);
}
