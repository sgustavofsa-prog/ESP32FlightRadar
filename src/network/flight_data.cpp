#include "flight_data.h"
#include "../config.h"

#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <math.h>
#include <string.h>

namespace {

static void safe_copy(char *dst, size_t dst_size, const char *src) {
    if (dst_size == 0 || dst == nullptr) return;
    if (src == nullptr) {
        dst[0] = '\0';
        return;
    }
    strncpy(dst, src, dst_size - 1);
    dst[dst_size - 1] = '\0';
}

// Extrai valores do array JSON do Flightradar24 diretamente do texto
// Formato: "hex":[icao, lat, lon, track, alt, speed, ..., callsign]
static bool parse_fr24_line(const String &payload, size_t &pos, FlightData &f) {
    int start_arr = payload.indexOf('[', pos);
    if (start_arr < 0) return false;

    int end_arr = payload.indexOf(']', start_arr);
    if (end_arr < 0) return false;

    pos = end_arr + 1; // Avança o cursor para o próximo elemento

    String arr_str = payload.substring(start_arr + 1, end_arr);
    
    // Divide os elementos por vírgula
    int idx = 0;
    int from = 0;
    String tokens[15];
    
    while (from < arr_str.length() && idx < 15) {
        int comma = arr_str.indexOf(',', from);
        if (comma < 0) {
            tokens[idx++] = arr_str.substring(from);
            break;
        }
        tokens[idx++] = arr_str.substring(from, comma);
        from = comma + 1;
    }

    if (idx < 14) return false;

    // Limpa aspas do ICAO (tokens[0])
    String hex = tokens[0];
    hex.replace("\"", "");
    safe_copy(f.icao24, sizeof(f.icao24), hex.c_str());

    f.lat = tokens[1].toFloat();
    f.lon = tokens[2].toFloat();
    f.heading_deg = tokens[3].toFloat();
    f.altitude_m = tokens[4].toFloat() * 0.3048f; // pés -> metros
    f.speed_mps = tokens[5].toFloat() * 0.51444f;  // knots -> m/s

    // Limpa aspas do Callsign (tokens[13])
    String cs = tokens[13];
    cs.replace("\"", "");
    safe_copy(f.callsign, sizeof(f.callsign), cs.c_str());

    return (f.lat != 0.0f && f.lon != 0.0f);
}

} // namespace

bool flight_data_fetch_flights(FlightData *out_flights, size_t out_capacity, size_t *out_count) {
    if (out_flights == nullptr || out_count == nullptr || out_capacity == 0) {
        return false;
    }

    *out_count = 0;

    // Cálculo da Bounding Box (~25km ou valor em RADAR_RADIUS_KM ao redor das coordenadas globais ativas)
    const float lat_delta = RADAR_RADIUS_KM / 111.0f;
    const float lon_delta = RADAR_RADIUS_KM / (111.0f * cosf(g_home_lat * 0.01745329252f));

    const float lat_max = g_home_lat + lat_delta;
    const float lat_min = g_home_lat - lat_delta;
    const float lon_min = g_home_lon - lon_delta;
    const float lon_max = g_home_lon + lon_delta;

    char url[256];
    snprintf(url, sizeof(url),
             "https://data-cloud.flightradar24.com/zones/fcgi/feed.json?bounds=%.4f,%.4f,%.4f,%.4f&faa=1&mlat=1&flarm=1&adsb=1&gnd=0&air=1",
             lat_max, lat_min, lon_min, lon_max);

    WiFiClientSecure client;
    client.setInsecure();

    HTTPClient http;
    Serial.printf("[fr24] GET %s\n", url);

    if (!http.begin(client, url)) {
        Serial.println("[fr24] HTTP begin failed");
        return false;
    }

    http.setUserAgent("Mozilla/5.0 (Windows NT 10.0; Win64; x64)");
    http.addHeader("Referer", "https://www.flightradar24.com/");
    http.setTimeout(8000);

    const int code = http.GET();
    if (code != HTTP_CODE_OK) {
        Serial.printf("[fr24] HTTP code: %d\n", code);
        http.end();
        client.stop();
        return false;
    }

    String payload = http.getString();
    http.end();
    client.stop();

    if (payload.length() < 20) {
        Serial.println("[fr24] Empty response");
        return true; // Sucesso, porém sem aeronaves na área
    }

    size_t n = 0;
    size_t pos = 0;

    // Varre o payload do FR24 procurando os vetores de aviões
    while (n < out_capacity) {
        FlightData f{};
        if (!parse_fr24_line(payload, pos, f)) {
            break; // Fim do payload ou sem mais aviões
        }
        out_flights[n++] = f;
    }

    *out_count = n;
    Serial.printf("[fr24] Parsed %u flights\n", (unsigned)n);
    return true;
}

bool flight_data_fetch_route(const char *icao24, const char *callsign,
                             char *dep, size_t dep_size,
                             char *arr, size_t arr_size,
                             char *callsign_iata, size_t callsign_iata_size) {
    if (dep != nullptr && dep_size > 0) dep[0] = '\0';
    if (arr != nullptr && arr_size > 0) arr[0] = '\0';
    if (callsign_iata != nullptr && callsign_iata_size > 0) callsign_iata[0] = '\0';
    return false;
}
