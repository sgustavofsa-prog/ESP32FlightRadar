#include "flight_data.h"
#include "../config.h"

#include <Arduino.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
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

} // namespace

bool flight_data_fetch_flights(FlightData *out_flights, size_t out_capacity, size_t *out_count) {
    if (out_flights == nullptr || out_count == nullptr || out_capacity == 0) {
        return false;
    }

    *out_count = 0;

    // Converte o raio de km em delta de graus de latitude/longitude (~50km)
    const float lat_delta = RADAR_RADIUS_KM / 111.0f;
    const float lon_delta = RADAR_RADIUS_KM / (111.0f * cosf(g_home_lat * 0.01745329252f));

    const float lat_max = g_home_lat + lat_delta;
    const float lat_min = g_home_lat - lat_delta;
    const float lon_min = g_home_lon - lon_delta;
    const float lon_max = g_home_lon + lon_delta;

    // URL leve do Flightradar24 sobre HTTP simples (sem overhead de TLS/SSL)
    char url[256];
    snprintf(url, sizeof(url),
             "http://data-cloud.flightradar24.com/zones/fcgi/feed.json?bounds=%.4f,%.4f,%.4f,%.4f&faa=1&mlat=1&flarm=1&adsb=1&gnd=0&air=1",
             lat_max, lat_min, lon_min, lon_max);

    HTTPClient http;
    Serial.printf("[fr24] GET %s\n", url);

    http.setReuse(false);
    http.setTimeout(8000);

    if (!http.begin(url)) {
        Serial.println("[fr24] HTTP begin failed");
        return false;
    }

    http.setUserAgent("Mozilla/5.0 (Windows NT 10.0; Win64; x64)");

    const int code = http.GET();
    if (code != HTTP_CODE_OK) {
        Serial.printf("[fr24] HTTP error code: %d\n", code);
        http.end();
        return false;
    }

    // Leitura direta do Stream para evitar alocar a string inteira na RAM
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, http.getStream());
    http.end();

    if (err) {
        Serial.printf("[fr24] JSON parse error: %s\n", err.c_str());
        return false;
    }

    JsonObject root = doc.as<JsonObject>();
    size_t n = 0;

    for (JsonPair kv : root) {
        if (n >= out_capacity) break;

        // O Flightradar24 envia as aeronaves como chaves de objeto (ex: "381b8f10"), ignoramos chaves de sistema como "full_count"
        const char *key = kv.key().c_str();
        if (strcmp(key, "full_count") == 0 || strcmp(key, "version") == 0 || strcmp(key, "stats") == 0) {
            continue;
        }

        JsonArray arr = kv.value().as<JsonArray>();
        if (arr.isNull() || arr.size() < 14) continue;

        FlightData f{};
        
        // Formato da array do FR24:
        // arr[0] = icao24 hex
        // arr[1] = lat
        // arr[2] = lon
        // arr[3] = heading
        // arr[4] = altitude (pés)
        // arr[5] = velocidade (knots)
        // arr[13] = callsign
        
        const char *hex = arr[0].as<const char*>();
        safe_copy(f.icao24, sizeof(f.icao24), hex ? hex : "");

        f.lat = arr[1].as<float>();
        f.lon = arr[2].as<float>();
        f.heading_deg = arr[3].as<float>();

        float alt_ft = arr[4].as<float>();
        f.altitude_m = alt_ft * 0.3048f;

        float speed_kts = arr[5].as<float>();
        f.speed_mps = speed_kts * 0.51444f;

        const char *callsign = arr[13].as<const char*>();
        safe_copy(f.callsign, sizeof(f.callsign), callsign ? callsign : "");

        out_flights[n++] = f;
    }

    *out_count = n;
    Serial.printf("[fr24] Successfully parsed %u flights\n", (unsigned)n);
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
