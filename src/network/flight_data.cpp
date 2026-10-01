#include "flight_data.h"
#include "../config.h"

#include <Arduino.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
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

} // namespace

bool flight_data_fetch_flights(FlightData *out_flights, size_t out_capacity, size_t *out_count) {
    if (out_flights == nullptr || out_count == nullptr || out_capacity == 0) {
        return false;
    }

    *out_count = 0;

    // Cálculo da área (~50km)
    const float lat_delta = RADAR_RADIUS_KM / 111.0f;
    const float lon_delta = RADAR_RADIUS_KM / (111.0f * cosf(g_home_lat * 0.01745329252f));

    const float lat_max = g_home_lat + lat_delta;
    const float lat_min = g_home_lat - lat_delta;
    const float lon_min = g_home_lon - lon_delta;
    const float lon_max = g_home_lon + lon_delta;

    // Endpoint seguro do Flightradar24
    char url[256];
    snprintf(url, sizeof(url),
             "https://data-cloud.flightradar24.com/zones/fcgi/feed.json?bounds=%.4f,%.4f,%.4f,%.4f&faa=1&mlat=1&flarm=1&adsb=1&gnd=0&air=1",
             lat_max, lat_min, lon_min, lon_max);

    WiFiClientSecure client;
    client.setInsecure(); // Ignora verificação rigorosa de CA para poupar RAM

    HTTPClient http;
    Serial.printf("[fr24] GET %s\n", url);

    if (!http.begin(client, url)) {
        Serial.println("[fr24] HTTP begin failed");
        return false;
    }

    // Cabeçalhos essenciais para contornar o bloqueio de bot/Cloudflare do FR24
    http.setUserAgent("Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/120.0.0.0 Safari/537.36");
    http.addHeader("Accept", "application/json, text/javascript, */*; q=0.01");
    http.addHeader("Accept-Language", "pt-BR,pt;q=0.9,en-US;q=0.8,en;q=0.7");
    http.addHeader("Referer", "https://www.flightradar24.com/");
    http.addHeader("Origin", "https://www.flightradar24.com");
    http.setTimeout(10000);

    const int code = http.GET();
    if (code != HTTP_CODE_OK) {
        Serial.printf("[fr24] HTTP error code: %d\n", code);
        http.end();
        client.stop();
        return false;
    }

    // Desserialização do JSON diretamente do Stream
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, http.getStream());
    http.end();
    client.stop();

    if (err) {
        Serial.printf("[fr24] JSON parse error: %s\n", err.c_str());
        return false;
    }

    JsonObject root = doc.as<JsonObject>();
    size_t n = 0;

    for (JsonPair kv : root) {
        if (n >= out_capacity) break;

        const char *key = kv.key().c_str();
        if (strcmp(key, "full_count") == 0 || strcmp(key, "version") == 0 || strcmp(key, "stats") == 0) {
            continue;
        }

        JsonArray arr = kv.value().as<JsonArray>();
        if (arr.isNull() || arr.size() < 14) continue;

        FlightData f{};

        // Mapeamento dos campos do FR24:
        // arr[0] = icao24, arr[1] = lat, arr[2] = lon, arr[3] = heading, arr[4] = alt, arr[5] = speed, arr[13] = callsign
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
