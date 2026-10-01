#include "flight_data.h"
#include "../config.h"

#include <Arduino.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>
#include <math.h>
#include <string.h>

namespace {

constexpr uint32_t ROUTE_CACHE_TTL_MS = 10UL * 60UL * 1000UL;
constexpr uint32_t ROUTE_RETRY_MS = 90UL * 1000UL;
constexpr size_t ROUTE_CACHE_SIZE = MAX_FLIGHTS;

struct RouteCacheEntry {
    char icao24[9];
    char departure[5];
    char arrival[5];
    char callsign_iata[9];
    uint32_t updated_ms;
    bool valid;
};

static RouteCacheEntry s_route_cache[ROUTE_CACHE_SIZE];

static String build_url() {
    const float lat = g_home_lat;
    const float lon = g_home_lon;
    const int dist_km = (int)RADAR_RADIUS_KM;

    // Usa endpoint v2 com limitação de raio para manter o JSON leve
    String url = "http://api.adsb.lol/v2/lat/";
    url += String(lat, 4);
    url += "/lon/";
    url += String(lon, 4);
    url += "/dist/";
    url += String(dist_km);
    return url;
}

static void safe_copy(char *dst, size_t dst_size, const char *src) {
    if (dst_size == 0 || dst == nullptr) return;
    if (src == nullptr) {
        dst[0] = '\0';
        return;
    }
    strncpy(dst, src, dst_size - 1);
    dst[dst_size - 1] = '\0';
}

static int find_cache_slot(const char *icao24) {
    for (size_t i = 0; i < ROUTE_CACHE_SIZE; ++i) {
        if (strncmp(s_route_cache[i].icao24, icao24, sizeof(s_route_cache[i].icao24)) == 0) {
            return (int)i;
        }
    }
    return -1;
}

static bool cache_lookup(const char *icao24, char *dep, size_t dep_size, char *arr, size_t arr_size,
                         char *cs_iata, size_t cs_iata_size) {
    const int slot = find_cache_slot(icao24);
    if (slot < 0 || !s_route_cache[slot].valid) return false;

    if ((millis() - s_route_cache[slot].updated_ms) > ROUTE_CACHE_TTL_MS) return false;

    safe_copy(dep, dep_size, s_route_cache[slot].departure);
    safe_copy(arr, arr_size, s_route_cache[slot].arrival);
    if (cs_iata != nullptr && cs_iata_size > 0) {
        safe_copy(cs_iata, cs_iata_size, s_route_cache[slot].callsign_iata);
    }
    return dep[0] != '\0' && arr[0] != '\0';
}

} // namespace

bool flight_data_fetch_flights(FlightData *out_flights, size_t out_capacity, size_t *out_count) {
    if (out_flights == nullptr || out_count == nullptr || out_capacity == 0) {
        return false;
    }

    *out_count = 0;

    HTTPClient http;
    const String url = build_url();
    Serial.printf("[adsb] GET %s\n", url.c_str());

    http.setReuse(false); // Força fecho de socket para evitar travamento de conexão
    http.setTimeout(8000);

    if (!http.begin(url)) {
        Serial.println("[adsb] HTTP begin failed");
        return false;
    }

    const int code = http.GET();
    if (code != HTTP_CODE_OK) {
        Serial.printf("[adsb] HTTP error code: %d\n", code);
        http.end();
        return false;
    }

    // Usa um filtro para extrair APENAS os campos necessários e economizar RAM na desserialização
    JsonDocument filter;
    filter["ac"][0]["hex"] = true;
    filter["ac"][0]["flight"] = true;
    filter["ac"][0]["lat"] = true;
    filter["ac"][0]["lon"] = true;
    filter["ac"][0]["alt_baro"] = true;
    filter["ac"][0]["alt"] = true;
    filter["ac"][0]["gs"] = true;
    filter["ac"][0]["track"] = true;
    filter["ac"][0]["t"] = true;

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, http.getStream(), DeserializationOption::Filter(filter));
    http.end();

    if (err) {
        Serial.printf("[adsb] JSON parse error: %s\n", err.c_str());
        return false;
    }

    JsonArray ac = doc["ac"].as<JsonArray>();
    if (ac.isNull()) {
        Serial.println("[adsb] No 'ac' array found");
        return true; // Retorna true (sucesso), apenas sem aeronaves no raio
    }

    size_t n = 0;
    for (JsonVariant v : ac) {
        if (n >= out_capacity) break;

        JsonObject obj = v.as<JsonObject>();
        if (obj.isNull() || obj["lat"].isNull() || obj["lon"].isNull()) continue;

        FlightData f{};
        const char *hex = obj["hex"].isNull() ? "" : obj["hex"].as<const char*>();
        safe_copy(f.icao24, sizeof(f.icao24), hex);
        safe_copy(f.callsign, sizeof(f.callsign), obj["flight"].isNull() ? "" : obj["flight"].as<const char*>());

        f.lat = obj["lat"].as<float>();
        f.lon = obj["lon"].as<float>();

        float alt_ft = 0.0f;
        if (!obj["alt_baro"].isNull()) alt_ft = obj["alt_baro"].as<float>();
        else if (!obj["alt"].isNull()) alt_ft = obj["alt"].as<float>();
        f.altitude_m = alt_ft * 0.3048f;

        f.speed_mps = obj["gs"].isNull() ? 0.0f : (obj["gs"].as<float>() * 0.51444f);
        f.heading_deg = obj["track"].isNull() ? 0.0f : obj["track"].as<float>();

        char dep_buf[5] = {0}, arr_buf[5] = {0}, iata_buf[9] = {0};
        if (cache_lookup(f.icao24, dep_buf, sizeof(dep_buf), arr_buf, sizeof(arr_buf), iata_buf, sizeof(iata_buf))) {
            safe_copy(f.departure, sizeof(f.departure), dep_buf);
            safe_copy(f.arrival, sizeof(f.arrival), arr_buf);
            safe_copy(f.callsign_iata, sizeof(f.callsign_iata), iata_buf);
        }

        out_flights[n++] = f;
    }

    *out_count = n;
    Serial.printf("[adsb] Successfully parsed %u flights\n", (unsigned)n);
    return true;
}

bool flight_data_fetch_route(const char *icao24, const char *callsign,
                             char *dep, size_t dep_size,
                             char *arr, size_t arr_size,
                             char *callsign_iata, size_t callsign_iata_size) {
    if (dep != nullptr && dep_size > 0) dep[0] = '\0';
    if (arr != nullptr && arr_size > 0) arr[0] = '\0';
    if (callsign_iata != nullptr && callsign_iata_size > 0) callsign_iata[0] = '\0';
    return false; // Desativado temporariamente para isolar estabilidade de rede
}
