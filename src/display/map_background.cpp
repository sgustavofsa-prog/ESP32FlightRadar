#include "map_background.h"
#include "../config.h"
#include "../display/ui.h"

#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <lvgl.h>

static uint8_t *s_map_png_data = nullptr;
static size_t s_map_png_size = 0;
static bool s_map_needs_install = false;
static lv_obj_t *s_map_img_obj = nullptr;

bool map_background_fetch_once() {
    // URL de servidor gratuito e leve de tiles estáticos do OpenStreetMap / CartoDB
    // Utiliza as variáveis globais g_home_lat e g_home_lon
    char url[384];
    snprintf(url, sizeof(url),
             "https://staticmap.openstreetmap.de/staticmap.php?center=%.4f,%.4f&zoom=10&size=240x240&maptype=mapnik",
             g_home_lat, g_home_lon);

    WiFiClientSecure client;
    client.setInsecure(); // Desativa validação TLS para poupar RAM

    HTTPClient http;
    Serial.printf("[map] Baixando mapa para %s: %s\n", g_location_name, url);

    if (!http.begin(client, url)) {
        Serial.println("[map] HTTP begin falhou");
        return false;
    }

    http.setUserAgent("Mozilla/5.0 (Windows NT 10.0; Win64; x64)");
    http.setTimeout(10000);
    const int code = http.GET();

    if (code != HTTP_CODE_OK) {
        Serial.printf("[map] Erro HTTP ao baixar mapa: %d\n", code);
        http.end();
        client.stop();
        return false;
    }

    int len = http.getSize();
    if (len <= 0) {
        Serial.println("[map] Tamanho de resposta invalido");
        http.end();
        client.stop();
        return false;
    }

    // Liberta a memória do mapa anterior se existir
    if (s_map_png_data != nullptr) {
        free(s_map_png_data);
        s_map_png_data = nullptr;
        s_map_png_size = 0;
    }

    s_map_png_data = (uint8_t *)malloc(len);
    if (!s_map_png_data) {
        Serial.println("[map] Sem RAM suficiente para alocar o novo mapa");
        http.end();
        client.stop();
        return false;
    }

    WiFiClient *stream = http.getStreamPtr();
    size_t total_read = 0;
    uint32_t start_time = millis();

    while (http.connected() && total_read < (size_t)len && (millis() - start_time < 8000)) {
        size_t avail = stream->available();
        if (avail) {
            int r = stream->readBytes(s_map_png_data + total_read, avail);
            total_read += r;
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    http.end();
    client.stop();

    s_map_png_size = total_read;
    s_map_needs_install = true; // Sinaliza à task de ecrã para atualizar a imagem no LVGL
    Serial.printf("[map] Download concluido (%d bytes) para %s\n", (int)total_read, g_location_name);
    return true;
}

void map_background_try_install() {
    if (!s_map_needs_install || s_map_png_data == nullptr) {
        return;
    }

    s_map_needs_install = false;

    // Se já existir um objeto de imagem no ecrã, remove o antigo
    if (s_map_img_obj != nullptr) {
        lv_obj_del(s_map_img_obj);
        s_map_img_obj = nullptr;
    }

    // Recria o objeto da imagem de fundo no LVGL
    static lv_img_dsc_t img_dsc;
    img_dsc.header.always_zero = 0;
    img_dsc.header.w = 240;
    img_dsc.header.h = 240;
    img_dsc.data_size = s_map_png_size;
    img_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;
    img_dsc.data = s_map_png_data;

    s_map_img_obj = lv_img_create(lv_scr_act());
    lv_img_set_src(s_map_img_obj, &img_dsc);
    lv_obj_move_background(s_map_img_obj); // Move o mapa para a camada de fundo do radar

    lv_obj_invalidate(lv_scr_act()); // Força o redesenho da tela
    Serial.println("[map] Novo mapa instalado com sucesso!");
}
