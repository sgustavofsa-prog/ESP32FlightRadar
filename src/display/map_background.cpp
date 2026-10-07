#include "map_background.h"
#include "../config.h"

#include <Arduino.h>

bool map_background_fetch_once() {
    // Retorna true instantaneamente para liberar a tarefa de rede.
    // Atualiza apenas os dados do radar e coordenadas para a localidade selecionada.
    Serial.printf("[map] Localidade ativa: %s (%.4f, %.4f)\n", g_location_name, g_home_lat, g_home_lon);
    return true;
}

void map_background_try_install() {
    // Mantida vazia para compatibilidade com as chamadas de renderizacao da display_task
}
