
/*
 *   Source [web.c]
 */

#include "web.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "secrets.h"

#define MAX_CLIENTS 7

static httpd_handle_t server;

static void on_network_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
	if (base == WIFI_EVENT && (id == WIFI_EVENT_STA_START || id == WIFI_EVENT_STA_DISCONNECTED)) {
		esp_wifi_connect();
	} else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
		ip_event_got_ip_t *event = data;
		printf("WiFi connected - IP: " IPSTR "\n", IP2STR(&event->ip_info.ip));
	}
}

static void wifi_init(void)
{
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK(err);

	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_netif_create_default_wifi_sta();

	wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_network_event, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_network_event, NULL));

	wifi_config_t wifi_cfg = {
		.sta = { .ssid = WIFI_SSID, .password = WIFI_PASSWORD },
	};
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
	ESP_ERROR_CHECK(esp_wifi_start());
}

// Called for every frame the browser sends (the server handles the handshake itself)
static esp_err_t ws_handler(httpd_req_t *req)
{
	// The page never sends anything, but drain any frame so the socket stays in sync
	uint8_t buf[32];
	httpd_ws_frame_t frame = { .payload = buf };
	return httpd_ws_recv_frame(req, &frame, sizeof(buf));
}

// Runs inside the server task (queued with httpd_queue_work); takes ownership of text
static void send_to_clients(void *text)
{
	httpd_ws_frame_t frame = {
		.type    = HTTPD_WS_TYPE_TEXT,
		.payload = text,
		.len     = strlen(text),
	};

	int fds[MAX_CLIENTS];
	size_t count = MAX_CLIENTS;
	if (httpd_get_client_list(server, &count, fds) == ESP_OK) {
		for (size_t i = 0; i < count; i++) {
			if (httpd_ws_get_fd_info(server, fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
				httpd_ws_send_frame_async(server, fds[i], &frame);
			}
		}
	}

	free(text);
}

void web_init(void)
{
	wifi_init();

	httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
	cfg.max_open_sockets = MAX_CLIENTS;
	ESP_ERROR_CHECK(httpd_start(&server, &cfg));

	httpd_uri_t ws_uri = {
		.uri          = "/ws",
		.method       = HTTP_GET,
		.handler      = ws_handler,
		.is_websocket = true,
	};
	ESP_ERROR_CHECK(httpd_register_uri_handler(server, &ws_uri));
}

void web_broadcast(const char *text)
{
	char *copy = strdup(text);
	if (copy != NULL && httpd_queue_work(server, send_to_clients, copy) != ESP_OK) {
		free(copy);
	}
}
