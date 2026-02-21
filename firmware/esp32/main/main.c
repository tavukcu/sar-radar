/**
 * @file main.c
 * @brief ESP32-C3 WiFi Bridge Firmware for SAR Radar
 *
 * This firmware runs on the ESP32-C3 module, acting as a WiFi bridge
 * between the FPGA radar data pipeline and a host computer. It receives
 * processed radar data from the FPGA via SPI and streams it over WiFi
 * using a simple TCP protocol.
 *
 * Architecture:
 *   - SPI Slave: Receives data from FPGA (master initiated transfers)
 *   - WiFi AP: Creates access point for host computer connection
 *   - TCP Server: Streams radar data to connected clients (port 5000)
 *   - UDP Server: Handles device discovery broadcasts (port 5001)
 *   - BLE: Configuration interface for WiFi settings
 *   - OTA: Over-the-air firmware update support
 *
 * Task Structure:
 *   - main_task:        Initialization and watchdog
 *   - spi_rx_task:      SPI slave receive (high priority)
 *   - tcp_server_task:  TCP client management
 *   - tcp_stream_task:  Data streaming per client
 *   - udp_disc_task:    UDP discovery broadcast
 *   - status_task:      LED control and status reporting
 *
 * Double Buffering:
 *   Two SPI receive buffers alternate to allow continuous reception
 *   while the previous buffer is being transmitted over WiFi.
 *
 * @author SAR Radar Project
 * @date   2026-02-21
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/spi_slave.h"
#include "lwip/sockets.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "wifi_stream.h"

/*============================================================================
 * Constants & Configuration
 *============================================================================*/

static const char *TAG = "sar-radar";

/** Firmware version */
#define FW_VERSION_MAJOR    1
#define FW_VERSION_MINOR    0
#define FW_VERSION_PATCH    0

/** SPI Slave Pin Definitions (ESP32-C3) */
#define SPI_SLAVE_MOSI_PIN  7
#define SPI_SLAVE_MISO_PIN  2
#define SPI_SLAVE_SCLK_PIN  6
#define SPI_SLAVE_CS_PIN    10

/** Task stack sizes */
#define SPI_RX_TASK_STACK   4096
#define TCP_SERVER_STACK    4096
#define TCP_STREAM_STACK    4096
#define UDP_DISC_STACK      2048
#define STATUS_TASK_STACK   2048

/** Task priorities (higher number = higher priority) */
#define SPI_RX_TASK_PRIO    10
#define TCP_SERVER_PRIO     5
#define TCP_STREAM_PRIO     6
#define UDP_DISC_PRIO       3
#define STATUS_TASK_PRIO    2

/** Event group bits */
#define WIFI_STARTED_BIT    BIT0
#define CLIENT_CONNECTED_BIT BIT1
#define DATA_READY_BIT      BIT2
#define STREAMING_BIT       BIT3

/*============================================================================
 * Global State
 *============================================================================*/

/** Event group for inter-task communication */
static EventGroupHandle_t s_event_group;

/** Mutex for protecting shared data */
static SemaphoreHandle_t s_data_mutex;

/** Stream configuration (loaded from NVS) */
static stream_config_t s_config;

/** Double buffer for SPI receive data */
static uint8_t s_spi_buf[DOUBLE_BUF_COUNT][SPI_RX_BUF_SIZE];
static volatile int s_active_buf = 0;     // Buffer currently being filled by SPI
static volatile int s_ready_buf = -1;     // Buffer ready for WiFi transmission
static volatile size_t s_ready_buf_len = 0;

/** Streaming statistics */
static uint64_t s_total_tx_bytes = 0;
static uint32_t s_total_tx_packets = 0;
static uint32_t s_dropped_packets = 0;
static uint32_t s_sequence_number = 0;

/** Connected TCP client sockets */
static int s_client_sockets[STREAM_MAX_CLIENTS];
static int s_num_clients = 0;
static SemaphoreHandle_t s_client_mutex;

/** FPGA status (read via SPI) */
static uint8_t s_fpga_status = 0;

/*============================================================================
 * Fletcher-16 Checksum
 *============================================================================*/

/**
 * @brief Calculate Fletcher-16 checksum over a data buffer
 *
 * Fletcher-16 provides a simple error detection mechanism with low
 * computational overhead, suitable for the ESP32-C3's limited CPU.
 */
uint16_t fletcher16(const uint8_t *data, size_t length)
{
    uint16_t sum1 = 0xFF, sum2 = 0xFF;

    while (length) {
        size_t tlen = (length > 20) ? 20 : length;
        length -= tlen;
        do {
            sum1 += *data++;
            sum2 += sum1;
        } while (--tlen);
        sum1 = (sum1 & 0xFF) + (sum1 >> 8);
        sum2 = (sum2 & 0xFF) + (sum2 >> 8);
    }
    sum1 = (sum1 & 0xFF) + (sum1 >> 8);
    sum2 = (sum2 & 0xFF) + (sum2 >> 8);
    return (sum2 << 8) | sum1;
}

/*============================================================================
 * NVS Configuration Storage
 *============================================================================*/

/**
 * @brief Load configuration from NVS, or set defaults
 */
static void config_load(stream_config_t *config)
{
    /* Set defaults first */
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);

    snprintf(config->wifi_ssid, sizeof(config->wifi_ssid),
             "%s%02X%02X", WIFI_AP_SSID_PREFIX, mac[4], mac[5]);
    strncpy(config->wifi_password, WIFI_AP_PASSWORD, sizeof(config->wifi_password));
    config->wifi_channel = WIFI_AP_CHANNEL;
    config->stream_enabled = true;
    config->discovery_enabled = true;
    config->stream_data_types = 0xFFFF;  /* All types enabled */
    config->stream_rate_hz = 50;

    /* TODO: Override defaults with NVS stored values */
    /*
    nvs_handle_t nvs;
    if (nvs_open("radar_cfg", NVS_READONLY, &nvs) == ESP_OK) {
        size_t ssid_len = sizeof(config->wifi_ssid);
        nvs_get_str(nvs, "wifi_ssid", config->wifi_ssid, &ssid_len);
        size_t pass_len = sizeof(config->wifi_password);
        nvs_get_str(nvs, "wifi_pass", config->wifi_password, &pass_len);
        nvs_get_u8(nvs, "wifi_ch", &config->wifi_channel);
        nvs_close(nvs);
    }
    */

    ESP_LOGI(TAG, "Config: SSID=%s, Channel=%d", config->wifi_ssid, config->wifi_channel);
}

/**
 * @brief Save configuration to NVS
 */
static esp_err_t config_save(const stream_config_t *config)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("radar_cfg", NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return err;
    }

    nvs_set_str(nvs, "wifi_ssid", config->wifi_ssid);
    nvs_set_str(nvs, "wifi_pass", config->wifi_password);
    nvs_set_u8(nvs, "wifi_ch", config->wifi_channel);

    err = nvs_commit(nvs);
    nvs_close(nvs);
    return err;
}

/*============================================================================
 * WiFi Initialization (AP Mode)
 *============================================================================*/

/**
 * @brief WiFi event handler
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_AP_START:
                ESP_LOGI(TAG, "WiFi AP started");
                xEventGroupSetBits(s_event_group, WIFI_STARTED_BIT);
                break;
            case WIFI_EVENT_AP_STACONNECTED: {
                wifi_event_ap_staconnected_t *event =
                    (wifi_event_ap_staconnected_t *)event_data;
                ESP_LOGI(TAG, "Station connected: " MACSTR, MAC2STR(event->mac));
                break;
            }
            case WIFI_EVENT_AP_STADISCONNECTED: {
                wifi_event_ap_stadisconnected_t *event =
                    (wifi_event_ap_stadisconnected_t *)event_data;
                ESP_LOGI(TAG, "Station disconnected: " MACSTR, MAC2STR(event->mac));
                break;
            }
            default:
                break;
        }
    }
}

/**
 * @brief Initialize WiFi in SoftAP mode
 */
static esp_err_t wifi_init_ap(const stream_config_t *config)
{
    esp_err_t err;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .max_connection = WIFI_AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    /* Copy SSID and password */
    strncpy((char *)wifi_config.ap.ssid, config->wifi_ssid,
            sizeof(wifi_config.ap.ssid));
    wifi_config.ap.ssid_len = strlen(config->wifi_ssid);
    strncpy((char *)wifi_config.ap.password, config->wifi_password,
            sizeof(wifi_config.ap.password));
    wifi_config.ap.channel = config->wifi_channel;

    /* If password is empty, use open auth */
    if (strlen(config->wifi_password) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));

    /* Set bandwidth to 40 MHz for maximum throughput */
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT40));

    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi start failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "WiFi AP initialized. SSID: %s", config->wifi_ssid);
    return ESP_OK;
}

/*============================================================================
 * SPI Slave Interface
 *============================================================================*/

/**
 * @brief SPI slave post-transaction callback
 *
 * Called from ISR context when an SPI transaction completes.
 * Swaps the double buffer and signals the streaming task.
 */
static void IRAM_ATTR spi_post_trans_cb(spi_slave_transaction_t *trans)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Mark current buffer as ready for transmission */
    s_ready_buf = s_active_buf;
    s_ready_buf_len = trans->trans_len / 8;  /* Convert bits to bytes */

    /* Swap to the other buffer for next SPI transfer */
    s_active_buf = (s_active_buf + 1) % DOUBLE_BUF_COUNT;

    /* Signal data ready */
    xEventGroupSetBitsFromISR(s_event_group, DATA_READY_BIT,
                              &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief Initialize SPI slave peripheral
 */
static esp_err_t spi_slave_init(void)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SPI_SLAVE_MOSI_PIN,
        .miso_io_num = SPI_SLAVE_MISO_PIN,
        .sclk_io_num = SPI_SLAVE_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_RX_BUF_SIZE,
    };

    spi_slave_interface_config_t slave_cfg = {
        .spics_io_num = SPI_SLAVE_CS_PIN,
        .flags = 0,
        .queue_size = 3,
        .mode = 0,                          /* SPI Mode 0 (CPOL=0, CPHA=0) */
        .post_trans_cb = spi_post_trans_cb,
    };

    esp_err_t err = spi_slave_initialize(SPI2_HOST, &bus_cfg, &slave_cfg,
                                          SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI slave init failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "SPI slave initialized (MOSI=%d, MISO=%d, SCLK=%d, CS=%d)",
             SPI_SLAVE_MOSI_PIN, SPI_SLAVE_MISO_PIN,
             SPI_SLAVE_SCLK_PIN, SPI_SLAVE_CS_PIN);
    return ESP_OK;
}

/*============================================================================
 * SPI Receive Task
 *============================================================================*/

/**
 * @brief SPI receive task
 *
 * Continuously queues SPI slave transactions. The FPGA (master) initiates
 * transfers whenever it has radar data available. Double buffering ensures
 * no data is lost during WiFi transmission.
 */
static void spi_rx_task(void *pvParameters)
{
    ESP_LOGI(TAG, "SPI RX task started");

    spi_slave_transaction_t trans;
    memset(&trans, 0, sizeof(trans));

    while (1) {
        /* Set up next receive transaction on the active buffer */
        trans.length = SPI_RX_BUF_SIZE * 8;  /* Length in bits */
        trans.rx_buffer = s_spi_buf[s_active_buf];
        trans.tx_buffer = NULL;               /* No data to send to FPGA */

        /* Queue the transaction and wait for FPGA to initiate transfer */
        esp_err_t err = spi_slave_transmit(SPI2_HOST, &trans, portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SPI slave transmit error: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* Transaction complete - callback already swapped buffers
         * and signaled DATA_READY_BIT */
        ESP_LOGD(TAG, "SPI RX: %d bytes", (int)(trans.trans_len / 8));
    }
}

/*============================================================================
 * TCP Server
 *============================================================================*/

/**
 * @brief Send a complete packet to a TCP client socket
 *
 * Constructs a packet with header, payload, and checksum, then sends
 * it atomically. Returns the number of bytes sent or -1 on error.
 */
static int send_packet(int sock, stream_data_type_t type,
                       const void *payload, uint16_t payload_len)
{
    /* Allocate buffer for complete packet */
    size_t total_len = sizeof(stream_packet_header_t) + payload_len + 2;
    uint8_t *pkt = malloc(total_len);
    if (!pkt) {
        return -1;
    }

    /* Build header */
    stream_packet_header_t *hdr = (stream_packet_header_t *)pkt;
    hdr->magic = STREAM_MAGIC;
    hdr->sequence = s_sequence_number++;
    hdr->data_type = (uint16_t)type;
    hdr->payload_length = payload_len;
    hdr->version = STREAM_PROTOCOL_VERSION;

    /* Copy payload */
    if (payload && payload_len > 0) {
        memcpy(pkt + sizeof(stream_packet_header_t), payload, payload_len);
    }

    /* Calculate and append checksum */
    uint16_t checksum = fletcher16(pkt, sizeof(stream_packet_header_t) + payload_len);
    memcpy(pkt + sizeof(stream_packet_header_t) + payload_len, &checksum, 2);

    /* Send */
    int sent = send(sock, pkt, total_len, 0);
    free(pkt);

    if (sent > 0) {
        s_total_tx_bytes += sent;
        s_total_tx_packets++;
    }

    return sent;
}

/**
 * @brief TCP streaming task for a single client
 *
 * Waits for data ready events and sends buffered radar data to the client.
 * Runs one instance per connected client.
 */
static void tcp_stream_task(void *pvParameters)
{
    int client_sock = (int)(intptr_t)pvParameters;
    ESP_LOGI(TAG, "Stream task started for client socket %d", client_sock);

    /* Send initial status */
    status_payload_t status = {0};
    status.system_state = s_fpga_status;
    send_packet(client_sock, DATA_TYPE_STATUS, &status, sizeof(status));

    while (1) {
        /* Wait for data ready event */
        EventBits_t bits = xEventGroupWaitBits(
            s_event_group, DATA_READY_BIT,
            pdTRUE,   /* Clear on exit */
            pdFALSE,  /* Wait for any bit */
            pdMS_TO_TICKS(1000)  /* 1 second timeout for heartbeat */
        );

        if (bits & DATA_READY_BIT) {
            /* Data available in ready buffer */
            int buf_idx = s_ready_buf;
            size_t buf_len = s_ready_buf_len;

            if (buf_idx >= 0 && buf_len > 0) {
                int ret = send_packet(client_sock, DATA_TYPE_RANGE_FFT,
                                      s_spi_buf[buf_idx], buf_len);
                if (ret < 0) {
                    ESP_LOGE(TAG, "Send failed, client disconnected");
                    break;
                }
            }
        } else {
            /* Timeout - send heartbeat */
            send_packet(client_sock, DATA_TYPE_HEARTBEAT, NULL, 0);
        }

        /* Check if socket is still valid */
        int error = 0;
        socklen_t len = sizeof(error);
        if (getsockopt(client_sock, SOL_SOCKET, SO_ERROR, &error, &len) != 0 ||
            error != 0) {
            ESP_LOGI(TAG, "Client socket error, disconnecting");
            break;
        }
    }

    /* Clean up */
    close(client_sock);

    /* Remove from client list */
    xSemaphoreTake(s_client_mutex, portMAX_DELAY);
    for (int i = 0; i < STREAM_MAX_CLIENTS; i++) {
        if (s_client_sockets[i] == client_sock) {
            s_client_sockets[i] = -1;
            s_num_clients--;
            break;
        }
    }
    xSemaphoreGive(s_client_mutex);

    ESP_LOGI(TAG, "Stream task ended for client socket %d", client_sock);
    vTaskDelete(NULL);
}

/**
 * @brief TCP server task
 *
 * Listens for incoming TCP connections on STREAM_TCP_PORT and spawns
 * a streaming task for each connected client.
 */
static void tcp_server_task(void *pvParameters)
{
    ESP_LOGI(TAG, "TCP server task started, waiting for WiFi...");

    /* Wait for WiFi to start */
    xEventGroupWaitBits(s_event_group, WIFI_STARTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    /* Create listening socket */
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    /* Allow socket reuse */
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    /* Bind to port */
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(STREAM_TCP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Socket bind failed: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    if (listen(listen_sock, STREAM_MAX_CLIENTS) < 0) {
        ESP_LOGE(TAG, "Socket listen failed: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "TCP server listening on port %d", STREAM_TCP_PORT);

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);

        int client_sock = accept(listen_sock,
                                  (struct sockaddr *)&client_addr, &addr_len);
        if (client_sock < 0) {
            ESP_LOGE(TAG, "Accept failed: errno %d", errno);
            continue;
        }

        ESP_LOGI(TAG, "TCP client connected from " IPSTR ":%d",
                 IP2STR(&client_addr.sin_addr), ntohs(client_addr.sin_port));

        /* Set TCP_NODELAY for low latency */
        int flag = 1;
        setsockopt(client_sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

        /* Set send buffer size */
        int send_buf = 16384;
        setsockopt(client_sock, SOL_SOCKET, SO_SNDBUF, &send_buf, sizeof(send_buf));

        /* Check if we can accept more clients */
        xSemaphoreTake(s_client_mutex, portMAX_DELAY);
        if (s_num_clients >= STREAM_MAX_CLIENTS) {
            ESP_LOGW(TAG, "Max clients reached, rejecting connection");
            close(client_sock);
            xSemaphoreGive(s_client_mutex);
            continue;
        }

        /* Add to client list */
        for (int i = 0; i < STREAM_MAX_CLIENTS; i++) {
            if (s_client_sockets[i] == -1) {
                s_client_sockets[i] = client_sock;
                s_num_clients++;
                break;
            }
        }
        xSemaphoreGive(s_client_mutex);

        /* Spawn streaming task for this client */
        char task_name[20];
        snprintf(task_name, sizeof(task_name), "stream_%d", client_sock);
        xTaskCreate(tcp_stream_task, task_name, TCP_STREAM_STACK,
                    (void *)(intptr_t)client_sock, TCP_STREAM_PRIO, NULL);

        xEventGroupSetBits(s_event_group, CLIENT_CONNECTED_BIT);
    }

    close(listen_sock);
    vTaskDelete(NULL);
}

/*============================================================================
 * UDP Discovery Service
 *============================================================================*/

/**
 * @brief UDP discovery task
 *
 * Listens for discovery requests and responds with device information.
 * Also periodically broadcasts announcements for passive discovery.
 */
static void udp_discovery_task(void *pvParameters)
{
    ESP_LOGI(TAG, "UDP discovery task started");

    /* Wait for WiFi */
    xEventGroupWaitBits(s_event_group, WIFI_STARTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "UDP socket creation failed");
        vTaskDelete(NULL);
        return;
    }

    /* Allow broadcast */
    int broadcast = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    /* Bind to discovery port */
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(STREAM_UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr));

    /* Set receive timeout for periodic announcements */
    struct timeval tv = { .tv_sec = 5, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    /* Prepare discovery response */
    discovery_payload_t disc = {0};
    strncpy(disc.device_name, "SAR-RADAR", sizeof(disc.device_name));
    esp_read_mac(disc.mac_address, ESP_MAC_WIFI_SOFTAP);
    disc.tcp_port = STREAM_TCP_PORT;
    disc.protocol_version = STREAM_PROTOCOL_VERSION;
    disc.hw_version = 1;
    disc.fw_version_major = FW_VERSION_MAJOR;
    disc.fw_version_minor = FW_VERSION_MINOR;
    disc.fw_version_patch = FW_VERSION_PATCH;

    uint8_t rx_buf[64];
    struct sockaddr_in client_addr;
    socklen_t addr_len;

    while (1) {
        addr_len = sizeof(client_addr);
        int len = recvfrom(sock, rx_buf, sizeof(rx_buf), 0,
                           (struct sockaddr *)&client_addr, &addr_len);

        if (len > 0) {
            /* Check for discovery magic in request */
            if (len >= 4 && *(uint32_t *)rx_buf == STREAM_MAGIC) {
                ESP_LOGI(TAG, "Discovery request from " IPSTR,
                         IP2STR(&client_addr.sin_addr));

                disc.status = s_fpga_status;

                /* Build and send response packet */
                size_t pkt_size = sizeof(stream_packet_header_t) +
                                  sizeof(discovery_payload_t) + 2;
                uint8_t *pkt = malloc(pkt_size);
                if (pkt) {
                    stream_packet_header_t *hdr = (stream_packet_header_t *)pkt;
                    hdr->magic = STREAM_MAGIC;
                    hdr->sequence = s_sequence_number++;
                    hdr->data_type = DATA_TYPE_DISCOVERY;
                    hdr->payload_length = sizeof(discovery_payload_t);
                    hdr->version = STREAM_PROTOCOL_VERSION;

                    memcpy(pkt + sizeof(stream_packet_header_t), &disc,
                           sizeof(discovery_payload_t));

                    uint16_t cs = fletcher16(pkt, pkt_size - 2);
                    memcpy(pkt + pkt_size - 2, &cs, 2);

                    sendto(sock, pkt, pkt_size, 0,
                           (struct sockaddr *)&client_addr, addr_len);
                    free(pkt);
                }
            }
        } else {
            /* Timeout - send periodic broadcast announcement */
            disc.status = s_fpga_status;

            struct sockaddr_in bcast_addr = {
                .sin_family = AF_INET,
                .sin_port = htons(STREAM_UDP_PORT),
                .sin_addr.s_addr = htonl(INADDR_BROADCAST),
            };

            /* Send minimal announcement (just magic + status) */
            uint8_t announce[8];
            *(uint32_t *)announce = STREAM_MAGIC;
            announce[4] = DATA_TYPE_DISCOVERY;
            announce[5] = s_fpga_status;
            announce[6] = FW_VERSION_MAJOR;
            announce[7] = FW_VERSION_MINOR;

            sendto(sock, announce, sizeof(announce), 0,
                   (struct sockaddr *)&bcast_addr, sizeof(bcast_addr));
        }
    }

    close(sock);
    vTaskDelete(NULL);
}

/*============================================================================
 * LED Status Task
 *============================================================================*/

/**
 * @brief LED status indicator task
 *
 * Controls the status LED to indicate system state:
 *   - Slow blink (1 Hz):  Idle, waiting for connection
 *   - Fast blink (4 Hz):  Client connected, not streaming
 *   - Solid on:           Actively streaming data
 *   - Double blink:       Error condition
 */
static void status_task(void *pvParameters)
{
    /* Configure LED GPIO */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_STATUS_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    int blink_count = 0;

    while (1) {
        EventBits_t bits = xEventGroupGetBits(s_event_group);

        if (bits & STREAMING_BIT) {
            /* Solid on while streaming */
            gpio_set_level(LED_STATUS_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
        } else if (bits & CLIENT_CONNECTED_BIT) {
            /* Fast blink when client connected */
            gpio_set_level(LED_STATUS_GPIO, (blink_count / 2) % 2);
            vTaskDelay(pdMS_TO_TICKS(125));
        } else if (bits & WIFI_STARTED_BIT) {
            /* Slow blink when WiFi ready */
            gpio_set_level(LED_STATUS_GPIO, (blink_count / 4) % 2);
            vTaskDelay(pdMS_TO_TICKS(250));
        } else {
            /* Very slow blink during init */
            gpio_set_level(LED_STATUS_GPIO, (blink_count / 8) % 2);
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        blink_count++;

        /* Periodically log status */
        if (blink_count % 40 == 0) {
            ESP_LOGI(TAG, "Status: clients=%d, tx_pkts=%lu, tx_bytes=%llu, dropped=%lu",
                     s_num_clients, (unsigned long)s_total_tx_packets,
                     (unsigned long long)s_total_tx_bytes,
                     (unsigned long)s_dropped_packets);
        }
    }
}

/*============================================================================
 * OTA Update Support
 *============================================================================*/

/**
 * @brief Initialize OTA update handling
 *
 * Checks the boot partition and validates the running firmware.
 * Marks the current firmware as valid if booting from an OTA partition.
 */
static void ota_init(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;

    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            ESP_LOGI(TAG, "OTA: Marking firmware as valid");
            esp_ota_mark_app_valid_cancel_rollback();
        }
    }

    ESP_LOGI(TAG, "Running from partition: %s (addr=0x%lx)",
             running->label, (unsigned long)running->address);
}

/*============================================================================
 * Main Entry Point
 *============================================================================*/

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "SAR Radar WiFi Bridge v%d.%d.%d",
             FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH);
    ESP_LOGI(TAG, "ESP32-C3 | ESP-IDF %s", esp_get_idf_version());
    ESP_LOGI(TAG, "========================================");

    /*--------------------------------------------------------------------
     * Initialize NVS (required for WiFi)
     *--------------------------------------------------------------------*/
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition needs erase");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    /*--------------------------------------------------------------------
     * Initialize FreeRTOS synchronization primitives
     *--------------------------------------------------------------------*/
    s_event_group = xEventGroupCreate();
    s_data_mutex = xSemaphoreCreateMutex();
    s_client_mutex = xSemaphoreCreateMutex();

    /* Initialize client socket list */
    for (int i = 0; i < STREAM_MAX_CLIENTS; i++) {
        s_client_sockets[i] = -1;
    }

    /*--------------------------------------------------------------------
     * Load configuration
     *--------------------------------------------------------------------*/
    config_load(&s_config);

    /*--------------------------------------------------------------------
     * Initialize OTA support
     *--------------------------------------------------------------------*/
    ota_init();

    /*--------------------------------------------------------------------
     * Initialize SPI slave for FPGA communication
     *--------------------------------------------------------------------*/
    ESP_ERROR_CHECK(spi_slave_init());

    /*--------------------------------------------------------------------
     * Initialize WiFi in AP mode
     *--------------------------------------------------------------------*/
    ESP_ERROR_CHECK(wifi_init_ap(&s_config));

    /*--------------------------------------------------------------------
     * Create tasks
     *--------------------------------------------------------------------*/
    ESP_LOGI(TAG, "Creating tasks...");

    /* SPI receive task (highest priority) */
    xTaskCreate(spi_rx_task, "spi_rx", SPI_RX_TASK_STACK,
                NULL, SPI_RX_TASK_PRIO, NULL);

    /* TCP server task */
    xTaskCreate(tcp_server_task, "tcp_srv", TCP_SERVER_STACK,
                NULL, TCP_SERVER_PRIO, NULL);

    /* UDP discovery task */
    if (s_config.discovery_enabled) {
        xTaskCreate(udp_discovery_task, "udp_disc", UDP_DISC_STACK,
                    NULL, UDP_DISC_PRIO, NULL);
    }

    /* LED status task */
    xTaskCreate(status_task, "status", STATUS_TASK_STACK,
                NULL, STATUS_TASK_PRIO, NULL);

    /*--------------------------------------------------------------------
     * BLE configuration service (optional)
     *--------------------------------------------------------------------*/
    /* TODO: Initialize BLE configuration service
     * ble_config_init(&s_config);
     *
     * The BLE service allows a mobile app to:
     * - Change WiFi SSID and password
     * - View device status
     * - Trigger scan start/stop
     * - Initiate OTA update
     */

    ESP_LOGI(TAG, "All tasks created. System ready.");
    ESP_LOGI(TAG, "Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());

    /*--------------------------------------------------------------------
     * Main loop: watchdog and periodic maintenance
     *--------------------------------------------------------------------*/
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));  /* 10 second watchdog interval */

        /* Log system health */
        ESP_LOGI(TAG, "Heap free: %lu, min: %lu",
                 (unsigned long)esp_get_free_heap_size(),
                 (unsigned long)esp_get_minimum_free_heap_size());

        /* TODO: Implement watchdog timer feed
         * TODO: Monitor task stack high water marks
         * TODO: Implement automatic WiFi channel scan for best channel
         */
    }
}
