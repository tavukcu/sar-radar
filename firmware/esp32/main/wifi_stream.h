/**
 * @file wifi_stream.h
 * @brief WiFi Streaming Protocol Definitions for SAR Radar
 *
 * Defines the packet format, data types, and configuration structures
 * for streaming radar data from the ESP32-C3 WiFi bridge to a host
 * computer. The protocol is designed for simplicity and low overhead
 * to maximize data throughput over WiFi.
 *
 * Protocol Overview:
 *   - TCP stream on port 5000 for reliable data transfer
 *   - UDP broadcast on port 5001 for device discovery
 *   - Simple packet format: header + payload
 *   - No compression (CPU limited on ESP32-C3)
 *
 * Packet Format:
 *   [4B magic] [4B sequence] [2B type] [2B length] [NB payload] [2B checksum]
 *
 * @author SAR Radar Project
 * @date   2026-02-21
 */

#ifndef WIFI_STREAM_H
#define WIFI_STREAM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * Protocol Constants
 *============================================================================*/

/** Magic number for packet identification ("SARR" in ASCII) */
#define STREAM_MAGIC            0x53415252

/** Protocol version */
#define STREAM_PROTOCOL_VERSION 0x0001

/** TCP data streaming port */
#define STREAM_TCP_PORT         5000

/** UDP discovery broadcast port */
#define STREAM_UDP_PORT         5001

/** Maximum payload size per packet (bytes) */
#define STREAM_MAX_PAYLOAD      4096

/** Maximum number of simultaneous TCP clients */
#define STREAM_MAX_CLIENTS      2

/** WiFi AP default settings */
#define WIFI_AP_SSID_PREFIX     "SAR-RADAR-"
#define WIFI_AP_PASSWORD        "sarradar2026"
#define WIFI_AP_CHANNEL         6
#define WIFI_AP_MAX_CONN        4

/** BLE configuration service UUID */
#define BLE_SERVICE_UUID        0x1234
#define BLE_CHAR_SSID_UUID      0x1235
#define BLE_CHAR_PASS_UUID      0x1236
#define BLE_CHAR_STATUS_UUID    0x1237

/** SPI transfer buffer sizes */
#define SPI_RX_BUF_SIZE         4096
#define SPI_TX_BUF_SIZE         256
#define DOUBLE_BUF_COUNT        2

/** LED GPIO pin */
#define LED_STATUS_GPIO         8

/** OTA partition label */
#define OTA_PARTITION_LABEL     "ota_0"

/*============================================================================
 * Data Type Enumeration
 *============================================================================*/

/**
 * @brief Packet data type identifiers
 *
 * Each packet carries a type field indicating the nature of the payload.
 * The host uses this to route data to the appropriate processing pipeline.
 */
typedef enum {
    /** Raw radar frame data (ADC samples, 4 channels) */
    DATA_TYPE_RADAR_FRAME   = 0x0001,

    /** FFT-processed range bins (complex I/Q) */
    DATA_TYPE_RANGE_FFT     = 0x0002,

    /** IMU measurement data (6-axis + timestamp) */
    DATA_TYPE_IMU_DATA      = 0x0003,

    /** System status information */
    DATA_TYPE_STATUS        = 0x0004,

    /** Device discovery response (UDP) */
    DATA_TYPE_DISCOVERY     = 0x0005,

    /** Configuration command (from host) */
    DATA_TYPE_CONFIG_CMD    = 0x0006,

    /** Configuration response (to host) */
    DATA_TYPE_CONFIG_RESP   = 0x0007,

    /** OTA firmware update data */
    DATA_TYPE_OTA_DATA      = 0x0008,

    /** Heartbeat / keepalive */
    DATA_TYPE_HEARTBEAT     = 0x00FF,
} stream_data_type_t;

/*============================================================================
 * Packet Header Structure
 *============================================================================*/

/**
 * @brief Stream packet header (14 bytes)
 *
 * Every packet begins with this header, followed by the payload data
 * and a 16-bit Fletcher checksum.
 *
 * Wire format (little-endian):
 *   Offset  Size  Field
 *   0       4     magic (0x53415252)
 *   4       4     sequence number
 *   8       2     data type (stream_data_type_t)
 *   10      2     payload length (bytes, excludes header and checksum)
 *   12      2     reserved (protocol version)
 *   14      N     payload data
 *   14+N    2     Fletcher-16 checksum over header + payload
 */
typedef struct __attribute__((packed)) {
    uint32_t magic;          /**< Magic number (STREAM_MAGIC) */
    uint32_t sequence;       /**< Monotonically increasing sequence number */
    uint16_t data_type;      /**< Payload type (stream_data_type_t) */
    uint16_t payload_length; /**< Payload size in bytes */
    uint16_t version;        /**< Protocol version */
} stream_packet_header_t;

/*============================================================================
 * Payload Structures
 *============================================================================*/

/**
 * @brief Radar frame payload
 *
 * Contains raw ADC samples or FFT output for one chirp.
 * Channel data is interleaved: [ch0_sample0, ch1_sample0, ..., ch3_sample0,
 *                                ch0_sample1, ...]
 */
typedef struct __attribute__((packed)) {
    uint32_t frame_number;       /**< Frame sequence number */
    uint16_t chirp_number;       /**< Chirp index within frame */
    uint16_t num_samples;        /**< Number of samples per channel */
    uint8_t  num_channels;       /**< Number of channels (typically 4) */
    uint8_t  sample_bits;        /**< Bits per sample (12 for ADC, 16 for FFT) */
    uint16_t reserved;           /**< Alignment padding */
    /* Followed by sample data: num_samples * num_channels * ceil(sample_bits/8) bytes */
} radar_frame_header_t;

/**
 * @brief IMU data payload
 *
 * Single IMU measurement with timestamp. All values are raw sensor
 * readings (signed 16-bit) that need host-side scaling:
 *   - Accelerometer: value * (16.0 / 32768.0) = g
 *   - Gyroscope:     value * (2000.0 / 32768.0) = deg/s
 */
typedef struct __attribute__((packed)) {
    uint64_t timestamp_us;       /**< Timestamp in microseconds (FPGA clock) */
    int16_t  accel_x;            /**< Accelerometer X (raw) */
    int16_t  accel_y;            /**< Accelerometer Y (raw) */
    int16_t  accel_z;            /**< Accelerometer Z (raw) */
    int16_t  gyro_x;             /**< Gyroscope X (raw) */
    int16_t  gyro_y;             /**< Gyroscope Y (raw) */
    int16_t  gyro_z;             /**< Gyroscope Z (raw) */
} imu_data_payload_t;

/**
 * @brief System status payload
 *
 * Periodic status update sent at ~1 Hz.
 */
typedef struct __attribute__((packed)) {
    uint8_t  system_state;       /**< FPGA state machine state */
    uint8_t  scan_active;        /**< Scanning flag */
    uint8_t  lvds_aligned;       /**< LVDS alignment status */
    uint8_t  sd_status;          /**< SD card status bits */
    uint32_t frame_count;        /**< Total frames captured */
    uint32_t dropped_packets;    /**< Dropped packet count */
    uint16_t fpga_buf_usage;     /**< FPGA buffer usage (bytes) */
    uint16_t wifi_buf_usage;     /**< WiFi buffer usage (bytes) */
    int8_t   wifi_rssi;          /**< WiFi RSSI (dBm) */
    uint8_t  battery_percent;    /**< Battery level (0-100, 0xFF=unknown) */
    uint16_t temperature;        /**< Board temperature (0.1 deg C units) */
} status_payload_t;

/**
 * @brief Discovery response payload (UDP broadcast)
 *
 * Sent in response to a discovery request broadcast, or periodically
 * as an announcement.
 */
typedef struct __attribute__((packed)) {
    char     device_name[16];    /**< Human-readable device name */
    uint8_t  mac_address[6];     /**< WiFi MAC address */
    uint16_t tcp_port;           /**< TCP streaming port */
    uint16_t protocol_version;   /**< Supported protocol version */
    uint8_t  hw_version;         /**< Hardware revision */
    uint8_t  fw_version_major;   /**< Firmware major version */
    uint8_t  fw_version_minor;   /**< Firmware minor version */
    uint8_t  fw_version_patch;   /**< Firmware patch version */
    uint8_t  status;             /**< Current system status */
    uint8_t  reserved[3];        /**< Padding */
} discovery_payload_t;

/**
 * @brief Configuration command payload
 *
 * Configuration commands from host to device.
 */
typedef struct __attribute__((packed)) {
    uint8_t  config_id;          /**< Configuration parameter ID */
    uint8_t  reserved;
    uint16_t value_length;       /**< Length of value data */
    /* Followed by value_length bytes of configuration data */
} config_cmd_payload_t;

/** Configuration parameter IDs */
typedef enum {
    CONFIG_WIFI_SSID       = 0x01,
    CONFIG_WIFI_PASSWORD   = 0x02,
    CONFIG_WIFI_CHANNEL    = 0x03,
    CONFIG_STREAM_RATE     = 0x04,
    CONFIG_DATA_FORMAT     = 0x05,
    CONFIG_START_SCAN      = 0x10,
    CONFIG_STOP_SCAN       = 0x11,
    CONFIG_REBOOT          = 0xFE,
    CONFIG_FACTORY_RESET   = 0xFF,
} config_param_id_t;

/*============================================================================
 * Runtime Configuration
 *============================================================================*/

/**
 * @brief WiFi streaming runtime configuration
 *
 * Stored in NVS (non-volatile storage) and modifiable via BLE or TCP.
 */
typedef struct {
    char     wifi_ssid[32];      /**< WiFi AP SSID */
    char     wifi_password[64];  /**< WiFi AP password */
    uint8_t  wifi_channel;       /**< WiFi channel (1-13) */
    bool     stream_enabled;     /**< Enable TCP streaming */
    bool     discovery_enabled;  /**< Enable UDP discovery broadcast */
    uint16_t stream_data_types;  /**< Bitmask of enabled data types */
    uint32_t stream_rate_hz;     /**< Target stream update rate */
} stream_config_t;

/*============================================================================
 * Function Prototypes
 *============================================================================*/

/**
 * @brief Initialize WiFi in AP mode
 * @param config Pointer to stream configuration
 * @return ESP_OK on success
 */
esp_err_t wifi_stream_init(const stream_config_t *config);

/**
 * @brief Start TCP server and UDP discovery
 * @return ESP_OK on success
 */
esp_err_t wifi_stream_start(void);

/**
 * @brief Stop streaming and close connections
 */
void wifi_stream_stop(void);

/**
 * @brief Send a data packet to all connected TCP clients
 * @param type Data type identifier
 * @param payload Pointer to payload data
 * @param length Payload length in bytes
 * @return Number of bytes sent, or negative on error
 */
int wifi_stream_send(stream_data_type_t type, const void *payload, uint16_t length);

/**
 * @brief Send UDP discovery announcement
 * @return ESP_OK on success
 */
esp_err_t wifi_stream_send_discovery(void);

/**
 * @brief Get current streaming statistics
 * @param[out] tx_bytes Total bytes transmitted
 * @param[out] tx_packets Total packets transmitted
 * @param[out] dropped Total dropped packets
 */
void wifi_stream_get_stats(uint64_t *tx_bytes, uint32_t *tx_packets, uint32_t *dropped);

/**
 * @brief Calculate Fletcher-16 checksum
 * @param data Pointer to data buffer
 * @param length Data length in bytes
 * @return 16-bit Fletcher checksum
 */
uint16_t fletcher16(const uint8_t *data, size_t length);

/**
 * @brief Initialize SPI slave for FPGA communication
 * @return ESP_OK on success
 */
esp_err_t spi_bridge_init(void);

/**
 * @brief Read data from FPGA via SPI
 * @param[out] buffer Output buffer
 * @param length Number of bytes to read
 * @return Number of bytes actually read
 */
int spi_bridge_read(uint8_t *buffer, size_t length);

/**
 * @brief Initialize BLE configuration service
 * @param config Pointer to current configuration
 * @return ESP_OK on success
 */
esp_err_t ble_config_init(stream_config_t *config);

/**
 * @brief Initialize OTA update handler
 * @return ESP_OK on success
 */
esp_err_t ota_update_init(void);

/**
 * @brief Process OTA update data chunk
 * @param data Pointer to OTA data
 * @param length Data length
 * @return ESP_OK on success, ESP_ERR_OTA_* on error
 */
esp_err_t ota_update_process(const uint8_t *data, size_t length);

/**
 * @brief Finalize OTA update and reboot
 * @return ESP_OK on success (does not return on success - reboots)
 */
esp_err_t ota_update_finish(void);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_STREAM_H */
