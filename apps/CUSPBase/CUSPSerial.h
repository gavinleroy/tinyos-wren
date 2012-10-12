
#ifndef TEST_SERIAL_H
#define TEST_SERIAL_H

// structure of incoming rssi measurements
typedef nx_struct rssi_msg {
    nx_uint16_t counter;
    nx_uint16_t src;
    nx_uint32_t localtime;
    nx_uint32_t globaltime; 
} rssi_msg_t;

// structure of incoming serial messages
typedef nx_struct cmd_serial_msg {
  nx_uint16_t cmd;
  nx_uint8_t channel;
  nx_uint16_t dst;
} cmd_serial_msg_t;

// structure of outgoing serial messages
typedef nx_struct rssi_serial_msg {
    nx_uint16_t counter;
    nx_uint16_t dst;
    nx_int8_t rssi;
    nx_uint16_t src;
    nx_uint32_t srclocaltime;
    nx_uint32_t srcglobaltime;
    nx_uint32_t localtime;
    nx_uint32_t globaltime;
    nx_uint8_t isSynced;
    nx_uint8_t reboot;
    nx_uint16_t bat;
    nx_uint32_t size;
} rssi_serial_msg_t;

typedef nx_struct serial_status_msg {
    nx_uint16_t src;
    nx_uint8_t sensing;
    nx_uint32_t localtime;
    nx_uint32_t globaltime;
    nx_uint32_t buffersize;
    nx_uint8_t isSynced;
    nx_uint16_t reboots;
    nx_uint16_t bat;
    nx_uint8_t isErased;
    nx_uint8_t download;
    nx_uint8_t channel;
} serial_status_msg_t;

typedef nx_struct wren_status_msg {
    nx_uint16_t src;
    nx_uint8_t sensing;
    nx_uint32_t buffersize;
} wren_status_msg_t;

typedef nx_struct wren_connection_msg {
    nx_uint16_t src;
    nx_uint16_t dst;
    nx_uint16_t cmd;
    nx_uint32_t logsize;
    nx_uint8_t channel;
    nx_uint8_t isAcked;
} wren_connection_msg_t;

typedef nx_struct wren_swp_msg {
    nx_uint32_t seqNumToAck;
} wren_swp_msg_t;

typedef nx_struct base_status_msg {
    nx_uint16_t src;
    nx_uint32_t localtime;
    nx_uint32_t globaltime;
    nx_uint8_t isSynced;
    nx_uint8_t channel;
} base_status_msg_t;

enum {
  AM_CMD_MSG = 0x80,
  AM_CMD_SERIAL_MSG = 0x89,
  AM_RSSI_MSG = 0x6,
  AM_RSSI_SERIAL_MSG = 0x90,
  AM_SERIAL_STATUS_MSG = 0x7,
  AM_BASE_STATUS_MSG = 0x8,
  AM_WREN_STATUS_MSG = 0x70,
  AM_BASE_MSG = 0x60,
  AM_HANDSHAKE_MSG = 0x50,
  AM_SWP_MSG = 0x40,
};

enum {
    CMD_DOWNLOAD        = 0,
    CMD_ERASE           = 1,
    CMD_START_SENSE     = 2,
    CMD_STOP_SENSE      = 3,
    CMD_STATUS          = 4,
    CMD_START_BLINK     = 5,
    CMD_STOP_BLINK      = 6,
    CMD_LOGSYNC         = 7,
    CMD_BASESTATUS      = 8,
    CMD_NONE            = 9,
    CMD_CHANNEL         = 10,
    CMD_CHANNEL_RESET   = 11,
    CMD_WREN_STATUS     = 12,
};

enum {
    ENTRY_EMPTY = 0,
    ENTRY_FULL = 1,
};

typedef struct FrameItem
{
    uint8_t     state;
    uint32_t    frameno;
    uint32_t    timeout;
} FrameItem;
#endif
