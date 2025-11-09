#ifndef __DW1000_CONFIG_H__
#define __DW1000_CONFIG_H__

#include "dw1000.h"
#include "deca_device_api.h"
#include "deca_regs.h"

// static dwt_config_t config = {
//     2,               /* Channel number. */
//     DWT_PRF_64M,     /* Pulse repetition frequency. */
//     DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
//     DWT_PAC64,       /* Preamble acquisition chunk size. Used in RX only. */
//     10,               /* TX preamble code. Used in TX only. */
//     10,               /* RX preamble code. Used in RX only. */
//     1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
//     DWT_BR_110K,     /* Data rate. */
//     DWT_PHRMODE_STD, /* PHY header mode. */
//     (1024 + 1 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
// };


static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
    DWT_PAC64,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1024 + 1 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

static dwt_txconfig_t txconfig = {
    0xC2,            /* PG delay. */
    0x1f1f1f1f,      /* TX power. */
};

#define RNG_DELAY_MS 100

#define ANT_DLY 16470
#define TX_ANT_DLY ANT_DLY
#define RX_ANT_DLY ANT_DLY

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536
#define SPEED_OF_LIGHT 299702547

// #define POLL_RX_TO_RESP_TX_DLY_UUS 6000
// #define RESP_TX_TO_FINAL_RX_DLY_UUS 5000 // rx delay
// #define FINAL_RX_TIMEOUT_UUS 60000

// #define POLL_TX_TO_RESP_RX_DLY_UUS 3000 // rx delay
// #define RESP_RX_TO_FINAL_TX_DLY_UUS 10000
// #define RESP_RX_TIMEOUT_UUS 60000

#define RX_BUF_LEN 128
#define TX_BUF_LEN 128

// ### ping ###
// #define PING_RX_TIMEOUT_UUS 60000

// for int test
#define PING_RX_TIMEOUT_UUS 30000

// ### range ###
// //only polling
// #define RANGE_RESP_TX_DELAY_UUS     6000 // resp tx delay
// #define RANGE_FINAL_TX_DELAY_UUS    6000 // final tx delay

// #define RANGE_RESP_RX_TIMEOUT_UUS   30000 // resp rx timeout
// #define RANGE_FINAL_RX_TIMEOUT_UUS  30000 // final rx timeout


// int test 2000:x 3000:OK 
#define RANGE_RESP_TX_DELAY_UUS     5000 // resp tx delay
#define RANGE_FINAL_TX_DELAY_UUS    5000 // final tx delay

#define RANGE_RESP_RX_TIMEOUT_UUS   30000 // resp rx timeout
#define RANGE_FINAL_RX_TIMEOUT_UUS  30000 // final rx timeout


// //polling
// #define RANGE_RESP_TX_DELAY_UUS     60000UL // 30000 // resp tx delay
// #define RANGE_FINAL_TX_DELAY_UUS    60000UL // 30000 // final tx delay

// #define RANGE_RESP_RX_TIMEOUT_UUS   65000 // resp rx timeout
// #define RANGE_FINAL_RX_TIMEOUT_UUS  65000 // final rx timeout

#endif // __DW1000_CONFIG_H__