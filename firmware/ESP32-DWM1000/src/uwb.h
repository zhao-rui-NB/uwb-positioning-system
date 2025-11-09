#ifndef __UWB_H__
#define __UWB_H__

#ifdef __cplusplus
extern "C" {
#endif

// #include <stdint.h>

#include "deca_device_api.h"
#include "deca_regs.h"

#include "dw1000.h"
#include "dw1000_config.h"

#include "power.h"
#include "safe_print.h"

#include "system_config.h"



typedef struct __attribute__((packed)) {
    uint16_t group_id;
    uint16_t src_id;
    uint16_t dest_id;
    uint8_t seq_num;
    uint8_t msg_type;
} uwb_common_header_t;


typedef struct __attribute__((packed)) {
    uwb_common_header_t header;
    // no payload
    uint16_t crc;
} uwb_pkt_ping_req_t;
    
typedef struct __attribute__((packed)) {
    uwb_common_header_t header;
    uint8_t system_state;
    uint16_t voltage_mv;
    uint16_t crc;
} uwb_pkt_ping_resp_t;


// the packet that send by master anchor to trigger which node to range
typedef struct __attribute__((packed)) {
    uwb_common_header_t header;
    uint16_t target_node_id;
    uint16_t crc;
} uwb_pkt_range_trigger_t;

typedef struct __attribute__((packed)) {
    uwb_common_header_t header;
    // no payload
    uint16_t crc;
} uwb_pkt_range_poll_t;

typedef struct __attribute__((packed)) {
    uwb_common_header_t header;
    // no payload
    uint16_t crc;
} uwb_pkt_range_resp_t;

typedef struct __attribute__((packed)) {
    uwb_common_header_t header;
    uint32_t poll_tx_ts;
    uint32_t resp_rx_ts;
    uint32_t final_tx_ts;
    uint16_t crc;
} uwb_pkt_range_final_t;

typedef struct __attribute__((packed)) {
    uwb_common_header_t header;
    uint16_t node_a_id;
    uint16_t node_b_id;
    uint16_t distance_cm;
    int16_t rssi_centi_dbm;
    uint16_t crc;
} uwb_pkt_range_report_t;

// UWB message types
typedef enum {
    UWB_MSG_TYPE_PING_REQ = 0x01, //
    UWB_MSG_TYPE_PING_RESP = 0x02,
    UWB_MSG_TYPE_RANGE_TRIGGER = 0x11, //
    UWB_MSG_TYPE_RANGE_POLL = 0x12, //
    UWB_MSG_TYPE_RANGE_RESP = 0x13,
    UWB_MSG_TYPE_RANGE_FINAL = 0x14,
    UWB_MSG_TYPE_RANGE_REPORT = 0x15
} uwb_msg_type_t;

// UWB states
typedef enum {
    UWB_STATE_IDLE = 0,
    UWB_STATE_WAIT_PING_RESP = 1,
    UWB_STATE_WAIT_RANGE_RESP = 2,
    UWB_STATE_WAIT_RANGE_FINAL = 3,
    UWB_STATE_WAIT_RANGE_REPORT = 4,
} uwb_state_t;

// UWB event types
typedef enum {
    UWB_EVENT_PING_RESP_TIMEOUT,
    UWB_EVENT_RANGE_RESP_TIMEOUT,
    UWB_EVENT_RANGE_FINAL_TIMEOUT,
    UWB_EVENT_RANGE_REPORT_TIMEOUT,

    UWB_EVENT_UNKNOWN_FRAME_TIMEOUT,
    UWB_EVENT_UNKNOWN_FRAME_ERROR,
    UWB_EVENT_INVALID_FRAME_RECEIVED
} uwb_event_t;


extern uint16_t uwb_group_id;
extern uint16_t uwb_node_id;
// for the ui display
extern bool uwb_node_is_anchor;
extern bool uwb_node_is_tag;
extern int uwb_node_id_int;



extern uint8_t uwb_state;

// ping resp results
extern bool ping_resp_received;
extern unsigned long ping_resp_ts;
extern uint16_t ping_resp_node_id;
extern uint8_t ping_resp_system_state;
extern uint16_t ping_resp_voltage_mv;

// range report results
extern bool range_report_received;
extern unsigned long range_report_ts;
extern uint16_t range_report_node_a_id;
extern uint16_t range_report_node_b_id;
extern float range_report_distance_m;
extern float range_report_rssi_dbm;

// range final results
extern bool range_final_received;
extern unsigned long range_final_ts;
extern uint16_t range_final_node_a_id;
extern uint16_t range_final_node_b_id;
extern float range_final_distance_m;
extern float range_final_rssi_dbm;


uint16_t get_uwb_group_id();
void set_uwb_group_id(uint16_t group_id);



// ui only support int type, so need to sync when set/get
// node high byte is role: 0x00=tag, 0xff=anchor
// node low byte is unique id in the role
uint16_t get_uwb_node_id();
void set_uwb_node_id(uint16_t node_id);
void sync_uwb_node_id_ui_to_uint16();

void uwb_register_event_callback(void (*cb)(uwb_event_t event, void *data));

void uwb_init();
uint64_t get_tx_timestamp();
uint64_t get_rx_timestamp();
bool uwb_check_frame_valid(uint8_t *buf, uint32_t len);
void print_rx_err_flags(uint32_t status_reg);


// SEND FUNCTIONS
uint8_t uwb_send_ping_req(uint16_t dest_id);
uint8_t uwb_send_range_trigger(uint16_t initiator_id, uint16_t responder_id);


// HANDLE FUNCTIONS 
void uwb_handle_ping_req(uwb_pkt_ping_req_t *pkt);
void uwb_handle_ping_resp(uwb_pkt_ping_resp_t *pkt);
void uwb_handle_range_trigger(uwb_pkt_range_trigger_t *pkt);
void uwb_handle_range_poll(uwb_pkt_range_poll_t *pkt);
void uwb_handle_range_resp(uwb_pkt_range_resp_t *pkt);
void uwb_handle_range_final(uwb_pkt_range_final_t *pkt);
void uwb_handle_range_report(uwb_pkt_range_report_t *pkt);
void uwb_process();

void uwb_task_init();
void uwb_task(void *pvParameters);
void IRAM_ATTR uwb_irq_handler();



void uwb_process_polling();
void uwb_process_polling_irq();




#ifdef __cplusplus
}
#endif

#endif // __UWB_H__