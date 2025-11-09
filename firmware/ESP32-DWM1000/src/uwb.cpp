#include <Arduino.h>
#include <SPI.h>
#include "freertos/semphr.h"

#include "uwb.h"



static SemaphoreHandle_t uwb_isr_sem;

uint8_t uwb_state = UWB_STATE_IDLE;

uint8_t seq_num;
uint16_t uwb_group_id;
uint16_t uwb_node_id;

// in ui uwb_node_id split to 2 boolean and 1 int
bool uwb_node_is_anchor = false;
bool uwb_node_is_tag = true;
int uwb_node_id_int = 0;


uint8_t rx_buffer[RX_BUF_LEN];
uint8_t tx_buffer[TX_BUF_LEN];

uint32_t poll_rx_ts_presave;


// ++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++ result storage and flags ++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++

// ping resp result
bool ping_resp_received = false;
unsigned long ping_resp_ts;
uint16_t ping_resp_node_id;
uint8_t ping_resp_system_state;
uint16_t ping_resp_voltage_mv;

// range result
bool range_report_received = false;
unsigned long range_report_ts;
uint16_t range_report_node_a_id;
uint16_t range_report_node_b_id;
float range_report_distance_m;
float range_report_rssi_dbm;

// range final 
bool range_final_received = false;
unsigned long range_final_ts;
uint16_t range_final_node_a_id;
uint16_t range_final_node_b_id;
float range_final_distance_m;
float range_final_rssi_dbm;



uint16_t get_uwb_group_id() {
    return uwb_group_id;
}
void set_uwb_group_id(uint16_t group_id){
    uwb_group_id = group_id;
    save_uwb_group_id();
}

// ui only support int type, so need to sync when set/get
// node high byte is role: 0x00=tag, 0xff=anchor
// node low byte is unique id in the role
uint16_t get_uwb_node_id(){
    return uwb_node_id;
}
void set_uwb_node_id(uint16_t node_id){
    uwb_node_id = node_id;
    // uint16 to int
    if (uwb_node_id & 0xFF00){
        uwb_node_is_anchor = true;
        uwb_node_is_tag = false;
    } else {
        uwb_node_is_anchor = false;
        uwb_node_is_tag = true;
    }
    uwb_node_id_int = uwb_node_id & 0x00FF;

    save_uwb_node_id();
}

void sync_uwb_node_id_ui_to_uint16(){
    uwb_node_id = (uwb_node_is_anchor ? 0xFF00 : 0x0000) | (uwb_node_id_int & 0x00FF);
    save_uwb_node_id();
}



static void (*_uwb_event_callback)(uwb_event_t event, void *data) = NULL;


void uwb_register_event_callback(void (*cb)(uwb_event_t event, void *data)){
    _uwb_event_callback = cb;
}

// QueueHandle_t log_queue;


static void rx_ok_cb(const dwt_cb_data_t *cb_data) {
    memset(rx_buffer, 0, RX_BUF_LEN);

    uint32_t frame_len = cb_data->datalength;
    
    if (frame_len <= RX_BUF_LEN){
        dwt_readrxdata(rx_buffer, frame_len, 0);
    }

    uint8_t rx_frame_valid = uwb_check_frame_valid(rx_buffer, frame_len);
    if(rx_frame_valid) {
        uwb_common_header_t *hdr = (uwb_common_header_t *)rx_buffer;
        switch (hdr->msg_type){
            case UWB_MSG_TYPE_PING_REQ: uwb_handle_ping_req((uwb_pkt_ping_req_t *)hdr); break;
            case UWB_MSG_TYPE_PING_RESP: uwb_handle_ping_resp((uwb_pkt_ping_resp_t *)hdr);  break;

            case UWB_MSG_TYPE_RANGE_TRIGGER: uwb_handle_range_trigger((uwb_pkt_range_trigger_t *)hdr); break;
            
            case UWB_MSG_TYPE_RANGE_POLL: uwb_handle_range_poll((uwb_pkt_range_poll_t *)hdr); break;
            case UWB_MSG_TYPE_RANGE_RESP: uwb_handle_range_resp((uwb_pkt_range_resp_t *)hdr); break;
            case UWB_MSG_TYPE_RANGE_FINAL: uwb_handle_range_final((uwb_pkt_range_final_t *)hdr); break;
            case UWB_MSG_TYPE_RANGE_REPORT: uwb_handle_range_report((uwb_pkt_range_report_t *)hdr); break;
            default:
                safe_printf("[rx_ok_cb] rx frame valid but not found handler for msg_type=0x%02X\n", hdr->msg_type);
        }
    }
    else {
        // safe_printf("[rx_ok_cb] Received frame is invalid\n");
        if (_uwb_event_callback) {
            _uwb_event_callback(UWB_EVENT_INVALID_FRAME_RECEIVED, NULL);
        }

        // go to idle state
        uwb_state = UWB_STATE_IDLE;
        dwt_rxreset();
        dwt_setrxtimeout(0);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
}

static void rx_to_cb(const dwt_cb_data_t *cb_data) {
    if(uwb_state != UWB_STATE_IDLE) {
        // timeout occurred while waiting for a response
        safe_printf("[rx_to_cb] Timeout, uwb_state: %d\n", uwb_state);
        
        if (_uwb_event_callback) {
            uint8_t event;
            switch (uwb_state) {
                case UWB_STATE_WAIT_PING_RESP: event = UWB_EVENT_PING_RESP_TIMEOUT; break;
                case UWB_STATE_WAIT_RANGE_RESP: event = UWB_EVENT_RANGE_RESP_TIMEOUT; break;
                case UWB_STATE_WAIT_RANGE_FINAL: event = UWB_EVENT_RANGE_FINAL_TIMEOUT; break;
                case UWB_STATE_WAIT_RANGE_REPORT: event = UWB_EVENT_RANGE_REPORT_TIMEOUT; break;
                default: event = UWB_EVENT_UNKNOWN_FRAME_TIMEOUT; break;
            }
            _uwb_event_callback((uwb_event_t)event, NULL);
        }
        print_rx_err_flags(cb_data->status);
    }
    
    // go to idle state
    uwb_state = UWB_STATE_IDLE;
    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

}

static void rx_err_cb(const dwt_cb_data_t *cb_data) {
    print_rx_err_flags(cb_data->status);
    if(uwb_state != UWB_STATE_IDLE) {
        safe_printf("[rx_err_cb] RX error occurred in state %d, %08X\n", uwb_state, cb_data->status);

        if (_uwb_event_callback) {
            uint8_t event;
            switch (uwb_state) {
                case UWB_STATE_WAIT_PING_RESP: event = UWB_EVENT_PING_RESP_TIMEOUT; break;
                case UWB_STATE_WAIT_RANGE_RESP: event = UWB_EVENT_RANGE_RESP_TIMEOUT; break;
                case UWB_STATE_WAIT_RANGE_FINAL: event = UWB_EVENT_RANGE_FINAL_TIMEOUT; break;
                case UWB_STATE_WAIT_RANGE_REPORT: event = UWB_EVENT_RANGE_REPORT_TIMEOUT; break;
                default: event = UWB_EVENT_UNKNOWN_FRAME_ERROR; break;
            }
            _uwb_event_callback((uwb_event_t)event, NULL);
        }
    }
    
    // go to idle state
    uwb_state = UWB_STATE_IDLE;
    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

}

static void tx_conf_cb(const dwt_cb_data_t *cb_data) {
}



void uwb_init(){
    init_DW1000();

    uint32_t devid = dwt_readdevid();
    if (devid != 0xDECA0130) {
        safe_printf("[ERROR] Not a DW1000 device!\n");
        while (1);
    }

    port_set_dw1000_slowrate();
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR){
        safe_printf("[ERROR] dw1000 INIT FAILED\n");
        while (1){}
    }
    port_set_dw1000_fastrate();

    dwt_configure(&config);
    dwt_setsmarttxpower(0);
    dwt_configuretxrf(&txconfig);

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);


    // Register RX call-back.
    dwt_setcallbacks(NULL, &rx_ok_cb, &rx_to_cb, &rx_err_cb);
    // Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors).
    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);
    // enable interrupts
    decamutexoff(true); // enable interrupts

    dwt_rxreset(); 
    dwt_setpreambledetecttimeout(0);
    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

}

uint64_t get_tx_timestamp() {
    uint8_t ts_buf[5];
    dwt_readtxtimestamp(ts_buf);
    uint64_t ts = 
        ((uint64_t)ts_buf[0]) |
        ((uint64_t)ts_buf[1] << 8) |
        ((uint64_t)ts_buf[2] << 16) |
        ((uint64_t)ts_buf[3] << 24) |
        ((uint64_t)ts_buf[4] << 32);
    return ts;
}

uint64_t get_rx_timestamp(){
    uint8_t ts_buf[5];
    dwt_readrxtimestamp(ts_buf);
    uint64_t ts = 
        ((uint64_t)ts_buf[0]) |
        ((uint64_t)ts_buf[1] << 8) |
        ((uint64_t)ts_buf[2] << 16) |
        ((uint64_t)ts_buf[3] << 24) |
        ((uint64_t)ts_buf[4] << 32);
    return ts;
}


static inline float fast_log10f(float x) {
    // 以 IEEE754 位元操作近似 log10
    union { float f; uint32_t i; } vx = { x };
    float y = (float)((vx.i >> 23) & 255) - 128.0f;  // exponent 部分
    vx.i = (vx.i & 0x7FFFFF) | 0x3F800000;           // mantissa normalize 到 [1,2)
    float m = vx.f;
    // 線性近似 log10(m) ≈ (m - 1) * 0.2895297
    return (y + (m - 1.0f) * 0.2895297f) * 0.3010299f; // 0.3010299 = log10(2)
}

static float calc_rssi(dwt_rxdiag_t *rxdiag) {
    // user manual 4.7.2

    float C = rxdiag->maxGrowthCIR; // cir_power
    float A = 121.74f; // 113.77 for PRF 16 MHz, 121.74 for 64 MHz
    float N = rxdiag->rxPreamCount; // rx_pacc

    float rssi = 10 * log10( C * 131072.0 / (N*N) ) - A;

    // fast log10
    // float rssi = 10 * fast_log10f( C * 131072.0 / (N*N) ) - A;

    // 找到的修正值
    // https://www.cnblogs.com/tuzhuke/p/12169538.html
    // corrFac = 1.1667;

    if (rssi > -88) {
        rssi = rssi + (rssi + 88) * 1.1667;
    }

    return rssi;
}

bool uwb_check_frame_valid(uint8_t *buf, uint32_t len){
    uwb_common_header_t *hdr = (uwb_common_header_t *)buf;

    if (hdr->group_id != uwb_group_id) {
        return false;
    }
    if (hdr->dest_id != uwb_node_id && hdr->dest_id != 0xFFFF) { // not to me or broadcast
        return false;
    }

    switch (hdr->msg_type) {
        case UWB_MSG_TYPE_PING_REQ:     if (len != sizeof(uwb_pkt_ping_req_t)) return false; break;
        case UWB_MSG_TYPE_PING_RESP:    if (len != sizeof(uwb_pkt_ping_resp_t)) return false; break;
        case UWB_MSG_TYPE_RANGE_TRIGGER: if (len != sizeof(uwb_pkt_range_trigger_t)) return false; break;
        case UWB_MSG_TYPE_RANGE_POLL:   if (len != sizeof(uwb_pkt_range_poll_t)) return false; break;
        case UWB_MSG_TYPE_RANGE_RESP:   if (len != sizeof(uwb_pkt_range_resp_t)) return false; break;
        case UWB_MSG_TYPE_RANGE_FINAL:  if (len != sizeof(uwb_pkt_range_final_t)) return false; break;
        case UWB_MSG_TYPE_RANGE_REPORT: if (len != sizeof(uwb_pkt_range_report_t)) return false; break;
        default: return false;
    }
    
    switch (uwb_state) {
        case UWB_STATE_IDLE:
            if (hdr->msg_type == UWB_MSG_TYPE_PING_REQ ||
                hdr->msg_type == UWB_MSG_TYPE_RANGE_TRIGGER ||
                hdr->msg_type == UWB_MSG_TYPE_RANGE_POLL ||
                hdr->msg_type == UWB_MSG_TYPE_RANGE_REPORT) {
                return true;
            }
            break;
        case UWB_STATE_WAIT_PING_RESP:
            if (hdr->msg_type == UWB_MSG_TYPE_PING_RESP) {
                return true;
            }
            break;
        case UWB_STATE_WAIT_RANGE_RESP:
            if (hdr->msg_type == UWB_MSG_TYPE_RANGE_RESP) {
                return true;
            }
            break;
        case UWB_STATE_WAIT_RANGE_FINAL:
            if (hdr->msg_type == UWB_MSG_TYPE_RANGE_FINAL) {
                return true;
            }
            break;
        case UWB_STATE_WAIT_RANGE_REPORT:
            if (hdr->msg_type == UWB_MSG_TYPE_RANGE_REPORT) {
                return true;
            }
            break;
    }

    safe_printf("[uwb_check_frame_valid] state=%d, msg_type=0x%02X\n", uwb_state, hdr->msg_type);
    safe_printf("[uwb_check_frame_valid] Frame not expected in current UWB state\n");
    return false;
}

void print_rx_err_flags(uint32_t status_reg){
    safe_printf("[print_rx_err_flags] RX error flags: 0x%08X---", status_reg);
    //SYS_STATUS_RXRFTO | SYS_STATUS_RXPTO
    //SYS_STATUS_RXPHE | SYS_STATUS_RXFCE | SYS_STATUS_RXRFSL | SYS_STATUS_RXSFDTO | SYS_STATUS_AFFREJ | SYS_STATUS_LDEERR

    safe_printf("rx_err_flags:");
    if (status_reg & SYS_STATUS_RXRFTO)   safe_printf(" RXRFTO");
    if (status_reg & SYS_STATUS_RXPTO)    safe_printf(" RXPTO");
    if (status_reg & SYS_STATUS_RXPHE)    safe_printf(" RXPHE");
    if (status_reg & SYS_STATUS_RXFCE)    safe_printf(" RXFCE");
    if (status_reg & SYS_STATUS_RXRFSL)   safe_printf(" RXRFSL");
    if (status_reg & SYS_STATUS_RXSFDTO)  safe_printf(" RXSFDTO");
    if (status_reg & SYS_STATUS_LDEERR)   safe_printf(" LDEERR");
    if (status_reg & SYS_STATUS_AFFREJ)   safe_printf(" AFFREJ");
    safe_printf("\n");

}

// SEND FUNCTIONS
uint8_t uwb_send_ping_req(uint16_t dest_id){

    if (uwb_state != UWB_STATE_IDLE) {
        safe_printf("[uwb_send_ping_req] Cannot send PING REQ, UWB not in IDLE state, state: %d\n", uwb_state);
        return false;
    }

    uwb_pkt_ping_req_t *pkt = (uwb_pkt_ping_req_t *)tx_buffer;
    pkt->header.group_id = uwb_group_id;
    pkt->header.src_id = uwb_node_id;
    pkt->header.dest_id = dest_id;
    pkt->header.seq_num = seq_num++;
    pkt->header.msg_type = UWB_MSG_TYPE_PING_REQ;


    // Send the packet
    dwt_writetxdata(sizeof(uwb_pkt_ping_req_t), (uint8_t *)pkt, 0);
    dwt_writetxfctrl(sizeof(uwb_pkt_ping_req_t), 0, 1);
    dwt_forcetrxoff();
    dwt_rxreset();
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(PING_RX_TIMEOUT_UUS);
    int succ = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    if (succ != DWT_SUCCESS) {
        safe_printf("[uwb_send_ping_req] Failed to start TX for PING REQ\n");
        
        uwb_state = UWB_STATE_IDLE;
        dwt_forcetrxoff();
        dwt_rxreset();
        dwt_setrxtimeout(0);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return false;
    }

    uwb_state = UWB_STATE_WAIT_PING_RESP;
    return true;
}

uint8_t uwb_send_range_trigger(uint16_t initiator_id, uint16_t responder_id){
    if (uwb_state != UWB_STATE_IDLE) {
        safe_printf("[uwb_send_range_trigger] Cannot send RANGE TRIGGER, UWB not in IDLE state, state: %d\n", uwb_state);
        return false;
    }

    uwb_pkt_range_trigger_t *pkt = (uwb_pkt_range_trigger_t *)tx_buffer;
    pkt->header.group_id = uwb_group_id;
    pkt->header.src_id = uwb_node_id;
    pkt->header.dest_id = initiator_id;
    pkt->header.seq_num = seq_num++;
    pkt->header.msg_type = UWB_MSG_TYPE_RANGE_TRIGGER;
    pkt->target_node_id = responder_id;

    // Send the packet
    dwt_writetxdata(sizeof(uwb_pkt_range_trigger_t), (uint8_t *)pkt, 0);
    dwt_writetxfctrl(sizeof(uwb_pkt_range_trigger_t), 0, 1);
    
    
    // go to IDLE state after sending RANGE TRIGGER
    uwb_state = UWB_STATE_IDLE;
    dwt_forcetrxoff();
    dwt_rxreset();
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);
    
    int succ = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    if (succ != DWT_SUCCESS) {
        safe_printf("[uwb_send_range_trigger] Failed to start TX for RANGE TRIGGER\n");
        dwt_forcetrxoff();
        dwt_rxreset();
        dwt_setrxtimeout(0);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return false;
    }

    return true;
}

// HANDLE FUNCTIONS
void uwb_handle_ping_req(uwb_pkt_ping_req_t *pkt){
    uwb_pkt_ping_resp_t *resp = (uwb_pkt_ping_resp_t *)tx_buffer;
    resp->header.group_id = uwb_group_id;
    resp->header.src_id = uwb_node_id;
    resp->header.dest_id = pkt->header.src_id;
    resp->header.seq_num = seq_num++;
    resp->header.msg_type = UWB_MSG_TYPE_PING_RESP;
    resp->system_state = millis(); // example system state
    resp->voltage_mv = get_battery_voltage_mv(); 
    
    dwt_writetxdata(sizeof(uwb_pkt_ping_resp_t), (uint8_t *)resp, 0);
    dwt_writetxfctrl(sizeof(uwb_pkt_ping_resp_t), 0, 1);
    
    
    uwb_state = UWB_STATE_IDLE;
    dwt_forcetrxoff();
    dwt_rxreset();
    dwt_setrxtimeout(0);
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    // safe_printf("[uwb_handle_ping_req] Sent PING RESP to node_id=0x%04X\n", pkt->header.src_id);
}

void uwb_handle_ping_resp(uwb_pkt_ping_resp_t *pkt){
    ping_resp_system_state = pkt->system_state;
    ping_resp_voltage_mv = pkt->voltage_mv;
    ping_resp_ts = millis();
    ping_resp_node_id = pkt->header.src_id;
    ping_resp_received = true;
    
    uwb_state = UWB_STATE_IDLE;
    dwt_forcetrxoff();
    dwt_rxreset();
    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    
    // safe_printf("[uwb_handle_ping_resp] device_id: 0x%04X, system_state: 0x%02X, voltage: %d mV\n", pkt->header.src_id, pkt->system_state, pkt->voltage_mv);
}

void uwb_handle_range_trigger(uwb_pkt_range_trigger_t *pkt){
    // send RANGE POLL to responder node
    
    uwb_pkt_range_poll_t *poll_pkt = (uwb_pkt_range_poll_t *)tx_buffer;
    poll_pkt->header.group_id = uwb_group_id;
    poll_pkt->header.src_id = uwb_node_id;
    poll_pkt->header.dest_id = pkt->target_node_id;
    poll_pkt->header.seq_num = seq_num++;
    poll_pkt->header.msg_type = UWB_MSG_TYPE_RANGE_POLL;

    dwt_writetxdata(sizeof(uwb_pkt_range_poll_t), (uint8_t *)poll_pkt, 0);
    dwt_writetxfctrl(sizeof(uwb_pkt_range_poll_t), 0, 1);
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(RANGE_RESP_RX_TIMEOUT_UUS);

    int succ = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    if (succ != DWT_SUCCESS) {
        safe_printf("[uwb_handle_range_trigger] Failed to start TX for RANGE POLL\n");
        
        uwb_state = UWB_STATE_IDLE;
        dwt_forcetrxoff();
        dwt_rxreset();
        dwt_setrxtimeout(0);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return;
    }

    uwb_state = UWB_STATE_WAIT_RANGE_RESP;
}

void uwb_handle_range_poll(uwb_pkt_range_poll_t *pkt){
    
    // save poll RX timestamp
    uint64_t poll_rx_ts_64 = get_rx_timestamp();

    poll_rx_ts_presave = (uint32_t)poll_rx_ts_64;

    uint32_t resp_tx_time = (poll_rx_ts_64 + RANGE_RESP_TX_DELAY_UUS*UUS_TO_DWT_TIME)>>8;
    
    // send RANGE RESP to initiator node
    uwb_pkt_range_resp_t *resp_pkt = (uwb_pkt_range_resp_t *)tx_buffer;
    resp_pkt->header.group_id = uwb_group_id;
    resp_pkt->header.src_id = uwb_node_id;
    resp_pkt->header.dest_id = pkt->header.src_id;
    resp_pkt->header.seq_num = seq_num++;
    resp_pkt->header.msg_type = UWB_MSG_TYPE_RANGE_RESP;

    dwt_writetxdata(sizeof(uwb_pkt_range_resp_t), (uint8_t *)resp_pkt, 0);
    dwt_writetxfctrl(sizeof(uwb_pkt_range_resp_t), 0, 1);
    // dwt_forcetrxoff();
    // dwt_rxreset();
    dwt_setdelayedtrxtime(resp_tx_time);
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(RANGE_FINAL_RX_TIMEOUT_UUS);
    int succ = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
    if (succ != DWT_SUCCESS) {
        safe_printf("[uwb_handle_range_poll] Failed to start TX for RANGE RESP\n");
        
        uwb_state = UWB_STATE_IDLE;
        dwt_forcetrxoff();
        dwt_rxreset();
        dwt_setrxtimeout(0);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return;
    }
    uwb_state = UWB_STATE_WAIT_RANGE_FINAL;
}

void uwb_handle_range_resp(uwb_pkt_range_resp_t *pkt){
    
    // send RANGE FINAL to responder node
    uint64_t poll_tx_ts = get_tx_timestamp();
    uint64_t resp_rx_ts = get_rx_timestamp();

    // high 32bit of timestamp, this if for dwt_setdelayedtrxtime()
    uint32_t final_tx_time = (resp_rx_ts + RANGE_FINAL_TX_DELAY_UUS*UUS_TO_DWT_TIME)>>8;
    
    // 真實從天線發射的時間 = 設定的時間 + TX_ANT_DLY
    // only keep lower 32bit for calculation
    uint32_t final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
    
    uwb_pkt_range_final_t *final_pkt = (uwb_pkt_range_final_t *)tx_buffer;
    final_pkt->header.group_id = uwb_group_id;
    final_pkt->header.src_id = uwb_node_id;
    final_pkt->header.dest_id = pkt->header.src_id;
    final_pkt->header.seq_num = seq_num++;
    final_pkt->header.msg_type = UWB_MSG_TYPE_RANGE_FINAL;
    final_pkt->poll_tx_ts = (uint32_t)(poll_tx_ts);
    final_pkt->resp_rx_ts = (uint32_t)(resp_rx_ts);
    final_pkt->final_tx_ts = (uint32_t)(final_tx_ts);
    
    dwt_writetxdata(sizeof(uwb_pkt_range_final_t), (uint8_t *)final_pkt, 0);
    dwt_writetxfctrl(sizeof(uwb_pkt_range_final_t), 0, 1);

    dwt_setdelayedtrxtime(final_tx_time); // 40bit time, but function takes upper 32bit
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);
    int succ = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
    if (succ != DWT_SUCCESS) {
        safe_printf("[uwb_handle_range_resp] Failed to start TX for RANGE FINAL\n");
        
        uwb_state = UWB_STATE_IDLE;
        dwt_forcetrxoff();
        dwt_rxreset();
        dwt_setrxtimeout(0);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return;
    }

    uwb_state = UWB_STATE_IDLE;
}

void uwb_handle_range_final(uwb_pkt_range_final_t *pkt){

    // from the initiator node timestamps
    uint32_t poll_tx_ts = pkt->poll_tx_ts;
    uint32_t resp_rx_ts = pkt->resp_rx_ts;
    uint32_t final_tx_ts = pkt->final_tx_ts;

    // from the responder node timestamp
    uint32_t poll_rx_ts = poll_rx_ts_presave;
    uint32_t resp_tx_ts = get_tx_timestamp();
    uint32_t final_rx_ts = get_rx_timestamp();

    // user manual p229, 12.3.2 using thre messages
    float round1 = (float)( resp_rx_ts  - poll_tx_ts);
    float round2 = (float)( final_rx_ts - resp_tx_ts);
    float reply1 = (float)( resp_tx_ts  - poll_rx_ts);
    float reply2 = (float)( final_tx_ts - resp_rx_ts);

    float tof_dtu = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2); // todo: round
    float tof_s = tof_dtu * DWT_TIME_UNITS;
    float distance_m = tof_s * SPEED_OF_LIGHT;

    dwt_rxdiag_t rx_diag;
    dwt_readdiagnostics(&rx_diag);
    float rssi = calc_rssi(&rx_diag);
    
    range_final_received = true;
    range_final_ts = millis();
    range_final_node_a_id = pkt->header.src_id;
    range_final_node_b_id = pkt->header.dest_id;
    range_final_distance_m = (distance_m>0) ? distance_m : 0.0;
    range_final_rssi_dbm = rssi;

    // send the range report to address 0xFFFF (broadcast)
    uwb_pkt_range_report_t *report_pkt = (uwb_pkt_range_report_t *)tx_buffer;
    report_pkt->header.group_id = uwb_group_id;
    report_pkt->header.src_id = uwb_node_id;
    report_pkt->header.dest_id = 0xFFFF; // broadcast
    report_pkt->header.seq_num = seq_num++;
    report_pkt->header.msg_type = UWB_MSG_TYPE_RANGE_REPORT;
    report_pkt->node_a_id = pkt->header.src_id;
    report_pkt->node_b_id = pkt->header.dest_id;
    report_pkt->distance_cm = (distance_m>0)  ? (uint16_t)(distance_m * 100.0f) : 0;
    report_pkt->rssi_centi_dbm = (int16_t)(rssi * 100.0f);

    dwt_writetxdata(sizeof(uwb_pkt_range_report_t), (uint8_t *)report_pkt, 0);
    dwt_writetxfctrl(sizeof(uwb_pkt_range_report_t), 0, 1);

    uwb_state = UWB_STATE_IDLE;
    dwt_forcetrxoff();
    dwt_rxreset();
    dwt_setrxtimeout(0);
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    
    // safe_printf("[uwb_handle_range_final] distance A(0x%04X) ~ B(0x%04X): %.3f m, rssi: %.2f dBm\n",pkt->header.src_id,pkt->header.dest_id,distance_m,rssi);
}

void uwb_handle_range_final__(uwb_pkt_range_final_t *pkt){
    // safe_printf("Handling RANGE FINAL\n");

    // // from the initiator node timestamps
    // uint32_t poll_tx_ts = pkt->poll_tx_ts;
    // uint32_t resp_rx_ts = pkt->resp_rx_ts;
    // uint32_t final_tx_ts = pkt->final_tx_ts;

    // // from the responder node timestamp
    // uint32_t poll_rx_ts = poll_rx_ts_presave;
    // uint32_t resp_tx_ts = get_tx_timestamp();
    // uint32_t final_rx_ts = get_rx_timestamp();


    uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
    uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
    double Ra, Rb, Da, Db;
    int64_t tof_dtu;

    uint64_t resp_tx_ts = get_tx_timestamp();
    uint64_t final_rx_ts = get_rx_timestamp();

    poll_tx_ts = pkt->poll_tx_ts;
    resp_rx_ts = pkt->resp_rx_ts;
    final_tx_ts = pkt->final_tx_ts;


    poll_rx_ts_32 = (uint32)poll_rx_ts_presave;
    resp_tx_ts_32 = (uint32)resp_tx_ts;
    final_rx_ts_32 = (uint32)final_rx_ts;
    Ra = (double)(resp_rx_ts - poll_tx_ts);
    Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
    Da = (double)(final_tx_ts - resp_rx_ts);
    Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
    tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

    double tof = tof_dtu * DWT_TIME_UNITS;
    double distance = tof * SPEED_OF_LIGHT;
    
    dwt_rxdiag_t rx_diag;
    dwt_readdiagnostics(&rx_diag);
    // float rssi = calc_rssi(&rx_diag);
    safe_printf("DIST: %3.2f m\n", distance);
                        


    // print all number for debug
    // safe_printf("poll_tx_ts: %u, resp_rx_ts: %u, final_tx_ts: %u\n",poll_tx_ts,resp_rx_ts,final_tx_ts);
    // safe_printf("poll_rx_ts: %u, resp_tx_ts: %u, final_rx_ts: %u\n",poll_rx_ts,resp_tx_ts,final_rx_ts);
    // safe_printf("round1: %u, round2: %u, reply1: %u, reply2: %u\n",round1,round2,reply1,reply2);
    // safe_printf("tof_dtu: %.3f, tof_s: %.9f\n",tof_dtu,tof_s);
   
    // safe_printf("calculated distance node A (0x%04X) to node B (0x%04X): %.3f m\n",pkt->header.src_id,pkt->header.dest_id,distance_m);

    uwb_state = UWB_STATE_IDLE;
    // dwt_forcetrxoff();
    // dwt_rxreset();
    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

}

void uwb_handle_range_report(uwb_pkt_range_report_t *pkt){
    
    range_report_node_a_id = pkt->node_a_id;
    range_report_node_b_id = pkt->node_b_id;
    range_report_distance_m = pkt->distance_cm / 100.0f;
    range_report_rssi_dbm = pkt->rssi_centi_dbm / 100.0f;
    range_report_ts = millis();
    range_report_received = true;
    
    uwb_state = UWB_STATE_IDLE;
    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    // safe_printf("[uwb_handle_range_report] A(0x%04X) ~ B(0x%04X): %.3f m, rssi: %.2f dBm\n", pkt->node_a_id, pkt->node_b_id, range_report_distance_m, range_report_rssi_dbm);
}


void uwb_task_init(){

    if(uwb_isr_sem != NULL){
        return;
    }

    uwb_isr_sem = xSemaphoreCreateBinary();
    
    xTaskCreatePinnedToCore(
        uwb_task, // Function that implements the task.
        "uwb_task", // Text name for the task.
        4096, // Stack size in words, not bytes.
        NULL, // Parameter passed to the task
        10, // Priority of the task
        NULL, // Task handle
        1  // run on core 1
    );

    uwb_init();

}

void uwb_task(void *pvParameters){
    while(1) {
        if (xSemaphoreTake(uwb_isr_sem, portMAX_DELAY) == pdTRUE) {
            dwt_isr();
        }
    }
}

void IRAM_ATTR uwb_irq_handler() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(uwb_isr_sem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();  // 要求立即切換到 uwb_task
    }
}






// ------------------------------------------------------------------------

// polling function to be called in main loop, if not using interrupts
void uwb_process(){
    uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID);
    
    if (status_reg & SYS_STATUS_RXFCG) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG); // clear good rx frame event

        uint32_t frame_len =  dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
        if (frame_len <= RX_BUF_LEN){
            dwt_readrxdata(rx_buffer, frame_len, 0);
        }

        uint8_t rx_frame_valid = uwb_check_frame_valid(rx_buffer, frame_len);
        if(rx_frame_valid) {
            uwb_common_header_t *hdr = (uwb_common_header_t *)rx_buffer;
            switch (hdr->msg_type){
                case UWB_MSG_TYPE_PING_REQ: uwb_handle_ping_req((uwb_pkt_ping_req_t *)hdr); break;
                case UWB_MSG_TYPE_PING_RESP: uwb_handle_ping_resp((uwb_pkt_ping_resp_t *)hdr); break;

                case UWB_MSG_TYPE_RANGE_TRIGGER: uwb_handle_range_trigger((uwb_pkt_range_trigger_t *)hdr); break;
                
                case UWB_MSG_TYPE_RANGE_POLL: uwb_handle_range_poll((uwb_pkt_range_poll_t *)hdr); break;
                case UWB_MSG_TYPE_RANGE_RESP: uwb_handle_range_resp((uwb_pkt_range_resp_t *)hdr); break;
                case UWB_MSG_TYPE_RANGE_FINAL: uwb_handle_range_final((uwb_pkt_range_final_t *)hdr); break;
                case UWB_MSG_TYPE_RANGE_REPORT: uwb_handle_range_report((uwb_pkt_range_report_t *)hdr); break;
                default:
                    safe_printf("[uwb_process] rx frame valid but not found handler for msg_type=0x%02X\n", hdr->msg_type);
                
            }
        }
        else {
            safe_printf("[uwb_process] Received frame is invalid\n");
            if (_uwb_event_callback) {
                _uwb_event_callback(UWB_EVENT_INVALID_FRAME_RECEIVED, NULL);
            }

            // go to idle state
            uwb_state = UWB_STATE_IDLE;
            dwt_rxreset();
            dwt_setrxtimeout(0);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
    }
    else if (status_reg & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

        if(uwb_state == UWB_STATE_IDLE) {
            // need to re-enable rx
            dwt_rxreset();
            dwt_setrxtimeout(0);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        } 
        else{
            // timeout occurred while waiting for a response
            safe_printf("[uwb_process] Timeout occurred in state %d, %08X\n", uwb_state, status_reg);

            if (_uwb_event_callback) {
                uint8_t event;
                switch (uwb_state) {
                    case UWB_STATE_WAIT_PING_RESP: event = UWB_EVENT_PING_RESP_TIMEOUT; break;
                    case UWB_STATE_WAIT_RANGE_RESP: event = UWB_EVENT_RANGE_RESP_TIMEOUT; break;
                    case UWB_STATE_WAIT_RANGE_FINAL: event = UWB_EVENT_RANGE_FINAL_TIMEOUT; break;
                    case UWB_STATE_WAIT_RANGE_REPORT: event = UWB_EVENT_RANGE_REPORT_TIMEOUT; break;
                    default: event = UWB_EVENT_UNKNOWN_FRAME_TIMEOUT; break;
                }
                _uwb_event_callback((uwb_event_t)event, NULL);
            }
            
            // go to idle state
            uwb_state = UWB_STATE_IDLE;
            dwt_rxreset();
            dwt_setrxtimeout(0);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
        
        print_rx_err_flags(status_reg);
    }
}

void uwb_process_polling_irq(){
    uint8_t irq = dwt_checkirq();
    if (irq) {
        dwt_isr();
    }
}














