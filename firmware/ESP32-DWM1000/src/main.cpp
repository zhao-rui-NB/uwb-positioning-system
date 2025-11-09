#include <Arduino.h>

#include "uwb.h"

// UI
#include "HAL_Button.h"
#include "display/dispDriver.h"
#include "core/ui.h"
#include "ui_conf.h"
#include "power.h"

#include "safe_print.h"
#include "system_config.h"



ui_t ui;
int Wave_TestData;

// int int_val = 0;
// float float_val = 0.0;
// bool switch_val = false;
// char string_val[32] = "Hello MiaoUI!";
extern int int_val;
extern float float_val;
extern bool switch_val;
extern char string_val[32];

uint8_t new_event_flag = false;
uwb_event_t event_type;
uint8_t event_data[128];


void uwb_event_callback(uwb_event_t event, void *data) {
    new_event_flag = true;
    event_type = event;
}

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_freertos_hooks.h"
#include "freertos/task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// add print the ui fps
void ui_task(void *param) {
    // static unsigned long last_time = 0;
    // static int frame_count = 0;
    while (1) {
        // frame_count++;
        // if (millis() - last_time >= 1000) {
        //     Serial.printf("[UI] FPS: %d\n", frame_count);
        //     frame_count = 0;
        //     last_time = millis();
        // }
        ui_loop(&ui);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {


    Serial.begin(115200);
    // Serial.begin(2000000);
    uint64_t chipid64 = ESP.getEfuseMac();
    Serial.printf("[esp32] ESP32 Chip ID: %08X%08X\n", (uint32_t)(chipid64>>32), (uint32_t)chipid64);
    uint32_t chipid = (uint32_t)(chipid64>>32);

    system_config_init();

    pinMode(BTN_UP, INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);
    pinMode(BTN_OK, INPUT_PULLUP);

    dispInit();
    MiaoUi_Setup(&ui);


    // uwb_init();
    uwb_task_init(); // UWB irq中斷觸發 UWB 任務

    uwb_register_event_callback(uwb_event_callback);

    // chip id to select role
    // uwb_group_id = 0x1234;
    // uwb_node_id = 0x0001;
    // Serial.printf("[role] setting group_id: 0x%04X, node_id: 0x%04X\n", uwb_group_id, uwb_node_id);
    
    /*
    set_uwb_group_id(0x1234);
    
    switch(chipid) {
        case 0xCCFD:
            set_uwb_node_id(0x0001);
            Serial.printf("[role] setting group_id: 0x%04X, node_id: 0x%04X\n", get_uwb_group_id(), get_uwb_node_id());
            break;

        case 0xE406:
            set_uwb_node_id(0xFF01);
            Serial.printf("[role] setting group_id: 0x%04X, node_id: 0x%04X\n", get_uwb_group_id(), get_uwb_node_id());
            break;

        // anchor
        case 0xDC74:
            set_uwb_node_id(0xFF03);
            Serial.printf("[role] setting group_id: 0x%04X, node_id: 0x%04X\n", get_uwb_group_id(), get_uwb_node_id());
            break;
        //tag
        case 0x0C00:
            set_uwb_node_id(0x0003);
            Serial.printf("[role] setting group_id: 0x%04X, node_id: 0x%04X\n", get_uwb_group_id(), get_uwb_node_id());
            break;

        default:
            Serial.println("[ERROR] chip id does not match any role");
            set_uwb_node_id(0x0000);
            Serial.printf("[role] setting group_id: 0x%04X, node_id: 0x%04X\n", get_uwb_group_id(), get_uwb_node_id());

            // while(1) {}
    }
    */
    
    
    
    
    power_init();

    safe_print_init();

    xTaskCreatePinnedToCore(
        ui_task,          /* Task function. */
        "ui_task",        /* name of task. */
        4096,             /* Stack size of task */
        NULL,             /* parameter of the task */
        1,                /* priority of the task */
        NULL,             /* Task handle to keep track of created task */
        0);               /* pin task to core 0 */
}





void loop() {
    // delay
    vTaskDelay(pdMS_TO_TICKS(10));

    // ui_loop(&ui);

    // uwb_process();
    // uwb_process_polling_irq();

    static unsigned long last_print_ping_resp = 0;
    static unsigned long last_print_range_final = 0;
    static unsigned long last_print_range_report = 0;

    if (ping_resp_ts != last_print_ping_resp) {
        last_print_ping_resp = ping_resp_ts;
        // Serial.printf("Ping Resp Received: Node ID: 0x%04X, System State: 0x%02X, Voltage: %d mV\n",
        //     ping_resp_node_id,
        //     ping_resp_system_state, 
        //     ping_resp_voltage_mv
        // );
    
        // print as JSON format for easy parsing
        Serial.printf("{\"event\":\"ping_resp\",\"node_id\":%d,\"system_state\":%d,\"voltage_mv\":%d}\n",
            ping_resp_node_id,
            ping_resp_system_state, 
            ping_resp_voltage_mv
        );
    }

    if (range_final_ts != last_print_range_final) {
        last_print_range_final = range_final_ts;
        // Serial.printf("Range Final Received: Node A ID: 0x%04X, Node B ID: 0x%04X, Distance: %.2f m\n",
        //     range_final_node_a_id,
        //     range_final_node_b_id,
        //     range_final_distance_m
        // );
    
        Serial.printf("{\"event\":\"range_final\",\"node_a_id\":%d,\"node_b_id\":%d,\"distance_m\":%.2f,\"rssi_dbm\":%.2f}\n",
            range_final_node_a_id,
            range_final_node_b_id,
            range_final_distance_m,
            range_final_rssi_dbm
        );
    }

    if (range_report_ts != last_print_range_report) {
        last_print_range_report = range_report_ts;
        // Serial.printf("Range Report Received: Node A ID: 0x%04X, Node B ID: 0x%04X, Distance: %.2f m, RSSI: %.2f dBm\n",
        //     range_report_node_a_id,
        //     range_report_node_b_id,
        //     range_report_distance_m,
        //     range_report_rssi_dbm
        // );
    
        // print as JSON format for easy parsing
        Serial.printf("{\"event\":\"range_report\",\"node_a_id\":%d,\"node_b_id\":%d,\"distance_m\":%.2f,\"rssi_dbm\":%.2f}\n",
            range_report_node_a_id,
            range_report_node_b_id,
            range_report_distance_m,
            range_report_rssi_dbm
        );
    }





    // handle UWB events
    if (new_event_flag) {
        new_event_flag = false;
        // // print event for debugging
        // switch (event_type) {
        //     case UWB_EVENT_PING_RESP_TIMEOUT:       Serial.printf("[UWB EVENT FLAG] PING RESP timeout\n");         break;
        //     case UWB_EVENT_RANGE_RESP_TIMEOUT:      Serial.printf("[UWB EVENT FLAG] RANGE RESP timeout\n");         break;
        //     case UWB_EVENT_RANGE_FINAL_TIMEOUT:     Serial.printf("[UWB EVENT FLAG] RANGE FINAL timeout\n");        break;
        //     case UWB_EVENT_RANGE_REPORT_TIMEOUT:    Serial.printf("[UWB EVENT FLAG] RANGE REPORT timeout\n");       break;
        //     case UWB_EVENT_UNKNOWN_FRAME_TIMEOUT:   Serial.printf("[UWB EVENT FLAG] UNKNOWN FRAME timeout\n");      break;
        //     case UWB_EVENT_UNKNOWN_FRAME_ERROR:     Serial.printf("[UWB EVENT FLAG] UNKNOWN FRAME error\n");        break;
        //     case UWB_EVENT_INVALID_FRAME_RECEIVED:  Serial.printf("[UWB EVENT FLAG] INVALID FRAME received\n");     break;
        // }
    }

    // parse uart commands 
    // cmd1: ping <node_id>
    // cmd2: trigger <node_id> <range_node_id>
    // cmd3: range <range_node_id>
    
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) {
            return;
        }

        // Serial.printf("Received command: %s\n", line.c_str());

        char cmd[32];
        char arg1[32];
        char arg2[32];

        int num_args = sscanf(line.c_str(), "%31s %31s %31s", cmd, arg1, arg2);

        if (strcmp(cmd, "ping") == 0 && num_args == 2) {
            uint16_t target_node_id = strtol(arg1, NULL, 0);
            uint8_t succ = uwb_send_ping_req(target_node_id);
        }
        else if (strcmp(cmd, "trigger") == 0 && num_args == 3) {
            uint16_t initiator_id = strtol(arg1, NULL, 0);
            uint16_t responder_id = strtol(arg2, NULL, 0);
            uint8_t succ = uwb_send_range_trigger(initiator_id, responder_id);
        }
        else {
            Serial.println("Unknown command or wrong number of arguments");
        }

    }

}