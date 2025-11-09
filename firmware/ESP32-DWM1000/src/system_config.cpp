#include "system_config.h"

#include <Preferences.h>

#include "uwb.h"
static Preferences prefs;

void system_factory_reset() {
    prefs.begin("syscfg", false);
    prefs.clear();
    prefs.end();
}

// 初始化設定系統
void system_config_init() {
    prefs.begin("syscfg", false); // 建立 NVS 命名空間

    if (prefs.isKey("uwb_gid")) {
        set_uwb_group_id(prefs.getUShort("uwb_gid"));
        Serial.printf("[system_config] Load uwb group_id %04X from NVS\n", get_uwb_group_id());
    } 
    else {
        set_uwb_group_id(0x1234); // 預設值
        save_uwb_group_id();
        Serial.printf("[system_config] No saved uwb group_id in NVS, use default %04X\n", get_uwb_group_id());
    }

    if (prefs.isKey("uwb_nid")) {
        set_uwb_node_id(prefs.getUShort("uwb_nid"));
        Serial.printf("[system_config] Load uwb node_id %04X from NVS\n", get_uwb_node_id());
    } 
    else {
        set_uwb_node_id(0x0000); // 預設值
        save_uwb_node_id();
        Serial.printf("[system_config] No saved uwb node_id in NVS, use default %04X\n", get_uwb_node_id());
    }
    
}

// 保存 UWB 群組 ID 到 NVS
void save_uwb_group_id() {
    // check if no existing or different, save only when changed
    if (!prefs.isKey("uwb_gid") || prefs.getUShort("uwb_gid") != get_uwb_group_id()) {
        prefs.putUShort("uwb_gid", get_uwb_group_id());
        Serial.printf("[system_config] Saved uwb group_id %04X to NVS\n", get_uwb_group_id());
    } 
    // else{
    //     Serial.printf("[system_config] uwb group_id %04X unchanged, not saving to NVS\n", get_uwb_group_id());
    // }
}

// 保存 UWB 節點 ID 到 NVS
void save_uwb_node_id() {
    // check if no existing or different, save only when changed
    if (!prefs.isKey("uwb_nid") || prefs.getUShort("uwb_nid") != get_uwb_node_id()) {
        prefs.putUShort("uwb_nid", get_uwb_node_id());
        Serial.printf("[system_config] Saved uwb node_id %04X to NVS\n", get_uwb_node_id());
    } 
    // else{
    //     Serial.printf("[system_config] uwb node_id %04X unchanged, not saving to NVS\n", get_uwb_node_id());
    // }
}
