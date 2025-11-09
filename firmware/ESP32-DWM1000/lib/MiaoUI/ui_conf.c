/**
<one line to give the program's name and a brief idea of what it does.>
Copyright (C) <2024>  <JianFeng>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "ui_conf.h"
#include "core/ui.h"
#include "display/dispDriver.h"
#include "images/image.h"
#include "widget/custom.h"
#include "version.h"
#include <Arduino.h>
#include "clib/u8g2.h"
#include "uwb.h"
#include "power.h"
// /*Page*/
// ui_page_t Home_Page, System_Page;
// /*item */
// ui_item_t HomeHead_Item, SystemHead_Item, System_Item, Image_Item, Github_Item, Bilibili_Item, Version_Item, Wave_Item;
// ui_item_t Contrast_Item, Power_Item, MenuColor_Item, CursorAni_Item, Test_Item;


ui_page_t page_home;
// ui_item_t item_print_time;
// ui_item_t item_test_data_type;
ui_item_t item_home;
ui_item_t item_uwb_setting;
ui_item_t item_tools;

ui_page_t page_test_data_type;
ui_item_t item_test_data_type_return;
ui_item_t item_data_int;
ui_item_t item_data_float;
ui_item_t item_data_switch;
ui_item_t item_data_string;

// uwb setting 
ui_page_t page_uwb_setting;
ui_item_t item_uwb_setting_return;
ui_item_t item_uwb_role_anchor;
ui_item_t item_uwb_role_tag;
ui_item_t item_uwb_id;

// page tool, ping range test
ui_page_t page_tools;
ui_item_t item_tools_return;
ui_item_t item_ping;
ui_item_t item_range;

ui_page_t page_ping;
ui_item_t item_ping_return;
ui_item_t item_ping_role_anchor;
ui_item_t item_ping_role_tag;
ui_item_t item_ping_target_id;
ui_item_t item_ping_start;

ui_page_t page_range;
ui_item_t item_range_return;
ui_item_t item_range_role_anchor;
ui_item_t item_range_role_tag;
ui_item_t item_range_target_id;
ui_item_t item_range_start;
ui_item_t item_range_start_big_font;


bool ping_role_is_anchor = true;
bool ping_role_is_tag = false;
int ping_target_uwb_id = 3;

bool range_role_is_anchor = true;
bool range_role_is_tag = false;
int range_target_uwb_id = 3;


void start_ping_test() {
    static unsigned long last_time = 0;
    char buffer[32];

    if (millis() - last_time > 250) {
        uint16_t target_id = (ping_target_uwb_id & 0xFF) | (ping_role_is_anchor ? 0xFF00 : 0x0000);
        printf("Starting Ping Test to Target ID: 0x%04X\n", target_id);
        // uwb_send_range_trigger(target_id, uwb_node_id);
        uwb_send_ping_req(target_id);
        last_time = millis();
    }

    if (ping_resp_received) {
        ping_resp_received = false;
    }

    // if > 1000 ms no response, data invalid
    bool data_valid = (millis() - ping_resp_ts) < 500;

    uint8_t color = 1;
    Disp_ClearBuffer();
    Disp_SetDrawColor(&color);
    Disp_SetFont(UI_FONT);

    snprintf(buffer, sizeof(buffer), "Ping Test %s%d", ping_role_is_anchor ? "Anchor " : "Tag ",ping_target_uwb_id);
    Disp_DrawStr(0, UI_FONT_HIGHT*1, buffer);

    if (data_valid) {
        snprintf(buffer, sizeof(buffer), "State: 0x%02X", ping_resp_system_state);
        Disp_DrawStr(0, UI_FONT_HIGHT*2, buffer);

        snprintf(buffer, sizeof(buffer), "Voltage: %d mV", ping_resp_voltage_mv);
        Disp_DrawStr(0, UI_FONT_HIGHT*3, buffer);

        snprintf(buffer, sizeof(buffer), "Timestamp: %d", ping_resp_ts);
        Disp_DrawStr(0, UI_FONT_HIGHT*4, buffer);
    } else {
        snprintf(buffer, sizeof(buffer), "No Response");
        Disp_DrawStr(0, UI_FONT_HIGHT*2, buffer);
    }
    Disp_SendBuffer();

}

void start_range_test() {
    static unsigned long last_time = 0;
    char buffer[32];

    uint16_t target_id = (range_target_uwb_id & 0xFF) | (range_role_is_anchor ? 0xFF00 : 0x0000);

    if (millis() - last_time > 200) {
        printf("Starting Range Test to Target ID: 0x%04X\n", target_id);
        uwb_send_range_trigger(target_id, uwb_node_id);
        last_time = millis();
    }
    if (range_final_received) {
        range_final_received = false;
    }

    // if > 1000 ms no response, data invalid
    bool data_valid = (millis() - range_final_ts) < 500 && range_final_node_a_id == target_id && (range_final_node_b_id == get_uwb_node_id());

    uint8_t color = 1;
    Disp_ClearBuffer();
    Disp_SetDrawColor(&color);
    Disp_SetFont(UI_FONT);
    snprintf(buffer, sizeof(buffer), "Range Test %s%d", range_role_is_anchor ? "Anchor " : "Tag ",range_target_uwb_id);
    Disp_DrawStr(0, UI_FONT_HIGHT*1, buffer);
    if (data_valid) {
        snprintf(buffer, sizeof(buffer), "A:0x%04X B:0x%04X", range_final_node_a_id, range_final_node_b_id);
        Disp_DrawStr(0, UI_FONT_HIGHT*2, buffer);

        snprintf(buffer, sizeof(buffer), "Distance: %.2f m", range_final_distance_m);
        Disp_DrawStr(0, UI_FONT_HIGHT*3, buffer);

        snprintf(buffer, sizeof(buffer), "RSSI: %.2f dBm", range_final_rssi_dbm);
        Disp_DrawStr(0, UI_FONT_HIGHT*4, buffer);

        snprintf(buffer, sizeof(buffer), "Timestamp: %d", range_final_ts);
        Disp_DrawStr(0, UI_FONT_HIGHT*5, buffer);
    } else {
        snprintf(buffer, sizeof(buffer), "No Response");
        Disp_DrawStr(0, UI_FONT_HIGHT*2, buffer);
    }
    Disp_SendBuffer();

}


void start_range_test_big_font() {
    static unsigned long last_time = 0;
    char buffer[32];

    uint16_t target_id = (range_target_uwb_id & 0xFF) | (range_role_is_anchor ? 0xFF00 : 0x0000);

    if (millis() - last_time > 200) {
        uwb_send_range_trigger(target_id, uwb_node_id);
        last_time = millis();
    }
    if (range_final_received) {
        range_final_received = false;
    }

    // if > 1000 ms no response, data invalid
    bool data_valid = (millis() - range_final_ts) < 500 && range_final_node_a_id == target_id && (range_final_node_b_id == get_uwb_node_id());

    uint8_t color = 1;
    Disp_ClearBuffer();
    Disp_SetDrawColor(&color);
    Disp_SetFont(u8g2_font_t0_30b_mf);
    uint8_t height = Disp_GetMaxCharHeight();


    //u8g2_font_t0_30b_mf
    if (data_valid) {
        snprintf(buffer, sizeof(buffer), "%.2fm", range_final_distance_m);
        Disp_DrawStr(0, height*1, buffer);

        snprintf(buffer, sizeof(buffer), "%.2fdBm", range_final_rssi_dbm);
        Disp_DrawStr(0, height*2, buffer);
    } else {
        snprintf(buffer, sizeof(buffer), "No Response");
        Disp_DrawStr(0, height*1, buffer);
    }
    Disp_SendBuffer();

}

void plot_home(){
    char buffer[16];
    uint16_t x, y, w, h;
    uint8_t color = 1;
    Disp_ClearBuffer();
    Disp_SetDrawColor(&color);

    
    Disp_SetFont(u8g2_font_t0_22_mf);
    snprintf(buffer, sizeof(buffer), "%s %d", (uwb_node_id&0xFF00) ? "ANCHOR" : "TAG", uwb_node_id & 0x00FF);
    w = Disp_GetStrWidth(buffer);
    h = Disp_GetMaxCharHeight();
    // center middle
    x = (UI_HOR_RES - w) / 2;
    y = (UI_VER_RES - h) / 2 + h;

    Disp_DrawStr(x, y, buffer);

    float voltage = get_battery_voltage_mv() / 1000.0f;
    Disp_SetFont(u8g2_font_t0_16_tf);
    snprintf(buffer, sizeof(buffer), "%.1fV", voltage);
    w = Disp_GetStrWidth(buffer);
    h = Disp_GetMaxCharHeight();
    // left top corner
    x = UI_HOR_RES - w - 2;
    y = 0 + h;

    Disp_DrawStr(x, y, buffer);

    
    Disp_SendBuffer();
}

void data_ping_role_anchor_function(){ping_role_is_tag = !ping_role_is_anchor;}
void data_ping_role_tag_function(){ping_role_is_anchor = !ping_role_is_tag;}
void data_range_role_anchor_function(){range_role_is_tag = !range_role_is_anchor;}
void data_range_role_tag_function(){range_role_is_anchor = !range_role_is_tag;}

void data_uwb_role_anchor_function(){uwb_node_is_tag = !uwb_node_is_anchor; sync_uwb_node_id_ui_to_uint16();}
void data_uwb_role_tag_function(){uwb_node_is_anchor = !uwb_node_is_tag; sync_uwb_node_id_ui_to_uint16();}

void create_parameter_uwb_setting(ui_t *ui){
    // +++++++++++++++++++ uwb setting +++++++++++++++++++++
    static ui_data_t data_uwb_role_anchor;
    data_uwb_role_anchor.name = "Anchor";
    data_uwb_role_anchor.ptr = &uwb_node_is_anchor;
    data_uwb_role_anchor.function = data_uwb_role_anchor_function;
    data_uwb_role_anchor.functionType = UI_DATA_FUNCTION_EXIT_EXECUTE;
    data_uwb_role_anchor.dataType = UI_DATA_SWITCH;
    data_uwb_role_anchor.actionType = UI_DATA_ACTION_RW;
    static ui_element_t element_uwb_role_anchor;
    element_uwb_role_anchor.data = &data_uwb_role_anchor;
    Create_element(&item_uwb_role_anchor, &element_uwb_role_anchor);

    static ui_data_t data_uwb_role_tag;
    data_uwb_role_tag.name = "Tag";
    data_uwb_role_tag.ptr = &uwb_node_is_tag;
    data_uwb_role_tag.function = data_uwb_role_tag_function;
    data_uwb_role_tag.functionType = UI_DATA_FUNCTION_EXIT_EXECUTE;
    data_uwb_role_tag.dataType = UI_DATA_SWITCH;
    data_uwb_role_tag.actionType = UI_DATA_ACTION_RW;
    static ui_element_t element_uwb_role_tag;
    element_uwb_role_tag.data = &data_uwb_role_tag;
    Create_element(&item_uwb_role_tag, &element_uwb_role_tag);

    static ui_data_t data_uwb_id;
    data_uwb_id.name = "UWB ID";
    data_uwb_id.ptr = &uwb_node_id_int;
    data_uwb_id.function = sync_uwb_node_id_ui_to_uint16;
    data_uwb_id.functionType = UI_DATA_FUNCTION_EXIT_EXECUTE;
    data_uwb_id.dataType = UI_DATA_INT;
    data_uwb_id.actionType = UI_DATA_ACTION_RW;
    data_uwb_id.max = 255;
    data_uwb_id.min = 0;
    data_uwb_id.step = 1;
    static ui_element_t element_uwb_id;
    element_uwb_id.data = &data_uwb_id;
    Create_element(&item_uwb_id, &element_uwb_id);
}

void create_parameter_ping_test(ui_t *ui){
    // +++++++++++++++++++ ping test +++++++++++++++++++++
    static ui_data_t data_ping_role_anchor;
    data_ping_role_anchor.name = "Anchor";
    data_ping_role_anchor.ptr = &ping_role_is_anchor;
    data_ping_role_anchor.function = data_ping_role_anchor_function;
    data_ping_role_anchor.functionType = UI_DATA_FUNCTION_EXIT_EXECUTE;
    data_ping_role_anchor.dataType = UI_DATA_SWITCH;
    data_ping_role_anchor.actionType = UI_DATA_ACTION_RW;
    static ui_element_t element_ping_role_anchor;
    element_ping_role_anchor.data = &data_ping_role_anchor;
    Create_element(&item_ping_role_anchor, &element_ping_role_anchor);

    static ui_data_t data_ping_role_tag;
    data_ping_role_tag.name = "Tag";
    data_ping_role_tag.ptr = &ping_role_is_tag;
    data_ping_role_tag.function = data_ping_role_tag_function;
    data_ping_role_tag.functionType = UI_DATA_FUNCTION_EXIT_EXECUTE;
    data_ping_role_tag.dataType = UI_DATA_SWITCH;
    data_ping_role_tag.actionType = UI_DATA_ACTION_RW;
    static ui_element_t element_ping_role_tag;
    element_ping_role_tag.data = &data_ping_role_tag;
    Create_element(&item_ping_role_tag, &element_ping_role_tag);

    static ui_data_t data_ping_target_id;
    data_ping_target_id.name = "Target ID";
    data_ping_target_id.ptr = &ping_target_uwb_id;
    // data_ping_target_id.function = uwb_update_id;
    // data_ping_target_id.functionType = UI_DATA_FUNCTION_EXIT_EXECUTE;
    data_ping_target_id.dataType = UI_DATA_INT;
    data_ping_target_id.actionType = UI_DATA_ACTION_RW;
    data_ping_target_id.max = 255;
    data_ping_target_id.min = 0;
    data_ping_target_id.step = 1;
    static ui_element_t element_ping_target_id;
    element_ping_target_id.data = &data_ping_target_id;
    Create_element(&item_ping_target_id, &element_ping_target_id);
}

void create_parameter_range_test(ui_t *ui){
    // +++++++++++++++++++ range test +++++++++++++++++++++
    static ui_data_t data_range_role_anchor;
    data_range_role_anchor.name = "Anchor";
    data_range_role_anchor.ptr = &range_role_is_anchor;
    data_range_role_anchor.function = data_range_role_anchor_function;
    data_range_role_anchor.functionType = UI_DATA_FUNCTION_EXIT_EXECUTE;
    data_range_role_anchor.dataType = UI_DATA_SWITCH;
    data_range_role_anchor.actionType = UI_DATA_ACTION_RW;
    static ui_element_t element_range_role_anchor;
    element_range_role_anchor.data = &data_range_role_anchor;
    Create_element(&item_range_role_anchor, &element_range_role_anchor);

    static ui_data_t data_range_role_tag;
    data_range_role_tag.name = "Tag";
    data_range_role_tag.ptr = &range_role_is_tag;
    data_range_role_tag.function = data_range_role_tag_function;
    data_range_role_tag.functionType = UI_DATA_FUNCTION_EXIT_EXECUTE;
    data_range_role_tag.dataType = UI_DATA_SWITCH;
    data_range_role_tag.actionType = UI_DATA_ACTION_RW;
    static ui_element_t element_range_role_tag;
    element_range_role_tag.data = &data_range_role_tag;
    Create_element(&item_range_role_tag, &element_range_role_tag);

    static ui_data_t data_range_target_id;
    data_range_target_id.name = "Target ID";
    data_range_target_id.ptr = &range_target_uwb_id;
    // data_range_target_id.function = uwb_update_id;
    // data_range_target_id.functionType = UI_DATA_FUNCTION_EXIT_EXECUTE;
    data_range_target_id.dataType = UI_DATA_INT;
    data_range_target_id.actionType = UI_DATA_ACTION_RW;
    data_range_target_id.max = 255;
    data_range_target_id.min = 0;
    data_range_target_id.step = 1;
    static ui_element_t element_range_target_id;
    element_range_target_id.data = &data_range_target_id;
    Create_element(&item_range_target_id, &element_range_target_id);
}

// DATA
void Create_Parameter(ui_t *ui){

    // create_parameter_test_data_type(ui);

    create_parameter_uwb_setting(ui);

    create_parameter_ping_test(ui);

    create_parameter_range_test(ui);

    


    /*
    static int Contrast = 255;
    static ui_data_t Contrast_data;
    Contrast_data.name = "Contrast";
    Contrast_data.ptr = &Contrast;
    Contrast_data.function = Disp_SetContrast;
    Contrast_data.functionType = UI_DATA_FUNCTION_STEP_EXECUTE;
    Contrast_data.dataType = UI_DATA_INT;
    Contrast_data.actionType = UI_DATA_ACTION_RW;
    Contrast_data.max = 255;
    Contrast_data.min = 0;
    Contrast_data.step = 5;
    static ui_element_t Contrast_element;
    Contrast_element.data = &Contrast_data;
    Create_element(&Contrast_Item, &Contrast_element);

    static uint8_t power_off = false;
    static ui_data_t Power_switch_data;
    Power_switch_data.ptr = &power_off;
    Power_switch_data.function = Disp_SetPowerSave;
    Power_switch_data.dataType = UI_DATA_SWITCH;
    Power_switch_data.actionType = UI_DATA_ACTION_RW;
    static ui_element_t Power_element;
    Power_element.data = &Power_switch_data;
    Create_element(&Power_Item, &Power_element);

    extern int Wave_TestData;
    static ui_data_t Wave_data;
    Wave_data.name = "Wave";
    Wave_data.ptr = &Wave_TestData;
    Wave_data.dataType = UI_DATA_INT;
    Wave_data.max = 600;
    Wave_data.min = 0;
    static ui_element_t Wave_element;
    Wave_element.data = &Wave_data;
    Create_element(&Wave_Item, &Wave_element);


    static ui_data_t MenuColor_data;
    MenuColor_data.name = "MenuColor";
    MenuColor_data.ptr = &ui->bgColor;
    MenuColor_data.dataType = UI_DATA_SWITCH;
    MenuColor_data.actionType = UI_DATA_ACTION_RW;
    static ui_element_t MenuColor_element;
    MenuColor_element.data = &MenuColor_data;
    Create_element(&MenuColor_Item, &MenuColor_element);

    static ui_data_t CursorAni_data;
    CursorAni_data.name = "CursorAni";
    CursorAni_data.ptr = &ui->animation.cursor_ani.kp;
    CursorAni_data.dataType = UI_DATA_FLOAT;
    CursorAni_data.actionType = UI_DATA_ACTION_RW;
    CursorAni_data.min = 0;
    CursorAni_data.max = 10;
    CursorAni_data.step = 0.1;
    static ui_element_t CursorAni_element;
    CursorAni_element.data = &CursorAni_data;
    Create_element(&CursorAni_Item, &CursorAni_element);
    */
}

void Create_Text(ui_t *ui){

}

void Create_MenuTree(ui_t *ui){

    
    // AddPage("Home", &page_home, UI_PAGE_TEXT);
    AddPage("Home", &page_home, UI_PAGE_ICON);
        AddItem("[HOME]", UI_ITEM_LOOP_FUNCTION, logo_allArray[0], &item_home, &page_home, NULL, plot_home);

        AddItem("[UWB Setting]", UI_ITEM_PARENTS, logo_allArray[1], &item_uwb_setting, &page_home, &page_uwb_setting, NULL);
            AddPage("UWB Setting", &page_uwb_setting, UI_PAGE_TEXT);
                AddItem(" < UWB Setting", UI_ITEM_RETURN, NULL, &item_uwb_setting_return, &page_uwb_setting, &page_home, NULL);
                AddItem(" -Anchor", UI_ITEM_DATA, NULL, &item_uwb_role_anchor, &page_uwb_setting, NULL, NULL);
                AddItem(" -Tag", UI_ITEM_DATA, NULL, &item_uwb_role_tag, &page_uwb_setting, NULL, NULL);
                AddItem(" -ID", UI_ITEM_DATA, NULL, &item_uwb_id, &page_uwb_setting, NULL, NULL);

        AddItem("[Tools]", UI_ITEM_PARENTS, logo_allArray[2], &item_tools, &page_home, &page_tools, NULL);
            AddPage("Tools", &page_tools, UI_PAGE_TEXT);
                AddItem(" < Tools", UI_ITEM_RETURN, NULL, &item_tools_return, &page_tools, &page_home, NULL);
                AddItem(" -Ping Test", UI_ITEM_PARENTS, NULL, &item_ping, &page_tools, &page_ping, NULL);
                    AddPage("Ping Test", &page_ping, UI_PAGE_TEXT);
                        AddItem(" < Return", UI_ITEM_RETURN, NULL, &item_ping_return, &page_ping, &page_tools, NULL);
                        AddItem(" -Anchor", UI_ITEM_DATA, NULL, &item_ping_role_anchor, &page_ping, NULL, NULL);
                        AddItem(" -Tag", UI_ITEM_DATA, NULL, &item_ping_role_tag, &page_ping, NULL, NULL);
                        AddItem(" -Target ID", UI_ITEM_DATA, NULL, &item_ping_target_id, &page_ping, NULL, NULL);
                        AddItem(" -Start Ping", UI_ITEM_LOOP_FUNCTION, NULL, &item_ping_start, &page_ping, NULL, start_ping_test);


                AddItem(" -Range Test", UI_ITEM_PARENTS, NULL, &item_range, &page_tools, &page_range, NULL);
                    AddPage("Range Test", &page_range, UI_PAGE_TEXT);
                        AddItem(" < Range Test", UI_ITEM_RETURN, NULL, &item_range_return, &page_range, &page_tools, NULL);
                        AddItem(" -Anchor", UI_ITEM_DATA, NULL, &item_range_role_anchor, &page_range, NULL, NULL);
                        AddItem(" -Tag", UI_ITEM_DATA, NULL, &item_range_role_tag, &page_range, NULL, NULL);
                        AddItem(" -ID", UI_ITEM_DATA, NULL, &item_range_target_id, &page_range, NULL, NULL);
                        AddItem(" -Start Range", UI_ITEM_LOOP_FUNCTION, NULL, &item_range_start, &page_range, NULL, start_range_test);
                        AddItem(" -StartRangeBF", UI_ITEM_LOOP_FUNCTION, NULL, &item_range_start_big_font, &page_range, NULL, start_range_test_big_font);


    // AddPage("Home", &page_home, UI_PAGE_ICON);
    // AddPage("Home", &page_home, UI_PAGE_TEXT);
    //     AddItem("print time", UI_ITEM_LOOP_FUNCTION, NULL, &item_home_head, &page_home, NULL, print_hello);
    //     AddItem("set bool", UI_ITEM_DATA, NULL, &item_set_bool, &page_home, NULL, NULL);
    // Draw_Home
    // AddPage("[HomePage]", &Home_Page, UI_PAGE_ICON);
    //     AddItem("[HomePage]", UI_ITEM_ONCE_FUNCTION, logo_allArray[0], &HomeHead_Item, &Home_Page, NULL, Draw_Home);
    //     AddItem(" +System", UI_ITEM_PARENTS, logo_allArray[1], &System_Item, &Home_Page, &System_Page, NULL);
    //         AddPage("[System]", &System_Page, UI_PAGE_TEXT);
    //             AddItem("[System]", UI_ITEM_RETURN, NULL, &SystemHead_Item, &System_Page, &Home_Page, NULL);
    //             AddItem(" -Contrast", UI_ITEM_DATA, NULL, &Contrast_Item, &System_Page, NULL, NULL);
    //             AddItem(" -Power off", UI_ITEM_DATA, NULL, &Power_Item, &System_Page, NULL, NULL);   
    //             AddItem(" -This is a testing program", UI_ITEM_DATA, NULL, &Test_Item, &System_Page, NULL, NULL);
    //             AddItem(" -Menu Color", UI_ITEM_DATA, NULL, &MenuColor_Item, &System_Page, NULL, NULL);   
    //             AddItem(" -Cursor Ani", UI_ITEM_DATA, NULL, &CursorAni_Item, &System_Page, NULL, NULL);
    //     AddItem(" -Image", UI_ITEM_LOOP_FUNCTION, logo_allArray[2], &Image_Item, &Home_Page, NULL, Show_Logo);
    //     AddItem(" -Github", UI_ITEM_WORD, logo_allArray[3], &Github_Item, &Home_Page, NULL, NULL);
    //     AddItem(" -This is a testing program", UI_ITEM_WORD, logo_allArray[4], &Bilibili_Item, &Home_Page, NULL, NULL);
    //     AddItem(" -Version", UI_ITEM_ONCE_FUNCTION, logo_allArray[5], &Version_Item, &Home_Page, NULL, Show_Version);
    //     AddItem(" -Wave", UI_ITEM_WAVE, logo_allArray[6], &Wave_Item, &Home_Page, NULL, NULL);
}

void MiaoUi_Setup(ui_t *ui){
    Create_UI(ui, &item_home); // 创建UI, 必须给定一个头项目
    Draw_Home(ui);
    // Process_UI_Run(ui_t *ui, UI_ACTION Action)
    ui_loop(ui);
    Process_UI_Run(ui, UI_ACTION_ENTER);
}
