#ifndef __SYSTEM_CONFIG_H__
#define __SYSTEM_CONFIG_H__

#include <Arduino.h>



#ifdef __cplusplus
extern "C" {
#endif


void system_factory_reset();
void system_config_init();

void save_uwb_group_id();
void save_uwb_node_id();


#ifdef __cplusplus
}
#endif


#endif // __SYSTEM_CONFIG_H__