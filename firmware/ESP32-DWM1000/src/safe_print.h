#ifndef __SAFE_PRINT_H__
#define __SAFE_PRINT_H__

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>


#ifdef __cplusplus
extern "C" {
#endif


#define SAFE_PRINT_QUEUE_SIZE 1024
#define SAFE_PRINT_BUFFER_SIZE 128

extern QueueHandle_t safe_print_queue;

void safe_print_init();

void safe_print_task(void *pvParameters);   

void safe_printf(const char *fmt, ...);



#ifdef __cplusplus
}
#endif

#endif // __SAFE_PRINT_H__