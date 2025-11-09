
#include "safe_print.h"

#include <Arduino.h>


QueueHandle_t safe_print_queue = NULL;

void safe_print_init(){
    if (safe_print_queue != NULL) {
        Serial.println("[safe_print_init] Already initialized");
        return;
    }

    safe_print_queue = xQueueCreate(SAFE_PRINT_QUEUE_SIZE, sizeof(char));
    if (safe_print_queue == NULL) {
        // Handle queue creation failure
        Serial.println("[safe_print_init] Failed to create safe print queue");
    } else {
        // Create the safe print task
        xTaskCreatePinnedToCore(
            safe_print_task,       // Task function
            "SafePrintTask",       // Name of the task
            4096,                  // Stack size in words
            NULL,                  // Task input parameter
            1,                     // Priority of the task
            NULL,                  // Task handle
            1                      // Core to run the task on
        );
    }
}


void safe_print_task(void *pvParameters){
    char c;
    while(1) {
        if (xQueueReceive(safe_print_queue, &c, portMAX_DELAY) == pdTRUE) {
            // Process the received character
            Serial.write(c);
        }
    }
}

void safe_printf(const char *fmt, ...){
    if (safe_print_queue == NULL) {
        return;
    }
    static char buffer[SAFE_PRINT_BUFFER_SIZE];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    len = (len < SAFE_PRINT_BUFFER_SIZE) ? len : SAFE_PRINT_BUFFER_SIZE;

    if (len){
        if (xPortInIsrContext()){
            // In ISR context, use FromISR version
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            for (int i = 0; i < len; i++){
                xQueueSendFromISR(safe_print_queue, &buffer[i], &xHigherPriorityTaskWoken);
            }
            if (xHigherPriorityTaskWoken) {
                portYIELD_FROM_ISR();
            }

        } else {
            // Normal context
            for (int i = 0; i < len; i++){
                xQueueSend(safe_print_queue, &buffer[i], portMAX_DELAY);
            }
        }
    }
}

