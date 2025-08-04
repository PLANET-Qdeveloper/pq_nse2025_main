#ifndef __CAN_H__
#define __CAN_H__

#include "main.h"

typedef struct {
    uint32_t id;
    uint8_t data[128];
    uint16_t length;
} can_message_t;

int can_init(void);
void can_send(uint16_t id, uint8_t *data, uint8_t length);
void can_check(void);
int add_buffer_can(uint8_t *data, uint16_t size);

#endif


