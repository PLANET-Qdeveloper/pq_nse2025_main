#ifndef __CYCLECOUNTER_H__
#define __CYCLECOUNTER_H__

#include "main.h"

int init_cyclecounter(){
    volatile uint8_t *counter_ptr = (volatile uint8_t *)0xE0001000;
    *counter_ptr = 0x01;
    volatile uint32_t *counter_ptr_16 = (volatile uint32_t *)0xE0001004;
    *counter_ptr_16 = 0x00;
    return 0;
}

uint32_t get_cyclecounter(){
    volatile uint32_t *counter_ptr_16 = (volatile uint32_t *)0xE0001004;
    return *counter_ptr_16;
}

#endif

