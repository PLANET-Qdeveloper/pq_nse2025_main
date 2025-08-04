#ifndef STATE_H
#define STATE_H

#include "main.h"
#include "constant.h"
#include "can.h"

typedef enum {
    STATE_SAFETY,
    STATE_READY,
    STATE_BURNING,
    STATE_FLIGHT,
    STATE_DECERELATION,
    STATE_LANDED,
    STATE_EMERGENCY,
} state_enum;


typedef struct {
    state_enum state;
    state_enum previous_state;
    uint32_t time;
} state_t;

void state_init(state_enum init_state);

void state_check();
void state_update(state_enum new_state);
static void state_change_condition(state_enum new_state);




#endif

