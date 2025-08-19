#include "state.h"
#include "environment.h"
#include "battery.h"

extern battery_data_t battery_data;
extern env_data_t env_data;
extern UART_HandleTypeDef huart3;
float previous_voltage = 0;
int voltage_count = 0;

state_t state;

void state_init(state_enum init_state){
    state.state = STATE_SAFETY;
    state.previous_state = STATE_SAFETY;
    state.time = 0;
}

void state_check(){
    uint32_t time_duration;
    switch (state.state){
        case STATE_SAFETY:
            break;
        case STATE_READY:
            if(battery_data.voltage < 12.7){
                voltage_count++;
                if(voltage_count > 10){
                    state_update(STATE_BURNING);
                }
            }else{
                voltage_count = 0;
            }
            previous_voltage = battery_data.voltage;
            break;
        case STATE_BURNING:
            time_duration = HAL_GetTick() - state.time;
            if (time_duration > STATE_BURNING_TIME){
                state_update(STATE_FLIGHT);
            }
            break;
        case STATE_FLIGHT:
            time_duration = HAL_GetTick() - state.time;
            if (time_duration > STATE_FLIGHT_TIME){
                state_update(STATE_DECERELATION);
            }
            if(env_data.delta_press * 100000 > 35.0f){
                state_update(STATE_DECERELATION);
            }
            break;
        case STATE_DECERELATION:
            time_duration = HAL_GetTick() - state.time;
            if (time_duration > STATE_DECERELATION_TIME){
                state_update(STATE_LANDED);
            }
            break;
        case STATE_LANDED:

            break;
        case STATE_EMERGENCY:
            break;
    }
    

}

void state_update(state_enum new_state){
    switch (new_state){
        case STATE_SAFETY:
            state_change_condition(STATE_SAFETY);
            break;
        case STATE_READY:
            state_change_condition(STATE_READY);
            break;
        case STATE_BURNING:
            state_change_condition(STATE_BURNING);
            break;
        case STATE_FLIGHT:
            state_change_condition(STATE_FLIGHT);
            break;
        case STATE_DECERELATION:
            state_change_condition(STATE_DECERELATION);
            break;
        case STATE_LANDED:
            state_change_condition(STATE_LANDED);
            break;
        case STATE_EMERGENCY:
            state_change_condition(STATE_EMERGENCY);
            break;
    }
}

static void state_change_condition(state_enum new_state){
    state.previous_state = state.state;
    state.state = new_state;
    state.time = HAL_GetTick();
}








