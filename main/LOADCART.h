#ifndef LOADCART_H
#define LOADCART_H
#include "LOADCART_loadcell.h"
#include "LOADCART_modbus.h"
#include "LOADCART_state.h"
namespace HSU {
class LoadCart {
public:
    void init();
    void stateMachine();
    void printLog();
private:
    enum class State {
        IDLE,
        CHECK_MOTOR_ENABLE,
        ENABLE_MOTOR,
        UPDATE_LOADCELL,
        CALCULATE_SPEED,
        SET_MOTOR_SPEED,
        MONITOR_STATUS
    };
    LoadCartLoadCell loadCell;
    LoadCartMotor motor;
    LoadCartState state;
    int leftWeight;
    int rightWeight;
    unsigned long lastSensorTime;
    unsigned long lastMotorCheckTime;
    unsigned long lastLogTime;
    State current_state;
    const long SENSOR_AND_CONTROL_INTERVAL = 10; // 20ms 마다 센서 읽고 모터 제어
    const long MOTOR_CHECK_INTERVAL = 500;       // 500ms 마다 모터 활성화 상태 점검
    const long LOG_INTERVAL = 1000;              // 1초마다 로그 출력
};
}
#endif
