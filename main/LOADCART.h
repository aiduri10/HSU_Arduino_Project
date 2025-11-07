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
    
    // printLog가 sendPythonData로 변경됨
    void sendPythonData(); 

private:
    // (신규) 시리얼 명령 체크 함수
    void checkSerialCommands();

    // (변경) MONITOR_STATUS -> SEND_PYTHON_DATA
    enum class State {
        IDLE,
        CHECK_MOTOR_ENABLE,
        ENABLE_MOTOR,
        UPDATE_LOADCELL,
        CALCULATE_SPEED,
        SET_MOTOR_SPEED,
        SEND_PYTHON_DATA // 이름 변경
    };

    LoadCartLoadCell loadCell;
    LoadCartMotor motor;
    LoadCartState state;
    
    int leftWeight;
    int rightWeight;
    
    unsigned long lastSensorTime;
    unsigned long lastMotorCheckTime;
    unsigned long lastPythonDataTime; // lastLogTime -> lastPythonDataTime
    
    State current_state;

    // (신규) 시뮬레이션 모드 플래그
    bool simulationMode = false; 

    // (변경) 인터벌 값 수정
    const long SENSOR_AND_CONTROL_INTERVAL = 10; // 10ms 마다 센서 읽고 모터 제어
    const long MOTOR_CHECK_INTERVAL = 500;     // 500ms 마다 모터 활성화 상태 점검
    const long PYTHON_DATA_INTERVAL = 50;      // 50ms 마다 파이썬으로 데이터 전송
};
}
#endif