#include "LOADCART.h"
#include "LOADCART_loadcell.h"
#include "LOADCART_modbus.h"
#include "LOADCART_state.h"
#include "LOADCART_config.h"
#include <Arduino.h>

void HSU::LoadCart::init()
{
    // (수정) Python과 통신을 위해 Baud Rate 9600으로 변경
    Serial.begin(9600); 
    delay(100);
    loadCell.init();
    delay(100);
    motor.init();
    delay(100);
}

// (신규) Python으로부터 'S'(Sim Start) 또는 'R'(Resume) 명령을 받음
void HSU::LoadCart::checkSerialCommands()
{
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'S') {
            simulationMode = true;
        } else if (cmd == 'R') {
            simulationMode = false;
        }
    }
}

void HSU::LoadCart::stateMachine()
{
    // (신규) 매 루프마다 시리얼 명령 확인
    checkSerialCommands();

    unsigned long currentTime = millis();

    if (current_state == State::IDLE) {
        
        // 스케줄링 1: 모터 점검 (가장 낮은 우선순위)
        if (currentTime - lastMotorCheckTime >= MOTOR_CHECK_INTERVAL) {
            lastMotorCheckTime = currentTime;
            current_state = State::CHECK_MOTOR_ENABLE;
        }
        
        // (수정) 스케줄링 2: 파이썬 데이터 전송 (중간 우선순위)
        else if (currentTime - lastPythonDataTime >= PYTHON_DATA_INTERVAL) {
            lastPythonDataTime = currentTime;
            current_state = State::SEND_PYTHON_DATA; // MONITOR_STATUS -> SEND_PYTHON_DATA
        }
        
        // 스케줄링 3: 센서 및 제어 (가장 높은 우선순위)
        else if (currentTime - lastSensorTime >= SENSOR_AND_CONTROL_INTERVAL) {
            lastSensorTime = currentTime;
            current_state = State::UPDATE_LOADCELL;
        }
    }

    switch (current_state) {
        case State::IDLE:
            break;
        
        case State::CHECK_MOTOR_ENABLE:
            motor.readControl();
            if (motor.getLeftControl() == 0x08 && motor.getRightControl() == 0x08) {
                current_state = State::IDLE;
            }
            else {
                // Serial.println("Enabling motors..."); // 로그 제거
                current_state = State::ENABLE_MOTOR;
            }
            break;
        
        case State::ENABLE_MOTOR:
            motor.writeControlEnable();
            current_state = State::IDLE;
            break;
        
        case State::UPDATE_LOADCELL:
            loadCell.readWeight();
            leftWeight = loadCell.getLeftWeight();
            rightWeight = loadCell.getRightWeight();
            current_state = State::CALCULATE_SPEED;
            break;
        
        case State::CALCULATE_SPEED:
            state.updateState(leftWeight, rightWeight);
            current_state = State::SET_MOTOR_SPEED;
            break;
        
        case State::SET_MOTOR_SPEED:
            // (수정) 시뮬레이션 모드가 아닐 때(false)만 모터를 실제로 구동
            if (!simulationMode) {
                motor.writeVelocity(state.getLeftMotorSpeed(), state.getRightMotorSpeed());
            }
            current_state = State::IDLE;
            break;
        
        // (수정) MONITOR_STATUS -> SEND_PYTHON_DATA
        case State::SEND_PYTHON_DATA:
            sendPythonData(); // printLog() -> sendPythonData()
            current_state = State::IDLE;
    }
}

// (수정) printLog() 함수를 Python 데이터 전송 함수로 변경
void HSU::LoadCart::sendPythonData()
{
    // 형식: L:{무게},R:{무게},L_RPM:{속도},R_RPM:{속도}
    Serial.print("L:");
    Serial.print(leftWeight);
    Serial.print(",R:");
    Serial.print(rightWeight);
    Serial.print(",L_RPM:");
    Serial.print(state.getLeftMotorSpeed());
    Serial.print(",R_RPM:");
    Serial.print(state.getRightMotorSpeed());
    Serial.println(); // 패킷의 끝
}