#ifndef LOADCART_MODBUS_H
#define LOADCART_MODBUS_H
#include <ModbusMaster.h>
#include <stdint.h>
namespace HSU {
class LoadCartMotor {
public:
    void writeControlEnable();
    void writeMode(uint16_t operatingMode);
    void writeVelocity(int16_t leftSpeed, int16_t rightSpeed);
    void writeAccelerationTime(uint16_t accelTime);
    void writeDecelerationTime(uint16_t decelTime);
    void readActualVelocity();
    void readControl();
    void readMode();
    uint8_t leftAvailable(void) {
        return leftMotor.available();
    }
    uint8_t rightAvailable(void) {
        return rightMotor.available();
    }
    int16_t getLeftVelocity() const {
        return leftVelocity;
    }
    int16_t getRightVelocity() const {
        return rightVelocity;
    }
    uint16_t getLeftControl() const {
        return leftControl;
    }
    uint16_t getRightControl() const {
        return rightControl;
    }
    uint16_t getLeftMode() const {
        return leftMode;
    }
    uint16_t getRightMode() const {
        return rightMode;
    }
    bool getLeftVelocityReadError() const {
        return leftVelocityReadError;
    }
    bool getRightVelocityReadError() const {
        return rightVelocityReadError;
    }
    bool getLeftControlReadError() const {
        return leftControlReadError;
    }
    bool getRightControlReadError() const {
        return rightControlReadError;
    }
    bool getLeftModeReadError() const {
        return leftModeReadError;
    }
    bool getRightModeReadError() const {
        return rightModeReadError;
    }
    void init();

private:
    ModbusMaster leftMotor;
    ModbusMaster rightMotor;
    // 모드버스 레지스터 주소 정의
    const uint16_t REG_ADDR_CONTROL_WORD      = 0x2031;
    const uint16_t REG_ADDR_OPERATING_MODE    = 0x2032;
    const uint16_t REG_ADDR_TARGET_SPEED      = 0x203A;
    const uint16_t REG_ADDR_ACCEL_TIME        = 0x2037;
    const uint16_t REG_ADDR_DECEL_TIME        = 0x2038;
    const uint16_t REG_ADDR_ACTUAL_SPEED      = 0x202C;

    // 제어 단어 값 정의
    const uint16_t CTRL_WORD_ENABLE_SERVO     = 0x08;
    const uint16_t CTRL_WORD_STOP_SERVO       = 0x07;

    // 동작 모드 값 정의
    const uint16_t OPERATING_MODE_POSITION    = 0x01;
    const uint16_t OPERATING_MODE_VELOCITY    = 0x03;
    const uint16_t OPERATING_MODE_TORQUE      = 0x04;

    // 모터 상태 변수
    int16_t leftVelocity;
    int16_t rightVelocity;
    uint16_t leftControl;
    uint16_t rightControl;
    uint16_t leftMode;
    uint16_t rightMode;

    // 오류 상태 변수
    bool leftVelocityReadError;
    bool rightVelocityReadError;
    bool leftControlReadError;
    bool rightControlReadError;
    bool leftModeReadError;
    bool rightModeReadError;

    static void preTransmission();
    static void postTransmission();
};
}
#endif