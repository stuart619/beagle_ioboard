#pragma once

#include <Arduino.h>
#include "fourTwenty.h"

#define HYDRAULIC_REST_POSITION 2444
#define PNEUMATIC_REST_POSITION 0

#define HYDRAULIC_RESOLUTION 4095
#define PNEUMATIC_RESOLUTION 255

// TODO: remove once jetson-side changes are made
#define CONVERSION_FACTOR                0.9107 // 255bits/280mm

enum ControlMode{
    AUTO = true,
    MANUAL = false,
};

enum ManualState{
    ACTIVATED = 0, // user button state inverted, pressed == 0
    DEACTIVATED = 1,
};

class Cylinder {
public:
    Cylinder(uint8_t pin, int restPosition, int resolution);
    void init();
    int getLastConf() const;
    uint16_t getLastCmd() const;
    int getLastPosition() const;

    void setBasePosition();
    virtual void setPosition(int cmd_val);
    void set(int cmd_val);
    void setVal(int cmd);
    void setJoystickVal(int cmd_val);
    void updateManual(bool state, int cmd_val);
    virtual void updateAuto(uint8_t confidence, float cmd_val);

protected:
    uint8_t pin;
    int resolution;
    int cylinderPosition;
    int base_pos;
    uint8_t switch_state;
    uint8_t prev_state;
    uint8_t log_confidence;
    float log_auto_cmd;
};

class CylinderHydraulic : public Cylinder {
public:
    CylinderHydraulic(uint8_t pin);
    void setPosition(int cmdVal) override;
    void updateAuto(float cmd_val);
};

class CylinderPneumatic : public Cylinder {
public:
    CylinderPneumatic(uint8_t pin);
};
