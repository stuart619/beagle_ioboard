#include "cylinder.h"

Cylinder::Cylinder(uint8_t pin, int restPosition, int resolution)
    : pin(pin), resolution(resolution) {
    pinMode(pin, OUTPUT);
    cylinderPosition = restPosition;
    setFourTwentyDAC(pin, cylinderPosition);
    switch_state = DEACTIVATED;
    log_confidence = 0;
    log_auto_cmd = 0.0;
}

void Cylinder::init() {
}

int Cylinder::getLastConf() const {
    return log_confidence;
}

uint16_t Cylinder::getLastCmd() const {
    return static_cast<uint16_t>(log_auto_cmd);
}

int Cylinder::getLastPosition() const {
    return static_cast<int>(cylinderPosition);
}

void Cylinder::setBasePosition() {
    base_pos = getLastPosition();
}

void Cylinder::setPosition(int cmd_val) {
    cylinderPosition = base_pos + (cmd_val - 140) * CONVERSION_FACTOR;
    cylinderPosition = max(0, min(cylinderPosition, PNEUMATIC_RESOLUTION));
    setVal(static_cast<int>(cylinderPosition));
}

void Cylinder::set(int cmd_val) {
    cylinderPosition = cylinderPosition + cmd_val - 127;
    cylinderPosition = max(0, min(cylinderPosition, PNEUMATIC_RESOLUTION));
    setVal(static_cast<int>(cylinderPosition));
}

void Cylinder::setVal(int cmd) {
    cylinderPosition = cmd;
    setFourTwentyDAC(pin, static_cast<int>(cylinderPosition));
}

void Cylinder::setJoystickVal(int cmd_val) {
    cylinderPosition = cylinderPosition + cmd_val;
    cylinderPosition = max(0, min(cylinderPosition, PNEUMATIC_RESOLUTION));
    setVal(static_cast<int>(cylinderPosition));
}

void Cylinder::updateManual(bool state, int cmd_val) {
    switch_state = state;
    if (switch_state == MANUAL) {
        this->set(cmd_val);
        prev_state = MANUAL;
    }
}

void Cylinder::updateAuto(uint8_t confidence, float cmd_val) {
    if (switch_state == AUTO && prev_state == MANUAL) {
        base_pos = getLastPosition();
        this->setVal(base_pos);
        prev_state = AUTO;
    } else if (switch_state == AUTO && prev_state == AUTO) {
        this->setPosition(static_cast<int>(cmd_val));
    }
    log_confidence = confidence;
    log_auto_cmd = cmd_val;
}

CylinderHydraulic::CylinderHydraulic(uint8_t pin)
    : Cylinder(pin, HYDRAULIC_REST_POSITION, HYDRAULIC_RESOLUTION) {
}

void CylinderHydraulic::setPosition(int cmdVal) {
        cylinderPosition = HYDRAULIC_REST_POSITION + cmdVal;

        cylinderPosition = max(0, min(cylinderPosition, HYDRAULIC_RESOLUTION));
        setFourTwentyDAC(pin, static_cast<int>(cylinderPosition));
}

void CylinderHydraulic::updateAuto(float cmd_val) {
    this->setPosition(static_cast<int>(cmd_val));
    log_auto_cmd = cmd_val;
}

CylinderPneumatic::CylinderPneumatic(uint8_t pin)
    : Cylinder(pin, PNEUMATIC_REST_POSITION, PNEUMATIC_RESOLUTION) {
}