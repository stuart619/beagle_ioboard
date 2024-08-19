
#include <Arduino.h>
#include <stdint.h>
#include "cylinder.h"
#include "Servo.h"
#include "gripper.h"
#include "relay.h"

#define SCALING_FACTOR         .1 // scaling factor for controlling cylinders
#define DEADMAN_RESET_INTERVAL 1000 // resets clicks after one seconds

#define CAN_JOYSTICK_ID         0x0CFDD633
#define CAN_JOYSTICK_ID_DEC     217962035 // Alternative joystick: 2365445683

// #define GRIPPER_POS 1   // gripper pos-close
// #define GRIPPER_NEG 2   // gripper neg-open
#define PIN_SCISSORS 1  // scissors
#define PIN_ROTATION 2  // horizontal cylinder 
#define PIN_EXTEND 3  // rotational valve
#define PIN_SECOND_CUTTER 4  // rotational valve

GripperMotor gripper_motor( PIN_SCISSORS, PIN_ROTATION, PIN_EXTEND ,PIN_SECOND_CUTTER);

class Joystick {
public:

    // Hold automatic mode vertical command values
    float ver_left_cmd_val = 0.0;
    float ver_right_cmd_val = 0.0;

    struct CanJoystick {
        // Signed handle X and handle Y values in the range -255 to 255
        float handle_x; // Bits [0-9] of byte 0 (X+) and [0-3] of byte 1 (X-)
        float handle_y; // Bits [0-9] of byte 2 (Y+) and [0-3] of byte 3 (Y-)

        // Button values
        bool button_1; // Bit 6 of byte 5
        bool button_2; // Bit 4 of byte 5
        bool button_3; // Bit 2 of byte 5
        bool button_4; // Bit 0 of byte 5
        bool button_5; // Bit 6 of byte 6
        bool button_6; // Bit 4 of byte 6
        bool button_7; // Bit 0 of byte 6
    };

    void init() {
        joystick_data = {0, 0, false, false, false, false, false, false, false};
    }

    Joystick(CylinderHydraulic* leftVerticalCyl, CylinderHydraulic* rightVerticalCyl)
        : leftVerticalCylinder(leftVerticalCyl), rightVerticalCylinder(rightVerticalCyl) {
        init();
    }

    void parseCanJoystickMessage(const uint8_t *can_message);

    // Getters for all joystick values
    float getHandleX() const { return joystick_data.handle_x; }
    float getHandleY() const { return joystick_data.handle_y; }
    bool getButton1() const { return joystick_data.button_1; }
    bool getButton2() const { return joystick_data.button_2; }
    bool getButton3() const { return joystick_data.button_3; }
    bool getButton4() const { return joystick_data.button_4; }
    bool getButton5() const { return joystick_data.button_5; }
    bool getButton6() const { return joystick_data.button_6; }
    bool getButton7() const { return joystick_data.button_7; }

    // Check if the right automatic mode is engaged
    bool isRightAutomaticMode() const { return (getButton2() && !getButton1()); }

    // Check if the right manual mode is engaged
    bool isRightManualMode() const { return (getButton1() && !getButton2()); }

    // Check if the left automatic mode is engaged
    bool isLeftAutomaticMode() const { return (getButton3() && !getButton4()); }

    // Check if the left manual mode is engaged
    bool isLeftManualMode() const { return (getButton4() && !getButton3()); }

    // Check if the left control is engaged (both horizontal and vertical)
    bool isLeftControlEngaged() const { return getButton5(); }

    // Check if the right control is engaged (both horizontal and vertical)
    bool isRightControlEngaged() const { return getButton6(); }

    // Check if the deadman switch is engaged (vertical directional valve control)
    bool isDeadmanSwitchEngaged() const { return getButton7(); }

    // Check if the deadman has been clicked for dynamic adjustment
    void handleDeadman();
    int getDeadmanClickCount() const { return deadman_click_count; }

    // Control horizontal or vertical cylinder based on joystick input and button statuses
    void setLeftCmdVal(int ver_val) { ver_left_cmd_val = ver_val; }
    void setRightCmdVal(int ver_val) { ver_right_cmd_val = ver_val; }
    void controlCylinders();

    // Function to print all the values of the joystick_data structure
    void printJoystickData() const {
        Serial.print("Handle X: "); Serial.print(joystick_data.handle_x);
        Serial.print("; Handle Y: "); Serial.print(joystick_data.handle_y);
        Serial.print("; Button 1: "); Serial.print(joystick_data.button_1);
        Serial.print("; Button 2: "); Serial.print(joystick_data.button_2);
        Serial.print("; Button 3: "); Serial.print(joystick_data.button_3);
        Serial.print("; Button 4: "); Serial.print(joystick_data.button_4);
        Serial.print("; Button 5: "); Serial.print(joystick_data.button_5);
        Serial.print("; Button 6: "); Serial.print(joystick_data.button_6);
        Serial.print("; Button 7: "); Serial.println(joystick_data.button_7);
    }

private:
    CanJoystick joystick_data;
    float accumulatedPositionYLeft = 0.0;  // Accumulated position for left vertical cylinder
    float accumulatedPositionYRight = 0.0; // Accumulated position for right vertical cylinder

    CylinderHydraulic* leftVerticalCylinder;
    CylinderHydraulic* rightVerticalCylinder;

    bool prev_deadman_state = false; // Previous state of the deadman switch
    int deadman_click_count = 0;     // Number of deadman switch clicks
    unsigned long last_deadman_click_time = 0; // holds the timestamp of the last deadman click
    unsigned long debounce_time = 0; // holds the timestamp for the deadman debouncing
};

void Joystick::parseCanJoystickMessage(const uint8_t *can_message) {
    // Parse the handle X value
    int16_t handle_x = (can_message[0] >> 8 | can_message[1]);

    // Parse the handle Y value
    int16_t handle_y = (can_message[2] >> 8 | can_message[3]);

    // Dictate polarity for handle X
    bool x_left_neg = (can_message[0] >> 2) & 0x01;
    bool x_right_pos = (can_message[0] >> 4) & 0x01;
    joystick_data.handle_x = (x_left_neg ? -1.0 : (x_right_pos ? 1.0 : 0.0)) * static_cast<float>(handle_x);
    
    // Dictate polarity for handle Y
    bool y_backward_neg = (can_message[2] >> 2) & 0x01;
    bool y_forward_pos = (can_message[2] >> 4) & 0x01;
    joystick_data.handle_y = (y_backward_neg ? -1.0 : (y_forward_pos ? 1.0 : 0.0)) * static_cast<float>(handle_y);
    
    joystick_data.button_1 = (can_message[5] & 0x40); // Bit 6 of byte 5
    joystick_data.button_2 = (can_message[5] & 0x10); // Bit 4 of byte 5
    joystick_data.button_3 = (can_message[5] & 0x04); // Bit 2 of byte 5
    joystick_data.button_4 = (can_message[5] & 0x01); // Bit 0 of byte 5

    joystick_data.button_5 = (can_message[6] & 0x40); // Bit 6 of byte 6
    joystick_data.button_6 = (can_message[6] & 0x10); // Bit 4 of byte 6
    joystick_data.button_7 = (can_message[6] & 0x01); // Bit 0 of byte 6

    // Check the deadman switch for dynamic adjustments
    handleDeadman();
}

void Joystick::handleDeadman() {
    bool current_deadman_state = isDeadmanSwitchEngaged();

    // Check if the deadman has been clicked, in which case we increment the deadman click counter
    if (current_deadman_state && !prev_deadman_state) {
        deadman_click_count = (deadman_click_count + 1) % 3; // restricts deadman clicks to 0, 1, or 2
        last_deadman_click_time = millis();
        // Serial.println(deadman_click_count);
    }
    
    // Reset the click count if no click for a certain interval
    if (millis() - last_deadman_click_time > DEADMAN_RESET_INTERVAL) {
        deadman_click_count = 0;
        // Serial.println(deadman_click_count);
    }

    prev_deadman_state = current_deadman_state;
}

void Joystick::controlCylinders() {
    // Determine the handle values for X and Y axes
    float handle_x = getHandleX();
    float handle_y = getHandleY();

    // Determine the button statuses
    bool is_left_manual_mode = isLeftManualMode();
    bool is_right_manual_mode = isRightManualMode();
    bool is_left_automatic_mode = isLeftAutomaticMode();
    bool is_right_automatic_mode = isRightAutomaticMode();
    bool is_left_control_engaged = isLeftControlEngaged();
    bool is_right_control_engaged = isRightControlEngaged();
    bool is_deadman_engaged = isDeadmanSwitchEngaged();


    // Check if the control and deadman switch are engaged for left control
    if (is_left_automatic_mode && is_right_automatic_mode) {
        setFourTwentyDAC(2, 0);
        if (is_left_control_engaged){
        setServoTarget(0x00140000,0x000061A8);//speed is 1000r/min
        }
        
    }
    if (!is_left_automatic_mode && is_right_automatic_mode) {
        setFourTwentyDAC(2, 2100);
    }


    // control the motor in maunul mode
    if (is_left_manual_mode && is_right_manual_mode) {
        
        // if the left button is engaged, control the position of the motor
        if (is_left_control_engaged) {

            float position_change_y = handle_y;
            // Serial.println(position_change_y);
            setServoposition(int(position_change_y));
        }
        // if the deadman switch is engaged, sequence is executed
        if (is_deadman_engaged) {
            // sequene
            gripper_motor.seq(true);
            
        }
        if (is_right_control_engaged) {
            gripper_motor.gripperReset();   
        }

    //     //  test gripper
    //     if (is_right_control_engaged) {

    //         setRelay(GRIPPER_POS, HIGH);
    //         setRelay(GRIPPER_NEG, LOW);
    //     }
    //     else{
    //         setRelay(GRIPPER_POS, LOW);
    //         setRelay(GRIPPER_NEG, LOW);
    //     }
    }
    
    if (is_left_manual_mode && !is_right_manual_mode) {
        // perform this function before turning off the power       
        if (is_deadman_engaged){
           zero_return_mode();
        }
    }
 

    //     if (is_deadman_engaged)
    //     {
    //         setRelay( PIN_SECOND_CUTTER, HIGH);
    //         delay(50);
    //         setRelay( PIN_SECOND_CUTTER, LOW);
    //     }
    // }

}
