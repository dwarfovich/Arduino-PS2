#include "PS2X_lib.h"

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include <pins_arduino.h>

static byte enter_config[]    = { 0x01, 0x43, 0x00, 0x01, 0x00 };
static byte set_mode[]        = { 0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00 };
static byte set_bytes_large[] = { 0x01, 0x4F, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00 };
static byte exit_config[]     = { 0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A };
static byte enable_rumble[]   = { 0x01, 0x4D, 0x00, 0x00, 0x01 };
static byte type_read[]       = { 0x01, 0x45, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A };

byte PS2Controller::configure(uint8_t clock, uint8_t command, uint8_t attribute, uint8_t data)
{
    return configure(clock, command, attribute, data, false, false);
}

byte PS2Controller::configure(
    uint8_t clockPin, uint8_t commandPin, uint8_t attributePin, uint8_t dataPin, bool pressures, bool rumble)
{
    const uint8_t oldSreg = SREG; // *** KJE *** save away the current state of interrupts
    byte          temp[sizeof(type_read)];

    clockMask_              = maskToBitNum(digitalPinToBitMask(clockPin));
    clockOuputRegister_     = portOutputRegister(digitalPinToPort(clockPin));
    commandMask_            = maskToBitNum(digitalPinToBitMask(commandPin));
    commandOutputRegister   = portOutputRegister(digitalPinToPort(commandPin));
    attributeMask_          = maskToBitNum(digitalPinToBitMask(attributePin));
    attributeOutputRegister = portOutputRegister(digitalPinToPort(attributePin));
    dataMask_               = maskToBitNum(digitalPinToBitMask(dataPin));
    dataInputRegister_      = portInputRegister(digitalPinToPort(dataPin));

    // Configure ports
    pinMode(clockPin, OUTPUT);
    pinMode(attributePin, OUTPUT);
    pinMode(commandPin, OUTPUT);
    pinMode(dataPin, INPUT);

    digitalWrite(dataPin, HIGH); // enable pull-up

    cli();                                     // *** KJE *** disable for now
    SET(*commandOutputRegister, commandMask_); // SET(*_cmd_oreg,_cmd_mask);
    SET(*clockOuputRegister_, clockMask_);
    SREG = oldSreg; // *** *** KJE *** *** Interrupts may be enabled again

    // new error checking. First, read gamepad a few times to see if it's talking
    read_gamepad();
    read_gamepad();

    // see if it talked
    if (PS2data[1] != 0x41 && PS2data[1] != 0x73
        && PS2data[1] != 0x79) { // see if mode came back. If still anything but 41, 73 or 79, then it's not talking
#ifdef PS2X_DEBUG
        Serial.println("Controller mode not matched or no controller found");
        Serial.print("Expected 0x41 or 0x73, got ");
        Serial.println(PS2data[1], HEX);
#endif
        return 1; // return error code 1
    }

    // Try setting mode, increasing delays if needed.
    read_delay = 1;
    for (int y = 0; y <= 10; y++) {
        sendCommandString(enter_config, sizeof(enter_config)); // start config run

        // read controllerType
        delayMicroseconds(CTRL_BYTE_DELAY);

        cli(); // *** KJE *** disable for now
        SET(*commandOutputRegister, commandMask_);
        SET(*clockOuputRegister_, clockMask_);
        CLR(*attributeOutputRegister, attributeMask_); // low enable joystick
        SREG = oldSreg;                                // *** *** KJE *** *** Interrupts may be enabled again

        delayMicroseconds(CTRL_BYTE_DELAY);

        for (int i = 0; i < 9; i++) {
            temp[i] = _gamepad_shiftinout(type_read[i]);
        }

        cli();                                         // *** KJE *** disable for now
        SET(*attributeOutputRegister, attributeMask_); // HI disable joystick
        SREG = oldSreg;                                // *** *** KJE *** *** Interrupts may be enabled again

        controllerType_ = temp[3];

        sendCommandString(set_mode, sizeof(set_mode));
        if (rumble) {
            sendCommandString(enable_rumble, sizeof(enable_rumble));
            enableRumble_ = true;
        }
        if (pressures) {
            sendCommandString(set_bytes_large, sizeof(set_bytes_large));
            enablePressures_ = true;
        }
        sendCommandString(exit_config, sizeof(exit_config));

        read_gamepad();

        if (pressures) {
            if (PS2data[1] == 0x79)
                break;
            if (PS2data[1] == 0x73)
                return 3;
        }

        if (PS2data[1] == 0x73)
            break;

        if (y == 10) {
#ifdef PS2X_DEBUG
            Serial.println("Controller not accepting commands");
            Serial.print("mode stil set at");
            Serial.println(PS2data[1], HEX);
#endif
            return 2; // exit function with error
        }

        read_delay += 1; // add 1ms to read_delay
    }

    return 0; // no error if here
}


boolean PS2Controller::buttonPressed(uint16_t button)
{
    return ((~buttons & button) > 0);
}

// boolean PS2Controller::buttonPressed(unsigned int button)
// {
//     return (NewButtonState(button) & buttonPressed(button));
// }

boolean PS2Controller::NewButtonState()
{
    return ((last_buttons ^ buttons) > 0);
}

boolean PS2Controller::NewButtonState(unsigned int button)
{
    return (((last_buttons ^ buttons) & button) > 0);
}

boolean PS2Controller::ButtonReleased(unsigned int button)
{
    return ((NewButtonState(button)) & ((~last_buttons & button) > 0));
}

unsigned int PS2Controller::ButtonDataByte()
{
    return (~buttons);
}

byte PS2Controller::Analog(byte button)
{
    return PS2data[button];
}
unsigned char PS2Controller::_gamepad_shiftinout(char byte)
{
    uint8_t oldSreg = SREG; // *** KJE *** save away the current state of interrupts

    unsigned char tmp = 0;
    cli(); // *** KJE *** disable for now
    for (i = 0; i < 8; i++) {
        if (CHK(byte, i))
            SET(*commandOutputRegister, commandMask_);
        else
            CLR(*commandOutputRegister, commandMask_);
        CLR(*clockOuputRegister_, clockMask_);

        SREG = oldSreg; // *** *** KJE *** *** Interrupts may be enabled again
        delayMicroseconds(CTRL_CLK);
        cli(); // *** KJE ***

        if (CHK(*dataInputRegister_, dataMask_))
            SET(tmp, i);
        SET(*clockOuputRegister_, clockMask_);
    }
    SET(*commandOutputRegister, commandMask_);
    SREG = oldSreg; // *** *** KJE *** *** Interrupts may be enabled again
    delayMicroseconds(CTRL_BYTE_DELAY);
    
    return tmp;
}

void PS2Controller::read_gamepad()
{
    read_gamepad(false, 0x00);
}

void PS2Controller::read_gamepad(boolean motor1, byte motor2)
{
    double  temp     = millis() - last_read;
    uint8_t old_sreg = SREG; // *** KJE **** save away the current state of interrupts - *** *** KJE *** ***

    if (temp > 1500) // waited to long
        reconfig_gamepad();

    if (temp < read_delay) // waited too short
        delay(read_delay - temp);

    last_buttons = buttons; // store the previous buttons states

    if (motor2 != 0x00)
        motor2 = map(motor2, 0, 255, 0x40, 0xFF); // noting below 40 will make it spin

    cli(); //*** KJE ***
    SET(*commandOutputRegister, commandMask_);
    SET(*clockOuputRegister_, clockMask_);
    CLR(*attributeOutputRegister, attributeMask_); // low enable joystick
    SREG = old_sreg;                               // *** KJE *** - Interrupts may be enabled again

    delayMicroseconds(CTRL_BYTE_DELAY);
    // Send the command to send button and joystick data;
    char dword[9]   = { 0x01, 0x42, 0, motor1, motor2, 0, 0, 0, 0 };
    byte dword2[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    for (int i = 0; i < 9; i++) {
        PS2data[i] = _gamepad_shiftinout(dword[i]);
    }
    if (PS2data[1] == 0x79) { // if controller is in full data return mode, get the rest of data
        for (int i = 0; i < 12; i++) {
            PS2data[i + 9] = _gamepad_shiftinout(dword2[i]);
        }
    }

    cli();
    SET(*attributeOutputRegister, attributeMask_); // HI disable joystick
    SREG = old_sreg;                               // Interrupts may be enabled again

#ifdef PS2X_COM_DEBUG
    Serial.println("OUT:IN");
    for (int i = 0; i < 9; i++) {
        Serial.print(dword[i], HEX);
        Serial.print(":");
        Serial.print(PS2data[i], HEX);
        Serial.print(" ");
    }
    for (int i = 0; i < 12; i++) {
        Serial.print(dword2[i], HEX);
        Serial.print(":");
        Serial.print(PS2data[i + 9], HEX);
        Serial.print(" ");
    }
    Serial.println("");
#endif

    buttons   = *(uint16_t*)(PS2data + 3); // store as one value for multiple functions
    last_read = millis();
}

void PS2Controller::sendCommandString(byte string[], byte len)
{
    uint8_t old_sreg = SREG; // *** KJE *** save away the current state of interrupts

#ifdef PS2X_COM_DEBUG
    byte temp[len];
    cli();                      // *** KJE *** disable for now
    CLR(*_att_oreg, _att_mask); // low enable joystick
    SREG = old_sreg;            // *** *** KJE *** *** Interrupts may be enabled again

    for (int y = 0; y < len; y++)
        temp[y] = _gamepad_shiftinout(string[y]);

    cli();                      // *** KJE *** disable for now
    SET(*_att_oreg, _att_mask); // high disable joystick
    SREG = old_sreg;            // *** *** KJE *** *** Interrupts may be enabled again
    delay(read_delay);          // wait a few

    Serial.println("OUT:IN Configure");
    for (int i = 0; i < len; i++) {
        Serial.print(string[i], HEX);
        Serial.print(":");
        Serial.print(temp[i], HEX);
        Serial.print(" ");
    }
    Serial.println("");

#else
    cli();                                         // *** KJE *** disable for now
    CLR(*attributeOutputRegister, attributeMask_); // low enable joystick
    SREG = old_sreg;                               // *** *** KJE *** *** Interrupts may be enabled again
    for (int y = 0; y < len; y++)
        _gamepad_shiftinout(string[y]);

    cli();                                         // *** KJE *** disable for now
    SET(*attributeOutputRegister, attributeMask_); // high disable joystick
    SREG = old_sreg;                               // *** *** KJE *** *** Interrupts may be enabled again
    delay(read_delay);                             // wait a few
#endif
}

uint8_t PS2Controller::maskToBitNum(uint8_t mask)
{
    for (int y = 0; y < 8; y++) {
        if (CHK(mask, y))
            return y;
    }
    return 0;
}

byte PS2Controller::controllerType()
{
    if (controllerType_ == 0x03) {
        return 1;
    } else if (controllerType_ == 0x01) {
        return 2;
    }

    return 0;
}

void PS2Controller::enableRumble()
{
    sendCommandString(enter_config, sizeof(enter_config));
    sendCommandString(enable_rumble, sizeof(enable_rumble));
    sendCommandString(exit_config, sizeof(exit_config));
    enableRumble_ = true;
}

bool PS2Controller::enablePressures()
{
    sendCommandString(enter_config, sizeof(enter_config));
    sendCommandString(set_bytes_large, sizeof(set_bytes_large));
    sendCommandString(exit_config, sizeof(exit_config));

    read_gamepad();
    read_gamepad();

    if (PS2data[1] != 0x79)
        return false;

    enablePressures_ = true;
    return true;
}

void PS2Controller::reconfig_gamepad()
{
    sendCommandString(enter_config, sizeof(enter_config));
    sendCommandString(set_mode, sizeof(set_mode));
    if (enableRumble_)
        sendCommandString(enable_rumble, sizeof(enable_rumble));
    if (enablePressures_)
        sendCommandString(set_bytes_large, sizeof(set_bytes_large));
    sendCommandString(exit_config, sizeof(exit_config));
}
