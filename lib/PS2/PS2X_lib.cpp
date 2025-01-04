#include "PS2X_lib.h"

#include <avr/io.h>
#include <pins_arduino.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>

static constexpr byte enter_config[]    = { 0x01, 0x43, 0x00, 0x01, 0x00 };
static constexpr byte set_mode[]        = { 0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00 };
static constexpr byte set_bytes_large[] = { 0x01, 0x4F, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00 };
static constexpr byte exit_config[]     = { 0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A };
static constexpr byte enable_rumble[]   = { 0x01, 0x4D, 0x00, 0x00, 0x01 };
static constexpr byte readTypeCommand[] = { 0x01, 0x45, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A };

byte PS2Controller::configure(uint8_t clockPin, uint8_t commandPin, uint8_t attentionPin, uint8_t dataPin)
{
    return configure(clockPin, commandPin, attentionPin, dataPin, false, false);
}

byte PS2Controller::configure(
    uint8_t clockPin, uint8_t commandPin, uint8_t attentionPin, uint8_t dataPin, bool pressureMode, bool enableRumble)
{
    const uint8_t oldSreg = SREG;

    clockMask_               = maskToBitNum(digitalPinToBitMask(clockPin));
    clockOuputRegister_      = portOutputRegister(digitalPinToPort(clockPin));
    commandMask_             = maskToBitNum(digitalPinToBitMask(commandPin));
    commandOutputRegister_   = portOutputRegister(digitalPinToPort(commandPin));
    attentionMask_           = maskToBitNum(digitalPinToBitMask(attentionPin));
    attentionOutputRegister_ = portOutputRegister(digitalPinToPort(attentionPin));
    dataMask_                = maskToBitNum(digitalPinToBitMask(dataPin));
    dataInputRegister_       = portInputRegister(digitalPinToPort(dataPin));

    // Configure pins.
    pinMode(clockPin, OUTPUT);
    pinMode(attentionPin, OUTPUT);
    pinMode(commandPin, OUTPUT);
    pinMode(dataPin, INPUT);
    digitalWrite(dataPin, HIGH); // enable pull-up

    cli();
    setBit(*commandOutputRegister_, commandMask_);
    setBit(*clockOuputRegister_, clockMask_);
    SREG = oldSreg;

    // Error checking: reading controller's data for a few times, at the end PS2data[1] should be one of the values: 41,
    // 73 or 79.
    readData();
    readData();
    if (data_[1] != 0x41 && data_[1] != 0x73 && data_[1] != 0x79) {
#ifdef PS2X_DEBUG
        Serial.println("Controller mode not matched or no controller found");
        Serial.print("Expected 0x41, 0x73 or 0x79, got ");
        Serial.println(PS2data[1], HEX);
#endif
        return 1;
    }

    return setControllerMode(pressureMode, enableRumble);
}

int PS2Controller::setControllerMode(bool pressureMode, bool enableRumble)
{
    byte answer[sizeof(readTypeCommand)];
    readDelay_            = 1; // readDelay_ will be saved to use later when reading data from controller.
    const uint8_t oldSreg = SREG;
    for (uint8_t attempt = 0; attempt <= 10; ++attempt) {
        sendCommandString(enter_config, sizeof(enter_config)); // start config run
        delayMicroseconds(controlByteDelayUs);

        cli();
        setBit(*commandOutputRegister_, commandMask_);
        setBit(*clockOuputRegister_, clockMask_);
        clearBit(*attentionOutputRegister_, attentionMask_); // low enable joystick
        SREG = oldSreg;

        delayMicroseconds(controlByteDelayUs);

        for (uint8_t j = 0; j < sizeof(readTypeCommand); ++j) {
            answer[j] = sendByte(readTypeCommand[j]);
        }

        cli();
        setBit(*attentionOutputRegister_, attentionMask_); // HI disable joystick
        SREG = oldSreg;

        controllerType_ = answer[3];

        sendCommandString(set_mode, sizeof(set_mode));
        if (enableRumble) {
            sendCommandString(enable_rumble, sizeof(enable_rumble));
            enableRumble_ = true;
        }
        if (pressureMode) {
            sendCommandString(set_bytes_large, sizeof(set_bytes_large));
            pressureMode = true;
        }
        sendCommandString(exit_config, sizeof(exit_config));

        readData();

        if (pressureMode) {
            if (data_[1] == 0x79)
                break;
            if (data_[1] == 0x73)
                return 3;
        }

        if (data_[1] == 0x73)
            break;

        if (attempt == 10) {
#ifdef PS2X_DEBUG
            Serial.println("Controller not accepting commands");
            Serial.print("mode stil set at");
            Serial.println(PS2data[1], HEX);
#endif
            return 2;
        }

        readDelay_ += 1;
    }

    return 0;
}

boolean PS2Controller::buttonPressed(uint16_t button) const
{
    return ((~buttonsState_ & button) > 0);
}

// boolean PS2Controller::buttonPressed(unsigned int button)
// {
//     return (NewButtonState(button) & buttonPressed(button));
// }

boolean PS2Controller::NewButtonState()
{
    return ((previousButtonsState_ ^ buttonsState_) > 0);
}

boolean PS2Controller::NewButtonState(unsigned int button)
{
    return (((previousButtonsState_ ^ buttonsState_) & button) > 0);
}

boolean PS2Controller::ButtonReleased(unsigned int button)
{
    return ((NewButtonState(button)) & ((~previousButtonsState_ & button) > 0));
}

unsigned int PS2Controller::ButtonDataByte()
{
    return (~buttonsState_);
}

byte PS2Controller::Analog(byte button)
{
    return data_[button];
}

byte PS2Controller::sendByte(byte inputByte)
{
    const uint8_t oldSreg = SREG;
    uint8_t       result  = 0;
    cli();
    for (int i = 0; i < 8; ++i) {
        if (getBit(inputByte, i)) {
            setBit(*commandOutputRegister_, commandMask_);
        } else {
            clearBit(*commandOutputRegister_, commandMask_);
        }
        clearBit(*clockOuputRegister_, clockMask_);

        SREG = oldSreg;
        delayMicroseconds(controlDelayUs);
        cli();

        if (getBit(*dataInputRegister_, dataMask_)) {
            setBit(result, i);
        }
        setBit(*clockOuputRegister_, clockMask_);
    }
    setBit(*commandOutputRegister_, commandMask_);
    SREG = oldSreg;
    delayMicroseconds(controlByteDelayUs);

    return result;
}

void PS2Controller::readData()
{
    readData(false, 0);
}

void PS2Controller::readData(boolean motor1, byte motor2)
{
    uint8_t old_sreg = SREG;

    const unsigned long msSinceLastReading = millis() - lastDataReadTimestamp_;
    if (msSinceLastReading > 1500) { // Waited too long, reconfiguration needed.
        reconfigureController();
    }
    if (msSinceLastReading < readDelay_) { // Waited too short.
        delay(readDelay_ - msSinceLastReading);
    }

    previousButtonsState_ = buttonsState_;

    cli();
    setBit(*commandOutputRegister_, commandMask_);
    setBit(*clockOuputRegister_, clockMask_);
    clearBit(*attentionOutputRegister_, attentionMask_); // low enable joystick
    SREG = old_sreg;
    delayMicroseconds(controlByteDelayUs);

    // Send the command to send button and joystick data;
    if (motor2) {
        motor2 = map(motor2, 0, 0xFF, 0x40, 0xFF); // Values lower than 0x40 will not trigger motor.
    }
    byte command[baseDataSize_] = { 0x01, 0x42, 0, motor1, motor2, 0, 0, 0, 0 };

    for (uint8_t i = 0; i < baseDataSize_; ++i) {
        data_[i] = sendByte(command[i]);
    }
    if (data_[1] == 0x79) { // if controller is in full data return mode, get the rest of data
        for (uint8_t i = 0; i < auxDataSize_; ++i) {
            data_[i + 9] = sendByte(0);
        }
    }

    cli();
    setBit(*attentionOutputRegister_, attentionMask_); // HI disable joystick
    SREG = old_sreg;

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

    buttonsState_          = *(decltype(buttonsState_)*)(data_ + 3); // store as one value for multiple functions
    lastDataReadTimestamp_ = millis();
}

void PS2Controller::sendCommandString(const byte string[], uint8_t size)
{
    const uint8_t oldSreg = SREG;
#ifdef PS2X_COM_DEBUG
    byte temp[size];
    cli();                                               // *** KJE *** disable for now
    clearBit(*attentionOutputRegister_, attentionMask_); // low enable joystick
    SREG = oldSreg;                                      // *** *** KJE *** *** Interrupts may be enabled again

    for (int y = 0; y < size; ++y)
        temp[y] = _gamepad_shiftinout(string[y]);

    cli();                                          // *** KJE *** disable for now
    SET(*attentionOutputRegister_, attentionMask_); // high disable joystick
    SREG = oldSreg;                                 // *** *** KJE *** *** Interrupts may be enabled again
    delay(readDelay_);                              // wait a few

    Serial.println("OUT:IN Configure");
    for (uint8_t i = 0; i < size; ++i) {
        Serial.print(string[i], HEX);
        Serial.print(":");
        Serial.print(temp[i], HEX);
        Serial.print(" ");
    }
    Serial.println("");

#else
    cli();
    clearBit(*attentionOutputRegister_, attentionMask_); // low enable joystick
    SREG = oldSreg;
    for (uint8_t i = 0; i < size; ++i) {
        sendByte(string[i]);
    }

    cli();
    setBit(*attentionOutputRegister_, attentionMask_); // high disable joystick
    SREG = oldSreg;
    delay(readDelay_);
#endif
}

uint8_t PS2Controller::maskToBitNum(uint8_t mask)
{
    for (uint8_t i = 0; i < 8; ++i) {
        if (getBit(mask, i)) {
            return i;
        }
    }
    return 0;
}

byte PS2Controller::type() const
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

    readData();
    //readData();

    if (data_[1] != 0x79)
        return false;

    pressureMode_ = true;
    return true;
}

void PS2Controller::reconfigureController()
{
    sendCommandString(enter_config, sizeof(enter_config));
    sendCommandString(set_mode, sizeof(set_mode));
    if (enableRumble_) {
        sendCommandString(enable_rumble, sizeof(enable_rumble));
    }
    if (pressureMode_) {
        sendCommandString(set_bytes_large, sizeof(set_bytes_large));
    }
    sendCommandString(exit_config, sizeof(exit_config));
}
