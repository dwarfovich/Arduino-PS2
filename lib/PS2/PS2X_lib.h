/******************************************************************
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
License as published by the Free Software Foundation, either version 3 of the License, or(at your option) any later
version. This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
details. <http://www.gnu.org/licenses/>
******************************************************************/

// $$$$$$$$$$$$ DEBUG ENABLE SECTION $$$$$$$$$$$$$$$$
// to debug ps2 controller, uncomment these two lines to print out debug to uart
// #define PS2X_DEBUG
// #define PS2X_COM_DEBUG

#ifndef PS2X_lib_h
#define PS2X_lib_h

#include <Arduino.h>

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>

namespace ps2 {

}

#define CTRL_CLK 4
#define CTRL_BYTE_DELAY 3

// These are our button constants
#define PSB_SELECT 0x0001
#define PSB_L3 0x0002
#define PSB_R3 0x0004
#define PSB_START 0x0008
#define PSB_PAD_UP 0x0010
#define PSB_PAD_RIGHT 0x0020
#define PSB_PAD_DOWN 0x0040
#define PSB_PAD_LEFT 0x0080
#define PSB_L2 0x0100
#define PSB_R2 0x0200
#define PSB_L1 0x0400
#define PSB_R1 0x0800
#define PSB_GREEN 0x1000
#define PSB_RED 0x2000
#define PSB_BLUE 0x4000
#define PSB_PINK 0x8000
#define PSB_TRIANGLE 0x1000
#define PSB_CIRCLE 0x2000
#define PSB_CROSS 0x4000
#define PSB_SQUARE 0x8000

// Guitar  button constants
#define GREEN_FRET 0x0200
#define RED_FRET 0x2000
#define YELLOW_FRET 0x1000
#define BLUE_FRET 0x4000
#define ORANGE_FRET 0x8000
#define STAR_POWER 0x0100
#define UP_STRUM 0x0010
#define DOWN_STRUM 0x0040
#define WHAMMY_BAR 8

// These are stick values
#define PSS_RX 5
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8

// These are analog buttons
#define PSAB_PAD_RIGHT 9
#define PSAB_PAD_UP 11
#define PSAB_PAD_DOWN 12
#define PSAB_PAD_LEFT 10
#define PSAB_L2 19
#define PSAB_R2 20
#define PSAB_L1 17
#define PSAB_R1 18
#define PSAB_GREEN 13
#define PSAB_RED 14
#define PSAB_BLUE 15
#define PSAB_PINK 16
#define PSAB_TRIANGLE 13
#define PSAB_CIRCLE 14
#define PSAB_CROSS 15
#define PSAB_SQUARE 16

template<typename T>
void setBit(T& target, int bitNumber){
    target |= (1 << bitNumber);
}

template<typename T>
bool getBit(const T& target, int bitNumber){
    return (target & (1 << bitNumber));
}

template<typename T>
void clearBit(T& target, int bitNumber){
    target &= ~(1 << bitNumber);
}

template<typename T>
void toggleBit(T& target, int bitNumber){
    target ^= (1 << bitNumber);
}

#define SET(x, y) (x |= (1 << y))
#define CLR(x, y) (x &= (~(1 << y)))
#define CHK(x, y) (x & (1 << y))
#define TOG(x, y) (x ^= (1 << y))

class PS2Controller
{
public:
    byte configure(uint8_t clock, uint8_t command, uint8_t attribute, uint8_t data);
    byte configure(uint8_t clock, uint8_t command, uint8_t attribute, uint8_t data, bool pressures, bool rumble);
    bool buttonPressed(uint16_t buttonId);
    // bool buttonPressed(unsigned int buttonId);
    unsigned int ButtonDataByte();
    boolean      NewButtonState();
    boolean      NewButtonState(unsigned int);
    boolean      ButtonReleased(unsigned int);
    void         read_gamepad();
    void         read_gamepad(boolean, byte);
    byte         type() const;
    void         enableRumble();
    bool         enablePressures();
    byte         Analog(byte);

private: // methods
    int setControllerMode(bool countPressures, bool enableRumble);

private: // data
    unsigned char     _gamepad_shiftinout(char);
    unsigned char     PS2data[21];
    void              sendCommandString(byte *, byte);
    void              reconfig_gamepad();
    unsigned char     i;
    unsigned int      last_buttons;
    unsigned int      buttons;
    uint8_t           maskToBitNum(uint8_t);
    uint8_t           clockMask_;
    volatile uint8_t *clockOuputRegister_;
    uint8_t           commandMask_;
    volatile uint8_t *commandOutputRegister;
    uint8_t           attributeMask_;
    volatile uint8_t *attributeOutputRegister;
    uint8_t           dataMask_;
    volatile uint8_t *dataInputRegister_;
    unsigned long     last_read;
    byte              readDelay_;
    byte              controllerType_;
    bool              enableRumble_;
    bool              enablePressures_;
};

#endif // PS2X_lib_h
