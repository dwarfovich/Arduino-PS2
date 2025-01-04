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
constexpr void setBit(T &target, int bitNumber)
{
    target |= (1 << bitNumber);
}

template<typename T>
constexpr bool getBit(const T &target, int bitNumber)
{
    return (target & (1 << bitNumber));
}

template<typename T>
constexpr void clearBit(T &target, int bitNumber)
{
    target &= ~(1 << bitNumber);
}

template<typename T>
constexpr void toggleBit(T &target, int bitNumber)
{
    target ^= (1 << bitNumber);
}

namespace ps2 {
namespace buttons{
    inline constexpr uint16_t select = 0x0001;
    inline constexpr uint16_t l3 = 0x0002;
    inline constexpr uint16_t r3 = 0x0004;
    inline constexpr uint16_t start = 0x0008;
    inline constexpr uint16_t padUp = 0x0010;
    inline constexpr uint16_t padRight = 0x0020;
    inline constexpr uint16_t padDown = 0x0040;
    inline constexpr uint16_t padLeft = 0x0080;
    inline constexpr uint16_t l2 = 0x0100;
    inline constexpr uint16_t r2 = 0x0200;
    inline constexpr uint16_t l1 = 0x0400;
    inline constexpr uint16_t r1 = 0x0800;
    inline constexpr uint16_t green = 0x1000;
    inline constexpr uint16_t red = 0x2000;
    inline constexpr uint16_t blue = 0x4000;
    inline constexpr uint16_t pink = 0x8000;
    inline constexpr const uint16_t& triangle = green;
    inline constexpr const uint16_t circle = red;
    inline constexpr const uint16_t cross = blue;
    inline constexpr const uint16_t square = pink;
}

namespace stick{
    inline constexpr uint16_t rx = 5;
    inline constexpr uint16_t ry = 6;
    inline constexpr uint16_t lx = 7;
    inline constexpr uint16_t ly = 8;
}

namespace analog_buttons{
    inline constexpr uint16_t padRight = 9;
    inline constexpr uint16_t padLeft = 10;
    inline constexpr uint16_t padUp = 11;
    inline constexpr uint16_t padDown = 12;
    inline constexpr uint16_t l2 = 19;
    inline constexpr uint16_t r2 = 20;
    inline constexpr uint16_t l1 = 17;
    inline constexpr uint16_t r1 = 18;
    inline constexpr uint16_t green = 13;
    inline constexpr uint16_t red = 14;
    inline constexpr uint16_t blue = 15;
    inline constexpr uint16_t pink = 16;
    inline constexpr const uint16_t& triangle = green;
    inline constexpr const uint16_t circle = red;
    inline constexpr const uint16_t cross = blue;
    inline constexpr const uint16_t square = pink;
}

namespace commands {
inline constexpr byte startConfiguration[] = { 0x01, 0x43, 0x00, 0x01, 0x00 };
inline constexpr byte setMode[]            = { 0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00 };
inline constexpr byte setAuxData[]         = { 0x01, 0x4F, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00 };
inline constexpr byte stopConfiguration[]  = { 0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A };
inline constexpr byte enableRumble[]       = { 0x01, 0x4D, 0x00, 0x00, 0x01 };
inline constexpr byte readType[]           = { 0x01, 0x45, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A };
} // namespace commands

class Controller
{
public:
    byte configure(uint8_t clockPin, uint8_t commandPin, uint8_t attentionPin, uint8_t dataPin);
    byte configure(uint8_t clockPin,
                   uint8_t commandPin,
                   uint8_t attentionPin,
                   uint8_t dataPin,
                   bool    pressureMode,
                   bool    enableRumble);
    byte type() const;
    bool buttonPressed(uint16_t buttonId) const;
    bool buttonsStateChanged();
    bool buttonStateChanged(unsigned int buttonId);
    void readData();
    void readData(bool motor1, byte motor2);
    void enableRumble();
    bool enablePressures();
    byte analogButtonState(byte buttonId);

private: // methods
    int     setControllerMode(bool countPressures, bool enableRumble);
    byte    sendByte(byte inputByte);
    void    sendCommandString(const byte string[], byte size);
    void    reconfigureController();
    uint8_t maskToBitNum(uint8_t);

private: // data
    inline static constexpr unsigned long controlDelayUs     = 4;
    inline static constexpr unsigned long controlByteDelayUs = 3;
    inline static constexpr uint8_t       baseDataSize_      = 9;
    inline static constexpr uint8_t       auxDataSize_       = 12;

    unsigned char  data_[baseDataSize_ + auxDataSize_];
    unsigned int   previousButtonsState_;
    unsigned int   buttonsState_;
    byte           clockMask_;
    volatile byte *clockOuputRegister_;
    byte           commandMask_;
    volatile byte *commandOutputRegister_;
    byte           attentionMask_;
    volatile byte *attentionOutputRegister_;
    byte           dataMask_;
    volatile byte *dataInputRegister_;
    unsigned long  lastDataReadTimestamp_;
    byte           readDelay_;
    byte           controllerType_;
    bool           enableRumble_;
    bool           pressureMode_;
};

} // namespace ps2

#endif // PS2X_lib_h
