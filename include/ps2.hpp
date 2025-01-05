/******************************************************************
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
License as published by the Free Software Foundation, either version 3 of the License, or(at your option) any later
version. This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
details. <http://www.gnu.org/licenses/>
*******************************************************************
The original library code is inside archive/OriginalPS2Lib folder.
******************************************************************/

// $$$$$$$$$$$$ DEBUG ENABLE SECTION $$$$$$$$$$$$$$$$
// to debug ps2 controller, uncomment these two lines to print out debug to uart
// #define PS2X_DEBUG
// #define PS2X_COM_DEBUG

#ifndef PS2X_lib_h
#define PS2X_lib_h

#include <Arduino.h>

// Regular buttons.
#define PSB_SELECT 0x0001u
#define PSB_L3 0x0002u
#define PSB_R3 0x0004u
#define PSB_START 0x0008u
#define PSB_PAD_UP 0x0010u
#define PSB_PAD_RIGHT 0x0020u
#define PSB_PAD_DOWN 0x0040u
#define PSB_PAD_LEFT 0x0080u
#define PSB_L2 0x0100u
#define PSB_R2 0x0200u
#define PSB_L1 0x0400u
#define PSB_R1 0x0800u
#define PSB_GREEN 0x1000u
#define PSB_RED 0x2000u
#define PSB_BLUE 0x4000u
#define PSB_PINK 0x8000u
#define PSB_TRIANGLE 0x1000u
#define PSB_CIRCLE 0x2000u
#define PSB_CROSS 0x4000u
#define PSB_SQUARE 0x8000u

// GuitarHero buttons.
#define PSG_GREEN_FRET 0x0200u
#define PSG_RED_FRET 0x2000u
#define PSG_YELLOW_FRET 0x1000u
#define PSG_BLUE_FRET 0x4000u
#define PSG_ORANGE_FRET 0x8000u
#define PSG_STAR_POWER 0x0100u
#define PSG_UP_STRUM 0x0010u
#define PSG_DOWN_STRUM 0x0040u
#define PSG_WHAMMY_BAR 8u

// Stick values.
#define PSS_RX 5u
#define PSS_RY 6u
#define PSS_LX 7u
#define PSS_LY 8u

// Analog buttons.
#define PSAB_PAD_RIGHT 9u
#define PSAB_PAD_UP 11u
#define PSAB_PAD_DOWN 12u
#define PSAB_PAD_LEFT 10u
#define PSAB_L2 19u
#define PSAB_R2 20u
#define PSAB_L1 17u
#define PSAB_R1 18u
#define PSAB_GREEN 13u
#define PSAB_RED 14u
#define PSAB_BLUE 15u
#define PSAB_PINK 16u
#define PSAB_TRIANGLE 13u
#define PSAB_CIRCLE 14u
#define PSAB_CROSS 15u
#define PSAB_SQUARE 16u

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

namespace commands {
inline constexpr byte startConfiguration[] = { 0x01, 0x43, 0x00, 0x01, 0x00 };
inline constexpr byte setMode[]            = { 0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00 };
inline constexpr byte setAuxData[]         = { 0x01, 0x4F, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00 };
inline constexpr byte stopConfiguration[]  = { 0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A };
inline constexpr byte enableRumble[]       = { 0x01, 0x4D, 0x00, 0x00, 0x01 };
inline constexpr byte readType[]           = { 0x01, 0x45, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A };
} // namespace commands

enum class ErrorCode : uint8_t
{
    Success,
    WrongControllerMode, // Controller mode not matched or no controller found.
    ControllerNotAcceptingCommands,
    PressureModeError
};

enum class ControllerType : uint8_t
{
    Unknown,
    DualShock,
    GuitarHero,
    WirelessDualShock
};

class Controller
{
public:
    ErrorCode      configure(uint8_t clockPin, uint8_t commandPin, uint8_t attentionPin, uint8_t dataPin);
    ErrorCode      configure(uint8_t clockPin,
                             uint8_t commandPin,
                             uint8_t attentionPin,
                             uint8_t dataPin,
                             bool    pressureMode,
                             bool    enableRumble);
    ControllerType type() const;
    bool           buttonPressed(uint16_t buttonId) const;
    bool           buttonsStateChanged() const;
    bool           buttonStateChanged(uint16_t buttonId) const;
    byte           analogButtonState(uint16_t buttonId) const;
    void           readData();
    void           readData(bool motor1, byte motor2);
    void           enableRumble();
    bool           enablePressures();

private: // methods
    ErrorCode setControllerMode(bool pressureMode, bool enableRumble);
    byte      sendByte(byte inputByte);
    void      sendCommandString(const byte string[], byte size);
    void      reconfigureController();
    uint8_t   maskToBitNum(uint8_t);

private: // data
    inline static constexpr unsigned long readPeriodUntilReconfiguration = 1500;
    inline static constexpr unsigned long controlDelayUs                 = 4;
    inline static constexpr unsigned long controlByteDelayUs             = 3;
    inline static constexpr uint8_t       baseDataSize                   = 9;
    inline static constexpr uint8_t       auxDataSize                    = 12;
    inline static constexpr uint8_t       correctMode1                   = 0x41;
    inline static constexpr uint8_t       correctMode2                   = 0x73;
    inline static constexpr uint8_t       correctMode3                   = 0x79;

    unsigned char  data_[baseDataSize + auxDataSize];
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
    ControllerType controllerType_;
    bool           enableRumble_;
    bool           pressureMode_;
};

} // namespace ps2

#endif // PS2X_lib_h
