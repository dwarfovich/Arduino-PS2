#include "PS2X_lib.h"

namespace ps2_gamepad {
constexpr int selectPin  = 10;
constexpr int commandPin = 11;
constexpr int dataPin    = 12;
constexpr int clockPin   = 13;
} // namespace ps2_gamepad

constexpr bool countPressures          = true;
constexpr bool enableRumble            = true;
constexpr int  stopLoopDelay           = 500;
constexpr int  readButtonsDelay        = 50;
constexpr int  baudRate                = 57600;
constexpr int  startSerialMonitorDelay = 300;

void stop();
void printWelcomeMessage();

PS2Controller controller;
int  configurationError = 0;
byte vibrate            = 0;

void setup()
{
    Serial.begin(baudRate);
    delay(startSerialMonitorDelay);
    configurationError = controller.configure(ps2_gamepad::clockPin,
                                             ps2_gamepad::commandPin,
                                             ps2_gamepad::selectPin,
                                             ps2_gamepad::dataPin,
                                             countPressures,
                                             enableRumble);

    switch (configurationError) {
        case 0:
            printWelcomeMessage();
            break;
        case 1:
            Serial.println(
                "No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info "
                "for troubleshooting tips");
            return;
        case 2:
            Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit "
                           "www.billporter.info for troubleshooting tips");
            return;
        case 3:
            Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
            break;
    }

    const auto controllerType = controller.controllerType_();
    switch (controllerType) {
        case 1: Serial.print("DualShock Controller found "); break;
        case 2: Serial.print("GuitarHero Controller found "); break;
        case 3: Serial.print("Wireless Sony DualShock Controller found "); break;
        default: Serial.print("Unknown Controller type found "); break;
    }
}

void loop()
{
    if (configurationError == 1) {
        Serial.println("Stopping main loop due errors.");
        stop();
    } 
    
    if (configurationError == 2) {
        controller.read_gamepad();
        if (controller.buttonPressed(GREEN_FRET))
            Serial.println("Green Fret Pressed");
        if (controller.buttonPressed(RED_FRET))
            Serial.println("Red Fret Pressed");
        if (controller.buttonPressed(YELLOW_FRET))
            Serial.println("Yellow Fret Pressed");
        if (controller.buttonPressed(BLUE_FRET))
            Serial.println("Blue Fret Pressed");
        if (controller.buttonPressed(ORANGE_FRET))
            Serial.println("Orange Fret Pressed");

        if (controller.buttonPressed(STAR_POWER))
            Serial.println("Star Power Command");

        if (controller.buttonPressed(UP_STRUM))
            Serial.println("Up Strum");
        if (controller.buttonPressed(DOWN_STRUM))
            Serial.println("DOWN Strum");

        if (controller.buttonPressed(PSB_START))
            Serial.println("Start is being held");
        if (controller.buttonPressed(PSB_SELECT))
            Serial.println("Select is being held");

        if (controller.buttonPressed(ORANGE_FRET)) {
            Serial.print("Wammy Bar Position:");
            Serial.println(controller.Analog(WHAMMY_BAR), DEC);
        }
    } else {
        controller.read_gamepad(false, vibrate);
        if (controller.buttonPressed(PSB_START))
            Serial.println("Start is being held");
        if (controller.buttonPressed(PSB_SELECT))
            Serial.println("Select is being held");

        if (controller.buttonPressed(PSB_PAD_UP)) {
            Serial.print("Up held this hard: ");
            Serial.println(controller.Analog(PSAB_PAD_UP), DEC);
        }
        if (controller.buttonPressed(PSB_PAD_RIGHT)) {
            Serial.print("Right held this hard: ");
            Serial.println(controller.Analog(PSAB_PAD_RIGHT), DEC);
        }
        if (controller.buttonPressed(PSB_PAD_LEFT)) {
            Serial.print("LEFT held this hard: ");
            Serial.println(controller.Analog(PSAB_PAD_LEFT), DEC);
        }
        if (controller.buttonPressed(PSB_PAD_DOWN)) {
            Serial.print("DOWN held this hard: ");
            Serial.println(controller.Analog(PSAB_PAD_DOWN), DEC);
        }

        vibrate = controller.Analog(PSAB_CROSS);
        if (controller.NewButtonState()) {
            if (controller.buttonPressed(PSB_L3))
                Serial.println("L3 pressed");
            if (controller.buttonPressed(PSB_R3))
                Serial.println("R3 pressed");
            if (controller.buttonPressed(PSB_L2))
                Serial.println("L2 pressed");
            if (controller.buttonPressed(PSB_R2))
                Serial.println("R2 pressed");

            if (controller.buttonPressed(PSB_GREEN))
                Serial.println("GREEN pressed");
            if (controller.buttonPressed(PSB_RED))
                Serial.println("RED pressed");
            if (controller.buttonPressed(PSB_BLUE))
                Serial.println("BLUE pressed");
            if (controller.buttonPressed(PSB_PINK))
                Serial.println("PINK pressed");
        }

        if (controller.buttonPressed(PSB_L1) || controller.buttonPressed(PSB_R1)) {
            Serial.print("Stick Values:");
            Serial.print(controller.Analog(PSS_LY), DEC); // Left stick, Y axis. Other options: LX, RY, RX
            Serial.print(",");
            Serial.print(controller.Analog(PSS_LX), DEC);
            Serial.print(",");
            Serial.print(controller.Analog(PSS_RY), DEC);
            Serial.print(",");
            Serial.println(controller.Analog(PSS_RX), DEC);
        }
    }
    delay(readButtonsDelay);
}

void stop()
{
    while (true) {
        delay(stopLoopDelay);
    }
}

void printWelcomeMessage()
{
    Serial.println("Found Controller, configured successful ");
    Serial.println("pressures = ");
    if (countPressures) {
        Serial.println("ture");
    } else {
        Serial.println("false");
    }

    Serial.println("rumble = ");
    if (enableRumble) {
        Serial.println("true");
    } else {
        Serial.println("false");
    }
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println("Note: Go to www.billporter.info for updates and to report bugs.");
}