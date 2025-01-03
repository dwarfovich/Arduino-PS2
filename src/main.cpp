#include "PS2X_lib.h"

#define PS2_DAT   12
#define PS2_CMD   11
#define PS2_SEL   10
#define PS2_CLK   13
#define ENABLE_PRESSURES   true
#define ENABLE_RUMBLE    true

PS2Controller ps2x;

int error=0;
byte controllerType=0;
byte vibrate=0;

constexpr int baudRate = 57600;
constexpr int serialMonitorStartDelay = 300;

void setup(){
  Serial.begin(57600);
  delay(serialMonitorStartDelay);
  error=ps2x.configure(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, ENABLE_PRESSURES, ENABLE_RUMBLE);
  if(error==0){
    Serial.println("Found Controller, configured successful ");
    Serial.println("pressures = ");
    if(ENABLE_PRESSURES) Serial.println("ture");
    else Serial.println("false");
    
    Serial.println("rumble = ");    
    if(ENABLE_RUMBLE) Serial.println("ture");
    else Serial.println("false");
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println("Note: Go to www.billporter.info for updates and to report bugs.");
  }
  else if(error==1){
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");    
  }
  else if(error==2){
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");  
  }
  else if(error==3){
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
  }
  
  controllerType=ps2x.type();
  switch(controllerType){
    case 0:   Serial.print("Unknown Controller type found ");   break;
    case 1:   Serial.print("DualShock Controller found ");    break;
    case 2:   Serial.print("GuitarHero Controller found ");   break;
    case 3:   Serial.print("Wireless Sony DualShock Controller found ");    break;
  }  
}

void loop(){
  if(error==1) return;
  if(error==2){
      ps2x.read_gamepad();
      if(ps2x.buttonPressed(GREEN_FRET))  Serial.println("Green Fret Pressed");
      if(ps2x.buttonPressed(RED_FRET))    Serial.println("Red Fret Pressed");
      if(ps2x.buttonPressed(YELLOW_FRET)) Serial.println("Yellow Fret Pressed");
      if(ps2x.buttonPressed(BLUE_FRET))   Serial.println("Blue Fret Pressed");
      if(ps2x.buttonPressed(ORANGE_FRET)) Serial.println("Orange Fret Pressed"); 

      if(ps2x.buttonPressed(STAR_POWER))  Serial.println("Star Power Command");

      if(ps2x.buttonPressed(UP_STRUM))    Serial.println("Up Strum");
      if(ps2x.buttonPressed(DOWN_STRUM))  Serial.println("DOWN Strum");

      if(ps2x.buttonPressed(PSB_START))    Serial.println("Start is being held");
      if(ps2x.buttonPressed(PSB_SELECT))   Serial.println("Select is being held");

      if(ps2x.buttonPressed(ORANGE_FRET)){
          Serial.print("Wammy Bar Position:");
          Serial.println(ps2x.Analog(WHAMMY_BAR), DEC);
      }
  }
  else{
    ps2x.read_gamepad(false, vibrate);
    if(ps2x.buttonPressed(PSB_START))      Serial.println("Start is being held");
    if(ps2x.buttonPressed(PSB_SELECT))     Serial.println("Select is being held");
    
    if(ps2x.buttonPressed(PSB_PAD_UP)){
      Serial.print("Up held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);  
    } 
    if(ps2x.buttonPressed(PSB_PAD_RIGHT)){
      Serial.print("Right held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
    }
    if(ps2x.buttonPressed(PSB_PAD_LEFT)){
      Serial.print("LEFT held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
    }
    if(ps2x.buttonPressed(PSB_PAD_DOWN)){
      Serial.print("DOWN held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
    }

    vibrate = ps2x.Analog(PSAB_CROSS);
    if (ps2x.NewButtonState()){
      if(ps2x.buttonPressed(PSB_L3))       Serial.println("L3 pressed");
      if(ps2x.buttonPressed(PSB_R3))       Serial.println("R3 pressed");
      if(ps2x.buttonPressed(PSB_L2))       Serial.println("L2 pressed");
      if(ps2x.buttonPressed(PSB_R2))       Serial.println("R2 pressed");

      if(ps2x.buttonPressed(PSB_GREEN)) Serial.println("GREEN pressed");
      if(ps2x.buttonPressed(PSB_RED)) Serial.println("RED pressed");
      if(ps2x.buttonPressed(PSB_BLUE)) Serial.println("BLUE pressed");
      if(ps2x.buttonPressed(PSB_PINK)) Serial.println("PINK pressed");
    }

    if(ps2x.buttonPressed(PSB_L1) || ps2x.buttonPressed(PSB_R1)){
      Serial.print("Stick Values:");
      Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX  
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_LX), DEC); 
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_RY), DEC);  
      Serial.print(",");
      Serial.println(ps2x.Analog(PSS_RX), DEC);
    }
  }
  delay(50);
}
