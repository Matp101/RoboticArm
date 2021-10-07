#include "include\VarSpeedServo.h"

//Push Buttons
#define DecreasePBNO 53
#define IncreasePBNO 51
#define ServoSelectPBNO 49
#define ConvFWDPBNO 45
#define ConvREVPBNO 47
#define StopPBNO 43
#define StartPBNO 41
#define ManPBNO 39
#define AutoPBNO 37

//LEDs
#define RunningLED 23
#define ManLED 25
#define AutoLED 29
#define StopLED 35
#define Servo1LED 40
#define Servo2LED 33
#define Servo3LED 31
#define Servo4LED 38

//Stepper Motor
#define ConvEn 46
#define ConvDir 48
#define ConvPulse 5

#define Electromagnet 52

#define BaseServo 30
#define LowerServo 32
#define UpperServo 34
#define HeadServo 36

#define ConvProxStartNO 22
#define ConvProxMidNO 24
#define ConvProxEndNO 26

#define ConvyorON LOW
#define ConvyorOFF HIGH
#define ConvyorREV LOW
#define ConvyorFWD HIGH
#define ConvyorPWMON 127
#define ConvyorPWMOFF 0

VarSpeedServo BaseRCServo;
VarSpeedServo LowerRCServo;
VarSpeedServo UpperRCServo;
VarSpeedServo HeadRCServo;

int IncreasingServo = 0;
int DecreasingServo = 0;
int ServoSelectorVal = 1;
int ServoSelectorButtonState = LOW;
int ServoSelectorButtonLastState = LOW;
int Servo1State = 90;
int Servo2State = 90;
int Servo3State = 90;
int Servo4State = 90;
int PartOnConv = 0;

int ManualPBState = LOW;
int AutoPBState = LOW;
int StopPBState = LOW;
int StartPBState = LOW;
int RunningState = LOW;
int ManMode = LOW;
int AutoMode = LOW;
int ConvProxStartLastState = LOW;

int servostep = 2;

int BaseServoisMoving = LOW;
int LowerServoisMoving = LOW;
int UpperServoisMoving = LOW;
int HeadServoisMoving = LOW;
int ServoDone = LOW;

int endproxlaststate = LOW;
int holdingpart = LOW;
int stoptorunning = HIGH;
int Servo3Max = 0;
int i = 1;
int switchedtoauto = LOW;
int armready = LOW;
int stoplaststate = LOW;
int electromagnetlaststate = HIGH;
int Insequence = 0;

unsigned long cyclestarttime = 0;

int erroron = 0;
int erroroff = 0;
int blinkamount = 0;
int cleanconveyor = 0;

unsigned long ProxDebouncetime = 0;
unsigned long DebounceDelay = 25;

int finishscan = LOW;

const int PartLocationLength = 200;
int PartLocation[PartLocationLength];


void setup() {

  //Onboard Power LED
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);

  //Initializing All Inputs
  pinMode(IncreasePBNO, INPUT);
  pinMode(DecreasePBNO, INPUT);
  pinMode(ServoSelectPBNO, INPUT);
  pinMode(ConvFWDPBNO, INPUT);
  pinMode(ConvREVPBNO, INPUT);
  pinMode(ManPBNO, INPUT);
  pinMode(AutoPBNO, INPUT);
  pinMode(StartPBNO, INPUT);
  pinMode(StopPBNO, INPUT);
  pinMode(ConvProxStartNO, INPUT);
  pinMode(ConvProxMidNO, INPUT);
  pinMode(ConvProxEndNO, INPUT);

  pinMode(ConvEn, OUTPUT);
  pinMode(ConvDir, OUTPUT);
  pinMode(ConvPulse, OUTPUT);
  pinMode(RunningLED, OUTPUT);
  pinMode(ManLED, OUTPUT);
  pinMode(AutoLED, OUTPUT);
  pinMode(StopLED, OUTPUT);
  pinMode(Servo1LED, OUTPUT);
  pinMode(Servo2LED, OUTPUT);
  pinMode(Servo3LED, OUTPUT);
  pinMode(Servo4LED, OUTPUT);
  pinMode(Electromagnet, OUTPUT);

  //Set Default Outputs
  digitalWrite(ConvEn, LOW);
  digitalWrite(Electromagnet, HIGH);
  digitalWrite(RunningLED, LOW);
  digitalWrite(ManLED, LOW);
  digitalWrite(AutoLED, LOW);
  digitalWrite(StopLED, HIGH);
  
  Serial.begin(9600);
}

void loop() {
  //Cycle time Couter and Debounce
  cyclestarttime = millis();

  //Read button inputs
  ManualPBState = digitalRead(ManPBNO);
  AutoPBState = digitalRead(AutoPBNO);
  StopPBState = digitalRead(StopPBNO);
  StartPBState = digitalRead(StartPBNO);

  //Start Machine if START PB is pressed
  if(StartPBState == HIGH && StopPBState == LOW){
    digitalWrite(StopLED, LOW);
    RunningState = HIGH; 
    digitalWrite(RunningLED, HIGH);
    
    //if it was in OFF state, Reset everything
    if(stoptorunning == HIGH){
      BaseRCServo.attach(BaseServo);
      LowerRCServo.attach(LowerServo);
      UpperRCServo.attach(UpperServo);
      HeadRCServo.attach(HeadServo);
      Servo1State = 100;
      Servo2State = 110;
      Servo3State = 180;
      Servo4State = 0;
      BaseRCServo.write(Servo1State);
      LowerRCServo.write(Servo2State);
      UpperRCServo.write(Servo3State);
      HeadRCServo.write(Servo4State);
      stoptorunning = LOW;
    }
  }
  
  //Turn off Machine if STOP PB pressed
  if(StopPBState == HIGH){
    digitalWrite(RunningLED, LOW);
    digitalWrite(AutoLED, LOW);
    digitalWrite(ManLED, LOW);
    digitalWrite(Servo1LED, LOW);
    digitalWrite(Servo2LED, LOW);
    digitalWrite(Servo3LED, LOW);
    digitalWrite(Servo4LED, LOW);
    digitalWrite(StopLED, HIGH);
    AutoMode = LOW;
    ManMode = LOW;
    RunningState = LOW;  
    stoptorunning = HIGH;
    ConvyorControl(ConvyorOFF, ConvyorFWD, ConvyorPWMOFF);
    digitalWrite(Electromagnet, HIGH);
    BaseRCServo.stop();
    LowerRCServo.stop();
    UpperRCServo.stop();
    HeadRCServo.stop();
  }

  //If Machine is running
  if(RunningState == HIGH){
    
    //Change Machine to Manual Mode
    if(ManualPBState == HIGH && AutoPBState == LOW){
      digitalWrite(AutoLED, LOW);
      AutoMode = LOW;
      digitalWrite(ManLED, HIGH);
      ManMode = HIGH;
    }
    
    //Change Machine to Auto Mode
    else if(AutoPBState == HIGH && ManualPBState == LOW && digitalRead(ConvProxEndNO) == LOW && AutoMode == LOW){
      digitalWrite(ManLED, LOW);
      ManMode = LOW;
      digitalWrite(Servo1LED, LOW);
      digitalWrite(Servo2LED, LOW);
      digitalWrite(Servo3LED, LOW);
      digitalWrite(Servo4LED, LOW);
      digitalWrite(AutoLED, HIGH);
      AutoMode = HIGH;
      armready = HIGH;
      switchedtoauto = HIGH;
      Insequence = 0;
    }

    //If Conveyor is not Clear and Auto is pressed
    else if(AutoPBState == HIGH && ManualPBState == LOW && digitalRead(ConvProxEndNO) == HIGH && AutoMode == LOW){
          cleanconveyor = 1;
    }
    
    //Change machine to No mode selected if both buttons pressed
    else if(AutoPBState == HIGH && ManualPBState == HIGH){
    digitalWrite(AutoLED, LOW);
    digitalWrite(ManLED, LOW);
    digitalWrite(Servo1LED, LOW);
    digitalWrite(Servo2LED, LOW);
    digitalWrite(Servo3LED, LOW);
    digitalWrite(Servo4LED, LOW);
    AutoMode = LOW;
    ManMode = LOW;
    ConvyorControl(ConvyorOFF, ConvyorFWD, ConvyorPWMOFF);
    }
    
    //Run Manual Mode
    if (ManMode == HIGH){
      manualmode();
    }
    
    //Run Auto Mode
    else if (AutoMode == HIGH){
      automode();
    }

    //Run conveyor Not Clean Blink Lights
    if (cleanconveyor == 1){
      cleanconv();
    }
  }

  //Delay needed to make smooth operation
  delay(20);
  finishscan = LOW;

  //Uncomment to Allow Debuging show Serial Monitor
  /*
   * debugging();
   * Serial.print(millis() - cyclestarttime);
  */
}

void cleanconv(){
  //How long lights should toggle for
  int timedelay = 12;

  //Turn Lights ON For specified time
  if(erroroff == 0 && erroron != 30){
    digitalWrite(Servo1LED, HIGH);
    digitalWrite(Servo2LED, HIGH);
    digitalWrite(Servo3LED, HIGH);
    digitalWrite(Servo4LED, HIGH);
    digitalWrite(AutoLED, HIGH);
    erroron++;
  }
  
  //Turn Lights OFF For specified time
  if(erroron == timedelay){
    digitalWrite(Servo1LED, LOW);
    digitalWrite(Servo2LED, LOW);
    digitalWrite(Servo3LED, LOW);
    digitalWrite(Servo4LED, LOW);
    digitalWrite(AutoLED, LOW);
    erroroff++;
  }

  if(erroroff == timedelay){
    erroron = 0;
    erroroff = 0;
    blinkamount++;
  }
  if(blinkamount == 2){
    cleanconveyor = 0;
    blinkamount = 0;
  }
}

void automode(){
  //If there is a part at end, stop conveyor
  if(digitalRead(ConvProxEndNO) == HIGH){
    ConvyorControl(ConvyorOFF, ConvyorFWD, ConvyorPWMOFF);
  }
  else{
    ConvyorControl(ConvyorON, ConvyorFWD, ConvyorPWMON); 
  }
  
  //Check to See if any Servos are still in Motion
  BaseServoisMoving = BaseRCServo.isMoving();
  LowerServoisMoving = LowerRCServo.isMoving();
  UpperServoisMoving = UpperRCServo.isMoving();
  HeadServoisMoving = HeadRCServo.isMoving();
  if(BaseServoisMoving == HIGH || LowerServoisMoving == HIGH || UpperServoisMoving == HIGH || HeadServoisMoving == HIGH){
    ServoDone = LOW;
  }else{
    ServoDone = HIGH;
  }

  //If Part has reached end of Conveyor start sequence of picking part up
  if(digitalRead(ConvProxEndNO) == HIGH && endproxlaststate == LOW){
    if(ProxDebouncetime == 0 || (millis() - ProxDebouncetime) > DebounceDelay){
      Insequence++;
    }
  }
  endproxlaststate = digitalRead(ConvProxEndNO);
  
  //If Servos had made it to New Position, Go to Next Position
  if(ServoDone == HIGH){
    if(Insequence >= 1){
      ArmPosEnd(i);
      //Turn on Electromagnet when on First Sequence
      if(i == 1){
        digitalWrite(Electromagnet, LOW);
        holdingpart = HIGH;
      }
      if(i == 2){
        ProxDebouncetime = millis();
      }
      if(i == 3){
        ProxDebouncetime = 0;
      }
      //Turn off Electromagent when on Last Sequence
      else if(i == 5){
        digitalWrite(Electromagnet, HIGH);
        holdingpart = LOW;
        Insequence--;
        i = 0;
        
      }
      i++;          
    }
  }
}

void manualmode() {
  //Read Button State
  IncreasingServo = digitalRead(IncreasePBNO);
  DecreasingServo = digitalRead(DecreasePBNO);
  ServoSelectorButtonState = digitalRead(ServoSelectPBNO);

  //Change to Next Servo
  if(ServoSelectorButtonState == HIGH && ServoSelectorButtonLastState == LOW && IncreasingServo == LOW && DecreasingServo == LOW){
    ServoSelectorVal++;
    if(ServoSelectorVal == 5){
      ServoSelectorVal = 1;
    }
  }
  ServoSelectorButtonLastState = ServoSelectorButtonState;
  //Update Servo Selected LED
  ServoLEDSelect();

  //Servo Motions
  if (IncreasingServo == HIGH || DecreasingServo == HIGH && ServoSelectorButtonState == LOW){
      Servomotion();
  }
  
  //Conveyor Belt
  ConvControl();
}

void ServoLEDSelect(){
  //Display the Selected Servo
  switch(ServoSelectorVal){
    case 1:
      digitalWrite(Servo1LED, HIGH);
      digitalWrite(Servo2LED, LOW);
      digitalWrite(Servo3LED, LOW);
      digitalWrite(Servo4LED, LOW);
    break;
    
    case 2:
      digitalWrite(Servo1LED, LOW);
      digitalWrite(Servo2LED, HIGH);
      digitalWrite(Servo3LED, LOW);
      digitalWrite(Servo4LED, LOW);    
    break;
    
    case 3:
      digitalWrite(Servo1LED, LOW);
      digitalWrite(Servo2LED, LOW);
      digitalWrite(Servo3LED, HIGH);
      digitalWrite(Servo4LED, LOW);    
    break;
    
    case 4:
      digitalWrite(Servo1LED, LOW);
      digitalWrite(Servo2LED, LOW);
      digitalWrite(Servo3LED, LOW);
      digitalWrite(Servo4LED, HIGH);    
    break;
  }
}

void Servomotion() {
  //Determine Max of Servo 3 so arm cannot hit conveyor
  Servo3Max = Servo2State * 2 + 48;

  //Determine which servo is selected and increase od decrease
  switch(ServoSelectorVal){
    case 1:
      if (IncreasingServo == HIGH && DecreasingServo == LOW){
        if (Servo1State < 180){
          Servo1State = Servo1State + servostep;
        }
      }
      else if (DecreasingServo == HIGH && IncreasingServo == LOW){
        if (Servo1State > 0){
          Servo1State = Servo1State - servostep;
        }
      }
      break;

    //reversed due to reverse Servo
    case 2:
      if (IncreasingServo == LOW && DecreasingServo == HIGH){
        if (Servo2State < 150){
          Servo2State = Servo2State + servostep;
        }
      }
      else if (DecreasingServo == LOW && IncreasingServo == HIGH){
        if (Servo2State > 40){
          if(Servo2State < 60 && Servo1State <= 146){
            if(Servo3Max > Servo3State){
              Servo2State = Servo2State - servostep;
            }
          }
          else{
             Servo2State = Servo2State - servostep; 
          }
        }
      }
      break;
    
    case 3:   
      if (IncreasingServo == HIGH && DecreasingServo == LOW){
        if (Servo3State < 180){
          if (Servo2State > 60){
            Servo3State = Servo3State + servostep;
          }
          else{
            if(Servo3State < Servo3Max){
              Servo3State = Servo3State + servostep;
            }
          }
        }
      }
      else if (DecreasingServo == HIGH && IncreasingServo == LOW){
        if (Servo3State > 0){
          Servo3State = Servo3State - servostep;
        }
      }
      break;

    //reversed due to reverse Servo
    case 4:
      if (IncreasingServo == LOW && DecreasingServo == HIGH){
        if (Servo4State < 180){
          Servo4State = Servo4State + servostep;
        }
      }
      else if (DecreasingServo == LOW && IncreasingServo == HIGH){
        if (Servo4State > 0){
          Servo4State = Servo4State - servostep;
        }
      }
      break;
  }

  //Write New Servo Value
  BaseRCServo.write(Servo1State);
  LowerRCServo.write(Servo2State);
  UpperRCServo.write(Servo3State);
  HeadRCServo.write(Servo4State);
}

void ConvControl(){
  if (digitalRead(ConvFWDPBNO) == HIGH && digitalRead(ConvREVPBNO) == LOW){
    ConvyorControl(ConvyorON, ConvyorFWD, ConvyorPWMON);  
  }
  else if (digitalRead(ConvREVPBNO) == HIGH && digitalRead(ConvFWDPBNO) == LOW){
    ConvyorControl(ConvyorON, ConvyorREV, ConvyorPWMON);
  }
  else{
    ConvyorControl(ConvyorOFF, ConvyorFWD, ConvyorPWMOFF);
  }
}

void ConvyorControl(int ConveyorEn, int ConveyorDir, int ConveyorPulse){
    digitalWrite(ConvEn, ConveyorEn);
    digitalWrite(ConvDir, ConveyorDir);
    analogWrite(ConvPulse, ConveyorPulse);  
}

void ArmPosEnd(int pos){
  int SS1 = 0;
  int SS2 = 0;
  int SS3 = 0;
  int SS4 = 0;
  
//end conveyor pos
  switch(pos){
    case 1:
      SS1 = 70;
      SS2 = 46;
      SS3 = 144;
      SS4 = 16;
    break;
    
//home offset pos
    case 2:
      SS1 = 70;
      SS2 = 110;
      SS3 = 180;
      SS4 = 0;
    break;

//over drop off pos
    case 3:
      SS1 = 180;
      SS2 = 110;
      SS3 = 180;
      SS4 = 0;
    break;
    
//drop off pos
    case 4:
      SS1 = 180;
      SS2 = 42;
      SS3 = 166;
      SS4 = 48;
    break;

//Home pos
    case 5:
      SS1 = 100;
      SS2 = 110;
      SS3 = 180;
      SS4 = 0;
    break;
  }

  //write new servo values
  BaseRCServo.write(SS1, 40, false);
  LowerRCServo.write(SS2, 30, false);
  UpperRCServo.write(SS3, 30, false);
  HeadRCServo.write(SS4, 100, false);
}

void debugging(){
  //if debugging is on, display all values
  Serial.println(" ");

  if(ManMode == HIGH){
    Serial.print("IncPB = ");
    Serial.print(digitalRead(IncreasePBNO));
    Serial.print("\tDecPB = ");
    Serial.print(digitalRead(DecreasePBNO));
    Serial.print("\tSSPB = ");
    Serial.print(digitalRead(ServoSelectPBNO));
    Serial.print("\tConvFWD= ");
    Serial.print(digitalRead(ConvFWDPBNO));
    Serial.print("\tConvREV = ");
    Serial.print(digitalRead(ConvREVPBNO));
    Serial.print("\tAutoPB = ");
    Serial.print(digitalRead(AutoPBNO));
    Serial.print("\tManPB = ");
    Serial.print(digitalRead(ManPBNO));
    Serial.print("\tStartPB = ");
    Serial.print(digitalRead(StartPBNO));
    Serial.print("\tStopPB = ");
    Serial.print(digitalRead(StopPBNO));

    Serial.print("\tSSVal = ");
    Serial.print(ServoSelectorVal);
    Serial.print("\tServo = ");
    Serial.print(Servo1State);
    Serial.print("  ");
    Serial.print(Servo2State);
    Serial.print("  ");
    Serial.print(Servo3State);
    Serial.print("  ");
    Serial.print(Servo4State);
    Serial.print(" S3Max ");
    Serial.print(Servo3Max);
  }

  Serial.println(" ");
  
  Serial.print(digitalRead(ConvProxEndNO));
  Serial.print(digitalRead(ConvProxMidNO));
  Serial.print(digitalRead(ConvProxStartNO));

  Serial.print("\tCycle Time\t");
}
