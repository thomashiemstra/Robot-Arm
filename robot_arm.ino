 //this script is mine, it's awesome (well it works anyway)

#include <EEPROM.h>
//inlcuce adafruit servo driver thingies
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int angles0,angles1,angles2,angles3;
int a = 0; //used to make sure everthing inside the if statement only gets executed once
int b = 0; //same as a
uint8_t potPin0 = 0;
uint8_t potPin1 = 1;
uint8_t potPin2 = 2;
int addr = 0;
int angles[5]={90,90,90,90,90};
int limit;
int pulse_length;
uint8_t ledpin1 = 11;
uint8_t ledpin2 = 12;
uint8_t switchpin = 6;
uint8_t buttonpin = 5;
uint8_t transistorpin = 7;
uint8_t recordbutton =8;

//pulse length values for 0,90,180 degrees for the 6 servos: (if only these retards were linear...)
int servo[6][3]={  
  {165,370,620},
  {130,340,600},
  {140,340,600},
  {150,400,645},
  {0,0,0},
  {0,0,0}
};

//pot meter calibration, fuckers aren't linear
int pots[3][3]={
  {0,290,625},
  {0,290,605},
  {0,290,625},
};
//pot meter 1 has a dead zone arround 0, replace the fucker (maybe)

void setup() {
  pinMode(buttonpin, INPUT);
  pinMode(transistorpin, OUTPUT);
  pinMode(ledpin1, OUTPUT);
  pinMode(ledpin2, OUTPUT);
  pinMode(switchpin, INPUT);
  digitalWrite(transistorpin, LOW);
  digitalWrite(switchpin, HIGH);
  digitalWrite(buttonpin, HIGH);
  digitalWrite(recordbutton, HIGH);
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
  Serial.println("setup complete");

  }   


void loop() 
{
  
  while (! digitalRead(switchpin) ) //1 && b == 0
  {
     
    digitalWrite(ledpin2, LOW);
  
    digitalWrite(ledpin1, LOW);
    mimic();
    if (! digitalRead(recordbutton) && a==0)
    {
    digitalWrite(ledpin1, HIGH);
    b = 1; //makes sure everything only gets excecuted once   
    record();
    }
    
  }


  
  while (digitalRead(switchpin) ) //2 && a == 0
  {
    b = 0;
    digitalWrite(ledpin1, LOW);
    digitalWrite(ledpin2, HIGH);
    
    
    addr = 0;
    play();
    delay(100);
  }

}

//repeat the input with the servos from the pots, returns angles and button status for the record function
void mimic( )
{
  
  float reading0 = analogRead(potPin0);
  float reading1 = analogRead(potPin1);
  float reading2 = analogRead(potPin2);
  angles0 = potangle(0,reading0);
  angles1 = potangle(1,reading1);
  angles2 = potangle(2,reading2);
  angles3 = (90 - angles1 + angles2);
  servoangle(0,0,180-angles0);
  servoangle(1,1,angles1); 
  servoangle(2,2,angles2);
  servoangle(3,3,angles3);
  if(! digitalRead(buttonpin))
  {
    digitalWrite(transistorpin, HIGH);
  }
  if(digitalRead(buttonpin))
  {
    digitalWrite(transistorpin, LOW);
  }
  
}

//plays the 1024 address values from record
void play() 
{
  uint16_t addr = 0;
  
  while(addr < limit)
  {

    angles[0] = EEPROM.read(addr);
    angles[1] = EEPROM.read(addr+1);
    angles[2] = EEPROM.read(addr+2);
    angles[3] = (90 - angles[1] + angles[2]);
    digitalWrite(transistorpin, EEPROM.read(addr+3));
    servoangle(0,0,180-angles[0]);
    servoangle(1,1,angles[1]); 
    servoangle(2,2,angles[2]);
    servoangle(3,3,angles[3]);
    delay(25);  
    addr = addr  + 4;
  }

}
//stores the movement of the pots in 1024 adresses while the servos mimic them 
void record() 
{
  uint16_t addr = 0;
  limit = 0;
  
  while(addr <= 1028 && !digitalRead(switchpin))
  {
    float reading0 = analogRead(potPin0);
    float reading1 = analogRead(potPin1);
    float reading2 = analogRead(potPin2);
    angles[0] = potangle(0,reading0);
    angles[1] = potangle(1,reading1);
    angles[2] = potangle(2,reading2);
    angles[3] = (90 - angles[1] + angles[2]);
    angles[4] = 1 - digitalRead(buttonpin);
    servoangle(0,0,180-angles[0]);
    servoangle(1,1,angles[1]); 
    servoangle(2,2,angles[2]);
    servoangle(3,3,angles[3]);
    if(! digitalRead(buttonpin))
  {
    digitalWrite(transistorpin, 1);
  }
  if(digitalRead(buttonpin))
  {
    digitalWrite(transistorpin, 0);
  }
    EEPROM.write(addr,angles[0]);
    EEPROM.write(addr+1,angles[1]);
    EEPROM.write(addr+2,angles[2]);
    EEPROM.write(addr+3,angles[4]);
    delay(10); 
    addr = addr  + 4;
    limit = limit +4;
  }
}




  
//correctly map pot reading to angle for given calibrated pot meter
int potangle(int pot_number, int reading){
  int mappedreading;
  if (reading <=pots[pot_number][1])
  {
    mappedreading = map(reading,pots[pot_number][0],pots[pot_number][1],0,90);
    return mappedreading;

  }
  else if(reading>pots[pot_number][1])
  {
    mappedreading = map(reading,pots[pot_number][1],pots[pot_number][2],90,180);
    return mappedreading;
  }

}
  
//set a specific servo (with angle values from the servo matrix) on a specific channel to a specific angle
void servoangle(int servo_number, int channel, float angle)
{
    int n = servo_number; //n so that the formulas don't get cluttered with servo_angle everywhere
    if (angle >= 0 && angle <= 90)
    {
      pulse_length = int(float(servo[n][0]+angle*( (servo[n][1]-servo[n][0]) )/90.0));  
    }    
    else if(angle > 90 && angle <= 180)
    {
      pulse_length = int(float(  servo[n][1] + (angle-90.0) *( (servo[n][2]-servo[n][1]))/90.0 ) );     
    }
    else // if (angle <0 && angle>180) //redudant protection just in case
    {
      return;
    }
    pwm.setPWM(channel, 0, pulse_length);
}

