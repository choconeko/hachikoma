/***************************************************
 * This is an example for our Adafruit 16-channel PWM & Servo driver
 * Servo test - this will drive 16 servos, one after the other
 *
 * Pick one up today in the adafruit shop!
 * ------> http://www.adafruit.com/products/815
 *
 * These displays use I2C to communicate, 2 pins are required to
 * interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 * BSD license, all text above must be included in any redistribution
 ****************************************************/
#define __AVR__

#include <RFduinoBLE.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  300 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  750 // this is the 'maximum' pulse length count (out of 4096)


class HachikomaServo {
  public:
    int minPulse;
    int maxPulse;
    int pin;
    HachikomaServo();
    void init(int pin, int minP, int maxP);
    void setPulse(int pulse);
};

class HachikomaLeg {
  public:
    HachikomaServo* top;
    HachikomaServo* mid;
    HachikomaServo* bottom;
    HachikomaLeg();
    void init(int startpin);
};


#define MAX_LEGS 4
#define MAX_LEG_SERVO 3
HachikomaLeg* legs;

byte tempcommand[11];

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");

  //setup servo profile
  legs = (HachikomaLeg*)malloc(sizeof(HachikomaLeg) * MAX_LEGS);
  Serial.println("initialize");
  for (int i = 0; i < MAX_LEGS; i++) {
    Serial.println(i * MAX_LEGS);
    legs[i].init(i * MAX_LEGS);
  }
  Serial.println("end initialization");
  //

  tempcommand = {
    120, 121, 0, 0, 0, 1, 0, 0, 1, 39, 122
  };


  pwm.begin(5, 6); // yellow clock, green pulse
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  
  
  //ble setup
  RFduinoBLE.deviceName = "Hachikoma-00";
  RFduinoBLE.advertisementInterval = 675;
  RFduinoBLE.advertisementData = "hkm";
  RFduinoBLE.begin();
}

// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;

  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  Serial.print(pulselength);
  Serial.println(" us per period");
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength);
  Serial.println(" us per bit");
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}


void serialEvent() {


}

boolean processCommand(char tempcommand[]) {
  //if (tempcommand[0] != 'x') return false;
  if (tempcommand[0] != 'y') return false;
  if (tempcommand[9] != 'z') return false;

  int servonum = ( (int)tempcommand[1] << 24 ) | ( (int)tempcommand[2] << 16 ) | ( (int)tempcommand[3] << 8 ) | (int)tempcommand[4];
  int pulse = ( (int)tempcommand[5] << 24 ) | ( (int)tempcommand[6] << 16 ) | ( (int)tempcommand[7] << 8 ) | (int)tempcommand[8];
  Serial.print("servo: ");
  Serial.println(servonum);
  Serial.print("pulse: ");
  Serial.println(pulse);

  if (pulse > SERVOMAX) pulse = SERVOMAX;
  if (pulse < SERVOMIN) pulse = SERVOMIN;

  pwm.setPWM(servonum, 0, pulse);

  return true;
}

boolean doit = true;
int commandState = 0;
void loop() {


  switch (commandState) {
    case 0:
      if (Serial.available() > 0) {
        char b = Serial.read();
        if (b == 'x') {
          Serial.println(" ");
          Serial.println("commandheader:");
          Serial.print(b);
          commandState = 1;
        };
      }
      break;
    case 1:
      if (Serial.available() >= 10) {
        Serial.println(" ");
        Serial.println("commandbody: ");
        char buf[10];
        for (int i = 0; i < 10; i++) {
          buf[i] = Serial.read();

          Serial.print(i);
          Serial.print(" ");
          Serial.println(buf[i]);
          

        }       
        if (processCommand(buf)){
          Serial.println("success!");
        } else {
          Serial.println("bad command!");
        }
        
        commandState = 0;
      }
      break;
  }

  if (doit) {
    //front right
    legs[0].top->setPulse(400);
    delay(10);
    legs[0].mid->setPulse(600);
    delay(10);
    legs[0].bottom->setPulse(400);
    delay(10);

    //front left
    legs[1].top->setPulse(400);
    delay(10);
    legs[1].mid->setPulse(400);
    delay(10);
    legs[1].bottom->setPulse(700);
    delay(10);

    //rear right
    legs[2].top->setPulse(650);
    delay(10);
    legs[2].mid->setPulse(600);
    delay(10);
    legs[2].bottom->setPulse(300);
    delay(10);

    //rear left
    legs[3].top->setPulse(455);
    delay(10);
    legs[3].mid->setPulse(455);
    delay(10);
    legs[3].bottom->setPulse(565);
    delay(10);

    delay(1500);
  }

  if (doit) {
    //front right
    legs[0].top->setPulse(550);
    delay(10);
    legs[0].mid->setPulse(500);
    delay(10);
    legs[0].bottom->setPulse(500);
    delay(10);

    //front left
    legs[1].top->setPulse(650);
    delay(10);
    legs[1].mid->setPulse(500);
    delay(10);
    legs[1].bottom->setPulse(500);
    delay(10);

    //rear right
    legs[2].top->setPulse(450);
    delay(10);
    legs[2].mid->setPulse(500);
    delay(10);
    legs[2].bottom->setPulse(500);
    delay(10);

    //rear left
    legs[3].top->setPulse(350);
    delay(10);
    legs[3].mid->setPulse(300);
    delay(10);
    legs[3].bottom->setPulse(500);
    delay(10);

    delay(1500);
  }

  doit = false;
}

HachikomaServo::HachikomaServo() {
  minPulse = 0;
  maxPulse = 0;
  pin = 0;
}

void HachikomaServo::init(int p, int minP, int maxP) {
  minPulse = minP;
  maxPulse = maxP;
  pin = p;

  //setPulse(600);
}

void HachikomaServo::setPulse(int pulse) {
  pwm.setPWM(pin, 0, pulse);
}


HachikomaLeg::HachikomaLeg() {
}

void HachikomaLeg::init(int startpin) {
  top = new HachikomaServo();
  top->init(startpin, 200, 750);
  mid = new HachikomaServo();
  mid->init(startpin + 1, 200, 750);
  bottom = new HachikomaServo();
  bottom->init(startpin + 2, 200, 750);
}


void RFduinoBLE_onDisconnect()
{
  // don't leave the led on if they disconnect
  //digitalWrite(led, LOW);
}

void RFduinoBLE_onReceive(char *data, int len)
{

    //RFduinoBLE.send(data[0]);
    RFduinoBLE.send(data,len);
}




















