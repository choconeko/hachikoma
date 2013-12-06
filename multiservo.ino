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
#define SERVOMIN  200 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  800 // this is the 'maximum' pulse length count (out of 4096)

// our servo # counter
uint8_t servonum = 0;


class HachikomaServo{
public:
  int minPulse;
  int maxPulse; 
  int pin;
  HachikomaServo();
  void init(int pin,int minP,int maxP);
  void setPulse(int pulse);
};

class HachikomaLeg{
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

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");
  
  //setup servo profile   
  legs = (HachikomaLeg*)malloc(sizeof(HachikomaLeg) * MAX_LEGS);
  Serial.println("initialize");
  for (int i = 0; i < MAX_LEGS; i++){
      Serial.println(i * MAX_LEGS);
      legs[i].init(i * MAX_LEGS);  
  }  
  Serial.println("end initialization");
  //   
  
  
  
  


  pwm.begin();


  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
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

void loop() {
  
  //front right
  legs[0].top->setPulse(400);
  Serial.println(legs[0].top->pin);
  delay(100);
  legs[0].mid->setPulse(600);
  Serial.println(legs[0].mid->pin);
  delay(100);
  legs[0].bottom->setPulse(300);
  Serial.println(legs[0].bottom->pin);
  delay(100);
  
  //front left
  legs[1].top->setPulse(500);
  Serial.println(legs[1].top->pin);
  delay(100);
  legs[1].mid->setPulse(400);
  Serial.println(legs[1].mid->pin);
  delay(100);
  legs[1].bottom->setPulse(700);
  Serial.println(legs[1].bottom->pin);
  delay(100);
  
  //rear right
  legs[2].top->setPulse(500);
  Serial.println(legs[2].top->pin);
  delay(100);
  legs[2].mid->setPulse(600);
  delay(100);
  legs[2].bottom->setPulse(300);
  delay(100);
  
   //rear right
  legs[3].top->setPulse(500);
  Serial.println(legs[3].top->pin);
  delay(100);
  legs[3].mid->setPulse(300);
  delay(100);
  legs[3].bottom->setPulse(600);
  delay(100);
  
  
  delay(1000);
  /*
  if (true){
    for (int i = 0; i < 16; i++)
    {
      //pwm.setPWM(i, 0, 200);//0
      delay(200);
    }    
    for (int i = 0; i < 16; i++)
    {
      pwm.setPWM(i, 0, 475);//center 475
      delay(100);
      //pwm.setPWM(i, 0, 675);//center 475
      //delay(100);
    }    
    for (int i = 0; i < 16; i++)
    {
      //pwm.setPWM(i, 0, 750);//800? one servo maxes out at 760...
      delay(200);
    }
    delay(1000);
    
    return;
  }
  */
  
  // Drive each servo one at a time
  /*
  Serial.println(servonum);
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm.setPWM(servonum, 0, pulselen);
    delay(10);
  }

  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(servonum, 0, pulselen);
    delay(10);
  }
  */
  //servonum ++;
  //if (servonum > 15) servonum = 0;
}




HachikomaServo::HachikomaServo(){
  minPulse = 0;
  maxPulse = 0;
  pin = 0;
}

void HachikomaServo::init(int p,int minP,int maxP){
  minPulse = minP;
  maxPulse = maxP;
  pin = p;
}

void HachikomaServo::setPulse(int pulse){
    pwm.setPWM(pin, 0, pulse);
    delay(200);
}


HachikomaLeg::HachikomaLeg(){
}

void HachikomaLeg::init(int startpin){
  top = new HachikomaServo();
  top->init(startpin,200,750);
  mid = new HachikomaServo();
  mid->init(startpin+1,200,750);
  bottom = new HachikomaServo();
  bottom->init(startpin+2,200,750);
}


