#include <Arduino.h>
#include <Servo.h>


#define BUTTON_PIN 12
#define SERVO_PIN 3
#define PEEP_PIN 7
#define NTC_PIN_PRIME A0
#define NTC_PIN_SECOND A4
#define LED_ON HIGH
#define LED_OFF LOW
#define SERV_ON_ANGLE 164
#define SERV_STDBY_ANGLE 145
#define SERV_OFF_ANGLE 125

#define LOW_BOUND 79.3
#define HIGH_BOUND 79.5

Servo theservo;
int button_state = 0;
int button_pause = 0;
int ntc_voltage_prime = 0;
int ntc_voltage_second = 0;
int delay_mill = 400;


enum State {SETUP, FIRST_HEATING, FIRST_SETTLING, HEATING, COOLING};
State state = SETUP;

enum Mode {DEBUG, NORMAL};
Mode mode = NORMAL;

unsigned long now;
unsigned long last_time = 0;


// in voltage:
// y = 1.4214x4 - 13.945x3 + 50.042x2 - 96.803x + 115.36
//const double coeff_4 = 1.4214;
//const double coeff_3 = -13.945;
//const double coeff_2 = 50.042;
//const double coeff_1 = -96.803;
//const double coeff_0 = 115.36;

// in code direct:
// y = 6.88934E-10x4 - 1.44055E-06x3 + 1.10170E-03x2 - 4.54209E-01x + 1.15360E+02
const double coeff_4 = 6.88934E-10;
const double coeff_3 = -1.44055E-06;
const double coeff_2 = 1.10170E-03;
const double coeff_1 = -4.54209E-01;
const double coeff_0 = 1.15360E+02;

double temperature = 0;
double last_temperature = 0;

void setup() {
  digitalWrite(LED_BUILTIN, LED_ON);
  
  Serial.begin(9600);

  theservo.attach(SERVO_PIN);
  theservo.write(SERV_STDBY_ANGLE);

//*********************************************************************************************
//  digitalWrite(PEEP_PIN, HIGH); delay(80); digitalWrite(PEEP_PIN, LOW); delay(2000);
//
//  theservo.write(SERV_OFF_ANGLE); 
//  digitalWrite(PEEP_PIN, HIGH); delay(80); digitalWrite(PEEP_PIN, LOW); delay(2000);
//
//  theservo.write(SERV_ON_ANGLE); 
//  digitalWrite(PEEP_PIN, HIGH); delay(80); digitalWrite(PEEP_PIN, LOW); delay(2000);
//
//  
//  while(true);
//*********************************************************************************************


  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  
  ntc_voltage_prime = analogRead(NTC_PIN_PRIME);
  ntc_voltage_second = analogRead(NTC_PIN_SECOND);

  temperature = coeff_4 * pow(ntc_voltage_prime, 4)
              + coeff_3 * pow(ntc_voltage_prime, 3)
              + coeff_2 * pow(ntc_voltage_prime, 2)
              + coeff_1 * ntc_voltage_prime
              + coeff_0;

  digitalWrite(LED_BUILTIN, LED_OFF);
  mode = DEBUG; // <----------------------------------------------- change here
}

void loop() {
  now = millis();
  if (now > (last_time + 250) || now < last_time) {

    /* get new temperature: */
    ntc_voltage_prime = analogRead(NTC_PIN_PRIME);
    ntc_voltage_second = analogRead(NTC_PIN_SECOND);
    last_temperature = temperature;
    temperature = coeff_4 * pow(ntc_voltage_prime, 4)
                + coeff_3 * pow(ntc_voltage_prime, 3)
                + coeff_2 * pow(ntc_voltage_prime, 2)
                + coeff_1 * ntc_voltage_prime
                + coeff_0;
    temperature = (last_temperature + temperature) / 2;
    // Serial.println(temperature);
    
    switch(mode)
    {
    case DEBUG:
      Serial.println("ntc_voltage_prime ntc_voltage_second"); // temperature");
      Serial.print(String(ntc_voltage_prime) + String(" "));
      Serial.println(String(ntc_voltage_second));
      //Serial.print(String(temperature) + String(" "));
      break;

    case NORMAL:
      /* reset statemachine if button is pressed: */
      button_state = digitalRead(BUTTON_PIN);
      if (button_pause == 0 && button_state == 0){
        state = SETUP;
        button_pause = 1;
      }
      else if (button_pause == 1 && button_state == 1){
        button_pause = 0;
      }

      
      /* statemachine: */
      switch (state)
      {
      case SETUP:
        Serial.println("SETUP");

        theservo.write(SERV_ON_ANGLE);
        delay(delay_mill);
        theservo.write(SERV_STDBY_ANGLE);

        state = FIRST_HEATING;
        break;
        
      case FIRST_HEATING:
        digitalWrite(LED_BUILTIN, LED_ON);
        Serial.println("FIRST_HEATING");
        if (temperature > 76.0){

          theservo.write(SERV_OFF_ANGLE);
          delay(delay_mill);
          theservo.write(SERV_STDBY_ANGLE);
          digitalWrite(PEEP_PIN,HIGH);
          delay(1000);
          digitalWrite(PEEP_PIN,LOW);

          state = FIRST_SETTLING; 
        }
        break;

      case FIRST_SETTLING:
        digitalWrite(LED_BUILTIN, LED_OFF);
        Serial.println("FIRST_SETTLING");
        if (temperature > 80.0){

          theservo.write(SERV_OFF_ANGLE);
          delay(delay_mill);
          theservo.write(SERV_STDBY_ANGLE);

          state = COOLING; 
        }
        if(temperature < 76.0){
          
          theservo.write(SERV_ON_ANGLE);
          delay(delay_mill);
          theservo.write(SERV_STDBY_ANGLE);

          state = HEATING;
        }
        break;
      
      case COOLING:
        digitalWrite(LED_BUILTIN, LED_OFF);
        Serial.println("COOLING");
        if (temperature < LOW_BOUND){

          theservo.write(SERV_ON_ANGLE);
          delay(delay_mill);
          theservo.write(SERV_STDBY_ANGLE);

          state = HEATING; 
        }
        break;

      case HEATING:
        digitalWrite(LED_BUILTIN, LED_ON);
        Serial.println("HEATING");
        if (temperature > HIGH_BOUND){

          theservo.write(SERV_OFF_ANGLE);
          delay(delay_mill);
          theservo.write(SERV_STDBY_ANGLE);

          state = COOLING; 
        }
        break;

      default:
        theservo.write(SERV_STDBY_ANGLE);
        Serial.println("default (ERROR)");
        break;
      }

      break;
    }
    
    
    last_time = now;

    // Serial.println(String("time left: ") + String(200 - (millis() - now)));
  }
}