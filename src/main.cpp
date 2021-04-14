#include <Arduino.h>
#include <Servo.h>


#define BUTTON_PIN 12
#define SERVO_PIN 3
#define NTC_PIN A0
#define LED_ON HIGH
#define LED_OFF LOW
#define SERV_ON_ANGLE 135
#define SERV_OFF_ANGLE 45

Servo theservo;
int button_state = 0;
int ntc_voltage = 0;

enum State {SETUP, FIRST_HEATING, FIRST_SETTLING, HEATING, COOLING};
State state = SETUP;

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
  theservo.write(90);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  
  ntc_voltage = analogRead(NTC_PIN);
  temperature = coeff_4 * pow(ntc_voltage, 4)
              + coeff_3 * pow(ntc_voltage, 3)
              + coeff_2 * pow(ntc_voltage, 2)
              + coeff_1 * ntc_voltage
              + coeff_0;

  digitalWrite(LED_BUILTIN, LED_OFF);
}

void loop() {
  now = millis();
  if (now > (last_time + 250) || now < last_time) {

    /* get new temperature: */
    ntc_voltage = analogRead(NTC_PIN);
    last_temperature = temperature;
    temperature = coeff_4 * pow(ntc_voltage, 4)
                + coeff_3 * pow(ntc_voltage, 3)
                + coeff_2 * pow(ntc_voltage, 2)
                + coeff_1 * ntc_voltage
                + coeff_0;
    temperature = (last_temperature + temperature) / 2;
    Serial.println(temperature);
    
    /* reset statemachine if button is pressed: */
    button_state = digitalRead(BUTTON_PIN);
    if (button_state == 0){
      state = SETUP;
    }
    
    /* statemachine: */
    switch (state)
    {
    case SETUP:
      state = FIRST_HEATING;
      Serial.println("SETUP");
      break;
      
    case FIRST_HEATING:
      digitalWrite(LED_BUILTIN, LED_ON);
      theservo.write(SERV_ON_ANGLE);
      Serial.println("FIRST_HEATING");
      if (temperature > 77.0){
        state = FIRST_SETTLING; 
      }
      break;

    case FIRST_SETTLING:
      digitalWrite(LED_BUILTIN, LED_OFF);
      theservo.write(SERV_OFF_ANGLE);
      Serial.println("FIRST_SETTLING");
      if (temperature > 80.0){
        state = COOLING; 
      }
      if (temperature < 76.0){
        state = HEATING;
      }
      break;
    
    case COOLING:
      digitalWrite(LED_BUILTIN, LED_OFF);
      theservo.write(SERV_OFF_ANGLE);
      Serial.println("COOLING");
      if (temperature < 79.65){
        state = HEATING; 
      }
      break;

    case HEATING:
      digitalWrite(LED_BUILTIN, LED_ON);
      theservo.write(SERV_ON_ANGLE);
      Serial.println("HEATING");
      if (temperature > 79.9){
        state = COOLING; 
      }
      break;

    default:
      Serial.println("default (ERROR)");
      break;
    }
    
    
    last_time = now;
    Serial.println(String("time left: ") + String(200 - (millis() - now)));
  }
}