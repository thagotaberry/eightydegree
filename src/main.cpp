#include <Arduino.h>
#include <Servo.h>

// #define DEBUG_LOG

/* hardware definitions: */
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

/* control switching boundaries: */
#define LOW_BOUND 79.6
#define HIGH_BOUND 80
#define FIRST_HEATING_BOUND 76.5
#define FIRST_SETTLING_HIGH_BOUND 80.0
#define FIRST_SETTLING_LOW_BOUND 76.0
#define HEATING_CNT_NUMBER 6 // 1.5 seconds
#define AFTER_HEAT_WAIT_CNT_NUMBER 24 // 6 seconds

Servo theservo;
int button_state = 0;
int button_pause = 0;
int ntc_voltage_prime = 0;
int ntc_voltage_second = 0;
int delay_mill = 400;
int heating_cnt = 0;
int after_heating_wait_cnt = 0;


enum State {SETUP, FIRST_HEATING, FIRST_SETTLING, HEATING, COOLING, AFTER_HEAT_WAITING};
State state = SETUP;

enum Mode {DEBUG, NORMAL};
Mode mode = NORMAL;

unsigned long now;
unsigned long last_time = 0;


// in code direct:
// range 22.3 (553) ... 49.8 (262): y = -0.0935x + 73.687
// range 49.9 (261) ... 100 (51): y = -30.66ln(x) + 220.69
const double coeff_1_low_range = -0.0935;
const double coeff_0_low_range = 73.687;
const double factor_high_range = -30.66;
const double offset_high_range = 220.69;


double temperature_prime = 0;
double temperature_second = 0;
double last_temperature_prime = 0;
double last_temperature_second = 0;


double volt_to_temp(int ntc_value){
  double temperature;
    if(ntc_value > 261) /* low range */
      temperature = coeff_1_low_range * ntc_value + coeff_0_low_range;
    else /* high range */
      temperature = factor_high_range * log(ntc_value) + offset_high_range;
    return temperature;
}

void heater_on(){
  theservo.write(SERV_ON_ANGLE);
  delay(delay_mill);
  theservo.write(SERV_STDBY_ANGLE);
}

void heater_off(){
  theservo.write(SERV_OFF_ANGLE);
  delay(delay_mill);
  theservo.write(SERV_STDBY_ANGLE);
}

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
  temperature_prime = volt_to_temp(ntc_voltage_prime);
  temperature_second = volt_to_temp(ntc_voltage_second);

  digitalWrite(LED_BUILTIN, LED_OFF);
  mode = NORMAL; // <----------------------------------------------- change here
}

void loop() {
  now = millis();
  if (now > (last_time + 250) || now < last_time) {

    /* get new temperature: */
    ntc_voltage_prime = analogRead(NTC_PIN_PRIME);
    ntc_voltage_second = analogRead(NTC_PIN_SECOND);
    last_temperature_prime = temperature_prime;
    last_temperature_second = temperature_second;

    /* prime sensor: */
    temperature_prime = volt_to_temp(ntc_voltage_prime);
    temperature_prime = (last_temperature_prime + temperature_prime) / 2;

    /* second sensor: */
    temperature_second = volt_to_temp(ntc_voltage_second);
    temperature_second = (last_temperature_second + temperature_second) / 2;
    
    switch(mode)
    {
    case DEBUG:
      Serial.println("temperature_second temperature_prime");
      Serial.print(String(temperature_second) + String(" "));
      Serial.println(String(temperature_prime));
      break;

    case NORMAL:

      #ifdef DEBUG_LOG
      Serial.println("temperature_second temperature_prime");
      Serial.print(String(temperature_second) + String(" "));
      Serial.println(String(temperature_prime));
      #endif

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
        #ifdef DEBUG_LOG
        Serial.println("SETUP");
        #endif
        heater_on();
        state = FIRST_HEATING;
        break;
        
      case FIRST_HEATING:
        digitalWrite(LED_BUILTIN, LED_ON);
        #ifdef DEBUG_LOG
        Serial.println("FIRST_HEATING");
        #endif
        if (temperature_prime > FIRST_HEATING_BOUND){

          heater_off();

          digitalWrite(PEEP_PIN,HIGH);
          delay(2000);
          digitalWrite(PEEP_PIN,LOW);

          state = FIRST_SETTLING; 
        }
        break;

      case FIRST_SETTLING:
        digitalWrite(LED_BUILTIN, LED_OFF);
        #ifdef DEBUG_LOG
        Serial.println("FIRST_SETTLING");
        #endif
        if (temperature_prime > FIRST_SETTLING_HIGH_BOUND){
          state = COOLING; 
        }
        if(temperature_prime < FIRST_SETTLING_LOW_BOUND){
          heater_on();
          state = HEATING;
        }
        break;
      
      case COOLING:
        digitalWrite(LED_BUILTIN, LED_OFF);
        #ifdef DEBUG_LOG
        Serial.println("COOLING");
        #endif
        if (temperature_prime < LOW_BOUND){
          heater_on();
          state = HEATING; 
        }
        break;

      case HEATING:
        digitalWrite(LED_BUILTIN, LED_ON);
        #ifdef DEBUG_LOG
        Serial.println("HEATING");
        #endif
        heating_cnt++;
        
        if (temperature_prime > HIGH_BOUND){
          heater_off();
          state = COOLING; 
        }
        else if (heating_cnt > HEATING_CNT_NUMBER){
          heating_cnt = 0;
          heater_off();
          state = AFTER_HEAT_WAITING; 
        }

        break;

      case AFTER_HEAT_WAITING:
        digitalWrite(LED_BUILTIN, LED_OFF);
        #ifdef DEBUG_LOG
        Serial.println("AFTER_HEAT_WAITING");
        #endif
        after_heating_wait_cnt++;
        if(after_heating_wait_cnt > AFTER_HEAT_WAIT_CNT_NUMBER){
          after_heating_wait_cnt = 0;
          if (temperature_prime > HIGH_BOUND){
            state = COOLING; 
          }
          else if (temperature_prime <= HIGH_BOUND){
            heater_on();
            state = HEATING;
          }
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