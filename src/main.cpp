#include <Arduino.h>
#include <Servo.h>
#include <EEPROM.h>

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
// debug // #define LOW_BOUND 10
// debug // #define HIGH_BOUND 11
// debug // #define FIRST_HEATING_BOUND 10
// debug // #define FIRST_SETTLING_HIGH_BOUND 11
// debug // #define FIRST_SETTLING_LOW_BOUND 10
#define LOW_BOUND 79.6
#define HIGH_BOUND 80
#define FIRST_HEATING_BOUND 76.5
#define FIRST_SETTLING_HIGH_BOUND 80.0
#define FIRST_SETTLING_LOW_BOUND 76.0
#define HEATING_CNT_NUMBER 6 // 1.5 seconds
#define AFTER_HEAT_WAIT_CNT_NUMBER 24 // 6 seconds

Servo theservo;
uint16_t loop_intervall = 250; // millis

uint16_t button_fast_register_pressed = 0;
uint16_t button_short_pressed_trigger = 0;
uint16_t button_long_held_trigger = 0;
uint16_t button_pressed_cnt = 0;
uint16_t button_hold_activation_duration = 3000; // millis

uint16_t led_blink_cnt = 0;
uint16_t led_blink_intervall = 240; // millis
uint16_t led_blitz_off_intervall = 1500;

uint16_t buzz_cnt = 0;
uint16_t buzz_short_duration = 240;
uint16_t buzz_long_duration = 2000;
uint16_t buzz_beep_intervall = 240;

uint16_t brew_intervall_cnt = 0;
uint16_t brew_intervall_training_cnt = 0;
uint16_t brew_intervall_array_idx = 0;
const int brew_intervall_array_size = 10;
uint16_t brew_intervall_array[brew_intervall_array_size] = {0,0,0,0,0,0,0,0,0,0}; // seconds
uint16_t brew_interval_end_cnt = 0;
uint16_t brew_interval_end_wait_time = 30; // seconds
uint16_t insanety_found = 0;
uint16_t max_intervall_seconds = 300; // 300 seconds = 5 min
uint16_t eeprom_write_cycle_count = 0;

int ntc_voltage_prime = 0;
int ntc_voltage_second = 0;
int delay_mill = 400;
int heating_cnt = 0;
int after_heating_wait_cnt = 0;

enum ProgramState {SETUP, FIRST_HEATING, FIRST_SETTLING, HEATING, COOLING, AFTER_HEAT_WAITING, BREW_INTERVALL_START, BREW_INTERVALL_STEP, BREW_INTERVALL_END_WAIT_AND_REMINDER, ENTER_INTERVALL_TRAINING_MODE, INTERVALL_TRAINING_START, INTERVALL_TRAINING_COUNTING_AND_ADDING, INTERVALL_TRAINING_END, PROGRAM_IDLE, PROGRAM_ERROR};
ProgramState program_state = SETUP;

enum ButtonState {BUTTON_IDLE, PRESSED, BUTTON_SHORT_TRIGGERED, BUTTON_LONG_HELD_ACTIVATED};
ButtonState button_state = BUTTON_IDLE;

enum LEDState {LED_STATE_IDLE, LED_OFF_ST, LED_ON_ST, BLINK_ON, BLINK_OFF, BLITZ_ON, BLITZ_OFF};
LEDState led_state = LED_OFF_ST;

enum BuzzerState {BUZZ_IDLE, BUZZ_OFF, BUZZ_ON, BUZZ_ON_SHORT, BUZZ_ON_LONG, BEEPING_ON, BEEPING_OFF};
BuzzerState buzzer_state = BUZZ_OFF;

enum DebugMode {DEBUG, NORMAL};
DebugMode debug_mode = NORMAL;

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

void write_int_array_to_EEPROM(uint16_t address, uint16_t array_to_store[], uint16_t size)
{
  uint16_t current_address = address;
  for (uint16_t i = 0; i < size; i++) 
  {
    EEPROM.write(current_address, array_to_store[i] >> 8);
    EEPROM.write(current_address + 1, array_to_store[i] & 0xFF);
    current_address += 2;
  }
}
void read_int_array_from_EEPROM(uint16_t address, uint16_t array_read[], uint16_t size)
{
  uint16_t current_address = address;
  for (uint16_t i = 0; i < size; i++)
  {
    array_read[i] = (EEPROM.read(current_address) << 8) + EEPROM.read(current_address + 1);
    current_address += 2;
  }
}

void setup() {
  digitalWrite(LED_BUILTIN, LED_ON);
  
  Serial.begin(9600);

  theservo.attach(SERVO_PIN);
  theservo.write(SERV_STDBY_ANGLE);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);

  read_int_array_from_EEPROM(2, brew_intervall_array, brew_intervall_array_size);
  for(brew_intervall_array_idx = 0; brew_intervall_array_idx < brew_intervall_array_size; brew_intervall_array_idx++){
    Serial.println(String(brew_intervall_array[brew_intervall_array_idx]));
  }
  brew_intervall_array_idx = 0;
  
  ntc_voltage_prime = analogRead(NTC_PIN_PRIME);
  ntc_voltage_second = analogRead(NTC_PIN_SECOND);
  temperature_prime = volt_to_temp(ntc_voltage_prime);
  temperature_second = volt_to_temp(ntc_voltage_second);

  digitalWrite(LED_BUILTIN, LED_OFF);
  debug_mode = NORMAL; // <----------------------------------------------- change here
}

void loop() {
  
  if (digitalRead(BUTTON_PIN) == 0) /* active low */
    button_fast_register_pressed = 1;
  
  now = millis();
  if (now > (last_time + loop_intervall) || now < last_time) {

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
    
    switch(debug_mode)
    {
    case DEBUG:
      /* to log the measured temperatures via USB COM and Arduino debug plot app */
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

      /**************************/
      /* button state machine:  */
      /**************************/
      switch (button_state)
      {
      case BUTTON_IDLE:
        /* the button is not presst. Just chilling... */
        if (button_fast_register_pressed == 1) /* detected outside of intervall loop */
        { 
          button_state = PRESSED;
        }
        break;
        
      case PRESSED:
        /* the button is currently pressed. Now what? */
        button_pressed_cnt++;
        if (digitalRead(BUTTON_PIN) == 1) /* inactive high */
        { 
          /* button was released quickly. Activate trigger and back to BUTTON_IDLE. */
          button_short_pressed_trigger = 1;
          button_pressed_cnt = 0;
          button_state = BUTTON_SHORT_TRIGGERED;
        }
        else if ((button_pressed_cnt * loop_intervall) > button_hold_activation_duration)
        {
          /* button was held for a long time. Enter the intervall timer training mode and wait until button is released. */
          button_long_held_trigger = 1;
          button_pressed_cnt = 0;
          button_state = BUTTON_LONG_HELD_ACTIVATED;
        }
        break;

      case BUTTON_SHORT_TRIGGERED:
        /* release the trigger after one loop cycle */
        button_short_pressed_trigger = 0;
        button_fast_register_pressed = 0;
        button_state = BUTTON_IDLE;
        break;
        
      case BUTTON_LONG_HELD_ACTIVATED:
        /* wait until button gets released, after button long hold has been registered */
        button_long_held_trigger = 0;
        if (digitalRead(BUTTON_PIN) == 1){ /* inactive high */
          /* button was released. Back to BUTTON_IDLE. */
          button_fast_register_pressed = 0;
          button_state = BUTTON_IDLE;
        }
        break;

      default:
        break;
      }

      /**************************/
      /* LED state machine:     */
      /**************************/
      switch (led_state)
      {
      case LED_STATE_IDLE:
        /* chilling the led base */
        break;

      case LED_OFF_ST:
        digitalWrite(LED_BUILTIN, LED_OFF);
        led_state = LED_STATE_IDLE;
        break;

      case LED_ON_ST:
        digitalWrite(LED_BUILTIN, LED_ON);
        led_state = LED_STATE_IDLE;
        break;

      case BLINK_ON:
        led_blink_cnt ++;
        if(((led_blink_cnt-1) * loop_intervall) > led_blink_intervall){
          led_blink_cnt = 0;
          digitalWrite(LED_BUILTIN, LED_OFF);
          led_state = BLINK_OFF;
        }
        break;

      case BLINK_OFF:
        led_blink_cnt ++;
        if(((led_blink_cnt-1) * loop_intervall) > led_blink_intervall){
          led_blink_cnt = 0;
          digitalWrite(LED_BUILTIN, LED_ON);
          led_state = BLINK_ON;
        }
        break;
      
      case BLITZ_ON:
        led_blink_cnt ++;
        if(((led_blink_cnt-1) * loop_intervall) > led_blink_intervall){
          led_blink_cnt = 0;
          digitalWrite(LED_BUILTIN, LED_OFF);
          led_state = BLITZ_OFF;
        }
        break;

      case BLITZ_OFF:
        led_blink_cnt ++;
        if(((led_blink_cnt-1) * loop_intervall) > led_blitz_off_intervall){
          led_blink_cnt = 0;
          digitalWrite(LED_BUILTIN, LED_ON);
          led_state = BLITZ_ON;
        }
        break;

      default:
        break;
      } 
      
      /**************************/
      /* buzzer state machine:  */
      /**************************/
      switch (buzzer_state)
      {
      case BUZZ_IDLE:
        /* chilling the buzzer base */
        break;

      case BUZZ_OFF:
        digitalWrite(PEEP_PIN,LOW);
        buzzer_state = BUZZ_IDLE;
        break;
      
      case BUZZ_ON:
        digitalWrite(PEEP_PIN,HIGH);
        buzzer_state = BUZZ_IDLE;
        break;

      case BUZZ_ON_SHORT:
        digitalWrite(PEEP_PIN,HIGH);
        buzz_cnt++;
        if(((buzz_cnt-1) * loop_intervall) > buzz_short_duration){
          buzz_cnt = 0;
          digitalWrite(PEEP_PIN,LOW);
          buzzer_state = BUZZ_IDLE;
        }
        break;

      case BUZZ_ON_LONG:
        digitalWrite(PEEP_PIN,HIGH);
        buzz_cnt++;
        if(((buzz_cnt-1) * loop_intervall) > buzz_long_duration){
          buzz_cnt = 0;
          digitalWrite(PEEP_PIN,LOW);
          buzzer_state = BUZZ_IDLE;
        }
        break;

      case BEEPING_ON:
        buzz_cnt++;
        if(((buzz_cnt-1) * loop_intervall) > buzz_beep_intervall){
          buzz_cnt = 0;
          digitalWrite(PEEP_PIN,LOW);
          buzzer_state = BEEPING_OFF;
        }
        break;
        
      case BEEPING_OFF:
        buzz_cnt++;
        if(((buzz_cnt-1) * loop_intervall) > buzz_beep_intervall){
          buzz_cnt = 0;
          digitalWrite(PEEP_PIN,HIGH);
          buzzer_state = BEEPING_ON;
        }
        break;

      default:
        break;
      }
      
      /**************************/
      /* program state machine: */
      /**************************/
      switch (program_state)
      {
      case SETUP:
        #ifdef DEBUG_LOG
        Serial.println("SETUP");
        #endif
        heater_on();
        if (button_long_held_trigger == 1){
          program_state = ENTER_INTERVALL_TRAINING_MODE;
        }
        else {
          program_state = FIRST_HEATING;
          led_state = BLITZ_ON;
        }
        break;
        
      case FIRST_HEATING:
        #ifdef DEBUG_LOG
        Serial.println("FIRST_HEATING");
        #endif
        if (button_long_held_trigger == 1){
          program_state = ENTER_INTERVALL_TRAINING_MODE;
        }
        else if (temperature_prime > FIRST_HEATING_BOUND){
          heater_off();
          buzzer_state = BUZZ_ON_LONG;
          led_state = LED_ON_ST;
          program_state = FIRST_SETTLING; 
        }
        break;

      case FIRST_SETTLING:
        #ifdef DEBUG_LOG
        Serial.println("FIRST_SETTLING");
        #endif
        if (temperature_prime > FIRST_SETTLING_HIGH_BOUND){
          led_state = LED_ON_ST;
          program_state = COOLING; 
        }
        if(temperature_prime < FIRST_SETTLING_LOW_BOUND){
          heater_on();
          led_state = BLITZ_ON;
          program_state = HEATING;
        }
        if(button_short_pressed_trigger == 1){
          program_state = BREW_INTERVALL_START;
        }
        else if (button_long_held_trigger == 1){
          program_state = ENTER_INTERVALL_TRAINING_MODE;
        }
        break;
      
      case COOLING:
        #ifdef DEBUG_LOG
        Serial.println("COOLING");
        #endif
        if (temperature_prime < LOW_BOUND){
          heater_on();
          led_state = BLITZ_ON;
          program_state = HEATING; 
        }
        if(button_short_pressed_trigger == 1){
          program_state = BREW_INTERVALL_START;
        }
        else if (button_long_held_trigger == 1){
          program_state = ENTER_INTERVALL_TRAINING_MODE;
        }
        break;

      case HEATING:
        #ifdef DEBUG_LOG
        Serial.println("HEATING");
        #endif
        heating_cnt++;
        
        if (temperature_prime > HIGH_BOUND){
          heater_off();
          led_state = LED_ON_ST;
          program_state = COOLING; 
        }
        else if (heating_cnt > HEATING_CNT_NUMBER){
          heating_cnt = 0;
          heater_off();
          program_state = AFTER_HEAT_WAITING; 
        }

        if(button_short_pressed_trigger == 1){
          program_state = BREW_INTERVALL_START;
        }
        else if (button_long_held_trigger == 1){
          program_state = ENTER_INTERVALL_TRAINING_MODE;
        }
        break;

      case AFTER_HEAT_WAITING:
        #ifdef DEBUG_LOG
        Serial.println("AFTER_HEAT_WAITING");
        #endif
        after_heating_wait_cnt++;
        if(after_heating_wait_cnt > AFTER_HEAT_WAIT_CNT_NUMBER){
          after_heating_wait_cnt = 0;
          if (temperature_prime > HIGH_BOUND){
            led_state = LED_ON_ST;
            program_state = COOLING; 
          }
          else if (temperature_prime <= HIGH_BOUND){
            heater_on();
            program_state = HEATING;
          }
        }
        if(button_short_pressed_trigger == 1){
          program_state = BREW_INTERVALL_START;
        }
        else if (button_long_held_trigger == 1){
          program_state = ENTER_INTERVALL_TRAINING_MODE;
        }
        break;
      
      /*********************************************** BREW INTERVALL TIMER ***********************************************/
      case BREW_INTERVALL_START:
        //Serial.println("BREW_INTERVALL_START");
        buzzer_state = BUZZ_ON_SHORT;
        led_state = BLITZ_ON;
        brew_intervall_cnt = 0;
        brew_intervall_array_idx = 0;
        program_state = BREW_INTERVALL_STEP;
      break;
      
      case BREW_INTERVALL_STEP:
        //Serial.println("BREW_INTERVALL_STEP");
        brew_intervall_cnt++;
        if((((long)brew_intervall_cnt * (long)loop_intervall) / 1000) >= (long)brew_intervall_array[brew_intervall_array_idx]){
          buzzer_state = BUZZ_ON_SHORT;
          brew_intervall_cnt = 0;
          brew_intervall_array_idx++;
          if(brew_intervall_array_idx >= brew_intervall_array_size){
            /* end the brewing session. You're running out of intervalls! */
            buzzer_state = BUZZ_ON_LONG;
            led_state = LED_ON_ST;
            brew_interval_end_cnt = 0;
            program_state = BREW_INTERVALL_END_WAIT_AND_REMINDER;
          }
          else if (brew_intervall_array[brew_intervall_array_idx] == 0){
            /* end the brewing session regularly. Enjoy your coffee! */
            buzzer_state = BUZZ_ON_LONG;
            led_state = BLINK_ON;
            brew_interval_end_cnt = 0;
            program_state = BREW_INTERVALL_END_WAIT_AND_REMINDER;
          }
        }
      break;
        
      case BREW_INTERVALL_END_WAIT_AND_REMINDER:
        /* okay, finish your brew, clean up your stuff, but don't forget to turn this thing off. */
        //Serial.println("BREW_INTERVALL_END_WAIT_AND_REMINDER");
        brew_interval_end_cnt++;
        if((((long)brew_interval_end_cnt * (long)loop_intervall) / 1000) >= brew_interval_end_wait_time){
          brew_interval_end_cnt = 0;
          led_state = BLINK_ON;
          buzzer_state = BUZZ_ON_LONG;
        }
      break;

      /*********************************************** INTERVALL TRAINER ***********************************************/
      case ENTER_INTERVALL_TRAINING_MODE:
        heater_off();
        led_state = BLINK_ON;
        buzzer_state = BUZZ_ON_LONG;
        program_state = INTERVALL_TRAINING_START;
        break;

      case INTERVALL_TRAINING_START:
        if (button_short_pressed_trigger == 1){
          buzzer_state = BUZZ_ON_SHORT;
          for(brew_intervall_array_idx = 0; brew_intervall_array_idx < brew_intervall_array_size; brew_intervall_array_idx++){
            brew_intervall_array[brew_intervall_array_idx] = 0;
          }
          brew_intervall_training_cnt = 0;
          brew_intervall_array_idx = 0;
          program_state = INTERVALL_TRAINING_COUNTING_AND_ADDING;
        }
        break;
        
      case INTERVALL_TRAINING_COUNTING_AND_ADDING:
        brew_intervall_training_cnt++;
        if (button_short_pressed_trigger == 1){
          buzzer_state = BUZZ_ON_SHORT;
          brew_intervall_array[brew_intervall_array_idx] = uint16_t(((long)brew_intervall_training_cnt * (long)loop_intervall) / 1000); /* store in seconds */
          brew_intervall_array_idx++;
          if(brew_intervall_array_idx >= brew_intervall_array_size) {
            /* maximum number of intervalls reached: end training */
            buzzer_state = BUZZ_ON_LONG;
            led_state = LED_ON_ST;
            program_state = INTERVALL_TRAINING_END;
          }
          brew_intervall_training_cnt = 0;
        }
        else if (button_long_held_trigger == 1){
          /* manual end of training with longpressing the button */
          buzzer_state = BUZZ_ON_LONG;
          led_state = LED_ON_ST;
          program_state = INTERVALL_TRAINING_END;
        }
        break;
      
      case INTERVALL_TRAINING_END:

        /* check print the new array for debugging: */
        insanety_found = 0;
        Serial.println("New brew_intervall_array:");
        for(brew_intervall_array_idx = 0; brew_intervall_array_idx < brew_intervall_array_size; brew_intervall_array_idx++){
          Serial.println(String(brew_intervall_array[brew_intervall_array_idx]));
          if (brew_intervall_array[brew_intervall_array_idx] >= max_intervall_seconds){
            /* no intervall is allowed to be longer than 5 minutes! */
            insanety_found = 1;
            program_state = PROGRAM_ERROR;
            break;
          }
        }
        brew_intervall_array_idx = 0;
        Serial.println();

        /* read modify write the number of EEPROM write-cycles: */
        EEPROM.get(0, eeprom_write_cycle_count);
        eeprom_write_cycle_count++;
        EEPROM.put(0, eeprom_write_cycle_count);

        eeprom_write_cycle_count = 0;
        EEPROM.get(0, eeprom_write_cycle_count);
        Serial.println("New number of EEPROM write cycles:");
        Serial.println(String(eeprom_write_cycle_count));
        Serial.println();

        /* write new array to EEPROM: */
        write_int_array_to_EEPROM(2, brew_intervall_array, brew_intervall_array_size);

        /* varify: */
        Serial.println("Newly written and read array:");
        read_int_array_from_EEPROM(2, brew_intervall_array, brew_intervall_array_size);
        for(brew_intervall_array_idx = 0; brew_intervall_array_idx < brew_intervall_array_size; brew_intervall_array_idx++){
          Serial.println(String(brew_intervall_array[brew_intervall_array_idx]));
          if (brew_intervall_array[brew_intervall_array_idx] >= max_intervall_seconds){
            /* no intervall is allowed to be longer than 5 minutes! */
            insanety_found = 1;
            program_state = PROGRAM_ERROR;
            break;
          }
        }
        brew_intervall_array_idx = 0;
        Serial.println();

        /* finish: */
        led_state = LED_OFF_ST;
        buzzer_state = BEEPING_ON;
        if(insanety_found == 1){
          program_state = PROGRAM_ERROR;
        }
        else {
          program_state = PROGRAM_IDLE;
        }
          
        break;
        
      case PROGRAM_IDLE:
        Serial.println("IDLING, CHILLING, DOING NOTHING AT ALL...");
        break;
        
      case PROGRAM_ERROR:
        Serial.println("ERROR ERROR ERROR. Something went terribly wrong.");
        buzzer_state = BUZZ_ON;
        led_state = BLINK_ON;
        program_state = PROGRAM_IDLE;
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