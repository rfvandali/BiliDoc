// PHYSICAL CONNECTIONS ****************************************************
/* BiliDoc functional code v3.0
 
 Liquid Crystal Display - LCD:
 * LCD RS pin - digital pin 12
 * LCD Enable pin - digital pin 11
 * LCD D4 pin - digital pin 5
 * LCD D5 pin - digital pin 4
 * LCD D6 pin - digital pin 3
 * LCD D7 pin - digital pin 2
 * 10K resistor:
 * ends to ground and +5V (LCD pins 1 and 2 respectively)
 * wiper to LCD VO pin (i.e LCD pin 3)
 
 SD card attached to SPI bus as follows:
 * SD Pin 1 (CS)    - Digital pin 4 (ATmega pin 6)
 * SD Pin 2 (MOSI)  - Digital pin 11 (Atmega pin 17)
 * SD Pin 3         - Ground
 * SD Pin 4         - Power 3.3V
 * Sd Pin 5 (CLK)   - Digital pin 13 (Atmega pin 19)
 * SD Pin 6         - Ground
 * SD Pin 7 (MISO)  - Digital pin 12 (Atmega pin 18)
 * SD Pin 8         - Nothing
 
 Buzzer
 * Buzzer Negative terminal - Ground
 * Buzzer Positive terminal - digital pin 3 (Atmega pin 5) via 100hm resistor
 
 Pololu Pushbutton Power Switch
 * Pololu OFF pin - digital pin 6 (Atmega pin 12)
 
 Indicator LED's
 * Green LED - Analog pin 2 (Atmega pin 25)
 * Red LED   - Analog pin 1 (Atmega pin 24)
 
 Transimpedance amplifier / Photo-sensing
 * Vout blue - Analog pin 4 (Atmega pin 27)
 * Vout UV   - Analog pin 3 (Atmega pin 26)
 
 Pushbutton
 * Main  - Digital pin 5 (Atmega pin 11)
 * Reset - RESET pin (Atmega pin 1)
 
 */

// CONSTANTS, GLOBALS AND LIBRARIES ****************************************                                                    
#include <math.h>          // Library for use of round, floor, etc...
#include <SD.h>            // Library used to read/write to SD card
#include <stdlib.h>        // Standard library
#include <string.h>        // String library
#include <EEPROM.h>        // Library to read/write to EEPROM
#include <LCD3Wire.h>      // Library for use of LCD (allows shift register)

#define UV_MAX 3              // LCD: Maximum allowable UV index value
#define BUTTON_PIN 5          // LCD: Arduino pin for user recognition
#define TT 900                // LCD: Initial treatment time in seconds
#define MAX_TSB_RESOL 10      // SD: Maximum index for TSB
#define MAX_TD_RESOL 1        // SD: Maximum index for treatment session
#define BLANK_CSV 2           // SD: Size of a new, blank csv file
#define TSB_EEPROM_ADR 0      // SD: Address of TSB_global in EEPROM
#define SESSION_EEPROM_ADR 4  // SD: Address of session_global in EEPROM
#define LCD_WIDTH 16          // SD: How many pixels in an LCD line
#define BUZZ_PIN 3            // BUZZER: Digital pin to piezo buzzer
#define RED_PIN A2            // LED: Analog pin as digital out for red LED
#define GREEN_PIN A1          // LED: Analog pin as dig. out for green LED
#define POLOLU_PIN 6          // POLOLU: Digital pin for auto-off
#define SHUTDOWN_DELAY 60000  // POLOLU: Time (ms) for automatic shutdown
#define UV_VOUT_PIN 3         // TRANS: Analog read pin for UV circuit
#define BLUE_VOUT_PIN 4       // TRANS: Analog read pin for blue circuit
#define Loop_time 20e-3       // TRANS: While loop time (s) - 1 period, 50Hz
#define V_IN 5.0              // TRANS: Supply voltage (volts)
#define Iteration_time 5e-4   // TRANS: Iteration time for light sampling. Where Iteration_time = Loop_time/#_of_period/samples_per_period. I.e. Iteration_time 0.5ms -> 40 samples per 50hz period

LCD3Wire lcd = LCD3Wire(2,8,9,7);// LCD: Initialize the LCD interface pins

double TSB_global;               // SD: Total Serum Bilirubin (umol/dL)
double BW_global;                // SD: Baby weight (g)
int session_global;              // SD: Treatment session
char baby_name_global[LCD_WIDTH];// SD: Name of the baby
File patient;                    // SD: Patient file
File therapy;                    // SD: Therapy file



// SETUP CODE *************************************************************
void setup() {
  pinMode(10,OUTPUT);            // Keep free to use SD card library
  pinMode(BUZZ_PIN,OUTPUT);      // Piezo buzzer pin as output
  pinMode(POLOLU_PIN,OUTPUT);    // Pololu digital pin as output
  pinMode(GREEN_PIN,OUTPUT);     // Green LED pin as ouput
  pinMode(RED_PIN,OUTPUT);       // Red LED pin as output
  pinMode(BUTTON_PIN,INPUT);     // Button pin as input

  lcd.init();                    // Initialises LCD
  lcd.clear();                   //
  lcd.cursorTo(1, 0);            // Device introduction message
  lcd.print("    Bili Doc    "); // 
  delay(2000);                   //  
  SDtest();                      // Check for an SD card
  patientScan();                 // Get TSB, BW & baby name from patient.csv
  therapyScan();                 // Check therapy.csv for newer TSB data
  lcd.clear();                   //
  lcd.cursorTo(1, 0);            //
  lcd.print("Welcome baby");     //
  lcd.cursorTo(2, 0);            // Personalised welcome baby message
  lcd.print(baby_name_global);   //
  lcd.cursorTo(2, 16);           //
  delay(2000);                   //
}



// MAIN LOOP *************************************************************
void loop() {
  int pushed = 0;                 // BUZZER:
  int buttonState;                // BUZZER: 
  int measured = 0;               // BUZZER: Marker for once per beep cycle
  int beep_gap = 2000;            // BUZZER: 
  double IrrBlue;                 // Blue: Irradiance in blue spectrum (W/m^2/nm)
  double UVI;                     // UVI: UV Index
  double V_to_UVI_Const = 1.2643125519;  // UVI: Voltage to UVI conversion constant (V*m^2*nm/W) = Ip_to_UVI_Const*Rf_UV
  double V_to_Irr_Const = 10.88065987;   // Blue: Voltage to blue irradiance conversion constant (V*m^2*nm/W) = Ip_to_Irr_Const*Rf_blue  

  lcd.clear();                    //
  lcd.cursorTo(1, 0);             //
  lcd.print("Press button to");   //
  lcd.cursorTo(2, 0);             // User presses button to test location
  lcd.print("begin testing.");    //
  lcd.cursorTo(2, 16);            //
  button();                       //

  lcd.clear();                    //
  lcd.cursorTo(1, 0);             //
  lcd.print("Press button to");   //
  lcd.cursorTo(2, 0);             // User presses button to accept location
  lcd.print("accept location.");  // 
  lcd.cursorTo(2, 16);            //
  unsigned long t0 = millis();

  while(pushed==0 || buttonState==LOW){
    if((millis()-t0)<(150)){
      digitalWrite(BUZZ_PIN,HIGH);
    }
    else if((millis()-t0)>=150 && (millis()-t0)<(150+beep_gap) && measured==0){
      unsigned long  t1  = millis();
      digitalWrite(BUZZ_PIN, LOW);      
      IrrBlue = sunlight(BLUE_VOUT_PIN,V_to_Irr_Const);
      UVI = sunlight(UV_VOUT_PIN,V_to_UVI_Const);
      measured = 1;
      unsigned long t2 = millis();
      beep_gap = - pow(IrrBlue*1.1664e10,0.3333333)+2000-(t2-t1); 
      
      if(UVI<3){
        digitalWrite(RED_PIN,LOW);
        digitalWrite(GREEN_PIN,HIGH);
      }
      else{
        digitalWrite(GREEN_PIN,LOW);
        digitalWrite(RED_PIN,HIGH);
      }    
    }
    else if((millis()-t0)>=(150+beep_gap)){
      t0 = millis();
      measured = 0;
    }     
    buttonState = digitalRead(BUTTON_PIN);
    if (buttonState == LOW){
      pushed = 1;
    }
  }
  digitalWrite(BUZZ_PIN,LOW);
  digitalWrite(RED_PIN,LOW);  
  digitalWrite(GREEN_PIN,LOW);  

  lcd.clear();
  lcd.cursorTo(1, 0);
  if (UVI <= UV_MAX) {                  // Checks UV Index calculations
    TSB_global = 4.33+0.692*TSB_global-5.59*IrrBlue-0.0829*(TT/60.0/60.0);
    lcd.print("Location is good");
    lcd.cursorTo(2, 0);
    lcd.print("for treatment.");
    lcd.cursorTo(2, 16);
    delay(2000);
    lcd.clear();
    lcd.cursorTo(1, 0);
    lcd.print("Push button to");
    lcd.cursorTo(2, 0);
    lcd.print("begin countdown.");
    lcd.cursorTo(2, 16);
    button();
    delay(200);

    // Counts down treatment, saves data and buzzes and displays message
    countdown();
    therapyPrint(IrrBlue,UVI); 
    buzz();

    // Print additional message if TSB is below phototherapy threshold
    if(TSB_global<=11){              //
      lcd.clear();                   //
      lcd.cursorTo(1, 0);            //
      lcd.print("Jaundice state");   //
      lcd.cursorTo(2, 0);            //
      lcd.print("is now safe.");     //
      lcd.cursorTo(2, 16);           // Message if jaundice severity improved
      delay(2000);                   // to a safe level
      lcd.clear();                   //
      lcd.cursorTo(1, 0);            //
      lcd.print("Return to hospi-"); //
      lcd.cursorTo(2, 0);            //
      lcd.print("tal for advice.");  //
      lcd.cursorTo(2, 16);           //
      delay(2000);                   //
    }    
    lcd.clear();                     //
    lcd.cursorTo(1, 0);              //
    lcd.print("Please turn off");    //
    lcd.cursorTo(2, 0);              // Turn off request
    lcd.print("device.");            //
    lcd.cursorTo(2, 16);             //
    delay(SHUTDOWN_DELAY);           // Waits 60 seconds before turning off
    digitalWrite(POLOLU_PIN,HIGH);   // Auto turn-off
  } 
  else {
    lcd.print("UVI is too high");    //
    lcd.cursorTo(2, 0);              //
    lcd.print("for treatment.");     //
    lcd.cursorTo(2, 16);             //
    delay(2000);                     //
    lcd.clear();                     // Message for UVI > threshold
    lcd.cursorTo(1, 0);              //
    lcd.print("Try location");       //
    lcd.cursorTo(2, 0);              //
    lcd.print("with less light.");   //
    lcd.cursorTo(2, 16);             //
    delay(2000);                     //
  }
}



// FUNCTION DEFINITIONS ***************************************************
// Tests for the presence of an SD card in the SD slot
void SDtest(void){
  int printed = 0;                        // Marker to print once
  unsigned long t0 = millis();  
  while(!SD.begin(4)){                    // While SD is available
    if(printed==0){
      lcd.clear();                        //
      lcd.cursorTo(1, 0);                 //
      lcd.print("Please insert SD");      //
      lcd.cursorTo(2, 0);                 // Message if no card present
      lcd.print("card to begin.");        //
      lcd.cursorTo(2, 16);                //
    }
    printed = 1;
    if((millis()-t0)>SHUTDOWN_DELAY){     // 
      digitalWrite(POLOLU_PIN,HIGH);      // Auto turn-off
    }                                     //
  }
}



// Tests if the main pusbutton has been pressed
void button(void){
  int buttonState_func;                   // Marker for button state
  int pushed_func = 0;                    // Marker for button press 
  unsigned long t0 = millis();            // Marker for waiting time
  
  while (pushed_func==0||buttonState_func==LOW){ // Check if button was pushed
    buttonState_func = digitalRead(BUTTON_PIN); // Read the button pin
    
    if (buttonState_func == LOW){         //
      pushed_func = 1;                    // Button has been pushed
    }                                     //
    if((millis()-t0)>SHUTDOWN_DELAY){     // 
      digitalWrite(POLOLU_PIN,HIGH);      // Auto turn-off
    }                                     //
  }
}



// 
void patientScan(void){
  int printed = 0;                        // Marker for error message printed
  int num_recognised = 0;
  int num_string_index = 0;
  int num_value_index = 0;
  char num_string[LCD_WIDTH];
  double num_value[2]; 
  unsigned long t0 = millis(); 
  patient = SD.open("patient.csv",O_READ);// Open the patient file to read
  while(!therapy) {                       // If the file didn't open, print an error and wait
    if(printed==0){                       // Print only once to screen rather than refreshing
      lcd.clear();                        //
      lcd.cursorTo(1, 0);                 //
      lcd.print("Error opening");         // Error message
      lcd.cursorTo(2, 0);                 //
      lcd.print("patient.csv");           //
      lcd.cursorTo(2, 16);                //
    }
    printed = 1;
    if((millis()-t0)>SHUTDOWN_DELAY){     // Auto turn-off
      digitalWrite(POLOLU_PIN,HIGH);
    }  
  }
  while (patient.available()) {           // Read from the patient file
    char character = patient.read();      // Assign each character from patient.csv 
    if(character==',') {                  // If new cell 
      num_recognised = 1;                 // Marker for a new cell
    }    
    if(character!=',' && num_recognised==1){
      if(character!='\n' && character!='\r'){
        if(num_value_index==2){
          baby_name_global[num_string_index] = character;
        }
        else{
          num_string[num_string_index] = character;            
        }
        num_string_index++;
      }
      else {
        num_recognised = 0;
        num_value[num_value_index] = atof(num_string);            
        num_string_index = 0;
        num_value_index++;
      }
    }  
  }
  patient.flush();                         // Close the file when finished
  patient.close();                         //
  TSB_global = num_value[0];               // Store the TSB value
  BW_global = num_value[1];                // Store the BW value
}


// Writes initial patient values to therapy.csv and checks size
void therapyScan(void){                    
  therapy = SD.open("therapy.csv",O_CREAT|O_APPEND|O_WRITE);  // Open, append or create therapy.csv

  if(therapy.size()>BLANK_CSV){                       // Checks for previous session by comparing file size to blank csv 
    TSB_global = EEPROM_read(TSB_EEPROM_ADR);         // Overwrite the initial TSB value with the final TSB value of last treatment 
    session_global = EEPROM_read(SESSION_EEPROM_ADR); // Find what the previous treatment session was
  }
  else {
    session_global = 0;                    // Asigns zero to treatment session, since treatment has not yet occured
    therapy.print("Treatment Session,Total Serum Bilirubin (umol/dL),Blue Irradiance (uW/cm^2/nm),UV Index,Treatment time (hours)\r\n");
    therapy.print(session_global);         // Prints treatment session zero
    therapy.print(",");                    // Moves to new column
    therapy.print(TSB_global);             // Prints TSB value from patient file.
    for(int i=0;i<3;i++){                  // No data for these cells on first therapy entry
      therapy.print(",");                  // Moves to new column 
      therapy.print("-");                  // Prints '-', since treatment has not yet occured
    } 
    therapy.print("\r\n");                 // Moves to new line
  }
  therapy.flush();                         // Close the file when finished
  therapy.close();                         //
}


// Reads from EEPROM, one byte at a time
double EEPROM_read(int address){            
  double value = 0.0;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++){
    *p++ = EEPROM.read(address++);
  }
  return value;
}



void therapyPrint(double IrrBlue, double UVI){        // Writes treatment data to therapy.csv
  therapy = SD.open("therapy.csv",O_APPEND|O_WRITE);  // Write or append to the therapy file
  therapy.print(session_global+1);       // prints the treatment session (=previous treatment session+1)
  therapy.print(",");
  therapy.print(TSB_global,3);           // Prints the TSB value in umol/dl
  therapy.print(",");
  therapy.print(IrrBlue*100,3);          // Prints the blue irradiance in uW/cm^2/nm
  therapy.print(",");
  therapy.print(UVI,3);                  // Prints the UV Index, unitless
  therapy.print(",");
  therapy.println(TT/60.0/60.0);         // Prints the treatment time HOURS

  EEPROM_write(TSB_EEPROM_ADR,TSB_global);// Stores the TSB value in EEPROM, for the following treatment
  EEPROM_write(SESSION_EEPROM_ADR,session_global+1);   // Stores the treatment session in EEPROM, for the following treatment

  therapy.flush();                       // Close and flush the file when finished 
  therapy.close();                       //
}


// Stores a 4 byte double to EEPROM, one byte at a time
void EEPROM_write(int address, double value){          
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++){
    EEPROM.write(address++, *p++);
  }
}



void countdown(void){         // Countdown timer for treatment
  int mins = floor(TT/60);    // Calculates how many minutes remain from treatment beginning
  int sec = fmod(TT,60);      // Calculates how many seconds remain from treatment beginning 
  int block_col = 1;          // loading bar position counter
  int i = 0;

  lcd.clear();                //
  lcd.cursorTo(1, 0);         // Template for countdown message
  lcd.print("Time left= ");   // (doesn't refresh every second)
  // Displays the treatment time remaining:
  while (sec<=59 && sec>=0) {    
    unsigned long t0 = micros();
    lcd.cursorTo(1, 11);      //
    if (mins<10) {            //        
      lcd.print("0");         // Prints the running value of remaining minutes
    }                         //
    lcd.print(mins);          //
    lcd.print(":");
    if (sec<10) {             //     
      lcd.print("0");         // Prints the running value of remaining seconds
    }                         //
    lcd.print(sec);           //

    if (sec!=0) {             //
      sec--;                  // Decreases the seconds counter
    }                         //
    else {                    //
      sec=59;                 // Checks if minute transition
      mins--;                 //
    }
    if((TT-(1+sec+mins*60.0))==round(TT/16.0*block_col)){ // calculates when to increment the loading bar
      for(i; i<(block_col); i++){  //
        lcd.cursorTo(2, i);        // Prints the loading bar symbols
        lcd.print(">");            //
      }                            //
      lcd.cursorTo(2,16);
      block_col++;                 // increments counter for the loading bar position
    }
    unsigned long code_delay = (micros()-t0)/1000;  // Delay to read through code for each loop
    delay(1000 - code_delay);      // Delay by a second       
 
    if (mins==-1 && sec==59) {     // Checks if treament is over
      lcd.clear();                 //
      lcd.cursorTo(1, 0);          //
      lcd.print("Treatment has");  //
      lcd.cursorTo(2, 0);          // Print treatment is over
      lcd.print("finished.");      //
      lcd.cursorTo(2, 16);         //
      return;
    }
  }
}



// Beep which sounds at the end of treatment
void buzz(void){
  int pushed = 0;                         // Marker for pushbutton press
  int beeps = 0;                          // Beep cycle counter
  int buttonState;                        // Marker for pushbutton state
  unsigned long t0 = millis();            // Marker for timing 0
  while(1){      
    buttonState = digitalRead(BUTTON_PIN);// Read the button state
    if (buttonState == LOW){              //
      pushed = 1;                         // A press has been registered
    }                                     //
    if (pushed==1 && buttonState==HIGH){  //
      digitalWrite(BUZZ_PIN,LOW);         // Stop beeping, button was pressed
      return;                             //
    }                                     //
    if((millis()-t0)<170){
      digitalWrite(BUZZ_PIN, HIGH);
    }
    else if((millis()-t0)>=170 && (millis()-t0)<260){
      digitalWrite(BUZZ_PIN, LOW);
    }
    else if((millis()-t0)>=260 && (millis()-t0)<530){
      digitalWrite(BUZZ_PIN, HIGH);
    }
    else if((millis()-t0)>=530){
      digitalWrite(BUZZ_PIN, LOW);
      if((millis()-t0)>1530){        
        beeps++;                          // Increment the beep cycle count
        if(beeps==15){                    // 15 beep cycles then stops
          return;                         //
        }
        t0 = millis();
      }
    }
  }
}


// Samples the transimpedance output and outputs the irradiance/UVI
double sunlight(int analog_PIN, double V_to_X_Const) {
  int Iteration_number = Loop_time/Iteration_time; // Number of iterations in sampling
  double Vout_raw;                                 // Raw voltage out value
  double Vout_sum_raw = 0;                         // Raw value of the summation of vout over loop_time
  double Vout_avg;                                 // The DC average output voltage read over loop_time
  double measurement;                              // Irradiance/UVI for the blue/UV circuit
  unsigned long t0;                                // Timing marker 0
  unsigned long t1;                                // Timing marker 1

  for(int i=0;i<Iteration_number;i++) {            // Sampling loop
    t0 = micros();                                 // Instantiates timing marker 0
    Vout_raw = analogRead(analog_PIN);             // Reads transimpedance output pin
    Vout_sum_raw += Vout_raw;                      // Sums the raw voltage value over loop_time
    t1 = micros()-t0;                              // Instantiates timing marker 2
    delayMicroseconds(Iteration_time*1e6-t1);      // Delay of iteration_time, accounting for code delay
  }
  Vout_avg = (Vout_sum_raw/(1023.0*Iteration_number))*V_IN;  // Average value over loop_time
  measurement = Vout_avg/V_to_X_Const;             // Average irradiance/UVI over loop_time

  return measurement;
}



