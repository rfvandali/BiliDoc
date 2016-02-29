/// PHYSICAL CONNECTIONS *******************************************
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

// CONSTANTS, GLOBALS AND LIBRARIES ********************************                                                    // LCD: Library for use of LCD (allows use of a shift register)
#include <math.h>                                                          // LCD: Library for use of round, floor, etc...
#include <SD.h>                                                            // SD: Library used to read/write to SD card - uses the FAT library.
#include <stdlib.h>                                                        // SD: Check whether need to actually include
#include <string.h>                                                        // SD: Check whether need to actually include
#include <EEPROM.h>  
#include <LCD3Wire.h>
// SD: Library to read/write to EEPROM

#define UV_MAX 3                                                           // LCD: Maximum allowable UV index value
#define BUTTON_PIN 5                                                       // LCD: Arduino pin for user recognition
#define TT 64                                                              // LCD: Initial treatment time in seconds
#define MAX_TSB_RESOL 10                                                   // SD: Maximum index needed to store TSB
#define MAX_TD_RESOL 1                                                     // SD: Maximum index needed to store treatment session
#define BLANK_CSV 2                                                        // SD: CURRENTLY NOT BEING USED!!!! Size of a new, blank csv file
#define TSB_EEPROM_ADR 0                                                   // SD: Address of TSB_global in EEPROM
#define SESSION_EEPROM_ADR 4                                               // SD: Address of session_global in EEPROM
#define LCD_WIDTH 16                                                       // SD: How many pixels in an LCD line
#define BUZZ_PIN 3                                                         // BUZZER: Digital pin connected to piezo buzzer
#define RED_PIN A2                                                         // LED: Analog pin acting as digital out for red indicator led
#define GREEN_PIN A1                                                       // LED: Analog pin acting as digital out for green indicator led
#define POLOLU_PIN 6                                                       // POLOLU: Digital pin for auto-off
#define SHUTDOWN_DELAY 60000                                               // POLOLU: Time (in ms) after which the device automatically shutsdown
#define UV_VOUT_PIN 3                                                      // TRANS: Analog read pin for UV circuit
#define BLUE_VOUT_PIN 4                                                    // TRANS: Analog read pin for blue circuit
#define Loop_time 20e-3                                                    // TRANS: While loop time (s) - 1 period of 50Hz
#define Iteration_time 5e-4                                                // TRANS: Iteration time for light sampling. Where Iteration_time = Loop_time/#_of_period/samples_per_period. I.e. Iteration_time 0.5ms -> 40 samples per 50hz period
#define pi 3.14159265                                                      // TRANS: Pi
#define V_IN 5.0                                                           // TRANS: Supply voltage (volts)


LCD3Wire lcd = LCD3Wire(2, 8, 9, 7);                                       // LCD: Initialize the library with the numbers of the interface pins

double TSB_global;                                                         // SD: Total Serum Bilirubin (umol/dL) !!!GLOBAL VARIABLE!!! 
double BW_global;                                                          // SD: Baby weight (g) !!!GLOBAL VARIABLE!!!
int session_global;                                                        // SD: Treatment session !!!GLOBAL VARIABLE!!!
char baby_name_global[LCD_WIDTH];                                          // SD: Name of the baby !!!GLOBAL VARIABLE!!!
File test;                                                                 // SD:
File patient;                                                              // SD:
File therapy;                                                              // SD:



// SETUP CODE *****************************
void setup() {
  pinMode(10, OUTPUT);                          // Must be kept free for SD card, otherwise SD library functions wont work
  pinMode(BUZZ_PIN,OUTPUT);                     // Piezo buzzer pin declared as an output
  pinMode(POLOLU_PIN,OUTPUT);                   // Pololu digital auto-off pin declared as an output
  pinMode(GREEN_PIN, OUTPUT);                   // Green LED indicator pin declared as ouput
  pinMode(RED_PIN, OUTPUT);                     // Red LED indicator pin declared as output
  pinMode(BUTTON_PIN, INPUT);                   // Button pin declared as input

  lcd.init();                                   // Initialises LCD
  lcd.clear();                                  //
  lcd.cursorTo(1, 0);                           // Device introduction message
  lcd.print("    Bili Doc    ");                // 
  delay(2000);                                  //  
  SDtest();                                     // Check for the presence of an SD card
  patientScan();                                // Retrieve TSB, BW and baby name from the patient file. CAN MOVE LOCATION OF THIS CODE IN SETUP
  therapyScan();                                // Check the therapy file for newer TSB data. CAN MOVE LOCATION OF THIS CODE IN SETUP
  lcd.clear();                                  //
  lcd.cursorTo(1, 0);                           //
  lcd.print("Welcome baby");                    //
  lcd.cursorTo(2, 0);                           // Personalised welcome baby message
  lcd.print(baby_name_global);                  //
  lcd.cursorTo(2, 16);                          //
  delay(2000);                                  //
}



// MAIN LOOP ******************************
void loop() {
  int pushed = 0;                               // Beep: Marker
  int buttonState;                              // Beep:
  int measured = 0;                             // Beep: Marker to measure photodiode once per beep cycle
  int beep_gap = 2000;                              // Beep:
  double IrrBlue;                               // Blue: Irradiance in blue spectrum (W/m^2/nm)
  double UVI;                                   // UVI: UV Index
  double V_to_Irr_Const = 10.88065987;             // Blue: Voltage to blue irradiance conversion constant (V*m^2*nm/W) i.e = Ip_to_Irr_Const*Rf_blue  
  double V_to_UVI_Const = 1.2643125519;           // UVI: Voltage to UVI conversion constant (V*m^2*nm/W) i.e = Ip_to_UVI_Const*Rf_UV

  lcd.clear();                                  //
  lcd.cursorTo(1, 0);                           //
  lcd.print("Press button to");                 //
  lcd.cursorTo(2, 0);                           // User presses button to test location
  lcd.print("begin testing.");                  //
  lcd.cursorTo(2, 16);                          //
  button();                                     //

  lcd.clear();
  lcd.cursorTo(1, 0);
  lcd.print("Press button to");
  lcd.cursorTo(2, 0);
  lcd.print("accept location.");
  lcd.cursorTo(2, 16);
  unsigned long t0 = millis();

  while(pushed==0 || buttonState==LOW){
    if((millis()-t0)<(150)){
      digitalWrite(BUZZ_PIN, HIGH);
    }
    else if((millis()-t0)>=150 && (millis()-t0)<(150+beep_gap) && measured==0){
      unsigned long  t1  = millis();
      digitalWrite(BUZZ_PIN, LOW);      
      IrrBlue = sunlight(BLUE_VOUT_PIN,V_to_Irr_Const);
      UVI = sunlight(UV_VOUT_PIN,V_to_UVI_Const);
      measured = 1;
      unsigned long t2 = millis();
      //beep_gap = -3600*IrrBlue+2000-(t2-t1);          Gradient and intercept can be adjusted to give good beep ratio. Maybe make this relationship non-linear
      beep_gap = - pow(IrrBlue*1.1664e10,0.3333333)+2000-(t2-t1);                    // Gradient and intercept can be adjusted to give good beep ratio. Maybe make this relationship non-linear
      if(UVI<3){
        digitalWrite(RED_PIN, LOW);
        digitalWrite(GREEN_PIN, HIGH);
      }
      else{
        digitalWrite(GREEN_PIN, LOW);
        digitalWrite(RED_PIN, HIGH);
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
  digitalWrite(RED_PIN, LOW);  
  digitalWrite(GREEN_PIN, LOW);  


  // Checks Blue and UV light calculations
  lcd.clear();
  lcd.cursorTo(1, 0);
  if (UVI <= UV_MAX) {
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

    // Counts down treatment time and then buzzes and displays message
    countdown();
    therapyPrint(IrrBlue,UVI); 
    buzz();

    if(TSB_global<=11){
      lcd.clear();
      lcd.cursorTo(1, 0);
      lcd.print("Jaundice state");
      lcd.cursorTo(2, 0);
      lcd.print("is now safe.");
      lcd.cursorTo(2, 16);
      delay(2000);
      lcd.clear();
      lcd.cursorTo(1, 0);
      lcd.print("Return to hospi-");
      lcd.cursorTo(2, 0);
      lcd.print("tal for advice.");
      lcd.cursorTo(2, 16);
      delay(2000);
    }    
    lcd.clear();
    lcd.cursorTo(1, 0);
    lcd.print("Please turn off");
    lcd.cursorTo(2, 0);
    lcd.print("device.");
    lcd.cursorTo(2, 16);
    delay(SHUTDOWN_DELAY);
    digitalWrite(POLOLU_PIN,HIGH);              // Auto turn-off after about 20 seconds
  } 
  else {
    lcd.print("UVI is too high");
    lcd.cursorTo(2, 0);
    lcd.print("for treatment.");
    lcd.cursorTo(2, 16);
    delay(2000);
    lcd.clear();
    lcd.cursorTo(1, 0);
    lcd.print("Try location");
    lcd.cursorTo(2, 0);
    lcd.print("with less light.");
    lcd.cursorTo(2, 16);
    delay(2000);
  }
}



// FUNCTION DEFINITIONS *********************************
void SDtest(void){
  int printed = 0;
  unsigned long t0 = millis();  
  while(!SD.begin(4)) {
    if(printed==0){
      lcd.clear();
      lcd.cursorTo(1, 0);
      lcd.print("Please insert SD");
      lcd.cursorTo(2, 0);
      lcd.print("card to begin.");
      lcd.cursorTo(2, 16);
    }
    printed = 1;
    if((millis()-t0)>SHUTDOWN_DELAY){                                 // Auto turn-off after about 20 seconds
      digitalWrite(POLOLU_PIN,HIGH);
    }  
  }
}



void button(void){
  int buttonState_func;
  int pushed_func = 0;
  unsigned long t0 = millis();  
  while (pushed_func==0 || buttonState_func==LOW) {     
    buttonState_func = digitalRead(BUTTON_PIN);
    if (buttonState_func == LOW){
      pushed_func = 1;
    }
    if((millis()-t0)>SHUTDOWN_DELAY){                                 // Auto turn-off after about 20 seconds
      digitalWrite(POLOLU_PIN,HIGH);
    }  
  }
}



void patientScan(void){
  int printed = 0;                                           // Marker variable to check if error message has been printed
  int num_recognised = 0;
  int num_string_index = 0;
  int num_value_index = 0;
  char num_string[LCD_WIDTH];
  double num_value[2]; 
  unsigned long t0 = millis(); 
  patient = SD.open("patient.csv", O_READ);               // Open the patient file for reading   CHANGE TO O_READ
  while(!therapy) {                                          // If the file didn't open, print an error and wait til its rectified
    if(printed==0){                                          // Print only once to screen rather than continually refreshing
      lcd.clear();
      lcd.cursorTo(1, 0);
      lcd.print("Error opening");
      lcd.cursorTo(2, 0);
      lcd.print("patient.csv");
      lcd.cursorTo(2, 16);
    }
    printed = 1;
    if((millis()-t0)>SHUTDOWN_DELAY){                                 // Auto turn-off after about 20 seconds
      digitalWrite(POLOLU_PIN,HIGH);
    }  
  }
  while (patient.available()) {                              // Read from the patient file
    char character = patient.read();                         // Assign each chracter from patient the variable, character
    if(character==',') {
      num_recognised = 1;
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
  patient.flush();                                           // Close the file when finished
  patient.close();                                           //
  TSB_global = num_value[0];
  BW_global = num_value[1];
}



void therapyScan(void){
  therapy = SD.open("therapy.csv", O_CREAT | O_APPEND | O_WRITE);             // Open, append or create a new therapy file if not present

  if(therapy.size()>BLANK_CSV){                            // Checks if there was a previous session by comparing file size to a blank csv 
    TSB_global = EEPROM_read(TSB_EEPROM_ADR);              // Overwrite the initial TSB value with the final TSB value of last treatment 
    session_global = EEPROM_read(SESSION_EEPROM_ADR);      // Find what the previous treatment session was
  }
  else {
    session_global = 0;                                    // Asigns zero to treatment session, since treatment has not yet occured
    therapy.print("Treatment Session,Total Serum Bilirubin (umol/dL),Blue Irradiance (uW/cm^2/nm),UV Index,Treatment time (hours)\r\n");
    therapy.print(session_global);                         // Prints treatment session zero
    therapy.print(",");
    therapy.print(TSB_global);                             // Prints TSB value from patient file.
    for(int i=0;i<3;i++){                                  // 
      therapy.print(",");                                  // Prints all other columns with '-', since treatment has not yet occured
      therapy.print("-");                                  //
    } 
    therapy.print("\r\n");                                 // Moves to new line
  }
  therapy.flush();                                         // Close the file when finished
  therapy.close();                                         //
}



double EEPROM_read(int address)                            // Stores a 4 byte double to EEPROM, one byte at a time
{
  double value = 0.0;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++){
    *p++ = EEPROM.read(address++);
  }
  return value;
}



void therapyPrint(double IrrBlue, double UVI){

  therapy = SD.open("therapy.csv", O_APPEND | O_WRITE);   // Write or append to the therapy file
  therapy.print(session_global+1);                        // prints the treatment session (=previous treatment session+1)
  therapy.print(",");
  therapy.print(TSB_global,3);                              // Prints the TSB value in umol/dl
  therapy.print(",");
  therapy.print(IrrBlue*100,3);                             // Prints the blue irradiance in uW/cm^2/nm
  therapy.print(",");
  therapy.print(UVI,3);                                     // Prints the UV Index
  therapy.print(",");
  therapy.println(TT/60.0/60.0);                          // Prints the treatment time HOURS

  EEPROM_write(TSB_EEPROM_ADR,TSB_global);                // Stores the TSB value in EEPROM, for the following treatment
  EEPROM_write(SESSION_EEPROM_ADR,session_global+1);        // Stores the treatment session in EEPROM, for the following treatment

  therapy.flush();                                        // Close and flush the file when finished 
  therapy.close();                                        //
}



void EEPROM_write(int address, double value)
{
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++){
    EEPROM.write(address++, *p++);
  }
}



void countdown(void){
  int mins = floor(TT/60);                      // Calculates how many minutes remain from treatment beginning
  int sec = fmod(TT,60);                        // Calculates how many seconds remain from treatment beginning 
  int block_col = 1;
  int i = 0;
  // LCD: Initial treatment time in seconds
  lcd.clear();
  lcd.cursorTo(1, 0);
  lcd.print("Time left= ");
  // Displays the treatment time remaining:
  while (sec<=59 && sec>=0) {    
    unsigned long t0 = micros();
    lcd.cursorTo(1, 11);
    // Displays the running value of remaining minutes         
    if (mins<10) {
      lcd.print("0");
    } 
    lcd.print(mins);
    lcd.print(":");
    // Displays the running value of remaining seconds      
    if (sec<10) {
      lcd.print("0");
    } 
    lcd.print(sec);
    // Updates the time remaining
    if (sec!=0) {
      sec--;
    } 
    else {
      sec=59;
      mins--;
    }
    if((TT-(1+sec+mins*60.0))==round(TT/16.0*block_col)){
      for(i; i<(block_col); i++){
        lcd.cursorTo(2, i);
        lcd.print(">");
      }
      lcd.cursorTo(2,16);
      block_col++;
    }
    // Delays for one second 
    unsigned long code_delay = (micros()-t0)/1000;
    delay(1000 - code_delay);          
    // Checks if treament time is over and prints to the screen
    if (mins==-1 && sec==59) {
      lcd.clear();
      lcd.cursorTo(1, 0);
      lcd.print("Treatment has");
      lcd.cursorTo(2, 0);
      lcd.print("finished.");
      lcd.cursorTo(2, 16);
      return;
    }
  }
}



void buzz(void){
  int pushed = 0;
  int beeps = 0;
  int buttonState;
  unsigned long t0 = millis();  
  while(1){      
    buttonState = digitalRead(BUTTON_PIN);
    if (buttonState == LOW){
      pushed = 1;
    }
    if (pushed==1 && buttonState==HIGH){
      digitalWrite(BUZZ_PIN,LOW);
      return;
    }
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
        beeps++;
        if(beeps==15){
          return;
        }
        t0 = millis();
      }
    }
  }
}


double sunlight(int analog_PIN, double V_to_X_Const) {
  //Internal Variables to the Function 
  int Iteration_number = Loop_time/Iteration_time;                // Number of iterations in light sampling
  double Vout_raw;                                                   // Raw voltage out value
  double Vout_sum_raw = 0;                                               // Raw value of the summation of vout over loop_time.
  double Vout_avg;                                                // The DC average output voltage read over loop_time
  double measurement;      // Irradiance for the blue circuit / UVI for the UV circuit
  unsigned long t0;
  unsigned long t1;

  // Light sampling: Records the voltage at the pin across loop_time and then takes the average value
  for(int i=0;i<Iteration_number;i++) {
    t0 = micros();
    Vout_raw = analogRead(analog_PIN);                     // Reads Vout
    Vout_sum_raw += Vout_raw;
    t1 = micros()-t0;
    delayMicroseconds(Iteration_time*1e6-t1);
  }
  Vout_avg = (Vout_sum_raw/(1023.0*Iteration_number))*V_IN;
  measurement = Vout_avg/V_to_X_Const;

  return measurement;
}







