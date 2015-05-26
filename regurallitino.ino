/*
Rally computer

Main goals:
- basic chronometer to time stage and checkpoint times
- wheelspeed sensing from ABS sensor for speedometer, distance travelled and live average speed calculation in stage/between checkpoints
- speedometer calibration mode by measuring given distance and entering wheelspeed signal count for a given distance (1km?)
- acceleration measuring mode (0-100kph)
- buttons should be separate for start and stop, and for checkpoint (probably need a dedicated mode button)

- (optional) enter stage target time and average speed (get notifications about finish time with buzzer, e.g. 10,5,4,3,2,1 secs out)
- (optional) trackday mode, for timing laps
- (optional) replay mode, for repeat/double regularity stages
- (optional) possibility to restart arduino without losing chrono time (RTC chrono + EEPROM?)

when chrono starts start measuring wheel signals. after X millis, adjust speedometer and recalculate average and odometer. do the same for checkpoint specific averages and timers.
wheelspeed should use interrupts and volatile types 

*/
#define USE_RTC_CHRONO false
#define USE_SERIAL true

#include <Wire.h>
#include <Time.h>
#include <DS3232RTC.h>

tmElements_t tm;
char buffer[21];

bool buttonState;
bool previousButtonState;
bool dirtyState;
int buttonDebounceCounter;


bool chronoStarted;

unsigned long time;
unsigned long last100msUpdate;
unsigned long last1000msUpdate;

unsigned long milliStart;
unsigned long milliStop;
unsigned long milliChrono;

#if USE_RTC_CHRONO

unsigned long rtcEpochStart;
unsigned long rtcEpochStop;
unsigned long rtcNow;

int rtcStartOffset;
int rtcStopOffset;
unsigned long rtcChrono;

bool rtcStartSecond;
bool rtcStopSecond;

#endif

volatile unsigned long wheelteeth;
unsigned long wheelteeth1k;
unsigned long wheelStop;
unsigned long teethPerKM = 24705;
float avgSpeed = 0;

//
//FOR SCREEN
//
#define PIN_RESET 12 //Pin 4 on LCD
#define PIN_SCE   11 //Pin 3 on LCD
#define PIN_DC    10 //Pin 5 on LCD
#define PIN_SDIN  9 //Pin 6 on LCD
#define PIN_SCLK  8 //Pin 7 on LCD
#define PIN_BACKLIGHT 6
#define PIN_BUTTON 2
#define PIN_LED  5
#define PIN_BUTTON_MODE 4

#define DEBOUNCE 10
#define LED_LEVEL 200

#define MILLIS_IN_HOUR 3600000UL
#define MILLIS_IN_MINUTE 60000UL
#define MILLIS_IN_SECOND 1000

//The DC pin tells the LCD if we are sending a command or data
#define LCD_COMMAND 0 
#define LCD_DATA  1

//You may find a different size screen, but this one is 84 by 48 pixels
#define LCD_X     84
#define LCD_Y     48

//This table contains the hex values that represent pixels
//for a font that is 5 pixels wide and 8 pixels high
static const byte ASCII[][5] = {
  {0x00, 0x00, 0x00, 0x00, 0x00} // 20  
  ,{0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
  ,{0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
  ,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
  ,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
  ,{0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
  ,{0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
  ,{0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
  ,{0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
  ,{0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
  ,{0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
  ,{0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
  ,{0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
  ,{0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
  ,{0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
  ,{0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
  ,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
  ,{0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
  ,{0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
  ,{0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
  ,{0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
  ,{0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
  ,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
  ,{0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
  ,{0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
  ,{0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
  ,{0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
  ,{0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
  ,{0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
  ,{0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
  ,{0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
  ,{0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
  ,{0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
  ,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
  ,{0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
  ,{0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
  ,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
  ,{0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
  ,{0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
  ,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
  ,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
  ,{0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
  ,{0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
  ,{0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
  ,{0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
  ,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
  ,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
  ,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
  ,{0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
  ,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
  ,{0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
  ,{0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
  ,{0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
  ,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
  ,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
  ,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
  ,{0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
  ,{0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
  ,{0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
  ,{0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
  ,{0x02, 0x04, 0x08, 0x10, 0x20} // 5c \
  ,{0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
  ,{0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
  ,{0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
  ,{0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
  ,{0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
  ,{0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
  ,{0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
  ,{0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
  ,{0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
  ,{0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
  ,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
  ,{0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
  ,{0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
  ,{0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j 
  ,{0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
  ,{0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
  ,{0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
  ,{0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
  ,{0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
  ,{0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
  ,{0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
  ,{0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
  ,{0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
  ,{0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
  ,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
  ,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
  ,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
  ,{0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
  ,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
  ,{0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
  ,{0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
  ,{0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
  ,{0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
  ,{0x10, 0x08, 0x08, 0x10, 0x08} // 7e ~
  ,{0x78, 0x46, 0x41, 0x46, 0x78} // 7f DEL
};
//
//
//






void setup() {
  LCDInit();
  LCDClear();
  
  pinMode(PIN_BACKLIGHT, OUTPUT);
  analogWrite(PIN_BACKLIGHT, 70);
  pinMode(PIN_LED, OUTPUT);
  analogWrite(PIN_LED, 0);
  
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BUTTON_MODE, INPUT_PULLUP);
  
  buttonState = digitalRead(PIN_BUTTON);
  previousButtonState = buttonState;
  dirtyState = buttonState;
  
  chronoStarted = false;
  milliStart = 0;
  milliStop = 0;
  milliChrono = 0;
  
#if USE_RTC_CHRONO
  rtcNow = 0;
  rtcEpochStart = 0;
  rtcEpochStop = 0;
  rtcStartSecond = false;
  rtcStopSecond = false;
  rtcStartOffset = 0;
  rtcStopOffset = 0;
#endif  
  
#if USE_SERIAL
  Serial.begin(9600);  
#endif

  wheelteeth = 0;

}

void loop() {
  if(millis() != time)
  {
    time = millis();
    //
    // EVERY MILLI
    //
    //Serial.println(analogRead(A5));
    
    //SOFTWARE DEBOUNCE FOR BUTTON
    dirtyState = digitalRead(PIN_BUTTON);
    if(dirtyState == buttonState && buttonDebounceCounter > 0) { 
      buttonDebounceCounter--;
    }
    if(dirtyState != buttonState) {
      buttonDebounceCounter++;
    }
    if(buttonDebounceCounter >= DEBOUNCE)
    {
      buttonDebounceCounter = 0;
      buttonState = dirtyState;
    }
    
    //
    if(buttonState != previousButtonState) {
      previousButtonState = buttonState;
      if(!buttonState){
        analogWrite(PIN_LED, LED_LEVEL);
#if USE_SERIAL
        Serial.print("MODE:");
        Serial.println(digitalRead(PIN_BUTTON_MODE));
#endif
        //gotoXY(0,4);
        //LCDString("BTN");
        
      } else {
        analogWrite(PIN_LED, LOW);
        //gotoXY(0,4);
        //LCDString("   ");
        
        //setTime(1, 6, 0, 26, 5, 2015);//setting AVR time
        //RTC.set(now());
      }
      
      if(!chronoStarted && buttonState == HIGH) { //chrono has not been started before, but button was pressed and now is released - start chrono
      
#if USE_RTC_CHRONO
        rtcEpochStart = RTC.get();
        rtcEpochStop = rtcEpochStart;
        rtcStartSecond = true;
#endif

        chronoStarted = true;
        milliStart = time;        
        LCDClear();
        wheelteeth = 0;
        attachInterrupt(INT1, wheelsignal, FALLING);
      } else if(chronoStarted && buttonState == HIGH) { //chrono was started, but now the button has been pressed
      
#if USE_RTC_CHRONO
        rtcStopSecond = true;
        rtcEpochStop = RTC.get();
#endif
        detachInterrupt(INT1);
        chronoStarted = false;
        milliStop = time;
        milliChrono = (time - milliStart);
        gotoXY(0,4);
        LCDChrono(milliChrono);
      }
    }
    
#if USE_RTC_CHRONO
    if(rtcStartSecond || rtcStopSecond) {
      rtcNow = RTC.get();
      //during the first second of chrono - try to find when the RTC second changes
      if(rtcStartSecond) {
        if(rtcEpochStart < rtcNow) { 
          //if starting RTC second has passed, save the offset in millis. 
          //this can then be added to rtc difference between start and stop for increased accuracy
          rtcStartOffset = (-1000 + (time - milliStart));
          rtcStartSecond = false;
        }
      }
      
      if(rtcStopSecond) {
  
        if(rtcEpochStop < rtcNow) {
          //if stopping RTC second has passed, save the offset in millis. 
          //this can then be added to rtc difference between start and stop for increased accuracy
          rtcStopOffset = 1000 - (time - milliStop);
          rtcStopSecond = false;
          
          rtcChrono = ((rtcEpochStop - rtcEpochStart) * 1000) + (rtcStopOffset + rtcStartOffset);

          gotoXY(0,5);
          LCDChrono(rtcChrono);
        }
      }
    
    }
#endif

    //
    // Every 100ms
    //
    if(time >= last100msUpdate+99) {
      last100msUpdate = time;
      
      gotoXY(0,0);
      stringRTC_Time();     
      
      int t = RTC.temperature();
      float celsius = t / 4.0;
      float fahrenheit = celsius * 9.0 / 5.0 + 32.0;
      gotoXY(0,2);
      LCDString("t(C) ");
      LCDString(dtostrf(celsius, 4, 1, buffer));
      
      

      if(chronoStarted) {
        
        gotoXY(0,4);
        milliChrono = (unsigned long)(time - milliStart);
        LCDChrono(milliChrono);
        
        avgSpeed = (float)wheelteeth / teethPerKM * 3600000 / milliChrono;
        Serial.print(wheelteeth);
        Serial.print("\t");
        Serial.print(milliChrono);
        Serial.print("\t");
        Serial.print(teethPerKM);
        Serial.print("\t");
        Serial.println(avgSpeed);

        gotoXY(0,3);
        LCDString(itoa(wheelteeth,buffer,10));
        gotoXY(42,3);
        LCDString(dtostrf(avgSpeed, 4, 1, buffer));
      }

    } //do only once per 100 milliseconds
    if(time >= last1000msUpdate+1000) {
      gotoXY(0,5);
      avgSpeed = (float)(wheelteeth - wheelteeth1k) / teethPerKM * 3600000 / (time - last1000msUpdate);
      LCDString(itoa((wheelteeth - wheelteeth1k),buffer,10));
      gotoXY(42,5);
      LCDString(dtostrf(avgSpeed, 4, 1, buffer));
      wheelteeth1k = wheelteeth;
      last1000msUpdate = time;
    }
    
  } //do only once per millisecond
}

void wheelsignal() {
  wheelteeth = wheelteeth+1;
}

void stringRTC_Time()
{
  RTC.read(tm);//TimeElements variable
  LCD2digit(tm.Hour);
  LCDString(":");
  LCD2digit(tm.Minute);
  LCDString(":");
  LCD2digit(tm.Second);
  gotoXY(0,1);
  LCDString(itoa(tmYearToCalendar(tm.Year),buffer,10));
  LCDString(".");
  LCD2digit(tm.Month);
  LCDString(".");
  LCD2digit(tm.Day);

}

//////
void gotoXY(int x, int y) {
  LCDWrite(0, 0x80 | x);  // Column.
  LCDWrite(0, 0x40 | y);  // Row.  ?
}

//This takes a large array of bits and sends them to the LCD
void LCDBitmap(char my_array[]){
  for (int index = 0 ; index < (LCD_X * LCD_Y / 8) ; index++)
    LCDWrite(LCD_DATA, my_array[index]);
}

//This function takes in a character, looks it up in the font table/array
//And writes it to the screen
//Each character is 8 bits tall and 5 bits wide. We pad one blank column of
//pixels on each side of the character for readability.
void LCDCharacter(char character) {
  LCDWrite(LCD_DATA, 0x00); //Blank vertical line padding

  for (int index = 0 ; index < 5 ; index++)
    LCDWrite(LCD_DATA, ASCII[character - 0x20][index]);
    //0x20 is the ASCII character for Space (' '). The font table starts with this character

  LCDWrite(LCD_DATA, 0x00); //Blank vertical line padding
}

//Given a string of characters, one by one is passed to the LCD
void LCDString(char *characters) {
  while (*characters) {
    LCDCharacter(*characters++);
  }
}


void LCDChrono(unsigned long inMilli) {
  unsigned long millisLeft = 0;
  unsigned long chronoHours = 0;
  unsigned long chronoMinutes = 0;
  unsigned long chronoSeconds = 0;
  unsigned long chronoSecPart = 0;
  chronoHours = floor(inMilli / MILLIS_IN_HOUR);
  millisLeft = inMilli - (chronoHours * MILLIS_IN_HOUR);
  chronoMinutes = floor(millisLeft / MILLIS_IN_MINUTE);
  millisLeft = millisLeft - (chronoMinutes * MILLIS_IN_MINUTE);
  chronoSeconds = floor(millisLeft / MILLIS_IN_SECOND);
  millisLeft = millisLeft - (chronoSeconds * MILLIS_IN_SECOND);
  chronoSecPart = floor(millisLeft / 100);
  if(chronoHours) {
    LCDString(itoa(chronoHours,buffer,10));
    LCDString(":");
  }
  //if(chronoMinutes) {
    LCD2digit(chronoMinutes);
    LCDString(":");
  //}
  LCD2digit(chronoSeconds);
  LCDString(".");
  LCDString(itoa(chronoSecPart,buffer,10));
}

void LCD2digit(int inDigits) {
  if(inDigits < 10) {
    LCDString("0");
  }  
  LCDString(itoa(inDigits,buffer,10));
}

//Clears the LCD by writing zeros to the entire screen
void LCDClear(void) {
  for (int index = 0 ; index < (LCD_X * LCD_Y / 8) ; index++)
    LCDWrite(LCD_DATA, 0x00);
    
  gotoXY(0, 0); //After we clear the display, return to the home position
}

//This sends the magical commands to the PCD8544
void LCDInit(void) {

  //Configure control pins
  pinMode(PIN_SCE, OUTPUT);
  pinMode(PIN_RESET, OUTPUT);
  pinMode(PIN_DC, OUTPUT);
  pinMode(PIN_SDIN, OUTPUT);
  pinMode(PIN_SCLK, OUTPUT);

  //Reset the LCD to a known state
  digitalWrite(PIN_RESET, LOW);
  digitalWrite(PIN_RESET, HIGH);

  LCDWrite(LCD_COMMAND, 0x21); //Tell LCD that extended commands follow
  LCDWrite(LCD_COMMAND, 0xB0); //Set LCD Vop (Contrast): Try 0xB1(good @ 3.3V) or 0xBF if your display is too dark
  LCDWrite(LCD_COMMAND, 0x04); //Set Temp coefficent
  LCDWrite(LCD_COMMAND, 0x14); //LCD bias mode 1:48: Try 0x13 or 0x14

  LCDWrite(LCD_COMMAND, 0x20); //We must send 0x20 before modifying the display control mode
  LCDWrite(LCD_COMMAND, 0x0C); //Set display control, normal mode. 0x0D for inverse
}

//There are two memory banks in the LCD, data/RAM and commands. This 
//function sets the DC pin high or low depending, and then sends
//the data byte
void LCDWrite(byte data_or_command, byte data) {
  digitalWrite(PIN_DC, data_or_command); //Tell the LCD that we are writing either to data or a command

  //Send the data
  digitalWrite(PIN_SCE, LOW);
  shiftOut(PIN_SDIN, PIN_SCLK, MSBFIRST, data);
  digitalWrite(PIN_SCE, HIGH);
}
///


