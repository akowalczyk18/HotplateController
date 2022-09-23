#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

//Set up defines for input and output pins

#define HEATER_OUT 22
#define UP_BTN 18
#define SEL_BTN 19
#define DWN_BTN 20
#define TEMP_IN A2



// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define VCC 3
#define R 100000
#define RT0 100000
#define Beta 3950





//Function Defines
float adcToCelcius(float);
void error();

//ISR defines
void upBtnISR();
void selBtnISR();
void dwnBtnISR();



//define floats used for temperature handling
float RT, VR, ln, TX, T0, VRT;
float tempSet = 150;

//define flags to be set for buttons
int upFlag = 0;
int selFlag = 0;
int dwnFlag = 0;
int debounceCount = 0;
int heaterState = LOW; 

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);



void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
Wire.setSDA(0);
Wire.setSCL(1);

 if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

T0 = 25 + 273.15; // set up T0 as variable
//Clear display from Adafruit logo (loaded in by default)
  
display.clearDisplay();

//Set up pin modes
pinMode(TEMP_IN, INPUT);
pinMode(HEATER_OUT, OUTPUT);
pinMode(UP_BTN, INPUT_PULLUP);
pinMode(SEL_BTN, INPUT_PULLUP);
pinMode(DWN_BTN, INPUT_PULLUP);

attachInterrupt(UP_BTN, upBtnISR, FALLING );
attachInterrupt(SEL_BTN, selBtnISR, FALLING );
attachInterrupt(DWN_BTN, dwnBtnISR, FALLING );

}

void loop() {
  int tempDisplayed = 0;

  VRT = analogRead(TEMP_IN);
  if (VRT > 850)
  {
    error();
  }
  else
  {
    
  }

  if(selFlag == 1)
  {
    if(heaterState == LOW)
    {
      heaterState = HIGH;
    }
    else
    {
      heaterState = LOW;
    }
    selFlag = 0;
    delay(100);
    digitalWrite(HEATER_OUT, heaterState);
  
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("TEMPERATURE"));
  display.setTextSize(2);
  VRT = analogRead(A2);
  Serial.println(VRT);
  tempDisplayed = adcToCelcius(VRT);
  display.setCursor(0, 10);
  display.println(tempDisplayed);
  display.setTextSize(1);
  display.setCursor(0, 25);
  if (heaterState == LOW)
  {
    display.println(F("HEATER OFF"));
  }
  else
  {
    display.println(F("HEAtER ON"));
  }
  delay(100);
  display.display();
  }


float adcToCelcius(float tempIn)
{
  float tempOut;
  tempIn = (3.3 / 1023.00) * tempIn;
  VR = VCC - tempIn;
  RT = tempIn / (VR / R);
  ln = log(RT / RT0);
  tempOut = (1 / ((ln / Beta) + (1 / T0)));
  tempOut = tempOut - 273.15;
  return tempOut;
}

void upBtnISR() {
  upFlag = 1;
}

void selBtnISR() {
  selFlag = 1;
}

void dwnBtnISR() {
  dwnFlag = 1;
}

void error() {
  digitalWrite(HEATER_OUT, LOW);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("ERROR"));
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.println(F("THERMISTOR UNPLUGGED"));
  display.display();
  while(1)
  {
    delay(100);
  }
}
