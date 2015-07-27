/*
BICYCLE Indicator strip using ws2812 chipset LEDS.

2 strips of 2o LED on each fork

Author: C. Stewart, cstewart000@gmail.com, hackingismakingisengineering.wordpress.com

15JUL   - Added analog read to check multiple switch values.
        - Refactored the code
22JUL   - Incuded rear strips (indication only)

TODO:
        - Include rear indicator strips
        - Accelerometer reads for breaking condition

 */


// Import the Adafuit library
#include <Adafruit_NeoPixel.h>
#include <avr/power.h>

// CIRCUIT DESCRIPTION
/*
  Two strips of WS2812 LEDs installed to the front forks of a bicycle.
  Each of the strips connected to 3pin JST connectors. The two 3 pin connectors fork to a 4 pin JST connector.
  2 signal wires for each led strip and shared power and ground lines.

  The cables should be cable tied to the break cable housing to relieve strain.

  The strips connect back to the Arduino Nano board per the pins defined below.

  A perf board with 3 microswitches connected with a series of voltage dividers is inserted into the hood of the
  right hand (if you prefer) brake handle. the cables can be run under the handlebar tape for neatness.



*/

// Set DEBUG_SERIAL_OUTPUT to true to enable serial out for debugging purposes.
# define DEBUG_SERIAL_OUTPUT true

// pin definitions
# define READ_PIN A0
//# define LIGHT_POWER_PIN 3
# define CONTROL_GRD_PIN 6
# define CONTROL_PWR_PIN 9
# define LEFT_INDICATOR_STRIP_PIN 2
# define RIGHT_INDICATOR_STRIP_PIN 4

# define LEFT_REAR_STRIP_PIN 11
# define RIGHT_REAR_STRIP_PIN 12

// CONTOL SWITCH THRESHOLDS
/*
  Contol pad is a series of micro switches that move the read pin to different voltage to register a certain button press.
  The voltage value read on the ADC (analog read) then determines which button was pressed. This was done to reduce the cabling
  from the micro control to the control pad from 5+ wires to 3 to allow easier incoration into the handlebars.

  Threshold values defined below (require tuning depending on the resistor values used in the controls)

*/
# define NO_TOUCH_THRES_LOW 1000
# define CANCEL_THRES_HIGH 20

# define LEFT_THRES_LOW 500
# define LEFT_THRES_HIGH 799

# define RIGHT_THRES_LOW 799
# define RIGHT_THRES_HIGH 999

// DEFINITION OF THE STRIP LENGTHS AND DIVISIONS
#define RIGHT_INDICATOR_STRIP_PIXELS 20
#define LEFT_INDICATOR_STRIP_PIXELS 20
#define RIGHT_INDICATOR_STRIP_DIVIDE 10
#define LEFT_INDICATOR_STRIP_DIVIDE 10

#define LEFT_REAR_STRIP_PIXELS 30
#define RIGHT_REAR_STRIP_PIXELS 30
#define LEFT_REAR_STRIP_DIVIDE 15
#define RIGHT_REAR_STRIP_DIVIDE 15


// SET DELAYS
#define IND_DELAY 500 // time between indicator LED toggle state.
#define DEBOUNCE 100 //Debounce (legacy)

// LED Colour value definitions.
# define HEADLIGHT 255 // full colour 
# define IND_COL_R 255
# define IND_COL_G 125
# define IND_COL_B 0

# define BRAKE_COL_R 76
# define BRAKE_COL_G 125
# define BRAKE_COL_B 0

//Initialise the Neopixel library objects
Adafruit_NeoPixel pixelsLeft = Adafruit_NeoPixel(LEFT_INDICATOR_STRIP_PIXELS, LEFT_INDICATOR_STRIP_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixelsRight = Adafruit_NeoPixel(RIGHT_INDICATOR_STRIP_PIXELS, RIGHT_INDICATOR_STRIP_PIN, NEO_GRB + NEO_KHZ800);

Adafruit_NeoPixel pixelsRearLeft = Adafruit_NeoPixel(LEFT_REAR_STRIP_PIXELS, LEFT_REAR_STRIP_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixelsRearRight = Adafruit_NeoPixel(RIGHT_REAR_STRIP_PIXELS, RIGHT_REAR_STRIP_PIN, NEO_GRB + NEO_KHZ800);

// DEFINE GLOBAL VARIABLES
bool leftState_LED = false; // State of LEDS (on/off)
bool rightState_LED = false;

// State of the Indicator. Node: LEDS may be on or off under this state depending on the timing
bool leftIndicatorState = false;
bool rightIndicatorState = false;

// timing variables - for performance and indicator flasing
long millisLast = 0;
long millisNow = 0;

char mode = ' ';
int reading;

void setup()
{

  // Initialise the NEO Pixel hardware Objects.
  pixelsLeft.begin();
  pixelsRight.begin();
    pixelsRearLeft.begin();
  pixelsRearRight.begin();

  if (DEBUG_SERIAL_OUTPUT) {
    Serial.begin(9600);
  }

  // define the hardware pins
  pinMode(READ_PIN, INPUT);

  pinMode(CONTROL_GRD_PIN, OUTPUT);
  pinMode(CONTROL_PWR_PIN, OUTPUT);
  //pinMode(LIGHT_POWER_PIN, OUTPUT);

  digitalWrite(CONTROL_GRD_PIN, LOW);
  //digitalWrite(LIGHT_POWER_PIN, HIGH);
  digitalWrite(CONTROL_PWR_PIN, HIGH);

  // Set pixels to blank to start
  for (int i = 0; i < RIGHT_INDICATOR_STRIP_PIXELS; i++) {
    pixelsRight.setPixelColor(i, pixelsRight.Color(0, 0, 0));
  }
  for (int i = 0; i < LEFT_INDICATOR_STRIP_PIXELS; i++) {
    pixelsLeft.setPixelColor(i, pixelsLeft.Color(0, 0, 0));
  }

  for (int i = 0; i < LEFT_REAR_STRIP_PIXELS; i++) {
    pixelsRearLeft.setPixelColor(i, pixelsRearLeft.Color(0, 0, 0));
  }

  for (int i = 0; i < RIGHT_REAR_STRIP_PIXELS; i++) {
    pixelsRearRight.setPixelColor(i, pixelsRearRight.Color(0, 0, 0));
  }

  // set left headlights
  for (int i = RIGHT_INDICATOR_STRIP_DIVIDE; i < RIGHT_INDICATOR_STRIP_PIXELS; i++) {
    pixelsRight.setPixelColor(i, pixelsRight.Color(HEADLIGHT, HEADLIGHT, HEADLIGHT));
  }
  //Set right headlight
  for (int i = LEFT_INDICATOR_STRIP_DIVIDE; i < LEFT_INDICATOR_STRIP_PIXELS; i++) {
    pixelsLeft.setPixelColor(i, pixelsLeft.Color(HEADLIGHT, HEADLIGHT, HEADLIGHT));
  }

  // set righ rearlights
  for (int i = RIGHT_REAR_STRIP_DIVIDE; i < RIGHT_REAR_STRIP_PIXELS; i++) {
    pixelsRearRight.setPixelColor(i, pixelsRearRight.Color(BRAKE_COL_R, BRAKE_COL_G, BRAKE_COL_B));
  }
  //Set left rearlightss
  for (int i = LEFT_REAR_STRIP_DIVIDE; i < LEFT_REAR_STRIP_PIXELS; i++) {
    pixelsRearLeft.setPixelColor(i, pixelsRearLeft.Color(BRAKE_COL_R, BRAKE_COL_G, BRAKE_COL_B));
  }

  // Display the pixels
  pixelsLeft.show();
  pixelsRight.show();
    pixelsRearLeft.show();
  pixelsRearRight.show();
}

void loop() {

  mode = checkControls();
  if (DEBUG_SERIAL_OUTPUT) {
    Serial.println(reading);
    Serial.println(mode);
  }
  switch (mode) {
    case 'n':
      break;
    case 'l':
      leftIndicatorState = true;

      break;
    case 'r':
      rightIndicatorState = true;
      break;
    case 'c':
      rightIndicatorState = false;
      leftIndicatorState = false;
      break;
    default:
      if (DEBUG_SERIAL_OUTPUT) {
        Serial.print("Some kind of error in the case statement \n");
      }
  }


  if (DEBUG_SERIAL_OUTPUT) {
    printStateToSerial();
  }

  writeToLedStrip();
} // end main loop

void writeToLedStrip() {

  millisNow = millis();

  if (millisLast < millisNow + IND_DELAY) {

    // set RIGHT indicator
    if (rightIndicatorState == true) {
      if (rightState_LED == true) {
        for (int i = 0; i < RIGHT_INDICATOR_STRIP_DIVIDE; i++) {
          pixelsRight.setPixelColor(i, pixelsRight.Color(IND_COL_R, IND_COL_G, IND_COL_B));

        }
        for (int i = 0; i < RIGHT_REAR_STRIP_DIVIDE; i++) {
          pixelsRearRight.setPixelColor(i, pixelsRearRight.Color(IND_COL_R, IND_COL_G, IND_COL_B));

        }


      } else {
        for (int i = 0; i < RIGHT_INDICATOR_STRIP_DIVIDE; i++) {
          pixelsRight.setPixelColor(i, pixelsRight.Color(0, 0, 0));
        }
        for (int i = 0; i < RIGHT_REAR_STRIP_DIVIDE; i++) {
          pixelsRearRight.setPixelColor(i, pixelsRearRight.Color(0, 0, 0));

        }
      }
      rightState_LED = !rightState_LED;

    } else {
      for (int i = 0; i < RIGHT_INDICATOR_STRIP_DIVIDE; i++) {

        // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
        pixelsRight.setPixelColor(i, pixelsRight.Color(0, 0, 0));
      }
      for (int i = 0; i < RIGHT_REAR_STRIP_DIVIDE; i++) {
        pixelsRearRight.setPixelColor(i, pixelsRearRight.Color(0, 0, 0));

      }
    }

    //Set LEFT indicator
    if (leftIndicatorState == true) {
      if (leftState_LED == true) {
        for (int i = 0; i < LEFT_INDICATOR_STRIP_DIVIDE; i++) {
          pixelsLeft.setPixelColor(i, pixelsLeft.Color(IND_COL_R, IND_COL_G, IND_COL_B));
        }
        for (int i = 0; i < LEFT_REAR_STRIP_DIVIDE; i++) {
          pixelsRearLeft.setPixelColor(i, pixelsRearLeft.Color(IND_COL_R, IND_COL_G, IND_COL_B));

        }
      } else {
        for (int i = 0; i < LEFT_INDICATOR_STRIP_DIVIDE; i++) {
          pixelsLeft.setPixelColor(i, pixelsLeft.Color(0, 0, 0));
        }
        for (int i = 0; i < LEFT_REAR_STRIP_DIVIDE; i++) {
          pixelsRearLeft.setPixelColor(i, pixelsRearLeft.Color(0, 0, 0));

        }
      }
      leftState_LED = !leftState_LED;
    }
    else {
      for (int i = 0; i < LEFT_INDICATOR_STRIP_DIVIDE; i++) {
        pixelsLeft.setPixelColor(i, pixelsLeft.Color(0, 0, 0));
      }
      for (int i = 0; i < LEFT_REAR_STRIP_DIVIDE; i++) {
        pixelsRearLeft.setPixelColor(i, pixelsRearLeft.Color(0, 0, 0));

      }
    }
    millisLast = millisNow;

  } // if millis condition passed

  pixelsLeft.show();
  pixelsRight.show();
      pixelsRearLeft.show();
  pixelsRearRight.show();

} // writeToLedStrip function end



char checkControls() {
  // put your main code here, to run repeatedly:
  reading = analogRead(READ_PIN);

  if (DEBUG_SERIAL_OUTPUT)
  {
    Serial.print("Control panel voltage reading: ");
    Serial.println(reading);
  }
  if (reading < LEFT_THRES_HIGH) {
    if (reading < CANCEL_THRES_HIGH) {
      mode = 'c';
    } else {
      mode = 'l';
    }
  }
  else {
    if (reading < NO_TOUCH_THRES_LOW) {
      mode = 'r';
    }
    else {
      mode = 'n';
    }
  }
}

void printStateToSerial()
{

  // Debug print to consoled

  Serial.print("mode: ");
  Serial.println(mode);

  // left indicator state
  Serial.print("leftIndicatorState: ");
  Serial.print(leftIndicatorState);
  Serial.print("\tleftState_LED: ");
  Serial.println(leftState_LED);

  // right indicator state
  Serial.print("rightIndicatorState: ");
  Serial.print(rightIndicatorState);
  Serial.print("\trightState_LED: ");
  Serial.println(rightState_LED);

}

/*
// Legacy code (for reference only)
void checkSwitch1() {

  switchState1 = !digitalRead(LEFT_BUTTON_PIN);
  if (oldSwitchState1 == false & switchState1 == true) {
    delay(DEBOUNCE);
    Serial.println("leftIndicatorState toggled! ");

    leftIndicatorState = !leftIndicatorState;
    //oldSwitchState1 = switchState1;
  }
}
*/
