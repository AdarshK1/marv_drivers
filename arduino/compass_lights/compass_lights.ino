/******************************************************************************
HMC6343_basics.ino
Simple example for using the HMC6343 3-axis compass library.
Jordan McConnell @ SparkFun Electronics
17 July 2014
https://github.com/sparkfun/SparkFun_HMC6343_Arduino_Library

This example declares an SFE_HMC6343 object called compass. The object/sensor
is initialized and if the initialization fails, the sensor could not
be found and read from successfully.

Each time through the loop, heading values (heading, pitch, and roll) and
accelerometer values (accelX, accelY, accelZ) are read from the sensor. They
are then printed to the Serial Monitor at 115200 baud. The raw sensor values 
are printed as well as scaled values in readable units (degrees and g forces).

The HMC6343 breakout board needs to be powered with 3.3V and uses I2C to talk
to the microcontroller. If you're using a 5V microcontroller board, such as 
the standard Arduino UNO, you'll need a Logic Level Converter for the I2C lines,
such as SparkFun's BOB-12009.

Developed/Tested with:
Arduino Uno
Arduino IDE 1.0.5 & 1.5.2

Requires:
SFE_HMC6343_Library

This code is beerware.
Distributed as-is; no warranty is given. 
******************************************************************************/

// Libraries for I2C and the HMC6343 sensor
#include <Wire.h>
#include "SFE_HMC6343.h"

SFE_HMC6343 compass; // Declare the sensor object


#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define LED_PIN    6

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 9

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)


int cell1 = A0;
int cell2 = A1;
int cell3 = A2;
int cell4 = A3;

double R2 = 1.32;
double R3 = 1.88;
double R4 = 2.82;


void setup()
{
  // Start serial communication at 115200 baud
  Serial.begin(115200);
  
  // Give the HMC6343 a half second to wake up
  delay(500); 
  
  // Initialize the HMC6343 and verify its physical presence
  if (!compass.init())
  {
    Serial.println("Sensor Initialization Failed\n\r"); // Report failure, is the sensor wiring correct?
  }

    // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.

//  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
//  strip.show();            // Turn OFF all pixels ASAP
//  strip.setBrightness(250); // Set BRIGHTNESS to about 1/5 (max = 255)
}

void loop()
{
  // Read, calculate, and print the heading, pitch, and roll from the sensor
  compass.readHeading();
//  printHeadingData();
  // Read, calculate, and print the acceleration on the x, y, and z axis of the sensor
  compass.readAccel();

  printAllData();
//  printAccelData();
  
  // Wait for two seconds
  delay(200); // Minimum delay of 200ms (HMC6343 has 5Hz sensor reads/calculations)

  double cell_1_ADC = analogRead(cell1) * 5.0/1024;
  double cell_2_ADC = analogRead(cell2) * 5.0/1024;
  double cell_3_ADC = analogRead(cell3) * 5.0/1024;
  double cell_4_ADC = analogRead(cell4) * 5.0/1024;

  double cell_1 = cell_1_ADC;
  double cell_2 = cell_2_ADC / (470.0/(470+620));
  double cell_3 = cell_3_ADC / (330.0/(330+620));
  double cell_4 = cell_4_ADC / (220.0/(220+620));

  Serial.print("Cell 1 Voltage: "); Serial.print(cell_1); Serial.print(" Cell 2 Voltage: "); Serial.println(cell_2);
  Serial.print("Cell 3 Voltage: "); Serial.print(cell_3); Serial.print(" Cell 4 Voltage: "); Serial.println(cell_4);
  Serial.println("......................................................");

    // Fill along the length of the strip in various colors...
//  colorWipe(strip.Color(255,   0,   0), 10); // Red
//  colorWipe(strip.Color(  0, 255,   0), 10); // Green
//  colorWipe(strip.Color(  0,   0, 255), 10); // Blue
//
//  // Do a theater marquee effect in various colors...
//  theaterChase(strip.Color(127, 127, 127), 50); // White, half brightness
//  theaterChase(strip.Color(127,   0,   0), 50); // Red, half brightness
//  theaterChase(strip.Color(  0,   0, 127), 50); // Blue, half brightness
//
//  rainbow(10);             // Flowing rainbow cycle along the whole strip
//  theaterChaseRainbow(50); // Rainbow-enhanced theaterChase variant
}

void  printAllData()
{
  Serial.print("YPR,XYZ: ");
  Serial.print((float) compass.heading/10.0); Serial.print(",");  
  Serial.print((float) compass.pitch/10.0); Serial.print(",");  
  Serial.print((float) compass.roll/10.0); Serial.print(",");
  Serial.print((float) compass.accelX/1024.0);Serial.print(","); 
  Serial.print((float) compass.accelY/1024.0);Serial.print(",");
  Serial.print((float) compass.accelZ/1024.0);Serial.println(",");


}

// Print both the raw values of the compass heading, pitch, and roll
// as well as calculate and print the compass values in degrees
// Sample Output:
// Heading Data (Raw value, in degrees):
// Heading: 3249  324.90°
// Pitch:   28    2.80°
// Roll:    24    2.40°
void printHeadingData()
{
  Serial.println("Heading Data (in degrees):");
//  Serial.print("Heading: ");
  Serial.print((float) compass.heading/10.0);Serial.print(" ");  //Serial.println(); // Print heading in degrees
//  Serial.print("Pitch: ");
  Serial.print((float) compass.pitch/10.0);Serial.print(" ");  //Serial.println();
//  Serial.print("Roll: ");
  Serial.print((float) compass.roll/10.0);Serial.print(" "); //Serial.println();
  Serial.println();
}

// Print both the raw values of the compass acceleration measured on each axis
// as well as calculate and print the accelerations in g forces
// Sample Output:
// Accelerometer Data (Raw value, in g forces):
// X: -52    -0.05g
// Y: -44    -0.04g
// Z: -1047  -1.02g
void printAccelData()
{
  Serial.println("Accelerometer Data (Raw value, in g forces):");
  Serial.print("X: ");
//  Serial.print(compass.accelX); Serial.print("  "); // Print raw acceleration measurement on x axis
  Serial.print((float) compass.accelX/1024.0);Serial.println("g"); // Print x axis acceleration measurement in g forces
  Serial.print("Y: ");
//  Serial.print(compass.accelY); Serial.print("  ");
  Serial.print((float) compass.accelY/1024.0);Serial.println("g");
  Serial.print("Z: ");
//  Serial.print(compass.accelZ); Serial.print("  ");
  Serial.print((float) compass.accelZ/1024.0);Serial.println("g");
  Serial.println();
}



// Some functions of our own for creating animated effects -----------------

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

// Theater-marquee-style chasing lights. Pass in a color (32-bit value,
// a la strip.Color(r,g,b) as mentioned above), and a delay time (in ms)
// between frames.
void theaterChase(uint32_t color, int wait) {
  for(int a=0; a<10; a++) {  // Repeat 10 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show(); // Update strip with new contents
      delay(wait);  // Pause for a moment
    }
  }
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this outer loop:
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
      // Offset pixel hue by an amount to make one full revolution of the
      // color wheel (range of 65536) along the length of the strip
      // (strip.numPixels() steps):
      int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the single-argument hue variant. The result
      // is passed through strip.gamma32() to provide 'truer' colors
      // before assigning to each pixel:
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
    }
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}

// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  for(int a=0; a<30; a++) {  // Repeat 30 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the strip (strip.numPixels() steps):
        int      hue   = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show();                // Update strip with new contents
      delay(wait);                 // Pause for a moment
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
    }
  }
}
