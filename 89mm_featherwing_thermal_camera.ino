/***************************************************************************
  This sketch makes a 24x24 interpolated pixel image of AMG88xx GridEYE 8x8 IR camera.

  Designed specifically to work with:
   * Adafruit AMG8833 Featherwing.
     https://www.adafruit.com/product/3622
     AMG8833
       * https://na.industrial.panasonic.com/products/sensors/sensors-automotive-industrial-applications/lineup/grid-eyer-infrared-array-sensor/series/70496/model/72453
       * https://b2b-api.panasonic.eu/file_stream/pids/fileversion/1300
       * https://cdn-learn.adafruit.com/assets/assets/000/043/261/original/Grid-EYE_SPECIFICATIONS%28Reference%29.pdf
       * https://cdn.sparkfun.com/assets/4/1/c/0/1/Grid-EYE_Datasheet.pdf
   * 3.5" TFT Featherwing:
      https://www.adafruit.com/product/3651

  "Panasonic Infrared Array Sensor Grid-EYE (AMG88)" datasheet "Pixel array and viewing field":
  NOTE: Inverted to place array index 0 first
   Up↑
  ┌───┐
  │ ~ │ <- Label
  │ █ │ <- Sensor
  └───┘
  [
  00, 01, 02, 03, 04, 05, 06, 07,
  08, 09, 10, 11, 12, 13, 14, 15,
  16, 17, 18, 19, 20, 21, 22, 23,
  24, 25, 26. 27, 28, 29, 30, 31,
  32, 33, 34, 35, 36, 37, 38, 39,
  40, 41, 42, 43, 44, 45, 46, 47,
  48, 49, 50, 51, 52, 53, 54, 55,
  56, 57, 58, 59, 60, 61, 62, 63,
  ]

  AMG88xx FeatherWing docked to 2.8" TFT FeatherWing:
  ┌─────────────────────────┐
  │┌─────────┐      ┌──────┐│
  ││:       :│      │SDCard││
  ││:  Up↑  :│      └──────┘│
  ││: ┌───┐ :│              │
  ││: │ ~ │ :│           Off│
  ││: │ █ │ :│              │
  ││: └───┘ :│            On│
  ││RST     :│              │
  ││        :│              │
  │└─────────┘           RST│
  └─────────────────────────┘
  Looking in to Sensor (opposite direction that Sensor is looking):
  [
  00, 01, 02, 03, 04, 05, 06, 07, // Top Left         Top Right
  08, 09, 10, 11, 12, 13, 14, 15,
  16, 17, 18, 19, 20, 21, 22, 23,
  24, 25, 26. 27, 28, 29, 30, 31,
  32, 33, 34, 35, 36, 37, 38, 39,
  40, 41, 42, 43, 44, 45, 46, 47,
  48, 49, 50, 51, 52, 53, 54, 55,
  56, 57, 58, 59, 60, 61, 62, 63, // Bottom Left   Bottom Right
  ]
  Looking in direction that Sensor is looking:
  [
  07, 06, 05, 04, 03, 02, 01, 00, // Top Left         Top Right
  15, 14, 13, 12, 11, 10, 09, 08,
  23, 22, 21, 20, 19, 18, 17, 16,
  31, 30, 29, 28, 27, 26, 25, 24,
  39, 38, 37, 36, 35, 34, 33, 32,
  47, 46, 45, 44, 43, 42, 41, 40,
  55, 54, 53, 52, 51, 50, 49, 48,
  63, 62, 61, 60, 59, 58, 57, 56, // Bottom Left   Bottom Right
  ]

  AMG88xx FeatherWing docked to 3.5" TFT FeatherWing:
  ┌────────────────────────────────┐
  │                        ┌──────┐│
  │                        │SDCard││
  │┌─────────────────┐     └──────┘│
  ││     ┌─────┐     │             │
  ││     │ █ ~ │     │          Off│
  ││     └─────┘     │             │
  │└─────────────────┘           On│
  │                                │
  │                             RST│
  └────────────────────────────────┘
  Looking in to Sensor (opposite direction that Sensor is looking):
  [
  56, 48, 40, 32, 24, 16, 08, 00, // Top Left         Top Right
  57, 49, 41, 33, 25, 17, 09, 01,
  58, 50, 42, 34, 26, 18, 10, 02,
  59, 51, 43, 35, 27, 19, 11, 03,
  60, 52, 44, 36, 28, 20, 12, 04,
  61, 53, 45, 37, 29, 21, 13, 05,
  62, 54, 46, 38, 30, 22, 14, 06,
  63, 55, 47, 39, 31, 23, 15, 07, // Bottom Left   Bottom Right
  ]
  Looking in direction that Sensor is looking:
  [
  00, 08, 16, 24, 32, 40, 48, 56, // Top Left         Top Right
  01, 09, 17, 25, 33, 41, 49, 57,
  02, 10, 18, 26, 34, 42, 50, 58,
  03, 11, 19, 27, 35, 43, 51, 59,
  04, 12, 20, 28, 36, 44, 52, 60,
  05, 13, 21, 29, 37, 45, 53, 61,
  06, 14, 22, 30, 38, 46, 54, 62,
  07, 15, 23, 31, 39, 47, 55, 63, // Bottom Left   Bottom Right
  ]

  If you display more than just the thermal colors then you will need to software
  rotate the array or display if you physically rotate the sensor or display.
 ***************************************************************************/

#include <Adafruit_HX8357.h>
#include <Adafruit_AMG88xx.h>

#include <Fonts/TomThumb.h> // 3x5 pixels: too hard to read

#ifdef ESP32
   #define STMPE_CS 32
   #define TFT_CS   15
   #define TFT_DC   33
   #define SD_CS    14
#endif

#define TFT_RST -1

// Comment this out to not log raw read pixel values
//#define LOG_READ_PIXEL_VALUES_RAW
// Comment this out to not log rotated read pixel values
//#define LOG_READ_PIXEL_VALUES_ROTATED
// Comment this out to not log rotation timing
//#define LOG_ROTATION_TIMING
// Comment this out to not log interpolation timing
//#define LOG_INTERPOLATION_TIMING

// Comment this out to not display temperature values
#define SHOW_TEMP_TEXT
// Comment this out to use celsius instead of fahrenheit
#define SHOW_FAHRENHEIT

// c * 9/5 + 32 == c * 1.8 + 32
#define C2F(c) ((c) * 1.8 + 32.0)
#define ROUND(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))

//
// 0,1,2,3 == 0,90,180,270 degrees respectively
//
#define ROTATION_DISPLAY 3
#define ROTATION_SENSOR 1

// Low range of the sensor (this will be blue on the screen)
// Min 0, Max 80, Suggest 20
#define MINTEMP 0

// High range of the sensor (this will be red on the screen)
// Min 0, Max 80, Suggest 28
#define MAXTEMP 80

//the colors we will be using
const uint16_t camColors[] = {0x480F,
0x400F,0x400F,0x400F,0x4010,0x3810,0x3810,0x3810,0x3810,0x3010,0x3010,
0x3010,0x2810,0x2810,0x2810,0x2810,0x2010,0x2010,0x2010,0x1810,0x1810,
0x1811,0x1811,0x1011,0x1011,0x1011,0x0811,0x0811,0x0811,0x0011,0x0011,
0x0011,0x0011,0x0011,0x0031,0x0031,0x0051,0x0072,0x0072,0x0092,0x00B2,
0x00B2,0x00D2,0x00F2,0x00F2,0x0112,0x0132,0x0152,0x0152,0x0172,0x0192,
0x0192,0x01B2,0x01D2,0x01F3,0x01F3,0x0213,0x0233,0x0253,0x0253,0x0273,
0x0293,0x02B3,0x02D3,0x02D3,0x02F3,0x0313,0x0333,0x0333,0x0353,0x0373,
0x0394,0x03B4,0x03D4,0x03D4,0x03F4,0x0414,0x0434,0x0454,0x0474,0x0474,
0x0494,0x04B4,0x04D4,0x04F4,0x0514,0x0534,0x0534,0x0554,0x0554,0x0574,
0x0574,0x0573,0x0573,0x0573,0x0572,0x0572,0x0572,0x0571,0x0591,0x0591,
0x0590,0x0590,0x058F,0x058F,0x058F,0x058E,0x05AE,0x05AE,0x05AD,0x05AD,
0x05AD,0x05AC,0x05AC,0x05AB,0x05CB,0x05CB,0x05CA,0x05CA,0x05CA,0x05C9,
0x05C9,0x05C8,0x05E8,0x05E8,0x05E7,0x05E7,0x05E6,0x05E6,0x05E6,0x05E5,
0x05E5,0x0604,0x0604,0x0604,0x0603,0x0603,0x0602,0x0602,0x0601,0x0621,
0x0621,0x0620,0x0620,0x0620,0x0620,0x0E20,0x0E20,0x0E40,0x1640,0x1640,
0x1E40,0x1E40,0x2640,0x2640,0x2E40,0x2E60,0x3660,0x3660,0x3E60,0x3E60,
0x3E60,0x4660,0x4660,0x4E60,0x4E80,0x5680,0x5680,0x5E80,0x5E80,0x6680,
0x6680,0x6E80,0x6EA0,0x76A0,0x76A0,0x7EA0,0x7EA0,0x86A0,0x86A0,0x8EA0,
0x8EC0,0x96C0,0x96C0,0x9EC0,0x9EC0,0xA6C0,0xAEC0,0xAEC0,0xB6E0,0xB6E0,
0xBEE0,0xBEE0,0xC6E0,0xC6E0,0xCEE0,0xCEE0,0xD6E0,0xD700,0xDF00,0xDEE0,
0xDEC0,0xDEA0,0xDE80,0xDE80,0xE660,0xE640,0xE620,0xE600,0xE5E0,0xE5C0,
0xE5A0,0xE580,0xE560,0xE540,0xE520,0xE500,0xE4E0,0xE4C0,0xE4A0,0xE480,
0xE460,0xEC40,0xEC20,0xEC00,0xEBE0,0xEBC0,0xEBA0,0xEB80,0xEB60,0xEB40,
0xEB20,0xEB00,0xEAE0,0xEAC0,0xEAA0,0xEA80,0xEA60,0xEA40,0xF220,0xF200,
0xF1E0,0xF1C0,0xF1A0,0xF180,0xF160,0xF140,0xF100,0xF0E0,0xF0C0,0xF0A0,
0xF080,0xF060,0xF040,0xF020,0xF800,};

Adafruit_HX8357 display = Adafruit_HX8357(TFT_CS, TFT_DC, TFT_RST);

Adafruit_AMG88xx amg;
unsigned long delayTime;

#define AMG_COLS 8
#define AMG_ROWS 8
float pixelsRaw[AMG_COLS * AMG_ROWS];
float pixelsRotated[AMG_COLS * AMG_ROWS];

#define INTERPOLATED_COLS 24
#define INTERPOLATED_ROWS 24
float pixelsInterpolated[INTERPOLATED_ROWS * INTERPOLATED_COLS];

float get_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
void interpolate_image(float *src, uint8_t src_rows, uint8_t src_cols, 
                       float *dest, uint8_t dest_rows, uint8_t dest_cols);

uint16_t displayWidth;
uint16_t displayHeight;

uint16_t displayPixelWidth;
uint16_t displayPixelHeight;

#define FONT_SCALE 1
const uint16_t SCREEN_COLOR_TEXT = HX8357_WHITE;
const uint16_t SCREEN_COLOR_BACKGROUND = HX8357_BLACK;

void setup() {
  Serial.begin(115200);
  Serial.println(F("AMG88xx thermal camera!"));

  display.begin();
  display.setRotation(ROTATION_DISPLAY);
  display.fillScreen(SCREEN_COLOR_BACKGROUND);
  
  display.setFont(&TomThumb);
  display.setTextSize(FONT_SCALE);
  display.setTextWrap(false);
  display.setTextColor(SCREEN_COLOR_TEXT);//, SCREEN_COLOR_BACKGROUND);

  displayWidth = display.width();
  displayHeight = display.height();
  Serial.println("Display Size: " + String(displayWidth) + "x" + String(displayHeight));

  displayPixelWidth = displayWidth / INTERPOLATED_COLS;
  displayPixelHeight = displayHeight / INTERPOLATED_ROWS;
  Serial.println("Display Pixel Size: " + String(displayPixelWidth) + "x" + String(displayPixelHeight));

  // default settings
  if (!amg.begin()) {
      Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
      while (1) { delay(1); }
  }
  
  Serial.println("-- Thermal Camera Test --");
  delay(100); // let sensor boot up
}

void printPixels(String name, float *pixels) {
  String debugText;
  debugText += "\n" + name + "=[\n";
  for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
    debugText += String(pixels[i-1]) + ", ";
    if( i % 8 == 0 ) debugText += "\n";
  }
  debugText += "]\n";
  Serial.println(debugText);
}

void rotatePixels(float *pIn, float* pOut, int rows, int cols, int rotation) {
  int total = rows*cols;
  for(int i=0; i < total; i++) {
    int _x = i / cols;
    int _y = i % rows;
    int x;
    int y;
    if (rotation == 0) {
      x = _x;
      y = _y;
    } else if (rotation == 1) {
      x = _y;
      y = (rows - 1) - _x;
    } else if (rotation == 2) {
      x = (rows - 1) - _x;
      y = (cols - 1) - _y;
    } else if (rotation == 3) {
      x = (cols - 1) - _y;
      y = _x;
    } else {
      return;
    }
    //Serial.println("i,x,y=" + String(i) + "," + String(x) + "," + String(y));
    
    int indexIn = x * rows + y;
    //Serial.println("indexIn=" + String(indexIn));
    float val = pIn[indexIn];

    int indexOut = i;
    //Serial.println("indexOut=" + String(indexOut));

    pOut[indexOut] = val;
  }
}

int32_t t;

void loop() {
  amg.readPixels(pixelsRaw);

  #ifdef LOG_READ_PIXEL_VALUES_RAW
  printPixels("pixelsRaw", pixelsRaw);
  #endif
  #ifdef LOG_ROTATION_TIMING
  t = millis();
  #endif
  rotatePixels(pixelsRaw, pixelsRotated, AMG_ROWS, AMG_COLS, ROTATION_SENSOR);
  #ifdef LOG_ROTATION_TIMING
  Serial.println("Rotation took " + String(millis()-t) + " ms");
  #endif
  #ifdef LOG_READ_PIXEL_VALUES_ROTATED
  printPixels("pixelsRotated", pixelsRotated);
  #endif

  #ifdef LOG_INTERPOLATION_TIMING
  t = millis();
  #endif
  interpolate_image(pixelsRotated, AMG_ROWS, AMG_COLS, pixelsInterpolated, INTERPOLATED_ROWS, INTERPOLATED_COLS);
  #ifdef LOG_INTERPOLATION_TIMING
  Serial.println("Interpolation took " + String(millis()-t) + " ms");
  #endif
  
  drawpixels(pixelsInterpolated, INTERPOLATED_ROWS, INTERPOLATED_COLS, displayPixelWidth, displayPixelHeight);
}

void drawpixels(float *pixels, uint8_t rows, uint8_t cols, uint8_t boxWidth, uint8_t boxHeight) {
  int colorTemp;
  for (int y=0; y<rows; y++) {
    for (int x=0; x<cols; x++) {
      float val = get_point(pixels, rows, cols, x, y);
      if(val >= MAXTEMP) colorTemp = MAXTEMP;
      else if(val <= MINTEMP) colorTemp = MINTEMP;
      else colorTemp = val;
      
      uint8_t colorIndex = map(colorTemp, MINTEMP, MAXTEMP, 0, 255);
      colorIndex = constrain(colorIndex, 0, 255);
      uint16_t color = camColors[colorIndex];

      int px = boxWidth * x;
      int py = boxHeight * y;
      
      display.fillRect(px, py, boxWidth, boxHeight, color);

      #ifdef SHOW_TEMP_TEXT
        display.setCursor(px, py);
        //display.setTextColor(SCREEN_COLOR_TEXT);//, color);
        #ifdef SHOW_FAHRENHEIT
          val = C2F(val);
        #endif
        display.print(ROUND(val));//, 1);
      #endif
    } 
  }
}
