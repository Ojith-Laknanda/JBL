//------------------------------------------------------------------------------------------------------------------------
// I2S configuration structures
#include <driver/i2s.h>
static const i2s_port_t i2s_num = I2S_NUM_0;

static const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100,                            // Note, this will be changed later
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,       // high interrupt priority
    .dma_buf_count = 8,                             // 8 buffers
    .dma_buf_len = 1024,                            // 1K per buffer, so 8K of buffer space
    .use_apll=0,
    .tx_desc_auto_clear= true, 
    .fixed_mclk=-1    
};

// These are the physical wiring connections to our I2S decoder board/chip from the esp32, there are other connections
// required for the chips mentioned at the top (but not to the ESP32), please visit the page mentioned at the top for
// further information regarding these other connections.

static const i2s_pin_config_t pin_config = {
    .bck_io_num = 27,                                 // The bit clock connectiom, goes to pin 27 of ESP32
    .ws_io_num = 26,                                  // Word select, also known as word select or left right clock
    .data_out_num = 25,                               // Data out from the ESP32, connect to DIN on 38357A
    .data_in_num = I2S_PIN_NO_CHANGE                  // we are not interested in I2S data into the ESP32
};

//------------------------------------------------------------------------------------------------------------------------

unsigned long startTime;//////////////////////////////////////////////////////////////////////////////for detect waiting
unsigned long startTime2;//////////////////////////////////////////////////////////////////////////////for auto turn off
int cout=0;
int lowBattCount=0;
//=================================================battery percentage =========================

const int batteryPin = 32;  // Analog input pin for battery voltage
const int R1 = 1000;        // 1k resistor
const int R2 = 10000;       // 10k resistor

#include "SPIFFS.h"

//=================================================leds=========================

#include <FastLED.h>
#define LED_PIN     13
#define NUM_LEDS    16
#define BRIGHTNESS  64

#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

#define UPDATES_PER_SECOND 100



CRGBPalette16 currentPalette;
TBlendType    currentBlending;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;

#define LED_PIN2 22 // Data pin for second LED strip
#define NUM_LEDSS 4 // Number of LEDs per strip
int numLedsToLight;

CRGB leds2[NUM_LEDSS];// power led
//=================================================leds=========================
#include "BluetoothA2DPSink.h"

BluetoothA2DPSink a2dp_sink;

#define BUTTON_FORWARD_PIN           15
#define BUTTON_PLAY_PAUSE_PIN        2
//#define BUTTON_BACKWARD_PIN          0
#define BUTTON_BACKWARD_PIN          4


#define relay           14

#define Name "OJIYA'S JBL 2.0"// add a name
uint8_t my_bt_speaker_state=ESP_AVRC_PLAYBACK_STOPPED;
bool avrc_conn=false;
static bool ct_button_play_pause_press=false;
static int i=0;
#define LONG_PRESS_DELAY 1000
//--------------------vol------------------------------------------------------

int volume = 50;  
//--------------------vol------------------------------------------------------
unsigned long button1PressTime = 0; 
unsigned long button2PressTime = 0;


void setup() {
  Serial.begin(115200);
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);        // ESP32 will allocated resources to run I2S
      i2s_set_pin(I2S_NUM_0, &pin_config);                        // Tell it the pins you will be using

  a2dp_sink.set_pin_config(pin_config);
    a2dp_sink.start(Name);
    
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  
  FastLED.addLeds<WS2812B, LED_PIN2, GRB>(leds2, NUM_LEDSS);
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
    FastLED.setBrightness(  BRIGHTNESS );
    
    currentPalette = RainbowColors_p;
    currentBlending = LINEARBLEND;

  
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33,1); //1 = High, 0 = Low
  Serial.println("system running");
  FastLED.setBrightness(BRIGHTNESS);
  openUpLEDs();

  fill_solid(leds2, NUM_LEDSS, CRGB::White);
  FastLED.show();
  delay(500);
  fill_solid(leds2, NUM_LEDSS, CRGB::Black);
  FastLED.show();
  delay(500);
  battery_percentage();//testing

  pinMode(batteryPin,INPUT);
  pinMode(BUTTON_FORWARD_PIN,INPUT);
  pinMode(BUTTON_PLAY_PAUSE_PIN,INPUT);
  pinMode(BUTTON_BACKWARD_PIN,INPUT);
  pinMode(33,INPUT);
  pinMode(relay,OUTPUT);

  

  Serial.println("system running 333333333333333333333333333");
  digitalWrite(relay,HIGH);////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    
    Serial.println("system running 555555555555555555555");
//    playWav("/Beep.wav");
      playWav("/on.wav");
      Serial.println("system running 2222222222222222");
    startTime = millis();
    startTime2 = millis();
    while(!a2dp_sink.is_connected()){
      
       battery_percentage();//testing
      unsigned long currentTime = millis();
      unsigned long elapsedTime = currentTime - startTime;
      unsigned long elapsedTime2 = currentTime - startTime2;
      
      Serial.print("waiting seconds ");
      Serial.println(elapsedTime);
      
      // Check if 15 seconds have elapsed
      if (elapsedTime >= 30000) {
          // Perform the desired action when 15 seconds have passed
       Serial.println("\n\n30 seconds have passed!");
        playWav("/waiting.wav");
       // Reset the start time for the next 30-second interval
        startTime = millis();
      }
      if (elapsedTime2 >= 200000) {                            // auto power off because of waiting 3min and 20 sec
          sleep();
      }
      avrc_conn=0;
      Serial.println("speaker is not ready to go");
      delay(300);
      if(digitalRead(33)==HIGH){
        turnOffLEDs();
         digitalWrite(relay,LOW);
              sleep();
            }
    }
      avrc_conn=1;
      Serial.println("speaker is ready to go");
//      playWav("/BT CONCTED.wav");
    
    set_volume(30);
}

void loop() {
  battery_percentage();//testing
  
  if (avrc_conn) {
    if(cout==0){
      battery_percentage();//testing
      playWav("/BT CONCTED.wav");
      cout++;
      delay(3000);//new part
     }
    led();
    if(!a2dp_sink.is_connected()){
      cout=0;
      avrc_conn=0;
      playWav("/BT disCONCTED.wav");
      Serial.println("connection terminated");
      battery_percentage();//testing
     }
     battery_percentage();//testing
            delay(5);
            if (digitalRead(BUTTON_FORWARD_PIN ) == HIGH) {
                button1PressTime = millis();led();
                while (digitalRead(BUTTON_FORWARD_PIN ) == HIGH) {
                  Serial.println("hiiiiiiiiiiiiiii+++");
                  volume += 1;   // Increase volume by 10%
                   if (volume > 100) volume = 100;   // Ensure volume doesn't exceed 100%
                  delay(300);
                };
                   if (millis() - button1PressTime > LONG_PRESS_DELAY) {
                     // Button 2 is long-pressed
                     battery_percentage();//testing
                     // Increase volume
                     uint8_t vol_level = volume * 127 / 100; 
                       a2dp_sink.set_volume(vol_level); 
                      Serial.print("vol");
                      Serial.println(vol_level);   
                     Serial.print("vol++");
                     Serial.println(volume);
                   } else {
                     // Button 2 is short-pressed
                     // Play next song
                     a2dp_sink.next();
                     Serial.println("next");
                   }
            }
            if (digitalRead(BUTTON_BACKWARD_PIN ) == HIGH) {
                button2PressTime = millis();led();battery_percentage();//testing
                while (digitalRead(BUTTON_BACKWARD_PIN) == HIGH) {
                  Serial.println("hiiiiiiiiiiiiiii-----------");led();
                  volume -= 1;   // Increase volume by 10%
                   if (volume < 0) volume = 0;   // Ensure volume doesn't exceed 100%
                  delay(300);
                };
                   if (millis() - button2PressTime > LONG_PRESS_DELAY) {
                     // Button 2 is long-pressed
                     battery_percentage();//testing
                     // Increase volume
                     uint8_t vol_level = volume * 127 / 100; 
                       a2dp_sink.set_volume(vol_level); 
                      Serial.print("vol");
                      Serial.println(vol_level);   
                     Serial.print("vol--");
                     Serial.println(volume);
                   } else {
                     // Button 2 is short-pressed
                     // Play next song
                     Serial.println("pre");
                     battery_percentage();//testing
                     a2dp_sink.previous();
                   }
            }
            if (digitalRead(BUTTON_PLAY_PAUSE_PIN)==HIGH) {led();
                if (my_bt_speaker_state == ESP_AVRC_PLAYBACK_STOPPED || my_bt_speaker_state == ESP_AVRC_PLAYBACK_PAUSED) {
                    a2dp_sink.play();
                    battery_percentage();//testing
                    Serial.println("ESP_AVRC_PLAYBACK_PLAYING");
                    my_bt_speaker_state = ESP_AVRC_PLAYBACK_PLAYING;
                } else  if (my_bt_speaker_state == ESP_AVRC_PLAYBACK_PLAYING) {
                   Serial.println("ESP_AVRC_PLAYBACK_PAUSED");
                   a2dp_sink.pause();
                   battery_percentage();//testing
                    my_bt_speaker_state = ESP_AVRC_PLAYBACK_PAUSED;
                }
                delay(300);
            }
            if(digitalRead(33)==HIGH){
              turnOffLEDs();
               digitalWrite(relay,LOW);
              sleep();
            }
        } else {
          ledloss();
            delay(5);
        }


        startTime = millis();
    startTime2 = millis();
    while(!a2dp_sink.is_connected()){
      
      
      unsigned long currentTime = millis();
      unsigned long elapsedTime = currentTime - startTime;
      unsigned long elapsedTime2 = currentTime - startTime2;
      
      Serial.print("waiting seconds ");
      Serial.print(elapsedTime);
      battery_percentage();//testing
      
      // Check if 15 seconds have elapsed
      if (elapsedTime >= 30000) {
          // Perform the desired action when 15 seconds have passed
       Serial.println("30 seconds have passed!");
        playWav("/waiting.wav");
        delay(1000);//new part
        battery_percentage();//testing
       // Reset the start time for the next 30-second interval
        startTime = millis();
      }
      if (elapsedTime2 >= 200000) {                            // auto power off because of waiting 3min and 20 sec
          sleep();
      }
      avrc_conn=0;
      Serial.println("speaker is not ready to go");
      delay(300);
      if(digitalRead(33)==HIGH){
        turnOffLEDs();
         digitalWrite(relay,LOW);
              sleep();
            }
    }
        
        if(digitalRead(33)==HIGH){
          turnOffLEDs();
           digitalWrite(relay,LOW);
              sleep();
            }
  Serial.println("loop");
}

void sleep(){
   Serial.println("Going to sleep now");
   for (int i = 0; i < NUM_LEDSS; i++) {
    leds2[i] = CRGB(0, 0, 0);
  }
  
   playWav("/off.wav");
   digitalWrite(relay,LOW);////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  delay(1000);
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}

void led(){
 ChangePalettePeriodically();
    
    static uint8_t startIndex = 0;
    startIndex = startIndex + 1; /* motion speed */
    
    FillLEDsFromPaletteColors( startIndex);
    
    FastLED.show();
    FastLED.delay(1000 / UPDATES_PER_SECOND);
}


void FillLEDsFromPaletteColors( uint8_t colorIndex)
{
    uint8_t brightness = 255;
    
    for( int i = 0; i < NUM_LEDS; i++) {
        leds[i] = ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
        colorIndex += 3;
    }
}

void ChangePalettePeriodically()
{
    uint8_t secondHand = (millis() / 1000) % 60;
    static uint8_t lastSecond = 99;
    
    if( lastSecond != secondHand) {
        lastSecond = secondHand;
        if( secondHand ==  0)  { currentPalette = RainbowColors_p;         currentBlending = LINEARBLEND; }
        if( secondHand == 10)  { currentPalette = RainbowStripeColors_p;   currentBlending = NOBLEND;  }
        if( secondHand == 15)  { currentPalette = RainbowStripeColors_p;   currentBlending = LINEARBLEND; }
        if( secondHand == 20)  { SetupPurpleAndGreenPalette();             currentBlending = LINEARBLEND; }
        if( secondHand == 25)  { SetupTotallyRandomPalette();              currentBlending = LINEARBLEND; }
        if( secondHand == 30)  { SetupBlackAndWhiteStripedPalette();       currentBlending = NOBLEND; }
        if( secondHand == 35)  { SetupBlackAndWhiteStripedPalette();       currentBlending = LINEARBLEND; }
        if( secondHand == 40)  { currentPalette = CloudColors_p;           currentBlending = LINEARBLEND; }
        if( secondHand == 45)  { currentPalette = PartyColors_p;           currentBlending = LINEARBLEND; }
        if( secondHand == 50)  { currentPalette = myRedWhiteBluePalette_p; currentBlending = NOBLEND;  }
        if( secondHand == 55)  { currentPalette = myRedWhiteBluePalette_p; currentBlending = LINEARBLEND; }
    }
}

// This function fills the palette with totally random colors.
void SetupTotallyRandomPalette()
{
    for( int i = 0; i < 16; i++) {
        currentPalette[i] = CHSV( random8(), 255, random8());
    }
}

void SetupBlackAndWhiteStripedPalette()
{
    // 'black out' all 16 palette entries...
    fill_solid( currentPalette, 16, CRGB::Black);
    // and set every fourth one to white.
    currentPalette[0] = CRGB::White;
    currentPalette[4] = CRGB::White;
    currentPalette[8] = CRGB::White;
    currentPalette[12] = CRGB::White;
    
}

// This function sets up a palette of purple and green stripes.
void SetupPurpleAndGreenPalette()
{
    CRGB purple = CHSV( HUE_PURPLE, 255, 255);
    CRGB green  = CHSV( HUE_GREEN, 255, 255);
    CRGB black  = CRGB::Black;
    
    currentPalette = CRGBPalette16(
                                   green,  green,  black,  black,
                                   purple, purple, black,  black,
                                   green,  green,  black,  black,
                                   purple, purple, black,  black );
}


const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM =
{
    CRGB::Red,
    CRGB::Gray, // 'white' is too bright compared to red and blue
    CRGB::Blue,
    CRGB::Black,
    
    CRGB::Red,
    CRGB::Gray,
    CRGB::Blue,
    CRGB::Black,
    
    CRGB::Red,
    CRGB::Red,
    CRGB::Gray,
    CRGB::Gray,
    CRGB::Blue,
    CRGB::Blue,
    CRGB::Black,
    CRGB::Black
};




 
void ledloss(){
  for(int i = 0; i < NUM_LEDS; i++) {
    for(int j=255;j>=0;j--){
      leds[i] = CRGB(j, j, j); // Red
      FastLED.show();
    } 
    delay(50);
    if(a2dp_sink.is_connected()){
      avrc_conn=1;
      Serial.println("go go");
     }
  }
}

// Function to create the "open up" LED pattern
void openUpLEDs() {
  int delayTime = 50; // Delay between each LED turning on
  int fadeStep = 5;   // Brightness increase step
  
  // Turn on LEDs gradually from dim to bright
  for (int brightness = 0; brightness <= 255; brightness += fadeStep) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB(BRIGHTNESS, BRIGHTNESS, BRIGHTNESS);
    }
    FastLED.show();
    delay(delayTime);
  }
}

// Function to turn off the LEDs
void turnOffLEDs() {
  int delayTime = 50; // Delay between each LED turning off
  int fadeStep = 5;   // Brightness decrease step
  
  // Turn off LEDs gradually from bright to dim
  for (int brightness = 255; brightness >= 0; brightness -= fadeStep) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB(BRIGHTNESS, BRIGHTNESS, BRIGHTNESS);
    }
    FastLED.show();
    delay(delayTime);
  }
  
  // Turn off all LEDs
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 0);
  }

  for (int i = 0; i < NUM_LEDSS; i++) {
    leds2[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
}

 
void set_volume(int vol_percent) {
  uint8_t vol_level = vol_percent * 127 / 100; 
  a2dp_sink.set_volume(vol_level); 
  Serial.print("vol");
  Serial.println(vol_level);
}




void battery_percentage() {
  int sensorValue = analogRead(batteryPin);
  Serial.println(sensorValue);
  float voltage = sensorValue * (5.0 / 1023.0)*11.03 / 0.95/5.3;
  
  Serial.print("Battery Voltage: ");
  Serial.println(voltage);
  
    
   if(voltage <= 10.4) {
      numLedsToLight = 1;
      fill_solid(leds2, NUM_LEDSS, CRGB::Red);FastLED.show();FastLED.setBrightness(BRIGHTNESS);
      Serial.println("Low battery");
      playWav("/battery low.wav");
      lowBattCount++;
      delay(1000);
      if(lowBattCount==2){
        sleep();                                
      }
   }else if (voltage <= 10.9) {
    numLedsToLight = 2;
    fill_solid(leds2, NUM_LEDSS, CRGB::Yellow);FastLED.show();FastLED.setBrightness(BRIGHTNESS);
    Serial.println("yellow");
   } else if (voltage <= 11.35) {
    numLedsToLight = 3;
    fill_solid(leds2, NUM_LEDSS, CRGB::Green);FastLED.show();FastLED.setBrightness(BRIGHTNESS);
    Serial.println("green");
   } else if (voltage <= 11.8) {
    numLedsToLight = 4;
    fill_solid(leds2, NUM_LEDSS, CRGB::White);FastLED.show();FastLED.setBrightness(BRIGHTNESS);
    Serial.println("white");
  }else if(voltage > 12.0){
     for (int j = 1; j<= NUM_LEDSS; j++) {
      // Display the LEDs
      for (int i = 0; i < NUM_LEDSS; i++) {
        leds2[i] = (i < j) ? CRGB::Green : CRGB::Black;FastLED.show();FastLED.setBrightness(BRIGHTNESS);
        Serial.println((i < j)?"Green":"black");
        
      }

      // Show the LED strip
      FastLED.show();FastLED.setBrightness(BRIGHTNESS);
      delay(500); // Adjust the delay as needed
    }
  }
  for (int i = 0; i < NUM_LEDSS; i++) {
    leds2[i] = (i < numLedsToLight) ? leds2[i] : CRGB::Black;FastLED.show();FastLED.setBrightness(BRIGHTNESS);
  }

  // Set LED brightness
  FastLED.setBrightness(BRIGHTNESS);

  // Show the LED strip
  FastLED.show();
  

  delay(1000);  // Delay for 1 seconds before the next reading
}

// ==> plays .wav records (in SPIFFS)
//    n : SPIFFS file name
//////////////////////////////////////////////////////////////////////////
#define I2SN (i2s_port_t)0
void playWav(char* n)
{
  struct header
  {
    uint8_t a[16];
    uint8_t cksize[4];
    uint8_t wFormatTag[2];
    uint8_t nChannels[2];
    uint8_t nSamplesPerSec[4];
    uint8_t c[16];
  };
  uint32_t rate;
  uint8_t b[46];
  int l;
  bool mono;
  size_t t;
  File f = SPIFFS.open(n, FILE_READ);
  l = (int) f.read(b, sizeof(b));
  if (b[22] == 1) mono = true; else mono = false;
  rate =  (b[25] << 8) + b[24];
  printf(" rate = %d\n", rate);
  i2s_set_clk(I2SN, rate, (i2s_bits_per_sample_t)16, (i2s_channel_t)2);
  do
  {
    if (mono == true)
    {
      l = (int)f.read((uint8_t*)b, 2);
      b[2] = b[0]; b[3] = b[1];

    }
    else
      l = (int)f.read((uint8_t*)b, 4);

    i2s_write(I2SN, b, 4, &t, 1000);
  }
  while (l != 0);
  i2s_zero_dma_buffer((i2s_port_t)0);
  
    Serial.println("audio play");
  f.close();
}
