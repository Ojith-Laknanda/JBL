/*
Deep Sleep with External Wake Up
=====================================
This code displays how to use deep sleep with
an external trigger as a wake up source and how
to store data in RTC memory to use it over reboots

This code is under Public Domain License.

Hardware Connections
======================
Push Button to GPIO 33 pulled down with a 10K Ohm
resistor

NOTE:
======
Only RTC IO can be used as a source for external wake
source. They are pins: 0,2,4,12-15,25-27,32-39.

*/

#define BUTTON_PIN_BITMASK 0x200000000 // 2^33 in hex



/*
Method to print the reason by which ESP32
has been awaken from sleep
*/

static int i=0;
void setup(){
  Serial.begin(115200);
  delay(1000); //Take some time to open up the Serial Monitor
pinMode(33,INPUT);
  
// 
//  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33,1); //1 = High, 0 = Low
//
//  //If you were to use ext1, you would use it like
//  //esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK,ESP_EXT1_WAKEUP_ANY_HIGH);
//
//  //Go to sleep now
//  Serial.println("Going to sleep now");
//  delay(1000);
//  esp_deep_sleep_start();
//  Serial.println("This will never be printed");
}

void loop(){
Serial.println("system ok11111111111111111111111111111111111");
 if(digitalRead(33)==HIGH){
  
 }
 if(i==1){
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33,1); //1 = High, 0 = Low
  Serial.println("system ok");
  i==0;
 }else if(i==2){
  i==0;
  delay(1000);
  x();
 }
   
  
  
}
void x(){
  
  Serial.println("Going to sleep now");
  delay(1000);
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}
