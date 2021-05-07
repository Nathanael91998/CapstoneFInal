
/*
   TODO

  Libraries used
  https://github.com/cmaglie/FlashStorage
  https://github.com/neu-rah/ArduinoMenu             ArduinoMenu library 4.18.2
  https://www.arduino.cc/en/Reference/SD             SD 1.2.4 
  https://github.com/adafruit/Adafruit_SSD1306
  https://github.com/adafruit/Adafruit-GFX-Library   Adafruit GFX Library 1.0.0
  https://github.com/adafruit/Adafruit_FeatherOLED   Adafruit Feather OLED 1.0.0
  https://github.com/michd/Arduino-MCP492X
  https://playground.arduino.cc/Code/Keypad/
  https://github.com/stblassitude/Adafruit_SSD1306_Wemos_OLED

  Board Manager: Adafruit nrf52 Ver 0.16.0
  Adafruit Bluefruit Feather nRF52832 


  The circuit:
   analog sensors on analog ins 0, 1, 2, 3
   SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)
    CS - pin 28 MCP4922
 ** OLED_reset pin 7
 **
 **

    Author: Lawrence Kincheloe

  add "{build.variant.path}/libarm_cortexM4lf_math.a" to platforms.txt in recipe.c.combine.pattern, before "{build.path/archive_file}"
  add to boards.txt the below two lines, and replace #feather52832.build.extra_flags=-DNRF52832_XXAA -DNRF52 -DARDUINO_NRF52_FEATHER
  feather52832.build.extra_flags=-DNRF52832_XXAA -DNRF52 -DARDUINO_NRF52_FEATHER -D__FPU_PRESENT -DARM_MATH_CM4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
  feather52832.compiler.arm.cmsis.ldflags="-L{build.variant.path}" -larm_cortexM4lf_math -mfloat-abi=hard -mfpu=fpv4-sp-d16


  edit operation:
  1 - first * -> enter field navigation use +/- to select character position
  2 - second * -> enter character edit use +/- to select character value
  3 - third * -> return to field navigation (1)
  4 - fourth * without changing position -> exit edit mode

*/
#define FIRMWARE_VERSION 0.004

#if defined(__MK66FX1M0__) || defined(__MK20DX256__)|| defined(NRF52)
#ifndef ARM_MATH_CM4
#define ARM_MATH_CM4
#endif
#endif

#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configINCLUDE_FREERTOS_TASK_C_ADDITIONS_H                                 1
#define configUSE_TRACE_FACILITY                                                  1

#include <Arduino.h>
//#include <rtos.h>
#include <MCP492X.h> //https://github.com/michd/Arduino-MCP492X
//#include <SPI.h>  //https://dorkbotpdx.org/blog/paul/spi_transactions_in_arduino/
#include <SD.h>
#include <avr/pgmspace.h>
#include <Wire.h>
#include <arm_math.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
//#include <MemoryFree.h> //for debugging only
#include "SmartFlowMeter.h"
#include <bluefruit.h>
#include "defines.h"

BLEDis  bledis;
BLEUart bleuart;

 
EthernetWebServer server(80);
void bluetoothSetup() {
  Serial.begin(115200);
 
  Serial.println("Bluefruit52 BLEUART Example");
 
 
  Bluefruit.begin();
  Bluefruit.setName("Bluefruit52");

 
  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();
 
  // Configure and Start BLE Uart Service
  bleuart.begin();
 

 
  // Set up Advertising Packet
  setupAdv();
 
  // Start Advertising
  Bluefruit.Advertising.start();
}

void setupAdv(void)
{
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
 
  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);
 
  // There is no room for Name in Advertising packet
  // Use Scan response for Name
  Bluefruit.ScanResponse.addName();
}

void handleRoot()
{
  String html = F("Hello from HelloServer running on ");

  html += String(BOARD_NAME); 
  
  server.send(200, F("text/plain"), html);
}

void handleNotFound()
{
  String message = F("File Not Found\n\n");
  
  message += F("URI: ");
  message += server.uri();
  message += F("\nMethod: ");
  message += (server.method() == HTTP_GET) ? F("GET") : F("POST");
  message += F("\nArguments: ");
  message += server.args();
  message += F("\n");
  
  for (uint8_t i = 0; i < server.args(); i++)
  {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  
  server.send(404, F("text/plain"), message);
  

}

 
void etherSetup(void)
{
 

#if USE_ETHERNET_WRAPPER

  EthernetInit();

#else

#if USE_NATIVE_ETHERNET
  ET_LOGWARN(F("======== USE_NATIVE_ETHERNET ========"));
#elif USE_ETHERNET
  ET_LOGWARN(F("=========== USE_ETHERNET ==========="));
#elif USE_ETHERNET2
  ET_LOGWARN(F("=========== USE_ETHERNET2 ==========="));
#elif USE_ETHERNET3
  ET_LOGWARN(F("=========== USE_ETHERNET3 ==========="));
#elif USE_ETHERNET_LARGE
  ET_LOGWARN(F("=========== USE_ETHERNET_LARGE ==========="));
#elif USE_ETHERNET_ESP8266
  ET_LOGWARN(F("=========== USE_ETHERNET_ESP8266 ==========="));
#elif USE_ETHERNET_ENC
  ET_LOGWARN(F("=========== USE_ETHERNET_ENC ==========="));  
#else
  ET_LOGWARN(F("========================="));
#endif

  ET_LOGWARN(F("Default SPI pinout:"));
  ET_LOGWARN1(F("MOSI:"), MOSI);
  ET_LOGWARN1(F("MISO:"), MISO);
  ET_LOGWARN1(F("SCK:"),  SCK);
  ET_LOGWARN1(F("SS:"),   SS);
  ET_LOGWARN(F("========================="));

#if defined(ESP8266)
  // For ESP8266, change for other boards if necessary
  #ifndef USE_THIS_SS_PIN
    #define USE_THIS_SS_PIN   D2    // For ESP8266
  #endif

  ET_LOGWARN1(F("ESP8266 setCsPin:"), USE_THIS_SS_PIN);

  #if ( USE_ETHERNET || USE_ETHERNET_LARGE || USE_ETHERNET2 || USE_ETHERNET_ENC )
    // For ESP8266
    // Pin                D0(GPIO16)    D1(GPIO5)    D2(GPIO4)    D3(GPIO0)    D4(GPIO2)    D8
    // Ethernet           0                 X            X            X            X        0
    // Ethernet2          X                 X            X            X            X        0
    // Ethernet3          X                 X            X            X            X        0
    // EthernetLarge      X                 X            X            X            X        0
    // Ethernet_ESP8266   0                 0            0            0            0        0
    // D2 is safe to used for Ethernet, Ethernet2, Ethernet3, EthernetLarge libs
    // Must use library patch for Ethernet, EthernetLarge libraries
    Ethernet.init (USE_THIS_SS_PIN);

  #elif USE_ETHERNET3
    // Use  MAX_SOCK_NUM = 4 for 4K, 2 for 8K, 1 for 16K RX/TX buffer
    #ifndef ETHERNET3_MAX_SOCK_NUM
      #define ETHERNET3_MAX_SOCK_NUM      4
    #endif
  
    Ethernet.setCsPin (USE_THIS_SS_PIN);
    Ethernet.init (ETHERNET3_MAX_SOCK_NUM);

  #elif USE_CUSTOM_ETHERNET
  
    // You have to add initialization for your Custom Ethernet here
    // This is just an example to setCSPin to USE_THIS_SS_PIN, and can be not correct and enough
    Ethernet.init(USE_THIS_SS_PIN);
  
  #endif  //( USE_ETHERNET || USE_ETHERNET2 || USE_ETHERNET3 || USE_ETHERNET_LARGE )

#elif defined(ESP32)

  // You can use Ethernet.init(pin) to configure the CS pin
  //Ethernet.init(10);  // Most Arduino shields
  //Ethernet.init(5);   // MKR ETH shield
  //Ethernet.init(0);   // Teensy 2.0
  //Ethernet.init(20);  // Teensy++ 2.0
  //Ethernet.init(15);  // ESP8266 with Adafruit Featherwing Ethernet
  //Ethernet.init(33);  // ESP32 with Adafruit Featherwing Ethernet

  #ifndef USE_THIS_SS_PIN
    #define USE_THIS_SS_PIN   22    // For ESP32
  #endif

  ET_LOGWARN1(F("ESP32 setCsPin:"), USE_THIS_SS_PIN);

  // For other boards, to change if necessary
  #if ( USE_ETHERNET || USE_ETHERNET_LARGE || USE_ETHERNET2 || USE_ETHERNET_ENC )
    // Must use library patch for Ethernet, EthernetLarge libraries
    // ESP32 => GPIO2,4,5,13,15,21,22 OK with Ethernet, Ethernet2, EthernetLarge
    // ESP32 => GPIO2,4,5,15,21,22 OK with Ethernet3
  
    //Ethernet.setCsPin (USE_THIS_SS_PIN);
    Ethernet.init (USE_THIS_SS_PIN);
  
  #elif USE_ETHERNET3
    // Use  MAX_SOCK_NUM = 4 for 4K, 2 for 8K, 1 for 16K RX/TX buffer
    #ifndef ETHERNET3_MAX_SOCK_NUM
      #define ETHERNET3_MAX_SOCK_NUM      4
    #endif
  
    Ethernet.setCsPin (USE_THIS_SS_PIN);
    Ethernet.init (ETHERNET3_MAX_SOCK_NUM);

  #elif USE_CUSTOM_ETHERNET
  
    // You have to add initialization for your Custom Ethernet here
    // This is just an example to setCSPin to USE_THIS_SS_PIN, and can be not correct and enough
    Ethernet.init(USE_THIS_SS_PIN); 
  
  #endif  //( USE_ETHERNET || USE_ETHERNET2 || USE_ETHERNET3 || USE_ETHERNET_LARGE )

#else   //defined(ESP8266)
  // unknown board, do nothing, use default SS = 10
  #ifndef USE_THIS_SS_PIN
    #define USE_THIS_SS_PIN   10    // For other boards
  #endif

  #if defined(BOARD_NAME)
    ET_LOGWARN3(F("Board :"), BOARD_NAME, F(", setCsPin:"), USE_THIS_SS_PIN);
  #else
    ET_LOGWARN1(F("Unknown board setCsPin:"), USE_THIS_SS_PIN);
  #endif

  // For other boards, to change if necessary
  #if ( USE_ETHERNET || USE_ETHERNET_LARGE || USE_ETHERNET2  || USE_ETHERNET_ENC || USE_NATIVE_ETHERNET )
    // Must use library patch for Ethernet, Ethernet2, EthernetLarge libraries
  
    Ethernet.init (USE_THIS_SS_PIN);
  
  #elif USE_ETHERNET3
    // Use  MAX_SOCK_NUM = 4 for 4K, 2 for 8K, 1 for 16K RX/TX buffer
    #ifndef ETHERNET3_MAX_SOCK_NUM
      #define ETHERNET3_MAX_SOCK_NUM      4
    #endif
  
    Ethernet.setCsPin (USE_THIS_SS_PIN);
    Ethernet.init (ETHERNET3_MAX_SOCK_NUM);

  #elif USE_CUSTOM_ETHERNET
  
    // You have to add initialization for your Custom Ethernet here
    // This is just an example to setCSPin to USE_THIS_SS_PIN, and can be not correct and enough
    Ethernet.init(USE_THIS_SS_PIN);
    
  #endif  //( USE_ETHERNET || USE_ETHERNET2 || USE_ETHERNET3 || USE_ETHERNET_LARGE )

#endif    //defined(ESP8266)


#endif  //USE_ETHERNET_WRAPPER


  // start the ethernet connection and the server:
  // Use DHCP dynamic IP and random mac
  uint16_t index = millis() % NUMBER_OF_MAC;
  // Use Static IP
  //Ethernet.begin(mac[index], ip);
  Ethernet.begin(mac[index]);

  // Just info to know how to connect correctly
  Serial.println(F("========================="));
  Serial.println(F("Currently Used SPI pinout:"));
  Serial.print(F("MOSI:"));
  Serial.println(MOSI);
  Serial.print(F("MISO:"));
  Serial.println(MISO);
  Serial.print(F("SCK:"));
  Serial.println(SCK);
  Serial.print(F("SS:"));
  Serial.println(SS);
#if USE_ETHERNET3
  Serial.print(F("SPI_CS:"));
  Serial.println(SPI_CS);
#endif
  Serial.println("=========================");

  Serial.print(F("Using mac index = "));
  Serial.println(index);

  Serial.print(F("Connected! IP address: "));
  Serial.println(Ethernet.localIP());

  server.on(F("/"), handleRoot);

  server.on(F("/inline"), []() 
  {
    server.send(200, F("text/plain"), F("This works as well"));
  });

  server.onNotFound(handleNotFound);

  server.begin();

  Serial.print(F("HTTP EthernetWebServer is @ IP : "));
  Serial.println(Ethernet.localIP());
}


void setup() {

  bluetoothSetup();
  etherSetup();
  dataString.reserve(2048);
  server.handleClient();
  //only for prototype board
  pinMode( OLED_RESET , OUTPUT);
  digitalWrite( OLED_RESET, LOW);
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  //arm_lms_init_f32(&S, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], BETA, 1);
  // process_least_squares_buffer();//test least_squares

  inputString.reserve(200);
  tempString.reserve(512);
  //init dac.
  DAC.begin();

  //load coefficients from device
  load_flash_coeff();

  //Enable Screen
  digitalWrite( OLED_RESET, HIGH); //needed for prototype
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D); // Address 0x3D for 128x64
  // Clear the buffer. to remove adafruit splash screen
  display.clearDisplay();
  display.display();



  // text display tests
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setRotation(3);
  display.setCursor(0, 0);
  display.display(); // actually display all of the above
  Serial.print("Initializing SD card... ");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    display.println("Card failed, or not present");
    // don't do anything more:
    return;
  } else {
    Serial.println("card initialized.");
    readSDConfig();
    open_SD();
  }

  display.setCursor(0, 0);
  display.display(); // actually display all of the above

  //Build the menu in a hierarchical structure  BuildMenu(struct menuLevel *currentNode, char name[16], void (*DoWork)(void) , struct menuLevel *prevNode, struct menuLevel *nextNode,struct menuLevel *upNode,struct menuLevel *downNode)
  Serial.print("algorithmEnable: ");
  Serial.println((int)(*algorithmEnable));
  if (((int)*algorithmEnable == 0) | ((int)*algorithmEnable == 6)) {
    BuildMenu(&blankM, "blankM", menuScreenSaver, &displayM, &displayM, &displayM, &displayM);
    BuildMenu(&displayM, "Output", menuCalibrate, &blankM, 0, 0, 0);
  }
  if ((int)*algorithmEnable == 1) {
    BuildMenu(&blankM, "blankM", menuScreenSaver, &displayM, &displayM, &displayM, &displayM);
    BuildMenu(&displayM, "Output", menu_fresh_air_fan, &blankM, 0, 0, 0);
  }
  if ((int)*algorithmEnable == 4) {
    BuildMenu(&blankM, "blankM", menuScreenSaver, &displayM, &displayM, &displayM, &displayM);
    BuildMenu(&displayM, "Output", menu_fan_flow_meter, &blankM, 0, 0, 0);
  }
  // BuildMenu(&displayM,"Output", menuDisplayRaw, &blankM, &configM, 0, 0);
  // BuildMenu(&configM,"Configure", menuCalibrate, &displayM, 0, 0, &editM);
  BuildMenu(&editM, "Edit", menuValueEdit, &configM, 0, &configM, &coeffM);
  BuildMenu(&coeffM, "Edit", menuCoeffEdit, &configM, 0, 0 , 0);
  BuildMenu(&addM, "Add", menuEntryAdd, &coeffM, &subtractM, 0 , 0);
  BuildMenu(&subtractM, "Subtract", menuEntrySubtract, &addM, &saveM,  0 , 0);
  BuildMenu(&removeM, "Remove", menuEntryRemove, 0, 0, 0 , 0);
  BuildMenu(&saveM, "Save", menuValueSave, &subtractM, 0, 0 , 0);
  // BuildMenu(&tempM,"Temperature", DoWork_Temp, &fanM,0, 0, 0);

  //Assign the current menu item the first item in the menu
  currentM = &blankM;
  //(*PIN_A0_Function);
  // (*PIN_A1_Function);
  // (*PIN_A2_Function);
  // (*PIN_A3_Function);

  if ((int)(* DAC0_output_variable) == 1) {
    (dac_value_normalized_V) = (C_dac_value_results);
  }
  if ((int)(* DAC0_output_variable) == 2) {
    (dac_value_normalized_V) = (M_dac_value_results);
  }
  if ((int)(* DAC0_output_variable) == 3) {
    (dac_value_normalized_V) = (D_dac_value_results);
  }
  if ((int)(* DAC0_output_variable) == 4) {
    (dac_value_normalized_V) = (R_dac_value_results);
  }
  if ((int)(* DAC1_output_variable) == 1) {
    dac_value_normalized_I = (C_dac_value_results);
  }
  if ((int)(* DAC1_output_variable) == 2) {
    dac_value_normalized_I = (M_dac_value_results);
  }
  if ((int)(* DAC1_output_variable) == 3) {
    dac_value_normalized_I = (D_dac_value_results);
  }
  if ((int)(* DAC1_output_variable) == 4) {
    dac_value_normalized_I = (R_dac_value_results);
  }

  //Serial.println((int)(*ADC0_config));
  // Serial.println((int)(*ADC1_config));
  //  Serial.println((int)(*ADC2_config));
  //  Serial.println((int)(*ADC3_config));

  //Enable pin interrupts
  if ((((int)(*ADC0_config)) == 3) or (((int)(*ADC0_config)) == 4)) {
    attachInterrupt(digitalPinToInterrupt(ADC0_PIN), ADC_FALLING_ISR, FALLING);
    PULSE_TIMER_SETUP();
    PIN_A0_Function = readTIMER0;
    ADC_COUNTER_IO = readDIO0;
    if (((int)(*ADC0_config)) == 4){
      ADC0_Conversion_Function =  imp_kWh_to_w;
    }
    if (((int)(*ADC0_config)) == 3){
      ADC0_Conversion_Function = normal_to_Frequency;
    }
  } else {
    Serial.println("ADC0_config initialized.");
    PIN_A0_Function = readADC0;
    ADC0_Conversion_Function = ADC0_Conversion;
  }
  if ((((int)(*ADC1_config)) == 3) or (((int)(*ADC1_config)) == 4)) {
    attachInterrupt(digitalPinToInterrupt(ADC1_PIN), ADC_FALLING_ISR, FALLING);
    PULSE_TIMER_SETUP();
    PIN_A1_Function = readTIMER1;
    ADC_COUNTER_IO = readDIO1;
    if (((int)(*ADC1_config)) == 4){
      ADC1_Conversion_Function =  imp_kWh_to_w;
    }
    if (((int)(*ADC1_config)) == 3){
      ADC1_Conversion_Function = normal_to_Frequency;
    }
  }  else {
    Serial.println("ADC1_config initialized.");
    PIN_A1_Function = readADC1;
    ADC1_Conversion_Function = ADC1_Conversion;
  }
  if ((((int)(*ADC2_config)) == 3) or (((int)(*ADC2_config)) == 4)) {
    attachInterrupt(digitalPinToInterrupt(ADC2_PIN), ADC_FALLING_ISR, FALLING);
    PULSE_TIMER_SETUP();
    PIN_A2_Function = readTIMER2;
    ADC_COUNTER_IO = readDIO2;
    if (((int)(*ADC2_config)) == 4){
      ADC2_Conversion_Function =  imp_kWh_to_w;
    }
    if (((int)(*ADC2_config)) == 3){
      ADC2_Conversion_Function = normal_to_Frequency;
    }
  } else {
    Serial.println("ADC2_config initialized.");
    PIN_A2_Function = readADC2;
    ADC2_Conversion_Function = ADC2_Conversion;
  }
  if ((((int)(*ADC3_config)) == 3) or (((int)(*ADC3_config)) == 4)) {
    attachInterrupt(digitalPinToInterrupt(ADC3_PIN), ADC_FALLING_ISR, FALLING);
    PULSE_TIMER_SETUP();
    PIN_A3_Function = readTIMER3;
    ADC_COUNTER_IO = readDIO3;    
    if (((int)(*ADC3_config)) == 4){
      ADC3_Conversion_Function =  imp_kWh_to_w;
    }
    if (((int)(*ADC3_config)) == 3){
      ADC3_Conversion_Function = normal_to_Frequency;
    }
  } else {
    Serial.println("ADC3_config initialized.");
    PIN_A3_Function = readADC3;
    ADC3_Conversion_Function = ADC3_Conversion;
    //ADC3_Conversion_Function = adc_value_Conversion_Function_list[ADC3_config];
  }
  // 0:Not used, 1: applied to ADC0_PIN, 2: applied to ADC1_PIN, 3: applied to ADC2_PIN, 4: applied to ADC3_PIN
 
  if ( (int)(*discont_R))
  {
    Serial.println("R_adc_value_normalized initialized.");
    R_adc_value_normalized = &adc_value_normalized[(int)(*discont_R) - 1];
    R_Conversion = adc_value_Conversion_Function_list[(int)(*discont_R) - 1];
  }

  if ( (int)(*discont_C))
  {
    Serial.println("C_adc_value_normalized initialized.");
    C_adc_value_normalized = &adc_value_normalized[(int)(*discont_C) - 1];
    C_Conversion = adc_value_Conversion_Function_list[(int)(*discont_C) - 1];
  }

  if ( (int)(*discont_M))
  {
    Serial.println("M_adc_value_normalized initialized.");
    M_adc_value_normalized = &adc_value_normalized[(int)(*discont_M) - 1];
    M_Conversion = adc_value_Conversion_Function_list[(int)(*discont_M) - 1];
  }

  if ( (int)(*discont_D))
  {
    Serial.println("D_adc_value_normalized initialized.");
//   // Serial.println((int)(*discont_D));
//  //  Serial.println(String((unsigned int)adc_value_Conversion_Function_list));
//    Serial.println(String((unsigned int)ADC3_Conversion));
//    Serial.println(String((unsigned int)ADC3_Conversion_Function));
//    Serial.println(String((unsigned int)adc_value_Conversion_Function_list[0]));
//    Serial.println(String((unsigned int)adc_value_Conversion_Function_list[1]));
//    Serial.println(String((unsigned int)adc_value_Conversion_Function_list[2]));
//    Serial.println(String((unsigned int)adc_value_Conversion_Function_list[3]));
    D_adc_value_normalized = &adc_value_normalized[(int)(*discont_D) - 1];
//    Serial.println(String((unsigned int)D_Conversion));
//    Serial.println(String((unsigned int)*D_Conversion));
    D_Conversion = adc_value_Conversion_Function_list[(int)(*discont_D) - 1];
//    Serial.println(String((unsigned int)D_Conversion));
//    Serial.println(String((unsigned int)*D_Conversion));
  }


  if ((int)*algorithmEnable == 0) {
    dataString += String("Calibrating");
  };
  if ((int)*algorithmEnable == 1) {
    dataString += "Fresh Air Fan\r\nADC0,ADC1,ADC2,ADC3,DAC0,DAC1," + String(unit_label.var.C_unit_label) + "," + String(unit_label.var.V_unit_label) + ",millisecond\r\n";
  };
  if ((int)*algorithmEnable == 2) {
    dataString += "valve_flow_rate\r\nADC0,ADC1,ADC2,ADC3,millisecond\r\n";
  };
  if ((int)*algorithmEnable == 3) {
    dataString += "pump_fan_flow_rat\r\nADC0,ADC1,ADC2,ADC3,millisecond\r\n";
  };
  if ((int)*algorithmEnable == 4) {
    dataString += "fan_flow_meter\r\nADC0,ADC1,ADC2,ADC3,DAC0,DAC1,Frequency normalized,Motor Efficiency,Shaft Power, Fan Efficiency, Flow Rate,millisecond\r\n";
  };
  if ((int)*algorithmEnable == 5) {
    dataString += String("dump memory");
  };
  if ((int)*algorithmEnable == 6) {
    dataString += String("Self Calibrating");
  };
  pinMode(BUTTON_UP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_UP), BUTTON_UP_ISR, FALLING);

  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_DOWN), BUTTON_DOWN_ISR, FALLING);

  pinMode(BUTTON_MENU, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_MENU), BUTTON_MENU_ISR, FALLING);

  pinMode(BUTTON_SELECT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_SELECT), BUTTON_SELECT_ISR, FALLING);
  //debounce timer, starts when button is pressed
  btnDownTime.begin(15, btnDownCheck, NULL, false);

  Scheduler.startLoop((SchedulerRTOS::taskfunc_t)menuloop, 1024, TASK_PRIO_LOW, "menuLoop");
  Scheduler.startLoop(SD_write, 1024, TASK_PRIO_LOW, "SD_write");
  // Scheduler.startLoop(menuloop);
  // Scheduler.startLoop(SD_write);
  //Scheduler.startLoop(
  // Initialize analogReadTimer for 100 ms and start it
  configureADC();
  // lastMillis = millis()-sampleRate;
  lastMillis = millis();
  analogReadTimer.begin(10, analog_timer_callback);
  analogReadTimer.start();
  //sdCardWriteTimer.begin(sampleRate,);
  // sdCardWriteTimer.start();

}


void loop() {
  //dbgMemInfo();
  // Serial.print(FreeRam());
  //  Serial.write(',');
  // Serial.println(freeMemory());
  //stop analog reading while processing current values
  //readADC();
  //Serial.println("Here1");
  unsigned long now = millis();
  // analogReadTimer.stop();
  // Serial.println("Here2");
  normalize_ADC();
   // Serial.println("Here3");
  //U9 Which algorithm to run Initialize calibartion (0,calibrate | 1 //fresh_air_fan();2  //valve_flow_rate();3  //pump_fan_flow_rate(); //4 fan_flow_meter) not stored in system, read from sd card only

  if ((int)*algorithmEnable == 0) {
    calibrate();
  }
  if ((int)*algorithmEnable == 1) {
    fresh_air_fan();
    //dataString+="Fresh Air Fan\r\nADC0,ADC1,ADC2,ADC3,DAC0,DAC1,W,m3/h,millisecond\r\n";};
    tempString = String(adc_value_calibrated[0], 4) + "," + String(adc_value_calibrated[1], 4) + "," + String( adc_value_calibrated[2], 4) + "," + \
                 String( adc_value_calibrated[3], 4) + "," + String(((*dac_value_normalized_V) * 10)) + "," +  String(((*dac_value_normalized_I) * 16) + 4) + "," + \
                 String(((*C_adc_value_normalized) * (*C_disp_coeff)) + (*C_disp_offset)) + "," + String(((*C_dac_value_results) * (*VOUT_disp_coeff)) + (*VOUT_disp_offset)) + "," + lastMillis + "\r\n";
  }
  if ((int)*algorithmEnable == 2) {
    valve_flow_rate();
    dataString += String(adc_value_calibrated[0], 4) + "," + String(adc_value_calibrated[1], 4) + "," + String( adc_value_calibrated[2], 4) + "," + String( adc_value_calibrated[3], 4) + "," + lastMillis + "\r\n";
  }
  if ((int)*algorithmEnable == 3) {
    pump_fan_flow_rate();
    dataString += String(adc_value_calibrated[0], 4) + "," + String(adc_value_calibrated[1], 4) + "," + String( adc_value_calibrated[2], 4) + "," + String( adc_value_calibrated[3], 4) + "," + lastMillis + "\r\n";
  }
  if ((int)*algorithmEnable == 4) {
    //Serial.println("Here4");
    fan_flow_meter();
    //Serial.println("Here5");
    //dataString+="fan_flow_meter\r\nADC0,ADC1,ADC2,ADC3,DAC0,DAC1,Frequency normalized,Motor Efficiency,Shaft Power, Fan Efficiency, Flow Rate,millisecond\r\n";}; (*D_dac_value_results) = (*C_dac_value_results)/(*Qmax);
    tempString = String(adc_value_calibrated[0], 4) + "," + String(adc_value_calibrated[1], 4) + "," + String( adc_value_calibrated[2], 4) + "," + String( adc_value_calibrated[3], 4) + "," + \
                 String(((*dac_value_normalized_V) * 10)) + "," +  String(((*dac_value_normalized_I) * 16) + 4) + "," + \
                 String((*M_adc_value_normalized) * (*FrequencyRange), 14) + "," + String(eff_m, 14) + "," + String(powerR, 14) + "," + String(eff_dev, 14) + "," + String((*C_dac_value_results), 14) + "," + lastMillis + "\r\n";
  }
  if ((int)*algorithmEnable == 5) {
    dumpMemory();
  }
  if ((int)*algorithmEnable == 6) {
    selfCalibrate();
  }
  if (dataString.length() > 1536) {
    recover_SD();
  }


  if ((dataString.length() + tempString.length()) < 2048) {
    dataString += tempString;
    //Serial.println( (*D_adc_value_normalized));
    //Serial.println( (*imp_kWh));
    //Serial.println(((3600000 / (*imp_kWh)) / (*D_adc_value_normalized)));
    //   Serial.print("dataString Length: ");
    // Serial.println(dataString.length());
    //   Serial.println(String(C_unit_label));
    //  Serial.println(unit_label.var.C_unit_label);
    //  Serial.println(unit_label.var.M_unit_label);
    //  Serial.println(unit_label.var.D_unit_label);
    //   Serial.println(unit_label.var.R_unit_label);
    //  Serial.println(unit_label.var.V_unit_label);
    // Serial.println(unit_label.var.I_unit_label);
//     Serial.println(adc_value_calibrated[3], 4);
//if(current_counter_period_fall > 3000000){
//Serial.print("overflow_counter = ");
//  Serial.println(overflow_counter);
//  Serial.print("current_counter_fall_new =");
//  Serial.println(current_counter_fall_new );
//   Serial.print("current_counter_fall_last =");
//  Serial.println(current_counter_fall_last );
//  Serial.print("current_counter_fall_old = ");
//  Serial.println(current_counter_fall_old);
//  Serial.print("current_counter_period_fall =");
//  Serial.println(current_counter_period_fall);
//  Serial.print("overflow_counter_fall = ");
//  Serial.println(overflow_counter_fall);
//}
//  Serial.print("current_counter_rise_new = ");
//  Serial.println(current_counter_rise_new);
//  Serial.print("current_counter_rise_old = ");
//  Serial.println(current_counter_rise_old);
//  Serial.print("current_counter_period_rise = ");
//  Serial.println(current_counter_period_rise);
//  Serial.print("overflow_counter_rise = ");
//  Serial.println(overflow_counter_rise);
//  Serial.print("ADC_COUNTER_IO = ");
//  Serial.println(ADC_COUNTER_IO());
//  Serial.print("flag_counter = ");
//  Serial.println(flag_counter);
  } else {
    Serial.println("dataString capacity overflow");
  }


  analogReadTimer.stop(); //nessisary to keep it from hanging
  update_DAC();
  analogReadTimer.start(); //nessisary to keep it from hanging

  //put a time stamp on the reading, when overflow occurs, open a new file
   


  //dataString +=
  //dataString += String(vTST, 4) + "," + String(deltaP, 4) + "," + String(power, 4) + "," + String(freq, 4) + "," + String(pump_fan_flow_rate_result, 4) + "," + String(valve_flowRate_result, 4)+ "," +lastMillis+ "\r\n";
  // dataString += String(adc_value_normalized[0],4) + "," + String(adc_value_normalized[1],4)+ "," + String( adc_value_normalized[2],4)+ "," + String( adc_value_normalized[3],4)+ "," +lastMillis+ "\r\n";
  //   dataString += String(adc_value_buffer2[0],4) + "," + String(adc_value_buffer2[1],4)+ "," + String( adc_value_buffer2[2],4)+ "," + String( adc_value_buffer2[3],4)+ "," +lastMillis+ "\r\n";

  //   dataString += String(1/(((float)current_counter_period_fall/1000000)+(36*(float)overflow_counter_fall)),14) + "," + current_counter_fall_new+ "," + current_counter_fall_old+ "," + String( (((float)current_counter_period_fall/1000000)+(36*(float)overflow_counter_fall)),14)+ "," +current_counter_period_fall+ "," +overflow_counter_fall+ ","+ flag_counter+"\r\n";

  // Serial.println(dataString);
  //Serial.println("Here6");
  // write_datalog_SD();
  //Serial.println("Here7");
  //could be the time sample rate
  // if((now -lastMillis)>0){
  //   delay(sampleRate - (sampleRate -(now -lastMillis)));
  // }
  // else{delay(sampleRate);}
  //Serial.println(sampleRate);
  //Serial.println(millis());
  lastMillis = now; // discard last lastMillis, use it to build the next one
  // wait for a second
  //Serial.println(sampleRate-(millis()-now));
  //Serial.println(now);
  unsigned long delayTime = 0;
  now = (millis());
  if (now < lastMillis) {
   // Serial.println("Here5");
    //sdCardWriteTimer.stop();
    // close_SD();
    //open_SD();
    //sdCardWriteTimer.start();
    dataString += "TIMER_OVERFLOW\r\n";
    delayTime = lastMillis - now;
  } else {
    delayTime = now - lastMillis;
  }
  if (delayTime > sampleRate) {
    Serial.println("calculation took too long, sample rate not met");
  } else {
    //dbgMemInfo();
    //delay(sampleRate-(millis()-now));
    delay(sampleRate - delayTime);
  }
  //lastMillis = now;
  //
  ////    analogReadTimer.start();
  //Serial.println("Here8");

}
//https://www.codeproject.com/Articles/794047/Finite-State-Menu
void menuloop() {
  //if((currentM) != (previousM))
  // {
  display.clearDisplay();
  if (currentM->up != 0) {
    display.println(currentM->up->name);
  } else {
    display.println("--------");
  }
  display.print('>');
  display.println(currentM->name);
  if (currentM->down != 0) {
    display.println(currentM->down->name);
  } else {
    display.println("--------");
  }
  // (previousM) = (currentM) ;
  // }
  if (currentM->DoWork != 0) {
    currentM->DoWork();
  }
  //  display.setCursor(0,0);
  // display.display();
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  if (buttonState) {
    if (buttonState & 0x1) {
      //display.println("UP");
      //display.println(currentM->up->name);
      menuUp(&currentM);
    }
    if (buttonState & 0x2) {
      //display.println("SELECT");
      //display.println(currentM->next->name);
      menuNext(&currentM);
    }
    if (buttonState & 0x4) {
      //display.print("MENU");
      menuPrev(&currentM);
    }
    if (buttonState & 0x8) {
      //display.print("DOWN");
      //display.println(currentM->down->name);
      menuDown(&currentM);
    }
    buttonState = 0;
  }

  display.setCursor(0, 0);
  // analogReadTimer.stop();
  display.display();
  //analogReadTimer.start();
  
  delay(400);
}
void BuildMenu(struct menuLevel *currentNode, const char * name, void (*DoWork)(void) , struct menuLevel *prevNode, struct menuLevel *nextNode, struct menuLevel *upNode, struct menuLevel *downNode)
{
  strncpy(currentNode->name, name, sizeof(currentNode->name));
  currentNode->prev = prevNode;
  currentNode->next = nextNode;
  currentNode->up = upNode;
  currentNode->down = downNode;
  currentNode->DoWork = DoWork;
}
void menuNext(struct menuLevel **currentNode) //Correct
{
  if ( (*currentNode) ->next != 0)
    (*currentNode) = (*currentNode)->next;
}
void menuUp(struct menuLevel **currentNode) //Correct
{
  if ( (*currentNode) ->up != 0)
    (*currentNode) = (*currentNode)->up;
}
void menuDown(struct menuLevel **currentNode) //Correct
{
  if ( (*currentNode) ->down != 0)
    (*currentNode) = (*currentNode)->down;
}
void menuPrev(struct menuLevel **currentNode) //Correct
{
  if ( (*currentNode) ->prev != 0)
    (*currentNode) = (*currentNode)->prev;
}

void menuScreenSaver() {
  display.clearDisplay();
}

void menuCoeffEdit() {

}

void menuValueEdit() {

}

void menuEntryAdd() {

}
void menuEntrySubtract() {

}
void menuEntryRemove() {

}
void menuValueSave() {

}
void menuCalibrate() {
  display.setRotation(3);
  display.println(adc_value_buffer2[0]);
  display.println(adc_value_buffer2[1]);
  display.println(adc_value_buffer2[2]);
  display.println(adc_value_buffer2[3]);
  display.print((*dac_value_normalized_I));
  display.println("mA   ");
  display.print((*dac_value_normalized_V));
  display.println("mV");
  display.println("--------");
  display.print(adc_value_calibrated[0]);
  if ((*ADC0_config) == 0) {
    display.println("ADC");
  }
  if ((*ADC0_config) == 1) {
    display.println("V");
  }
  if ((*ADC0_config) == 2) {
    display.println("mA   ");
  }
  if ((*ADC0_config) == 3) {
    display.println("Hz");
  }
  display.print(adc_value_calibrated[1]);
  if ((*ADC1_config) == 0) {
    display.println("ADC");
  }
  if ((*ADC1_config) == 1) {
    display.println("V");
  }
  if ((*ADC1_config) == 2) {
    display.println("mA   ");
  }
  if ((*ADC1_config) == 3) {
    display.println("Hz");
  }  display.print(adc_value_calibrated[2]);
  if ((*ADC2_config) == 0) {
    display.println("ADC");
  }
  if ((*ADC2_config) == 1) {
    display.println("V");
  }
  if ((*ADC2_config) == 2) {
    display.println("mA   ");
  }
  if ((*ADC2_config) == 3) {
    display.println("Hz");
  }  display.print(adc_value_calibrated[3]);
  if ((*ADC3_config) == 0) {
    display.println("ADC");
  }
  if ((*ADC3_config) == 1) {
    display.println("V");
  }
  if ((*ADC3_config) == 2) {
    display.println("mA   ");
  }
  if ((*ADC3_config) == 3) {
    display.println("Hz");
  }
}
//   dataString += String(3.6/(((float)current_counter_period_fall/1000000)+(36*(float)overflow_counter_fall)),14) + "," + current_counter_fall_new+ "," + current_counter_fall_old+ "," + String( (((float)current_counter_period_fall/1000000)+(36*(float)overflow_counter_fall)),14)+ "," +current_counter_period_fall+ "," +overflow_counter_fall+ ","+ flag_counter+"\r\n";

//(0,calibrate | 1 //fresh_air_fan();2  //valve_flow_rate();3  //pump_fan_flow_rate(); //4 fan_flow_meter)
void menu_fan_flow_meter() {
  display.setRotation(3);
  display.print((*dac_value_normalized_V) * 10, 2);
  display.println(unit_label.var.V_unit_label);
  display.print(((*dac_value_normalized_I) * 16) + 4, 2);
  display.println(unit_label.var.I_unit_label);
  display.print(((*C_dac_value_results) * (*C_disp_coeff)) + (*C_disp_offset), 2);
  display.println((unit_label.var.C_unit_label));
  
  //display.println((*C_unit_label));
  //display.print(((*D_adc_value_normalized) * (*D_disp_coeff)) + (*D_disp_offset));
  //display.println((unit_label.var.D_unit_label));
  //display.print(((3600000 / (*imp_kWh)) / (*D_adc_value_normalized)), 2);
  // display.print(((3600*(*WHpP))/(*D_adc_value_normalized)*(*D_disp_coeff))+(*D_disp_offset));
  display.print(D_Conversion(D_adc_value_normalized));
  display.println((unit_label.var.D_unit_label));
  display.print(((*M_adc_value_normalized) * (*M_disp_coeff)) + (*M_disp_offset), 2);
  display.println((unit_label.var.M_unit_label));
  //display.print((1/((float)current_counter_period_fall/1000000)+(36*(float)overflow_counter_fall)),2);
  //display.print(1/(adc_value_normalized[3]));
  //display.println("Hz");
  display.print(((*C_adc_value_normalized)*(*C_disp_coeff))+(*C_disp_offset));
  display.println(unit_label.var.C_unit_label);
  //display.print(((*C_dac_value_results)*(*VOUT_disp_coeff))+(*VOUT_disp_offset));
  //display.println(*V_unit_label);
  //display.println("--------");
  display.print(ADC0_Conversion_Function(&(adc_value_normalized[0])));
  display.println(unit_label.var.ADC0_unit_label);
  display.print(ADC1_Conversion_Function(&(adc_value_normalized[1])));
  display.println(unit_label.var.ADC1_unit_label);
  display.print(ADC2_Conversion_Function(&(adc_value_normalized[2])));
  display.println(unit_label.var.ADC2_unit_label);
  display.print(ADC3_Conversion_Function(&(adc_value_normalized[3])));
  display.println(unit_label.var.ADC3_unit_label);
  //change this to use U54-U566
  display.print(adc_value_calibrated[0]);
  if ((*ADC0_config) == 0) {
    display.println("ADC");
  }
  if ((*ADC0_config) == 1) {
    display.println("V");
  }
  if ((*ADC0_config) == 2) {
    display.println("mA   ");
  }
  if ((*ADC0_config) == 3) {
    display.println("Hz");
  }
  if ((*ADC0_config) == 4) {
    display.println("Sec.");
  }
  display.print(adc_value_calibrated[1]);
  if ((*ADC1_config) == 0) {
    display.println("ADC");
  }
  if ((*ADC1_config) == 1) {
    display.println("V");
  }
  if ((*ADC1_config) == 2) {
    display.println("mA   ");
  }
  if ((*ADC1_config) == 3) {
    display.println("Hz");
  }
  if ((*ADC1_config) == 4) {
    display.println("Sec.");
  }
  display.print(adc_value_calibrated[2]);
  if ((*ADC2_config) == 0) {
    display.println("ADC");
  }
  if ((*ADC2_config) == 1) {
    display.println("V");
  }
  if ((*ADC2_config) == 2) {
    display.println("mA   ");
  }
  if ((*ADC2_config) == 3) {
    display.println("Hz");
  }
  if ((*ADC2_config) == 4) {
    display.println("Sec.");
  }
  display.print(adc_value_calibrated[3]);
  if ((*ADC3_config) == 0) {
    display.println("ADC");
  }
  if ((*ADC3_config) == 1) {
    display.println("V");
  }
  if ((*ADC3_config) == 2) {
    display.println("mA   ");
  }
  if ((*ADC3_config) == 3) {
    display.println("Hz");
  }
  if ((*ADC3_config) == 4) {
    display.println("Sec.");
  }


  /*
    display.clearDisplay();
    display.setTextSize(3);
    display.setRotation(3);
    display.setCursor(0,0);
    display.print(pump_fan_flow_rate_result);
    display.setTextSize(2);
    display.println("mA");
    display.setTextSize(3);
    display.print(valve_flowRate_result);
    display.setTextSize(2);
    display.println("V");
  */
   if(bleuart.read() == '1'){
  bleuart.write("Value ADC0: %f",ADC0_Conversion_Function(&(adc_value_normalized[0])));
}

}
//(0,calibrate | 1 //fresh_air_fan();2  //valve_flow_rate();3  //pump_fan_flow_rate(); //4 fan_flow_meter)
void menu_fresh_air_fan() {

  // dumpMemory();

  display.setRotation(3);
  display.print((*dac_value_normalized_V) * 10);
  display.println("V");
  display.print(((*dac_value_normalized_I) * 16) + 4);
  display.println("mA");
  display.print(((*C_adc_value_normalized) * (*C_disp_coeff)) + (*C_disp_offset));
  display.println((unit_label.var.C_unit_label));
  display.print(((*C_dac_value_results) * (*VOUT_disp_coeff)) + (*VOUT_disp_offset));
  display.println(unit_label.var.V_unit_label);
  display.println("--------");
  display.print(adc_value_calibrated[0]);
  if ((*ADC0_config) == 0) {
    display.println("ADC");
  }
  if ((*ADC0_config) == 1) {
    display.println("V");
  }
  if ((*ADC0_config) == 2) {
    display.println("mA   ");
  }
  if ((*ADC0_config) == 3) {
    display.println("Hz");
  }
  if ((*ADC0_config) == 4) {
    display.println("Sec.");
  }
  display.print(adc_value_calibrated[1]);
  if ((*ADC1_config) == 0) {
    display.println("ADC");
  }
  if ((*ADC1_config) == 1) {
    display.println("V");
  }
  if ((*ADC1_config) == 2) {
    display.println("mA   ");
  }
  if ((*ADC1_config) == 3) {
    display.println("Hz");
  }
  if ((*ADC0_config) == 4) {
    display.println("Sec.");
  }
  display.print(adc_value_calibrated[2]);
  if ((*ADC2_config) == 0) {
    display.println("ADC");
  }
  if ((*ADC2_config) == 1) {
    display.println("V");
  }
  if ((*ADC2_config) == 2) {
    display.println("mA   ");
  }
  if ((*ADC2_config) == 3) {
    display.println("Hz");
  }
  if ((*ADC0_config) == 4) {
    display.println("Sec.");
  }
  display.print(adc_value_calibrated[3]);
  if ((*ADC3_config) == 0) {
    display.println("ADC");
  }
  if ((*ADC3_config) == 1) {
    display.println("V");
  }
  if ((*ADC3_config) == 2) {
    display.println("mA   ");
  }
  if ((*ADC3_config) == 3) {
    display.println("Hz");
  }
  if ((*ADC0_config) == 4) {
    display.println("Sec.");
  }
  /*
     display.clearDisplay();
     display.setTextSize(3);
     display.setRotation(3);
     display.setCursor(0,0);
     display.print(pump_fan_flow_rate_result);
     display.setTextSize(2);
     display.println("mA");
     display.setTextSize(3);
     display.print(valve_flowRate_result);
     display.setTextSize(2);
     display.println("V");
  */
  //displaytest.print("DEADBEEF");

  // displaytest.display();
  // delay(10);
  // yield();
  // display.setCursor(0, 0);
  // display.display();
  // delay(400);

}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}


// used in
int EMA_function(float32_t alpha, uint32_t latest, uint32_t stored) {
  // Serial.println(alpha*latest);
  // Serial.println((1-alpha)*stored);
  // Serial.println(round(alpha*latest) + round((1-alpha)*stored));
  return round(alpha * latest) + round((1 - alpha) * stored);
}

void selfCalibrate() {
  //run least squares lienar regression each measurement, take the existing coefficients, output the error value and minimise the error based on the input from serial
  //so, set psu to 10v, type 10.0 into the console, and watch the linear regression error decrease. when you enter in a n
  /*
     The idea is to assume a linear function, when the error increases over a set threshold, we rerun the LMS function to find a new set of linear coefficients for the segment
     http://math.mit.edu/~gs/linearalgebra/ila0403.pdf



     //C1 : ±#.#######e±##

  */

  //float32_t calibration_buff_X[BUFSIZE];
  //float32_t calibration_buff_Y[BUFSIZE];
  if (stringComplete) {
    // Serial.println(inputString);
    // clear the string:
    inputString.trim();
    Serial.println( "selfCalibrate");
    if (inputString != "") {
      std::size_t token_index = inputString.indexOf(':');
      char coeff_type = inputString[0];
      // const char *coeff_val_str_tmp = conf_line.substring((token_index+1),conf_line.length()).c_str();
      String conf_val = inputString.substring((token_index + 1), inputString.length());
      conf_val.trim();
      const char *coeff_val_str_tmp = conf_val.c_str();
      char *end;
      double z = strtod(coeff_val_str_tmp, &end);

      delay(1);
      if (end == coeff_val_str_tmp) {
        Serial.println("Conversion value error ");
        Serial.println(*coeff_val_str_tmp);
        Serial.println(inputString.substring((token_index + 1), inputString.length()));
      } else {
        //   Serial.println(conf_line.substring(1,(token_index)));
        int y = atoi(inputString.substring(1, (token_index)).c_str());
        Serial.print(coeff_type);
        Serial.print(y);
        Serial.print(":");
        Serial.println(z, 16);
        if ((BUFSIZE / 2) > y) {
          //A is analog channel 0, D is analog channel 3
          if (coeff_type == 'A') {
            calibration_buff_Y[y] = (float32_t)z;
            calibration_buff_X[(y * 2)] = 1.0;
            calibration_buff_X[((y * 2) + 1)] = adc_value_buffer2[0];
          }

          if (coeff_type == 'B')
          {
            calibration_buff_Y[y] = (float32_t)z;
            calibration_buff_X[(y * 2)] = 1.0;
            calibration_buff_X[((y * 2) + 1)] = adc_value_buffer2[1];
          }

          if (coeff_type == 'C')
          {
            calibration_buff_Y[y] = (float32_t)z;
            calibration_buff_X[(y * 2)] = 1.0;
            calibration_buff_X[((y * 2) + 1)] = adc_value_buffer2[2];
          }

          if (coeff_type == 'D')
          {
            calibration_buff_Y[y] = (float32_t)z;
            calibration_buff_X[(y * 2)] = 1.0;
            calibration_buff_X[((y * 2) + 1)] = adc_value_buffer2[3];
          }
          if (coeff_type == 'E')
          {
            process_least_squares_buffer(&calibration_buff_X[0], &calibration_buff_Y[0]);
          }
          if (coeff_type == 'F')
          {

          }
        } else {
          Serial.print((BUFSIZE / 2));
          Serial.println(":exceeded");
        }
      }
      //pump_fan_flow_rate_result = strtod(inputString.c_str(), NULL);
      // calibration_value =(float32_t) strtod(inputString.c_str(), NULL);
      inputString = "";
      stringComplete = false;

    }
  }
}


//
//https://medium.com/@andrew.chamberlain/the-linear-algebra-view-of-least-squares-regression-f67044b7f39b
//modified from linear matrix example, for linear approximation
//https://www.keil.com/pack/doc/CMSIS/DSP/html/arm_matrix_example_f32_8c-example.html
void process_least_squares_buffer(float32_t* A_f32, float32_t* B_f32) {
  /* Transpose of A Buffer */
  float32_t AT_f32[8];
  /* (Transpose of A * A) Buffer */
  float32_t ATMA_f32[4];
  /* Inverse(Transpose of A * A)  Buffer */
  float32_t ATMAI_f32[4];
  float32_t ATMAIMAT_f32[8];
  /* Test Output Buffer */
  float32_t X_f32[2];
  //float32_t*( A_f32) = &(ADC0_buff[0]);
  //float32_t* B_f32 =  &(ADC1_buff[0]);
  arm_matrix_instance_f32 A;      /* Matrix A Instance */
  arm_matrix_instance_f32 AT;     /* Matrix AT(A transpose) instance */
  arm_matrix_instance_f32 ATMA;   /* Matrix ATMA( AT multiply with A) instance */
  arm_matrix_instance_f32 ATMAI;  /* Matrix ATMAI(Inverse of ATMA) instance */
  arm_matrix_instance_f32 ATMAIMAT;  /* Matrix ATMAIMAT(ATMAI multiply with AT) instance */
  arm_matrix_instance_f32 B;      /* Matrix B instance */
  arm_matrix_instance_f32 X;      /* Matrix X(Unknown Matrix) instance */
  uint32_t srcRows, srcColumns;  /* Temporary variables */
  float32_t snr;
  arm_status status;
  /* Initialise A Matrix Instance with numRows, numCols and data array(A_f32) */
  srcRows = 4;
  srcColumns = 2;
  arm_mat_init_f32(&A, srcRows, srcColumns, (float32_t *)A_f32);
  //  Serial.println("A_f32");
  //  for(int i=0; i<8 ; i++)
  //  {
  //
  //            Serial.println(A_f32[i],20);
  //
  //  }
  /* Initialise Matrix Instance AT with numRows, numCols and data array(AT_f32) */
  srcRows = 2;
  srcColumns = 4;
  arm_mat_init_f32(&AT, srcRows, srcColumns, AT_f32);
  /* calculation of A transpose */
  status = arm_mat_trans_f32(&A, &AT);
  //      Serial.println("AT_f32[i]");

  //  for(int i=0; i<8 ; i++)
  //  {
  //            Serial.println(AT_f32[i],20);
  //
  //  }
  /* Initialise ATMA Matrix Instance with numRows, numCols and data array(ATMA_f32) */
  srcRows = 2;
  srcColumns = 2;
  arm_mat_init_f32(&ATMA, srcRows, srcColumns, ATMA_f32);
  /* calculation of AT Multiply with A */
  status = arm_mat_mult_f32(&AT, &A, &ATMA);
  //  Serial.println("ATMA_f32[i]");
  //for(int i=0; i<4 ; i++)
  //{
  //          Serial.println(ATMA_f32[i],20);

  //}
  /* Initialise ATMAI Matrix Instance with numRows, numCols and data array(ATMAI_f32) */
  srcRows = 2;
  srcColumns = 2;
  arm_mat_init_f32(&ATMAI, srcRows, srcColumns, ATMAI_f32);
  /* calculation of Inverse((Transpose(A) * A) */
  status = arm_mat_inverse_f32(&ATMA, &ATMAI);
  //  Serial.println("ATMAI_f32[i]");
  //  for(int i=0; i<4 ; i++)
  //  {
  //            Serial.println(ATMAI_f32[i],20);
  //
  //  }
  srcRows = 2;
  srcColumns = 4;
  arm_mat_init_f32(&ATMAIMAT, srcRows, srcColumns, ATMAIMAT_f32);
  /* calculation of (Inverse((Transpose(A) * A)) *  Transpose(A)) */
  status = arm_mat_mult_f32(&ATMAI, &AT, &ATMAIMAT);
  //  Serial.println("ATMAIMAT_f32[i]");
  //  for(int i=0; i<8 ; i++)
  //  {
  //            Serial.println(ATMAIMAT_f32[i],20);
  //
  //  }
  /* Initialise B Matrix Instance with numRows, numCols and data array(B_f32) */
  srcRows = 4;
  srcColumns = 1;
  arm_mat_init_f32(&B, srcRows, srcColumns, (float32_t *)B_f32);
  //        Serial.println("B_f32[i]");
  //  for(int i=0; i<4 ; i++)
  //  {
  //            Serial.println(B_f32[i],20);
  //
  //  }
  /* Initialise X Matrix Instance with numRows, numCols and data array(X_f32) */
  srcRows = 2;
  srcColumns = 1;
  arm_mat_init_f32(&X, srcRows, srcColumns, X_f32);
  /* calculation ((Inverse((Transpose(A) * A)) *  Transpose(A)) * B) */
  status = arm_mat_mult_f32(&ATMAIMAT, &B, &X);
  Serial.println("X_f32[i]");
  for (int i = 0; i < 2 ; i++)
  {
    Serial.println(X_f32[i], 20);

  }
  /* Comparison of reference with test output */
  //snr = arm_snr_f32((float32_t *)xRef_f32, X_f32, 4);
  /*------------------------------------------------------------------------------
               Initialise status depending on SNR calculations
    ------------------------------------------------------------------------------*/
  //if ( snr > SNR_THRESHOLD)
  //{
  //  status = ARM_MATH_SUCCESS;
  // }
  // else
  // {
  //   status = ARM_MATH_TEST_FAILURE;
  //  }

  //  for (i=0 ; i<(BUFSIZE) ; i=i+BLOCK_SIZE)
  // {

  //right_in_sample =;
  //left_in_sample = *rxbuf++;
  // Serial.print( "calibration_value: ");
  //   Serial.println( calibration_value);
  //  Serial.print( "vTST_buff[]: ");
  //   Serial.println(vTST_buff[i]);

  //  refnoise = (float32_t)(calibration_value);
  // signoise = (float32_t)( vTST_buff[i]);
  /*
     Parameters
    [in] S points to an instance of the floating-point LMS filter structure
    [in]  pSrc  points to the block of input data
    [in]  pRef  points to the block of reference data
    [out] pOut  points to the block of output data
    [out] pErr  points to the block of error data
    [in]  blockSize number of samples to process
   * */



  //arm_lms_f32(&S, &signoise , &refnoise, &yout, &error, BLOCK_SIZE);

  // Serial.println((int16_t)(yout));

  //   Serial.println((error));
  //}
}



void calibrate() {
  //should read 1
  //vTST =  (*vTST_calib)+(*vTST_calib_offset);
  //deltaP =  (* deltaP_calib)+(* deltaP_calib);
  //power =  (* power_calib)+(* power_calib);
  //(*M_adc_value_normalized) =  (*freq_calib)+(*freq_calib);
  //vTST = (*adWeight)*analogRead(ADC0_PIN) + (1.0-(*adWeight))*vTST ;
  //deltaP = (*adWeight)*analogRead(ADC1_PIN) + (1.0-(*adWeight))*deltaP;
  //power = (*adWeight)*analogRead(ADC3_PIN) + (1.0-(*adWeight))*power;
  //(*M_adc_value_normalized) = (*adWeight)*analogRead(ADC2_PIN) + (1.0-(*adWeight))*(*M_adc_value_normalized);
  //   ADC1_buff[BUFSIZE];
  //        ADC2_buff[BUFSIZE];
  //        ADC3_buff[BUFSIZE];
  //        DAC0_buff[BUFSIZE];
  //        DAC1_buff[BUFSIZE];
  //  analogReference(AR_DEFAULT);
  //  analogOversampling(256);
  //
  //  analogReadResolution(14);
  //  delay(1);
  //  vTST = analogRead(ADC0_PIN);
  //  delay(1);
  //  deltaP = analogRead(ADC1_PIN) ;
  //  delay(1);
  //  power = analogRead(ADC3_PIN) ;
  //  delay(1);
  //  (*M_adc_value_normalized) = analogRead(ADC2_PIN);
  delay(1);
  if (stringComplete) {
    Serial.println(inputString);
    // clear the string:

    dac_value_temp[0] = (uint32_t)strtod(inputString.c_str(), NULL);
    dac_value_temp[1] = dac_value_temp[0];
    inputString = "";
    stringComplete = false;
  }

}

void dumpMemory() {
  //troubleshooting function
  for (int i = 0; i < COEFF_NUM; i++) {
    Serial.println(coeff_buff.c_list[i], 16);
  }

  for (int i = 0; i < COEFF_NUM; i++) {
    Serial.println(coeff_buff.m_list[i], 16);
  }
  for (int i = 0; i < LIMIT_NUM; i++) {
    Serial.println(coeff_buff.limits[i], 16);
  }
  /*
    Serial.print("valve_limit_upper address  = ");
    Serial.println((unsigned int)&valve_limit_upper);

    Serial.print("valve_limit_upper address  = ");
    Serial.println((unsigned int)valve_limit_upper);

    Serial.print("valve_limit_upper content  = ");
    Serial.println(*valve_limit_upper);
    Serial.print("&(coeff_buff.limits[0]  = ");
    Serial.println((unsigned int)&(coeff_buff.limits[0]));

  */

  //global config storage, gets rebuilt based on SD card CONFIG.TXT, manual data entry, or internal storage
  // coefficients coeff_buff;
  // disables normal output routine, and sets outputs to known configuration so coefficients can be adjusted
  Serial.print("calibrateEnabled = ");
  Serial.println(calibrateEnabled);


  /*
    //Valve position input (0-10v signal)
    Serial.print("vTST = ");
    Serial.println(vTST);
    Serial.print("vTST mV = ");
    Serial.println(vTST * ADC_MV_PER_LSB);
    //differential pressure sensor input 4-20mA signal
    Serial.print("deltaP = ");
    Serial.println(deltaP);
    Serial.print("deltaP mA= ");
    Serial.println(deltaP * ADC_MV_PER_LSB);
    //motor power input (4-20mA or 0-10v)
    Serial.print("power = ");
    Serial.println(power);
    Serial.print("power mA= ");
    Serial.println(power);
    //intermediary result
    Serial.print("powerS = ");
    Serial.println(powerS);
    Serial.print("powerR = ");
    Serial.println(powerR);
    //motor frequency input (4-20mA or 0-10v)
    Serial.print("freq = ");
    Serial.println(freq);
    Serial.print("freq mA= ");
    Serial.println(freq);
    //dot product results
    Serial.print("dotResult = ");
    Serial.println(dotResult);
    //sqrt results
    Serial.print("sqrtResult = ");
    Serial.println(sqrtResult);
    //eff_m
    Serial.print("eff_m = ");
    Serial.println(eff_m);
    //eff_dev
    Serial.print("eff_dev = ");
    Serial.println(eff_dev);


    //valve upper limit
    Serial.print("valve_limit_upper = ");
    Serial.println(*valve_limit_upper);
    //valve lower limit
    Serial.print("valve_limit_lower = ");
    Serial.println(*valve_limit_lower);
    //motor frequency upper limit
    Serial.print("freq_limit_upper = ");
    Serial.println(*freq_limit_upper);
    //motor power lower limit
    Serial.print("power_limit_lower = ");
    Serial.println(*power_limit_lower);

    // * U10 calibration coeff, vTST
    Serial.print("vTST_calib = ");
    Serial.println(*vTST_calib);
    // * Ull calibration coeff, deltaP
    Serial.print("deltaP_calib = ");
    Serial.println(*deltaP_calib);
    // * U12 calibration coeff, power
    Serial.print("power_calib = ");
    Serial.println(*power_calib);
    // * U13 calibration coeff, freq
    Serial.print("freq_calib = ");
    Serial.println(*freq_calib);
    // * U14 calibration coeff, pump_fan_flow_rate_result
    Serial.print("VOUT_calib = ");
    Serial.println(*VOUT_calib);
    // * U15 calibration coeff, valve_flowRate_result
    Serial.print("IOUT_calib = ");
    Serial.println(*IOUT_calib);
    // * U16 calibration coeff, vTST
    Serial.print("vTST_calib_offset = ");
    Serial.println(*vTST_calib_offset);
    // * Ul7 calibration coeff, deltaP
    Serial.print("deltaP_calib_offset = ");
    Serial.println(*deltaP_calib_offset);
    // * U18 calibration coeff, power
    Serial.print("power_calib_offset = ");
    Serial.println(*power_calib_offset);
    // * U19 calibration coeff, freq
    Serial.print("freq_calib_offset = ");
    Serial.println(*freq_calib_offset);
    // * U20 calibration coeff, pump_fan_flow_rate_result
    Serial.print("VOUT_calib_offset = ");
    Serial.println(*VOUT_calib_offset);
    // * U21 calibration coeff, valve_flowRate_result
  */
  Serial.print("IOUT_calib_offset = ");
  Serial.println(*IOUT_calib_offset);
  //  Serial.print("pump_fan_flow_rate_result = ");
  //  Serial.println(pump_fan_flow_rate_result);
  //  Serial.print("valve_flowRate_result = ");
  //  Serial.println(valve_flowRate_result);
  Serial.print("overflow_counter = ");
  Serial.println(overflow_counter);
  Serial.print("current_counter_fall_new =");
  Serial.println(current_counter_fall_new );
  Serial.print("current_counter_fall_old = ");
  Serial.println(current_counter_fall_old);
  Serial.print("current_counter_period_fall =");
  Serial.println(current_counter_period_fall);
  Serial.print("overflow_counter_fall = ");
  Serial.println(overflow_counter_fall);
  Serial.print("current_counter_rise_new = ");
  Serial.println(current_counter_rise_new);
  Serial.print("current_counter_rise_old = ");
  Serial.println(current_counter_rise_old);
  Serial.print("current_counter_period_rise = ");
  Serial.println(current_counter_period_rise);
  Serial.print("overflow_counter_rise = ");
  Serial.println(overflow_counter_rise);
}

void update_DAC() {
  // DAC increments (0..4095) --> output voltage (0..Vref)
  //0 V to 4095/4096 * VREF, with VREF ==3.3v
  //   U28 DAC0_PIN 0:Not used, 1:C Output, 2:M Output, 3:D Output, 4:R Output


  //GPM_Vout 0-10v
  // dac_value_normalized_V =   4+((dac_value_temp[0]/coeff_buff.c_list[3])*16);

  if (digitalRead(dacChipSelect) == LOW) {
    digitalWrite(dacChipSelect, HIGH);
     Serial.println("DAC left on in update_DAC");

  } else if (digitalRead(chipSelect) == LOW) {
    digitalWrite(chipSelect, HIGH);
        Serial.println("SD left on in update_DAC");
  } else {
    if (((int)*algorithmEnable == 0) | ((int)*algorithmEnable == 6)) {
      (*dac_value_normalized_V) = (float32_t)(dac_value_temp[0] / 4095);
      (*dac_value_normalized_I) = (float32_t)(dac_value_temp[1] / 4095);
      DAC.analogWrite(0, dac_value_temp[0] );
      DAC.analogWrite(1, dac_value_temp[1] );
    } else {
      if ((*dac_value_normalized_V) > 1) {
        (*dac_value_normalized_V) = 1;
      }

      dac_value_calibrated[0] = (( (*dac_value_normalized_V) * (* VOUT_calib)) + (* VOUT_calib_offset));
      // Serial.print("dac_value_calibrated[0]");
      //Serial.println(dac_value_calibrated[0]);
      if ((dac_value_calibrated[0]) < 0) {
        (*dac_value_normalized_V) = 0;
        (dac_value_calibrated[0]) = 0;
      }
      //Serial.print("dac_value_calibrated[0]");
      //Serial.println(dac_value_calibrated[0]);
      DAC.analogWrite(0, (int)dac_value_calibrated[0] ); //((pump_fan_flow_rate_result * (*VOUT_calib)) + (*VOUT_calib_offset)));
      //VLVctrl 4-20mA out
      //normalize it, then convert to 4-20mA
      //(*dac_value_normalized_I) =   4+((valve_flowRate_result/(*Qmax))*16);
      if ((*dac_value_normalized_I) > 1) {
        (*dac_value_normalized_I) = 1;
      }

      dac_value_calibrated[1] = (( (*dac_value_normalized_I) * (* IOUT_calib)) + (* IOUT_calib_offset));
      //Serial.print("dac_value_calibrated[1]");
      //Serial.println(dac_value_calibrated[1]);
      if (( dac_value_calibrated[1]) < 0) {
        (*dac_value_normalized_I) = 0;
        ( dac_value_calibrated[1]) = 0;
      }
      DAC.analogWrite(1, (int) dac_value_calibrated[1]);
    }
    //Serial.print("dac_value_calibrated[1]");
    //Serial.println(dac_value_calibrated[1]);
  }
}

void configureADC() {
  analogReference(AR_DEFAULT);
  analogOversampling(32);
  analogReadResolution(14);
}

inline void readADC0() {
  // flag_counter++;

  adc_value_temp[0] = analogRead(ADC0_PIN);
}
inline void readADC1() {
  //  flag_counter++;

  adc_value_temp[1] = analogRead(ADC1_PIN);
}
inline void readADC2() {
  //  flag_counter++;

  adc_value_temp[2] = analogRead(ADC2_PIN);
}
inline void readADC3() {
  //   flag_counter++;

  adc_value_temp[3] = analogRead(ADC3_PIN);
}
//readTIMER is in Seconds, and it's the period between falling impusle events. no offset or multiplier should be required
inline void readTIMER0() {
//  adc_value_temp[0] = (current_counter_period_fall / 1000000) + (36 * overflow_counter_fall);
  adc_value_temp[0] = (current_counter_period_fall ) ;
}
inline void readTIMER1() {
//  adc_value_temp[1] = (current_counter_period_fall / 1000000) + (36 * overflow_counter_fall);
  adc_value_temp[1] = (current_counter_period_fall) ;
}
inline void readTIMER2() {
//  adc_value_temp[2] = (current_counter_period_fall / 1000000) + (36 * overflow_counter_fall);
  adc_value_temp[2] = (current_counter_period_fall ) ;
}
inline void readTIMER3() {
  // adc_value_temp[3] = (current_counter_period_fall/1000000)+(36*overflow_counter_fall);
//  adc_value_temp[3] = (current_counter_period_fall) + (36000000 * overflow_counter_fall);
  adc_value_temp[3] = (current_counter_period_fall); 
}
inline bool readDIO0() {
  return digitalRead(ADC0_PIN);
}
inline bool readDIO1() {
  return digitalRead(ADC1_PIN);
}
inline bool readDIO2() {
  return digitalRead(ADC2_PIN);
}
inline bool readDIO3() {
  return digitalRead(ADC3_PIN);
}
void readADC() {
  // flag_counter++;

  //uint32_t
  //Serial.println("readadc");

  if (PIN_A0_Function)(*PIN_A0_Function)();
  if (PIN_A1_Function)(*PIN_A1_Function)();
  if (PIN_A2_Function)(*PIN_A2_Function)();
  if (PIN_A3_Function)(*PIN_A3_Function)();


  for (int i = 0; i < 4; i++) {
    // if(adc_config[i]==3){
    //adc_value_calibrated[i] = EMA_function((*adWeight),((((float)current_counter_period_fall/1000000)+(36*(float)overflow_counter_fall))), adc_value_buffer[i]);
    //  adc_value_temp[i] = EMA_function((*adWeight), ((((float)current_counter_period_fall/1000000)+(36*(float)overflow_counter_fall))) , adc_value_buffer[i]);
    //adc_value_calibrated[i] =1/(((float)current_counter_period_fall/1000000)+(36*(float)overflow_counter_fall));
    //adc_value_buffer[i] =1/(((float)current_counter_period_fall/1000000)+(36*(float)overflow_counter_fall));
    //Serial.println(adc_value_calibrated[i]);
    //}else if(adc_config[i]==4){
    //adc_value_buffer[i] =(((float)current_counter_period_fall/1000000)+(36*(float)overflow_counter_fall));
    //  adc_value_temp[i] = EMA_function((*adWeight), ((((float)current_counter_period_fall/1000000)+(36*(float)overflow_counter_fall))) , adc_value_buffer[i]);
    // }else{


    // }
    adc_value_buffer[i] = EMA_function((*adWeight), adc_value_temp[i] , adc_value_buffer[i]);
    adc_value_buffer2[i] = EMA_function((*adWeight), adc_value_buffer[i] ,  adc_value_buffer2[i]);
    adc_value_calibrated[i] =  (((adc_value_buffer2[i]) * (adc_calib[i])) + (adc_calib_offset[i]));
    // Serial.println(adc_value_buffer[i]);
    //         Serial.println(adc_value_buffer2[i]);
    //              Serial.println(adc_value_calibrated[i]);
    //  Serial.println(adc_value_temp[i]);

  }
}

// calibrated does the same thing, only useful if the values are normalized
void normalize_ADC() {

  for (int i = 0; i < 4; i++) {
    // adc_value_buffer[i] = EMA_function((*adWeight),adc_value_temp[i] , adc_value_buffer[i]);
    //  adc_value_buffer2[i] = EMA_function((*adWeight), adc_value_buffer[i] ,  adc_value_buffer2[i]);
    //  adc_value_calibrated[i] =  (((adc_value_buffer2[i]) * (adc_calib[i])) + (adc_calib_offset[i]));
    //  0: disable, 1: 0-10V in, 2: 4-20mA in, 3: Pulse in, was Valve PIN
    if (adc_config[i] == 0) {
      adc_value_normalized[i] = adc_value_calibrated[i];
    }
    if (adc_config[i] == 1) {
      adc_value_normalized[i] = adc_value_calibrated[i] / 10 ;
    }
    if (adc_config[i] == 2) {
      adc_value_normalized[i] = (adc_value_calibrated[i] - 4) / 16;
    }
    if (adc_config[i] == 3) {

      adc_value_normalized[i] = 1 / (adc_value_calibrated[i]);
    }
    if (adc_config[i] == 4) {

      adc_value_normalized[i] = adc_value_calibrated[i];
    }
  }
}



void power_series(float32_t * _output,  uint32_t _list_length, float32_t  _input, uint32_t _start) {
  //arm_power_f32()
  //    Serial.println("Here3");
  //   Serial.println("_input _input."+String(_input));

  if (_input < 0) {
    _input = 0;
  }
  for (uint32_t x = 0;  x < _list_length; x++) {
    // Serial.println("Here4");
    float32_t temp = pow(_input, (x + _start));
    _output[x] = temp;
    //   Serial.println("iter"+String((x+_start)));

    //    Serial.println("power series."+String(temp));
    //   Serial.println("power series saved."+String(_output[x]));

  }
}
/*
   hard coded fresh_air_fan, needs to have an SD card set as follows in unit 2
   U22 ADC0_PIN , 2: 4-20mA in          frequency
   U23 ADC1_PIN  2: 4-20mA in,          head
   U24 ADC2_PIN 0: disable,
   U25 ADC3_PIN    3:
   U26 ADC_Samples_Per_Second 10
   U27 ADC_Settling_Percent  0.06;
   U28 ADC0_PIN 0: discontunity
   U29 ADC1_PIN 0: discontunity
   U30 ADC2_PIN 0: discontunity
   U31 ADC3_PIN 0: discontunity
   U32 discontinuity  C, 0:Not used, 1: applied to ADC0_PIN, 2: applied to ADC1_PIN, 3: applied to ADC2_PIN, 4: applied to ADC3_PIN
   U33 discontinuity  M, 0:Not used, 1: applied to ADC0_PIN, 2: applied to ADC1_PIN, 3: applied to ADC2_PIN, 4: applied to ADC3_PIN
   U34 discontinuity  D, 0:Not used, 1: applied to ADC0_PIN, 2: applied to ADC1_PIN, 3: applied to ADC2_PIN, 4: applied to ADC3_PIN
   U35 discontinuity value C, 0: Not used, anything else, Coeff list lower applied to input < discontinuity value and doeff list upper applied to input >= discontinuity value
   U36 discontinuity value M, 0: Not used, anything else, Coeff list lower applied to input < discontinuity value and doeff list upper applied to input >= discontinuity value
   U37 discontinuity value D, 0: Not used, anything else, Coeff list lower applied to input < discontinuity value and doeff list upper applied to input >= discontinuity value
*/
void fresh_air_fan() {
  // Serial.println("fresh_air_fan initialized.");
  //input is 4-20mA, will be set in config file
  //using C for coefficients c_num[0]=a1,c_num[1]= e1 and c_num[2]=kWRange
  //using channel ADC0, set as 4-20mA
  //Q=a1*W^e1, where W=normalizedmA*kWRange
  //float32_t W = adc_value_normalized[0]*coeff_buff.c_list[2];
  float32_t W = (*C_adc_value_normalized);
  //Serial.print("C_adc_value_normalized :");
  // Serial.println(String(W));
  float32_t temp = pow(W, coeff_buff.c_list[1]);
  // Serial.print("temp :");
  // Serial.println(String(temp));
  //temp = temp * pow((*Qmax),coeff_buff.c_list[1]);
  //4-20mA output, is configured in SD config
  (*C_dac_value_results) = temp * coeff_buff.c_list[0];
  //  Serial.print("C_dac_value_results :");
  // Serial.println(String((*C_dac_value_results)));
  // current_DAC_output =
  // power_series(M_power_series_list, m_num, freq, 0);

}
/*
   hard coded fan flow meter, needs to have an SD card set as follows in unit 1
   U22 ADC0_PIN , 1: 0-10v          frequency
   U23 ADC1_PIN  2: 4-20mA in,          head
   U24 ADC2_PIN 2: 4-20mA in,  n/a
   U25 ADC3_PIN    3: Pulse in
   U26 ADC_Samples_Per_Second 10
   U27 ADC_Settling_Percent  0.06;
   U28 ADC0_PIN 0: discontunity
   U29 ADC1_PIN 0: discontunity
   U30 ADC2_PIN 0: discontunity
   U31 ADC3_PIN 0: discontunity
   U32 discontinuity  C, 0:Not used, 1: applied to ADC0_PIN, 2: applied to ADC1_PIN, 3: applied to ADC2_PIN, 4: applied to ADC3_PIN
   U33 discontinuity  M, 0:Not used, 1: applied to ADC0_PIN, 2: applied to ADC1_PIN, 3: applied to ADC2_PIN, 4: applied to ADC3_PIN
   U34 discontinuity  D, 0:Not used, 1: applied to ADC0_PIN, 2: applied to ADC1_PIN, 3: applied to ADC2_PIN, 4: applied to ADC3_PIN
   U35 discontinuity value C, 0: Not used, anything else, Coeff list lower applied to input < discontinuity value and doeff list upper applied to input >= discontinuity value
   U36 discontinuity value M, 0: Not used, anything else, Coeff list lower applied to input < discontinuity value and doeff list upper applied to input >= discontinuity value
   U37 discontinuity value D, 0: Not used, anything else, Coeff list lower applied to input < discontinuity value and doeff list upper applied to input >= discontinuity value
*/
void fan_flow_meter() {
  //  Serial.println("fan_flow_meter initialized.");
  //input is 4-20mA, will be set in config file
  //using M for coefficients m_num a1-a8, d_num = b1-b8 and c_num[2]=kWRange
  //using channel ADC0, set as 0-10v C
  //not using channel ADC2
  //using channel ADC1, set as  M
  //using channel ADC3, set as pulse D

  //DP sensor range
  //Q=Ws*eff_p/H*4, where W=normalizedmA*kWRange
  //36000000 us compare value, generates EVENTS_COMPARE[0] every 36 seconds
  // watt is (3600000/10)/period = 360000/period


  // Serial.print("(*M_adc_value_normalized): ");
  //Serial.println((*M_adc_value_normalized));
  if (((*C_adc_value_normalized) * (*FrequencyRange)) > 1500) {
    (*FrequencyRange) = 1500;
  }
  if (((*C_adc_value_normalized) * (*FrequencyRange)) > (*discont_value_C)) {

    //build the power series list void power_series(float32_t * _output,  int _list_length, float32_t  _input, uint32_t _start) {
    power_series(C_power_series_list, m_num, ((*C_adc_value_normalized) * (*FrequencyRange)), 0);
    //calculate the dot product
    arm_dot_prod_f32(coeff_buff.c_list, C_power_series_list, c_num, &eff_m);

  } else {
    //build the power series list
    power_series(C_power_series_list, m_num, ((*C_adc_value_normalized) * (*FrequencyRange)), 0);
    //calculate the dot product
    arm_dot_prod_f32(high_c_list, C_power_series_list, c_num, &eff_m);
  }
  // 36000000uS/
  //power = WHpP*3.6/(((float)current_counter_period_fall/1000000)+(36*(float)overflow_counter_fall));
  // for (int i = 0; i < COEFF_NUM; i++) {
  // Serial.println(M_power_series_list[i], 16);
  // }
//   Serial.print("eff_m");
//   Serial.println(eff_m);
//   
//   Serial.print("*D_adc_value_normalized");
//   Serial.println(*D_adc_value_normalized);
//   Serial.print("D_Conversion");
//     if (D_Conversion){Serial.println((*D_Conversion)(D_adc_value_normalized));
//     Serial.println((*D_Conversion)(D_adc_value_normalized));
//     }else {Serial.println("Error");}
//     

   
  if ((*D_adc_value_normalized)) {
    
    // ((3600*(*WHpP))/(*D_adc_value_normalized))
    //(((3600000 / (*imp_kWh)) / (*D_adc_value_normalized)))
     if (D_Conversion){powerR = D_Conversion(D_adc_value_normalized) * eff_m;}
  } else {
    powerR = 0.001;
  }
  if(powerR<0){powerR = 0.001;}
  //  Serial.print("powerR");
  // Serial.println(powerR);
  //  Serial.print("(*M_adc_value_normalized)");
  // Serial.println((*M_adc_value_normalized));
  //  Serial.print("(*DPSensorRange)");
  //  Serial.println((*DPSensorRange));
  if (((*M_adc_value_normalized) * (*DPSensorRange)) > 0) {
    powerS = (pow(((*M_adc_value_normalized) * (*DPSensorRange)), 1.5f));
    //if(powerS){
    //   Serial.print("powerS");
    // Serial.println(powerS,14);
    powerS = (powerR / powerS) / 4;
  }
  else {
    powerS = 0.001;
  }
  if(powerS<0){powerS = 0.001;}
  // Serial.print("powerS");
  // Serial.println(powerS,14);
  if (powerS < (*discont_value_D)) {
    //   Serial.println("Here10");
    power_series(M_power_series_list, m_num, powerS, 0);
    arm_dot_prod_f32(coeff_buff.m_list, M_power_series_list, m_num, &eff_dev);
  } else {
    //  Serial.println("Here20");
    //Serial.print( "powerS:" + String(powerS));
    power_series(M_power_series_list, m_num, powerS, 0);
    arm_dot_prod_f32(high_m_list, M_power_series_list, m_num, &eff_dev);
  }
  //  for (int i = 0; i < COEFF_NUM; i++) {
  //  Serial.println(D_power_series_list[i], 16);
  // }
  //  Serial.print("eff_dev");
  // Serial.println(eff_dev,4);
  if (((*M_adc_value_normalized) * (*DPSensorRange)) > 0) {
    (*C_dac_value_results) = powerR * (eff_dev / ((*M_adc_value_normalized) * (*DPSensorRange))) * 3600;
  } else {
    (*C_dac_value_results) = 0.001;
  }
  // Serial.print("flowRate_result");
  //Serial.println((*C_dac_value_results));
  if((*C_dac_value_results)/(*Qmax) >= 1) 
  {
    (*D_dac_value_results) = 1;
  }
  else if((*C_dac_value_results)/(*Qmax) < 0){
    (*D_dac_value_results) = 0.001;  
  }
  else{
    (*D_dac_value_results) = (*C_dac_value_results) / (*Qmax);
  }
  // Serial.print("normalized_result");
  //Serial.println((*D_dac_value_results));
  //dac_value_temp[0] = (flowRate_result/(*Qmax)) * 4095;
  //  Serial.print("dac_value_temp[0]");
  //  Serial.println(dac_value_temp[0]);
  //if(dac_value_temp[0]>4095){dac_value_temp[0]=4095;}

}

void valve_flow_rate() {
  //    Serial.println("valve_limit_upper."+String(*valve_limit_lower));
  //  Serial.println("dotResult."+String(vTST));
  //  Serial.println("c_num."+String(c_num));
  // float32_t vTST = R_adc_value_normalized
  if ((*R_adc_value_normalized) > (*discont_value_R)) {
    // vTST = (*valve_limit_upper);
    //build the power series list
    power_series(C_power_series_list, c_num, (*discont_value_R), 0);
    //   Serial.println("Here1");

  }
  if ((*R_adc_value_normalized) < (*discont_value_R)) {
    // (*R_adc_value_normalized) = (*valve_limit_lower);
    //build the power series list
    power_series(C_power_series_list, c_num, (*discont_value_R), 0);
    //   Serial.println("Here2");

  }

  //build the power series list
  //power_series(C_power_series_list,c_num, vTST, 0);
  //calculate the dot product
  arm_dot_prod_f32(coeff_buff.c_list , C_power_series_list, c_num, &dotResult);
  //Output Signal flowrate
  arm_sqrt_f32((*C_adc_value_normalized), &sqrtResult);
  //    Serial.println("dotResult."+String(dotResult));
  //    Serial.println("sqrtResult."+String(sqrtResult));

  (*C_dac_value_results) = (dotResult * sqrtResult) / 1; //figure out normalization factor
  //if(valve_flowRate_result>9999){valve_flowRate_result=999;}
}

void pump_fan_flow_rate() {
  //uses M variable, can be setup as any normalized input was frequency
  //uses D variable, was power
  //uses C variable, was deltaP
  if ((*M_adc_value_normalized) > (*discont_value_M)) {

    //build the power series list

    //calculate the dot product
    arm_dot_prod_f32(coeff_buff.m_list, M_power_series_list, m_num, &eff_m);

  } else {
    //build the power series list
    power_series(M_power_series_list, m_num, (*M_adc_value_normalized), m_num);
    //calculate the dot product
    arm_dot_prod_f32(high_m_list, M_power_series_list, m_num, &eff_m);
  }
  powerR = ((*D_adc_value_normalized) * eff_m);
  powerS = powerR / (pow((*C_adc_value_normalized), 1.5f));

  if (powerS < (*discont_value_D)) {
    // Serial.println("Here1");

    power_series(D_power_series_list, d_num, powerS, 0);
    arm_dot_prod_f32(low_d_list, D_power_series_list, d_num, &eff_dev);
  } else {
    //Serial.println("Here2");
    //Serial.print( "powerS:" + String(powerS));

    power_series(D_power_series_list, d_num, powerS, d_num);

    arm_dot_prod_f32(coeff_buff.d_list, D_power_series_list, d_num, &eff_dev);
  }

  dac_value_temp[0] = (powerR * eff_dev) / (*C_adc_value_normalized);

  if (dac_value_temp[0] > 4095) {
    dac_value_temp[0] = 4095;
  }
  /*
     for(int i = 0; i < COEFF_NUM; i++){
              Serial.print( "M_power_series_list:" + String(i));
              Serial.println(M_power_series_list[i],16);
     }
      for(int i = 0; i < COEFF_NUM; i++){
                      Serial.print( "D_power_series_list:" + String(i));

              Serial.println(D_power_series_list[i],16);
     }
      for(int i = 0; i < COEFF_NUM; i++){
                      Serial.print( "coeff_buff.d_list:" + String(i));

              Serial.println(coeff_buff.d_list[i],16);
     }
       Serial.println("powerR."+String(powerR));

     Serial.println("eff_dev."+String(eff_dev));
     Serial.println("deltaP."+String(deltaP));}
  */
}



//namespace Adafruit_LittleFS_Namespace
//{
void load_flash_coeff() {

  Adafruit_LittleFS_Namespace::File flash_file(InternalFS);

  // Initialize Internal File System
  InternalFS.begin();

  flash_file.open(FLASHFILENAME, Adafruit_LittleFS_Namespace::FILE_O_READ);

  // file existed
  if ( flash_file )
  {
    Serial.println(FLASHFILENAME " file exists");

    uint32_t readlen;
    //int buff[64] = { 0 };
    //coefficients buff ;
    readlen = flash_file.read(&coeff_buff, sizeof(struct coefficients));

    //buff[readlen] = 0;
    Serial.println(readlen);
    flash_file.close();
  } else
  {

    coeff_buff = (const struct coefficients) {
      0
    };
    flash_file.close();

    Serial.println("No " FLASHFILENAME " file found, setting coefficients to 0 ");

  }

}

void format_flash() {
  Adafruit_LittleFS_Namespace::File flash_file(InternalFS);

  InternalFS.begin();

  Serial.print("Formating ... ");
  delay(1); // for message appear on monitor

  // Format
  InternalFS.format();
}

void save_flash_coeff() {

  Adafruit_LittleFS_Namespace::File flash_file(InternalFS);

  // Initialize Internal File System
  InternalFS.begin();

  if ( flash_file.open(FLASHFILENAME, Adafruit_LittleFS_Namespace::FILE_O_WRITE) )
  {
    Serial.println("OK");
    //lfs_t *lfs = InternalFS.getFS();
    //lfs_file_seek(,, 0,0);
    //lfs_file_rewind(lfs, lfs_file_t *file);
    //coeff_buff
    // flash_file.write(&coeff_buff, strlen(CONTENTS));
    flash_file.close();

  } else {
    flash_file.close();
  }
}

void save_SD_coeff() {
  String  fileNameConfigFile = String(FILE_NAME_CONFIG_DATA) + String("_0.txt");

  while (SD.exists(fileNameConfigFile)) {

    int i = fileNameLogFile.substring((fileNameConfigFile.indexOf('_') + 1), fileNameConfigFile.indexOf('.')).toInt();
    // Serial.println(i);
    i++;
    fileNameConfigFile = String(FILE_NAME_LOG) + "_" + String(i) + ".txt";
    //   Serial.println(fileNameLogFile);
  }
  File SDconfFile = SD.open(fileNameConfigFile, FILE_WRITE);
  if (SDconfFile) {
    unsigned int chunkSize = SDconfFile.availableForWrite();
    if (chunkSize && sizeof(struct coefficients) >= chunkSize) {
      // if the file is available, write to it:
      SDconfFile.write((byte *)&coeff_buff, chunkSize);
    }

    if (SDconfFile) {
      SDconfFile.close();
    } else {
      Serial.print("error closing ");
    }
  }
}





void set_file_name() {


  while (SD.exists(fileNameLogFile)) {

    int i = fileNameLogFile.substring((fileNameLogFile.indexOf('_') + 1), fileNameLogFile.indexOf('.')).toInt();
    // Serial.println(i);
    i++;
    fileNameLogFile = String(FILE_NAME_LOG) + "_" + String(i) + ".txt";
    //   Serial.println(fileNameLogFile);
  }
}
void open_SD() {
  // Serial.print("set_file_name ");
  set_file_name();

  dataFile = SD.open(fileNameLogFile, FILE_WRITE);
  if (!dataFile) {
    Serial.print("error opening ");
    Serial.println(fileNameLogFile);
    //set_file_name();
    //dataFile = SD.open(fileNameLogFile, FILE_WRITE);
  } else {
    Serial.println("Opening Log file: " + String(dataFile.name()));
  }
}
void recover_SD() {
  if (digitalRead(dacChipSelect) == LOW) {
    digitalWrite(dacChipSelect, HIGH);
    Serial.println("DAC left on ");

  }
  if (digitalRead(chipSelect) == LOW) {
    digitalWrite(chipSelect, HIGH);
    Serial.println("SD left on ");
  }
  close_SD();
  dataFile = SD.open(fileNameLogFile, FILE_WRITE);
  if (!dataFile) {
    Serial.println("Failed to Open Log file: " + String(dataFile.name()));
    open_SD();
  } else {
    Serial.println("Recovered Log file: " + String(dataFile.name()));
  }
}
void close_SD() {
  if (dataFile) {
    dataFile.close();
  } else {
    Serial.print("error closing ");
  }
}
void write_datalog_SD() {

  if (dataFile) {
    unsigned int chunkSize = dataFile.availableForWrite();
    //Serial.print("chunkSize: ");
    //Serial.println(chunkSize);
    //Serial.print("datastring.length: ");
    //Serial.println(dataString.length());
    if (chunkSize && (dataString.length() >= chunkSize)) {
      //Serial.println("file is available, write to it: ");
      //Serial.println(dataString);
      analogReadTimer.stop(); //nessisary to keep it from hanging
      dataFile.write(dataString.c_str(), chunkSize);
      analogReadTimer.start(); //nessisary to keep it from hanging
      dataString.remove(0, chunkSize);
      //Serial.println("Remainder dataString: ");
      // Serial.println(dataString);
    }
    else {
      //Serial.println("buffer not ready yet");
      delay(0);
    }
  } else {
    // if the file isn't open, pop up an error:
    Serial.println("error opening " + String(dataFile.name()) + " and writing: " + dataString);
  }
}

void parseFloat(float32_t * z, const char * value_str) {

  char *end;
  (*z) = (float32_t) strtod(value_str, &end);

  delay(1);
  if (end == value_str) {
    Serial.println("Conversion value error ");
    //  Serial.println(*coeff_val_str_tmp);
    //  Serial.println((*conf_line).substring((token_index+1),(*conf_line).length()));
  } else {
    //   Serial.println(conf_line.substring(1,(token_index)));

    //double z = strtod((conf_line.substring(token_index)).c_str(),NULL);

    //  Serial.print(y);
    //  Serial.print(":");
    Serial.println((*z), 16);
  }
}

void parseUInt(uint32_t * z, const char * value_str) {
  (*z) = (uint32_t) atoi(value_str);
  Serial.println((*z), 16);

}

void readSDConfig() {
  //Serial.println("Initializing SD card... ");

  //C1 : ±#.#######e±##
  File confFile = SD.open(FILE_NAME_CONFIG, FILE_READ);
  String conf_line;
  conf_line.reserve(128); //Avoids heap memory fragmentation
  while (confFile.available()) {
    conf_line = confFile.readStringUntil('\n');
    Serial.println(conf_line); // print to the serial port too:
    conf_line.trim();
    char coeff_type = conf_line[0];
    Serial.print(coeff_type);
    std::size_t token_index = conf_line.indexOf(':');
    int y = atoi(conf_line.substring(1, (token_index)).c_str());
    // const char *coeff_val_str_tmp = conf_line.substring((token_index+1),conf_line.length()).c_str();
    String conf_val = conf_line.substring((token_index + 1), conf_line.length());
    conf_val.trim();
    const char *coeff_val_str_tmp = conf_val.c_str();
    if (conf_line != "") {
      if (coeff_type == '#') {
        //do nothing, comment
      }
      if (coeff_type == 'C') {
        if (COEFF_NUM > y) {
          //pointer to coefficient storage
          parseFloat(&coeff_buff.c_list[y], coeff_val_str_tmp);

        }
      }
      if (coeff_type == 'M') {
        if (COEFF_NUM > y) {
          parseFloat(&coeff_buff.m_list[y], coeff_val_str_tmp);
        }
      }
      if (coeff_type == 'D') {
        if (COEFF_NUM > y) {
          parseFloat(&coeff_buff.d_list[y], coeff_val_str_tmp);
        }
      }
      if (coeff_type == 'U') {
        if (UTILITY_NUM > y) {
          if (y <= 3) {
            // coeff_buff.limits[y] = z;
            parseFloat(&coeff_buff.utility_variables[y], coeff_val_str_tmp);
          }
          if (y == 4) {
            parseUInt(&c_num, coeff_val_str_tmp);
          }
          if (y == 5) {
            parseUInt(&m_num, coeff_val_str_tmp);
          }
          if (y == 6) {
            parseUInt(&d_num, coeff_val_str_tmp);
          }
          if (y == 8) { // U8 Sample rate in mS, default 500
            parseUInt(&sampleRate, coeff_val_str_tmp);
          }
          if (y == 9) {
            //calibrateEnabled = (bool)z;
            parseFloat(&coeff_buff.utility_variables[y], coeff_val_str_tmp);
          }
          if ((y >= 10) && (y < 22)) {
            parseFloat(&coeff_buff.utility_variables[y], coeff_val_str_tmp);
          }
          if ((y >= 22) && (y < 54)) {
            parseFloat(&coeff_buff.utility_variables[y], coeff_val_str_tmp);
          }
          if ((y >= 54) && (y < 66)) {
            unit_label.list[y - 54] = (char*)malloc(sizeof(coeff_val_str_tmp));
            strncpy(unit_label.list[y - 54], coeff_val_str_tmp, sizeof(unit_label.list[y - 54]) - 1);
            unit_label.list[y - 54][sizeof(unit_label.list[y - 54]) - 1] = '\0';
            Serial.println(String(unit_label.list[y - 54]));

          }
           if ((y >= 66) && (y < 75)) {
            parseFloat(&coeff_buff.utility_variables[y], coeff_val_str_tmp);
          }
        }
        if (y >= UTILITY_NUM) {
          Serial.println("unrecognized U");
        }

      }//endif U

    }//skip blank (NULL) lines
  }//Read the file line by line
  confFile.close();

  // see if the card is present and can be initialized:

  Serial.println("card initialized.");
}

/**
   Software Timer callback is invoked via a built-in FreeRTOS thread with
   minimal stack size. Therefore it should be as simple as possible. If
   a periodically heavy task is needed, please use Scheduler.startLoop() to
   create a dedicated task for it.

   More information http://www.freertos.org/RTOS-software-timer.html
*/
void analog_timer_callback(TimerHandle_t xTimerID)
{
  (void) xTimerID;
  readADC();
  // if (calibrateEnabled)
  // {
  // display.println("Cal: ");
  // Serial.println("calibration mode");
  //dumpMemory();
  //  calibrate();
  // } else {
  //update_ADC();
  //update_ADC_avg();
  //  readADC();

  // }
}

void SD_write()
{
  write_datalog_SD();
  delay(1500);
}

void btnDownCheck(TimerHandle_t xTimerID)
{
  (void) xTimerID;

  if ( !digitalRead(BUTTON_UP)) {
    buttonState = (buttonState | 0x1);
  };
  if ( !digitalRead(BUTTON_SELECT)) {
    buttonState = (buttonState | 0x2);
  };
  if ( !digitalRead(BUTTON_MENU)) {
    buttonState = (buttonState | 0x4);
  };
  if ( !digitalRead(BUTTON_DOWN)) {
    buttonState = (buttonState | 0x8);
  };

}
//https://forums.adafruit.com/viewtopic.php?f=24&t=125410
//elapsed time = (stop count – start count) + number of overflows x (2^n-1), f[MHz]= 16MHz / 2^PRESCALER
//uint32_t   current_counter1 = 0; //timer counter for the pulse timer
//uint32_t   current_counter2 = 0; //timer counter for the pulse timer

//uint32_t   overflow_counter = 0; //timer overflow counter for the pulse timer
//was extern "C++", https://devzone.nordicsemi.com/f/nordic-q-a/6829/timers-not-working-when-using-c/24074#24074
extern "C"
{
  void TIMER2_IRQHandler()
  {
    if ((NRF_TIMER2->EVENTS_COMPARE[0] != 0) && ((NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE0_Msk) != 0)) {
      NRF_TIMER2->EVENTS_COMPARE[0] = 0;    //Clear compare register 0 event
      // Need to update the CC[0]
      overflow_counter++;
      //NRF_TIMER2->CC[0] += TIMER2_CC_DELAY;
    }

    if ((NRF_TIMER2->EVENTS_COMPARE[1] != 0) && ((NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE1_Msk) != 0)) {
      NRF_TIMER2->EVENTS_COMPARE[1] = 0;    //Clear compare register 1 event, not needed with the shortcut?
      ADC_COUNTER_IO();
    }
  }

}
//1000ms*3600seconds per hour/Volts/Amps/num of legs/(impulses per kWh)
void PULSE_TIMER_SETUP() {
  NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;  // Set the timer in Counter Mode
  NRF_TIMER2->TASKS_CLEAR = 1;               // clear the task first to be usable for later

  // 32-bit timer will need to be set when
  NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;

  // 1us timer period 16M/2^PRESCALER
  NRF_TIMER2->PRESCALER = 4 << TIMER_PRESCALER_PRESCALER_Pos;

  // 36000000 us compare value, generates EVENTS_COMPARE[0] every 36 seconds
  NRF_TIMER2->CC[0] = 36000000;

  // Enable IRQ on EVENTS_COMPARE[0]
  //NRF_TIMER2->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos) | (TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos);
  NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;
  // Clear the timer when COMPARE0 event is triggered
  NRF_TIMER2->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos;

  NVIC_EnableIRQ(TIMER2_IRQn);
  NRF_TIMER2->TASKS_START = 1;                                                            // Start TIMER


}
//https://learn.openenergymonitor.org/electricity-monitoring/pulse-counting/introduction-to-pulse-counting
void ADC_CHANGE_ISR() {
  //add in an isr timer to do filtering, also add in a cap
  NRF_TIMER2->TASKS_CAPTURE[1] = 1;
  current_counter_new = NRF_TIMER2->CC[1];
  if (ADC_COUNTER_IO()and ( abs(current_counter_fall_new - current_counter_fall_old) > 3)) {
    current_counter_rise_new = current_counter_new;
    current_counter_period_rise = current_counter_rise_new - current_counter_rise_old;
    current_counter_rise_old = current_counter_rise_new;
    overflow_counter_rise = overflow_counter;
    overflow_counter = 0;
  } else if((abs(current_counter_rise_new - current_counter_rise_old) > 3)){
    current_counter_fall_new = current_counter_new;
    current_counter_period_fall = current_counter_fall_new - current_counter_fall_old;
    current_counter_fall_old = current_counter_fall_new;
    overflow_counter_fall = overflow_counter;
    overflow_counter = 0;
  }

}

void ADC_FALLING_ISR() {
  // noInterrupts();
  NRF_TIMER2->TASKS_CAPTURE[2] = 1;  // This is asking to capture the current counter into CC[1]
  current_counter_fall_new = NRF_TIMER2->CC[2];   // counter value  at the instant when capture task was triggered is now ready to be read in CC[2]
  overflow_counter_fall = overflow_counter;
  if( (abs(current_counter_fall_new - current_counter_fall_old) > (200)) and !((bool)ADC_COUNTER_IO())){
//  if( (abs(current_counter_fall_new - current_counter_fall_last) > (200)) ){
//  if( !(ADC_COUNTER_IO())){
    current_counter_period_fall = (current_counter_fall_new - current_counter_fall_old)+(36000000 * overflow_counter_fall);
   // if(current_counter_fall_new < current_counter_fall_old){
   //   current_counter_period_fall=current_counter_period_fall+36000000;
   // }
    current_counter_fall_old = current_counter_fall_new;

    overflow_counter = 0;
    //NRF_TIMER2->TASKS_CLEAR = 1;  //clear the timer to wait
    //flag_counter++;
//    if(ADC_COUNTER_IO()){
//      flag_counter++;
//    }else{
//      flag_counter--;
//    }
   }else{
      flag_counter++;
    }
  current_counter_fall_last = current_counter_fall_new;
   //interrupts();
}
void ADC_RISING_ISR() {
  NRF_TIMER2->TASKS_CAPTURE[3] = 1;  // This is asking to capture the current counter into CC[2]
  current_counter_rise_new = NRF_TIMER2->CC[3];   // counter value  at the instant when capture task was triggered is now ready to be read in CC[2]
  if (abs(current_counter_rise_new - current_counter_rise_old) > 30) {
    current_counter_period_rise = (current_counter_rise_new - current_counter_rise_old);
    current_counter_rise_old = current_counter_rise_new;
    overflow_counter_rise = overflow_counter;
    overflow_counter = 0;
  }
}



void BUTTON_UP_ISR() {
  btnDownTime.reset();

  //buttonState = buttonState | 0x1;
}
void BUTTON_DOWN_ISR() {
  btnDownTime.reset();

  // buttonState = buttonState | 0x2;
}
void BUTTON_MENU_ISR() {
  btnDownTime.reset();

  //buttonState = buttonState | 0x4;
}
void BUTTON_SELECT_ISR() {
  btnDownTime.reset();

  // buttonState = buttonState | 0x8;
}
