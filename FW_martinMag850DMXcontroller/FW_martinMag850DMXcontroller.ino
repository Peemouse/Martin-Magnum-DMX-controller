//Clément LE PRIOL
//15 feb 2026
//Martin Magnum 850 DMX/sACN controller
//Github : https://github.com/Peemouse/Martin-Magnum-DMX-controller

#define FW_MAJOR      0
#define FW_MINOR      1

#define DEBUG
//#define ETHERNET

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#include "HW.h"
#include "Parameters.h"
#include <Arduino.h>
#include <esp_dmx.h>
#include <SPI.h>
#include <Wire.h>
#include <Ethernet.h>
#include "ESP_Async_E1.31_ETH\ESPAsyncE131ETH.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>

unsigned int DMXstartAddress = 5;
dmx_port_t dmxPort = 1; //Serial port for DMX
byte DMXdata[DMX_PACKET_SIZE];

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#ifdef ETHERNET
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(10, 0, 0, 190);
IPAddress dns(10, 0, 0, 250);
IPAddress gateway(10, 0, 0, 250);
IPAddress subnet(255, 255, 255, 0);

#define UNIVERSE 1                      // First DMX Universe to listen for
#define UNIVERSE_COUNT 1                // Total number of Universes to listen for, starting at UNIVERSE

EthernetServer server(80);// Initialize the Ethernet server library with the IP address and port you want to use (port 80 is default for HTTP)
ESPAsyncE131ETH e131(UNIVERSE_COUNT);
#endif
bool dmxIsConnected = false, memoDmxIsConnected = false;

bool btnMinusMemo = false, btnEnterMemo = false, btnPlusMemo = false; //keep last states of buttons for finding rising and falling edges
bool btnMinusShortPress = false, btnEnterShortPress = false, btnPlusShortPress = false; //boolean to use for triggering actions in code
bool btnEnterLongPress = false, btnEnterLongPressMemo = false; //Center button can have a long press action

bool foggerHeating = false, foggerReady = false;
bool PWMactivated = true;
bool PWMactivatedMemo = false;

bool pleaseUpdateEEPROM = false;

//display variables
bool flag1s = true;
uint8_t menuSelected = 0;

uint8_t fogIntensity = 0;
uint8_t tOn = 0;
uint8_t tOff = 0;

// Timers
unsigned long lastDMXpacketTimestamp = 0;
unsigned long btnEnterTimer = 0;
unsigned long btnPlusTimer = 0;
unsigned long btnMinusTimer = 0;
unsigned long fogOnTimer = 0;
unsigned long fogOffTimer = 0;

// Auto repeat management
unsigned long btnPlusPressTime = 0;
unsigned long btnMinusPressTime = 0;
unsigned long btnPlusRepeatTimer = 0;
unsigned long btnMinusRepeatTimer = 0;

bool btnPlusHeld = false;
bool btnMinusHeld = false;

Preferences preferences;

//Tasks definition
void TaskDMXwired( void *pvParameters );
void TaskDisplay( void *pvParameters );
void TaskInput( void *pvParameters );
void TaskWebServer( void *pvParameters );
void TaskE131( void *pvParameters );
void TaskPWM( void *pvParameters );

void setup() {
  // Setup Serial Monitor
  Serial.begin(115200);

  pinMode(SW_MINUS, INPUT_PULLUP);
  pinMode(SW_ENTER, INPUT_PULLUP);
  pinMode(SW_PLUS, INPUT_PULLUP);
  pinMode(FOGGER_HEATING, INPUT_PULLUP);
  pinMode(FOGGER_READY, INPUT_PULLUP);

  ledcAttach(PWM_OUT, PWM_FREQ, 8);
  ledcWrite(PWM_OUT, 0);

  //Ethernet.init(ETH_PHY_CS); //if need to change CS pin

  //Retrieve parameters from memory
  preferences.begin("params", false);
  DMXstartAddress = preferences.getUInt("dmxStartAddr", 0);
  if (DMXstartAddress > 510){
    DMXstartAddress = 1;
    preferences.putUInt("dmxStartAddr", DMXstartAddress);
  }
  preferences.end();

  //DMX initialization
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_personality_t personalities[] = {
    {1, "Default Personality"}
  };
  int personality_count = 1;
  dmx_driver_install(dmxPort, &config, personalities, personality_count);
  dmx_set_pin(dmxPort, DMX_TX_PIN, DMX_RX_PIN, DMX_EN_PIN);

#ifdef ETHERNET
  //Ethernet.begin(mac); //Version DHCP
  Ethernet.begin(mac, ip, dns, gateway, subnet); // start the Ethernet connection and the server, fixed address
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("HW ERROR : Ethernet interface not found.");
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }
  // start the server
  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());
#endif

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.println("Martin Magnum");
  display.println("DMX controller");
  display.println("");
  display.print("v.");
  display.print(FW_MAJOR);
  display.print(".");
  display.print(FW_MINOR);
  display.display();
  delay(3000);
  display.clearDisplay();

// Now set up tasks to run independently.
  xTaskCreatePinnedToCore( 
    TaskPWM
    ,  "TaskPWM"
    ,  2048  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskDMXwired
    ,  "TaskDMXwired"   // A name just for humans
    ,  2048  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskDisplay
    ,  "TaskDisplay"   // A name just for humans
    ,  4096  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskInput
    ,  "TaskInput"
    ,  2048  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskWebServer
    ,  "TaskWebServer"
    ,  4096  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL
    ,  ARDUINO_RUNNING_CORE);

    xTaskCreatePinnedToCore(
    TaskE131
    ,  "TaskE131"
    ,  4096  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL
    ,  ARDUINO_RUNNING_CORE);

}

void loop() {}

void TaskDMXwired(void *pvParameters){ // Take care of DMX IN packets

  (void) pvParameters;
  UBaseType_t uxHighWaterMark;

  while(1){

  dmx_packet_t packet;

    if (dmx_receive(dmxPort, &packet, DMX_TIMEOUT_TICK)) { //DMX data received
      unsigned long now = millis();

      if (!packet.err) {
        if (!dmxIsConnected) {
          Serial.println("DMX is connected!");
          dmxIsConnected = true;
        }
        dmx_read(dmxPort, DMXdata, packet.size);

      } else {
        Serial.println("A DMX error occurred.");
      }
    } else if (dmxIsConnected) {
      Serial.println("DMX was disconnected.");
      dmxIsConnected = false;
    }
    memoDmxIsConnected = dmxIsConnected;
#ifdef DEBUG
    // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    // Serial.print("TaskDMXwired ");
    // Serial.println(uxHighWaterMark);
#endif

    yield();
  }
}

void TaskE131(void *pvParameters){ // Take care of DMX IN packets

  (void) pvParameters;
  UBaseType_t uxHighWaterMark;

  while(1){ 
#ifdef ETHERNET
    if (!e131.isEmpty()) {
      e131_packet_t packet;
      e131.pull(&packet);     // Pull packet from ring buffer
      
      Serial.printf("Universe %u / %u Channels | Packet#: %u / Errors: %u / CH1: %u\n",
              htons(packet.universe),                 // The Universe for this packet
              htons(packet.property_value_count) - 1, // Start code is ignored, we're interested in dimmer data
              e131.stats.num_packets,                 // Packet counter
              e131.stats.packet_errors,               // Packet error counter
              packet.property_values[1]);             // Dimmer data for Channel 1
    }
#endif
    yield();
    //vTaskDelay(10);
  }
}

void TaskPWM(void *pvParameters){ // Take care of DMX IN packets

  (void) pvParameters;
  UBaseType_t uxHighWaterMark;

  while(1){
    if (dmxIsConnected){
      fogIntensity = constrain(DMXdata[DMXstartAddress],0,255);
      fogIntensity = map(fogIntensity,0,255,0,100); //converts fog intensity in percent
      tOn = constrain(DMXdata[DMXstartAddress+1],0,255);
      tOff = constrain(DMXdata[DMXstartAddress+2],0,255);
    }

    uint8_t PWMvalue = map(fogIntensity, 0, 100, VOUT_START, VOUT_END);

    if (tOn == 0 || tOff == 0){ //Force to ON (continuous)
      PWMactivated = true;
    }
    else if(!PWMactivated && (millis() - fogOffTimer) > ((unsigned long)tOff)*1000){ //Timer OFF is over, switch to ON
      PWMactivated = true;
      fogOnTimer = millis();
    }
    else if(PWMactivated && (millis() - fogOnTimer) > ((unsigned long)tOn)*1000){ //Timer ON is over, switch to OFF
      PWMactivated = false;
      fogOffTimer = millis();
    }
    
    if(PWMactivated){
      ledcWrite(PWM_OUT, PWMvalue);
    }
    else{
      ledcWrite(PWM_OUT, VOUT_IDLE);
    }

    PWMactivatedMemo = PWMactivated;
#ifdef DEBUG
      // Serial.print("PWM Value: ");
      // Serial.print(PWMvalue);
      // Serial.print(" - Activated: ");
      // Serial.println(PWMactivated);
#endif
    
#ifdef DEBUG
    // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    // Serial.print("TaskPWM ");
    // Serial.println(uxHighWaterMark);
#endif
    //yield();
    vTaskDelay(100);
  }
}

void TaskWebServer(void *pvParameters){ // Take care of DMX IN packets

  (void) pvParameters;
  UBaseType_t uxHighWaterMark;

  while(1){
#ifdef ETHERNET
    EthernetClient client = server.available();
    if (client) {
      Serial.println("new client");
      // an HTTP request ends with a blank line
      bool currentLineIsBlank = true;
      while (client.connected()) {
        if (client.available()) {
          char c = client.read();
          Serial.write(c);
          // if you've gotten to the end of the line (received a newline character) and the line is blank, the HTTP request has ended, so you can send a reply
          if (c == '\n' && currentLineIsBlank) {
            // send a standard HTTP response header
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: text/html");
            client.println("Connection: close");  // the connection will be closed after completion of the response
            client.println("Refresh: 5");  // refresh the page automatically every 5 sec
            client.println();
            client.println("<!DOCTYPE HTML>");
            client.println("<html>");
            // output the value of each analog input pin
            for (int analogChannel = 0; analogChannel < 6; analogChannel++) {
              int sensorReading = analogRead(analogChannel);
              client.print("analog input ");
              client.print(analogChannel);
              client.print(" is ");
              client.print(sensorReading);
              client.println("<br />");
            }
            client.println("</html>");
            break;
          }
          if (c == '\n') {
            // you're starting a new line
            currentLineIsBlank = true;
          } else if (c != '\r') {
            // you've gotten a character on the current line
            currentLineIsBlank = false;
          }
        }
      }
      // give the web browser time to receive the data
      vTaskDelay(1);
      // close the connection:
      client.stop();
      Serial.println("client disconnected");
    
      //vTaskDelay(10);
    }
#endif
    yield();
  }
}

void TaskDisplay(void *pvParameters){ // Take care of DMX IN packets

  (void) pvParameters;
  UBaseType_t uxHighWaterMark;

  while(1){

      //Blinking flag
    if ((millis()/1000) % 2){ 
      flag1s = true;
    }
    else{
      flag1s = false;
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(0);

    if (dmxIsConnected){
      if(!memoDmxIsConnected){menuSelected=0;}
      display.setCursor(25, 24);
      display.print(F("DMX CONNECTED"));
    }
    else if(flag1s){ // DMX not connected
      display.setCursor(25, 24);
      display.print(F("NO CONNECTION"));
    }

    //Common display
    display.setCursor(0, 0);
    
    if(!dmxIsConnected && fogIntensity == 0 && menuSelected == 0){
      display.setTextSize(2);
      display.setCursor(15, 5);
      display.println("OFF");
    }
    else{
      display.print("FOG : ");
      display.print(fogIntensity);
      display.println("%");
      display.print("TON : ");
      display.print(tOn);
      display.println("s");
      display.print("TOFF: ");
      display.print(tOff);
      display.println("s");
    }
    
    display.setCursor(102, 0);
    display.setTextSize(0);
    display.print("CH");
    display.setTextSize(2);
    if(DMXstartAddress <10){display.setCursor(102, 8);}
    else if(DMXstartAddress <100){display.setCursor(96, 8);}
    else{display.setCursor(90, 8);}
    display.print(DMXstartAddress);
  //cursor display
    if(menuSelected>0 && menuSelected <4){
      display.setTextSize(0);
      display.setCursor(62, 8*(menuSelected-1));
      display.print(F("<"));
    }
    else if(menuSelected==4){
      display.setTextSize(2);
      display.setCursor(78, 8);
      display.print(">");
    }
    
    display.display();
    vTaskDelay(100);
  }
}

void TaskInput(void *pvParameters){ // Take care of digital inputs (user switch panel)

  (void) pvParameters;
  UBaseType_t uxHighWaterMark;

  while(1){
    bool btnMinus = !digitalRead(SW_MINUS);
    bool btnEnter = !digitalRead(SW_ENTER);
    bool btnPlus = !digitalRead(SW_PLUS);

#ifdef DEBUG
  	// Serial.print("Btn ");
  	// Serial.print(btnMinus);
  	// Serial.print(btnEnter);
  	// Serial.println(btnPlus);
#endif

    //Enter button management (short and long press detection)
    if (btnEnterMemo != btnEnter){ //Button state change
      if (btnEnter) { //Rising Edge
        btnEnterTimer = millis();
      }
      else { //Falling edge
        if (!btnEnterLongPressMemo && (millis() - btnEnterTimer) < LONGPRESSTIMER){
          btnEnterShortPress = true;
        }
        btnEnterLongPressMemo = false;
      }
    }

    if (btnEnter && (millis() - btnEnterTimer) >= LONGPRESSTIMER){
      btnEnterLongPress = true;
      btnEnterLongPressMemo = true;
      btnEnterTimer = millis();
    }

    btnEnterMemo = btnEnter; //Save the state for next cycle

unsigned long now = millis();

    // ===== PLUS BUTTON =====
    if (btnPlus) {
      if (!btnPlusHeld) {  
        btnPlusHeld = true;
        btnPlusPressTime = now;
        btnPlusRepeatTimer = now;
        btnPlusShortPress = true; // first immediate increment
      }
      else {
        unsigned long heldTime = now - btnPlusPressTime;
        if (heldTime > REPEAT_START_DELAY) {
          // Dynamix interval compute
          unsigned long accelTime = heldTime - REPEAT_START_DELAY;
          unsigned long interval;
          if (accelTime >= REPEAT_ACCEL_TIME) {
            interval = REPEAT_FAST_INTERVAL;
          }
          else {
            interval = REPEAT_INITIAL_INTERVAL - ((REPEAT_INITIAL_INTERVAL - REPEAT_FAST_INTERVAL) * accelTime) / REPEAT_ACCEL_TIME;
          }
          if (now - btnPlusRepeatTimer >= interval) {
            btnPlusRepeatTimer = now;
            btnPlusShortPress = true;
          }
        }
      }
    }
    else {
      btnPlusHeld = false;
    }

    // ===== MINUS BUTTON =====
    if (btnMinus) {
      if (!btnMinusHeld) {  
        btnMinusHeld = true;
        btnMinusPressTime = now;
        btnMinusRepeatTimer = now;
        btnMinusShortPress = true;
      } else {
        unsigned long heldTime = now - btnMinusPressTime;
        if (heldTime > REPEAT_START_DELAY) {
          unsigned long accelTime = heldTime - REPEAT_START_DELAY;
          unsigned long interval;
          if (accelTime >= REPEAT_ACCEL_TIME) {
              interval = REPEAT_FAST_INTERVAL;
          } else {
              interval = REPEAT_INITIAL_INTERVAL -
                  ((REPEAT_INITIAL_INTERVAL - REPEAT_FAST_INTERVAL) * accelTime) / REPEAT_ACCEL_TIME;
          }
          if (now - btnMinusRepeatTimer >= interval) {
              btnMinusRepeatTimer = now;
              btnMinusShortPress = true;
          }
        }
      }
    } 
    else {
      btnMinusHeld = false;
    }

    //Handle button pushes
    if(btnEnterLongPress){
      if(menuSelected == 0){//no menu selected yet
        if(dmxIsConnected){
          menuSelected = 4;
        }
        else{
          menuSelected = 1;
        }
      }
      else{ //exiting menu
        menuSelected = 0;
        preferences.begin("params", false);
        preferences.putUInt("dmxStartAddr", DMXstartAddress);
        preferences.end();
      }
    }

    switch(menuSelected){
      case 1:
        if(btnMinusShortPress && fogIntensity>0){fogIntensity--;}
        if(btnPlusShortPress && fogIntensity<100){fogIntensity++;}
      break;
      case 2:
        if(btnMinusShortPress && tOn>0){tOn--;}
        if(btnPlusShortPress && tOn<255){tOn++;}
      break;
      case 3:
        if(btnMinusShortPress && tOff>0){tOff--;}
        if(btnPlusShortPress && tOff<255){tOff++;}
      break;
      case 4:
        if(btnMinusShortPress && DMXstartAddress>1){DMXstartAddress--;}
        if(btnPlusShortPress && DMXstartAddress<510){DMXstartAddress++;}
      break;
      default:
      break;
    }
    if(btnEnterShortPress){
      if(!dmxIsConnected){
        menuSelected++;
        if(menuSelected == 5){menuSelected=1;}
      }
    }
    
    btnEnterLongPress = btnEnterShortPress = btnMinusShortPress = btnPlusShortPress = false;

    //Fogger statuts inputs reading
    foggerHeating = !digitalRead(FOGGER_HEATING);
    foggerReady = !digitalRead(FOGGER_READY);
    
    vTaskDelay(10);
#ifdef DEBUG
    // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    // Serial.print("TaskInput ");
    // Serial.println(uxHighWaterMark);
#endif
  }
}