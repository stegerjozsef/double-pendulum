// Author: József Stéger
// Date: 25. April 2024 
// Double pendulum measurement
// ---------------------------

#include <TFT_eSPI.h> 
#include <SPI.h>
#include "NotoSansBold15.h"
#include "esp_adc_cal.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <AS5600.h>
#include <Wire.h>                   
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// define refresh rates (msec)
#define REFRESH_DISPLAY 1000
#define REFRESH_BATTERY 5000
#define REFRESH_ANGLE 25
#define HEARTBEAT_TIMEOUT 5000
//TODO implement #define FADE_MESSAGE 15000
#define BUTTON_RESOLUTION 500
#define VERBOSE
//#define BUTTONS_ENABLED

// define pinout
#define BUTTON1PIN 35
#define BUTTON2PIN 0
#define BATTERYPIN 34
#define PWRENPIN 14
#define W1_DATA 21
#define W1_CLOCK 22
#define W2_DATA 17
#define W2_CLOCK 32

// the longest serial line message
#define MESSAGE_SIZE 64

// how many samples inter task queue may store
#define QUEUE_SIZE 256
#define MSG_QUEUE_SIZE 32
#define SAMPLES_PER_PACKET 64

#define NEQ(a,b) ((a) != (b))

// sample representation
struct sample_t {
  uint32_t timestamp;
  uint16_t angle_1;
  uint16_t angle_2;
};

// sensor info to display
struct sensor_t {
  uint8_t  state;
  uint16_t angle;
};


// global variables
const char* ssid = "XXXXXXXXXX";            // WiFi ESSID
const char* password = "XXXXXXXXXX";        // WiFi secret
unsigned int localUdpPort = 4210;           // port to listen on
int voltage = -1;                           // last battery voltage
QueueHandle_t queue;                        // inter task queue handler
QueueHandle_t msg_queue;                    // inter task queue handler for serial line messages
volatile uint32_t button_display_last = 0;  // when button interrupt was last handled
volatile uint32_t button_measure_last = 0;  // when button interrupt was last handled
sensor_t disp_sensor[2];                    // store sensor infor to be displayed
TaskHandle_t Task_measure_angle;            // angle measurement task handler
TaskHandle_t Task_display;                  // display task handler
TaskHandle_t Task_measure_battery;          // voltage measurement task handler
TaskHandle_t Task_udp;                      // flood the client with data task handler
char disp_message[MESSAGE_SIZE];            // buffer to write extra message to display
volatile bool consumer_ready = false;       // whether the client is listening to data
volatile bool display_changed = false;      // whether there is new info to display


// function declarations
void message(const char *format, ...);
void refresh_display(void* pvParameters);
void measure_voltage(void* pvParameters);
void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info);
void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info);
void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info);

// implementation starts here

void fatal(const char * message) {
  uint32_t t0 = millis();
  for (;;) {
    Serial.printf("FATAL ERROR %d ms ago %s\n", t0 - millis(), message);
    delay(1000);
  }
}

void IRAM_ATTR btn_display() {
  if (millis() - button_display_last > BUTTON_RESOLUTION) {
    message("handling button 1 pressed");
    button_display_last = millis();
    start_stop_display();
  }
}

void IRAM_ATTR btn_measurement() {
  if (millis() - button_measure_last > BUTTON_RESOLUTION) {
    message("handling button 2 pressed");
    button_measure_last = millis();
    start_stop_measurement();
  }
}

void start_stop_display() {
  negate_task_state(Task_display);
}

void start_stop_measurement() {
  consumer_ready = !consumer_ready;
}

/*void negate_task_state(TaskHandle_t &task) {
  TaskStatus_t details;
  configASSERT(task);
  //eTaskState state = eTaskGetState(task);
  vTaskGetInfo(task, &details, pdTRUE, eInvalid);//FIXME details also holds state???
  if (details.eCurrentState == eSuspended) {
    Serial.printf("task %s was suspended, resuming...\n", details.pcTaskName);
    vTaskResume(task);
  } else if (details.eCurrentState == eRunning || details.eCurrentState == eBlocked) {
    Serial.printf("task %s was running, suspending...\n", details.pcTaskName);
    vTaskSuspend(task);
  } else {
    Serial.printf("ERROR at %d task %s is in state %s, unhandled situation", millis(), details.pcTaskName, details.eCurrentState);
  }
}*/

eTaskState negate_task_state(TaskHandle_t &task) {
  configASSERT(task);
  eTaskState state = eTaskGetState(task);
  if (state == eSuspended) {
    vTaskResume(task);
  } else if (state == eRunning || state == eBlocked) {
    vTaskSuspend(task);
  } else {
    //oops unhandled situ
  }
  eTaskState state_ = eTaskGetState(task);
  message("task %x state change %x -> %x", task, state, state_);
  return state_;
}

uint8_t check_sensor(AS5600 &hall) {
  uint8_t state = 0;
  if (hall.detectMagnet()) {
    state = 1;
    state |= ((uint8_t)hall.magnetTooStrong()) << 1;
    state |= ((uint8_t)hall.magnetTooWeak()) << 2;
  }
  return state;
}

void debug_magnets(uint8_t state) {
  if (state) {
    if (state&0b010) {
      message("\tmagnet too strong");
    }
    if (state&0b100) {
      message("\tmagnet too weak");
    }
    message("\tmagnet detected");
  } else {
    message("\tno magnet detected");
  }
}


sensor_t sample_as5600(AS5600 &hall) {
  sensor_t r;
  if( hall.isConnected() ) {
    r.state = check_sensor(hall);
    if (r.state) {
      r.angle = hall.readAngle();
    }
  } else {
    fatal("ERROR AS5600 lost connection");
  }
  return r;
}



void setup() {
  // setup serial line for debug purposes
  Serial.begin(115200);
  Serial.printf("setup running on core %d\n", xPortGetCoreID());

  // allocate memory for the queue
  queue = xQueueCreate(QUEUE_SIZE, sizeof(sample_t));
  msg_queue = xQueueCreate(MSG_QUEUE_SIZE, MESSAGE_SIZE);

  if ((queue == NULL) || (msg_queue == NULL)) {
    fatal("Cannot allocate queue");
  }

  // start various tasks  
  xTaskCreatePinnedToCore(measure_angle, "T_meas_ang", 10000, NULL, 1, &Task_measure_angle, 0);
  xTaskCreatePinnedToCore(refresh_display, "T_disp", 10000, NULL, 1, &Task_display, 1);
  xTaskCreatePinnedToCore(measure_voltage, "T_meas_volt", 10000, NULL, 1, &Task_measure_battery, 1);
  xTaskCreatePinnedToCore(server, "T_networking", 10000, NULL, 1, &Task_udp, 1);
  // handle button events
#ifdef BUTTONS_ENABLED
  attachInterrupt(BUTTON1PIN, btn_display, FALLING);
  attachInterrupt(BUTTON2PIN, btn_measurement, FALLING);
#endif

}

void message(const char *format, ...) {
  char message[MESSAGE_SIZE];
  va_list argptr;
  va_start(argptr, format);
  vsprintf(message, format, argptr);
  if ( xQueueSend(msg_queue, (void *)&message, (TickType_t)0) != pdPASS) {
    fatal("cannot buffer message");
  }
  va_end(argptr);
}


          //sample.angle_1 = hall_1.readAngle(); // * AS5600_RAW_TO_DEGREES;
          //sample.angle_2 = hall_2.rawAngle();

void measure_angle(void* pvParameters) {
  message("Measurement task running on core %d", xPortGetCoreID());
  static uint32_t lastTime = 0;
  static uint32_t lastTimeDisp = 0;
  sample_t sample;
  sensor_t s1, s2;
  //initialize sensors
  TwoWire w1 = TwoWire(0);
  TwoWire w2 = TwoWire(1);
  w1.begin(W1_DATA, W1_CLOCK);
  w2.begin(W2_DATA, W2_CLOCK);
  AS5600 hall_1 = AS5600(&w1);
  AS5600 hall_2 = AS5600(&w2);
  if( hall_1.isConnected() ) {
    message("AS5600 connected %d", hall_1.getAddress());
    hall_1.begin();
    hall_1.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.
  } else {
    fatal("ERROR AS5600 not connected");
  }
  if( hall_2.isConnected() ) {
    message("AS5600 connected %d", hall_2.getAddress());
    hall_2.begin();
    hall_2.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.
  } else {
    fatal("ERROR AS5600 not connected");
  }

  for (;;) {
    uint32_t t = millis();
    if (t - lastTime >= REFRESH_ANGLE) {
      lastTime = t;
      bool todisp = t - lastTimeDisp >= REFRESH_DISPLAY;

      if (consumer_ready || todisp) {
        // sample taken
        s1 = sample_as5600(hall_1);
        s2 = sample_as5600(hall_2);

        if (consumer_ready) {
          lastTime = sample.timestamp = t;
          sample.angle_1 = s1.angle;
          sample.angle_2 = s2.angle;

          if ( xQueueSend(queue, (void *)&sample, (TickType_t)0) != pdPASS) {
            message("OOPS cannot queue sample");
            delay(1000);
          }
        }

        if (todisp) {
          lastTimeDisp = t;
          bool change = NEQ(s1.state, disp_sensor[0].state) || NEQ(s1.angle, disp_sensor[0].angle) || NEQ(s2.state, disp_sensor[1].state) || NEQ(s2.angle, disp_sensor[1].angle);
          display_changed |= change;
          if (change) {
            memcpy((void *)disp_sensor, (void *)&s1, sizeof(sensor_t));
            memcpy((void *)(disp_sensor+1), (void *)&s2, sizeof(sensor_t));
            message("some sensory data changed");
          } else {
            message("all sensory data remained");
          }
        }
      }
    }
    delay(1);
  }
}

void server(void* pvParameters) {
  message("Server task running on core %d", xPortGetCoreID());

  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);         // make sure not to sore config in flash
  WiFi.disconnect(true);          // delete old config
  delay(1000);
  WiFi.onEvent(WiFiStationConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  WiFi.begin(ssid, password);

  // packet structure
  // timestamp
  // id
  // voltage
  // list of samples
  unsigned int bufsize = sizeof(unsigned long)+sizeof(unsigned long)+sizeof(int)+SAMPLES_PER_PACKET*sizeof(sample_t);
  char incomingPacket[255];                // buffer for incoming packets
  char outgoingPacket[bufsize];            // buffer for outgoing packets
  unsigned long pcnt = 0;                  // packet counter
  uint8_t scnt = 0;                        // sample counter
  char * ptr;                              // pointer in buffer
  sample_t sample;                         // to store items from the queue
  unsigned long lastHeartbeatTime;         // when client last ping was received

  WiFiUDP Udp;
  Udp.begin(localUdpPort); // udp.stop()
  message("Now listening at IP %s, UDP port %d", WiFi.localIP().toString().c_str(), localUdpPort);

  for (;;) {
    int packetSize = Udp.parsePacket();
    if (packetSize) {
      message("Received %d bytes from %s, port %d", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
      int len = Udp.read(incomingPacket, 255);
      if (len > 0) {
        incomingPacket[len] = 0;
      }
      message("received: %s", incomingPacket);
      //TODO parse received command
      if (strcmp(incomingPacket, "start") == 0) {
        consumer_ready = true;
        sprintf(disp_message, "client at %d", Udp.remotePort());
        message("client at %d", Udp.remotePort());
        display_changed = true;
        lastHeartbeatTime = millis();
      } else if (strcmp(incomingPacket, "ping") == 0) {
        lastHeartbeatTime = millis();
      }
    } 
    //message("tick");

    if (consumer_ready) {
      //if (xQueuePeek(queue, (void *)&sample, (TickType_t) 10) == pdPASS) {
      if (xQueueReceive(queue, (void *)&sample, (TickType_t) 0) == pdPASS) {
        // push to packet 
        if (scnt++ == 0) {
          Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
          unsigned long t = micros();
          ptr = outgoingPacket;
          memcpy((void *)ptr, (void *)&t, sizeof(unsigned long));      // store timestamp
          ptr += sizeof(unsigned long);
          memcpy((void *)ptr, (void *)&pcnt, sizeof(unsigned long));   // store packet id
          ptr += sizeof(unsigned long);
          memcpy((void *)ptr, (void *)&voltage, sizeof(unsigned long));// store battery voltage
          ptr += sizeof(unsigned long);
        }
        memcpy((void *)ptr, (void *)&sample, sizeof(sample_t));        // store this sample
        ptr += sizeof(sample_t);
        if (scnt == SAMPLES_PER_PACKET) {                              // packet ready to send
          Udp.write((uint8_t *)outgoingPacket, bufsize);
          Udp.endPacket();
          ++pcnt;
          scnt = 0;
        }
      } else {
        message("No valid sample queued");
        delay(5);
      }
    }

    if (consumer_ready && (millis() - lastHeartbeatTime > HEARTBEAT_TIMEOUT)) {           // remote party disappeared
      consumer_ready = false;
      unsigned int drp = 0;
      while (xQueueReceive(queue, (void *)&sample, (TickType_t) 0) == pdPASS) {
        ++drp;                                                        // empty queue
      }
      message("client lost. %d samples dropped", drp);
      sprintf(disp_message, "dropped %d samples", drp);
      display_changed = true;
    }

    delay(1);
  }
}


void refresh_display(void* pvParameters) {
  message("Refresh display task running on core %d", xPortGetCoreID());
  disp_message[0] = '\0';
  TFT_eSPI tft = TFT_eSPI();
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.loadFont(NotoSansBold15);
  for (;;) {
    if (display_changed) {
#ifdef VERBOSE
      message("refresh display");
#endif
      tft.fillScreen(TFT_BLACK);
      tft.setCursor(0, 0);
      // print network info
      tft.setTextColor(TFT_WHITE);
      tft.print("IP addr: ");
      tft.println(WiFi.localIP());
      // print battery info
      tft.setTextColor(TFT_GREEN);
      tft.print("Battery: ");
      tft.print(voltage);
      tft.println(" mV");
      for (int p=0; p<2; p++) {
        uint8_t state = disp_sensor[p].state;
        if (state&0b010) {
          tft.setTextColor(TFT_RED);          // too strong
          tft.printf("angle %d\n", disp_sensor[p].angle);
        } else if (state&0b100) {
          tft.setTextColor(TFT_YELLOW);       // too weak
          tft.printf("angle %d\n", disp_sensor[p].angle);
        } else if (state) {
          tft.setTextColor(TFT_BLUE);         // okay
          tft.printf("angle %d\n", disp_sensor[p].angle);
        } else {
          tft.setTextColor(TFT_RED);          // too strong
          tft.printf("no magnet\n");
        }
      }
      if (consumer_ready) {
        tft.setTextColor(TFT_BLUE);         // okay
        tft.printf("a client is receiving data\n");
      } else {
        tft.setTextColor(TFT_BLUE);         // okay
        tft.printf("waiting for client\n");
      }
      display_changed = false;
    }
    delay(REFRESH_DISPLAY);
  }
}

// message relay worker
void loop() {
  message("main loop running on core %d", xPortGetCoreID());
  char msg[MESSAGE_SIZE];
  for (;;) {
    if (xQueueReceive(msg_queue, (void *)&msg, (TickType_t) 0) == pdPASS) {
      if (strlen(msg)) {
        Serial.printf("%d -- %s\n", millis(), msg);
      }
    }
  }
}

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  message("Connected to AP successfully!");
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info) {
  message("WiFi connected, IP address: %x", WiFi.localIP());
  display_changed = true;
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  message("Disconnected from WiFi, reason: %d", info.wifi_sta_disconnected.reason);
  WiFi.begin(ssid, password);
  //TODO: display event
}


void measure_voltage(void* pvParameters) {
  message("Task measure voltage is running on core %d", xPortGetCoreID());
  int vref = 1100;
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t)ADC_UNIT_1, (adc_atten_t)ADC1_CHANNEL_6, (adc_bits_width_t)ADC_WIDTH_BIT_12, vref, &adc_chars);
  //Check type of calibration value used to characterize ADC
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
      message("eFuse Vref:%u mV", adc_chars.vref);
      vref = adc_chars.vref;
  } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
      message("Two Point --> coeff_a:%umV coeff_b:%umV", adc_chars.coeff_a, adc_chars.coeff_b);
  } else {
      message("Default Vref: 1100mV");
  }
  for (;;) {
    digitalWrite(PWRENPIN, HIGH);
    int raw = 0;
    for(int j = 0; j < 100; j++ ) {
      raw += analogRead(BATTERYPIN);
    }
    int voltage_now = 2 * int((float)raw / 409500.0 * 3.3 * vref);
    display_changed |= NEQ(voltage, voltage_now);
    voltage = voltage_now;
    digitalWrite(PWRENPIN, LOW);
  #ifdef VERBOSE
    message("battery %d mV", voltage);
  #endif
    delay(REFRESH_BATTERY);
  }
}
