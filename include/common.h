#include <Arduino.h>
#include <configs.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

// Actively used pins
// #define PGOOD 20 
#define NRST 7 
#define WARMRST 35
#define SOP_0 14
#define SOP_1 36
#define SOP_2 9
#define CLI_TX 17
#define CLI_RX 18
// #define MSS_RX 4

// Unused but connected pins
#define AWR_IO1 16
#define HOSTINT 8
#define PMIC_EN 37
#define SYNC_OUT 48
#define SYNC_IN 47
#define AWR_IO0 21
// #define NERRIN 19
#define SS 10
#define MOSI 11
#define CLK 12
#define MISO 13
#define AWR_IO2 2 

// Global variables
// TODO may need to manage with Mutex/Sephamore if multi-core processing is implemented
volatile bool radar_ready = false; // True when radar TX buffer is filled.
String curr_setting = "functional"; // Indicates current radar board setting.
bool radar_state = false; // True when radar power supply is stable (PGOOD = HIGH).
bool radar_config = false; // True when a config has been loaded into the radar board.
bool radar_started = false; // True when startRadar has been called.
RADAR_CONFIG current_config; // Reference to current config being used (within /include/configs.h)

TaskHandle_t checkSerialTaskHandle = NULL;
TaskHandle_t serialInputTaskHandle = NULL;
TaskHandle_t parseConfigTaskHandle = NULL;
TaskHandle_t dataLoggingTaskHandle = NULL;
TaskHandle_t dataParsingTaskHandle = NULL;
TaskHandle_t dataStreamingTaskHandle = NULL;

struct configParamStruct {
  uint16_t numRangeBins;
  uint16_t numDopplerBins;
  float rangeResolutionMeters;
  float rangeIdxToMeters;
  float dopplerResolutionMps;
  float maxRange;
  float maxVelocity;
};

configParamStruct configParams;

struct serialHandlerTaskParams {
  const String &input;
};

struct Frame {
  uint8_t* data;
  size_t length;
};

struct DetectedObject {
  int16_t rangeVal;
  int16_t dopplerVal;
  int16_t peakVal;
  int16_t x;
  int16_t y;
  int16_t z;
};

struct ProcessedFrame {
  uint32_t subFrameNum;
  uint32_t timeCpuCycles;
  uint32_t numDetectedObj;
  DetectedObject objects[128];
};

HardwareSerial debugSerial(0);
HardwareSerial cliSerial(1);

// Buffer variables & declarations
#define dataSerial            UART_NUM_2
#define dataSerial_RX         (GPIO_NUM_4)
#define dataSerial_BUF_SIZE   (32768)
#define FRAME_BUFFER_SIZE     32768
#define FRAME_POOL_SIZE       4
extern uint8_t framePool[FRAME_POOL_SIZE][FRAME_BUFFER_SIZE];
extern Frame frameStructs[FRAME_POOL_SIZE];
extern QueueHandle_t freeFrameQueue; // Available slots in framePool
extern QueueHandle_t parseFrameQueue; // Frames ready to parse in framePool
extern QueueHandle_t streamQueue; // Frames ready to stream out