#include <common.h>

uint8_t framePool[FRAME_POOL_SIZE][FRAME_BUFFER_SIZE];
Frame frameStructs[FRAME_POOL_SIZE];
QueueHandle_t freeFrameQueue; // Available slots in framePool
QueueHandle_t parseFrameQueue; // Frames ready to parse in framePool
QueueHandle_t streamQueue; // Frames ready to stream out

// Setup function prototypes
void radar_begin(const RADAR_CONFIG &config);
void checkSerialInput(void * params);
void checkRadarState();
void setPinModes();

// Data function prototypes
void mssHandler(void * params);
uint16_t bytesToUint16(const uint8_t* byte_array);
uint32_t bytesToUint32(const uint8_t* byte_array);
int16_t bytesToSignedInt16(const uint8_t* byte_array);
int findMagicWord(const uint8_t* buffer, size_t length);
void shiftBufferLeft(uint8_t* buffer, size_t& len, int processedBytes);

void parseData(void * params);
void parseDataFrame(uint8_t* data, size_t len, ProcessedFrame &processedFrame);
void parseObj(uint8_t* data, int& idx, uint16_t xyzQformat, DetectedObject &object);
void parseTLV(uint8_t* data, size_t len, int& idx, ProcessedFrame &processedFrame);
void parseObj(uint8_t* data, int& idx, uint16_t xyzQformat, DetectedObject &object);

void parseConfig(const RADAR_CONFIG &cfg);
void parseConfigLine(const String &configLine, uint8_t cfgType, int (&intarr)[8], float (&fltarr)[4]);
void calcConfigParams(int (&intarr)[8], float (&fltarr)[4]);
void parseConfigTask(void * params);

// CLI function prototypes
bool sendCommand(const String &command, unsigned long timeout = 1000);
void sendConfig(const RADAR_CONFIG &config);
// void updateConfig(String &update); // TODO after confirming how CLI sends confirmations back to ESP

// UI/prototyping function prototypes
void startRadar();
void stopRadar(bool flushCfg = false);
void radarReset(uint8_t reset_type);
void cliTest();
void serialInputHandler(void * pvParameters);
void streamData(void * params);
void radar_test(const RADAR_CONFIG &config);
void simulateDataStream(void * params);

bool stop_test = false;

void setup() {
  Serial.begin(115200);
  debugSerial.begin(115200);
  const RADAR_CONFIG &cfg = default_config;
  radar_test(default_config);

  // cliTest();
}

void loop() {}

void radar_test(const RADAR_CONFIG &config) {
  if (stop_test) {
    Serial.print("Hello! USB");
    debugSerial.print("Hello! UART");
    return;
  }

  debugSerial.println("radar_test called");
  setPinModes();
  current_config = config;

  xTaskCreate(parseConfigTask, "parseConfig", 4096, NULL, 1, &parseConfigTaskHandle);
  while (!configParams.numRangeBins) {delay(10);}

  // Create data frame queues and fill freeFrameQueue with pointers to frame pool slots
  freeFrameQueue = xQueueCreate(FRAME_POOL_SIZE, sizeof(Frame*));
  parseFrameQueue = xQueueCreate(FRAME_POOL_SIZE, sizeof(Frame*));
  streamQueue = xQueueCreate(5, sizeof(ProcessedFrame*));

  if (freeFrameQueue == nullptr) {
    debugSerial.println("[init] Error: freeFrameQueue not initialized");
  }

  for (int i = 0; i < FRAME_POOL_SIZE; i++) {
    frameStructs[i].data = framePool[i];
    // memset(framePool[i], 0xAA, FRAME_BUFFER_SIZE);
    frameStructs[i].length = 0;
    Frame* ptr = &frameStructs[i]; // THE CULPRIT
    if (xQueueSend(freeFrameQueue, &ptr, 0) != pdPASS) {
      debugSerial.printf("Failed to enqueue frameStructs[%d]\n", i);
      continue;
    }
    debugSerial.printf("[init] frameStructs[%d]: %p, framePool data: %p\n", i, &frameStructs[i], framePool[i]);
  }

  for (int i = 0; i < FRAME_POOL_SIZE; i++) {
    if (frameStructs[i].data == NULL) {
      debugSerial.printf("Init error: frameStructs[%d].data is NULL\n", i);
    }
  }

  debugSerial.println("Creating parseData and streamData tasks");
  xTaskCreatePinnedToCore(parseData, "parseData", 5120, NULL, 2, &dataParsingTaskHandle, 1); // I'm sorry i ever doubted you
  xTaskCreatePinnedToCore(streamData, "streamData", 3072, NULL, 1, &dataStreamingTaskHandle, 1);

  delay(100);

  debugSerial.println("Starting data stream...");
  xTaskCreatePinnedToCore(simulateDataStream, "simStream", 4096, NULL, 1, NULL, 0);
  // simulateDataStream();
}

void streamData(void * params) {
  debugSerial.println("streamData started");
  Serial.println("streamData started");
  while (true) {
    ProcessedFrame* frame;
    if (xQueueReceive(streamQueue, &frame, portMAX_DELAY)) {
      debugSerial.printf("[streamData] FRAME,%lu,%lu,%lu\n", frame->subFrameNum, frame->timeCpuCycles, frame->numDetectedObj);
      Serial.printf("FRAME,%lu,%lu,%lu\n", frame->subFrameNum, frame->timeCpuCycles, frame->numDetectedObj);

      for (uint32_t i = 0; i < frame->numDetectedObj; ++i) {
        const DetectedObject &obj = frame->objects[i];
        Serial.printf("OBJ,%d,%d,%d,%d,%d,%d\n",
          obj.rangeVal,
          obj.dopplerVal,
          obj.peakVal,
          obj.x,
          obj.y,
          obj.z
        );
      }
      delete frame;
    } else {
      debugSerial.println("streamData failed to receive from streamQueue");
    }
    UBaseType_t watermark = uxTaskGetStackHighWaterMark(NULL);
    debugSerial.printf("streamData stack remaining: %u bytes\n", watermark * sizeof(StackType_t));
  }
}

void parseData(void * params) {
  debugSerial.println("parseData started");
  while (true) {
    Frame* framePtr;

    if (xQueueReceive(parseFrameQueue, &framePtr, portMAX_DELAY)) {
      ProcessedFrame processedFrame;
      uint8_t* data = framePtr->data;
      size_t len = framePtr->length;
      parseDataFrame(data, len, processedFrame);

      // Return raw data frame to freeFrameQueue
      if (framePtr->data == NULL) {
        debugSerial.println("Warning [parseData]: framePtr->data is NULL before enqueue");
      }

      xQueueSend(freeFrameQueue, &framePtr, portMAX_DELAY);
      debugSerial.println("[parseData] Enqueued framePtr into freeFrameQueue");

      // Copy processed frame and send to streamQueue
      ProcessedFrame* frameCopy = new ProcessedFrame(processedFrame); // Copy contents
      if (frameCopy->objects == NULL) {
        debugSerial.println("Warning [parseData]: frameCopy->objects is NULL before enqueue");
      }
      xQueueSend(streamQueue, &frameCopy, portMAX_DELAY);
      debugSerial.println("[parseData] Enqueued frameCopy into streamQueue");
      UBaseType_t watermark = uxTaskGetStackHighWaterMark(NULL);
      debugSerial.printf("parseData stack remaining: %u bytes\n", watermark * sizeof(StackType_t));
    } else {
      debugSerial.println("parseData failed to receive from parseFrameQueue");
    }
    delay(1);
  }
}

void parseDataFrame(uint8_t* data, size_t len, ProcessedFrame &processedFrame) {
  int idx = 8;

  // Read header
  uint32_t version, totalPacketLen, platform, frameNumber, timeCpuCycles, numDetectedObj, numTLVs, subFrameNumber;
  version = bytesToUint32(data + idx);
  idx += 4;
  totalPacketLen = bytesToUint32(data + idx);
  if (totalPacketLen != len) {debugSerial.println("Error: totalPacketLen mismatch between mssHandler and parseDataFrame");}
  idx += 4;
  platform = bytesToUint32(data + idx);
  idx += 4;
  frameNumber = bytesToUint32(data + idx);
  idx += 4;
  timeCpuCycles = bytesToUint32(data + idx);
  idx += 4;
  numDetectedObj = bytesToUint32(data + idx);
  idx += 4;
  numTLVs = bytesToUint32(data + idx);
  idx += 4;
  subFrameNumber = bytesToUint32(data + idx);
  idx += 4;

  processedFrame.numDetectedObj = numDetectedObj;
  processedFrame.subFrameNum = subFrameNumber;
  processedFrame.timeCpuCycles = timeCpuCycles;

  // Only parse further if objects are detected
  if (numDetectedObj > 0) {
    const uint32_t MMWDEMO_UART_MSG_DETECTED_POINTS = 1;
    const uint32_t MMWDEMO_UART_MSG_RANGE_PROFILE = 2;

    // Loop through all TLVs and parse only the TLV containing detected points info
    for (uint32_t tlvIdx = 0; tlvIdx < numTLVs; ++tlvIdx) {
      uint32_t tlv_type, tlv_length;
      tlv_type = bytesToUint32(data + idx);
      idx += 4;
      tlv_length = bytesToUint32(data + idx);
      idx += 4;
      if (idx + tlv_length <= len && tlv_type == MMWDEMO_UART_MSG_DETECTED_POINTS) {
        parseTLV(data, len, idx, processedFrame);
      }
      delay(1);
    }
  }
}

void parseTLV(uint8_t* data, size_t len, int& idx, ProcessedFrame &processedFrame) {
  debugSerial.println("parseTLV called");
  uint16_t tlv_numObj, tlv_xyzQFormat;
  DetectedObject* objectList = processedFrame.objects;

  tlv_numObj = bytesToUint16(data + idx);
  idx += 2;
  tlv_xyzQFormat = bytesToUint16(data + idx);
  idx += 2;
  for (uint16_t objIdx = 0; objIdx < tlv_numObj; ++objIdx) {
    parseObj(data, idx, tlv_xyzQFormat, objectList[objIdx]);
    delay(1);
  }
}

void parseObj(uint8_t* data, int& idx, uint16_t xyzQformat, DetectedObject &object) {
  debugSerial.println("parseObj called");
  int16_t rangeIdx, dopplerIdx, x, y, z;

  // Parse raw object data
  rangeIdx = bytesToSignedInt16(data + idx);
  idx += 2;
  dopplerIdx = bytesToSignedInt16(data + idx);
  idx += 2;
  object.peakVal = bytesToSignedInt16(data + idx); // No processing needed, load directly into struct
  idx += 2;
  x = bytesToSignedInt16(data + idx);
  idx += 2;
  y = bytesToSignedInt16(data + idx);
  idx += 2;
  z = bytesToSignedInt16(data + idx);
  idx += 2;
  
  // Process object data and load into DetectedObject struct
  object.rangeVal = rangeIdx * configParams.rangeIdxToMeters;
  if (dopplerIdx > configParams.numDopplerBins/2 - 1) {dopplerIdx -= 65535;} // Capture negative doppler values through 2's complement
  object.dopplerVal = dopplerIdx * configParams.dopplerResolutionMps;
  object.x = x / xyzQformat;
  object.y = y / xyzQformat;
  object.z = z / xyzQformat;
}

void simulateDataStream(void * params) {
  debugSerial.println("simulateDataStream called");
  const int chunkSize = 256;
  static uint8_t radar_rx_buf[dataSerial_BUF_SIZE];
  static size_t radar_rx_len = 0;
  int testidx = 0;

  const int totalLen = sizeof(testDataStream);
  int offset = 0;

  while (offset < totalLen) {
    size_t len = min(chunkSize, totalLen - offset);
    // debugSerial.printf("[simulateDataStream] len: %d, radar_rx_len: %d, offset: %d\n",len, radar_rx_len, offset);

    memcpy(radar_rx_buf + radar_rx_len, testDataStream + offset, len);
    radar_rx_len += len;
    offset += len;

    int startIdx = findMagicWord(radar_rx_buf, radar_rx_len);
    if (startIdx >= 0 && radar_rx_len - startIdx >= 16) {

      uint32_t totalPacketLen = bytesToUint32(radar_rx_buf + startIdx + 12);
      // debugSerial.print("Frame packet len: ");
      // debugSerial.println(totalPacketLen);

      if (radar_rx_len - startIdx >= totalPacketLen) {
        Frame* bufferPtr;
        if (xQueueReceive(freeFrameQueue, &bufferPtr, portMAX_DELAY)) {
          debugSerial.printf("[simulateDataStream] received bufferPtr: %p\n", bufferPtr);
          debugSerial.printf("[simulateDataStream] First byte of bufferPtr: 0x%02X\n", bufferPtr->data[0]);
          if (bufferPtr == nullptr) {
            debugSerial.println("[simulateDataStream] Error: bufferPtr is NULL");
            continue;
          }
          if (bufferPtr->data == nullptr) {
            debugSerial.println("[simulateDataStream] Error: bufferPtr->data is NULL");
            continue;
          }
          if (totalPacketLen > FRAME_BUFFER_SIZE) {
            debugSerial.printf("Error: totalPacketLen (%u) exceeds buffer size!\n", totalPacketLen);
            continue;
          }
          memcpy(bufferPtr->data, radar_rx_buf + startIdx, totalPacketLen);
          bufferPtr->length = totalPacketLen;
          if (bufferPtr->data == NULL) {
            debugSerial.println("[simulateDataStream] Warning: bufferPtr->data is NULL before enqueue");
          }
          xQueueSend(parseFrameQueue, &bufferPtr, portMAX_DELAY);
          debugSerial.printf("[simulateDataStream] Enqueued bufferPtr. testidx: %d\n", testidx);
          testidx += 1;

          debugSerial.printf("[simulateDataStream] Pre-shift: startIdx: %d, totalPacketLen: %d, radar_rx_len: %d\n", startIdx, totalPacketLen, radar_rx_len);
          shiftBufferLeft(radar_rx_buf, radar_rx_len, startIdx + totalPacketLen);
          debugSerial.printf("[simulateDataStream] Post-shift: startIdx: %d, totalPacketLen: %d, radar_rx_len: %d\n", startIdx, totalPacketLen, radar_rx_len);
        } else {
          debugSerial.println("simulateDataStream failed to receive from freeFrameQueue");
        }
      }
    }

    delay(1);  // Optional: mimic real-world UART pacing
  }
  debugSerial.println("Simulation done. Self-deleting");
  vTaskDelete(NULL);
}

// Checks for magic word, gets packet length, copies complete radar frames into buffer pool
void mssHandler(void * params) {
  static uint8_t radar_rx_buf[dataSerial_BUF_SIZE];
  static size_t radar_rx_len = 0;

  while(true) {
    if (radar_started) {
      const uint32_t chunkSize = 512;
      size_t len = uart_read_bytes(dataSerial, radar_rx_buf + radar_rx_len, chunkSize, 10/portTICK_PERIOD_MS);
      radar_rx_len += len;
      if (radar_rx_len >= dataSerial_BUF_SIZE) {
        debugSerial.println("Radar RX buffer overflow. Resetting buffer.");
        radar_rx_len = 0;
      }
      delay(1);
      int startIdx = findMagicWord(radar_rx_buf, radar_rx_len);
      if (startIdx >= 0) {
        shiftBufferLeft(radar_rx_buf, radar_rx_len, startIdx);
        uint32_t totalPacketLen = bytesToUint32(radar_rx_buf + 12);
        debugSerial.print("Frame packet len: ");
        debugSerial.println(totalPacketLen);
        if (radar_rx_len >= totalPacketLen && radar_rx_len != 0) {
          Frame* bufferPtr;
          if (xQueueReceive(freeFrameQueue, &bufferPtr, portMAX_DELAY)) {
            memcpy(bufferPtr->data, radar_rx_buf + startIdx, totalPacketLen);
            bufferPtr->length = totalPacketLen;
            xQueueSend(parseFrameQueue, &bufferPtr, portMAX_DELAY);
          } else {
            debugSerial.println("mssHandler failed to receive from freeFrameQueue");
          }
        }
      }
    }
    delay(10);
  }
}

// Helper function for mssHandler & parseData
// Converts 2 bytes to uint16_t
uint16_t bytesToUint16(const uint8_t* byte_array) {
  return (static_cast<uint16_t>(byte_array[1]) << 8) | byte_array[0];
}
// Helper function for mssHandler & parseData
// Converts 4 bytes to uint32_t
uint32_t bytesToUint32(const uint8_t* byte_array) {
  return  (static_cast<uint32_t>(byte_array[3]) << 24) |
            (static_cast<uint32_t>(byte_array[2]) << 16) |
            (static_cast<uint32_t>(byte_array[1]) << 8)  |
            static_cast<uint32_t>(byte_array[0]);;
}
// Helper function for mssHandler & parseData
// Converts 2 bytes to int16_t (SIGNED)
int16_t bytesToSignedInt16(const uint8_t* byte_array) {
  return (int16_t)((byte_array[0] << 8) | byte_array[1]);
}

// Helper function for mssHandler
// Searches through given buffer array for magic word
int findMagicWord(const uint8_t* buffer, size_t length) {
  debugSerial.println("findMagicWord called");
  const uint8_t magicWord[8] = {2,1,4,3,6,5,8,7};
  if (length < 8) {return -1;}
  for (size_t i = 0; i <= length-8; ++i) {
    if (memcmp(&buffer[i], magicWord, 8) == 0) {
      debugSerial.printf("Magic word found at index %d: %02X %02X %02X %02X %02X %02X %02X %02X\n", 
        i, buffer[i], buffer[i+1], buffer[i+2], buffer[i+3],
        buffer[i+4], buffer[i+5], buffer[i+6], buffer[i+7]);
      return i;
    }
    delay(1);
  }
  return -1;
}

// Copies everything after buffer[processedBytes] to buffer[0]
void shiftBufferLeft(uint8_t* buffer, size_t& len, int processedBytes) {
  debugSerial.println("shiftBufferLeft called");
  if (processedBytes < len) {
    size_t remaining = len - processedBytes;
    memmove(buffer, buffer + processedBytes, remaining);
    len = remaining;
  } else {
    len = 0;
  }
}

void cliTest() {
  const RADAR_CONFIG &cfg = default_config;
  debugSerial.println("Testing radar_begin");
  radar_begin(cfg);
  debugSerial.println("radar_begin test complete.");
  delay(1000);

  debugSerial.println("Testing stopRadar with flushCfg == true");
  stopRadar(true);
  debugSerial.println("stopRadar with config flush test complete.");
  delay(1000);

  debugSerial.println("Testing startRadar with radar_config == false");
  startRadar();
  debugSerial.println("startRadar with config setup test complete.");
  delay(1000);

  debugSerial.println("Testing stopRadar with flushCfg == false");
  stopRadar();
  debugSerial.println("stopRadar test complete.");
  delay(1000);
}


void radar_begin(const RADAR_CONFIG &config) {
  debugSerial.println("radar_begin called");
  setPinModes();

  current_config = config;
  xTaskCreate(parseConfigTask, "parseConfig", 8192, NULL, 1, &parseConfigTaskHandle);

  // Create data frame queues and fill freeFrameQueue with pointers to frame pool slots
  freeFrameQueue = xQueueCreate(FRAME_POOL_SIZE, sizeof(Frame*));
  parseFrameQueue = xQueueCreate(FRAME_POOL_SIZE, sizeof(Frame*));
  streamQueue = xQueueCreate(5, sizeof(ProcessedFrame));

  for (int i = 0; i < FRAME_POOL_SIZE; i++) {
    frameStructs[i].data = framePool[i];
    frameStructs[i].length = 0;
    Frame* frameStructsPtr = &frameStructs[i];
    xQueueSend(freeFrameQueue, frameStructsPtr, 0);
  }

  // Set up Serial lines
  cliSerial.begin(115200, SERIAL_8N1, CLI_RX, CLI_TX);

  // Setup dataSerial with ESP-IDF to take advantage of DMA-backed RX
  uart_config_t dataSerial_config = {
    .baud_rate = 921600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };

  uart_param_config(dataSerial, &dataSerial_config);
  uart_set_pin(dataSerial, UART_PIN_NO_CHANGE, dataSerial_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(dataSerial, dataSerial_BUF_SIZE, 0, 0, NULL, 0);

  debugSerial.println("Initialised -- cliSerial: 115200, dataSerial: 921600");
  delay(1000);

  // Set radar board to functional mode
  digitalWrite(SOP_0, LOW);
  digitalWrite(SOP_1, LOW);
  digitalWrite(SOP_2, HIGH);
  
  debugSerial.println("Resetting radar...");
  radarReset(0); // Hardware reset
  sendConfig(config);
  xTaskCreate(mssHandler, "mssHandler", 4096, NULL, 4, &dataLoggingTaskHandle);

  xTaskCreate(checkSerialInput, "checkSerialInput", 2048, NULL, 1, &checkSerialTaskHandle);
  debugSerial.println("You may start sending commands to the MCU via Terminal/Serial Monitor. Type 'help' for the commands list.");
}

/* Sends a command to the radar and waits for confirmation. Returns true if successful, false otherwise.
Default timeout is 1000ms. Infinite timeout can be set by passing -1 in the timeout parameter.
Invalid timeout values (0 or negative) will set timeout to default.
Note: timeout value is not strict - time elapsed is measured after transmit attempts. */ 
bool sendCommand(const String &command, unsigned long timeout) {
  if (!radar_state) {checkRadarState();} // Ensure radar is stable before sending any commands

  if (timeout == 0) {
    debugSerial.println("Error: sendCommand timeout cannot be 0. Setting to default 1000ms.");
    timeout = 1000;
  }

  debugSerial.print("Sending command: ");
  debugSerial.println(command);
  cliSerial.println(command);

  // Wait for confirmation
  static const String expected_response_1 = "Skipped";
  static const String expected_response_2 = "Done";
  String response;

  unsigned long start_time = millis();

  while (response != expected_response_1 && response != expected_response_2) {
    if (millis() - start_time > timeout && timeout > 0) {
      debugSerial.println("Error: Timeout waiting for sendCommand confirmation.");
      return false;
    }

    // while (!cliSerial.available()) {delay(1);}
    response = cliSerial.readStringUntil('\n');
    response.trim();
    debugSerial.println(response);
    delay(10);
  }
  return true;
}

// Sends radar config over cliSerial line-by-line using sendCommand. Sets radar_config to true once completed.
void sendConfig(const RADAR_CONFIG &cfg) {
  debugSerial.print("Sending config: ");
  debugSerial.println(cfg.name);

  char cfgCopy[strlen(cfg.config) + 1];
  strcpy_P(cfgCopy, cfg.config);
  String cfgString = String(cfgCopy);

  int startidx = 0;
  int endidx = 0;
  debugSerial.println("Entering config loop");
  String command;
  while (startidx < cfgString.length()) {
    debugSerial.print("startidx: ");
    debugSerial.print(startidx);
    endidx = cfgString.indexOf("\n", startidx);
    debugSerial.print(" endidx: ");
    debugSerial.println(endidx);
    if (endidx == -1) {
      debugSerial.println("Error with config file formatting: delimiter not found. Check /include/configs.h");
      break;
    }
    else if (endidx == 0) {
      startidx = endidx + 1;
      continue;
    }

    command = cfgString.substring(startidx, endidx);
    debugSerial.print("Pre-trimmed command: ");
    debugSerial.println(command);

    command.trim();
    while (!sendCommand(command)){delay(10);}
    startidx = endidx + 1;
    delay(1);
  }

  radar_config = true;
  if (current_config.name != cfg.name) {current_config = cfg;}
  debugSerial.println("Radar configured and sensor started.");
}

// Runs parseConfig, deletes task after done
void parseConfigTask(void * params) {
  debugSerial.println("parseConfig called");
  parseConfig(current_config);
  UBaseType_t watermark = uxTaskGetStackHighWaterMark(NULL);
  debugSerial.printf("parseConfig stack remaining: %u bytes\n", watermark * sizeof(StackType_t));
  debugSerial.println("Config parsed.");
  vTaskDelete(NULL);
}

/* Parse radar config line-by-line with helper parseConfigLine. 
Calls calcConfigParams to process params and load into configParams map. */
void parseConfig(const RADAR_CONFIG &cfg) {
  char cfgCopy[strlen(cfg.config) + 1];
  strcpy_P(cfgCopy, cfg.config);
  String cfgString = String(cfgCopy);

  int startidx = 0;
  int endidx = 0;
  String configLine;
  int cfgIntArr[8];
  float cfgFloatArr[4];

  while (startidx < cfgString.length()) {
    endidx = cfgString.indexOf("\n", startidx);
    if (endidx == -1) {
      break;
    }
    else if (endidx == 0) {
      startidx = endidx + 1;
      continue;
    }

    configLine = cfgString.substring(startidx, endidx);
    configLine.trim();

    if (configLine.startsWith("profileCfg")) {
      // debugSerial.println("Found profileCfg line.");
      parseConfigLine(configLine, 0, cfgIntArr, cfgFloatArr);
    }
    else if (configLine.startsWith("frameCfg")) {
      // debugSerial.println("Found frameCfg line.");
      parseConfigLine(configLine, 1, cfgIntArr, cfgFloatArr);
    }
    
    startidx = endidx + 1;
    delay(1);
  }
  // debugSerial.println("Printing filled int array:");
  // for (uint8_t i = 0; i < 8; ++i) {
  //   debugSerial.print(cfgIntArr[i]);
  //   debugSerial.print(",");
  // }
  // delay(10);
  // debugSerial.println("\nPrinting filled float array:");
  // for (uint8_t i = 0; i < 4; ++i) {
  //   debugSerial.print(cfgFloatArr[i]);
  //   debugSerial.print(",");
  // }
  calcConfigParams(cfgIntArr, cfgFloatArr);
}

// Parses individual config line and loads data into int/float arrays. Called by parseConfig
void parseConfigLine(const String &configLine, uint8_t cfgType, int (&intarr)[8], float (&fltarr)[4]) {
  // Based on RPi implementation for mmWave SDK 2
  // debugSerial.print("Parsing: ");
  // debugSerial.println(configLine);
  // Profile config variables
  uint16_t startFreq, idleTime, numAdcSamples, digOutSampleRate;
  float rampEndTime, freqSlopeConst;
  int numAdcSamplesRoundTo2 = 1;

  // Frame config variables
  uint16_t chirpStartIdx, chirpEndIdx, numLoops, numFrames, framePeriodicity;

  // Parse profile configuration (cfgType == 0)
  if (cfgType == 0) {
    int startIdx = 0;
    int endIdx = 0;

    for (uint8_t i = 0; i < 12; i++) {
      endIdx = configLine.indexOf(" ", startIdx);
      if (i == 2) {debugSerial.println(configLine.substring(startIdx, endIdx));
        startFreq = configLine.substring(startIdx, endIdx).toInt();}
      else if (i == 3) {idleTime = configLine.substring(startIdx, endIdx).toInt();}
      else if (i == 5) {rampEndTime = configLine.substring(startIdx, endIdx).toFloat();}
      else if (i == 8) {freqSlopeConst = configLine.substring(startIdx, endIdx).toFloat();}
      else if (i == 10) {numAdcSamples = configLine.substring(startIdx, endIdx).toInt();}
      else if (i == 11) {digOutSampleRate = configLine.substring(startIdx, endIdx).toInt();}
      startIdx = endIdx + 1;
      delay(10);
    }
    while (numAdcSamples > numAdcSamplesRoundTo2) {
      numAdcSamplesRoundTo2 = numAdcSamplesRoundTo2 * 2;
      delay(1);
    }

    // Load into arrays
    // debugSerial.printf("startFreq %d, idleTime %d, numAdcSamples %d, numAdcSamplesRoundTo2 %d, digOutSampleRate %d, rampEndTime %f, freqSlopeConst %f\n",
    //   startFreq, idleTime, numAdcSamples, numAdcSamplesRoundTo2, digOutSampleRate, rampEndTime, freqSlopeConst);
    intarr[0] = startFreq;
    intarr[1] = idleTime;
    intarr[2] = numAdcSamples;
    intarr[3] = numAdcSamplesRoundTo2;
    intarr[4] = digOutSampleRate;

    fltarr[0] = rampEndTime;
    fltarr[1] = freqSlopeConst;
  }
  else {
    // Frame config variables (cfgType == 1)
    int startIdx = 0;
    int endIdx = 0;
    for (uint8_t i = 0; i < 6; i++) {
      endIdx = configLine.indexOf(" ", startIdx);
      if (i == 1) {chirpStartIdx = configLine.substring(startIdx, endIdx).toInt();}
      else if (i == 2) {chirpEndIdx = configLine.substring(startIdx, endIdx).toInt();}
      else if (i == 3) {numLoops = configLine.substring(startIdx, endIdx).toFloat();}
      else if (i == 4) {numFrames = configLine.substring(startIdx, endIdx).toFloat();}
      else if (i == 5) {framePeriodicity = configLine.substring(startIdx, endIdx).toInt();}
      startIdx = endIdx + 1;
      delay(10);
    }

    // Load into arrays
    intarr[5] = chirpStartIdx;
    intarr[6] = chirpEndIdx;
    intarr[7] = framePeriodicity;

    fltarr[2] = numLoops;
    fltarr[3] = numFrames;
  }
}

/* Int/float array references
intarr[0] = startFreq;
intarr[1] = idleTime;
intarr[2] = numAdcSamples;
intarr[3] = numAdcSamplesRoundTo2;
intarr[4] = digOutSampleRate;

fltarr[0] = rampEndTime;
fltarr[1] = freqSlopeConst;

intarr[5] = chirpStartIdx;
intarr[6] = chirpEndIdx;
intarr[7] = framePeriodicity;

fltarr[2] = numLoops;
fltarr[3] = numFrames; 
*/

// Calculates processed config params from data in int/float arrays. Called by parseConfig
void calcConfigParams(int (&intarr)[8], float (&fltarr)[4]) {
  uint8_t numRxAnt = 4;
  uint8_t numTxAnt = 2;

  // See comment above calcConfigParams function for array index references
  int numChirpsPerFrame = (intarr[6] - intarr[5] + 1) * fltarr[2];
  configParams.numDopplerBins = numChirpsPerFrame / numTxAnt;
  configParams.numRangeBins = intarr[3];
  configParams.rangeResolutionMeters = float(3e8 * intarr[4] * 1e3) / float(2 * fltarr[1] * 1e12 * intarr[2]);
  configParams.rangeIdxToMeters = float(3e8 * intarr[4] * 1e3) / float(2 * fltarr[1] * 1e12 * intarr[3]);
  configParams.dopplerResolutionMps = float(3e8) / float(2 * intarr[0] * 1e9 * float(intarr[1] + fltarr[0]) * 1e-6 * (numChirpsPerFrame / numTxAnt) * numTxAnt);
  configParams.maxRange = float(300 * 0.9 * intarr[4])/float(2 * fltarr[1] * 1e3);
  configParams.maxVelocity = float(3e8) / float(4 * intarr[0] * 1e9 * (intarr[1] + fltarr[0]) * 1e-6 * numTxAnt);
}
    
// Sends sensorStart command over cliSerial using sendCommand. If config has been flushed, resends current_config.
void startRadar() {
  debugSerial.println("startRadar called");
  if (radar_config) {
    String command = "sensorStart 0";
    while (!sendCommand(command)){delay(10);}
    debugSerial.println("Radar sensor started.");
  } else {
    sendConfig(current_config);
  }
  radar_started = true;
}

/* Sends sensorStop command over cliSerial using sendCommand.
Set flushCfg parameter to true to clear existing config. */
void stopRadar(bool flushCfg) {
  if (!radar_config) {
    debugSerial.println("Error: stopRadar cannot be called because radar has not been configured and started.");
    return;
  }
  debugSerial.println("stopRadar called");
  String command = "sensorStop";
  while (!sendCommand(command)){delay(10);}
  debugSerial.println("Radar sensor stopped.");
  if (flushCfg) {
    command = "flushCfg";
    while (!sendCommand(command)){delay(10);}
    debugSerial.println("Radar config flushed.");
    radar_config = false;
  }
  debugSerial.println("stopRadar complete");
  radar_started = false;
}

// Handles serial input commands.
void serialInputHandler(void * pvParameters) {
  serialHandlerTaskParams* params = (serialHandlerTaskParams*)pvParameters;
  String input = params->input;

  // Functional mode [001]
  if (input == "func") {
    if (curr_setting == "functional") {
      debugSerial.println("Radar board is already in functional mode.");
    }
    digitalWrite(SOP_0, LOW);
    digitalWrite(SOP_1, LOW);
    digitalWrite(SOP_2, HIGH);
    radarReset(0);
    debugSerial.println("Radar board set to functional mode.");
    curr_setting = "functional";
    // debugSerial.println("Restarting mssHandler task");
  }
  // Debug mode [011]
  else if (input == "debug") {
    if (curr_setting == "debug") {
      debugSerial.println("Radar board is already in debug mode.");
    }
    digitalWrite(SOP_0, LOW);
    digitalWrite(SOP_1, HIGH);
    digitalWrite(SOP_2, HIGH);
    radarReset(0);
    debugSerial.println("Radar board set to debug mode.");
    curr_setting = "debug";
    // debugSerial.println("Restarting mssHandler task");
  }
  // Flash mode [101]
  else if (input == "flash") {
    if (curr_setting == "flash") {
      debugSerial.println("Radar board is already in flash mode.");
    }
    digitalWrite(SOP_0, HIGH);
    digitalWrite(SOP_1, LOW);
    digitalWrite(SOP_2, HIGH);
    radarReset(0);
    debugSerial.println("Radar board set to flash mode.");
    curr_setting = "flash";
  }
  // Print setting
  else if (input == "check") {
    debugSerial.print("Current radar board setting: ");
    debugSerial.println(curr_setting);
  }
  // Hardware reset
  else if (input == "hardrst") {
    radarReset(0); // Hardware reset
  }
  // Software reset
  else if (input == "softrst") {
    radarReset(1); // Software reset
  }
  // Help (print command list)
  else if (input == "help") {
    debugSerial.println(R"(COMMAND LIST:
      'func': sets board to functional mode [001]
      'debug': sets board to debug mode [011]
      'flash': sets board to flash mode [101]
      'check': prints current board setting
      'hardrst': restarts radar hardware (full hardware & software restart using NRST)
      'softrst': restarts radar software (software-only restart using WARMRST)
      'help': prints full list of commands)");
  }
  else if (input == "start") {
    debugSerial.print("Starting radar.");
    startRadar();
  }
  else if (input == "stop") {
    debugSerial.print("Stopping radar.");
    stopRadar();
  }
  else {
    debugSerial.println("Invalid setting. Use 'help' for full commands list.");
  }

  delete params;
  vTaskDelete(NULL);
}

// Resets the radar. Takes a reset type (0 for hardware, 1 for software) as input.
void radarReset(uint8_t reset_type) {
  if (reset_type == 0) { // Hardware reset
    if (radar_config) {
      stopRadar(true);
    }
    radar_state = false;
    digitalWrite(NRST, LOW); 
    delay(1000);
    digitalWrite(NRST, HIGH);
    checkRadarState();
    debugSerial.println("Radar hardware reset completed.");
  } 
  else if (reset_type == 1) { // Software reset
    if(!radar_state) {checkRadarState();}
    stopRadar(true);
    digitalWrite(WARMRST, LOW);
    delay(1000); 
    digitalWrite(WARMRST, HIGH); 
    debugSerial.println("Radar software reset completed.");
  }
}

// Takes in commands from Terminal (must be connected to MCU via serial)
void checkSerialInput(void * params) {
  while(1) {
    if (debugSerial.available()) {
      String input = debugSerial.readStringUntil('\n');
      input.trim(); // Remove any leading/trailing whitespace

      serialHandlerTaskParams* taskParams = new serialHandlerTaskParams{input};
      xTaskCreate(serialInputHandler, "serialInputHandler", 6144, (void*)taskParams, 2, &serialInputTaskHandle);
    }
    delay(100);
  }
}

// Blocks until PGOOD is HIGH (NOTE: currently set to just wait 600ms due to shield routing errors)
void checkRadarState() {
  debugSerial.println("Checking radar state (PGOOD)...");
  uint8_t i = 0;
  // Poll for 3 consecutive HIGH readings on PGOOD
  while (i < 3) {
    // if (digitalRead(PGOOD) == HIGH) {
    //   i++;
    // } 
    // else {
    //   i = 0;
    // }
    delay(200);
    i++;
  }
  radar_state = true;
  debugSerial.println("Radar state stable.");
}

// Sets all designated pinModes.
void setPinModes() {
  // Used pins
  // pinMode(PGOOD, INPUT); // PGOOD
  pinMode(NRST, OUTPUT_OPEN_DRAIN); // Radar hardware reset pin
  pinMode(WARMRST, OUTPUT_OPEN_DRAIN); // Radar software reset pin
  pinMode(SOP_0, OUTPUT);
  pinMode(SOP_1, OUTPUT);
  pinMode(SOP_2, OUTPUT);

  // Unused but connected pins (set as input to prevent floating)
  pinMode(AWR_IO0, INPUT);
  pinMode(AWR_IO1, INPUT);
  pinMode(AWR_IO2, INPUT);
  pinMode(HOSTINT, INPUT);
  pinMode(PMIC_EN, INPUT);
  pinMode(SYNC_OUT, INPUT);
  pinMode(SYNC_IN, INPUT);
  // pinMode(NERRIN, INPUT);
  pinMode(MISO, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(CLK, INPUT);
  pinMode(SS, INPUT);

  debugSerial.println("All pinModes set.");
}