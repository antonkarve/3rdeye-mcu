#include <Arduino.h>
#include <configs.h>

// Actively used pins
#define PGOOD 20 
#define NRST 7 
#define WARMRST 35
#define READY_PIN 2 
#define SOP_0 14
#define SOP_1 36
#define SOP_2 9
#define CLI_TX 17
#define CLI_RX 18
#define MSS_RX 4

// Unused but connected pins
#define AWR_IO1 16
#define HOSTINT 8
#define PMIC_EN 37
#define SYNC_OUT 48
#define SYNC_IN 47
#define AWR_IO0 21
#define NERRIN 19
#define SS 10
#define MOSI 11
#define CLK 12
#define MISO 13

// Global variables
// TODO may need to manage with Mutex/Sephamore if multi-core processing is implemented
volatile bool radar_ready = false; // True when radar TX buffer is filled.
String curr_setting = "functional"; // Indicates current radar board setting.
bool radar_state = false; // True when radar power supply is stable (PGOOD = HIGH).
bool radar_config = false; // True when a config has been loaded into the radar board.
RADAR_CONFIG current_config; // Reference to current config being used (within /include/configs.h)

TaskHandle_t serialTaskHandle = NULL;

HardwareSerial cliSerial(1);
HardwareSerial dataSerial(2);

void radar_begin(const RADAR_CONFIG &config);
bool serialInputHandler(const String &input);
void radarReset(uint8_t reset_type);
void checkSerialInput(void * params);
void radarSPI(void * params);
void checkRadarState();
void setPinModes();
// void mssHandler(void * params); // TODO after setting up CLI
// void parseData(String &packet); // TODO after setting up CLI
bool sendCommand(const String &command, unsigned long timeout = 1000);
void sendConfig(const RADAR_CONFIG &config);
// void updateConfig(String &update); // TODO after confirming how CLI sends confirmations back to ESP
void startRadar();
void stopRadar(bool flushCfg = false);
void cliTest();

void setup() {
  Serial.begin(115200);
  cliTest();
}

void loop() {}

void cliTest() {
  const RADAR_CONFIG &cfg = default_config;
  Serial.println("Testing radar_begin");
  radar_begin(cfg);
  Serial.println("radar_begin test complete.");
  delay(1000);

  Serial.println("Testing stopRadar with flushCfg == true");
  stopRadar(true);
  Serial.println("stopRadar with config flush test complete.");
  delay(1000);

  Serial.println("Testing startRadar with radar_config == false");
  startRadar();
  Serial.println("startRadar with config setup test complete.");
  delay(1000);

  Serial.println("Testing stopRadar with flushCfg == false");
  stopRadar();
  Serial.println("stopRadar test complete.");
  delay(1000);
}

void radar_begin(const RADAR_CONFIG &config) {
  Serial.println("radar_begin called");
  setPinModes();

  cliSerial.begin(115200, SERIAL_8N1, CLI_RX, CLI_TX);
  dataSerial.begin(921600, SERIAL_8N1, MSS_RX, -1);
  Serial.println("Initialised -- cliSerial: 115200, dataSerial: 921600");
  delay(1000);

  // Set radar board to functional mode
  digitalWrite(SOP_0, LOW);
  digitalWrite(SOP_1, LOW);
  digitalWrite(SOP_2, HIGH);
  
  Serial.println("Resetting radar...");
  radarReset(0); // Hardware reset
  sendConfig(config);

  xTaskCreate(checkSerialInput, "checkSerial", 2048, NULL, 1, &serialTaskHandle);
  Serial.println("You may start sending commands to the MCU via Terminal/Serial Monitor.");
  String input = String("help");
  serialInputHandler(input);
}

/* Sends a command to the radar and waits for confirmation. Returns true if successful, false otherwise.
Default timeout is 1000ms. Infinite timeout can be set by passing -1 in the timeout parameter.
Invalid timeout values (0 or negative) will set timeout to default.
Note: timeout value is not strict - time elapsed is measured after transmit attempts. */ 
bool sendCommand(const String &command, unsigned long timeout) {
  if (!radar_state) {checkRadarState();} // Ensure radar is stable before sending any commands

  if (timeout == 0) {
    Serial.println("Error: sendCommand timeout cannot be 0. Setting to default 1000ms.");
    timeout = 1000;
  }

  Serial.print("Sending command: ");
  Serial.print(command + "\n");
  cliSerial.print(command + "\n");

  // Wait for confirmation
  // TODO edit confirmation based on test results of CLI output
  const String expected_confirmation = "confirmation";
  String response;

  unsigned long start_time = millis();

  if (cliSerial.available()) {
    response = cliSerial.readString();
    Serial.println(response);
  }

  // while (response != expected_confirmation) {
  //   if (millis() - start_time > timeout && timeout > 0) {
  //     Serial.println("Error: Timeout waiting for sendCommand confirmation.");
  //     return true; // TODO change to false after confirming what CLI output looks like
  //   }

  //   // while (!cliSerial.available()) {delay(1);}
  //   response = cliSerial.readString(); // TODO change to readStringUntil("\n") if CLI returns \n delimiter
  //   // response.trim();
  //   Serial.println(response);
  //   delay(10);
  // }
  return true;
}

// Sends radar config over cliSerial line-by-line using sendCommand. Sets radar_config to true once completed.
void sendConfig(const RADAR_CONFIG &cfg) {
  Serial.print("Sending config: ");
  Serial.println(cfg.name);

  char cfgCopy[strlen(cfg.config) + 1];
  strcpy_P(cfgCopy, cfg.config);
  String cfgString = String(cfgCopy);

  int startidx = 0;
  int endidx = 0;
  Serial.println("Entering config loop");
  while (startidx < cfgString.length()) {
    Serial.print("startidx: ");
    Serial.print(startidx);
    endidx = cfgString.indexOf("\n", startidx);
    Serial.print(" endidx: ");
    Serial.println(endidx);
    if (endidx == -1) {
      Serial.println("Error with config file formatting: delimiter not found. Check /include/configs.h");
      break;
    }
    else if (endidx == 0) {
      startidx = endidx + 1;
      continue;
    }

    String command = cfgString.substring(startidx, endidx);
    Serial.print("Pre-trimmed command: ");
    Serial.println(command);

    command.trim();
    while (!sendCommand(command)){delay(10);}
    startidx = endidx + 1;
    delay(1);
  }

  radar_config = true;
  if (current_config.name != cfg.name) {current_config = cfg;}
  Serial.println("Radar configured and sensor started.");
}

// Sends sensorStart command over cliSerial using sendCommand. If config has been flushed, resends current_config.
void startRadar() {
  Serial.println("startRadar called");
  if (radar_config) {
    String command = "sensorStart";
    while (!sendCommand(command)){delay(10);}
    Serial.println("Radar sensor started.");
  } else {
    sendConfig(current_config);
  }
}

/* Sends sensorStop command over cliSerial using sendCommand.
Set flushCfg parameter to true to clear existing config. */
void stopRadar(bool flushCfg) {
  if (!radar_config) {
    Serial.println("Error: stopRadar cannot be called because radar has not been configured and started.");
    return;
  }
  Serial.println("stopRadar called");
  String command = "sensorStop";
  while (!sendCommand(command)){delay(10);}
  Serial.println("Radar sensor stopped.");
  if (flushCfg) {
    command = "flushCfg";
    while (!sendCommand(command)){delay(10);}
    Serial.println("Radar config flushed.");
    radar_config = false;
  }
  Serial.println("stopRadar complete");
}

// Handles serial input commands. Returns true if successful, false otherwise.
bool serialInputHandler(const String &input) {
  // Functional mode [001]
  if (input == "func") {
    if (curr_setting == "functional") {
      Serial.println("Radar board is already in functional mode.");
      return true;
    }
    digitalWrite(SOP_0, LOW);
    digitalWrite(SOP_1, LOW);
    digitalWrite(SOP_2, HIGH);
    radarReset(0);
    Serial.println("Radar board set to functional mode.");
    curr_setting = "functional";
    // Serial.println("Restarting mssHandler task");
  }
  // Debug mode [011]
  else if (input == "debug") {
    if (curr_setting == "debug") {
      Serial.println("Radar board is already in debug mode.");
      return true;
    }
    digitalWrite(SOP_0, LOW);
    digitalWrite(SOP_1, HIGH);
    digitalWrite(SOP_2, HIGH);
    radarReset(0);
    Serial.println("Radar board set to debug mode.");
    curr_setting = "debug";
    // Serial.println("Restarting mssHandler task");
  }
  // Flash mode [101]
  else if (input == "flash") {
    if (curr_setting == "flash") {
      Serial.println("Radar board is already in flash mode.");
      return true;
    }
    digitalWrite(SOP_0, HIGH);
    digitalWrite(SOP_1, LOW);
    digitalWrite(SOP_2, HIGH);
    radarReset(0);
    Serial.println("Radar board set to flash mode.");
    curr_setting = "flash";
  }
  // Print setting
  else if (input == "check") {
    Serial.print("Current radar board setting: ");
    Serial.println(curr_setting);
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
    Serial.println(R"(COMMAND LIST:
      'func': sets board to functional mode [001]
      'debug': sets board to debug mode [011]
      'flash': sets board to flash mode [101]
      'check': prints current board setting
      'hardrst': restarts radar hardware (full hardware & software restart using NRST)
      'softrst': restarts radar software (software-only restart using WARMRST)
      'help': prints full list of commands)");
  }
  else if (input == "start") {
    Serial.print("Starting radar.");
    startRadar();
  }
  else if (input == "stop") {
    Serial.print("Stopping radar.");
    stopRadar();
  }
  else {
    Serial.println("Invalid setting. Use 'help' for full commands list.");
    return false;
  }
  return true;
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
    Serial.println("Radar hardware reset completed.");
  } 
  else if (reset_type == 1) { // Software reset
    if(!radar_state) {checkRadarState();}
    stopRadar(true);
    digitalWrite(WARMRST, LOW);
    delay(1000); 
    digitalWrite(WARMRST, HIGH); 
    Serial.println("Radar software reset completed.");
  }
}

// Takes in commands from Terminal (must be connected to MCU via serial)
void checkSerialInput(void * params) {
  while(1) {
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim(); // Remove any leading/trailing whitespace
      serialInputHandler(input);
    }
    delay(100);
  }
}

// Blocks until PGOOD is HIGH
void checkRadarState() {
  Serial.println("Checking radar state (PGOOD)...");
  uint8_t i = 0;
  // Poll for 3 consecutive HIGH readings on PGOOD
  while (i < 3) {
    if (digitalRead(PGOOD) == HIGH) {
      i++;
    } 
    else {
      i = 0;
    }
    delay(100);
  }
  radar_state = true;
  Serial.println("Radar state stable.");
}

// Sets all designated pinModes.
void setPinModes() {
  // Used pins
  pinMode(PGOOD, INPUT); // PGOOD
  pinMode(NRST, OUTPUT_OPEN_DRAIN); // Radar hardware reset pin
  pinMode(WARMRST, OUTPUT_OPEN_DRAIN); // Radar software reset pin
  pinMode(READY_PIN, INPUT); // AWR ready signal
  pinMode(SOP_0, OUTPUT);
  pinMode(SOP_1, OUTPUT);
  pinMode(SOP_2, OUTPUT);

  // Unused but connected pins (set as input to prevent floating)
  pinMode(AWR_IO1, INPUT);
  pinMode(HOSTINT, INPUT);
  pinMode(PMIC_EN, INPUT);
  pinMode(SYNC_OUT, INPUT);
  pinMode(SYNC_IN, INPUT);
  pinMode(AWR_IO0, INPUT);
  pinMode(NERRIN, INPUT);
  pinMode(MISO, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(CLK, INPUT);
  pinMode(SS, INPUT);

  Serial.println("All pinModes set.");
}