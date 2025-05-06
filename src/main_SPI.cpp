#include <Arduino.h>
#include <SPI.h>

#define SPI_FREQ 1000000
#define PGOOD 20 
#define NRST 7 
#define WARMRST 35
#define READY_PIN 2 
#define SOP_0 14
#define SOP_1 36
#define SOP_2 9
#define SS 10
#define MOSI 11
#define CLK 12
#define MISO 13

// Unused but connected pins
#define AWR_IO1 16
#define HOSTINT 8
#define PMIC_EN 37
#define SYNC_OUT 48
#define SYNC_IN 47
#define AWR_IO0 21
#define NERRIN 19

volatile bool radar_ready = false; // Flag to indicate if radar TX buffer if filled
String curr_setting = "functional"; // Indicates current radar board setting
bool radar_state = false; // True when PGOOD = HIGH, false when PGOOD = LOW

TaskHandle_t spiTaskHandle = NULL;
TaskHandle_t serialTaskHandle = NULL;

SPIClass *hspi = NULL;

const uint16_t SPI_DATA_BLOCK_SIZE = 128;
uint8_t spi_rx_buf[SPI_DATA_BLOCK_SIZE];

void IRAM_ATTR onRadarReady() {
  Serial.println("Radar ready interrupt triggered.");
  radar_ready = true;
  detachInterrupt(READY_PIN);
}

bool serialInputHandler(String setting);
void radarReset(uint8_t reset_type);
void checkSerialInput(void * params);
void radarSPI(void * params);
void checkRadarState();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // Start the serial communication at 115200 baud rate
  delay(1000);

  // Used pins
  pinMode(MISO, INPUT);
  pinMode(SS, OUTPUT);
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

  // Set radar board to functional mode
  digitalWrite(SOP_0, LOW);
  digitalWrite(SOP_1, LOW);
  digitalWrite(SOP_2, HIGH);

  // Setup SPI
  hspi = new SPIClass(HSPI);
  hspi->begin(CLK, MISO, MOSI, SS); // SCK, MISO, MOSI, SS

  attachInterrupt(READY_PIN, onRadarReady, RISING); // Attach interrupt to the ready pin

  digitalWrite(SS, HIGH); // Deselect slave
  
  Serial.println("Resetting radar...");
  radarReset(0); // Hardware reset
  Serial.println("Radar is ready. Starting SPI transfer...");
  xTaskCreate(radarSPI, "radarSPI", 3072, NULL, 1, &spiTaskHandle);
  xTaskCreate(checkSerialInput, "checkSerial", 2048, NULL, 1, &serialTaskHandle);
}

void loop() {}

void radarSPI(void * params) {
  while(radar_state) {
    if (radar_ready) {
      radar_ready = false;

      // Start SPI transaction
      digitalWrite(SS, LOW);
      delayMicroseconds(2);

      hspi->beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
      hspi->transferBytes(NULL, spi_rx_buf, SPI_DATA_BLOCK_SIZE);
      hspi->endTransaction();

      digitalWrite(SS, HIGH); // Deselect radar

      // Print whole buffer for debug
      Serial.print("Start RX Buffer:\n");
      for (int j = 0; j < 8; j++) {
          for (int i = 0; i < 16; i++) {
            int k = j * 16 + i;
            // if (k >= SPI_DATA_BLOCK_SIZE) {break;} // Avoid out of bounds access
            Serial.printf("%02X ", spi_rx_buf[k]);
          }
          Serial.println();
        delayMicroseconds(100);
      }
      Serial.println();
      Serial.println("End RX Buffer");
    
    
      // // Print raw bytes for debug
      // Serial.print("Raw SPI data: ");
      // for (int i = 0; i < 16; i++) {
      //   Serial.printf("%02X ", spi_rx_buf[i]);
      // }
      // Serial.println();

      // // Parse exactly once
      // uint16_t msgID   = (spi_rx_buf[0] << 8) | spi_rx_buf[1];
      // uint16_t seqNo   = (spi_rx_buf[2] << 8) | spi_rx_buf[3];
      // uint16_t dataLen = (spi_rx_buf[4] << 8) | spi_rx_buf[5];

      // Serial.printf("Parsed: MsgID=0x%04X, SeqNo=%u, DataLen=%u\n", msgID, seqNo, dataLen);
      
      attachInterrupt(READY_PIN, onRadarReady, RISING); // Re-enable interrupt
    
    }
    delay(1);
  }
  Serial.println("Radar SPI task terminated.");
  vTaskDelete(NULL);
}

// Function to handle serial input commands. Returns true if successful, false otherwise.
bool serialInputHandler(String setting) {
  // Functional mode [001]
  if (setting == "func") {
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
    Serial.println("Restarting SPI task");
    xTaskCreate(radarSPI, "radarSPI", 3072, NULL, 1, &spiTaskHandle);
    return true;
  }
  // Debug mode [011]
  else if (setting == "debug") {
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
    Serial.println("Restarting SPI task");
    xTaskCreate(radarSPI, "radarSPI", 3072, NULL, 1, &spiTaskHandle);
    return true;
  }
  // Flash mode [101]
  else if (setting == "flash") {
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
    return true;
  }
  // Print setting
  else if (setting == "check") {
    Serial.print("Current radar board setting: ");
    Serial.println(curr_setting);
    return true;
  }
  // Hardware reset
  else if (setting == "hardrst") {
    radarReset(0); // Hardware reset
    return true;
  }
  // Software reset
  else if (setting == "softrst") {
    radarReset(1); // Software reset
    return true;
  }
  else {
    Serial.println("Invalid setting. Use 'func', 'debug', 'flash', 'check', 'hardrst' or 'softrst'.");
    return false;
  }
}

// Function to reset the radar. Takes a reset type (0 for hardware, 1 for software) as input.
void radarReset(uint8_t reset_type) {
  if (reset_type == 0) { // Hardware reset
    radar_state = false;
    digitalWrite(NRST, LOW); 
    delay(1000);
    digitalWrite(NRST, HIGH);
    checkRadarState();
    Serial.println("Radar hardware reset completed.");
  } 
  else if (reset_type == 1) { // Software reset
    if(!radar_state) {checkRadarState();}
    digitalWrite(WARMRST, LOW);
    delay(1000); 
    digitalWrite(WARMRST, HIGH); 
    Serial.println("Radar software reset completed.");
  }
}

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
}