#include <SPI.h>
#include <SparkFun_I2C_Expander_Arduino_Library.h>
#include "FS.h"
#include "SD_MMC.h"

// Error printing helpers
#define ERR_CAN_FAILED_TO_READ_BUFFER_STATUS 0x01
#define ERR_SD_FAILED_TO_OPEN_FILE 0x02
#define ERR_SD_FAILED_TO_WRITE_FILE 0x04
#define ERR_SD_FAILED_TO_DELETE_FILE 0x08
#define ERR_LOG_BUFFER_WRAP 0x10
#define ERR_CAN_FAILED_TO_READ_REGISTER 0x12
#define VERBOSE_ERR true
uint8_t errorRegister = 0;

// Mutex for critical section control in ISR
portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;

// Map INT and CS pins for MCP2515s
byte const canInterrupts[] = {42, 40, 48, 21, 13, 8};
byte const canSelects[] = {41, 39, 47, 14, 12, 18};

// GPIO Expander
SFE_PCA95XX io(PCA95XX_PCA9534);

// Task handles for pinning to separate cores
TaskHandle_t canTask;
TaskHandle_t loggingTask;

// Struct for the CAN frame buffer
// volatile because ISR can write them unexpectedly 
struct CANFrame{
  uint8_t volatile channel; 
  int64_t volatile timeStamp;
  uint8_t volatile raw[13];
};

// Buffer to write CAN traffic to
struct CANFrame bufferCAN[256];
uint8_t volatile wPtr = 0;
uint8_t volatile rPtr = 0;

// For Debugging
uint8_t wPtr_prev = 0;
uint8_t rPtr_prev = 0;
uint16_t SDpos_prev = 0;

// Buffer for writing to storage
char bufferSD[1024];
uint16_t SDpos = 0;

// Instantiate SPI Class
SPIClass *fspi = NULL;

// Flag to empty bufferSD to storage even if it's not full
bool endLogging = false;
// Lazy flag to break from serial menu loop
bool exitMenu = false;

// Stuff the ISRs into RAM
void IRAM_ATTR rxISR0() { rx(0); }
void IRAM_ATTR rxISR1() { rx(1); }
void IRAM_ATTR rxISR2() { rx(2); }
void IRAM_ATTR rxISR3() { rx(3); }
void IRAM_ATTR rxISR4() { rx(4); }
void IRAM_ATTR rxISR5() { rx(5); }

// Add an error code to the register for printing in VERBOSE mode
void ERR(uint8_t errorCode) {
  errorRegister = errorRegister | errorCode;
  return;
}

// Get RXBuffers from MCP2515 over SPI. Time critical. 
void rx(uint8_t channel) {
  taskENTER_CRITICAL(&myMutex);
  uint8_t IFRread[] = {0x03, 0x2C, 0xFF};
  // Read Interrupt Flag Register to find out if both buffers are full
  digitalWrite(canSelects[channel], 0);
  fspi->transfer(IFRread, 3);
  uint8_t intf = IFRread[2];
  digitalWrite(canSelects[channel], 1);
  // Read RXBn With READBUFFER command (n,m = 0,0 for RXB0. 1,0 for RXB1)
  // start pointer at RXBnSIDH and burst read 13 bytes
  // READBUFFER will clear RX flag when CS is raised
  if (intf & 0b00000010) {
    uint8_t rxb1read[] = {0x94, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                          0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    digitalWrite(canSelects[channel], 0); 
    fspi->transfer(rxb1read, 14);
    digitalWrite(canSelects[channel], 1);
    CANFrame newFrame;
    newFrame.channel = channel;
    for(uint8_t i = 1; i < 14; i++){
      newFrame.raw[i-1] = rxb1read[i];
    }
    bufferCAN[wPtr] = newFrame;
    wPtr++;
  }
  if (intf & 0b00000001) {
    uint8_t rxb0read[] = {0x90, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                          0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    digitalWrite(canSelects[channel], 0);
    fspi->transfer(rxb0read, 14);
    digitalWrite(canSelects[channel], 1);
    CANFrame newFrame;
    newFrame.channel = channel;
    for(uint8_t i = 1; i < 14; i++){
      newFrame.raw[i-1] = rxb0read[i];
    }
    bufferCAN[wPtr] = newFrame;
    wPtr++;
  }
  taskEXIT_CRITICAL(&myMutex);
  return;
}

/***SD Convenience Functions***/
void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);
  Serial.println("");

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    ERR(ERR_SD_FAILED_TO_OPEN_FILE);
    return;
  }
  if (file.print(message)) {
  } else {
    ERR(ERR_SD_FAILED_TO_WRITE_FILE);
  }
  file.close();
  return;
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    ERR(ERR_SD_FAILED_TO_OPEN_FILE);
    return;
  }
  if (file.print(message)) {
  } else {
    ERR(ERR_SD_FAILED_TO_WRITE_FILE);
  }
  file.close();
  return;
}

void readFile(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\n", path);
  Serial.println("");

  File file = fs.open(path);
  if (!file) {
    ERR(ERR_SD_FAILED_TO_OPEN_FILE);
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
  return;
}

void deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
  } else {
    ERR(ERR_SD_FAILED_TO_DELETE_FILE);
  }
  return;
}
/***************************/

/***SPI Convenience Functions***/
void writeRegister(uint8_t channel, uint8_t address, uint8_t value) {
  digitalWrite(canSelects[channel], 0);
  fspi->transfer(0x02);  // WRITE command
  fspi->transfer(address);
  fspi->transfer(value);
  digitalWrite(canSelects[channel], 1);
  return;
}

uint8_t readRegister(uint8_t channel, uint8_t address) {
  digitalWrite(canSelects[channel], 0);
  fspi->transfer(0x03);     // READ command
  fspi->transfer(address);  // register address
  uint8_t regval = fspi->transfer(0xFF);
  digitalWrite(canSelects[channel], 1);
  return regval;
}
/***************************/

/***MCP2515 Configuration***/
bool startCANController(uint8_t channel) {
  // Reset MCP2515
  digitalWrite(canSelects[channel], 0);
  fspi->transfer(0xC0);  // RESET command
  digitalWrite(canSelects[channel], 1);
  delayMicroseconds(10);  // Give it a few mics to reset

  // Test-write CANCTRL register
  digitalWrite(canSelects[channel], 0);
  fspi->transfer(0x02);  // WRITE command
  fspi->transfer(0x0F);  // CANCTRL register
  fspi->transfer(0x80);  // 0b10000000
  digitalWrite(canSelects[channel], 1);
  digitalWrite(canSelects[channel], 0);
  fspi->transfer(0x03);  // READ command
  fspi->transfer(0x0F);  // CANCTRL register
  uint8_t ctrlreg = fspi->transfer(0xFF);
  digitalWrite(canSelects[channel], 1);
  if (ctrlreg != 0x80) {
    ERR(ERR_CAN_FAILED_TO_READ_REGISTER);
    return 0;
  }

  // Configuration
  // Bit timing for 250kbit CAN with 20MHz Xtal
  // Calculated using Microchip CAN Bit Timing Calculator
  writeRegister(channel, 0x2A, 0x01); // CNF1 register
  writeRegister(channel, 0x29, 0xBA); // CNF2 register
  writeRegister(channel, 0x28, 0x07); // CNF3 register
  writeRegister(channel, 0x2B, 0b00000011); // CANINTE register = RX0IE|RX1IE (enable interrupts)
  writeRegister(channel, 0x0C, 0x00); // BFPCTRL register
  writeRegister(channel, 0x0D, 0x00); // TXRTSCTRL register
  writeRegister(channel, 0x60, 0b01100100); // RXB0CTRL = RXM1|RXM0|BUKT (rx all messages, enable rollover)
  writeRegister(channel, 0x70, 0b01100000); // RXB1CTRL = RXM1|RXM0 (rx all messages)

  // Reset CANCTRL register
  digitalWrite(canSelects[channel], 0);
  fspi->transfer(0x02);  // WRITE command
  fspi->transfer(0x0F);  // CANCTRL register
  fspi->transfer(0x00);  // 0b00000000
  digitalWrite(canSelects[channel], 1);
  digitalWrite(canSelects[channel], 0);
  fspi->transfer(0x03);  // READ command
  fspi->transfer(0x0F);  // CANCTRL register
  ctrlreg = fspi->transfer(0xFF);
  digitalWrite(canSelects[channel], 1);
  if (ctrlreg != 0x00) {
    ERR(ERR_CAN_FAILED_TO_READ_REGISTER);
    return 0;
  }

  return 1;
}

// Pretty-print the contents of the Error Register to Serial
void printErrors() {
  if (errorRegister & ERR_CAN_FAILED_TO_READ_BUFFER_STATUS) {
    Serial.println(
        "MCP2515 interrupt fired but no buffer flags were read. Possible SPI "
        "Error.");
  }
  if (errorRegister & ERR_SD_FAILED_TO_OPEN_FILE) {
    Serial.println("Failed to open file on storage.");
  }
  if (errorRegister & ERR_SD_FAILED_TO_WRITE_FILE) {
    Serial.println("Failed to write to file on storage.");
  }
  if (errorRegister & ERR_SD_FAILED_TO_DELETE_FILE) {
    Serial.println("Failed to delete file from storage.");
  }
  if (errorRegister & ERR_LOG_BUFFER_WRAP) {
    Serial.println("Log buffer has wrapped around.");
  }
  if (errorRegister & ERR_CAN_FAILED_TO_READ_REGISTER) {
    Serial.println("Register read from MCP2515 was not as expected. Possible "
    "SPI Error.");
  }
  errorRegister = 0;
  return;
}

// Initialize ALL THE THINGS
void setup() {
  // Create pinned tasks
  xTaskCreatePinnedToCore(logFromBuffer, // Task Function
                          "LoggingTask", // Task Name
                          10000,         // Stack Size (words)
                          NULL,          // Input Param
                          1,             // Priority
                          &loggingTask,  // Task Handle
                          0);            // Core where the task should run

  xTaskCreatePinnedToCore(canMonitor,   // Task Function
                          "CANMonitor", // Task Name
                          10000,        // Stack Size (words)
                          NULL,         // Input Param
                          1,            // Priority
                          &canTask,     // Task Handle
                          1);           // Core where the task should run

  Serial.begin(115200);
  Wire.begin(1, 2);

  // Set all SPI CS pins high (unasserted)
  for (uint8_t i = 0; i < 6; i++) {
    pinMode(canSelects[i], OUTPUT);
    digitalWrite(canSelects[i], 1);
  }

  // Start SPI 
  fspi = new SPIClass(FSPI);
  fspi->begin(11, 9, 10, -1);
  fspi->beginTransaction(SPISettings(10e6, MSBFIRST, SPI_MODE0));

  // Assign SD_MMC interface pins
  SD_MMC.setPins(6, 5, 7, 15, 16, 17);

  // Start SD_MMC interface
  if (!SD_MMC.begin()) {
    if (VERBOSE_ERR) {
      Serial.println("Storage Mount Failed");
    }
    return;
  }
  uint8_t cardType = SD_MMC.cardType();

  if (cardType == CARD_NONE) {
    if (VERBOSE_ERR) {
      Serial.println("No SD_MMC card attached");
    }
    return;
  }

  // Initialize the PCA9554 GPIO expander
  if (io.begin(0x3F) == false) {
    if (VERBOSE_ERR) {
      Serial.println("PCA95xx not detected");
    }
  }

  // Enable all MCP2515s via the GPIO expander
  for (uint8_t i = 0; i < 8; i++) {
    io.pinMode(i, OUTPUT);
    io.digitalWrite(i, 1);
  }

  // Configure and start each CAN tcvr
  for (uint8_t i = 0; i < 6; i++) {
    if (!startCANController(i)) {
      if (VERBOSE_ERR) {
        Serial.print("Error initializing MCP2515 Channel ");
        Serial.println(i + 1);
      }
    }
  }

  // register the receive callbacks
  attachInterrupt(canInterrupts[0], rxISR0, FALLING);
  attachInterrupt(canInterrupts[1], rxISR1, FALLING);
  attachInterrupt(canInterrupts[2], rxISR2, FALLING);
  attachInterrupt(canInterrupts[3], rxISR3, FALLING);
  attachInterrupt(canInterrupts[4], rxISR4, FALLING);
  attachInterrupt(canInterrupts[5], rxISR5, FALLING);

  // Start Logfile
  writeFile(SD_MMC, "/log.txt", "START\n");

  //DEBUG FOR TIMING
  pinMode(43, OUTPUT);
  digitalWrite(43, 0);

  return;
}

// This task is pinned to Core 0. Its job is to move bytes around
// and write to the storage. 
void logFromBuffer(void *parameter) {
  for (;;) {
    if (rPtr != wPtr) {
      char rawtoa[64];
      uint8_t rawidx = 0;
      char buf[3];
      for (uint8_t tmpidx = 0; tmpidx < ((bufferCAN[rPtr].raw[4]&0xF)+5); tmpidx++) {
        itoa(bufferCAN[rPtr].raw[tmpidx], buf, 16);
        for (uint8_t bufidx = 0; buf[bufidx] != '\0'; bufidx++) {
          rawtoa[rawidx] = buf[bufidx];
          rawidx++;
        }
        rawtoa[rawidx] = ' ';
        rawidx++;
        memset(buf, 0, sizeof(buf));
      }
      char logEntry[64];
      sprintf(logEntry, "CH %d RAW %s", bufferCAN[rPtr].channel, rawtoa);
      rPtr++;
      for (uint8_t logidx = 0; logEntry[logidx] != '\0'; logidx++) {
        bufferSD[SDpos] = logEntry[logidx];
        SDpos++;
      }
      bufferSD[SDpos] = '\n';
      SDpos++;     
    } else {
      delay(20);
    }

    if (SDpos > 512) {
      appendFile(SD_MMC, "/log.txt", bufferSD);
      memset(bufferSD, '\0', sizeof(bufferSD));
      SDpos = 0;
    }

    if (endLogging) {
      appendFile(SD_MMC, "/log.txt", bufferSD);
      memset(bufferSD, '\0', sizeof(bufferSD));
      SDpos = 0;
      endLogging = false;
    }
  }
}

// This task is pinned to Core 1. Its job is mostly to get interrupted.
// It also waits for Serial and reports errors. 
void canMonitor(void *parameter) {
  for (;;) {
    while (!Serial.available()) {
      if (errorRegister && VERBOSE_ERR) {
        printErrors();
      }
      // DEBUG: Just to keep an eye on ring buffers during testing
      if(SDpos != SDpos_prev || rPtr != rPtr_prev || wPtr != wPtr_prev) {
        Serial.printf("rPtr: %d  wPtr: %d  SDpos: %d \n", rPtr, wPtr, SDpos);
        SDpos_prev = SDpos;
        rPtr_prev = rPtr;
        wPtr_prev = wPtr;
      }
    }
    debugMenu();
  }
  return;
}

// Convenience function to write the ANSI Terminal Clear Escape Sequence
// Arduino IDE Serial Terminal doesn't respect ANSI Escape codes but 
// Most terminal emulators do
void ANSI_clear() { Serial.write("\033[2J\033[H", 7); }

// Convenience function to empty the Serial receive buffer and wait for 
// user input during menu navigation
void wait() {
  while (Serial.available()) {
    Serial.read();
  }  // Empty Serial Buffer
  while (!Serial.available()) {
    delay(200);
  }
  return;
}

// Debug menu. Pauses logging and allows manipulation of MCP2515 and Storage
void debugMenu() {
  endLogging = true; // Flag to Core 0 process to dump bufferSD
  exitMenu = false; // Lazy flag to break from menu loop
  for (;;) {
    Serial.println("DEBUG MENU");
    Serial.println("---------------------");
    Serial.println("1) Dump Log");
    Serial.println("2) Delete Log");
    Serial.println("3) Test SD Card");
    Serial.println("4) Resume Logging");
    Serial.println("5) Check Overflow Err");
    Serial.println("6) Reset Overflow Err");
    wait();
    ANSI_clear();
    switch (Serial.read()) {
      case '1':
        readFile(SD_MMC, "/log.txt");
        Serial.println("Press Any Key To Exit");
        wait();
        ANSI_clear();
        break;
      case '2':
        deleteFile(SD_MMC, "/log.txt");
        writeFile(SD_MMC, "/log.txt", "START\n");
        break;
      case '3':
        Serial.println("Attempting to Write to SD");
        writeFile(SD_MMC, "/test.txt", "TEST SUCCESSFUL!");
        Serial.println("Attempting to Read from SD");
        readFile(SD_MMC, "/test.txt");
        deleteFile(SD_MMC, "/test.txt");
        break;
      case '4':
        exitMenu = true;
        break;
      case '5':
        Serial.println(
            "RX1OVR | RX0OVR | TXBO | TXEP | RXEP | TXWAR | RXWAR | EWARN");
        for (uint8_t ch = 0; ch < 6; ch++) {
          Serial.print("Channel ");
          Serial.print(ch);
          Serial.print(": ");
          Serial.println(readRegister(ch, 0x2D), BIN);
        }
        Serial.println("Press Any Key To Exit");
        wait();
        ANSI_clear();
        break;
      case '6':
        Serial.println("Resetting Error Flag Registers");
        for (uint8_t ch = 0; ch < 6; ch++) {
          writeRegister(ch, 0x2D, 0x00);
        }
        break;
      default:
        Serial.println("Invalid Command (Try '4' to leave menu?)");
        break;
    }
    while (Serial.available()) {
      Serial.read();
    }  // Empty Serial Buffer
    if (exitMenu) {
      break;
    }
  }
  return;
}

// Vestigial loop function
void loop() {}