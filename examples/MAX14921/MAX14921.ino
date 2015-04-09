#include <max14921.h>
#include <HardwareSerial.h>
#include <SPI.h>

// for Uno
//#define SELPIN 10    // chip-select
//#define DATAOUT 11   // MOSI 
//#define DATAIN 12    // MISO 
//#define SPICLOCK 13  // Clock 

// for Nano
#define SELPIN 10    // chip-select MAX11163
#define DATAOUT 11   // MOSI 
#define DATAIN 12    // MISO 
#define SPICLOCK 13  // Clock 

#define SELMAXPIN 10 // chip-select MAX14921
#define SAMPLPIN 10 // SAMPLE pin MAX14921  

max14921 _max14921(SELPIN, SELMAXPIN, SAMPLPIN);

void setup(){ 
  Serial.begin(115200); 
}

void loop() {
  _max14921.MD_AK35_ScanCell(false);
}

