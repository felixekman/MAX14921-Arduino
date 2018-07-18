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

#define CELLS 16 //Number of cells active
#define CHEMISTRY LiPo //LiPo, LIFe : LiPo: 4.2V full 3.0V minimum, LiFe: 3.6V full 2.8V minimum
#define BALANCING true // Balancing all cells
#define OPENWIRE false // Dectection of Open-Wire
#define SAMPLETIME 40




max14921 _max14921(SELPIN, SELMAXPIN, SAMPLPIN);

void setup(){ 
  Serial.begin(115200); 
}


Serial.print(packBalancing.status); // On, Battery pack balancing is active
Serial.print(packBalancing.status); // Off, Battery pack balancing is NOT active

Serial.print(OpenWire.status); // Yes, Warning if open-wire condtion is met
Serial.print(OpenWire.status); // No, Status if open wire is not met. I.e. no problem

Serial.print(packVoltage.procentage); // The procentage of the power left in the pack. Ex. 8S LiPo battery Vmax. 33,6v Vmin 28,8v

Serial.print(packVoltage.total); //The current Voltage value of the entire battery pack i.e. 33.251v cell 1-16

Serial.print(CellVoltage.1); // The current Voltage value of Cell 1 i.e. 3.151v
Serial.print(CellVoltage.2); // The current Voltage value of Cell 2 i.e. 3.151v
Serial.print(CellVoltage.3); // The current Voltage value of Cell 3 i.e. 3.151v
Serial.print(CellVoltage.4); // The current Voltage value of Cell 4 i.e. 3.151v
Serial.print(CellVoltage.5); // The current Voltage value of Cell 5 i.e. 3.151v
Serial.print(CellVoltage.6); // The current Voltage value of Cell 6 i.e. 3.151v
Serial.print(CellVoltage.7); // The current Voltage value of Cell 7 i.e. 3.151v
Serial.print(CellVoltage.8); // The current Voltage value of Cell 8 i.e. 3.151v
Serial.print(CellVoltage.9); // The current Voltage value of Cell 9 i.e. 3.151v
Serial.print(CellVoltage.10); // The current Voltage value of Cell 10 i.e. 3.151v
Serial.print(CellVoltage.11); // The current Voltage value of Cell 11 i.e. 3.151v
Serial.print(CellVoltage.12); // The current Voltage value of Cell 12 i.e. 3.151v
Serial.print(CellVoltage.13); // The current Voltage value of Cell 13 i.e. 3.151v
Serial.print(CellVoltage.14); // The current Voltage value of Cell 14 i.e. 3.151v
Serial.print(CellVoltage.15); // The current Voltage value of Cell 15 i.e. 3.151v
Serial.print(CellVoltage.16); // The current Voltage value of Cell 16 i.e. 3.151v


void loop() {
  _max14921.MD_AK35_ScanCell(false);
}

