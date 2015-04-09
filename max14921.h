/*
 MAX14921.h - Library for reading from a MAX14921.
 Created by Felix
 */

// ensure this library description is only included once
#ifndef MAX14921_h
#define MAX14921_h

#include "Arduino.h"
#include <inttypes.h>

// library interface description
class max14921 {
public:
	max14921(int ADC_CS_pin, int CS_pin, int SAMPLPIN_pin);
	void SetCellNumber(uint8_t cellNum);
	void SetBalancing(bool bal);
	void SetOpenWireDetection(bool openwire);
	void SetSampleTime(int sampletime);
	void SetSettlingTime(int settlingtime);
	void SetRepeatTime(int repeattime);

	int MAX11163_ReadData16(void);

	void MD_AK35_ScanCell(bool calibrationData);

private:
	long MD_AK35_SpiTransfer24(uint8_t byte1, uint8_t byte2, uint8_t byte3);
	void MD_AK35_Cmd_AcquireCell(uint8_t cellNum,
			int *pAdcCellVoltage,
			long *pSpiRxData );

	int _ADC_CS_pin;
	int _CS_pin;
	int _SAMPLPIN_pin;
};

#endif

