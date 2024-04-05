/*
 * hall_sensor.cpp
 *
 *  Created on: Jan 5, 2024
 *      Author: Wangy
 */

#include <hall_sensor.hpp>

Sensor::Sensor(I2C_HandleTypeDef *handler, int ads1115_addr,
				int ads1115_channel_number, int data_rate, int PGA_rate) :
				currentRawHead(0), currentProcessedHead(0), rawDataCount(0), sumRawData(
								0.0f), sumProcessedData(0.0f) {
	// Initialize arrays with zeros
	for (int i = 0; i < SENSOR_RAW_BUFFER_SIZE; ++i) {
		this->rawDataBuffer[i] = 0.0f;
	}
	for (int i = 0; i < SENSOR_PROCESSED_BUFFER_SIZE; ++i) {
		this->processedDataBuffer[i] = 0.0f;
	}
	memcpy(&this->i2cHandler, handler, sizeof(*handler));
	this->ads1115Addr = ads1115_addr;
	this->channelNO = ads1115_channel_number;
	this->PGARate = PGA_rate;
	this->dataRate = data_rate;
	this->SetAdcConvCoeff(this->PGARate);
	this->SetChannel();

}

void Sensor::SetAdcConvCoeff(int PGA_Rate) {
	// Voltage coefficient update.
	switch (PGA_Rate) {

		case ADS1115_PGA_TWOTHIRDS:
			ADC_conversionCoeff = 0.1875;
			break;

		case ADS1115_PGA_ONE:
			ADC_conversionCoeff = 0.125;
			break;

		case ADS1115_PGA_TWO:
			ADC_conversionCoeff = 0.0625;
			break;

		case ADS1115_PGA_FOUR:
			ADC_conversionCoeff = 0.03125;
			break;

		case ADS1115_PGA_EIGHT:
			ADC_conversionCoeff = 0.015625;
			break;

		case ADS1115_PGA_SIXTEEN:
			ADC_conversionCoeff = 0.0078125;
			break;

	}
}

HAL_StatusTypeDef Sensor::SetChannel(void) {

	uint8_t ADS1115_config[2];

	ADS1115_config[0] = ADS1115_OS | this->channelNO | this->PGARate
					| ADS1115_MODE;
	ADS1115_config[1] = this->dataRate | ADS1115_COMP_MODE | ADS1115_COMP_POL
					| ADS1115_COMP_LAT | ADS1115_COMP_QUE;

	return HAL_I2C_Mem_Write(&this->i2cHandler, (uint16_t) (this->ads1115Addr),
					ADS1115_CONFIG_REG, 1, ADS1115_config, 2, ADS1115_TIMEOUT);
}

void Sensor::SampleStore(void) {

	uint8_t ADS1115_rawValue[2];
	float curReading;

	// read from the ads1115 for the converted magnetic field correlated voltage value
	if (HAL_I2C_Mem_Read(&this->i2cHandler,
					(uint16_t) ((this->ads1115Addr) | 0x1),
					ADS1115_CONVER_REG, 1, ADS1115_rawValue, 2, ADS1115_TIMEOUT)
					== HAL_OK) {

		curReading = (float) (((int16_t) (ADS1115_rawValue[0] << 8)
						| ADS1115_rawValue[1]) * this->ADC_conversionCoeff);

	} else {
		exit(EXIT_FAILURE);
	}

	// Update the sum by subtracting the value being overwritten and adding the new value
	this->sumRawData -= this->rawDataBuffer[this->currentRawHead];
	this->sumRawData += curReading;

	// Update the buffer and head index
	this->rawDataBuffer[this->currentRawHead] = curReading;
	this->currentRawHead = (this->currentRawHead + 1) % SENSOR_RAW_BUFFER_SIZE;

	// Update current sensor's raw buffer count (less than 50, beginning)
	if (this->rawDataCount < SENSOR_RAW_BUFFER_SIZE) {
		this->rawDataCount++;
	}
}

void Sensor::ProcessData() {

	// Compute the avg of the current updated raw buffer
	float averagedReading = this->sumRawData / this->rawDataCount;

	// Update the processed data buffer: sum value
	sumProcessedData -= processedDataBuffer[currentProcessedHead];
	sumProcessedData += averagedReading;

	// Update the processed data buffer: entries and head
	processedDataBuffer[currentProcessedHead] = averagedReading;
	currentProcessedHead = (currentProcessedHead + 1)
					% SENSOR_PROCESSED_BUFFER_SIZE;
}

void Sensor::Update() {

	// Set the ADC configuration according to current sensor requirement
//	this->SetChannel();

// Sample and store the data into the raw data buffer 5 times
	this->SampleStore();
	this->SampleStore();
	this->SampleStore();
	this->SampleStore();
	this->SampleStore();

	// Compute avg from the raw and update into processedDataBuffer
	this->ProcessData();
}

float Sensor::ReportReading() {

	int prevHead = (this->currentProcessedHead + SENSOR_PROCESSED_BUFFER_SIZE
					- 1) % SENSOR_PROCESSED_BUFFER_SIZE;
	return this->processedDataBuffer[prevHead]; // Returning the most recent processed reading
}

