/*
 * hall_sensor.h
 *
 *  Created on: Jan 8, 2024
 *      Author: Wangy
 */

#ifndef INC_HALL_SENSOR_HPP_
#define INC_HALL_SENSOR_HPP_


#include "stm32wbxx_hal.h"
#include <stdlib.h>
#include <string.h>

/* Definitions */
#define ADS1115_OS (0b1 << 7) // Default

#define ADS1115_MUX_AIN0 (0b100 << 4)		// Analog input 1
#define ADS1115_MUX_AIN1 (0b101 << 4)		// Analog input 2
#define ADS1115_MUX_AIN2 (0b110 << 4)		// Analog input 3
#define ADS1115_MUX_AIN3 (0b111 << 4)		// Analog input 4

#define ADS1115_PGA_TWOTHIRDS 	(0b000 << 1) 		// 2/3x Gain	-- 0.1875 mV by one bit		MAX: +- VDD + 0.3V
#define ADS1115_PGA_ONE			(0b001 << 1) 		// 1x Gain		-- 0.125 mV by one bit		MAX: +- VDD + 0.3V
#define ADS1115_PGA_TWO			(0b010 << 1) 		// 2x Gain		-- 0.0625 mV by one bit		MAX: +- 2.048 V
#define ADS1115_PGA_FOUR		(0b011 << 1) 		// 4x Gain		-- 0.03125 mV by one bit	MAX: +- 1.024 V
#define ADS1115_PGA_EIGHT		(0b100 << 1) 		// 8x Gain		-- 0.015625 mV by one bit	MAX: +- 0.512 V
#define ADS1115_PGA_SIXTEEN		(0b111 << 1) 		// 16x Gain		-- 0.0078125 mV by one bit	MAX: +- 0.256 V

// changed to continuous conversion mode
#define ADS1115_MODE (0b0) // Default

#define ADS1115_DATA_RATE_8		(0b000 << 5)			// 8SPS
#define ADS1115_DATA_RATE_16	(0b001 << 5)			// 16SPS
#define ADS1115_DATA_RATE_32	(0b010 << 5)			// 32SPS
#define ADS1115_DATA_RATE_64	(0b011 << 5)			// 64SPS
#define ADS1115_DATA_RATE_128	(0b100 << 5)			// 128SPS
#define ADS1115_DATA_RATE_250	(0b101 << 5)			// 250SPS
#define ADS1115_DATA_RATE_475	(0b110 << 5)			// 475SPS
#define ADS1115_DATA_RATE_860	(0b111 << 5)			// 860SPS

#define ADS1115_COMP_MODE 	(0b0 << 4) // Default
#define ADS1115_COMP_POL 	(0b0 << 3) // Default
#define ADS1115_COMP_LAT 	(0b0 << 2) // Default
#define ADS1115_COMP_QUE 	(0b11)	   // Default

/* ADS1115 register configurations */
#define ADS1115_CONVER_REG 0x0
#define ADS1115_CONFIG_REG 0x1

/* TIMEOUT */
#define ADS1115_TIMEOUT 1 // Timeout for HAL I2C functions.

/**
 * different ads1115 device address on i2c bus line
 * 		i). standard address(ADDR pin connected to GND): 0b1001000
 * 	   ii). first modified(ADDR pin connected to VDD): 0b1001001
 * 	  iii). second modified(ADDR pin connected to SDA): 0b1001010
 * 	  iv). third modified(ADDR pin connected to SCL): 0b1001011
 */
#define ADS1115_ADDR_STD 0b1001000 << 1
#define ADS1115_ADDR_VDD 0b1001001 << 1
#define ADS1115_ADDR_SDA 0b1001010 << 1
#define ADS1115_ADDR_SCL 0b1001011 << 1




/**
 * size of the real-time hall sensory reading sequence buffer
 */
#define SENSOR_RAW_BUFFER_SIZE 50
#define SENSOR_PROCESSED_BUFFER_SIZE 200

/**
 * hall sensor object class definition
 */
class Sensor {
private:




	/**
	 * there could be multiple ads1115 in use, and connected addr could be different
	 */
	int ads1115Addr;
	/**
	 * channel connected to different hall sensors is different
	 */
	int channelNO;
	/**
	 * field for describing the required data rate for sampling
	 */
	int dataRate;

	/**
	 * field for describing programmable amplifier rate
	 */
	int PGARate;

	/**
	  * i2c handle type definition for executing i2c related works
	  */

	I2C_HandleTypeDef i2cHandler;

	/**
	 * voltage coefficient for later conversion computation
	 */
	float ADC_conversionCoeff;

	/**
	 * buffer for retaining raw, unfiltered data
	 */
    float rawDataBuffer[SENSOR_RAW_BUFFER_SIZE];
    /**
     * buffer for containing filtered data, extracted from raw buffer and purified
     */
    float processedDataBuffer[SENSOR_PROCESSED_BUFFER_SIZE];

    /**
     * integer value indicating the current count of data entries
     */
    int rawDataCount;

    /**
     * integer value for retaining the head of the CYCLIC raw buffer
     */
    int currentRawHead;
    /**
     * integer value for retaining the head of the CYCLIC processed buffer
     */
    int currentProcessedHead;
    /**
     * summed value of raw buffer
     */
    float sumRawData;
    /**
     * summed value of processed buffer
     */
    float sumProcessedData;

public:
    /**
     * constructor for Sensor object
     */
    Sensor(I2C_HandleTypeDef* handler, int ads1115_addr, int ads1115_channel_number, int data_rate, int PGA_rate);

    float ReportReading();


    /**
     * Set the ADC configuration according to the current sensor requirement->
     * Sample the current sensor 5 times and update raw value buffer->
     * Update the processedData buffer accordingly
     */
    void Update();



private:

    /**
      * Change the current listening channel to be the corrsponding channel of this sensor
      */
     HAL_StatusTypeDef SetChannel(void);

     /**
      * Get samples from the current sensor object
      */
     void SampleStore(void);
     /**
      * Average the current raw values in raw buffer, add into processed buffer
      */
     void ProcessData();

     /**
      * Set the corresponding adc conversion coeff
      */
     void SetAdcConvCoeff(int PGA_Rate);



};


#endif /* INC_HALL_SENSOR_HPP_ */
