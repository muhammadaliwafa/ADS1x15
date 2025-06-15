/*
 * ADS1X15.c
 *
 *  Created on: Jun 11, 2025
 *      Author: me
 */

#include "ADS1X15.h"

const uint16_t MUX_BY_CHANNEL[4] = {
    ADS1X15_REG_CONFIG_MUX_SINGLE_0,
    ADS1X15_REG_CONFIG_MUX_SINGLE_1,
    ADS1X15_REG_CONFIG_MUX_SINGLE_2,
    ADS1X15_REG_CONFIG_MUX_SINGLE_3
};

static void ADSbegin(m_i2c_dev *i2c) {
	if (HAL_I2C_IsDeviceReady(i2c->hi2c, i2c->m_addr, 10, 10) != HAL_OK)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // This MUST have GPIO PA5 ready to use - ERROR I2C - Wrong address

}

/**************************************************************************/
/*!
    @brief  Sets the gain and input voltage range

    @param gain gain setting to use
*/
/**************************************************************************/
void setGain(m_i2c_dev *i2c, adsGain_t gain) { i2c->m_gain = gain; }

/**************************************************************************/
/*!
    @brief  Gets a gain and input voltage range

    @return the gain setting
*/
/**************************************************************************/
adsGain_t getGain(m_i2c_dev *i2c) { return i2c->m_gain; }

/**************************************************************************/
/*!
    @brief  Sets the data rate

    @param rate the data rate to use
*/
/**************************************************************************/
void setDataRate(m_i2c_dev *i2c, uint16_t rate) { i2c->m_dataRate = rate; }

/**************************************************************************/
/*!
    @brief  Gets the current data rate

    @return the data rate
*/
/**************************************************************************/
uint16_t getDataRate(m_i2c_dev *i2c) { return i2c->m_dataRate; }

/**************************************************************************/
/*!
    @brief  Gets a single-ended ADC reading from the specified channel

    @param channel ADC channel to read

    @return the ADC reading
*/
/**************************************************************************/

float computeVolts(m_i2c_dev *i2c, int16_t counts) {
  // see data sheet Table 3
  float fsRange;
  switch (i2c->m_gain) {
  case GAIN_TWOTHIRDS:
    fsRange = 6.144f;
    break;
  case GAIN_ONE:
    fsRange = 4.096f;
    break;
  case GAIN_TWO:
    fsRange = 2.048f;
    break;
  case GAIN_FOUR:
    fsRange = 1.024f;
    break;
  case GAIN_EIGHT:
    fsRange = 0.512f;
    break;
  case GAIN_SIXTEEN:
    fsRange = 0.256f;
    break;
  default:
    fsRange = 0.0f;
  }
  return counts * (fsRange / (32768 >> (i2c->m_bitShift)));
}



static void writeRegister(m_i2c_dev *i2c, uint8_t reg, uint16_t value) {
	uint8_t buffer[] = {
			reg,
			(uint8_t) (value >> 8),
			(uint8_t) (value & 0xFF)
	};
  HAL_I2C_Master_Transmit(i2c->hi2c, i2c->m_addr, buffer, 3, 10);
}
/**************************************************************************/
/*!
    @brief  Read 16-bits from the specified destination register

    @param reg register address to read from

    @return 16 bit register value read
*/
/**************************************************************************/


static uint16_t readRegister(m_i2c_dev *i2c, uint8_t reg) {
	HAL_I2C_Master_Transmit(i2c->hi2c, i2c->m_addr, &reg, 1, 10);
	uint8_t buffer[2];
	HAL_I2C_Master_Receive(i2c->hi2c, i2c->m_addr, buffer, 2, 10);
	return ((buffer[0] << 8) | buffer[1]);
}

uint8_t conversionComplete(m_i2c_dev *i2c) {
  return (readRegister(i2c, ADS1X15_REG_POINTER_CONFIG) & 0x8000) != 0;
}



void ADS1115_init(m_i2c_dev *i2c, I2C_HandleTypeDef *hi2c, uint8_t i2cAddress) {
	i2c->hi2c = hi2c;
	i2c->m_addr = i2cAddress << 1; //  It's Important to shift the address << 1
	i2c->m_bitShift = 0;
	i2c->m_dataRate = RATE_ADS1115_128SPS;
	i2c->m_gain = GAIN_TWOTHIRDS; /* +/- 6.144V range (limited to VDD +0.3V max!) */
	ADSbegin(i2c);
}


void startADCReading(m_i2c_dev *i2c, uint16_t mux, uint8_t continuous) {
  // Start with default values
  uint16_t config =
      ADS1X15_REG_CONFIG_CQUE_1CONV |   // Set CQUE to any value other than
                                        // None so we can use it in RDY mode
      ADS1X15_REG_CONFIG_CLAT_NONLAT |  // Non-latching (default val)
      ADS1X15_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1X15_REG_CONFIG_CMODE_TRAD;    // Traditional comparator (default val)

  if (continuous) {
    config |= ADS1X15_REG_CONFIG_MODE_CONTIN;
  } else {
    config |= ADS1X15_REG_CONFIG_MODE_SINGLE;
  }

  // Set PGA/voltage range
  config |= i2c->m_gain;

  // Set data rate
  config |= i2c->m_dataRate;

  // Set channels
  config |= mux;

  // Set 'start single-conversion' bit
  config |= ADS1X15_REG_CONFIG_OS_SINGLE;
  // Write config register to the ADC
  writeRegister(i2c, ADS1X15_REG_POINTER_CONFIG, config);

  // Set ALERT/RDY to RDY mode.
//  writeRegister(ADS1X15_REG_POINTER_HITHRESH, 0x8000);
//  writeRegister(ADS1X15_REG_POINTER_LOWTHRESH, 0x0000);
}



int16_t getLastConversionResults(m_i2c_dev *i2c) {
	// Read the conversion

	uint16_t res = readRegister(i2c, ADS1X15_REG_POINTER_CONVERT)
			>> (i2c->m_bitShift);
	if (i2c->m_bitShift == 0) {
		return (int16_t) res;
	} else {
		// Shift 12-bit results right 4 bits for the ADS1015,
		// making sure we keep the sign bit intact
		if (res > 0x07FF) {
			// negative number - extend the sign to 16th bit
			res |= 0xF000;
		}
		return (int16_t) res;
	}
}


int16_t readADC_SingleEnded(m_i2c_dev *i2c, uint8_t channel) {
	if (channel > 3) {
		return 0;
	}

	startADCReading(i2c, MUX_BY_CHANNEL[channel], 0);
	while (!conversionComplete(i2c))
		;

	// Read the conversion results
	// Shift 12-bit results right 4 bits for the ADS1015
	return getLastConversionResults(i2c);
}

void startComparator_SingleEnded(m_i2c_dev *i2c, uint8_t channel, int16_t threshold) {
  // Start with default values
  uint16_t config =
      ADS1X15_REG_CONFIG_CQUE_1CONV |   // Comparator enabled and asserts on 1
                                        // match
      ADS1X15_REG_CONFIG_CLAT_LATCH |   // Latching mode
      ADS1X15_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1X15_REG_CONFIG_CMODE_TRAD |   // Traditional comparator (default val)
      ADS1X15_REG_CONFIG_MODE_CONTIN |  // Continuous conversion mode
      ADS1X15_REG_CONFIG_MODE_CONTIN;   // Continuous conversion mode

  // Set PGA/voltage range
  config |= i2c->m_gain;

  // Set data rate
  config |= i2c->m_dataRate;

  config |= MUX_BY_CHANNEL[channel];

  // Set the high threshold register
  // Shift 12-bit results left 4 bits for the ADS1015
  writeRegister(i2c, ADS1X15_REG_POINTER_HITHRESH, threshold << (i2c->m_bitShift));

  // Write config register to the ADC
  writeRegister(i2c, ADS1X15_REG_POINTER_CONFIG, config);
}


/**************************************************************************/
/*!
    @brief  Reads the conversion results, measuring the voltage
            difference between the P (AIN0) and N (AIN1) input.  Generates
            a signed value since the difference can be either
            positive or negative.

    @return the ADC reading
*/
/**************************************************************************/
int16_t readADC_Differential_0_1(m_i2c_dev *i2c) {
  startADCReading(i2c, ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/0);

  // Wait for the conversion to complete
  while (!conversionComplete(i2c))
    ;

  // Read the conversion results
  return getLastConversionResults(i2c);
}

/**************************************************************************/
/*!
    @brief  Reads the conversion results, measuring the voltage
            difference between the P (AIN0) and N (AIN3) input.  Generates
            a signed value since the difference can be either
            positive or negative.
    @return the ADC reading
*/
/**************************************************************************/
int16_t readADC_Differential_0_3(m_i2c_dev *i2c) {
  startADCReading(i2c, ADS1X15_REG_CONFIG_MUX_DIFF_0_3, /*continuous=*/0);

  // Wait for the conversion to complete
  while (!conversionComplete(i2c))
    ;

  // Read the conversion results
  return getLastConversionResults(i2c);
}


/**************************************************************************/
/*!
    @brief  Reads the conversion results, measuring the voltage
            difference between the P (AIN1) and N (AIN3) input.  Generates
            a signed value since the difference can be either
            positive or negative.
    @return the ADC reading
*/
/**************************************************************************/
int16_t readADC_Differential_1_3(m_i2c_dev *i2c) {
  startADCReading(i2c, ADS1X15_REG_CONFIG_MUX_DIFF_1_3, /*continuous=*/0);

  // Wait for the conversion to complete
  while (!conversionComplete(i2c))
    ;

  // Read the conversion results
  return getLastConversionResults(i2c);
}
/**************************************************************************/
/*!
    @brief  Reads the conversion results, measuring the voltage
            difference between the P (AIN2) and N (AIN3) input.  Generates
            a signed value since the difference can be either
            positive or negative.

    @return the ADC reading
*/
/**************************************************************************/
int16_t readADC_Differential_2_3(m_i2c_dev *i2c) {
  startADCReading(i2c, ADS1X15_REG_CONFIG_MUX_DIFF_2_3, /*continuous=*/0);

  // Wait for the conversion to complete
  while (!conversionComplete(i2c))
    ;

  // Read the conversion results
  return getLastConversionResults(i2c);
}
