/*************************************************************************//**
* \file sensor_vl53l0x.c
*
* \brief contains all the necessary functions required to support the VL53L0X sensor
*
* \details contains all the necessary functions required to support the VL53L0X sensor
*
* \author Original by: Jasmine Priatna

* \date 6/1/2021
*
* \copyright Copyright (c) 2020 by METco. All rights reserved.
*****************************************************************************/
/*****************************************************************************
 * FILE INCLUDES
 *****************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include "tool_configuration.h"
#include "global_defines.h"
#include "global_variables.h"
#include "hardware_abstraction.h"
#include "user_configuration_functions.h"
#include "fault_functions.h"
#include "motor_control.h"
#include "hal_i2c.h"
#include "imu_lsm6dsl.h"
#include "sensor_vl53l0x.h"
#include "hal_busywait.h"

/*****************************************************************************
 * GLOBAL VARIABLES
 *****************************************************************************/
// Stored information of first sensor
VL53L0X_Dev_t MyDevice_0;
VL53L0X_Dev_t *pMyDevice_0 = &MyDevice_0;  
// Stored information of second sensor
VL53L0X_Dev_t MyDevice_1;
VL53L0X_Dev_t *pMyDevice_1 = &MyDevice_1;  

// Used for depth calculation
uint16_t start_distance; // start distance used in depth calculation
float_t target_inch = 1; // target depth in inches
float_t target_mm; // target depth in mm

// Used for distance adjustments
uint16_t distance; // adjusted distance for depth calculation
uint16_t actual_dist_1; // measured distance from sensor 1
uint16_t actual_dist_2; // measured distance from sensor 2
uint16_t rangestatus_0; // status of distance from sensor 1
uint16_t rangestatus_1; // status of distance from sensor 2
static int16_t last_adj[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // last 10 adjusted distances used in distance adjustment
static uint16_t last_changed_1[5] = {0, 0, 0, 0, 0}; // last 5 measured distances of sensor 1
static uint16_t last_changed_2[5] = {0, 0, 0, 0, 0}; // last 5 measured distances of sensor 1
int16_t diff_dist_1[5] = {0, 0, 0, 0, 0}; // difference of last 5 measured distances of sensor 1
int16_t diff_dist_2[5] = {0, 0, 0, 0, 0}; // difference of last 5 measured distances of sensor 2

/*****************************************************************************
 * FUNCTION PROTOTYPES
 *****************************************************************************/
// Read/Write from sensor registers, modified from VL53L0X API
uint8_t readReg(uint8_t sensor, uint8_t reg);
uint16_t readReg16Bit(uint8_t sensor, uint8_t reg);
void writeReg16Bit(uint8_t sensor, uint8_t reg, uint16_t value);

// Used to calculate and set timing budget in between distance measurements, taken from VL53L0X API
uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);
uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
void getSequenceStepEnables(uint8_t sensor, SequenceStepEnables * enables);
uint8_t getVcselPulsePeriod(uint8_t sensor, vcselPeriodType type);
uint16_t decodeTimeout(uint16_t reg_val);
void getSequenceStepTimeouts(uint8_t sensor, SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts);
uint16_t encodeTimeout(uint32_t timeout_mclks);
bool setMeasurementTimingBudget(uint8_t sensor, uint32_t budget_us);

// Used to set signal rate
bool setSignalRateLimit(uint8_t sensor, float limit_Mcps);

// Initialization
void init_vl53l0x_gpio(void);
void init_vl53l0x(void);

// Used to get distance measurements and status, modified from VL53L0X API
VL53L0X_Error VL53L0X_GetLimitCheckEnable(VL53L0X_DEV Dev, uint16_t LimitCheckId, uint8_t *pLimitCheckEnable);
void VL53L0X_get_pal_range_status(VL53L0X_DEV Dev, uint8_t DeviceRangeStatus, FixPoint1616_t SignalRate, uint16_t EffectiveSpadRtnCount, VL53L0X_RangingMeasurementData_t *pRangingMeasurementData, uint8_t *pPalRangeStatus);
void get_ranging_measurement_data(VL53L0X_DEV Dev, VL53L0X_RangingMeasurementData_t *pRangingMeasurementData, uint8_t write_addr, uint8_t read_addr);

// Distance and depth calculations
void get_distance_vl53l0x(void);
int16_t get_hole_depth();

/*****************************************************************************
 * VARIABLE DECLARATIONS
 *****************************************************************************/
/** VL53L0X parameters local to the module */
uint32_t measurement_timing_budget_us; // timing budget in between distance measurements
static uint16_t last_temp; // last adjusted distance
static uint16_t last_actual_1; // last measured distance from sensor 1
static uint16_t last_actual_2; // last measured distance from sensor 2



// function definition example
/*************************************************************************//**
* \fn bool set_adaptive_output_params()
*
* \brief Enables adaptive output control feature.
*
* \details Enables adaptive output control by bounds checking and setting
* the enable flag high.
*
* @param[in]  target_adpt_pwm       Desired PWM delivered to the motor
* @param[in]  target_ccw_pwm        Desired PWM delivered to the motor when counter-clockwise
* @param[out] max_pwm               The maximum motor percent power acheivable when running in motor default mode
* @param[out] min_pwm               The minimum motor percent power acheivable when running in motor default mode
* @param[out] bbounded_pwm_enabled  TRUE for when bounded pwm (open loop) feature is enabled, FALSE otherwise
*
* \calls set_motor_adapt_flags_adaptive_output_control, set_motor_adapt_params_adaptive_output_control
*
* \return TRUE if all passed parameters are within the system's allowable bounds, FALSE if not
*****************************************************************************/


// ----------- Functions used to Read/Write Registers ----------------------------

/*************************************************************************//**
* \fn readReg()
*
* \brief Reads an 8-bit register
*
* @param[in]  sensor                    Sensor device being read from (1 or 2)
* @param[in]  reg                       Register that is being read
*
* \return Value from read register 
*****************************************************************************/
uint8_t readReg(uint8_t sensor, uint8_t reg)
{
  uint8_t ret_val;
  uint8_t data_temp;
  data_temp = reg;
  if (sensor == 1)
  {
    i2c_write(0, I2C_ADDR_VL53L0X_WRITE_1, &data_temp, sizeof(data_temp));
    i2c_read(0, I2C_ADDR_VL53L0X_READ_1, &ret_val, sizeof(ret_val));
  }
  else 
  {
    i2c_write(0, I2C_ADDR_VL53L0X_WRITE_2, &data_temp, sizeof(data_temp));
    i2c_read(0, I2C_ADDR_VL53L0X_READ_2, &ret_val, sizeof(ret_val));
  }
  return ret_val;
}

/*************************************************************************//**
* \fn readReg16Bit()
*
* \brief Reads a 16-bit register
*
* @param[in]  sensor                    Sensor device being read from (1 or 2)
* @param[in]  reg                       Register that is being read
*
* \return Value from read register 
*****************************************************************************/
uint16_t readReg16Bit(uint8_t sensor, uint8_t reg)
{
  uint16_t ret_val;
  uint8_t ret_data[2];
  uint8_t data_temp;
  data_temp = reg;
  if (sensor == 1)
  {
    i2c_write(0, I2C_ADDR_VL53L0X_WRITE_1, &data_temp, sizeof(data_temp));
    i2c_read(0, I2C_ADDR_VL53L0X_READ_1, (uint8_t*)&ret_data[0], sizeof(ret_data));
  }
  else
  {
    i2c_write(0, I2C_ADDR_VL53L0X_WRITE_2, &data_temp, sizeof(data_temp));
    i2c_read(0, I2C_ADDR_VL53L0X_READ_2, (uint8_t*)&ret_data[0], sizeof(ret_data));
  }
  ret_val = (ret_data[1]<<8)|ret_data[0];
  return ret_val;
}

/*************************************************************************//**
* \fn writeReg16Bit()
*
* \brief Writes to a 16-bit register
*
* @param[in]  sensor                    Sensor device being written to(1 or 2)
* @param[in]  reg                       Register that is being written to
* @param[in]  value                     Value that is written
*
* \return None
*****************************************************************************/
void writeReg16Bit(uint8_t sensor, uint8_t reg, uint16_t value)
{
  uint8_t data[3];
  data[0] = reg;
  data[1] = ((value >> 8) & 0xFF);
  data[2] = (value & 0xFF);
  if (sensor == 1)
  {
    i2c_write(0, I2C_ADDR_VL53L0X_WRITE_1, &data[0], sizeof(data));
  }
  else
  {
    i2c_write(0, I2C_ADDR_VL53L0X_WRITE_2, &data[0], sizeof(data));
  }
}


// ----------- Functions used to set Timing Budget ----------------------------

/*************************************************************************//**
* \fn timeoutMicrosecondsToMclks()
*
* \brief Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
*              
* @param[in]  timeout_period_us                    Timeout in microseconds
* @param[in]  vcsel_period_pclks                   VCSEL period
*
* \return Sequence step timeout period in MCLKs
*****************************************************************************/
uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

/*************************************************************************//**
* \fn timeoutMclksToMicroseconds()
*
* \brief Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
*              
* @param[in]  timeout_period_mclks                    Timeout in MCLKs
* @param[in]  vcsel_period_pclks                      VCSEL period
*
* \return Sequence step timeout period in microseconds
*****************************************************************************/
uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
  return ((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}

/*************************************************************************//**
* \fn getSequenceStepEnables()
*
* \brief Gets the (on/off) state of all sequence steps
*
* \details Retrieves the state of all sequence step in the scheduler
*              
* @param[in]  sensor                        Sensor device being read from (1 or 2)
* @param[in]  SequenceStepEnables           Pointer to struct containing result
* @param[out] 
*
* \return None
*****************************************************************************/
void getSequenceStepEnables(uint8_t sensor, SequenceStepEnables * enables)
{
  uint8_t sequence_config = readReg(sensor, SYSTEM_SEQUENCE_CONFIG); // change
  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}

/*************************************************************************//**
* \fn getVcselPulsePeriod()
*
* \brief Gets the VCSEL pulse period in PCLKs for the given period type
*              
* @param[in]  sensor         Sensor device being read from (1 or 2)
* @param[in]  type           Either the VcselPeriodPreRange or the VcselPeriodFinalRange
*
* \return VCSEL pulse period
*****************************************************************************/
uint8_t getVcselPulsePeriod(uint8_t sensor, vcselPeriodType type)
{
  if (type == VcselPeriodPreRange)
  { 
    return decodeVcselPeriod(readReg(sensor, PRE_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else if (type == VcselPeriodFinalRange)
  {
    return decodeVcselPeriod(readReg(sensor, FINAL_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else { return 255; }
}

/*************************************************************************//**
* \fn decodeTimeout()
*
* \brief Decode sequence step timeout in MCLKs from register value
*
* \details Note: the original function returned a uint32_t, but the return value is
* always stored in a uint16_t.
*              
* @param[in]  reg_val               Register value
*
* \return Decoded 16-bit timeout register value
*****************************************************************************/
uint16_t decodeTimeout(uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) << (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

/*************************************************************************//**
* \fn getSequenceStepTimeouts()
*
* \brief Get sequence step timeouts
*
* \details Gets all timeouts instead of just the requested one, and also stores
* intermediate values
*              
* @param[in]  sensor               Sensor device being read from (1 or 2)
* @param[in]  enables              Pointer to struct containing enables
* @param[in]  timeouts             Pointer to struct containing result
*
* \return None
*****************************************************************************/
void getSequenceStepTimeouts(uint8_t sensor, SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts)
{
  timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(sensor, VcselPeriodPreRange);
  
  timeouts->msrc_dss_tcc_mclks = readReg(sensor, MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us = timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);

  timeouts->pre_range_mclks = decodeTimeout(readReg16Bit(sensor, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->pre_range_us = timeoutMclksToMicroseconds(timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(sensor, VcselPeriodFinalRange);

  timeouts->final_range_mclks = decodeTimeout(readReg16Bit(sensor, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

  if (enables->pre_range)
  {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }

  timeouts->final_range_us = timeoutMclksToMicroseconds(timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);
}

/*************************************************************************//**
* \fn encodeTimeout()
*
* \brief Encode sequence step timeout register value from timeout in MCLKs
*              
* @param[in]  timeout_mclks               Timeout in MCLKs
*
* \return Encoded 16-bit timeout value
*****************************************************************************/
uint16_t encodeTimeout(uint32_t timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

/*************************************************************************//**
* \fn setMeasurementTimingBudget()
*
* \brief Set the measurement timing budget in microseconds, which is the time allowed
* for one measurement
*
* \details Takes care of splitting the timing budget among the sub-steps in the ranging sequence.
* A longer timing budget allows for more accurate measurements. Increasing the budget by a
* factor of N decreases the range measurement standard deviation by a factor of
* sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
*              
* @param[in]  sensor               Sensor device being read from (1 or 2)
* @param[in]  budget_us            Desired timing budget in microseconds
*
* \return True if timing budget is allowed, false otherwise
*****************************************************************************/
bool setMeasurementTimingBudget(uint8_t sensor, uint32_t budget_us)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910;
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t const MinTimingBudget = 20000;

  if (budget_us < MinTimingBudget) { return false; }

  uint32_t used_budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(sensor, &enables);
  getSequenceStepTimeouts(sensor, &enables, &timeouts);

  if (enables.tcc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    used_budget_us += FinalRangeOverhead;

    // "Note that the final range timeout is determined by the timing
    // budget and the sum of all other timeouts within the sequence.
    // If there is no room for the final range timeout, then an error
    // will be set. Otherwise the remaining time will be applied to
    // the final range."

    if (used_budget_us > budget_us)
    {
      // "Requested timeout too big."
      return false;
    }

    uint32_t final_range_timeout_us = budget_us - used_budget_us;

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint32_t final_range_timeout_mclks = timeoutMicrosecondsToMclks(final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range)
    {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    writeReg16Bit(sensor, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(final_range_timeout_mclks)); // change

    // set_sequence_step_timeout() end

    measurement_timing_budget_us = budget_us; // store for internal reuse
  }
  return true;
}


// ----------- Function used to set Signal Rate Limit ----------------------------

/*************************************************************************//**
* \fn setSignalRateLimit()
*
* \brief Set the return signal rate limit check value in units of MCPS (mega counts
* per second)
*
* \details "This represents the amplitude of the signal reflected from the
* target and detected by the device"; setting this limit presumably determines
* the minimum measurement necessary for the sensor to report a valid reading.
* Setting a lower limit increases the potential range of the sensor but also
* seems to increase the likelihood of getting an inaccurate reading because of
* unwanted reflections from objects other than the intended target.
* Defaults to 0.25 MCPS
*         
* @param[in]  sensor               Sensor device being read from (1 or 2)     
* @param[in]  limit_Mcps           Desired signal rate limit in MCPS
*
* \return True if signal rate limit is allowed
*****************************************************************************/
bool setSignalRateLimit(uint8_t sensor, float limit_Mcps)
{
  if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }

  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  writeReg16Bit(sensor, VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
  return true;
}


// ----------- Initialization ----------------------------

/*************************************************************************//**
* \fn init_vl53l0x_gpio()
*
* \brief Sets XSHUT pins to low on both sensors as recommended by Adafruit
*              
* \return None
*****************************************************************************/
void init_vl53l0x_gpio(void)
{
  dio_clear(BLE_WAKE);
  dio_clear(BT_CS);
  busyWait_delayms(10);
}

/*************************************************************************//**
* \fn init_vl53l0x()
*
* \brief Initializes required registers of both sensors, sets the signal rate 
* limit and timing budget for both
*
* \details To use both sensors at once, change addresses one at a time. Bring
* sensor 1 out of reset by setting XSHUT high and change to a new I2C address. 
* Repeat for second sensor. 
*
* \return None
*****************************************************************************/
void init_vl53l0x(void)
{ 
  /* Local Variables */
  I2C_return_status status = I2C_OK;
//  VL53L0X_DeviceParameters_t CurrentParameters;
  uint8_t StopVariable;
  static uint8_t data[2]; /* array used to write to VL53L0X */
  static uint8_t data_1[2];
  uint8_t data_temp = 0;
  
  // CHANGE ADDRESS
  dio_set(BLE_WAKE);
//  dio_clear(BLE_WAKE);
  busyWait_delayms(10);
  data_1[0] = VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS;
  data_1[1] = 0x16;
  status = i2c_write(0, 0x52, &data_1[0], sizeof(data_1));
  
  // INIT
  data_temp = 0x91;
  status |= i2c_write(0, I2C_ADDR_VL53L0X_WRITE_1, &data_temp, sizeof(data_temp));
  if (status == I2C_OK)
  {
    status = i2c_read(0, I2C_ADDR_VL53L0X_READ_1, &StopVariable, sizeof(StopVariable));
    PALDevDataSet(pMyDevice_0, StopVariable, StopVariable);
  }
  data[0] = VL53L0X_REG_SYSRANGE_START;
  data[1] = VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK;
  status = i2c_write(0, I2C_ADDR_VL53L0X_WRITE_1, &data[0], sizeof(data));
  
  // CHANGE ADDRESS
  dio_set(BT_CS);  
//  dio_set(BLE_WAKE);
  busyWait_delayms(10);
  data_1[0] = VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS;
  data_1[1] = 0x22;
  status = i2c_write(0, 0x52, &data_1[0], sizeof(data_1));
  
    // INIT
  data_temp = 0x91;
  status |= i2c_write(0, I2C_ADDR_VL53L0X_WRITE_2, &data_temp, sizeof(data_temp));
  if (status == I2C_OK)
  {
    status = i2c_read(0, I2C_ADDR_VL53L0X_READ_2, &StopVariable, sizeof(StopVariable));
    PALDevDataSet(pMyDevice_1, StopVariable, StopVariable);
  }
  data[0] = VL53L0X_REG_SYSRANGE_START;
  data[1] = VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK;
  status = i2c_write(0, I2C_ADDR_VL53L0X_WRITE_2, &data[0], sizeof(data));
  
  target_mm = target_inch*25.4;

  setSignalRateLimit(0, 0.65);
  setSignalRateLimit(1, 0.65);

  setMeasurementTimingBudget(1, 333000); // 333ms
  setMeasurementTimingBudget(2, 200000); // 200 ms
}


// ----------- Functions used to get Distance Measurements/Status ----------------------------

/*************************************************************************//**
* \fn VL53L0X_GetLimitCheckEnable()
*
* \brief Get specific limit check enable state
*
* \details Gets the enable state of a specific limit check. The limit check 
* is identified with the LimitCheckId.
*              
* @param[in]  Dev                     Struct that contains device information
* @param[in]  LimitCheckId            Limit Check ID
* @param[in]  pLimitCheckEnable       Pointer to the check limit enable value.
* (1 if enabled, 0 otherwise)
*
* \return VL53L0X_ERROR_NONE if success, VL53L0X_ERROR_INVALID_PARAMS if 
* LimitCheckId value is out of range.
*****************************************************************************/
VL53L0X_Error VL53L0X_GetLimitCheckEnable(VL53L0X_DEV Dev, uint16_t LimitCheckId, uint8_t *pLimitCheckEnable)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t Temp8;
	if (LimitCheckId >= VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS) {
		Status = VL53L0X_ERROR_INVALID_PARAMS;
		*pLimitCheckEnable = 0;
	} else {
		VL53L0X_GETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
			LimitCheckId, Temp8);
		*pLimitCheckEnable = Temp8;
	}
	return Status;
}


/*************************************************************************//**
* \fn VL53L0X_get_pal_range_status()
*
* \brief Gets range status of sensor
*              
* @param[in]  Dev                     Struct that contains device information
* @param[in]  DeviceRangeStatus       
* @param[in]  SignalRate
* @param[in]  EffectiveSpadRtnCount
* @param[in]  pRangingMeasurementData
* @param[in]  pPalRangeStatus
*
* \return None
*****************************************************************************/
void VL53L0X_get_pal_range_status(VL53L0X_DEV Dev, uint8_t DeviceRangeStatus, FixPoint1616_t SignalRate, uint16_t EffectiveSpadRtnCount, VL53L0X_RangingMeasurementData_t *pRangingMeasurementData, uint8_t *pPalRangeStatus)
{
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  uint8_t NoneFlag;
  uint8_t SigmaLimitflag = 0;
  uint8_t SignalRefClipflag = 0;
  uint8_t RangeIgnoreThresholdflag = 0;
  uint8_t SigmaLimitCheckEnable = 0;
  uint8_t SignalRateFinalRangeLimitCheckEnable = 0;
  uint8_t SignalRefClipLimitCheckEnable = 0;
  uint8_t RangeIgnoreThresholdLimitCheckEnable = 0;
  FixPoint1616_t SigmaEstimate;
  FixPoint1616_t SigmaLimitValue;
  FixPoint1616_t SignalRefClipValue;
  FixPoint1616_t RangeIgnoreThresholdValue;
  FixPoint1616_t SignalRatePerSpad;
  uint8_t DeviceRangeStatusInternal = 0;
  uint16_t tmpWord = 0;
  uint8_t Temp8;
  uint32_t Dmax_mm = 0;
  FixPoint1616_t LastSignalRefMcps;

  /*
   * VL53L0X has a good ranging when the value of the
   * DeviceRangeStatus = 11. This function will replace the value 0 with
   * the value 11 in the DeviceRangeStatus.
   * In addition, the SigmaEstimator is not included in the VL53L0X
   * DeviceRangeStatus, this will be added in the PalRangeStatus.
   */

  DeviceRangeStatusInternal = ((DeviceRangeStatus & 0x78) >> 3);

  if (DeviceRangeStatusInternal == 0 ||
          DeviceRangeStatusInternal == 5 ||
          DeviceRangeStatusInternal == 7 ||
          DeviceRangeStatusInternal == 12 ||
          DeviceRangeStatusInternal == 13 ||
          DeviceRangeStatusInternal == 14 ||
          DeviceRangeStatusInternal == 15
                  ) {
          NoneFlag = 1;
  } else {
          NoneFlag = 0;
  }

  if (NoneFlag == 1) {
          *pPalRangeStatus = 255;	 /* NONE */
  } else if (DeviceRangeStatusInternal == 1 ||
                          DeviceRangeStatusInternal == 2 ||
                          DeviceRangeStatusInternal == 3) {
          *pPalRangeStatus = 5; /* HW fail */
  } else if (DeviceRangeStatusInternal == 6 ||
                          DeviceRangeStatusInternal == 9) {
          *pPalRangeStatus = 4;  /* Phase fail */
  } else if (DeviceRangeStatusInternal == 8 ||
                          DeviceRangeStatusInternal == 10 ||
                          SignalRefClipflag == 1) {
          *pPalRangeStatus = 3;  /* Min range */
  } else if (DeviceRangeStatusInternal == 4 ||
                          RangeIgnoreThresholdflag == 1) {
          *pPalRangeStatus = 2;  /* Signal Fail */
  } else if (SigmaLimitflag == 1) {
          *pPalRangeStatus = 1;  /* Sigma	 Fail */
  } else {
          *pPalRangeStatus = 0; /* Range Valid */
  }

  /* DMAX only relevant during range error */
  if (*pPalRangeStatus == 0)
          pRangingMeasurementData->RangeDMaxMilliMeter = 0;

  /* fill the Limit Check Status */

  VL53L0X_GetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, &SignalRateFinalRangeLimitCheckEnable);

  if (Status == VL53L0X_ERROR_NONE) {
          if ((SigmaLimitCheckEnable == 0) || (SigmaLimitflag == 1))
                  Temp8 = 1;
          else
                  Temp8 = 0;
          VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksStatus, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, Temp8);

          if ((DeviceRangeStatusInternal == 4) || (SignalRateFinalRangeLimitCheckEnable == 0))
                  Temp8 = 1;
          else
                  Temp8 = 0;
          VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksStatus, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, Temp8);
  }
}

/*************************************************************************//**
* \fn get_ranging_measurement_data()
*
* \brief Gets ranging measurement data from sensor
*              
* @param[in]  Dev                     Struct that contains device information
* @param[in]  pRangingMeasurementData
* @param[in]  write_addr
* @param[in]  read_addr
*
* \return None
*****************************************************************************/
void get_ranging_measurement_data(VL53L0X_DEV Dev, VL53L0X_RangingMeasurementData_t *pRangingMeasurementData, uint8_t write_addr, uint8_t read_addr)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t DeviceRangeStatus;
    uint8_t RangeFractionalEnable;
    uint8_t PalRangeStatus;
    uint8_t XTalkCompensationEnable;
    uint16_t AmbientRate;
    FixPoint1616_t SignalRate;
    uint16_t XTalkCompensationRateMegaCps;
    uint16_t EffectiveSpadRtnCount;
    uint16_t tmpuint16;
    uint16_t XtalkRangeMilliMeter;
//    uint16_t LinearityCorrectiveGain;
    uint8_t localBuffer[12];
    VL53L0X_RangingMeasurementData_t LastRangeDataBuffer;
    uint8_t data_temp;

    // Read RangeStatus Register
    data_temp = VL53L0X_REG_RESULT_RANGE_STATUS;
    i2c_write(0, write_addr, &data_temp, sizeof(data_temp));
    i2c_read(0, read_addr, (uint8_t*)&localBuffer[0], 12);

//    pRangingMeasurementData->ZoneId = 0; /* Only one zone */
//    pRangingMeasurementData->TimeStamp = 0; /* Not Implemented */
    tmpuint16 = VL53L0X_MAKEUINT16(localBuffer[11], localBuffer[10]);
    pRangingMeasurementData->MeasurementTimeUsec = 0;
    SignalRate = VL53L0X_FIXPOINT97TOFIXPOINT1616(VL53L0X_MAKEUINT16(localBuffer[7], localBuffer[6])); /* peak_signal_count_rate_rtn_mcps */
    pRangingMeasurementData->SignalRateRtnMegaCps = SignalRate; 
    AmbientRate = VL53L0X_MAKEUINT16(localBuffer[9], localBuffer[8]);
    pRangingMeasurementData->AmbientRateRtnMegaCps = VL53L0X_FIXPOINT97TOFIXPOINT1616(AmbientRate);
    EffectiveSpadRtnCount = VL53L0X_MAKEUINT16(localBuffer[3], localBuffer[2]);
    pRangingMeasurementData->EffectiveSpadRtnCount = EffectiveSpadRtnCount; /* EffectiveSpadRtnCount is 8.8 format */
    DeviceRangeStatus = localBuffer[0];

//    /* Get Linearity Corrective Gain */
//    LinearityCorrectiveGain = PALDevDataGet(Dev, LinearityCorrectiveGain);
    /* Get ranging configuration */
    RangeFractionalEnable = PALDevDataGet(Dev, RangeFractionalEnable);

////     RANGE FRACTIONAL
//     if (RangeFractionalEnable) {
//         pRangingMeasurementData->RangeMilliMeter = (uint16_t)((tmpuint16) >> 2);
//         pRangingMeasurementData->RangeFractionalPart = (uint8_t)((tmpuint16 & 0x03) << 6);
//     } else {
         pRangingMeasurementData->RangeMilliMeter = tmpuint16;
//         pRangingMeasurementData->RangeFractionalPart = 0;
//     }

    /*
        * For a standard definition of RangeStatus, this should
        * return 0 in case of good result after a ranging
        * The range status depends on the device so call a device
        * specific function to obtain the right Status.
        */
    VL53L0X_get_pal_range_status(Dev, DeviceRangeStatus,
        SignalRate, EffectiveSpadRtnCount,
        pRangingMeasurementData, &PalRangeStatus);

    if (Status == VL53L0X_ERROR_NONE)
        pRangingMeasurementData->RangeStatus = PalRangeStatus;

	if (Status == VL53L0X_ERROR_NONE) {
		/* Copy last read data into Dev buffer */
		LastRangeDataBuffer = PALDevDataGet(Dev, LastRangeMeasure);
		LastRangeDataBuffer.RangeMilliMeter =
			pRangingMeasurementData->RangeMilliMeter;
		LastRangeDataBuffer.RangeFractionalPart =
			pRangingMeasurementData->RangeFractionalPart;
		LastRangeDataBuffer.RangeDMaxMilliMeter =
			pRangingMeasurementData->RangeDMaxMilliMeter;
		LastRangeDataBuffer.MeasurementTimeUsec =
			pRangingMeasurementData->MeasurementTimeUsec;
		LastRangeDataBuffer.SignalRateRtnMegaCps =
			pRangingMeasurementData->SignalRateRtnMegaCps;
		LastRangeDataBuffer.AmbientRateRtnMegaCps =
			pRangingMeasurementData->AmbientRateRtnMegaCps;
		LastRangeDataBuffer.EffectiveSpadRtnCount =
			pRangingMeasurementData->EffectiveSpadRtnCount;
		LastRangeDataBuffer.RangeStatus =
			pRangingMeasurementData->RangeStatus;

    PALDevDataSet(Dev, LastRangeMeasure, LastRangeDataBuffer);
}
}


/*************************************************************************//**
* \fn get_distance_vl53l0x()
*
* \brief Adjusts distance using measurement data from both sensors
*
* \return None
*****************************************************************************/
void get_distance_vl53l0x(void)
{
  I2C_return_status status = I2C_OK;
  uint16_t dist_sum = 0;
  uint16_t total = 0;
  uint16_t dist_sum_1 = 0;
  uint16_t total_1 = 0;
  uint16_t ave_1 = 0;
  uint16_t dist_sum_2 = 0;
  uint16_t total_2 = 0;
  uint16_t ave_2 = 0;
  
  VL53L0X_RangingMeasurementData_t    RangingMeasurementData_0;
  VL53L0X_RangingMeasurementData_t   *pRangingMeasurementData_0 = &RangingMeasurementData_0;
  
  VL53L0X_RangingMeasurementData_t    RangingMeasurementData_1;
  VL53L0X_RangingMeasurementData_t   *pRangingMeasurementData_1 = &RangingMeasurementData_1;
  
  get_ranging_measurement_data(pMyDevice_0, pRangingMeasurementData_0, I2C_ADDR_VL53L0X_WRITE_1, I2C_ADDR_VL53L0X_READ_1);
  get_ranging_measurement_data(pMyDevice_1, pRangingMeasurementData_1, I2C_ADDR_VL53L0X_WRITE_2, I2C_ADDR_VL53L0X_READ_2);
  
  actual_dist_1 = pRangingMeasurementData_0->RangeMilliMeter; // actual distance in mm
  actual_dist_2 = pRangingMeasurementData_1->RangeMilliMeter; // actual distance in mm
  
  rangestatus_0 = pRangingMeasurementData_0->RangeStatus; // range status
  rangestatus_1 = pRangingMeasurementData_1->RangeStatus;
  
  // calc change b/w last dist and new dist from two sensors (to see variance)
  if (last_actual_1 != actual_dist_1) 
  { 
    diff_dist_1[4] = diff_dist_1[3];
    diff_dist_1[3] = diff_dist_1[2];
    diff_dist_1[2] = diff_dist_1[1];
    diff_dist_1[1] = diff_dist_1[0];
    diff_dist_1[0] = abs((int16_t)actual_dist_1 - (int16_t)last_actual_1);
  }
 
  if (last_actual_2 != actual_dist_2) 
  { 
    diff_dist_2[4] = diff_dist_2[3];
    diff_dist_2[3] = diff_dist_2[2];
    diff_dist_2[2] = diff_dist_2[1];
    diff_dist_2[1] = diff_dist_2[0];
    diff_dist_2[0] = abs((int16_t)actual_dist_2 - (int16_t)last_actual_2);
  }
  
  // calc to get rolling average of 10 last adjusted distances
// for (int i = 0; i < sizeof(last_adj); i++)
  for (int i = 0; i < 5; i++)
 {
   if (last_adj[i] > 100 && last_adj[i] < 500)
   {
     dist_sum += last_adj[i];
     total += 1;
   }
 }
  
 // if both data points are invalid
  if (actual_dist_1 > 500 || actual_dist_1 < 100 || rangestatus_0 != 0)
  {
    if (actual_dist_2 > 500 || actual_dist_2 < 100 || rangestatus_1 != 0)
    {
      distance = last_adj[0];
    }
  }
  
  // if sensor 2 is invalid but sensor 1 is valid
  if (actual_dist_2 > 500 || actual_dist_2 < 100 || rangestatus_1 != 0)
  {
    if (actual_dist_1 < 500 && actual_dist_1 > 100)
    {
      // if there is a difference of more than one inch (variance)
      if (diff_dist_1[0] > 25) 
      {
        for (int i = 0;  i < sizeof(last_changed_1); i++)
        {
          if (last_changed_1[i] > 100 && last_changed_1[i] < 500)
          {
            dist_sum_1 += last_changed_1[i];
            total_1 += 1;
          }
        }
        // 10 point rolling average to account for large variance
        if (total_1 != 0)
        {
          ave_1 = dist_sum_1/total_1;
          distance = (dist_sum + ave_1) / (total + 1);
        }
        else if (total != 0)
        {
          distance = (dist_sum) / total;
        }
        else
        {
          distance = actual_dist_1;
        }
      }
      
      else
      {
        distance = (dist_sum + actual_dist_1) / (total + 1);
      }
    }
  }
  
  // if sensor 1 is invalid but sensor 2 is valid
  if (actual_dist_1 > 500 || actual_dist_1 < 100 || rangestatus_0 != 0)
  {
    if (actual_dist_2 < 500 && actual_dist_2 > 100)
    {
      // if there is a difference of more than one inch
      if (diff_dist_2[0] > 25) 
      {
        for (int i = 0;  i < sizeof(last_changed_2); i++) // calculate rolling average of last 5 datapoints for sensor 2
        {
          if (last_changed_2[i] > 100 && last_changed_2[i] < 500)
          {
            dist_sum_2 += last_changed_2[i];
            total_2 += 1;
          }
        }
        // 10 point rolling average to account for large variance
        if (total_2 != 0)
        {
          ave_2 = dist_sum_2/total_2;
          distance = (dist_sum + ave_2) / (total + 1);
        }
        else if (total != 0)
        {
          distance = (dist_sum) / total;
        }
        else
        {
          distance = actual_dist_2;
        }
      }
      
      else
      {
        distance = (dist_sum + actual_dist_2) / (total + 1);
      }
    }
  }
  
  // if both data points are valid
  if (actual_dist_1 < 500 && actual_dist_1 > 100 && rangestatus_0 == 0)
  {
    if (actual_dist_2 < 500 && actual_dist_2 > 100 && rangestatus_1 == 0)
    {
        // if there is a steady slope before current invalid data point (momentarily - for inside/ambient light)
      if (diff_dist_1[0] < 25 && diff_dist_1[1] < 25 && diff_dist_1[2] < 25 && diff_dist_2[1] < 25 && diff_dist_2[2] < 25 && diff_dist_2[3] < 25 && diff_dist_2[4] < 25)
        {
          distance = (actual_dist_1 + actual_dist_2) / 2;
        }
        // if there is more variance in data (for bright sunlight)
        else
        {
          distance = (dist_sum + actual_dist_1 + actual_dist_2) /(total + 2);
        }
    }
  }
 
   if (last_actual_1 != actual_dist_1) // if goes to 20 
    { 
      last_changed_1[4] = last_changed_1[3];
      last_changed_1[3] = last_changed_1[2];
      last_changed_1[2] = last_changed_1[1];
      last_changed_1[1] = last_changed_1[0];
      last_changed_1[0] = actual_dist_1; // most recent
    }
  
   if (last_actual_2 != actual_dist_2) // if goes to 20 
    { 
      last_changed_2[4] = last_changed_2[3];
      last_changed_2[3] = last_changed_2[2];
      last_changed_2[2] = last_changed_2[1];
      last_changed_2[1] = last_changed_2[0];
      last_changed_2[0] = actual_dist_2; // most recent
    }
    
   if (last_temp != distance)
   {
      last_adj[9] = last_adj[8];
      last_adj[8] = last_adj[7];
      last_adj[7] = last_adj[6];
      last_adj[6] = last_adj[5];
      last_adj[5] = last_adj[4];
      last_adj[4] = last_adj[3];
      last_adj[3] = last_adj[2];
      last_adj[2] = last_adj[1];
      last_adj[1] = last_adj[0];
      last_adj[0] = distance; // most recent
   }
    
    dist_sum = 0;
    total = 0;
    last_temp = distance;
    last_actual_1 = actual_dist_1;
    last_actual_2 = actual_dist_2;
}

/*************************************************************************//**
* \fn get_hole_depth()
*
* \brief Calculates the hole depth depending on adjusted distance
*
* \return Hole depth
*****************************************************************************/
int16_t get_hole_depth() 
{ 
  int32_t diff = 0;
  if (distance == 0)
  {
    diff = 0;
  }
  else 
  {
    diff = (int32_t)start_distance - (int32_t)distance;
  }
  
  return diff;
}
