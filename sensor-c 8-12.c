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
VL53L0X_DevData_t MyDevice;
VL53L0X_DevData_t *pMyDevice = &MyDevice;

uint16_t distance;
uint16_t actual_dist_1;
uint16_t actual_dist_2;
uint8_t devicerangestatus_0;
uint8_t devicerangestatus_1;
uint16_t devicerangestatusinternal_0 = 0;
uint16_t devicerangestatusinternal_1 = 0;
uint16_t rangestatus_0;
uint16_t rangestatus_1;
static uint16_t last_actual_1;
static uint16_t last_actual_2;

static int16_t last_temp_ten[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static int16_t last_actual_ten[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int16_t diff_dist_1 = 0;
int16_t diff_dist_2 = 0;
static uint16_t last_changed_ten[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint16_t last_temp;

//float_t dist_sensor_tip_inch = 4.93;
//float_t dist_sensor_tip_mm = 125.18; // distance from sensor placement to tip of drill (4.93 in)

float_t dist_drill_bit_inch = 3; // distance from drill tip to end of drill bit
float_t dist_drill_bit_mm;

// CHANGE
float_t target_inch = 1; // 2, 2.5
float_t target_mm;

uint16_t distance_stop;
uint16_t start_distance;
static uint16_t current_distance;

int16_t range1;
int16_t range2;
int16_t range3;
int16_t range4;
int16_t range5;
int16_t range6;

uint32_t measurement_timing_budget_us;
/*****************************************************************************
 * FUNCTION PROTOTYPES
 *****************************************************************************/
void init_vl53l0x(void);
// void data_init(VL53L0X_DevData_t *dev);
// void static_init(VL53L0X_DevData_t *dev);
void get_distance_vl53l0x(void);
void sensor_data_read_complete(I2C_return_status status);
void refresh_sensor_data_read(I2C_return_status status);
I2C_return_status get_ranging_measurement_data(VL53L0X_DevData_t *dev, VL53L0X_RangingMeasurementData_t *pRangingMeasurementData);

/*****************************************************************************
 * VARIABLE DECLARATIONS
 *****************************************************************************/
/** VL53L0X parameters local to the module */

uint8_t SysRangeStatusRegister;
//uint8_t localBuffer[12];
uint8_t localBuffer0[2];
uint8_t localBuffer1[2];

/*************************************************************************//**
* \fn Read Registers
*
* \brief 
*
*\calls 
*****************************************************************************/
uint8_t readReg(uint8_t sensor, uint8_t reg)
{
  uint8_t ret_val;
  uint8_t data_temp;
  data_temp = reg;
  if (sensor == 0)
  {
    i2c_write(0, I2C_ADDR_VL53L0X_WRITE_0, &data_temp, sizeof(data_temp));
    i2c_read(0, I2C_ADDR_VL53L0X_READ_0, &ret_val, sizeof(ret_val));
  }
  else 
  {
    i2c_write(0, I2C_ADDR_VL53L0X_WRITE_1, &data_temp, sizeof(data_temp));
    i2c_read(0, I2C_ADDR_VL53L0X_READ_1, &ret_val, sizeof(ret_val));
  }
  
  return ret_val;
}

uint16_t readReg16Bit(uint8_t sensor, uint8_t reg)
{
  uint16_t ret_val;
  uint8_t ret_data[2];
  uint8_t data_temp;
  data_temp = reg;
  if (sensor == 0)
  {
    i2c_write(0, I2C_ADDR_VL53L0X_WRITE_0, &data_temp, sizeof(data_temp));
    i2c_read(0, I2C_ADDR_VL53L0X_READ_0, (uint8_t*)&ret_data[0], sizeof(ret_data));
  }
  else
  {
    i2c_write(0, I2C_ADDR_VL53L0X_WRITE_1, &data_temp, sizeof(data_temp));
    i2c_read(0, I2C_ADDR_VL53L0X_READ_1, (uint8_t*)&ret_data[0], sizeof(ret_data));
  }
  ret_val = (ret_data[1]<<8)|ret_data[0];
  return ret_val;
}

void writeReg16Bit(uint8_t sensor, uint8_t reg, uint16_t value)
{
  uint8_t data[3];
  data[0] = reg;
  data[1] = ((value >> 8) & 0xFF);
  data[2] = (value & 0xFF);
  if (sensor == 0)
  {
    i2c_write(0, I2C_ADDR_VL53L0X_WRITE_0, &data[0], sizeof(data));
  }
  else
  {
    i2c_write(0, I2C_ADDR_VL53L0X_WRITE_1, &data[0], sizeof(data));
  }
}


/*************************************************************************//**
* \fn Set Timing Budget
*
* \brief 
*
*\calls 
*****************************************************************************/

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}

void getSequenceStepEnables(uint8_t sensor, SequenceStepEnables * enables)
{
  uint8_t sequence_config = readReg(sensor, SYSTEM_SEQUENCE_CONFIG);
  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
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

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
uint16_t decodeTimeout(uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) <<
         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
void getSequenceStepTimeouts(uint8_t sensor, SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts)
{
  timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(sensor, VcselPeriodPreRange);

  timeouts->msrc_dss_tcc_mclks = readReg(sensor, MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us =
    timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->pre_range_mclks =
    decodeTimeout(readReg16Bit(sensor, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->pre_range_us =
    timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(sensor, VcselPeriodFinalRange);

  timeouts->final_range_mclks =
    decodeTimeout(readReg16Bit(sensor, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

  if (enables->pre_range)
  {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }

  timeouts->final_range_us =
    timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                               timeouts->final_range_vcsel_period_pclks);
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
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

    uint32_t final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(final_range_timeout_us,
                                 timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range)
    {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    writeReg16Bit(sensor, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(final_range_timeout_mclks)); // change

    // set_sequence_step_timeout() end

    measurement_timing_budget_us = budget_us; // store for internal reuse
  }
  return true;
}

/*************************************************************************//**
* \fn Set Limit Check Value
*
* \brief 
*
*\calls 
*****************************************************************************/
void VL53L0X_SetLimitCheckValue (uint16_t LimitCheckId, FixPoint1616_t LimitCheckValue)
{
  switch (LimitCheckId) {
    
    case VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE:
        /* internal computation: */
        VL53L0X_SETARRAYPARAMETERFIELD(pMyDevice, LimitChecksValue, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, LimitCheckValue);
	break;
        
    case VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
        writeReg16Bit(0, VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, VL53L0X_FIXPOINT1616TOFIXPOINT97(LimitCheckValue));
        break;
  }
}

void VL53L0X_SetLimitCheckEnable (uint16_t LimitCheckId, FixPoint1616_t LimitCheckEnable)
{
  FixPoint1616_t TempFix1616 = 0;
  uint8_t LimitCheckEnableInt = 1;
  VL53L0X_GETARRAYPARAMETERFIELD(pMyDevice, LimitChecksValue, LimitCheckId, TempFix1616);
  switch (LimitCheckId) {
    
      case VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE:
          /* internal computation: */
          VL53L0X_SETARRAYPARAMETERFIELD(pMyDevice, LimitChecksEnable, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, LimitCheckEnableInt);
          break;

      case VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
          writeReg16Bit(0, VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, VL53L0X_FIXPOINT1616TOFIXPOINT97(TempFix1616));
          break;
  }
}

/*************************************************************************//**
* \fn init_vl53l0x
*
* \brief function to be called in app_tool_specific.c
*
*\calls 
*****************************************************************************/
void init_vl53l0x_gpio(void)
{
  dio_clear(BLE_WAKE);
  dio_clear(BT_CS);
  busyWait_delayms(10);
}

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
  busyWait_delayms(10);
  data_1[0] = VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS;
  data_1[1] = 0x16;
  status = i2c_write(0, 0x52, &data_1[0], sizeof(data_1));
  
  // INIT
  data_temp = 0x91;
  status |= i2c_write(0, I2C_ADDR_VL53L0X_WRITE_0, &data_temp, sizeof(data_temp));
  if (status == I2C_OK)
  {
    status = i2c_read(0, I2C_ADDR_VL53L0X_READ_0, &StopVariable, sizeof(StopVariable));
    PALDevDataSet(pMyDevice, StopVariable, StopVariable);
  }
  data[0] = VL53L0X_REG_SYSRANGE_START;
  data[1] = VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK;
  status = i2c_write(0, I2C_ADDR_VL53L0X_WRITE_0, &data[0], sizeof(data));
  
  // CHANGE ADDRESS
  dio_set(BT_CS);  
  busyWait_delayms(10);
  data_1[0] = VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS;
  data_1[1] = 0x22;
  status = i2c_write(0, 0x52, &data_1[0], sizeof(data_1));
  
// INIT
  data_temp = 0x91;
  status |= i2c_write(0, I2C_ADDR_VL53L0X_WRITE_1, &data_temp, sizeof(data_temp));
  if (status == I2C_OK)
  {
    status = i2c_read(0, I2C_ADDR_VL53L0X_READ_1, &StopVariable, sizeof(StopVariable));
    PALDevDataSet(pMyDevice, StopVariable, StopVariable);
  }
  data[0] = VL53L0X_REG_SYSRANGE_START;
  data[1] = VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK;
  status = i2c_write(0, I2C_ADDR_VL53L0X_WRITE_1, &data[0], sizeof(data));
  
  // Define target distance = (length from sensor to tip) + (length from tip of drill to tip of drill bit) - (target depths)
  
  dist_drill_bit_mm = dist_drill_bit_inch*25.4;
  target_mm = target_inch*25.4;
//  distance_stop = (uint16_t)round(dist_sensor_tip_mm + dist_drill_bit_mm - target_mm);
  
  VL53L0X_SetLimitCheckEnable (VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
  VL53L0X_SetLimitCheckEnable (VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
  VL53L0X_SetLimitCheckValue (VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(18*65536));
  VL53L0X_SetLimitCheckValue (VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25*65536));
//  setMeasurementTimingBudget(0, 200000);
  setMeasurementTimingBudget(0, 500000);
  //  setMeasurementTimingBudget(1, 200000);
}

/*************************************************************************//**
* \fn get_distance_vl53l0x
*
* \brief function to be called in app_tool_specific.c
*
*\calls 
*****************************************************************************/

void get_distance_vl53l0x(void)
{
  I2C_return_status status = I2C_OK;
  VL53L0X_RangingMeasurementData_t    RangingMeasurementData;
  VL53L0X_RangingMeasurementData_t   *pRangingMeasurementData = &RangingMeasurementData;
  uint8_t NewDatReady=0;
  get_ranging_measurement_data(pMyDevice, pRangingMeasurementData);
  
//  actual_dist = pRangingMeasurementData->RangeMilliMeter; // actual distance in mm
  // calc change b/w last dist and new dist from two sensors
  if (last_actual_1 != actual_dist_1) 
  { 
    diff_dist_1 = abs((int16_t)actual_dist_1 - (int16_t)last_actual_1);
  }
 
  if (last_actual_2 != actual_dist_2) 
  { 
    diff_dist_2 = abs((int16_t)actual_dist_2 - (int16_t)last_actual_2);
  }
 
  // if going from invalid state to valid state
  if (actual_dist_1 > 30 && actual_dist_1 < 1000)
  {
    range1 = abs(last_changed_ten[0] - last_changed_ten[1]);
    range2 = abs(last_changed_ten[1] - last_changed_ten[2]);
    range3 = abs(last_changed_ten[2] - last_changed_ten[3]);
    range4 = abs(last_changed_ten[3] - last_changed_ten[4]);
    range5 = abs(last_changed_ten[4] - last_changed_ten[5]);
    range6 = abs(last_changed_ten[5] - last_changed_ten[6]);
    
    if (range1 < 50 && range2 < 50 && range3 < 50 && range4 > 50 && range5 > 50 &&  range6 > 50)
    {
      if (abs(actual_dist_1 - actual_dist_2) < 25)
      {
        distance = (actual_dist_2 + actual_dist_1)/2;
      }
      else
      {
        distance = actual_dist_1;
      }
    }
  }
   
  // if change of distance is too great - one or neither is in range
  if (diff_dist_1 >= 25) // sensor 1 peak/dip
      { 
        if (diff_dist_2 <= 25) // if sensor 2 valid
        {
          distance = actual_dist_2; 
        }
        else // if both invalid
        {
          distance = last_temp;
        }
      }
   else if (diff_dist_2 >= 25) // sensor 2 peak/dip
   {
     if (diff_dist_1 <= 25) // if sensor 1 valid
        {
          distance = actual_dist_1; 
        }
   }
 
   else // if it is reasonable number
   {
     if (actual_dist_1 < 100 || actual_dist_1 > 1000)
     {
       distance = last_temp;
     }
     else { 
//          distance = actual_dist;
       distance = (actual_dist_2 + actual_dist_1)/2;
     }
   }
 
   if (last_actual_1 != actual_dist_1) // if goes to 20 
    { 
      last_changed_ten[9] = last_changed_ten[8];
      last_changed_ten[8] = last_changed_ten[7];
      last_changed_ten[7] = last_changed_ten[6];
      last_changed_ten[6] = last_changed_ten[5];
      last_changed_ten[5] = last_changed_ten[4];
      last_changed_ten[4] = last_changed_ten[3];
      last_changed_ten[3] = last_changed_ten[2];
      last_changed_ten[2] = last_changed_ten[1];
      last_changed_ten[1] = last_changed_ten[0];
      last_changed_ten[0] = actual_dist_1; // most recent
    }
    
    last_actual_ten[9] = last_actual_ten[8];
    last_actual_ten[8] = last_actual_ten[7]; // most old
    last_actual_ten[7] = last_actual_ten[6];
    last_actual_ten[6] = last_actual_ten[5];
    last_actual_ten[5] = last_actual_ten[4];
    last_actual_ten[4] = last_actual_ten[3];
    last_actual_ten[3] = last_actual_ten[2];
    last_actual_ten[2] = last_actual_ten[1];
    last_actual_ten[1] = last_actual_ten[0];
    last_actual_ten[0] = actual_dist_1; // most recent
    
    last_temp = distance;
    last_actual_1 = actual_dist_1;
    last_actual_2 = actual_dist_2;
//    temp_sum = 0;
}

/*************************************************************************//**
* \fn sensor_data_read_complete
*
* \brief function to be called in readAsync/writeAsync
*
*\calls 
*****************************************************************************/
  
void sensor_data_read_complete(I2C_return_status status)
{
}

/*************************************************************************//**
* \fn refresh_sensor_data
*
* \brief function to be called in writeAsync
*
*\calls 
*****************************************************************************/
void refresh_sensor_data(I2C_return_status status)
{
    if(status == I2C_OK)
    {
//      I2C_return_status read = i2c_readAsync(0, I2C_ADDR_VL53L0X_READ, (uint8_t*)&localBuffer[0], 12, &sensor_data_read_complete);
        I2C_return_status read = i2c_readAsync(0, I2C_ADDR_VL53L0X_READ_0, (uint8_t*)&localBuffer0[0], 2U, &sensor_data_read_complete);
    }
}




I2C_return_status get_ranging_measurement_data(VL53L0X_DevData_t *dev, VL53L0X_RangingMeasurementData_t *pRangingMeasurementData, uint8_t read_addr, uint8_t write_addr)
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
	uint16_t LinearityCorrectiveGain;
	uint8_t localBuffer[12];
	VL53L0X_RangingMeasurementData_t LastRangeDataBuffer;

    data_temp = VL53L0X_REG_DISTANCE;
    i2c_write(0, I2C_ADDR_VL53L0X_WRITE_0, &data_temp, sizeof(data_temp));
    i2c_read(0, I2C_ADDR_VL53L0X_READ_0, (uint8_t*)&localBuffer0[0], 2U);


/*************************************************************************//**
* \fn get_ranging_measurement_data
*
* \brief retrieves measurements from device for a given setup.
*        Get data from last successful ranging measurement.
*
*\calls 
*****************************************************************************/
I2C_return_status get_ranging_measurement_data(VL53L0X_DevData_t *dev, VL53L0X_RangingMeasurementData_t *pRangingMeasurementData)
{
  I2C_return_status status = I2C_OK;
  uint16_t tmpuint16;
  VL53L0X_RangingMeasurementData_t LastRangeDataBuffer;
  uint8_t data_temp;

  // SENSOR 1
//  i2c_writeAsync(0, I2C_ADDR_VL53L0X_WRITE, &dist_reg, sizeof(dist_reg), &refresh_sensor_data);
  data_temp = VL53L0X_REG_DISTANCE;
  i2c_write(0, I2C_ADDR_VL53L0X_WRITE_0, &data_temp, sizeof(data_temp));
  i2c_read(0, I2C_ADDR_VL53L0X_READ_0, (uint8_t*)&localBuffer0[0], 2U);
  
//  tmpuint16 = VL53L0X_MAKEUINT16(localBuffer[11], localBuffer[10]);
  tmpuint16  = VL53L0X_MAKEUINT16(localBuffer0[1], localBuffer0[0]);
  actual_dist_1 = tmpuint16;
  
  data_temp = VL53L0X_REG_RESULT_RANGE_STATUS;
  i2c_write(0, I2C_ADDR_VL53L0X_WRITE_0, &data_temp, sizeof(data_temp));
  i2c_read(0, I2C_ADDR_VL53L0X_READ_0, &devicerangestatus_0, sizeof(devicerangestatus_0));
  devicerangestatusinternal_0 = ((devicerangestatus_0 & 0x78) >> 3);
  
  // SENSOR 2
  data_temp = VL53L0X_REG_DISTANCE;
  i2c_write(0, I2C_ADDR_VL53L0X_WRITE_1, &data_temp, sizeof(data_temp));
  i2c_read(0, I2C_ADDR_VL53L0X_READ_1, (uint8_t*)&localBuffer1[0], 2U);
  actual_dist_2 = VL53L0X_MAKEUINT16(localBuffer1[1], localBuffer1[0]);
  
  data_temp = VL53L0X_REG_RESULT_RANGE_STATUS;
  i2c_write(0, I2C_ADDR_VL53L0X_WRITE_1, &data_temp, sizeof(data_temp));
  i2c_read(0, I2C_ADDR_VL53L0X_READ_1, &devicerangestatus_1, sizeof(devicerangestatus_1));
  devicerangestatusinternal_1 = ((devicerangestatus_1 & 0x78) >> 3);
  
  
  if (devicerangestatusinternal_0 == 1 || devicerangestatusinternal_0 == 2 || devicerangestatusinternal_0 == 3)
  {
    rangestatus_0 = 5;
  }
  else if (devicerangestatusinternal_0 == 6 || devicerangestatusinternal_0 == 9)
  {
    rangestatus_0 = 4;
  }
  else if (devicerangestatusinternal_0 == 8 || devicerangestatusinternal_0 == 10)
  {
    rangestatus_0 = 3;
  }
  else if (devicerangestatusinternal_0 == 4 || devicerangestatusinternal_0 == 1)
  {
    rangestatus_0 = 2;
  }
  else
  {
    rangestatus_0 = 0;
  }
  
  if (devicerangestatusinternal_1 == 1 || devicerangestatusinternal_1 == 2 || devicerangestatusinternal_1 == 3)
  {
    rangestatus_1 = 5;
  }
  else if (devicerangestatusinternal_1 == 6 || devicerangestatusinternal_1 == 9)
  {
    rangestatus_1 = 4;
  }
  else if (devicerangestatusinternal_1 == 8 || devicerangestatusinternal_1 == 10)
  {
    rangestatus_1 = 3;
  }
  else if (devicerangestatusinternal_1 == 4 || devicerangestatusinternal_1 == 1)
  {
    rangestatus_1 = 2;
  }
  else
  {
    rangestatus_1 = 0;
  }
  
  pRangingMeasurementData->RangeMilliMeter = tmpuint16;
  pRangingMeasurementData->RangeFractionalPart = 0;
  
  LastRangeDataBuffer = PALDevDataGet(dev, LastRangeMeasure); 

  LastRangeDataBuffer.RangeMilliMeter = pRangingMeasurementData->RangeMilliMeter;

  PALDevDataSet(dev, LastRangeMeasure, LastRangeDataBuffer);
  return status;
}

// possible function prototype not finished
int16_t get_hole_depth() 
{
  // get starting measurement = when motor transitions to motor run state
  // reset starting meaurement = when motor transitions out of motor run state
  // modify to calc depth only when in motor run state
  
//  current_distance = distance; // make static
  int32_t diff = 0;
  if (distance == 0)
  {
    diff = 0;
  }
  else 
  {
    diff = (int32_t)start_distance - (int32_t)distance;
  }
//  if (diff < 0)
//  {
//    diff = 0;
//  }
  
  return diff;
}

