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
  
  return diff;
}

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

	/*
	 * Check if Sigma limit is enabled, if yes then do comparison with limit
	 * value and put the result back into pPalRangeStatus.
	 */
	if (Status == VL53L0X_ERROR_NONE)
		Status =  VL53L0X_GetLimitCheckEnable(Dev,
			VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
			&SigmaLimitCheckEnable);

	if ((SigmaLimitCheckEnable != 0) && (Status == VL53L0X_ERROR_NONE)) {
		/*
		* compute the Sigma and check with limit
		*/
		Status = VL53L0X_calc_sigma_estimate(
			Dev,
			pRangingMeasurementData,
			&SigmaEstimate,
			&Dmax_mm);
		if (Status == VL53L0X_ERROR_NONE)
			pRangingMeasurementData->RangeDMaxMilliMeter = Dmax_mm;

		if (Status == VL53L0X_ERROR_NONE) {
			Status = VL53L0X_GetLimitCheckValue(Dev,
				VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
				&SigmaLimitValue);

			if ((SigmaLimitValue > 0) &&
				(SigmaEstimate > SigmaLimitValue))
					/* Limit Fail */
					SigmaLimitflag = 1;
		}
	}

	if (Status == VL53L0X_ERROR_NONE) {
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
	}

	/* DMAX only relevant during range error */
	if (*pPalRangeStatus == 0)
		pRangingMeasurementData->RangeDMaxMilliMeter = 0;

	/* fill the Limit Check Status */

	Status =  VL53L0X_GetLimitCheckEnable(Dev,
			VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
			&SignalRateFinalRangeLimitCheckEnable);

	if (Status == VL53L0X_ERROR_NONE) {
		if ((SigmaLimitCheckEnable == 0) || (SigmaLimitflag == 1))
			Temp8 = 1;
		else
			Temp8 = 0;
		VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksStatus,
				VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, Temp8);

		if ((DeviceRangeStatusInternal == 4) ||
				(SignalRateFinalRangeLimitCheckEnable == 0))
			Temp8 = 1;
		else
			Temp8 = 0;
		VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksStatus,
				VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
				Temp8);
	}
}

/*************************************************************************//**
* \fn get_ranging_measurement_data
*
* \brief 
*
*\calls 
*****************************************************************************/
void get_ranging_measurement_data(VL53L0X_DevData_t *dev, VL53L0X_RangingMeasurementData_t *pRangingMeasurementData, uint8_t write_addr, uint8_t read_addr)
{
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

    // Read RangeStatus Register
    data_temp = VL53L0X_REG_RESULT_RANGE_STATUS;
    i2c_write(0, write_addr, &data_temp, sizeof(data_temp));
    i2c_read(0, read_addr, (uint8_t*)&localBuffer[0], 12);

    pRangingMeasurementData->ZoneId = 0; /* Only one zone */
    pRangingMeasurementData->TimeStamp = 0; /* Not Implemented */
    tmpuint16 = VL53L0X_MAKEUINT16(localBuffer[11], localBuffer[10]);
    pRangingMeasurementData->MeasurementTimeUsec = 0;
    SignalRate = VL53L0X_FIXPOINT97TOFIXPOINT1616(VL53L0X_MAKEUINT16(localBuffer[7], localBuffer[6])); /* peak_signal_count_rate_rtn_mcps */
    pRangingMeasurementData->SignalRateRtnMegaCps = SignalRate; 
    AmbientRate = VL53L0X_MAKEUINT16(localBuffer[9], localBuffer[8]);
    pRangingMeasurementData->AmbientRateRtnMegaCps = VL53L0X_FIXPOINT97TOFIXPOINT1616(AmbientRate);
    EffectiveSpadRtnCount = VL53L0X_MAKEUINT16(localBuffer[3], localBuffer[2]);
    pRangingMeasurementData->EffectiveSpadRtnCount = EffectiveSpadRtnCount; /* EffectiveSpadRtnCount is 8.8 format */
    DeviceRangeStatus = localBuffer[0];

    /* Get Linearity Corrective Gain */
    LinearityCorrectiveGain = PALDevDataGet(Dev, LinearityCorrectiveGain);
    /* Get ranging configuration */
    RangeFractionalEnable = PALDevDataGet(Dev, RangeFractionalEnable);

    // LINEARITY CORRECTIVE GAIN
    // if (LinearityCorrectiveGain != 1000) {
    //     tmpuint16 = (uint16_t)((LinearityCorrectiveGain * tmpuint16 + 500) / 1000);
    //     /* Implement Xtalk */
    //     VL53L0X_GETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps, XTalkCompensationRateMegaCps);
    //     VL53L0X_GETPARAMETERFIELD(Dev, XTalkCompensationEnable, XTalkCompensationEnable);
    //     if (XTalkCompensationEnable) {
    //         if ((SignalRate - ((XTalkCompensationRateMegaCps * EffectiveSpadRtnCount) >> 8)) <= 0) {
    //             if (RangeFractionalEnable)
    //                 XtalkRangeMilliMeter = 8888;
    //             else
    //                 XtalkRangeMilliMeter = 8888 << 2;
    //         } else {
    //             XtalkRangeMilliMeter = (tmpuint16 * SignalRate) / (SignalRate - ((XTalkCompensationRateMegaCps * EffectiveSpadRtnCount) >> 8));
    //         }
    //         tmpuint16 = XtalkRangeMilliMeter;
    //     }
    // }

    // RANGE FRACTIONAL
    // if (RangeFractionalEnable) {
    //     pRangingMeasurementData->RangeMilliMeter = (uint16_t)((tmpuint16) >> 2);
    //     pRangingMeasurementData->RangeFractionalPart = (uint8_t)((tmpuint16 & 0x03) << 6);
    // } else {
    //     pRangingMeasurementData->RangeMilliMeter = tmpuint16;
    //     pRangingMeasurementData->RangeFractionalPart = 0;
    // }

    /*
        * For a standard definition of RangeStatus, this should
        * return 0 in case of good result after a ranging
        * The range status depends on the device so call a device
        * specific function to obtain the right Status.
        */
    Status |= VL53L0X_get_pal_range_status(Dev, DeviceRangeStatus,
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

VL53L0X_Error VL53L0X_calc_sigma_estimate(VL53L0X_DEV Dev,
	VL53L0X_RangingMeasurementData_t *pRangingMeasurementData,
	FixPoint1616_t *pSigmaEstimate,
	uint32_t *pDmax_mm)
{
	/* Expressed in 100ths of a ns, i.e. centi-ns */
	const uint32_t cPulseEffectiveWidth_centi_ns   = 800;
	/* Expressed in 100ths of a ns, i.e. centi-ns */
	const uint32_t cAmbientEffectiveWidth_centi_ns = 600;
	const FixPoint1616_t cDfltFinalRangeIntegrationTimeMilliSecs	= 0x00190000; /* 25ms */
	const uint32_t cVcselPulseWidth_ps	= 4700; /* pico secs */
	const FixPoint1616_t cSigmaEstMax	= 0x028F87AE;
	const FixPoint1616_t cSigmaEstRtnMax	= 0xF000;
	const FixPoint1616_t cAmbToSignalRatioMax = 0xF0000000/
		cAmbientEffectiveWidth_centi_ns;
	/* Time Of Flight per mm (6.6 pico secs) */
	const FixPoint1616_t cTOF_per_mm_ps		= 0x0006999A;
	const uint32_t c16BitRoundingParam		= 0x00008000;
	const FixPoint1616_t cMaxXTalk_kcps		= 0x00320000;
	const uint32_t cPllPeriod_ps			= 1655;

	uint32_t vcselTotalEventsRtn;
	uint32_t finalRangeTimeoutMicroSecs;
	uint32_t preRangeTimeoutMicroSecs;
	uint32_t finalRangeIntegrationTimeMilliSecs;
	FixPoint1616_t sigmaEstimateP1;
	FixPoint1616_t sigmaEstimateP2;
	FixPoint1616_t sigmaEstimateP3;
	FixPoint1616_t deltaT_ps;
	FixPoint1616_t pwMult;
	FixPoint1616_t sigmaEstRtn;
	FixPoint1616_t sigmaEstimate;
	FixPoint1616_t xTalkCorrection;
	FixPoint1616_t ambientRate_kcps;
	FixPoint1616_t peakSignalRate_kcps;
	FixPoint1616_t xTalkCompRate_mcps;
	uint32_t xTalkCompRate_kcps;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	FixPoint1616_t diff1_mcps;
	FixPoint1616_t diff2_mcps;
	FixPoint1616_t sqr1;
	FixPoint1616_t sqr2;
	FixPoint1616_t sqrSum;
	FixPoint1616_t sqrtResult_centi_ns;
	FixPoint1616_t sqrtResult;
	FixPoint1616_t totalSignalRate_mcps;
	FixPoint1616_t correctedSignalRate_mcps;
	FixPoint1616_t sigmaEstRef;
	uint32_t vcselWidth;
	uint32_t finalRangeMacroPCLKS;
	uint32_t preRangeMacroPCLKS;
	uint32_t peakVcselDuration_us;
	uint8_t finalRangeVcselPCLKS;
	uint8_t preRangeVcselPCLKS;
	/*! \addtogroup calc_sigma_estimate
	 * @{
	 *
	 * Estimates the range sigma
	 */

	LOG_FUNCTION_START("");

	VL53L0X_GETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps,
			xTalkCompRate_mcps);

	/*
	 * We work in kcps rather than mcps as this helps keep within the
	 * confines of the 32 Fix1616 type.
	 */

	ambientRate_kcps =
		(pRangingMeasurementData->AmbientRateRtnMegaCps * 1000) >> 16;

	correctedSignalRate_mcps =
		pRangingMeasurementData->SignalRateRtnMegaCps;


	Status = VL53L0X_get_total_signal_rate(
		Dev, pRangingMeasurementData, &totalSignalRate_mcps);
	Status = VL53L0X_get_total_xtalk_rate(
		Dev, pRangingMeasurementData, &xTalkCompRate_mcps);


	/* Signal rate measurement provided by device is the
	 * peak signal rate, not average.
	 */
	peakSignalRate_kcps = (totalSignalRate_mcps * 1000);
	peakSignalRate_kcps = (peakSignalRate_kcps + 0x8000) >> 16;

	xTalkCompRate_kcps = xTalkCompRate_mcps * 1000;

	if (xTalkCompRate_kcps > cMaxXTalk_kcps)
		xTalkCompRate_kcps = cMaxXTalk_kcps;

	if (Status == VL53L0X_ERROR_NONE) {

		/* Calculate final range macro periods */
		finalRangeTimeoutMicroSecs = VL53L0X_GETDEVICESPECIFICPARAMETER(
			Dev, FinalRangeTimeoutMicroSecs);

		finalRangeVcselPCLKS = VL53L0X_GETDEVICESPECIFICPARAMETER(
			Dev, FinalRangeVcselPulsePeriod);

		finalRangeMacroPCLKS = VL53L0X_calc_timeout_mclks(
			Dev, finalRangeTimeoutMicroSecs, finalRangeVcselPCLKS);

		/* Calculate pre-range macro periods */
		preRangeTimeoutMicroSecs = VL53L0X_GETDEVICESPECIFICPARAMETER(
			Dev, PreRangeTimeoutMicroSecs);

		preRangeVcselPCLKS = VL53L0X_GETDEVICESPECIFICPARAMETER(
			Dev, PreRangeVcselPulsePeriod);

		preRangeMacroPCLKS = VL53L0X_calc_timeout_mclks(
			Dev, preRangeTimeoutMicroSecs, preRangeVcselPCLKS);

		vcselWidth = 3;
		if (finalRangeVcselPCLKS == 8)
			vcselWidth = 2;


		peakVcselDuration_us = vcselWidth * 2048 *
			(preRangeMacroPCLKS + finalRangeMacroPCLKS);
		peakVcselDuration_us = (peakVcselDuration_us + 500)/1000;
		peakVcselDuration_us *= cPllPeriod_ps;
		peakVcselDuration_us = (peakVcselDuration_us + 500)/1000;

		/* Fix1616 >> 8 = Fix2408 */
		totalSignalRate_mcps = (totalSignalRate_mcps + 0x80) >> 8;

		/* Fix2408 * uint32 = Fix2408 */
		vcselTotalEventsRtn = totalSignalRate_mcps *
			peakVcselDuration_us;

		/* Fix2408 >> 8 = uint32 */
		vcselTotalEventsRtn = (vcselTotalEventsRtn + 0x80) >> 8;

		/* Fix2408 << 8 = Fix1616 = */
		totalSignalRate_mcps <<= 8;
	}

	if (Status != VL53L0X_ERROR_NONE) {
		LOG_FUNCTION_END(Status);
		return Status;
	}

	if (peakSignalRate_kcps == 0) {
		*pSigmaEstimate = cSigmaEstMax;
		PALDevDataSet(Dev, SigmaEstimate, cSigmaEstMax);
		*pDmax_mm = 0;
	} else {
		if (vcselTotalEventsRtn < 1)
			vcselTotalEventsRtn = 1;

		sigmaEstimateP1 = cPulseEffectiveWidth_centi_ns;

		/* ((FixPoint1616 << 16)* uint32)/uint32 = FixPoint1616 */
		sigmaEstimateP2 = (ambientRate_kcps << 16)/peakSignalRate_kcps;
		if (sigmaEstimateP2 > cAmbToSignalRatioMax) {
			/* Clip to prevent overflow. Will ensure safe
			 * max result. */
			sigmaEstimateP2 = cAmbToSignalRatioMax;
		}
		sigmaEstimateP2 *= cAmbientEffectiveWidth_centi_ns;

		sigmaEstimateP3 = 2 * VL53L0X_isqrt(vcselTotalEventsRtn * 12);

		/* uint32 * FixPoint1616 = FixPoint1616 */
		deltaT_ps = pRangingMeasurementData->RangeMilliMeter *
					cTOF_per_mm_ps;

		/*
		 * vcselRate - xtalkCompRate
		 * (uint32 << 16) - FixPoint1616 = FixPoint1616.
		 * Divide result by 1000 to convert to mcps.
		 * 500 is added to ensure rounding when integer division
		 * truncates.
		 */
		diff1_mcps = (((peakSignalRate_kcps << 16) -
			2 * xTalkCompRate_kcps) + 500)/1000;

		/* vcselRate + xtalkCompRate */
		diff2_mcps = ((peakSignalRate_kcps << 16) + 500)/1000;

		/* Shift by 8 bits to increase resolution prior to the
		 * division */
		diff1_mcps <<= 8;

		/* FixPoint0824/FixPoint1616 = FixPoint2408 */
		xTalkCorrection	 = abs(diff1_mcps/diff2_mcps);

		/* FixPoint2408 << 8 = FixPoint1616 */
		xTalkCorrection <<= 8;

		if(pRangingMeasurementData->RangeStatus != 0){
			pwMult = 1 << 16;
		} else {
			/* FixPoint1616/uint32 = FixPoint1616 */
			pwMult = deltaT_ps/cVcselPulseWidth_ps; /* smaller than 1.0f */

			/*
			 * FixPoint1616 * FixPoint1616 = FixPoint3232, however both
			 * values are small enough such that32 bits will not be
			 * exceeded.
			 */
			pwMult *= ((1 << 16) - xTalkCorrection);

			/* (FixPoint3232 >> 16) = FixPoint1616 */
			pwMult =  (pwMult + c16BitRoundingParam) >> 16;

			/* FixPoint1616 + FixPoint1616 = FixPoint1616 */
			pwMult += (1 << 16);

			/*
			 * At this point the value will be 1.xx, therefore if we square
			 * the value this will exceed 32 bits. To address this perform
			 * a single shift to the right before the multiplication.
			 */
			pwMult >>= 1;
			/* FixPoint1715 * FixPoint1715 = FixPoint3430 */
			pwMult = pwMult * pwMult;

			/* (FixPoint3430 >> 14) = Fix1616 */
			pwMult >>= 14;
		}

		/* FixPoint1616 * uint32 = FixPoint1616 */
		sqr1 = pwMult * sigmaEstimateP1;

		/* (FixPoint1616 >> 16) = FixPoint3200 */
		sqr1 = (sqr1 + 0x8000) >> 16;

		/* FixPoint3200 * FixPoint3200 = FixPoint6400 */
		sqr1 *= sqr1;

		sqr2 = sigmaEstimateP2;

		/* (FixPoint1616 >> 16) = FixPoint3200 */
		sqr2 = (sqr2 + 0x8000) >> 16;

		/* FixPoint3200 * FixPoint3200 = FixPoint6400 */
		sqr2 *= sqr2;

		/* FixPoint64000 + FixPoint6400 = FixPoint6400 */
		sqrSum = sqr1 + sqr2;

		/* SQRT(FixPoin6400) = FixPoint3200 */
		sqrtResult_centi_ns = VL53L0X_isqrt(sqrSum);

		/* (FixPoint3200 << 16) = FixPoint1616 */
		sqrtResult_centi_ns <<= 16;

		/*
		 * Note that the Speed Of Light is expressed in um per 1E-10
		 * seconds (2997) Therefore to get mm/ns we have to divide by
		 * 10000
		 */
		sigmaEstRtn = (((sqrtResult_centi_ns+50)/100) /
				sigmaEstimateP3);
		sigmaEstRtn		 *= VL53L0X_SPEED_OF_LIGHT_IN_AIR;

		/* Add 5000 before dividing by 10000 to ensure rounding. */
		sigmaEstRtn		 += 5000;
		sigmaEstRtn		 /= 10000;

		if (sigmaEstRtn > cSigmaEstRtnMax) {
			/* Clip to prevent overflow. Will ensure safe
			 * max result. */
			sigmaEstRtn = cSigmaEstRtnMax;
		}
		finalRangeIntegrationTimeMilliSecs =
			(finalRangeTimeoutMicroSecs + preRangeTimeoutMicroSecs + 500)/1000;

		/* sigmaEstRef = 1mm * 25ms/final range integration time (inc pre-range)
		 * sqrt(FixPoint1616/int) = FixPoint2408)
		 */
		sigmaEstRef =
			VL53L0X_isqrt((cDfltFinalRangeIntegrationTimeMilliSecs +
				finalRangeIntegrationTimeMilliSecs/2)/
				finalRangeIntegrationTimeMilliSecs);

		/* FixPoint2408 << 8 = FixPoint1616 */
		sigmaEstRef <<= 8;
		sigmaEstRef = (sigmaEstRef + 500)/1000;

		/* FixPoint1616 * FixPoint1616 = FixPoint3232 */
		sqr1 = sigmaEstRtn * sigmaEstRtn;
		/* FixPoint1616 * FixPoint1616 = FixPoint3232 */
		sqr2 = sigmaEstRef * sigmaEstRef;

		/* sqrt(FixPoint3232) = FixPoint1616 */
		sqrtResult = VL53L0X_isqrt((sqr1 + sqr2));
		/*
		 * Note that the Shift by 4 bits increases resolution prior to
		 * the sqrt, therefore the result must be shifted by 2 bits to
		 * the right to revert back to the FixPoint1616 format.
		 */

		sigmaEstimate	 = 1000 * sqrtResult;

		if ((peakSignalRate_kcps < 1) || (vcselTotalEventsRtn < 1) ||
				(sigmaEstimate > cSigmaEstMax)) {
				sigmaEstimate = cSigmaEstMax;
		}

		*pSigmaEstimate = (uint32_t)(sigmaEstimate);
		PALDevDataSet(Dev, SigmaEstimate, *pSigmaEstimate);
		Status = VL53L0X_calc_dmax(
			Dev,
			totalSignalRate_mcps,
			correctedSignalRate_mcps,
			pwMult,
			sigmaEstimateP1,
			sigmaEstimateP2,
			peakVcselDuration_us,
			pDmax_mm);
	}

	LOG_FUNCTION_END(Status);
	return Status;
}

VL53L0X_Error VL53L0X_get_total_signal_rate(VL53L0X_DEV Dev,
	VL53L0X_RangingMeasurementData_t *pRangingMeasurementData,
	FixPoint1616_t *ptotal_signal_rate_mcps)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	FixPoint1616_t totalXtalkMegaCps;

	LOG_FUNCTION_START("");

	*ptotal_signal_rate_mcps =
		pRangingMeasurementData->SignalRateRtnMegaCps;

	Status = VL53L0X_get_total_xtalk_rate(
		Dev, pRangingMeasurementData, &totalXtalkMegaCps);

	if (Status == VL53L0X_ERROR_NONE)
		*ptotal_signal_rate_mcps += totalXtalkMegaCps;

	return Status;
}

VL53L0X_Error VL53L0X_get_total_xtalk_rate(VL53L0X_DEV Dev,
	VL53L0X_RangingMeasurementData_t *pRangingMeasurementData,
	FixPoint1616_t *ptotal_xtalk_rate_mcps)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	uint8_t xtalkCompEnable;
	FixPoint1616_t totalXtalkMegaCps;
	FixPoint1616_t xtalkPerSpadMegaCps;

	*ptotal_xtalk_rate_mcps = 0;

	Status = VL53L0X_GetXTalkCompensationEnable(Dev, &xtalkCompEnable);
	if (Status == VL53L0X_ERROR_NONE) {

		if (xtalkCompEnable) {

			VL53L0X_GETPARAMETERFIELD(
				Dev,
				XTalkCompensationRateMegaCps,
				xtalkPerSpadMegaCps);

			/* FixPoint1616 * FixPoint 8:8 = FixPoint0824 */
			totalXtalkMegaCps =
				pRangingMeasurementData->EffectiveSpadRtnCount *
				xtalkPerSpadMegaCps;

			/* FixPoint0824 >> 8 = FixPoint1616 */
			*ptotal_xtalk_rate_mcps =
				(totalXtalkMegaCps + 0x80) >> 8;
		}
	}

	return Status;
}

VL53L0X_Error VL53L0X_GetXTalkCompensationEnable(VL53L0X_DEV Dev,
	uint8_t *pXTalkCompensationEnable)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t Temp8;
	LOG_FUNCTION_START("");

	VL53L0X_GETPARAMETERFIELD(Dev, XTalkCompensationEnable, Temp8);
	*pXTalkCompensationEnable = Temp8;

	LOG_FUNCTION_END(Status);
	return Status;
}

uint32_t VL53L0X_calc_timeout_mclks(VL53L0X_DEV Dev,
		uint32_t timeout_period_us,
		uint8_t vcsel_period_pclks)
{
	uint32_t macro_period_ps;
	uint32_t macro_period_ns;
	uint32_t timeout_period_mclks = 0;

	macro_period_ps = VL53L0X_calc_macro_period_ps(Dev, vcsel_period_pclks);
	macro_period_ns = (macro_period_ps + 500) / 1000;

	timeout_period_mclks =
		(uint32_t) (((timeout_period_us * 1000)
		+ (macro_period_ns / 2)) / macro_period_ns);

    return timeout_period_mclks;
}

uint32_t VL53L0X_calc_macro_period_ps(VL53L0X_DEV Dev, uint8_t vcsel_period_pclks)
{
	uint64_t PLL_period_ps;
	uint32_t macro_period_vclks;
	uint32_t macro_period_ps;

	LOG_FUNCTION_START("");

	/* The above calculation will produce rounding errors,
	   therefore set fixed value
	*/
	PLL_period_ps = 1655;

	macro_period_vclks = 2304;
	macro_period_ps = (uint32_t)(macro_period_vclks
			* vcsel_period_pclks * PLL_period_ps);

	LOG_FUNCTION_END("");
	return macro_period_ps;
}

uint32_t VL53L0X_isqrt(uint32_t num)
{
	/*
	 * Implements an integer square root
	 *
	 * From: http://en.wikipedia.org/wiki/Methods_of_computing_square_roots
	 */

	uint32_t  res = 0;
	uint32_t  bit = 1 << 30;
	/* The second-to-top bit is set:
	 *	1 << 14 for 16-bits, 1 << 30 for 32 bits */

	 /* "bit" starts at the highest power of four <= the argument. */
	while (bit > num)
		bit >>= 2;


	while (bit != 0) {
		if (num >= res + bit) {
			num -= res + bit;
			res = (res >> 1) + bit;
		} else
			res >>= 1;

		bit >>= 2;
	}

	return res;
}

VL53L0X_Error VL53L0X_calc_dmax(
	VL53L0X_DEV Dev,
	FixPoint1616_t totalSignalRate_mcps,
	FixPoint1616_t totalCorrSignalRate_mcps,
	FixPoint1616_t pwMult,
	uint32_t sigmaEstimateP1,
	FixPoint1616_t sigmaEstimateP2,
	uint32_t peakVcselDuration_us,
	uint32_t *pdmax_mm)
{
	const uint32_t cSigmaLimit		= 18;
	const FixPoint1616_t cSignalLimit	= 0x4000; /* 0.25 */
	const FixPoint1616_t cSigmaEstRef	= 0x00000042; /* 0.001 */
	const uint32_t cAmbEffWidthSigmaEst_ns = 6;
	const uint32_t cAmbEffWidthDMax_ns	   = 7;
	uint32_t dmaxCalRange_mm;
	FixPoint1616_t dmaxCalSignalRateRtn_mcps;
	FixPoint1616_t minSignalNeeded;
	FixPoint1616_t minSignalNeeded_p1;
	FixPoint1616_t minSignalNeeded_p2;
	FixPoint1616_t minSignalNeeded_p3;
	FixPoint1616_t minSignalNeeded_p4;
	FixPoint1616_t sigmaLimitTmp;
	FixPoint1616_t sigmaEstSqTmp;
	FixPoint1616_t signalLimitTmp;
	FixPoint1616_t SignalAt0mm;
	FixPoint1616_t dmaxDark;
	FixPoint1616_t dmaxAmbient;
	FixPoint1616_t dmaxDarkTmp;
	FixPoint1616_t sigmaEstP2Tmp;
	uint32_t signalRateTemp_mcps;

	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	LOG_FUNCTION_START("");

	dmaxCalRange_mm =
		PALDevDataGet(Dev, DmaxCalRangeMilliMeter);

	dmaxCalSignalRateRtn_mcps =
		PALDevDataGet(Dev, DmaxCalSignalRateRtnMegaCps);

	/* uint32 * FixPoint1616 = FixPoint1616 */
	SignalAt0mm = dmaxCalRange_mm * dmaxCalSignalRateRtn_mcps;

	/* FixPoint1616 >> 8 = FixPoint2408 */
	SignalAt0mm = (SignalAt0mm + 0x80) >> 8;
	SignalAt0mm *= dmaxCalRange_mm;

	minSignalNeeded_p1 = 0;
	if (totalCorrSignalRate_mcps > 0) {

		/* Shift by 10 bits to increase resolution prior to the
		 * division */
		signalRateTemp_mcps = totalSignalRate_mcps << 10;

		/* Add rounding value prior to division */
		minSignalNeeded_p1 = signalRateTemp_mcps +
			(totalCorrSignalRate_mcps/2);

		/* FixPoint0626/FixPoint1616 = FixPoint2210 */
		minSignalNeeded_p1 /= totalCorrSignalRate_mcps;

		/* Apply a factored version of the speed of light.
		 Correction to be applied at the end */
		minSignalNeeded_p1 *= 3;

		/* FixPoint2210 * FixPoint2210 = FixPoint1220 */
		minSignalNeeded_p1 *= minSignalNeeded_p1;

		/* FixPoint1220 >> 16 = FixPoint2804 */
		minSignalNeeded_p1 = (minSignalNeeded_p1 + 0x8000) >> 16;
	}

	minSignalNeeded_p2 = pwMult * sigmaEstimateP1;

	/* FixPoint1616 >> 16 =	 uint32 */
	minSignalNeeded_p2 = (minSignalNeeded_p2 + 0x8000) >> 16;

	/* uint32 * uint32	=  uint32 */
	minSignalNeeded_p2 *= minSignalNeeded_p2;

	/* Check sigmaEstimateP2
	 * If this value is too high there is not enough signal rate
	 * to calculate dmax value so set a suitable value to ensure
	 * a very small dmax.
	 */
	sigmaEstP2Tmp = (sigmaEstimateP2 + 0x8000) >> 16;
	sigmaEstP2Tmp = (sigmaEstP2Tmp + cAmbEffWidthSigmaEst_ns/2)/
		cAmbEffWidthSigmaEst_ns;
	sigmaEstP2Tmp *= cAmbEffWidthDMax_ns;

	if (sigmaEstP2Tmp > 0xffff) {
		minSignalNeeded_p3 = 0xfff00000;
	} else {

		/* DMAX uses a different ambient width from sigma, so apply
		 * correction.
		 * Perform division before multiplication to prevent overflow.
		 */
		sigmaEstimateP2 = (sigmaEstimateP2 + cAmbEffWidthSigmaEst_ns/2)/
			cAmbEffWidthSigmaEst_ns;
		sigmaEstimateP2 *= cAmbEffWidthDMax_ns;

		/* FixPoint1616 >> 16 = uint32 */
		minSignalNeeded_p3 = (sigmaEstimateP2 + 0x8000) >> 16;

		minSignalNeeded_p3 *= minSignalNeeded_p3;

	}

	/* FixPoint1814 / uint32 = FixPoint1814 */
	sigmaLimitTmp = ((cSigmaLimit << 14) + 500) / 1000;

	/* FixPoint1814 * FixPoint1814 = FixPoint3628 := FixPoint0428 */
	sigmaLimitTmp *= sigmaLimitTmp;

	/* FixPoint1616 * FixPoint1616 = FixPoint3232 */
	sigmaEstSqTmp = cSigmaEstRef * cSigmaEstRef;

	/* FixPoint3232 >> 4 = FixPoint0428 */
	sigmaEstSqTmp = (sigmaEstSqTmp + 0x08) >> 4;

	/* FixPoint0428 - FixPoint0428	= FixPoint0428 */
	sigmaLimitTmp -=  sigmaEstSqTmp;

	/* uint32_t * FixPoint0428 = FixPoint0428 */
	minSignalNeeded_p4 = 4 * 12 * sigmaLimitTmp;

	/* FixPoint0428 >> 14 = FixPoint1814 */
	minSignalNeeded_p4 = (minSignalNeeded_p4 + 0x2000) >> 14;

	/* uint32 + uint32 = uint32 */
	minSignalNeeded = (minSignalNeeded_p2 + minSignalNeeded_p3);

	/* uint32 / uint32 = uint32 */
	minSignalNeeded += (peakVcselDuration_us/2);
	minSignalNeeded /= peakVcselDuration_us;

	/* uint32 << 14 = FixPoint1814 */
	minSignalNeeded <<= 14;

	/* FixPoint1814 / FixPoint1814 = uint32 */
	minSignalNeeded += (minSignalNeeded_p4/2);
	minSignalNeeded /= minSignalNeeded_p4;

	/* FixPoint3200 * FixPoint2804 := FixPoint2804*/
	minSignalNeeded *= minSignalNeeded_p1;

	/* Apply correction by dividing by 1000000.
	 * This assumes 10E16 on the numerator of the equation
	 * and 10E-22 on the denominator.
	 * We do this because 32bit fix point calculation can't
	 * handle the larger and smaller elements of this equation,
	 * i.e. speed of light and pulse widths.
	 */
	minSignalNeeded = (minSignalNeeded + 500) / 1000;
	minSignalNeeded <<= 4;

	minSignalNeeded = (minSignalNeeded + 500) / 1000;

	/* FixPoint1616 >> 8 = FixPoint2408 */
	signalLimitTmp = (cSignalLimit + 0x80) >> 8;

	/* FixPoint2408/FixPoint2408 = uint32 */
	if (signalLimitTmp != 0)
		dmaxDarkTmp = (SignalAt0mm + (signalLimitTmp / 2))
			/ signalLimitTmp;
	else
		dmaxDarkTmp = 0;

	dmaxDark = VL53L0X_isqrt(dmaxDarkTmp);

	/* FixPoint2408/FixPoint2408 = uint32 */
	if (minSignalNeeded != 0)
		dmaxAmbient = (SignalAt0mm + minSignalNeeded/2)
			/ minSignalNeeded;
	else
		dmaxAmbient = 0;

	dmaxAmbient = VL53L0X_isqrt(dmaxAmbient);

	*pdmax_mm = dmaxDark;
	if (dmaxDark > dmaxAmbient)
		*pdmax_mm = dmaxAmbient;

	LOG_FUNCTION_END(Status);

	return Status;
}