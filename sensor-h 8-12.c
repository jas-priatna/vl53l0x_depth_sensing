/*************************************************************************//**
* \file sensor_vl53l0x.h
*
* \brief VL53L0X functions header
*
* \details This file holds the definitions for the sensor commands
*
* \author Original by: (JP) Jasmine Priatna
*
* \date 6/1/2021 (JP) Original creation
*
* \copyright Copyright (c) 2020 by METco. All rights reserved.
*****************************************************************************/
#ifndef VL53L0X_H_
#define VL53L0X_H_

#include "hal_i2c.h"
/****************************************************************************
 * Addresses
*****************************************************************************/
//#define I2C_ADDR_VL53L0X_WRITE 0x52 // Device address
//#define I2C_ADDR_VL53L0X_READ 0x53

// Alternate Device Addresses
#define I2C_ADDR_VL53L0X_WRITE_0 0x2C 
#define I2C_ADDR_VL53L0X_READ_0 0x2D
#define I2C_ADDR_VL53L0X_WRITE_1 0x44 
#define I2C_ADDR_VL53L0X_READ_1 0x45
#define VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS    0x8A

// Change Time Budget
#define SYSTEM_SEQUENCE_CONFIG          0x01
#define MSRC_CONFIG_TIMEOUT_MACROP      0x46
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI      0x51
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI    0x71
#define PRE_RANGE_CONFIG_VCSEL_PERIOD           0x50
#define FINAL_RANGE_CONFIG_VCSEL_PERIOD         0x70

// Change Limit
#define VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT   0x0044
#define VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE           0
#define VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE     1
#define VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS            6

// Distance Measurement
#define VL53L0X_REG_RESULT_RANGE_STATUS         0x14
#define VL53L0X_REG_DISTANCE            0x1E

#define VL53L0X_REG_SYSRANGE_START                        0x000
#define VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK    0x02

/*****************************************************************************
 * GLOBAL typedefs
 *****************************************************************************/
 /** use where fractional values are expected
 *
 * Given a floating point value f it's .16 bit point is (int)(f*(1<<16))*/
typedef unsigned int FixPoint1616_t;
typedef enum vcselPeriodType{ VcselPeriodPreRange, VcselPeriodFinalRange } vcselPeriodType;
/*****************************************************************************
 * GLOBAL typedefs structs
 *****************************************************************************/
typedef struct
  {
    bool tcc, msrc, dss, pre_range, final_range;
  } SequenceStepEnables;

typedef struct
  {
    uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

    uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
    uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
  } SequenceStepTimeouts;
   
/** 
 * @struct VL53L0X_DeviceParameters_t
 *
 * @brief Defines all parameters for the device
 */
typedef struct {
////  VL53L0X_DeviceModes DeviceMode; // change
////    /*!< Defines type of measurement to be done for the next measure */
////  //VL53L0X_HistogramModes HistogramMode;
////    /*!< Defines type of histogram measurement to be done for the next
////     *	measure */
////  uint32_t MeasurementTimingBudgetMicroSeconds;
////    /*!< Defines the allowed total time for a single measurement */
////  uint32_t InterMeasurementPeriodMilliSeconds;
////    /*!< Defines time between two consecutive measurements (between two
////     *	measurement starts). If set to 0 means back-to-back mode */
////  uint8_t XTalkCompensationEnable;
////    /*!< Tells if Crosstalk compensation shall be enable or not	 */
////  uint16_t XTalkCompensationRangeMilliMeter;
////    /*!< CrossTalk compensation range in millimeter	 */
////  FixPoint1616_t XTalkCompensationRateMegaCps;
////    /*!< CrossTalk compensation rate in Mega counts per seconds.
////     *	Expressed in 16.16 fixed point format.	*/
//  int32_t RangeOffsetMicroMeters;
//    /*!< Range offset adjustment (mm).	*/
  uint8_t LimitChecksEnable[VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS];
    /*!< This Array store all the Limit Check enable for this device. */
//  uint8_t LimitChecksStatus[VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS];
//    /*!< This Array store all the Status of the check linked to last
//    * measurement. */
  FixPoint1616_t LimitChecksValue[VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS];
    /*!< This Array store all the Limit Check value for this device */
//  uint8_t WrapAroundCheckEnable;
//    /*!< Tells if Wrap Around Check shall be enable or not */
} VL53L0X_DeviceParameters_t;

/**
 * @struct VL53L0X_RangingMeasurementData_t
 *
 * @brief Range measurement data.
 */
typedef struct {
  uint32_t TimeStamp;		/*!< 32-bit time stamp. */
  uint32_t MeasurementTimeUsec;
    /*!< Give the Measurement time needed by the device to do the
     * measurement.*/
  uint16_t RangeMilliMeter;	/*!< range distance in millimeter. */
  uint16_t RangeDMaxMilliMeter;
    /*!< Tells what is the maximum detection distance of the device
     * in current setup and environment conditions (Filled when
     *	applicable) */
//  FixPoint1616_t SignalRateRtnMegaCps;
//    /*!< Return signal rate (MCPS)\n these is a 16.16 fix point
//     *	value, which is effectively a measure of target
//     *	 reflectance.*/
//  FixPoint1616_t AmbientRateRtnMegaCps;
//    /*!< Return ambient rate (MCPS)\n these is a 16.16 fix point
//     *	value, which is effectively a measure of the ambien
//     *	t light.*/
//  uint16_t EffectiveSpadRtnCount;
//    /*!< Return the effective SPAD count for the return signal.
//     *	To obtain Real value it should be divided by 256 */
//  uint8_t ZoneId;
//    /*!< Denotes which zone and range scheduler stage the range
//     *	data relates to. */
  uint8_t RangeFractionalPart;
    /*!< Fractional part of range distance. Final value is a
     *	FixPoint168 value. */
  uint8_t RangeStatus;
    /*!< Range Status for the current measurement. This is device
     *	dependent. Value = 0 means value is valid.
     *	See \ref RangeStatusPage */
}VL53L0X_RangingMeasurementData_t;

/** 
 * @struct VL53L0X_DeviceSpecificParameters_t
 *
 * @brief Defines all specific parameters for the device
 */
//typedef struct {
//  FixPoint1616_t OscFrequencyMHz; /* Frequency used */
//  uint16_t LastEncodedTimeout;
//    /* last encoded Time out used for timing budget*/
//  //VL53L0X_GpioFunctionality Pin0GpioFunctionality;
//    /* store the functionality of the GPIO: pin0 */
//  uint32_t FinalRangeTimeoutMicroSecs;
//   /*!< Execution time of the final range*/
//  uint8_t FinalRangeVcselPulsePeriod;
//   /*!< Vcsel pulse period (pll clocks) for the final range measurement*/
//  uint32_t PreRangeTimeoutMicroSecs;
//   /*!< Execution time of the final range*/
//  uint8_t PreRangeVcselPulsePeriod;
//   /*!< Vcsel pulse period (pll clocks) for the pre-range measurement*/
//  uint16_t SigmaEstRefArray;
//   /*!< Reference array sigma value in 1/100th of [mm] e.g. 100 = 1mm */
//  uint16_t SigmaEstEffPulseWidth;
//   /*!< Effective Pulse width for sigma estimate in 1/100th
//    * of ns e.g. 900 = 9.0ns */
//  uint16_t SigmaEstEffAmbWidth;
//   /*!< Effective Ambient width for sigma estimate in 1/100th of ns
//    * e.g. 500 = 5.0ns */
//  uint8_t ReadDataFromDeviceDone; /* Indicate if read from device has
//  been done (==1) or not (==0) */
//  uint8_t ModuleId; /* Module ID */
//  uint8_t Revision; /* test Revision */
//  //char ProductId[VL53L0X_MAX_STRING_LENGTH];
//          /* Product Identifier String  */
//  uint8_t ReferenceSpadCount; /* used for ref spad management */
//  uint8_t ReferenceSpadType;	/* used for ref spad management */
//  uint8_t RefSpadsInitialised; /* reports if ref spads are initialised. */
//  uint32_t PartUIDUpper; /*!< Unique Part ID Upper */
//  uint32_t PartUIDLower; /*!< Unique Part ID Lower */
//  FixPoint1616_t SignalRateMeasFixed400mm; /*!< Peek Signal rate
//  at 400 mm*/
  
//} VL53L0X_DeviceSpecificParameters_t;

/**
 * @struct VL53L0X_DevData_t
 *
 * @brief VL53L0X PAL device ST private data structure \n
 * End user should never access any of these field directly
 *
 * These must never access directly but only via macro
 */
typedef struct {
  //VL53L0X_DMaxData_t DMaxData;
    /*!< Dmax Data */
//  int32_t	 Part2PartOffsetNVMMicroMeter;
//    /*!< backed up NVM value */
//  int32_t	 Part2PartOffsetAdjustmentNVMMicroMeter;
//    /*!< backed up NVM value representing additional offset adjustment */ 
  VL53L0X_DeviceParameters_t CurrentParameters;
    /*!< Current Device Parameter */
  VL53L0X_RangingMeasurementData_t LastRangeMeasure;
    /*!< Ranging Data */
  //VL53L0X_HistogramMeasurementData_t LastHistogramMeasure;
    /*!< Histogram Data */
//  VL53L0X_DeviceSpecificParameters_t DeviceSpecificParameters;
//    /*!< Parameters specific to the device */
//  //VL53L0X_SpadData_t SpadData;
//    /*!< Spad Data */
//  uint8_t SequenceConfig;
//    /*!< Internal value for the sequence config */
  uint8_t RangeFractionalEnable;
    /*!< Enable/Disable fractional part of ranging data */
//  VL53L0X_State PalState;
//    /*!< Current state of the PAL for this device */
  //VL53L0X_PowerModes PowerMode;
    /*!< Current Power Mode	 */
//  uint16_t SigmaEstRefArray;
//    /*!< Reference array sigma value in 1/100th of [mm] e.g. 100 = 1mm */
//  uint16_t SigmaEstEffPulseWidth;
//    /*!< Effective Pulse width for sigma estimate in 1/100th
//    * of ns e.g. 900 = 9.0ns */
//  uint16_t SigmaEstEffAmbWidth;
//    /*!< Effective Ambient width for sigma estimate in 1/100th of ns
//    * e.g. 500 = 5.0ns */
  uint8_t StopVariable;
    /*!< StopVariable used during the stop sequence */
//  uint16_t targetRefRate;
//    /*!< Target Ambient Rate for Ref spad management */
//  FixPoint1616_t SigmaEstimate;
//    /*!< Sigma Estimate - based on ambient & VCSEL rates and
//    * signal_total_events */
//  FixPoint1616_t SignalEstimate;
//    /*!< Signal Estimate - based on ambient & VCSEL rates and cross talk */
//  FixPoint1616_t LastSignalRefMcps;
//    /*!< Latest Signal ref in Mcps */
//  uint8_t *pTuningSettingsPointer;
//    /*!< Pointer for Tuning Settings table */
//  uint8_t UseInternalTuningSettings;
//    /*!< Indicate if we use	 Tuning Settings table */
//  uint16_t LinearityCorrectiveGain;
//    /*!< Linearity Corrective Gain value in x1000 */
//  uint16_t DmaxCalRangeMilliMeter;
//    /*!< Dmax Calibration Range millimeter */
//  FixPoint1616_t DmaxCalSignalRateRtnMegaCps;
//    /*!< Dmax Calibration Signal Rate Return MegaCps */
}VL53L0X_DevData_t;

/*****************************************************************************
 * VL53L0X FUNCTIONS
 *****************************************************************************/
#define PALDevDataSet(Dev, field, data) (Dev->field)=(data)
#define PALDevDataGet(Dev, field) (Dev->field)

#define VL53L0X_MAKEUINT16(lsb, msb) (uint16_t)((((uint16_t)msb)<<8) + \
		(uint16_t)lsb) 
#define VL53L0X_FIXPOINT1616TOFIXPOINT97(Value) \
	(uint16_t)((Value>>9)&0xFFFF)

#define VL53L0X_SETARRAYPARAMETERFIELD(Dev, field, index, value) \
	PALDevDataSet(Dev, CurrentParameters.field[index], value)
#define VL53L0X_GETARRAYPARAMETERFIELD(Dev, field, index, variable) \
	variable = PALDevDataGet(Dev, CurrentParameters).field[index]


#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

/**********************************
GLOBAL FUNCTION PROTOTYPES
**********************************/
void init_vl53l0x_gpio(void);
void init_vl53l0x(void);
//void data_init(VL53L0X_DevData_t *dev);
//void static_init(VL53L0X_DevData_t *dev);
void get_distance_vl53l0x(void);
I2C_return_status get_ranging_measurement_data(VL53L0X_DevData_t *dev, VL53L0X_RangingMeasurementData_t *pRangingMeasurementData);
int16_t get_hole_depth();

#endif