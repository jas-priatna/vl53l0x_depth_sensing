#define VL53L0X_SETARRAYPARAMETERFIELD(Dev, field, index, value) \
	PALDevDataSet(Dev, CurrentParameters.field[index], value)

#define VL53L0X_GETARRAYPARAMETERFIELD(Dev, field, index, variable) \
	variable = PALDevDataGet(Dev, CurrentParameters).field[index]

#define VL53L0X_FIXPOINT97TOFIXPOINT1616(Value) \
	(FixPoint1616_t)(Value<<9)

#define VL53L0X_GETPARAMETERFIELD(Dev, field, variable) \
	variable = PALDevDataGet(Dev, CurrentParameters).field
    
#define VL53L0X_GETDEVICESPECIFICPARAMETER(Dev, field) \
	PALDevDataGet(Dev, DeviceSpecificParameters).field

# NOT NEEDED?
    VL53L0X_Error VL53L0X_GetLimitCheckEnable(VL53L0X_DEV Dev, uint16_t LimitCheckId,
	uint8_t *pLimitCheckEnable)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t Temp8;

	LOG_FUNCTION_START("");

	if (LimitCheckId >= VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS) {
		Status = VL53L0X_ERROR_INVALID_PARAMS;
		*pLimitCheckEnable = 0;
	} else {
		VL53L0X_GETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
			LimitCheckId, Temp8);
		*pLimitCheckEnable = Temp8;
	}

	LOG_FUNCTION_END(Status);
	return Status;
}

VL53L0X_Error VL53L0X_GetLimitCheckValue(VL53L0X_DEV Dev, uint16_t LimitCheckId,
	FixPoint1616_t *pLimitCheckValue)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t EnableZeroValue = 0;
	uint16_t Temp16;
	FixPoint1616_t TempFix1616;

	LOG_FUNCTION_START("");

	switch (LimitCheckId) {

	case VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE:
		/* internal computation: */
		VL53L0X_GETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
			VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, TempFix1616);
		EnableZeroValue = 0;
		break;

	case VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
		Status = VL53L0X_RdWord(Dev,
		VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
			&Temp16);
		if (Status == VL53L0X_ERROR_NONE)
			TempFix1616 = VL53L0X_FIXPOINT97TOFIXPOINT1616(Temp16);


		EnableZeroValue = 1;
		break;
	}

	if (Status == VL53L0X_ERROR_NONE) {

		if (EnableZeroValue == 1) {

			if (TempFix1616 == 0) {
				/* disabled: return value from memory */
				VL53L0X_GETARRAYPARAMETERFIELD(Dev,
					LimitChecksValue, LimitCheckId,
					TempFix1616);
				*pLimitCheckValue = TempFix1616;
				VL53L0X_SETARRAYPARAMETERFIELD(Dev,
					LimitChecksEnable, LimitCheckId, 0);
			} else {
				*pLimitCheckValue = TempFix1616;
				VL53L0X_SETARRAYPARAMETERFIELD(Dev,
					LimitChecksValue, LimitCheckId,
					TempFix1616);
				VL53L0X_SETARRAYPARAMETERFIELD(Dev,
					LimitChecksEnable, LimitCheckId, 1);
			}
		} else {
			*pLimitCheckValue = TempFix1616;
		}
	}

	LOG_FUNCTION_END(Status);
	return Status;

}