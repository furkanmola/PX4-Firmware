#include "bno055.hpp"
#include <string.h>

uint16_t accelScale = 100;
uint16_t tempScale = 1;
uint16_t angularRateScale = 16;
uint16_t eulerScale = 16;
uint16_t magScale = 16;
uint16_t quaScale = (1<<14);    // 2^14

BNO055::BNO055(device::Device *interface, enum Rotation rotation, I2CSPIBusOption bus_option, int bus):
        I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id()), bus_option, bus),
        _px4_mag(interface->get_device_id(), rotation),
        _interface(interface),
        _comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms_errors")),
	      _conf_errors(perf_alloc(PC_COUNT, MODULE_NAME": conf_errors")),
	      _range_errors(perf_alloc(PC_COUNT, MODULE_NAME": range_errors")),
	      _sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	      _mode(BNO055_OPERATION_MODE_MAGONLY),
	      _measure_interval(0),
	      _check_state_cnt(0)
{
  _px4_mag.set_external(_interface->external());
  _px4_mag.set_scale(1.0f/(100.0f*75.0f));
}

BNO055::~BNO055()
{
  // free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_range_errors);
	perf_free(_conf_errors);

	delete _interface;
}

bno055_self_test_result_t BNO055::bno055_getSelfTestResult() {
  _interface->write(BNO055_PAGE_ID, 0, 1);
  uint8_t tmp;
  bno055_self_test_result_t res = {
      .mcuState = 0, .gyrState = 0, .magState = 0, .accState = 0};
  _interface->read(BNO055_ST_RESULT, &tmp, 1);
  res.mcuState = (tmp >> 3) & 0x01;
  res.gyrState = (tmp >> 2) & 0x01;
  res.magState = (tmp >> 1) & 0x01;
  res.accState = (tmp >> 0) & 0x01;
  return res;
}

bno055_opmode_t BNO055::bno055_getOperationMode() {
  bno055_opmode_t mode;
  _interface->read(BNO055_OPR_MODE, &mode, 1);
  return mode;
}

void BNO055::bno055_setOperationMode(bno055_opmode_t mode) {
  _interface->write(BNO055_OPR_MODE, (uint8_t*)mode, 1);
  if (mode == BNO055_OPERATION_MODE_CONFIG) {
    //bno055_delay(19);
  } else {
    //bno055_delay(7);
  }
}

void BNO055::bno055_setOperationModeConfig() {
  bno055_setOperationMode(BNO055_OPERATION_MODE_CONFIG);
}

void BNO055::bno055_setOperationModeNDOF() {
  bno055_setOperationMode(BNO055_OPERATION_MODE_NDOF);
}

void BNO055::bno055_setExternalCrystalUse(bool state) {
  _interface->write(BNO055_PAGE_ID, 0, 1);
  uint8_t tmp = 0;
  _interface->read(BNO055_SYS_TRIGGER, &tmp, 1);
  tmp |= (state == true) ? 0x80 : 0x0;
  _interface->write(BNO055_SYS_TRIGGER, &tmp, 1);
  //bno055_delay(700);
}

void BNO055::bno055_enableExternalCrystal() { bno055_setExternalCrystalUse(true); }
void BNO055::bno055_disableExternalCrystal() { bno055_setExternalCrystalUse(false); }

void BNO055::bno055_reset() {
  _interface->write(BNO055_SYS_TRIGGER, (uint8_t *)0x20, 1);
  //bno055_delay(700);
}

int8_t BNO055::bno055_getTemp() {
  _interface->write(BNO055_PAGE_ID, 0, 1);
  uint8_t t;
  _interface->read(BNO055_TEMP, &t, 1);
  return t;
}

int BNO055::bno055_setup() {
  bno055_reset();

  uint8_t id = 0;
  _interface->read(BNO055_CHIP_ID, &id, 1);
  if (id != BNO055_ID) {
    printf("Can't find BNO055, id: 0x%02x. Please check your wiring.\r\n", id);
  }
  _interface->write(BNO055_PAGE_ID, 0, 1);
  _interface->write(BNO055_SYS_TRIGGER, 0x0, 1);

  // Select BNO055 config mode
  bno055_setOperationModeConfig();
  //bno055_delay(10);

  int ret = OK;

  return ret;
}

int16_t BNO055::bno055_getSWRevision() {
  _interface->write(BNO055_PAGE_ID, 0, 1);
  uint8_t buffer[2];
  _interface->read(BNO055_SW_REV_ID_LSB, buffer, 2);
  return (int16_t)((buffer[1] << 8) | buffer[0]);
}

uint8_t BNO055::bno055_getBootloaderRevision() {
  _interface->write(BNO055_PAGE_ID, 0, 1);
  uint8_t tmp;
  _interface->read(BNO055_BL_REV_ID, &tmp, 1);
  return tmp;
}

uint8_t BNO055::bno055_getSystemStatus() {
  _interface->write(BNO055_PAGE_ID, 0, 1);
  uint8_t tmp;
  _interface->read(BNO055_SYS_STATUS, &tmp, 1);
  return tmp;
}

uint8_t BNO055::bno055_getSystemError() {
  _interface->write(BNO055_PAGE_ID, 0, 1);
  uint8_t tmp;
  _interface->read(BNO055_SYS_ERR, &tmp, 1);
  return tmp;
}

bno055_calibration_state_t BNO055::bno055_getCalibrationState() {
  _interface->write(BNO055_PAGE_ID, 0, 1);
  bno055_calibration_state_t cal = {.sys = 0, .gyro = 0, .mag = 0, .accel = 0};
  uint8_t calState = 0;
  _interface->read(BNO055_CALIB_STAT, &calState, 1);
  cal.sys = (calState >> 6) & 0x03;
  cal.gyro = (calState >> 4) & 0x03;
  cal.accel = (calState >> 2) & 0x03;
  cal.mag = calState & 0x03;
  return cal;
}


bno055_calibration_data_t BNO055::bno055_getCalibrationData() {
  bno055_calibration_data_t calData;
  uint8_t buffer[22];
  bno055_opmode_t operationMode = bno055_getOperationMode();
  bno055_setOperationModeConfig();
  _interface->write(BNO055_PAGE_ID, 0, 1);

  _interface->read(BNO055_ACC_OFFSET_X_LSB, buffer, 22);

  // Assumes little endian processor
  memcpy(&calData.offset.accel, buffer, 6);
  memcpy(&calData.offset.mag, buffer + 6, 6);
  memcpy(&calData.offset.gyro, buffer + 12, 6);
  memcpy(&calData.radius.accel, buffer + 18, 2);
  memcpy(&calData.radius.mag, buffer + 20, 2);

  bno055_setOperationMode(operationMode);

  return calData;
}

void BNO055::bno055_setCalibrationData(bno055_calibration_data_t calData) {
  uint8_t buffer[22];
  bno055_opmode_t operationMode = bno055_getOperationMode();
  bno055_setOperationModeConfig();
  _interface->write(BNO055_PAGE_ID, 0, 1);

  // Assumes litle endian processor
  memcpy(buffer, &calData.offset.accel, 6);
  memcpy(buffer + 6, &calData.offset.mag, 6);
  memcpy(buffer + 12, &calData.offset.gyro, 6);
  memcpy(buffer + 18, &calData.radius.accel, 2);
  memcpy(buffer + 20, &calData.radius.mag, 2);

  for (uint8_t i=0; i < 22; i++) {
    // TODO(oliv4945): create multibytes write
    _interface->write(BNO055_ACC_OFFSET_X_LSB+i, &buffer[i], sizeof(buffer));
  }

  bno055_setOperationMode(operationMode);
}

bno055_vector_t BNO055::bno055_getVector(uint8_t vec) {
  _interface->write(BNO055_PAGE_ID, 0, 1);
  uint8_t buffer[8];    // Quaternion need 8 bytes

  if (vec == BNO055_VECTOR_QUATERNION)
    _interface->read(vec, buffer, 8);
  else
    _interface->read(vec, buffer, 6);

  double scale = 1;

  if (vec == BNO055_VECTOR_MAGNETOMETER) {
    scale = magScale;
  } else if (vec == BNO055_VECTOR_ACCELEROMETER ||
           vec == BNO055_VECTOR_LINEARACCEL || vec == BNO055_VECTOR_GRAVITY) {
    scale = accelScale;
  } else if (vec == BNO055_VECTOR_GYROSCOPE) {
    scale = angularRateScale;
  } else if (vec == BNO055_VECTOR_EULER) {
    scale = eulerScale;
  } else if (vec == BNO055_VECTOR_QUATERNION) {
    scale = quaScale;
  }

  bno055_vector_t xyz = {.w = 0, .x = 0, .y = 0, .z = 0};
  if (vec == BNO055_VECTOR_QUATERNION) {
    xyz.w = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
    xyz.x = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
    xyz.y = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;
    xyz.z = (int16_t)((buffer[7] << 8) | buffer[6]) / scale;
  } else {
    xyz.x = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
    xyz.y = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
    xyz.z = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;
  }

  return xyz;
}

bno055_vector_t BNO055::bno055_getVectorAccelerometer() {
  return bno055_getVector(BNO055_VECTOR_ACCELEROMETER);
}
bno055_vector_t BNO055::bno055_getVectorMagnetometer() {
  return bno055_getVector(BNO055_VECTOR_MAGNETOMETER);
}
bno055_vector_t BNO055::bno055_getVectorGyroscope() {
  return bno055_getVector(BNO055_VECTOR_GYROSCOPE);
}
bno055_vector_t BNO055::bno055_getVectorEuler() {
  return bno055_getVector(BNO055_VECTOR_EULER);
}
bno055_vector_t BNO055::bno055_getVectorLinearAccel() {
  return bno055_getVector(BNO055_VECTOR_LINEARACCEL);
}
bno055_vector_t BNO055::bno055_getVectorGravity() {
  return bno055_getVector(BNO055_VECTOR_GRAVITY);
}
bno055_vector_t BNO055::bno055_getVectorQuaternion() {
  return bno055_getVector(BNO055_VECTOR_QUATERNION);
}

void BNO055::bno055_setAxisMap(bno055_axis_map_t axis) {
  uint8_t axisRemap = (axis.z << 4) | (axis.y << 2) | (axis.x);
  uint8_t axisMapSign = (axis.x_sign << 2) | (axis.y_sign << 1) | (axis.z_sign);
  _interface->write(BNO055_AXIS_MAP_CONFIG, &axisRemap, 1);
  _interface->write(BNO055_AXIS_MAP_SIGN, &axisMapSign, 1);
}

int BNO055::init()
{
  bno055_setup();
  int ret = OK;
  return ret;
}

void BNO055::RunImpl()
{
	if (_measure_interval > 0) {
		/* schedule a fresh cycle call when the measurement is done */
		ScheduleDelayed(_measure_interval);
	}
}

void BNO055::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	PX4_INFO("poll interval:  %u", _measure_interval);
}
