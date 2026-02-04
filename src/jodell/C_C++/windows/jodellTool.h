#pragma once

#include<iostream>
#include <stdio.h>
#include <tchar.h>
#include<Windows.h>


extern "C"	_declspec(dllexport) int* getComPort();
extern "C"	_declspec(dllexport) int serialOperation(int comIndex, int portNo, int baudRate, bool status);
extern "C"	_declspec(dllexport) int* getStatus(int comIndex, int salveId, int address, int readMode, int count);
extern "C"	_declspec(dllexport) int writeDataToRegister(int comIndex, int salveId, int address, uint16_t sendCmdBuf[], int count);
extern "C"	_declspec(dllexport) int clawEnable(int comIndex, int salveId, bool status);
extern "C"	_declspec(dllexport) int rotateEnable(int comIndex, int salveId, bool status);
extern "C"	_declspec(dllexport) int runWithoutParam(int comIndex, int salveId, int cmdId);
extern "C"	_declspec(dllexport) int rotateWithoutParam(int comIndex, int salveId, int cmdId);
extern "C"	_declspec(dllexport) int runWithParam(int comIndex, int salveId, int pos, int speed, int torque);
extern "C"	_declspec(dllexport) int runWithParamOfErg32(int comIndex, int salveId, int pos, int speed, int torque);
extern "C"	_declspec(dllexport) int rotateWithParam(int comIndex, int salveId, int angle, int speed, int torque, bool absStatus, int cycleNum);
extern "C"	_declspec(dllexport) int changeSlaveId(int comIndex, int oldId, int newId);
extern "C"	_declspec(dllexport) int changeSlaveIdOfErg32(int comIndex, int oldId, int newId);
extern "C"	_declspec(dllexport) int changeBaudRate(int comIndex, int salveId, int baudRate);
extern "C"	_declspec(dllexport) int changeBaudRateOfErg32(int comIndex, int salveId, int baudRate);
extern "C"	_declspec(dllexport) int clawEncoderZero(int comIndex, int salveId);
extern "C"	_declspec(dllexport) int switchMode(int comIndex, int salveId, int modeIndex);
extern "C"	_declspec(dllexport) int getClawCurrentStatus(int comIndex, int salveId);
extern "C"	_declspec(dllexport) int getClawCurrentStatusOfErg32(int comIndex, int salveId);
extern "C"	_declspec(dllexport) int getRotateCurrentStatusOfErg32(int comIndex, int salveId);
extern "C"	_declspec(dllexport) int getClawCurrentLocation(int comIndex, int salveId);
extern "C"	_declspec(dllexport) int getClawCurrentLocationOfErg32(int comIndex, int salveId);
extern "C"	_declspec(dllexport) int getCurrentAbsAngleOfErg32(int comIndex, int salveId);
extern "C"	_declspec(dllexport) int getCurrentRelAngleOfErg32(int comIndex, int salveId);
extern "C"	_declspec(dllexport) int getClawCurrentSpeed(int comIndex, int salveId);
extern "C"	_declspec(dllexport) int getClawCurrentSpeedOfErg32(int comIndex, int salveId);
extern "C"	_declspec(dllexport) int getRotateCurrentSpeedOfErg32(int comIndex, int salveId);
extern "C"	_declspec(dllexport) int getClawCurrentTorque(int comIndex, int salveId);
extern "C"	_declspec(dllexport) int getClawCurrentTorqueOfErg32(int comIndex, int salveId);
extern "C"	_declspec(dllexport) int getRotateCurrentTorqueOfErg32(int comIndex, int salveId);
extern "C"	_declspec(dllexport) int getClawCurrentTemperature(int comIndex, int salveId);
extern "C"	_declspec(dllexport) int getClawCurrentTemperatureOfErg32(int comIndex, int salveId);
extern "C"	_declspec(dllexport) int getClawCurrentVoltage(int comIndex, int salveId);
extern "C"	_declspec(dllexport) int getClawCurrentVoltageOfErg32(int comIndex, int salveId);
extern "C"	_declspec(dllexport) int* querySoftwareVersion(int comIndex, int salveId);
extern "C"	_declspec(dllexport) int* querySoftwareVersionOfErg32(int comIndex, int salveId);

extern "C"	_declspec(dllexport) int enableDeviceOfEvs(int comIndex, int salveId, int state);
extern "C"	_declspec(dllexport) int startOrStopDeviceOfEvs(int comIndex, int salveId, int mode, int runFlag, int breakState);
extern "C"	_declspec(dllexport) int startOrStopDeviceOfEvs2(int comIndex, int salveId, int mode, int runFlag, int channelIndex, int breakState);
extern "C"	_declspec(dllexport) int switchModeOfEvs(int comIndex, int salveId, int modeIndex);
extern "C"	_declspec(dllexport) int writeCustomParamOfEvs(int comIndex, int salveId, int maxPressure, int minPressure, int timeout);
extern "C"	_declspec(dllexport) int writeCustomParamOfEvs2(int comIndex, int salveId, int channelNo, int maxPressure, int minPressure, int timeout);
extern "C"	_declspec(dllexport) int getDeviceCurrentStateOfEvs(int comIndex, int salveId);
extern "C"	_declspec(dllexport) int getDeviceCurrentStateOfEvs2(int comIndex, int salveId, int channelNo);
extern "C"	_declspec(dllexport) int getDeviceCurrentVacuumDegreeOfEvs(int comIndex, int salveId);
extern "C"	_declspec(dllexport) int getDeviceCurrentVacuumDegreeOfEvs2(int comIndex, int salveId, int channelNo);
extern "C"	_declspec(dllexport) int getDeviceCurrentTemperatureOfEvs(int comIndex, int salveId);
extern "C"	_declspec(dllexport) int getDeviceCurrentVoltageOfEvs(int comIndex, int salveId);

extern "C"	_declspec(dllexport) int enableCollisionOfLepg(int comIndex, int salveId, bool state);
extern "C"	_declspec(dllexport) int setCollisionThresholdOfLepg_1(int comIndex, int salveId, int value);
extern "C"	_declspec(dllexport) int setCollisionThresholdOfLepg_2(int comIndex, int salveId, int value);
extern "C"	_declspec(dllexport) int setCollisionThresholdOfLepg_3(int comIndex, int salveId, int value);
extern "C"	_declspec(dllexport) int setDropThresholdOfLepg(int comIndex, int salveId, int value);
extern "C"	_declspec(dllexport) int searchRangeOfLepg(int comIndex, int salveId);
extern "C"	_declspec(dllexport) int manualSavePositionOfLepg(int comIndex, int salveId);
extern "C"	_declspec(dllexport) int setPreImpactOfLepg(int comIndex, int salveId, int value);
extern "C"	_declspec(dllexport) int setPreImpactSpeedOfLepg(int comIndex, int salveId, int value);

extern "C"	_declspec(dllexport) int restartDeviceOfEls(int comIndex, int salveId);
extern "C"	_declspec(dllexport) int saveParamOfEls(int comIndex, int salveId);
extern "C"	_declspec(dllexport) int restoreDefaultOfEls(int comIndex, int slaveId);
extern "C"	_declspec(dllexport) int chooseSignalSourceOfEls(int comIndex, int slaveId, int value);
extern "C"	_declspec(dllexport) int chooseUserControlModeOfEls(int comIndex, int slaveId, int value);
extern "C"	_declspec(dllexport) int enableDeviceOfEls(int comIndex, int slaveId, int value);
extern "C"	_declspec(dllexport) int resetDeviceOfEls(int comIndex, int slaveId);
extern "C"	_declspec(dllexport) int controlActionOfEls(int comIndex, int slaveId, int value);
extern "C"	_declspec(dllexport) int chooseMotorDirdctionOfEls(int comIndex, int slaveId, int value);
extern "C"	_declspec(dllexport) int setPointPositionOfEls(int comIndex, int slaveId, int pointNo, int value);
extern "C"	_declspec(dllexport) int readPointPositionOfEls(int comIndex, int slaveId, int pointNo);
extern "C"	_declspec(dllexport) int setPointSpeedOfEls(int comIndex, int slaveId, int pointNo, int value);
extern "C"	_declspec(dllexport) int readPointSpeedOfEls(int comIndex, int slaveId, int pointNo);
extern "C"	_declspec(dllexport) int setPointTorqueOfEls(int comIndex, int slaveId, int pointNo, int value);
extern "C"	_declspec(dllexport) int readPointTorqueOfEls(int comIndex, int slaveId, int pointNo);
extern "C"	_declspec(dllexport) int setRelativeMotionDistanceOfEls(int comIndex, int slaveId, int value);
extern "C"	_declspec(dllexport) int readRelativeMotionDistanceOfEls(int comIndex, int slaveId);
extern "C"	_declspec(dllexport) int setJogStepValueOfEls(int comIndex, int slaveId, int value);
extern "C"	_declspec(dllexport) int readJogStepValueOfEls(int comIndex, int slaveId);
extern "C"	_declspec(dllexport) int setDecelerationPercentageOfPositionOfEls(int comIndex, int slaveId, int value);
extern "C"	_declspec(dllexport) int readDecelerationPercentageOfPositionOfEls(int comIndex, int slaveId);
extern "C"	_declspec(dllexport) int setDecelerationPercentageOfSpeedOfEls(int comIndex, int slaveId, int value);
extern "C"	_declspec(dllexport) int readDecelerationPercentageOfSpeedOfEls(int comIndex, int slaveId);
extern "C"	_declspec(dllexport) int setDecelerationDirectionOfEls(int comIndex, int slaveId, int value);
extern "C"	_declspec(dllexport) int readDecelerationDirectionOfEls(int comIndex, int slaveId);
extern "C"	_declspec(dllexport) int setIoForcedOutputOfEls(int comIndex, int slaveId, int value);
extern "C"	_declspec(dllexport) int readIoForcedOutputOfEls(int comIndex, int slaveId);
extern "C"	_declspec(dllexport) int forcedControlBrakeOfEls(int comIndex, int slaveId, int value);
extern "C"	_declspec(dllexport) int changeSlaveIdOfEls(int comIndex, int slaveId, int value);
extern "C"	_declspec(dllexport) int changeBaudRateOfEls(int comIndex, int slaveId, int value);
extern "C"	_declspec(dllexport) int getDeviceRunStateOfEls(int comIndex, int slaveId);
extern "C"	_declspec(dllexport) int getDeviceActionStateOfEls(int comIndex, int slaveId);
extern "C"	_declspec(dllexport) int getDeciveCurrentLocationOfEls(int comIndex, int slaveId);
extern "C"	_declspec(dllexport) int getDeciveCurrentSpeedOfEls(int comIndex, int slaveId);
extern "C"	_declspec(dllexport) int getDeviceCurrentTorqueOfEls(int comIndex, int slaveId);
extern "C"	_declspec(dllexport) int getDeviceErrorStateOfEls(int comIndex, int slaveId);
extern "C"	_declspec(dllexport) int getMotorErrorStateOfEls(int comIndex, int slaveId);
extern "C"	_declspec(dllexport) int getIoInputMonitoringOfEls(int comIndex, int slaveId);
extern "C"	_declspec(dllexport) int getIoOutputMonitoringOfEls(int comIndex, int slaveId);

