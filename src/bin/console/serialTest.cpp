/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

/** \file serialTest.cpp
    \brief This file is a testing binary for serial port.
  */

#include <iostream>
#include <string>
#include <cstdlib>
#include <memory>

#include "com/SerialPort.h"
#include "sensor/Dynamixel.h"

using namespace dynamixel;

int main(int argc, char **argv) {
  if (argc != 4) {
    std::cerr << "Usage: " << argv[0] << " <device> <baudrate> <id>"
      << std::endl;
    return -1;
  }
  Dynamixel device(std::make_shared<SerialPort>(std::string(argv[1]),
    atoi(argv[2])));
  const auto id = atoi(argv[3]);
  std::cout << "Model number: " << device.getModelNumber(id) << std::endl;
  std::cout << "Firmware version: " << (unsigned)device.getFirmwareVersion(id)
    << std::endl;
  std::cout << "Device ID: " << (unsigned)device.getId(id) << std::endl;
  std::cout << "Baud rate: " << (unsigned)device.getBaudRate(id) << std::endl;
  std::cout << "Return delay time (us): " << device.getReturnDelayTimeUs(id)
    << std::endl;
  std::cout << "Clockwise angle limit: " << device.getCwAngleLimit(id)
    << std::endl;
  std::cout << "Counter-clockwise angle limit: "
    << device.getCcwAngleLimit(id) << std::endl;
  std::cout << "Highest limit temperature (C): "
    << (unsigned)device.getHighestLimitTemperature(id) << std::endl;
  std::cout << "Highest limit voltage (V): "
    << device.getHighestLimitVoltageVolt(id) << std::endl;
  std::cout << "Lowest limit voltage (V): "
    << device.getLowestLimitVoltageVolt(id) << std::endl;
  std::cout << "Maximum torque (%): " << device.getMaxTorquePercent(id)
    << std::endl;
  std::cout << "Status return level: "
    << (unsigned)device.getStatusReturnLevel(id) << std::endl;
  std::cout << "Alarm LED: " << (unsigned)device.getAlarmLed(id) << std::endl;
  std::cout << "Alarm shutdown: " << (unsigned)device.getAlarmShutdown(id)
    << std::endl;
  std::cout << "Multi-turn offset: " << device.getMultiTurnOffset(id)
    << std::endl;
  std::cout << "Resolution divider: "
    << (unsigned)device.getResolutionDivider(id) << std::endl;
  std::cout << "Torque enable: " << (unsigned)device.isTorqueEnable(id)
    << std::endl;
  std::cout << "LED: " << (unsigned)device.isLed(id) << std::endl;
  if (device.isLed(id))
    device.setLed(id, false, true);
  else
    device.setLed(id, true, true);
  device.action(id);
  std::cout << "D gain: " << device.getDGainK(id) << std::endl;
  std::cout << "I gain: " << device.getIGainK(id) << std::endl;
  std::cout << "P gain: " << device.getPGainK(id) << std::endl;
  std::cout << "Goal position (deg): " << device.getGoalPositionDeg(id)
    << std::endl;
  std::cout << "Moving speed (rpm): " << device.getMovingSpeedRpm(id)
    << std::endl;
  std::cout << "Torque limit (%): " << device.getTorqueLimitPercent(id)
    << std::endl;
  std::cout << "Present position (deg): " << device.getPresentPositionDeg(id)
    << std::endl;
  std::cout << "Present speed (rpm): " << device.getPresentSpeedRpm(id)
    << std::endl;
  std::cout << "Present load (%): " << device.getPresentLoadPercent(id)
    << std::endl;
  std::cout << "Present voltage (V): " << device.getPresentVoltageVolt(id)
    << std::endl;
  std::cout << "Present temperature (C): "
    << (unsigned)device.getPresentTemperature(id) << std::endl;
  std::cout << "Instruction registered: "
    << (unsigned)device.isInstructionRegistered(id) << std::endl;
  std::cout << "Moving: " << (unsigned)device.isMoving(id) << std::endl;
  std::cout << "EEPROM lock: " << (unsigned)device.isEEPROMLock(id)
    << std::endl;
  std::cout << "Punch: " << device.getPunch(id) << std::endl;
  std::cout << "Current (A): " << device.getCurrentAmp(id) << std::endl;
  std::cout << "Torque control mode enable: "
    << (unsigned)device.isTorqueControlModeEnable(id) << std::endl;
  std::cout << "Goal torque (A): " << device.getGoalTorqueAmp(id) << std::endl;
  std::cout << "Goal acceleration (deg/sec^2): "
    << device.getGoalAccelerationDegSec2(id) << std::endl;
  device.setMovingSpeed(1, 0);
  device.setGoalPosition(1, 0);
  return 0;
}
