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
#include "sensor/Controller.h"

using namespace dynamixel;

int main(int argc, char **argv) {
  if (argc != 4) {
    std::cerr << "Usage: " << argv[0] << " <device> <baudrate> <id>"
      << std::endl;
    return -1;
  }
  Controller controller(std::make_shared<SerialPort>(std::string(argv[1]),
    atoi(argv[2])));
  const auto id = atoi(argv[3]);
  std::cout << "Model number: " << controller.getModelNumber(id) << std::endl;
  controller.getModelInformation(controller.getModelNumber(id)).ostream(
    std::cout);
  std::cout << "Firmware version: "
    << (unsigned)controller.getFirmwareVersion(id) << std::endl;
  std::cout << "Device ID: " << (unsigned)controller.getId(id) << std::endl;
  std::cout << "Baud rate: " << (unsigned)controller.getBaudRate(id)
    << std::endl;
  std::cout << "Return delay time (us): " << controller.getReturnDelayTimeUs(id)
    << std::endl;
  std::cout << "Clockwise angle limit (rad): "
    << controller.getCwAngleLimitAngle(id) << std::endl;
  std::cout << "Counter-clockwise angle limit (rad): "
    << controller.getCcwAngleLimitAngle(id) << std::endl;
  std::cout << "Highest limit temperature (C): "
    << (unsigned)controller.getHighestLimitTemperature(id) << std::endl;
  std::cout << "Highest limit voltage (V): "
    << controller.getHighestLimitVoltageVolt(id) << std::endl;
  std::cout << "Lowest limit voltage (V): "
    << controller.getLowestLimitVoltageVolt(id) << std::endl;
  std::cout << "Maximum torque (%): " << controller.getMaxTorquePercent(id)
    << std::endl;
  std::cout << "Status return level: "
    << (unsigned)controller.getStatusReturnLevel(id) << std::endl;
  std::cout << "Alarm LED: " << (unsigned)controller.getAlarmLed(id)
    << std::endl;
  std::cout << "Alarm shutdown: " << (unsigned)controller.getAlarmShutdown(id)
    << std::endl;
  std::cout << "Multi-turn offset: " << controller.getMultiTurnOffset(id)
    << std::endl;
  std::cout << "Resolution divider: "
    << (unsigned)controller.getResolutionDivider(id) << std::endl;
  std::cout << "Torque enable: " << (unsigned)controller.isTorqueEnable(id)
    << std::endl;
  std::cout << "LED: " << (unsigned)controller.isLed(id) << std::endl;
  if (controller.isLed(id))
    controller.setLed(id, false, true);
  else
    controller.setLed(id, true, true);
  controller.action(id);
  std::cout << "D gain: " << controller.getDGainK(id) << std::endl;
  std::cout << "I gain: " << controller.getIGainK(id) << std::endl;
  std::cout << "P gain: " << controller.getPGainK(id) << std::endl;
  std::cout << "Goal position (rad): " << controller.getGoalPositionAngle(id)
    << std::endl;
  std::cout << "Moving speed (rad/s): "
    << Controller::revPerMin2RadPerSec(controller.getMovingSpeedRpm(id))
    << std::endl;
  std::cout << "Torque limit (%): " << controller.getTorqueLimitPercent(id)
    << std::endl;
  std::cout << "Present position (rad): "
    << controller.getPresentPositionAngle(id) << std::endl;
  std::cout << "Present speed (rad/s): "
    << Controller::revPerMin2RadPerSec(controller.getPresentSpeedRpm(id))
    << std::endl;
  std::cout << "Present load (%): " << controller.getPresentLoadPercent(id)
    << std::endl;
  std::cout << "Present voltage (V): " << controller.getPresentVoltageVolt(id)
    << std::endl;
  std::cout << "Present temperature (C): "
    << (unsigned)controller.getPresentTemperature(id) << std::endl;
  std::cout << "Instruction registered: "
    << (unsigned)controller.isInstructionRegistered(id) << std::endl;
  std::cout << "Moving: " << (unsigned)controller.isMoving(id) << std::endl;
  std::cout << "EEPROM lock: " << (unsigned)controller.isEEPROMLock(id)
    << std::endl;
  std::cout << "Punch: " << controller.getPunch(id) << std::endl;
  std::cout << "Current (A): " << controller.getCurrentAmp(id) << std::endl;
  std::cout << "Torque control mode enable: "
    << (unsigned)controller.isTorqueControlModeEnable(id) << std::endl;
  std::cout << "Goal torque (A): " << controller.getGoalTorqueAmp(id)
    << std::endl;
  std::cout << "Goal acceleration (rad/sec^2): "
    << controller.getGoalAccelerationRadSec2(id) << std::endl;
  controller.setMovingSpeed(1, 0);
  controller.setGoalPosition(1, 0);
  return 0;
}
