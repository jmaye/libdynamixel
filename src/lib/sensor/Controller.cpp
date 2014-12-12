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

#include "sensor/Controller.h"

#include <bitset>
#include <sstream>

#include "com/SerialPort.h"
#include "sensor/Packet.h"
#include "sensor/Instructions.h"
#include "sensor/Addresses.h"
#include "exceptions/IOException.h"
#include "exceptions/BadArgumentException.h"

namespace dynamixel {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  Controller::Controller(const std::shared_ptr<SerialPort>& serialPort) :
      serialPort_(serialPort) {
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void Controller::writePacket(const std::shared_ptr<Packet>& packet) {
    *this << *packet;
  }

  std::shared_ptr<Packet> Controller::readPacket() {
    auto packet = std::make_shared<Packet>();
    *this >> *packet;
    return packet;
  }

  bool Controller::ping(uint8_t id) {
    auto packet = std::make_shared<Packet>();
    packet->setId(id);
    packet->setLength(2);
    packet->setInstructionOrError(Instructions::PING);
    packet->setChecksum(packet->computeChecksum());
    try {
      writePacket(packet);
      auto status = readPacket();
      if (status->getInstructionOrError())
        return false;
      else
        return true;
    }
    catch (const IOException& e) {
      return false;
    }
  }

  bool Controller::reset(uint8_t id) {
    auto packet = std::make_shared<Packet>();
    packet->setId(id);
    packet->setLength(2);
    packet->setInstructionOrError(Instructions::RESET);
    packet->setChecksum(packet->computeChecksum());
    try {
      writePacket(packet);
      auto status = readPacket();
      if (status->getInstructionOrError())
        return false;
      else
        return true;
    }
    catch (const IOException& e) {
      return false;
    }
  }

  bool Controller::action(uint8_t id) {
    auto packet = std::make_shared<Packet>();
    packet->setId(id);
    packet->setLength(2);
    packet->setInstructionOrError(Instructions::ACTION);
    packet->setChecksum(packet->computeChecksum());
    try {
      writePacket(packet);
      auto status = readPacket();
      if (status->getInstructionOrError())
        return false;
      else
        return true;
    }
    catch (const IOException& e) {
      return false;
    }
  }

  std::shared_ptr<Packet> Controller::writeData(uint8_t id, uint8_t address,
      const std::vector<uint8_t>& data) {
    auto packet = std::make_shared<Packet>();
    packet->setId(id);
    packet->setLength(data.size() + 3);
    packet->setInstructionOrError(Instructions::WRITE_DATA);
    std::vector<uint8_t> parameters;
    parameters.reserve(data.size() + 1);
    parameters.push_back(address);
    parameters.insert(parameters.end(), data.cbegin(), data.cend());
    packet->setParameters(parameters);
    packet->setChecksum(packet->computeChecksum());
    writePacket(packet);
    return readPacket();
  }

  std::shared_ptr<Packet> Controller::regWriteData(uint8_t id, uint8_t address,
      const std::vector<uint8_t>& data) {
    auto packet = std::make_shared<Packet>();
    packet->setId(id);
    packet->setLength(data.size() + 3);
    packet->setInstructionOrError(Instructions::REG_WRITE);
    std::vector<uint8_t> parameters;
    parameters.reserve(data.size() + 1);
    parameters.push_back(address);
    parameters.insert(parameters.end(), data.cbegin(), data.cend());
    packet->setParameters(parameters);
    packet->setChecksum(packet->computeChecksum());
    writePacket(packet);
    return readPacket();
  }

  std::shared_ptr<Packet> Controller::syncWriteData(uint8_t address, const
      std::unordered_map<uint8_t, std::vector<uint8_t> >& data) {
    if (data.empty())
      throw BadArgumentException<size_t>(data.size(),
        "Controller::syncWriteData: data is empty");
    const auto dataLength = data.begin()->second.size();
    const auto numIds = data.size();
    auto packet = std::make_shared<Packet>();
    packet->setId(broadcastingId);
    packet->setLength((dataLength + 1) * numIds + 4);
    packet->setInstructionOrError(Instructions::SYNC_WRITE);
    std::vector<uint8_t> parameters;
    parameters.push_back(address);
    parameters.push_back(dataLength);
    for (const auto& id : data) {
      if (id.second.size() != dataLength)
        throw BadArgumentException<size_t>(id.second.size(),
          "Controller::syncWriteData: every IDs must have same data length");
      parameters.push_back(id.first);
      parameters.insert(parameters.end(), id.second.cbegin(), id.second.cend());
    }
    packet->setParameters(parameters);
    packet->setChecksum(packet->computeChecksum());
    writePacket(packet);
    return readPacket();
  }

  std::shared_ptr<Packet> Controller::readData(uint8_t id, uint8_t address,
      uint8_t numBytes) {
    auto packet = std::make_shared<Packet>();
    packet->setId(id);
    packet->setLength(4);
    packet->setInstructionOrError(Instructions::READ_DATA);
    std::vector<uint8_t> parameters;
    parameters.reserve(2);
    parameters.push_back(address);
    parameters.push_back(numBytes);
    packet->setParameters(parameters);
    packet->setChecksum(packet->computeChecksum());
    writePacket(packet);
    return readPacket();
  }

  uint16_t Controller::getModelNumber(uint8_t id) {
    auto status = readData(id, Addresses::modelNumberLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getModelNumber(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  uint8_t Controller::getFirmwareVersion(uint8_t id) {
    auto status = readData(id, Addresses::firmwareVersion, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getFirmwareVersion(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  uint8_t Controller::getId(uint8_t id) {
    auto status = readData(id, Addresses::id, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getId(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setId(uint8_t id, uint8_t newId, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(newId);
    auto status = registered ? regWriteData(id, Addresses::id, data) :
      writeData(id, Addresses::id, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setId(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getBaudRate(uint8_t id) {
    auto status = readData(id, Addresses::baudRate, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getBaudRate(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setBaudRate(uint8_t id, uint8_t baudRate, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(baudRate);
    auto status = registered ? regWriteData(id, Addresses::baudRate, data) :
      writeData(id, Addresses::baudRate, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setBaudRate(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getReturnDelayTime(uint8_t id) {
    auto status = readData(id, Addresses::returnDelayTime, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getReturnDelayTime(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  uint16_t Controller::getReturnDelayTimeUs(uint8_t id) {
    return getReturnDelayTime(id) * 2;
  }

  void Controller::setReturnDelayTime(uint8_t id, uint8_t returnDelayTime, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(returnDelayTime);
    auto status = registered ? regWriteData(id, Addresses::returnDelayTime,
      data) : writeData(id, Addresses::returnDelayTime, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setReturnDelayTime(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Controller::getCwAngleLimit(uint8_t id) {
    auto status = readData(id, Addresses::CWAngleLimitLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getCwAngleLimit(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  double Controller::getCwAngleLimitAngle(uint8_t id, double range, uint16_t
      maxTicks) {
    return getCwAngleLimit(id) / static_cast<double>(maxTicks) * range;
  }

  void Controller::setCwAngleLimit(uint8_t id, uint16_t cwAngleLimit, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&cwAngleLimit)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&cwAngleLimit)[1]);
    auto status = registered ? regWriteData(id, Addresses::CWAngleLimitLow,
      data) : writeData(id, Addresses::CWAngleLimitLow, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setCwAngleLimit(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Controller::getCcwAngleLimit(uint8_t id) {
    auto status = readData(id, Addresses::CCWAngleLimitLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getCcwAngleLimit(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  double Controller::getCcwAngleLimitAngle(uint8_t id, double range, uint16_t
      maxTicks) {
    return getCcwAngleLimit(id) / static_cast<double>(maxTicks) * range;
  }

  void Controller::setCcwAngleLimit(uint8_t id, uint16_t ccwAngleLimit, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&ccwAngleLimit)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&ccwAngleLimit)[1]);
    auto status = registered ? regWriteData(id, Addresses::CCWAngleLimitLow,
      data) : writeData(id, Addresses::CCWAngleLimitLow, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setCcwAngleLimit(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getHighestLimitTemperature(uint8_t id) {
    auto status = readData(id, Addresses::highestLimitTemperature, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getHighestLimitTemperature(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setHighestLimitTemperature(uint8_t id, uint8_t temperature,
      bool registered) {
    std::vector<uint8_t> data;
    data.push_back(temperature);
    auto status = registered ? regWriteData(id,
      Addresses::highestLimitTemperature, data) : writeData(id,
      Addresses::highestLimitTemperature, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setHighestLimitTemperature(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getHighestLimitVoltage(uint8_t id) {
    auto status = readData(id, Addresses::highestLimitVoltage, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getHighestLimitVoltage(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  double Controller::getHighestLimitVoltageVolt(uint8_t id) {
    return getHighestLimitVoltage(id) * 0.1;
  }

  void Controller::setHighestLimitVoltage(uint8_t id, uint8_t voltage, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(voltage);
    auto status = registered ? regWriteData(id, Addresses::highestLimitVoltage,
      data) : writeData(id, Addresses::highestLimitVoltage, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setHighestLimitVoltage(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getLowestLimitVoltage(uint8_t id) {
    auto status = readData(id, Addresses::lowestLimitVoltage, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getLowestLimitVoltage(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  double Controller::getLowestLimitVoltageVolt(uint8_t id) {
    return getLowestLimitVoltage(id) * 0.1;
  }

  void Controller::setLowestLimitVoltage(uint8_t id, uint8_t voltage, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(voltage);
    auto status = registered ? regWriteData(id, Addresses::lowestLimitVoltage,
      data) : writeData(id, Addresses::lowestLimitVoltage, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setLowestLimitVoltage(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Controller::getMaxTorque(uint8_t id) {
    auto status = readData(id, Addresses::maxTorqueLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getMaxTorque(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  double Controller::getMaxTorquePercent(uint8_t id) {
    return getMaxTorque(id) / 1023.0 * 100.0;
  }

  void Controller::setMaxTorque(uint8_t id, uint16_t torque, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&torque)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&torque)[1]);
    auto status = registered ? regWriteData(id, Addresses::maxTorqueLow, data) :
      writeData(id, Addresses::maxTorqueLow, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setMaxTorque(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getStatusReturnLevel(uint8_t id) {
    auto status = readData(id, Addresses::statusReturnLevel, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getStatusReturnLevel(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setStatusReturnLevel(uint8_t id, uint8_t level, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(level);
    auto status = registered ? regWriteData(id, Addresses::statusReturnLevel,
      data) : writeData(id, Addresses::statusReturnLevel, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setStatusReturnLevel(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getAlarmLed(uint8_t id) {
    auto status = readData(id, Addresses::alarmLED, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getAlarmLed(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setAlarmLed(uint8_t id, uint8_t code, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(code);
    auto status = registered ? regWriteData(id, Addresses::alarmLED, data) :
      writeData(id, Addresses::alarmLED, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setAlarmLed(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getAlarmShutdown(uint8_t id) {
    auto status = readData(id, Addresses::alarmShutdown, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getAlarmShutdown(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setAlarmShutdown(uint8_t id, uint8_t code, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(code);
    auto status = registered ? regWriteData(id, Addresses::alarmShutdown,
      data) : writeData(id, Addresses::alarmShutdown, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setAlarmShutdown(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Controller::getMultiTurnOffset(uint8_t id) {
    auto status = readData(id, Addresses::multiTurnOffsetLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getMultiTurnOffset(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  void Controller::setMultiTurnOffset(uint8_t id, uint16_t offset, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&offset)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&offset)[1]);
    auto status = registered ? regWriteData(id, Addresses::multiTurnOffsetLow,
      data) : writeData(id, Addresses::multiTurnOffsetLow, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setMultiTurnOffset(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getResolutionDivider(uint8_t id) {
    auto status = readData(id, Addresses::resolutionDivider, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getResolutionDivider(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setResolutionDivider(uint8_t id, uint8_t divider, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(divider);
    auto status = registered ? regWriteData(id, Addresses::resolutionDivider,
      data) : writeData(id, Addresses::resolutionDivider, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setResolutionDivider(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  bool Controller::isTorqueEnable(uint8_t id) {
    auto status = readData(id, Addresses::torqueEnable, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::isTorqueEnable(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0] ? true : false;
  }

  void Controller::setTorqueEnable(uint8_t id, bool enable, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(enable ? 1 : 0);
    auto status = registered ? regWriteData(id, Addresses::torqueEnable, data) :
      writeData(id, Addresses::torqueEnable, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setTorqueEnable(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  bool Controller::isLed(uint8_t id) {
    auto status = readData(id, Addresses::led, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::isLed(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0] ? true : false;
  }

  void Controller::setLed(uint8_t id, bool enable, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(enable ? 1 : 0);
    auto status = registered ? regWriteData(id, Addresses::led, data) :
      writeData(id, Addresses::led, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setLed(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getDGain(uint8_t id) {
    auto status = readData(id, Addresses::dGain, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getDGain(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  double Controller::getDGainK(uint8_t id) {
    return getDGain(id) * 4.0 / 1000.0;
  }

  void Controller::setDGain(uint8_t id, uint8_t gain, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(gain);
    auto status = registered ? regWriteData(id, Addresses::dGain, data) :
      writeData(id, Addresses::dGain, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setDGain(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getIGain(uint8_t id) {
    auto status = readData(id, Addresses::iGain, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getIGain(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  double Controller::getIGainK(uint8_t id) {
    return getIGain(id) * 1000.0 / 2048.0;
  }

  void Controller::setIGain(uint8_t id, uint8_t gain, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(gain);
    auto status = registered ? regWriteData(id, Addresses::iGain, data) :
      writeData(id, Addresses::iGain, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setIGain(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getPGain(uint8_t id) {
    auto status = readData(id, Addresses::pGain, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getPGain(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  double Controller::getPGainK(uint8_t id) {
    return getPGain(id) / 8.0;
  }

  void Controller::setPGain(uint8_t id, uint8_t gain, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(gain);
    auto status = registered ? regWriteData(id, Addresses::pGain, data) :
      writeData(id, Addresses::pGain, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setPGain(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Controller::getGoalPosition(uint8_t id) {
    auto status = readData(id, Addresses::goalPositionLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getGoalPosition(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  double Controller::getGoalPositionAngle(uint8_t id, double range, uint16_t
      maxTicks) {
    return getGoalPosition(id) / static_cast<double>(maxTicks) * range;
  }

  void Controller::setGoalPosition(uint8_t id, uint16_t position, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&position)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&position)[1]);
    auto status = registered ? regWriteData(id, Addresses::goalPositionLow,
      data) : writeData(id, Addresses::goalPositionLow, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setGoalPosition(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Controller::getMovingSpeed(uint8_t id) {
    auto status = readData(id, Addresses::movingSpeedLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getMovingSpeed(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  double Controller::getMovingSpeedRpm(uint8_t id, double rpmPerTick) {
    return getMovingSpeed(id) * rpmPerTick;
  }

  void Controller::setMovingSpeed(uint8_t id, uint16_t speed, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&speed)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&speed)[1]);
    auto status = registered ? regWriteData(id, Addresses::movingSpeedLow,
      data) : writeData(id, Addresses::movingSpeedLow, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setMovingSpeed(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Controller::getTorqueLimit(uint8_t id) {
    auto status = readData(id, Addresses::torqueLimitLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getTorqueLimit(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  double Controller::getTorqueLimitPercent(uint8_t id) {
    return getTorqueLimit(id) / 1023.0 * 100.0;
  }

  void Controller::setTorqueLimit(uint8_t id, uint16_t torque, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&torque)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&torque)[1]);
    auto status = registered ? regWriteData(id, Addresses::torqueLimitLow,
      data) : writeData(id, Addresses::torqueLimitLow, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setTorqueLimit(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Controller::getPresentPosition(uint8_t id) {
    auto status = readData(id, Addresses::presentPositionLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getPresentPosition(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  double Controller::getPresentPositionAngle(uint8_t id, double range, uint16_t
      maxTicks) {
    return getPresentPosition(id) / static_cast<double>(maxTicks) * range;
  }

  uint16_t Controller::getPresentSpeed(uint8_t id) {
    auto status = readData(id, Addresses::presentSpeedLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getPresentSpeed(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  double Controller::getPresentSpeedRpm(uint8_t id, double rpmPerTick) {
    const auto speed = getPresentSpeed(id);
    const auto speedMasked = speed & 0x03FF;
    const auto direction = speed & 0x0400;
    return direction ? speedMasked * rpmPerTick * -1.0 : speedMasked *
      rpmPerTick;
  }

  uint16_t Controller::getPresentLoad(uint8_t id) {
    auto status = readData(id, Addresses::presentLoadLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getPresentLoad(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  double Controller::getPresentLoadPercent(uint8_t id) {
    const auto load = getPresentLoad(id);
    const auto loadMasked = load & 0x03FF;
    const auto direction = load & 0x0400;
    return direction ? loadMasked / 1023.0 * -100.0 : loadMasked / 1023.0
      * 100.0;
  }

  uint8_t Controller::getPresentVoltage(uint8_t id) {
    auto status = readData(id, Addresses::presentVoltage, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getPresentVoltage(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  double Controller::getPresentVoltageVolt(uint8_t id) {
    return getPresentVoltage(id) * 0.1;
  }

  uint8_t Controller::getPresentTemperature(uint8_t id) {
    auto status = readData(id, Addresses::presentTemperature, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getPresentTemperature(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  bool Controller::isInstructionRegistered(uint8_t id) {
    auto status = readData(id, Addresses::registered, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::isInstructionRegistered(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0] ? true : false;
  }

  bool Controller::isMoving(uint8_t id) {
    auto status = readData(id, Addresses::moving, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::isMoving(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0] ? true : false;
  }

  bool Controller::isEEPROMLock(uint8_t id) {
    auto status = readData(id, Addresses::lock, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::isEEPROMLock(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0] ? true : false;
  }

  void Controller::setEEPROMLock(uint8_t id, bool enable, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(enable ? 1 : 0);
    auto status = registered ? regWriteData(id, Addresses::lock, data) :
      writeData(id, Addresses::lock, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setEEPROMLock(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Controller::getPunch(uint8_t id) {
    auto status = readData(id, Addresses::punchLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getPunch(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  void Controller::setPunch(uint8_t id, uint16_t punch, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&punch)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&punch)[1]);
    auto status = registered ? regWriteData(id, Addresses::punchLow, data) :
      writeData(id, Addresses::punchLow, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setPunch(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Controller::getCurrent(uint8_t id) {
    auto status = readData(id, Addresses::currentLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getCurrent(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  double Controller::getCurrentAmp(uint8_t id) {
    return 4.5 * (getCurrent(id) - 2048.0);
  }

  bool Controller::isTorqueControlModeEnable(uint8_t id) {
    auto status = readData(id, Addresses::torqueControlModeEnable, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getTorqueControlModeEnable(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0] ? true : false;
  }

  void Controller::setTorqueControlModeEnable(uint8_t id, bool enable, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(enable ? 1 : 0);
    auto status = registered ? regWriteData(id,
      Addresses::torqueControlModeEnable, data) : writeData(id,
      Addresses::torqueControlModeEnable, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setTorqueControlModeEnable(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Controller::getGoalTorque(uint8_t id) {
    auto status = readData(id, Addresses::goalTorqueLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getGoalTorque(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  double Controller::getGoalTorqueAmp(uint8_t id) {
    const auto torque = getGoalTorque(id);
    const auto torqueMasked = torque & 0x03FF;
    const auto direction = torque & 0x0400;
    return (direction ? torqueMasked -4.5 : torqueMasked * 4.5) * 0.001;
  }

  void Controller::setGoalTorque(uint8_t id, uint16_t torque, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&torque)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&torque)[1]);
    auto status = registered ? regWriteData(id, Addresses::goalTorqueLow,
      data) : writeData(id, Addresses::goalTorqueLow, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setGoalTorque(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getGoalAcceleration(uint8_t id) {
    auto status = readData(id, Addresses::goalAcceleration, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getGoalAcceleration(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  double Controller::getGoalAccelerationRadSec2(uint8_t id) {
    return getGoalAcceleration(id) / 254.0 * 2180.0 * M_PI / 180.0;
  }

  void Controller::setGoalAcceleration(uint8_t id, uint8_t acceleration, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(acceleration);
    auto status = registered ? regWriteData(id, Addresses::goalAcceleration,
      data) : writeData(id, Addresses::goalAcceleration, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setGoalAcceleration(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getCwComplianceMargin(uint8_t id) {
    auto status = readData(id, Addresses::cwComplianceMargin, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getCwComplianceMargin(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setCwComplianceMargin(uint8_t id, uint8_t margin, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(margin);
    auto status = registered ? regWriteData(id, Addresses::cwComplianceMargin,
      data) : writeData(id, Addresses::cwComplianceMargin, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setCwComplianceMargin(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getCcwComplianceMargin(uint8_t id) {
    auto status = readData(id, Addresses::ccwComplianceMargin, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getCcwComplianceMargin(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setCcwComplianceMargin(uint8_t id, uint8_t margin, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(margin);
    auto status = registered ? regWriteData(id, Addresses::ccwComplianceMargin,
      data) : writeData(id, Addresses::ccwComplianceMargin, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setCcwComplianceMargin(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getCwComplianceSlope(uint8_t id) {
    auto status = readData(id, Addresses::cwComplianceSlope, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getCwComplianceSlope(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setCwComplianceSlope(uint8_t id, uint8_t slope, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(slope);
    auto status = registered ? regWriteData(id, Addresses::cwComplianceSlope,
      data) : writeData(id, Addresses::cwComplianceSlope, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setCwComplianceSlope(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getCcwComplianceSlope(uint8_t id) {
    auto status = readData(id, Addresses::ccwComplianceSlope, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getCcwComplianceSlope(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setCcwComplianceSlope(uint8_t id, uint8_t slope, bool
    registered) {
    std::vector<uint8_t> data;
    data.push_back(slope);
    auto status = registered ? regWriteData(id, Addresses::ccwComplianceSlope,
      data) : writeData(id, Addresses::ccwComplianceSlope, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setCcwComplianceSlope(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getDriveMode(uint8_t id) {
    auto status = readData(id, Addresses::driveMode, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getDriveMode(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setDriveMode(uint8_t id, uint8_t mode, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(mode);
    auto status = registered ? regWriteData(id, Addresses::driveMode,
      data) : writeData(id, Addresses::driveMode, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setDriveMode(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Controller::getSensedCurrent(uint8_t id) {
    auto status = readData(id, Addresses::sensedCurrentLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getSensedCurrent(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  void Controller::write(const char* buffer, size_t numBytes) {
    serialPort_->write(buffer, numBytes);
  }

  void Controller::read(char* buffer, size_t numBytes) {
    serialPort_->read(buffer, numBytes);
  }

  std::string Controller::getErrorString(uint8_t errorCode) const {
    std::bitset<8> bitset(errorCode);
    std::stringstream errorStringStream;
    if (bitset.test(7))
      errorStringStream << std::string("-") << std::endl;
    if (bitset.test(6))
      errorStringStream << std::string("Instruction Error") << std::endl;
    if (bitset.test(5))
      errorStringStream << std::string("Overload Error") << std::endl;
    if (bitset.test(4))
      errorStringStream << std::string("Checksum Error") << std::endl;
    if (bitset.test(3))
      errorStringStream << std::string("Range Error") << std::endl;
    if (bitset.test(2))
      errorStringStream << std::string("Overheating Error") << std::endl;
    if (bitset.test(1))
      errorStringStream << std::string("Angle Limit Error") << std::endl;
    if (bitset.test(0))
      errorStringStream << std::string("Input Voltage Error") << std::endl;
    return errorStringStream.str();
  }

  Model Controller::getModelInformation(uint8_t modelNumber) {
    if (isModelSupported(modelNumber))
      return Models::table.at(modelNumber);
    else
      throw BadArgumentException<size_t>(modelNumber,
        "Controller::getModelInformation: invalid model number");
  }

}
