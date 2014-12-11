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

#include "sensor/Dynamixel.h"

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

  Dynamixel::Dynamixel(const std::shared_ptr<SerialPort>& serialPort) :
      serialPort_(serialPort) {
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void Dynamixel::writePacket(const std::shared_ptr<Packet>& packet) {
    *this << *packet;
  }

  std::shared_ptr<Packet> Dynamixel::readPacket() {
    auto packet = std::make_shared<Packet>();
    *this >> *packet;
    return packet;
  }

  bool Dynamixel::ping(uint8_t id) {
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

  bool Dynamixel::reset(uint8_t id) {
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

  bool Dynamixel::action(uint8_t id) {
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

  std::shared_ptr<Packet> Dynamixel::writeData(uint8_t id, uint8_t address,
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

  std::shared_ptr<Packet> Dynamixel::regWriteData(uint8_t id, uint8_t address,
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

  std::shared_ptr<Packet> Dynamixel::syncWriteData(uint8_t address, const
      std::unordered_map<uint8_t, std::vector<uint8_t> >& data) {
    if (data.empty())
      throw BadArgumentException<size_t>(data.size(),
        "Dynamixel::syncWriteData: data is empty");
    const auto dataLength = data.begin()->second.size();
    const auto numIds = data.size();
    auto packet = std::make_shared<Packet>();
    packet->setId(0XFE);
    packet->setLength((dataLength + 1) * numIds + 4);
    packet->setInstructionOrError(Instructions::SYNC_WRITE);
    std::vector<uint8_t> parameters;
    parameters.push_back(address);
    parameters.push_back(dataLength);
    for (const auto& id : data) {
      if (id.second.size() != dataLength)
        throw BadArgumentException<size_t>(id.second.size(),
          "Dynamixel::syncWriteData: every IDs must have same data length");
      parameters.push_back(id.first);
      parameters.insert(parameters.end(), id.second.cbegin(), id.second.cend());
    }
    packet->setParameters(parameters);
    packet->setChecksum(packet->computeChecksum());
    writePacket(packet);
    return readPacket();
  }

  std::shared_ptr<Packet> Dynamixel::readData(uint8_t id, uint8_t address,
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

  uint16_t Dynamixel::getModelNumber(uint8_t id) {
    auto status = readData(id, Addresses::modelNumberLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getModelNumber(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  uint8_t Dynamixel::getFirmwareVersion(uint8_t id) {
    auto status = readData(id, Addresses::firmwareVersion, 1);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getFirmwareVersion(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  uint8_t Dynamixel::getId(uint8_t id) {
    auto status = readData(id, Addresses::id, 1);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getId(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Dynamixel::setId(uint8_t id, uint8_t newId, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(newId);
    auto status = registered ? regWriteData(id, Addresses::id, data) :
      writeData(id, Addresses::id, data);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::setId(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Dynamixel::getBaudRate(uint8_t id) {
    auto status = readData(id, Addresses::baudRate, 1);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getBaudRate(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Dynamixel::setBaudRate(uint8_t id, uint8_t baudRate, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(baudRate);
    auto status = registered ? regWriteData(id, Addresses::baudRate, data) :
      writeData(id, Addresses::baudRate, data);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::setBaudRate(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Dynamixel::getReturnDelayTime(uint8_t id) {
    auto status = readData(id, Addresses::returnDelayTime, 1);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getReturnDelayTime(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  uint16_t Dynamixel::getReturnDelayTimeUs(uint8_t id) {
    return getReturnDelayTime(id) * 2;
  }

  void Dynamixel::setReturnDelayTime(uint8_t id, uint8_t returnDelayTime, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(returnDelayTime);
    auto status = registered ? regWriteData(id, Addresses::returnDelayTime,
      data) : writeData(id, Addresses::returnDelayTime, data);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::setReturnDelayTime(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Dynamixel::getCwAngleLimit(uint8_t id) {
    auto status = readData(id, Addresses::CWAngleLimitLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getCwAngleLimit(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  void Dynamixel::setCwAngleLimit(uint8_t id, uint16_t cwAngleLimit, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&cwAngleLimit)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&cwAngleLimit)[1]);
    auto status = registered ? regWriteData(id, Addresses::CWAngleLimitLow,
      data) : writeData(id, Addresses::CWAngleLimitLow, data);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::setCwAngleLimit(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Dynamixel::getCcwAngleLimit(uint8_t id) {
    auto status = readData(id, Addresses::CCWAngleLimitLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getCcwAngleLimit(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  void Dynamixel::setCcwAngleLimit(uint8_t id, uint16_t ccwAngleLimit, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&ccwAngleLimit)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&ccwAngleLimit)[1]);
    auto status = registered ? regWriteData(id, Addresses::CCWAngleLimitLow,
      data) : writeData(id, Addresses::CCWAngleLimitLow, data);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::setCcwAngleLimit(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Dynamixel::getHighestLimitTemperature(uint8_t id) {
    auto status = readData(id, Addresses::highestLimitTemperature, 1);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getHighestLimitTemperature(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Dynamixel::setHighestLimitTemperature(uint8_t id, uint8_t temperature,
      bool registered) {
    std::vector<uint8_t> data;
    data.push_back(temperature);
    auto status = registered ? regWriteData(id,
      Addresses::highestLimitTemperature, data) : writeData(id,
      Addresses::highestLimitTemperature, data);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::setHighestLimitTemperature(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Dynamixel::getHighestLimitVoltage(uint8_t id) {
    auto status = readData(id, Addresses::highestLimitVoltage, 1);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getHighestLimitVoltage(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  double Dynamixel::getHighestLimitVoltageVolt(uint8_t id) {
    return getHighestLimitVoltage(id) * 0.1;
  }

  void Dynamixel::setHighestLimitVoltage(uint8_t id, uint8_t voltage, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(voltage);
    auto status = registered ? regWriteData(id, Addresses::highestLimitVoltage,
      data) : writeData(id, Addresses::highestLimitVoltage, data);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::setHighestLimitVoltage(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Dynamixel::getLowestLimitVoltage(uint8_t id) {
    auto status = readData(id, Addresses::lowestLimitVoltage, 1);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getLowestLimitVoltage(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  double Dynamixel::getLowestLimitVoltageVolt(uint8_t id) {
    return getLowestLimitVoltage(id) * 0.1;
  }

  void Dynamixel::setLowestLimitVoltage(uint8_t id, uint8_t voltage, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(voltage);
    auto status = registered ? regWriteData(id, Addresses::lowestLimitVoltage,
      data) : writeData(id, Addresses::lowestLimitVoltage, data);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::setLowestLimitVoltage(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Dynamixel::getMaxTorque(uint8_t id) {
    auto status = readData(id, Addresses::maxTorqueLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getMaxTorque(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  double Dynamixel::getMaxTorquePercent(uint8_t id) {
    return getMaxTorque(id) / 1023.0 * 100.0;
  }

  void Dynamixel::setMaxTorque(uint8_t id, uint16_t torque, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&torque)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&torque)[1]);
    auto status = registered ? regWriteData(id, Addresses::maxTorqueLow, data) :
      writeData(id, Addresses::maxTorqueLow, data);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::setMaxTorque(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Dynamixel::getStatusReturnLevel(uint8_t id) {
    auto status = readData(id, Addresses::statusReturnLevel, 1);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getStatusReturnLevel(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Dynamixel::setStatusReturnLevel(uint8_t id, uint8_t level, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(level);
    auto status = registered ? regWriteData(id, Addresses::statusReturnLevel,
      data) : writeData(id, Addresses::statusReturnLevel, data);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::setStatusReturnLevel(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Dynamixel::getAlarmLed(uint8_t id) {
    auto status = readData(id, Addresses::alarmLED, 1);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getAlarmLed(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Dynamixel::setAlarmLed(uint8_t id, uint8_t code, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(code);
    auto status = registered ? regWriteData(id, Addresses::alarmLED, data) :
      writeData(id, Addresses::alarmLED, data);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::setAlarmLed(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Dynamixel::getAlarmShutdown(uint8_t id) {
    auto status = readData(id, Addresses::alarmShutdown, 1);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getAlarmShutdown(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Dynamixel::setAlarmShutdown(uint8_t id, uint8_t code, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(code);
    auto status = registered ? regWriteData(id, Addresses::alarmShutdown,
      data) : writeData(id, Addresses::alarmShutdown, data);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::setAlarmShutdown(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Dynamixel::getMultiTurnOffset(uint8_t id) {
    auto status = readData(id, Addresses::multiTurnOffsetLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getMultiTurnOffset(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  void Dynamixel::setMultiTurnOffset(uint8_t id, uint16_t offset, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&offset)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&offset)[1]);
    auto status = registered ? regWriteData(id, Addresses::multiTurnOffsetLow,
      data) : writeData(id, Addresses::multiTurnOffsetLow, data);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::setMultiTurnOffset(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Dynamixel::getResolutionDivider(uint8_t id) {
    auto status = readData(id, Addresses::resolutionDivider, 1);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getResolutionDivider(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Dynamixel::setResolutionDivider(uint8_t id, uint8_t divider, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(divider);
    auto status = registered ? regWriteData(id, Addresses::resolutionDivider,
      data) : writeData(id, Addresses::resolutionDivider, data);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::setResolutionDivider(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  bool Dynamixel::isTorqueEnable(uint8_t id) {
    auto status = readData(id, Addresses::torqueEnable, 1);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::isTorqueEnable(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0] ? true : false;
  }

  void Dynamixel::setTorqueEnable(uint8_t id, bool enable, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(enable ? 1 : 0);
    auto status = registered ? regWriteData(id, Addresses::torqueEnable, data) :
      writeData(id, Addresses::torqueEnable, data);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::setTorqueEnable(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  bool Dynamixel::isLed(uint8_t id) {
    auto status = readData(id, Addresses::led, 1);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::isLed(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0] ? true : false;
  }

  void Dynamixel::setLed(uint8_t id, bool enable, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(enable ? 1 : 0);
    auto status = registered ? regWriteData(id, Addresses::led, data) :
      writeData(id, Addresses::led, data);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::setLed(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Dynamixel::getDGain(uint8_t id) {
    auto status = readData(id, Addresses::dGain, 1);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getDGain(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  double Dynamixel::getDGainK(uint8_t id) {
    return getDGain(id) * 4.0 / 1000.0;
  }

  void Dynamixel::setDGain(uint8_t id, uint8_t gain, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(gain);
    auto status = registered ? regWriteData(id, Addresses::dGain, data) :
      writeData(id, Addresses::dGain, data);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::setDGain(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Dynamixel::getIGain(uint8_t id) {
    auto status = readData(id, Addresses::iGain, 1);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getIGain(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  double Dynamixel::getIGainK(uint8_t id) {
    return getIGain(id) * 1000.0 / 2048.0;
  }

  void Dynamixel::setIGain(uint8_t id, uint8_t gain, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(gain);
    auto status = registered ? regWriteData(id, Addresses::iGain, data) :
      writeData(id, Addresses::iGain, data);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::setIGain(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Dynamixel::getPGain(uint8_t id) {
    auto status = readData(id, Addresses::pGain, 1);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getPGain(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  double Dynamixel::getPGainK(uint8_t id) {
    return getPGain(id) / 8.0;
  }

  void Dynamixel::setPGain(uint8_t id, uint8_t gain, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(gain);
    auto status = registered ? regWriteData(id, Addresses::pGain, data) :
      writeData(id, Addresses::pGain, data);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::setPGain(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Dynamixel::getGoalPosition(uint8_t id) {
    auto status = readData(id, Addresses::goalPositionLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getGoalPosition(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  double Dynamixel::getGoalPositionDeg(uint8_t id, double unit) {
    return getGoalPosition(id) * unit;
  }

  void Dynamixel::setGoalPosition(uint8_t id, uint16_t position, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&position)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&position)[1]);
    auto status = registered ? regWriteData(id, Addresses::goalPositionLow,
      data) : writeData(id, Addresses::goalPositionLow, data);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::setGoalPosition(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Dynamixel::getMovingSpeed(uint8_t id) {
    auto status = readData(id, Addresses::movingSpeedLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getMovingSpeed(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  double Dynamixel::getMovingSpeedRpm(uint8_t id, double unit) {
    return getMovingSpeed(id) * unit;
  }

  void Dynamixel::setMovingSpeed(uint8_t id, uint16_t speed, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&speed)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&speed)[1]);
    auto status = registered ? regWriteData(id, Addresses::movingSpeedLow,
      data) : writeData(id, Addresses::movingSpeedLow, data);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::setMovingSpeed(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Dynamixel::getTorqueLimit(uint8_t id) {
    auto status = readData(id, Addresses::torqueLimitLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getTorqueLimit(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  double Dynamixel::getTorqueLimitPercent(uint8_t id) {
    return getTorqueLimit(id) / 1023.0 * 100.0;
  }

  void Dynamixel::setTorqueLimit(uint8_t id, uint16_t torque, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&torque)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&torque)[1]);
    auto status = registered ? regWriteData(id, Addresses::torqueLimitLow,
      data) : writeData(id, Addresses::torqueLimitLow, data);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::setTorqueLimit(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Dynamixel::getPresentPosition(uint8_t id) {
    auto status = readData(id, Addresses::presentPositionLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getPresentPosition(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  double Dynamixel::getPresentPositionDeg(uint8_t id, double unit) {
    return getPresentPosition(id) * unit;
  }

  uint16_t Dynamixel::getPresentSpeed(uint8_t id) {
    auto status = readData(id, Addresses::presentSpeedLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getPresentSpeed(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  double Dynamixel::getPresentSpeedRpm(uint8_t id, double unit) {
    const auto speed = getPresentSpeed(id);
    const auto speedMasked = speed & 0x03FF;
    const auto direction = speed & 0x0400;
    return direction ? speedMasked * unit * -1.0 : speedMasked * unit;
  }

  uint16_t Dynamixel::getPresentLoad(uint8_t id) {
    auto status = readData(id, Addresses::presentLoadLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getPresentLoad(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  double Dynamixel::getPresentLoadPercent(uint8_t id) {
    const auto load = getPresentLoad(id);
    const auto loadMasked = load & 0x03FF;
    const auto direction = load & 0x0400;
    return direction ? loadMasked / 1023.0 * -1.0 : loadMasked / 1023.0;
  }

  uint8_t Dynamixel::getPresentVoltage(uint8_t id) {
    auto status = readData(id, Addresses::presentVoltage, 1);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getPresentVoltage(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  double Dynamixel::getPresentVoltageVolt(uint8_t id) {
    return getPresentVoltage(id) * 0.1;
  }

  uint8_t Dynamixel::getPresentTemperature(uint8_t id) {
    auto status = readData(id, Addresses::presentTemperature, 1);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getPresentTemperature(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  bool Dynamixel::isInstructionRegistered(uint8_t id) {
    auto status = readData(id, Addresses::registered, 1);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::isInstructionRegistered(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0] ? true : false;
  }

  bool Dynamixel::isMoving(uint8_t id) {
    auto status = readData(id, Addresses::moving, 1);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::isMoving(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0] ? true : false;
  }

  bool Dynamixel::isEEPROMLock(uint8_t id) {
    auto status = readData(id, Addresses::lock, 1);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::isEEPROMLock(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0] ? true : false;
  }

  void Dynamixel::setEEPROMLock(uint8_t id, bool enable, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(enable ? 1 : 0);
    auto status = registered ? regWriteData(id, Addresses::lock, data) :
      writeData(id, Addresses::lock, data);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::setEEPROMLock(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Dynamixel::getPunch(uint8_t id) {
    auto status = readData(id, Addresses::punchLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getPunch(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  void Dynamixel::setPunch(uint8_t id, uint16_t punch, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&punch)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&punch)[1]);
    auto status = registered ? regWriteData(id, Addresses::punchLow, data) :
      writeData(id, Addresses::punchLow, data);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::setPunch(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Dynamixel::getCurrent(uint8_t id) {
    auto status = readData(id, Addresses::currentLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getCurrent(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  double Dynamixel::getCurrentAmp(uint8_t id) {
    return 4.5 * (getCurrent(id) - 2048.0);
  }

  bool Dynamixel::isTorqueControlModeEnable(uint8_t id) {
    auto status = readData(id, Addresses::torqueControlModeEnable, 1);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getTorqueControlModeEnable(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0] ? true : false;
  }

  void Dynamixel::setTorqueControlModeEnable(uint8_t id, bool enable, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(enable ? 1 : 0);
    auto status = registered ? regWriteData(id,
      Addresses::torqueControlModeEnable, data) : writeData(id,
      Addresses::torqueControlModeEnable, data);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::setTorqueControlModeEnable(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Dynamixel::getGoalTorque(uint8_t id) {
    auto status = readData(id, Addresses::goalTorqueLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getGoalTorque(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  double Dynamixel::getGoalTorqueAmp(uint8_t id) {
    const auto torque = getGoalTorque(id);
    const auto torqueMasked = torque & 0x03FF;
    const auto direction = torque & 0x0400;
    return (direction ? torqueMasked -4.5 : torqueMasked * 4.5) * 0.001;
  }

  void Dynamixel::setGoalTorque(uint8_t id, uint16_t torque, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&torque)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&torque)[1]);
    auto status = registered ? regWriteData(id, Addresses::goalTorqueLow,
      data) : writeData(id, Addresses::goalTorqueLow, data);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::setGoalTorque(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Dynamixel::getGoalAcceleration(uint8_t id) {
    auto status = readData(id, Addresses::goalAcceleration, 1);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::getGoalAcceleration(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  double Dynamixel::getGoalAccelerationDegSec2(uint8_t id, double unit) {
    return getGoalAcceleration(id) * unit;
  }

  void Dynamixel::setGoalAcceleration(uint8_t id, uint8_t acceleration, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(acceleration);
    auto status = registered ? regWriteData(id, Addresses::goalAcceleration,
      data) : writeData(id, Addresses::goalAcceleration, data);
    if (status->getInstructionOrError())
      throw IOException("Dynamixel::setGoalAcceleration(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  void Dynamixel::write(const char* buffer, size_t numBytes) {
    serialPort_->write(buffer, numBytes);
  }

  void Dynamixel::read(char* buffer, size_t numBytes) {
    serialPort_->read(buffer, numBytes);
  }

  std::string Dynamixel::getErrorString(uint8_t errorCode) const {
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

}
