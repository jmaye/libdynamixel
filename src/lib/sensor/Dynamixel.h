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

/** \file Dynamixel.h
    \brief This file defines the Dynamixel class which represents a Dynamixel 
           device.
  */

#ifndef LIBDYNAMIXEL_SENSOR_DYNAMIXEL_H
#define LIBDYNAMIXEL_SENSOR_DYNAMIXEL_H

#include <cstdint>

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>

#include "com/BinaryReader.h"
#include "com/BinaryWriter.h"

namespace dynamixel {

  class SerialPort;
  class Packet;

  /** The class Dynamixel represents a Dynamixel device.
      \brief Dynamixel device.
    */
  class Dynamixel :
    public BinaryReader, BinaryWriter {
  public:
    /** \name Constructors/destructor
      @{
      */
    /// Constructs the Dynamixel device with its associated serial port
    Dynamixel(const std::shared_ptr<SerialPort>& serialPort);
    /// Copy constructor
    Dynamixel(const Dynamixel& other) = delete;
    /// Copy assignment operator
    Dynamixel& operator = (const Dynamixel& other) = delete;
    /// Move constructor
    Dynamixel(Dynamixel&& other) = delete;
    /// Move assignment operator
    Dynamixel& operator = (Dynamixel&& other) = delete;
     /// Destructor
    virtual ~Dynamixel() = default;
    /** @}
      */

    /** \name Accessors
      @{
      */
    /// Return the serial port
    const std::shared_ptr<SerialPort>& getSerialPort() {return serialPort_;}
    /** @}
      */

    /** \name Methods
      @{
      */
    /// Ping command
    bool ping(uint8_t id);
    /// Reset to factory settings
    bool reset(uint8_t id);
    /// Action command
    bool action(uint8_t id);
    /// Returns the model number
    uint16_t getModelNumber(uint8_t id);
    /// Returns the firmware version
    uint8_t getFirmwareVersion(uint8_t id);
    /// Returns the Id
    uint8_t getId(uint8_t id);
    /// Sets the Id
    void setId(uint8_t id, uint8_t newId, bool registered = false);
    /// Returns the baud rate
    uint8_t getBaudRate(uint8_t id);
    /// Sets the baud rate
    void setBaudRate(uint8_t id, uint8_t baudRate, bool registered = false);
    /// Returns the return delay time
    uint8_t getReturnDelayTime(uint8_t id);
    /// Returns the return delay time in microseconds
    uint16_t getReturnDelayTimeUs(uint8_t id);
    /// Sets the return delay time
    void setReturnDelayTime(uint8_t id, uint8_t returnDelayTime, bool
      registered = false);
    /// Returns the clockwise angle limit
    uint16_t getCwAngleLimit(uint8_t id);
    /// Sets the clockwise angle limit
    void setCwAngleLimit(uint8_t id, uint16_t cwAngleLimit, bool
      registered = false);
    /// Returns the counter-clockwise angle limit
    uint16_t getCcwAngleLimit(uint8_t id);
    /// Sets the counter-clockwise angle limit
    void setCcwAngleLimit(uint8_t id, uint16_t ccwAngleLimit, bool
      registered = false);
    /// Returns the highest limit temperature
    uint8_t getHighestLimitTemperature(uint8_t id);
    /// Sets the highest limit temperature
    void setHighestLimitTemperature(uint8_t id, uint8_t temperature, bool
      registered = false);
    /// Returns the highest limit voltage
    uint8_t getHighestLimitVoltage(uint8_t id);
    /// Returns the highest limit voltage in volts
    double getHighestLimitVoltageVolt(uint8_t id);
    /// Sets the highest limit voltage
    void setHighestLimitVoltage(uint8_t id, uint8_t voltage, bool
      registered = false);
    /// Returns the lowest limit voltage
    uint8_t getLowestLimitVoltage(uint8_t id);
    /// Returns the highest limit voltage in volts
    double getLowestLimitVoltageVolt(uint8_t id);
    /// Sets the lowest limit voltage
    void setLowestLimitVoltage(uint8_t id, uint8_t voltage, bool
      registered = false);
    /// Returns the maximum torque
    uint16_t getMaxTorque(uint8_t id);
    /// Returns the maximum torque in percent
    double getMaxTorquePercent(uint8_t id);
    /// Sets the maximum torque
    void setMaxTorque(uint8_t id, uint16_t torque, bool registered = false);
    /// Returns the status return level
    uint8_t getStatusReturnLevel(uint8_t id);
    /// Sets the status return level
    void setStatusReturnLevel(uint8_t id, uint8_t level, bool
      registered = false);
    /// Returns the alarm LED
    uint8_t getAlarmLed(uint8_t id);
    /// Sets the alarm LED
    void setAlarmLed(uint8_t id, uint8_t code, bool registered = false);
    /// Returns the alarm shutdown
    uint8_t getAlarmShutdown(uint8_t id);
    /// Sets the alarm shutdown
    void setAlarmShutdown(uint8_t id, uint8_t code, bool registered = false);
    /// Returns the multi-turn offset
    uint16_t getMultiTurnOffset(uint8_t id);
    /// Sets the multi-turn offset
    void setMultiTurnOffset(uint8_t id, uint16_t offset, bool
      registered = false);
    /// Returns the resolution divider
    uint8_t getResolutionDivider(uint8_t id);
    /// Sets the resolution divider
    void setResolutionDivider(uint8_t id, uint8_t divider, bool
      registered = false);
    /// Returns torque enable status
    bool isTorqueEnable(uint8_t id);
    /// Sets torque enable status
    void setTorqueEnable(uint8_t id, bool enable, bool registered = false);
    /// Returns LED status
    bool isLed(uint8_t id);
    /// Sets LED status
    void setLed(uint8_t id, bool enable, bool registered = false);
    /// Returns the D gain
    uint8_t getDGain(uint8_t id);
    /// Returns the KD gain
    double getDGainK(uint8_t id);
    /// Sets the D gain
    void setDGain(uint8_t id, uint8_t gain, bool registered = false);
    /// Returns the I gain
    uint8_t getIGain(uint8_t id);
    /// Returns the KI gain
    double getIGainK(uint8_t id);
    /// Sets the I gain
    void setIGain(uint8_t id, uint8_t gain, bool registered = false);
    /// Returns the P gain
    uint8_t getPGain(uint8_t id);
    /// Returns the KP gain
    double getPGainK(uint8_t id);
    /// Sets the P gain
    void setPGain(uint8_t id, uint8_t gain, bool registered = false);
    /// Returns the goal position
    uint16_t getGoalPosition(uint8_t id);
    /// Returns the goal position in degrees
    double getGoalPositionDeg(uint8_t id, double unit = 0.088);
    /// Sets the goal position
    void setGoalPosition(uint8_t id, uint16_t position, bool
      registered = false);
    /// Returns the moving speed
    uint16_t getMovingSpeed(uint8_t id);
    /// Returns the moving speed in rpm
    double getMovingSpeedRpm(uint8_t id, double unit = 0.114);
    /// Sets the moving speed
    void setMovingSpeed(uint8_t id, uint16_t speed, bool registered = false);
    /// Returns the torque limit
    uint16_t getTorqueLimit(uint8_t id);
    /// Returns the torque limit in percent
    double getTorqueLimitPercent(uint8_t id);
    /// Sets the torque limit
    void setTorqueLimit(uint8_t id, uint16_t torque, bool registered = false);
    /// Returns the present position
    uint16_t getPresentPosition(uint8_t id);
    /// Returns the present position in degrees
    double getPresentPositionDeg(uint8_t id, double unit = 0.088);
    /// Returns the present speed
    uint16_t getPresentSpeed(uint8_t id);
    /// Returns the present speed in rpm
    double getPresentSpeedRpm(uint8_t id, double unit = 0.114);
    /// Returns the present load
    uint16_t getPresentLoad(uint8_t id);
    /// Returns the present load in percent
    double getPresentLoadPercent(uint8_t id);
    /// Returns the present voltage
    uint8_t getPresentVoltage(uint8_t id);
    /// Returns the present voltage in volt
    double getPresentVoltageVolt(uint8_t id);
    /// Returns the present temperature
    uint8_t getPresentTemperature(uint8_t id);
    /// Returns registered instruction status
    bool isInstructionRegistered(uint8_t id);
    /// Returns moving status
    bool isMoving(uint8_t id);
    /// Returns EEPROM lock status
    bool isEEPROMLock(uint8_t id);
    /// Sets EEPROM lock status
    void setEEPROMLock(uint8_t id, bool enable, bool registered = false);
    /// Returns the punch
    uint16_t getPunch(uint8_t id);
    /// Sets the punch
    void setPunch(uint8_t id, uint16_t punch, bool registered = false);
    /// Returns the consuming current
    uint16_t getCurrent(uint8_t id);
    /// Returns the consuming current in amp
    double getCurrentAmp(uint8_t id);
    /// Returns torque control mode enable status
    bool isTorqueControlModeEnable(uint8_t id);
    /// Sets torque control mode enable status
    void setTorqueControlModeEnable(uint8_t id, bool enable, bool
      registered = false);
    /// Returns the goal torque
    uint16_t getGoalTorque(uint8_t id);
    /// Returns the goal torque in amp
    double getGoalTorqueAmp(uint8_t id);
    /// Sets the goal torque
    void setGoalTorque(uint8_t id, uint16_t torque, bool registered = false);
    /// Returns the goal acceleration
    uint8_t getGoalAcceleration(uint8_t id);
    /// Returns the goal acceleration in degrees per seconds^2
    double getGoalAccelerationDegSec2(uint8_t id, double unit = 8.583);
    /// Sets the goal acceleration
    void setGoalAcceleration(uint8_t id, uint8_t acceleration, bool
      registered = false);
    /// Synchronous writes data at the given address
    std::shared_ptr<Packet> syncWriteData(uint8_t address, const
      std::unordered_map<uint8_t, std::vector<uint8_t> >& data);
    /** @}
      */

  private:
    /** \name Private methods
      @{
      */
    /// Performs a write on the Dynamixel device
    virtual void write(const char* buffer, size_t numBytes);
    /// Performs a read on the Dynamixel device
    virtual void read(char* buffer, size_t numBytes);
    /// Writes a packet to the Dynamixel device
    void writePacket(const std::shared_ptr<Packet>& packet);
    /// Reads a packet from the Dynamixel device
    std::shared_ptr<Packet> readPacket();
    /// Writes data at the given address
    std::shared_ptr<Packet> writeData(uint8_t id, uint8_t address, const
      std::vector<uint8_t>& data);
    /// Register writes data at the given address
    std::shared_ptr<Packet> regWriteData(uint8_t id, uint8_t address, const
      std::vector<uint8_t>& data);
    /// Reads data at the given address
    std::shared_ptr<Packet> readData(uint8_t id, uint8_t address, uint8_t
      numBytes);
    /// Returns error strings
    std::string getErrorString(uint8_t errorCode) const;
    /** @}
      */

    /** \name Private members
      @{
      */
    /// Serial port
    std::shared_ptr<SerialPort> serialPort_;
    /** @}
      */

  };

}

#endif // LIBDYNAMIXEL_SENSOR_DYNAMIXEL_H
