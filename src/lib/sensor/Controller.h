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

/** \file Controller.h
    \brief This file defines the Controller class which abstracts a
           Dynamixel controller.
  */

#ifndef LIBDYNAMIXEL_SENSOR_CONTROLLER_H
#define LIBDYNAMIXEL_SENSOR_CONTROLLER_H

#include <cstdint>
#include <cmath>

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>

#include "com/BinaryReader.h"
#include "com/BinaryWriter.h"
#include "sensor/Models.h"

namespace dynamixel {

  class SerialPort;
  class Packet;

  /** The class Controller represents a Dynamixel controller.
      \brief Dynamixel controller.
    */
  class Controller :
    public BinaryReader, BinaryWriter {
  public:
    /** \name Constructors/destructor
      @{
      */
    /// Constructs the Dynamixel controller with its associated serial port
    Controller(const std::shared_ptr<SerialPort>& serialPort);
    /// Copy constructor
    Controller(const Controller& other) = delete;
    /// Copy assignment operator
    Controller& operator = (const Controller& other) = delete;
    /// Move constructor
    Controller(Controller&& other) = delete;
    /// Move assignment operator
    Controller& operator = (Controller&& other) = delete;
     /// Destructor
    virtual ~Controller() = default;
    /** @}
      */

    /** \name Accessors
      @{
      */
    /// Return the serial port
    const std::shared_ptr<SerialPort>& getSerialPort() {return serialPort_;}
    /** @}
      */

    /** \name Low-level protocol
      @{
      */
    /// Ping command
    bool ping(uint8_t id);
    /// Reset to factory settings
    bool reset(uint8_t id);
    /// Action command
    bool action(uint8_t id);
    /// Synchronous writes data at the given address
    std::shared_ptr<Packet> syncWriteData(uint8_t address, const
      std::unordered_map<uint8_t, std::vector<uint8_t> >& data);
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

    /** \name Convenience methods applicable to all servo motors
      @{
      */
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
    /// Returns the clockwise angle limit as an angle
    double getCwAngleLimitAngle(uint8_t id, double range = 2 * M_PI, uint16_t
      maxTicks = 4095);
    /// Sets the clockwise angle limit
    void setCwAngleLimit(uint8_t id, uint16_t cwAngleLimit, bool
      registered = false);
    /// Returns the counter-clockwise angle limit
    uint16_t getCcwAngleLimit(uint8_t id);
    /// Returns the counterclockwise angle limit as an angle
    double getCcwAngleLimitAngle(uint8_t id, double range = 2 * M_PI, uint16_t
      maxTicks = 4095);
    /// Sets the counter-clockwise angle limit
    void setCcwAngleLimit(uint8_t id, uint16_t ccwAngleLimit, bool
      registered = false);
    /// Returns the highest limit temperature
    uint8_t getHighestLimitTemperature(uint8_t id);
    /// Sets the highest limit temperature (use with caution!)
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
    /// Returns torque enable status
    bool isTorqueEnable(uint8_t id);
    /// Sets torque enable status
    void setTorqueEnable(uint8_t id, bool enable, bool registered = false);
    /// Returns LED status
    bool isLed(uint8_t id);
    /// Sets LED status
    void setLed(uint8_t id, bool enable, bool registered = false);
    /// Returns the goal position
    uint16_t getGoalPosition(uint8_t id);
    /// Returns the goal position as an angle
    double getGoalPositionAngle(uint8_t id, double range = 2 * M_PI, uint16_t
      maxTicks = 4095);
    /// Sets the goal position
    void setGoalPosition(uint8_t id, uint16_t position, bool
      registered = false);
    /// Sets the goal position as an angle
    void setGoalPositionAngle(uint8_t id, double angle, double range = 2 * M_PI,
      uint16_t maxTicks = 4095, bool registered = false);
    /// Returns the moving speed
    uint16_t getMovingSpeed(uint8_t id);
    /// Returns the moving speed in rpm
    double getMovingSpeedRpm(uint8_t id, double rpmPerTick = 0.114);
    /// Sets the moving speed
    void setMovingSpeed(uint8_t id, uint16_t speed, bool registered = false);
    /// Sets the goal position and the moving speed
    void setGoalPositionAndSpeed(uint8_t id, uint16_t position, uint16_t speed,
      bool registered = false);
    /// Sets the goal position angle and the moving speed in rpm
    void setGoalPositionAngleAndSpeedRpm(uint8_t id, double angle, double speed,
       double range = 2 * M_PI, uint16_t maxTicks = 4095, double
       rpmPerTick = 0.114, bool registered = false);
    /// Returns the torque limit
    uint16_t getTorqueLimit(uint8_t id);
    /// Returns the torque limit in percent
    double getTorqueLimitPercent(uint8_t id);
    /// Sets the torque limit
    void setTorqueLimit(uint8_t id, uint16_t torque, bool registered = false);
    /// Returns the present position
    uint16_t getPresentPosition(uint8_t id);
    /// Returns the present position as an angle
    double getPresentPositionAngle(uint8_t id, double range = 2 * M_PI, uint16_t
      maxTicks = 4095);
    /// Returns the present speed
    uint16_t getPresentSpeed(uint8_t id);
    /// Returns the present speed in rpm
    double getPresentSpeedRpm(uint8_t id, double rpmPerTick = 0.114);
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
    /** @}
      */

    /** \name Convenience methods specific to MX series
      @{
      */
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
    /// Returns the D gain
    uint8_t getDGain(uint8_t id);
    /// Returns the K_D gain
    double getDGainK(uint8_t id);
    /// Sets the D gain
    void setDGain(uint8_t id, uint8_t gain, bool registered = false);
    /// Returns the I gain
    uint8_t getIGain(uint8_t id);
    /// Returns the K_I gain
    double getIGainK(uint8_t id);
    /// Sets the I gain
    void setIGain(uint8_t id, uint8_t gain, bool registered = false);
    /// Returns the P gain
    uint8_t getPGain(uint8_t id);
    /// Returns the K_P gain
    double getPGainK(uint8_t id);
    /// Sets the P gain
    void setPGain(uint8_t id, uint8_t gain, bool registered = false);
    /// Returns the goal acceleration
    uint8_t getGoalAcceleration(uint8_t id);
    /// Returns the goal acceleration in radians per seconds^2
    double getGoalAccelerationRadSec2(uint8_t id);
    /// Sets the goal acceleration
    void setGoalAcceleration(uint8_t id, uint8_t acceleration, bool
      registered = false);
    /** @}
      */

    /** \name Convenience methods specific to MX-64 and MX-106
      @{
      */
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
    /** @}
      */

    /** \name Convenience methods specific to EX/RX/AX/DX series
      @{
      */
    /// Returns the clockwise compliance margin
    uint8_t getCwComplianceMargin(uint8_t id);
    /// Sets the clockwise compliance margin
    void setCwComplianceMargin(uint8_t id, uint8_t margin, bool
      registered = false);
    /// Returns the counterclockwise compliance margin
    uint8_t getCcwComplianceMargin(uint8_t id);
    /// Sets the counterclockwise compliance margin
    void setCcwComplianceMargin(uint8_t id, uint8_t margin, bool
      registered = false);
    /// Returns the clockwise compliance slope
    uint8_t getCwComplianceSlope(uint8_t id);
    /// Sets the clockwise compliance slope
    void setCwComplianceSlope(uint8_t id, uint8_t slope, bool
      registered = false);
    /// Returns the counterclockwise compliance slope
    uint8_t getCcwComplianceSlope(uint8_t id);
    /// Sets the counterclockwise compliance slope
    void setCcwComplianceSlope(uint8_t id, uint8_t slope, bool
      registered = false);
    /** @}
      */

    /** \name Convenience methods specific to EX-106+
      @{
      */
    /// Returns the sensed current
    uint16_t getSensedCurrent(uint8_t id);
    /** @}
      */

    /** \name Convenience methods specific to EX-106+ and MX-106
      @{
      */
    /// Returns the drive mode
    uint8_t getDriveMode(uint8_t id);
    /// Sets the drive mode
    void setDriveMode(uint8_t id, uint8_t mode, bool registered = false);
    /** @}
      */

    /** \name Helper methods
      @{
      */
    /// Converts revolutions per minute to radians per second
    static double revPerMin2RadPerSec(double rpm) {
      return rpm / 60.0 * 2 * M_PI;
    }
    /// Converts radians per second to revolutions per minute
    static double radPerSec2RevPerMin(double rps) {
      return rps * 30 / M_PI;
    }
    /// Converts degree to radian
    static float deg2rad(float deg) {
      return deg * M_PI / 180.0;
    }
    /// Converts radian to degree
    static float rad2deg(float rad) {
      return rad * 180.0 / M_PI;
    }
    /// Get model information
    static Model getModelInformation(uint8_t modelNumber);
    /// Is the model supported by the controller
    static bool isModelSupported(uint8_t modelNumber) {
      return Models::table.find(modelNumber) != Models::table.end();
    }
    /** @}
      */

    /** \name Public members
      @{
      */
    /// Broadcasting ID
    static constexpr uint8_t broadcastingId = 0xFE;
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

#endif // LIBDYNAMIXEL_SENSOR_CONTROLLER_H
