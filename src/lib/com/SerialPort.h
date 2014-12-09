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

/** \file SerialPort.h
    \brief This file defines the SerialPort class which interfaces a serial
           port.
  */

#ifndef LIBDYNAMIXEL_COM_SERIAL_PORT_H
#define LIBDYNAMIXEL_COM_SERIAL_PORT_H

#include <string>

#include <boost/asio/io_service.hpp>
#include <boost/asio/serial_port.hpp>

namespace dynamixel {

  /** The class SerialPort is an interface for serial communication.
      \brief Serial communication interface.
    */
  class SerialPort {
  public:

    /** \name Constructors/destructor
      @{
      */
    /// Constructs the serial port from parameters
    SerialPort(const std::string& device, unsigned int baudRate);
    /// Copy constructor
    SerialPort(const SerialPort& other) = delete;
    /// Copy assignment operator
    SerialPort& operator = (const SerialPort& other) = delete;
    /// Move constructor
    SerialPort(SerialPort&& other) = delete;
    /// Move assignment operator
    SerialPort& operator = (SerialPort&& other) = delete;
     /// Destructor
    ~SerialPort();
    /** @}
      */

    /** \name Accessors
      @{
      */
    /** @}
      */

    /** \name Methods
      @{
      */
    /** @}
      */

  private:

    /** \name Private members
      @{
      */
    /// Boost IO service
    boost::asio::io_service ioService_;
    /// Boost serial port
    boost::asio::serial_port serialPort_;
    /** @}
      */

  };

}

#endif // LIBDYNAMIXEL_COM_SERIAL_PORT_H_H
