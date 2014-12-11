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

/** \file Instructions.h
    \brief This file defines the Instructions namespace which contains the
           different instruction codes.
  */

#ifndef LIBDYNAMIXEL_SENSOR_INSTRUCTIONS_H
#define LIBDYNAMIXEL_SENSOR_INSTRUCTIONS_H

#include <cstdint>

namespace dynamixel {

  /** The namespace Instructions contains the different instruction codes.
      \brief Instruction codes.
    */
  namespace Instructions {
    /** \name Public members
      @{
      */
    /// Ping instruction
    static constexpr uint8_t PING = 0x01;
    /// Read data instruction
    static constexpr uint8_t READ_DATA = 0x02;
    /// Write data instruction
    static constexpr uint8_t WRITE_DATA = 0x03;
    /// Register write instruction
    static constexpr uint8_t REG_WRITE = 0x04;
    /// Action instruction
    static constexpr uint8_t ACTION = 0x05;
    /// Reset instruction
    static constexpr uint8_t RESET = 0x06;
    /// Synchronous write instruction
    static constexpr uint8_t SYNC_WRITE = 0x07;
    /** @}
      */

  };

}

#endif // LIBDYNAMIXEL_SENSOR_INSTRUCTIONS_H
