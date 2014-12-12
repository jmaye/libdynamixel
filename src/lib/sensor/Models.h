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

/** \file Models.h
    \brief This file defines the various Dynamixel servo motors and their
           specification.
  */

#ifndef LIBDYNAMIXEL_SENSOR_MODELS_H
#define LIBDYNAMIXEL_SENSOR_MODELS_H

#include <cstdint>

#include <unordered_map>
#include <string>
#include <iostream>

namespace dynamixel {

  /** The structure Model contains the specification of Dynamixel servo motor.
      \brief Dynamixel servo motor model.
    */
  struct Model {
    /// Model name
    std::string name;
    /// Maximum number of ticks
    uint16_t maxTicks;
    /// Range in degrees
    double rangeInDegrees;
    /// Revolutions per minutes per tick
    double rpmPerTick;
    /// Output to a stream
    void ostream(std::ostream& stream) {
      stream << "Name: " << name << "\n";
      stream << "Maximum ticks: " << maxTicks << "\n";
      stream << "Range in degrees: " << rangeInDegrees << "\n";
      stream << "Rpm per tick: " << rpmPerTick << "\n";
    };
  };

  /** The namespace Models contains the different supported Dynamixel models.
      \brief Dynamixel models.
    */
  namespace Models {
    /// Model table
    static const std::unordered_map<uint8_t, Model> table {
      {0x0140, Model{"MX-106", 4095, 360.0, 0.114}},
      {0x0136, Model{"MX-64", 4095, 360.0, 0.114}},
      {0x001D, Model{"MX-28", 4095, 360.0, 0.114}},
      {0x0168, Model{"MX-12W", 4095, 360.0, 0.114}},
      {0x006B, Model{"EX-106+", 4095, 250.92, 0.111}},
      {0x0040, Model{"RX-64", 1023, 300.0, 0.111}},
      {0x001C, Model{"RX-28", 1023, 300.0, 0.111}},
      {0x0018, Model{"RX-24F", 1023, 300.0, 0.111}},
      {0x000A, Model{"RX-10", 1023, 300.0, 0.111}},
      {0x0012, Model{"AX-18", 1023, 300.0, 0.111}},
      {0x012C, Model{"AX-12W", 1023, 300.0, 0.111}},
      {0x000C, Model{"AX-12", 1023, 300.0, 0.111}},
      {0x0075, Model{"DX-117", 1023, 300.0, 0.111}},
      {0x0074, Model{"DX-116", 1023, 300.0, 0.111}},
      {0x0071, Model{"DX-113", 1023, 300.0, 0.111}}
    };

  }

}

#endif // LIBDYNAMIXEL_SENSOR_MODELS_H
