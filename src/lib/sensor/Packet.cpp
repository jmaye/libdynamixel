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

#include "sensor/Packet.h"

#include <string>

#include "com/BinaryReader.h"
#include "com/BinaryWriter.h"
#include "exceptions/IOException.h"

namespace dynamixel {

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  uint8_t Packet::computeChecksum() const {
    uint32_t sum = static_cast<uint32_t>(id_) + static_cast<uint32_t>(length_) +
      static_cast<uint32_t>(instructionOrError_);
    for (const auto& parameter : parameters_)
      sum += static_cast<uint32_t>(parameter);
    return ~(static_cast<uint8_t>(sum & 0x000000FF));
  }

  void Packet::read(BinaryReader& stream) {
    uint8_t input;
    stream >> input;
    while (true) {
      while (input != 0xFF)
        stream >> input;
      stream >> input;
      if (input != 0xFF)
        continue;
      stream >> input;
      if (input != id_)
        continue;
      break;
    }
    stream >> length_;
    if (length_ < 2)
      throw IOException("Packet::read(): wrong length");
    stream >> instructionOrError_;
    parameters_.clear();
    parameters_.reserve(length_ - 2);
    for (size_t i = 0; i < (static_cast<size_t>(length_) - 2); ++i) {
      uint8_t parameter;
      stream >> parameter;
      parameters_.push_back(parameter);
    }
    stream >> checksum_;
    if (checksum_ != computeChecksum())
      throw IOException("Packet::read(): wrong checksum");
  }

  void Packet::write(BinaryWriter& stream) const {
    stream << identifier_ << id_ << length_ << instructionOrError_;
    for (const auto& parameter : parameters_)
      stream << parameter;
    stream << checksum_;
  }

  BinaryReader& operator >> (BinaryReader& stream, Packet& obj) {
    obj.read(stream);
    return stream;
  }

  BinaryWriter& operator << (BinaryWriter& stream, const Packet& obj) {
    obj.write(stream);
    return stream;
  }

}
