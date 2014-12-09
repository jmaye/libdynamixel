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

#include "exceptions/Exception.h"

#include <sstream>

namespace dynamixel {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  Exception::Exception(const std::string& msg, const std::string& filename,
      size_t line, const std::string& function) :
      msg_(msg),
      filename_(filename),
      function_(function),
      line_(line) {
    std::stringstream stream;
    if (function != " ")
      stream << function << ": ";
    stream << msg;
    if (filename != " ")
      stream << " [file = " << filename << "]";
    if (line)
      stream << "[line = " << line << "]";
    outputMessage_ = stream.str();
  }

  Exception::Exception(const Exception& other) throw() :
      msg_(other.msg_),
      filename_(other.filename_),
      function_(other.function_),
      line_(other.line_),
      outputMessage_(other.outputMessage_) {
  }

  Exception& Exception::operator = (const Exception& other) throw() {
    if (this != &other) {
      msg_ = other.msg_;
      filename_ = other.filename_;
      function_ = other.function_;
      line_ = other.line_;
      outputMessage_ = other.outputMessage_;
    }
    return *this;
  }

  Exception::~Exception() throw () {
  }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

  const char* Exception::what() const throw() {
    return outputMessage_.c_str();
  }

}
