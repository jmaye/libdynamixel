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

/** \file IOException.h
    \brief This file defines the IOException class, which represents
           input/output exceptions.
  */

#ifndef LIBDYNAMIXEL_EXCEPTIONS_IO_EXCEPTION_H
#define LIBDYNAMIXEL_EXCEPTIONS_IO_EXCEPTION_H

#include <cstddef>

#include <string>

#include "exceptions/Exception.h"

namespace dynamixel {

  /** The class IOException represents input/output exceptions.
      \brief Input/output exceptions
    */
  class IOException :
    public Exception {
  public:
    /** \name Constructors/Destructor
      @{
      */
    /// Constructs exception
    IOException(const std::string& msg = "", const std::string&
      filename = " ", size_t line = 0, const std::string& function = " ");
    /// Copy constructor
    IOException(const IOException& other) throw ();
    /// Assignment operator
    IOException& operator = (const IOException& other) throw();
    /// Destructor
    virtual ~IOException() throw () = default;
    /** @}
      */

  protected:

  };

}

#endif // LIBDYNAMIXEL_EXCEPTIONS_IO_EXCEPTION_H
