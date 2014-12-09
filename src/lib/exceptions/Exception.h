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

/** \file Exception.h
    \brief This file defines the Exception class, which is the base class for
           all exceptions.
  */

#ifndef LIBDYNAMIXEL_EXCEPTIONS_EXCEPTION_H
#define LIBDYNAMIXEL_EXCEPTIONS_EXCEPTION_H

#include <cstddef>

#include <exception>
#include <string>

namespace dynamixel {

  /** The class Exception represents the base class for all exceptions.
      \brief Exception base class
    */
  class Exception :
    public std::exception {
  public:
    /** \name Constructors/Destructor
      @{
      */
    /// Constructs exception from message
    Exception(const std::string& msg = "", const std::string& filename = " ",
      size_t line = 0, const std::string& function = " ");
    /// Copy constructor
    Exception(const Exception& other) throw ();
    /// Assignment operator
    Exception& operator = (const Exception& other) throw();
    /// Destructor
    virtual ~Exception() throw ();
    /** @}
      */

    /** \name Accessors
      @{
      */
    /// Access the exception string
    virtual const char* what() const throw();
    /** @}
      */

  protected:
    /** \name Protected members
      @{
      */
    /// Message in the exception
    std::string msg_;
    /// Filename where the exception occurs
    std::string filename_;
    /// Function where the exception occurs
    std::string function_;
    /// Line number where the exception occurs
    size_t line_;
    /// Output message
    std::string outputMessage_;
    /** @}
      */

  };

}

#endif // LIBDYNAMIXEL_EXCEPTIONS_EXCEPTION_H
