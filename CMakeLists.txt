cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  libdynamixel
  VERSION 0.1
  RELEASE alpha
  SUMMARY "A C++-based library for interfacing Dynamixel servo motors."
  AUTHOR "Jerome Maye"
  CONTACT "jerome.maye@mavt.ethz.ch"
  HOME "https://github.com/jmaye/libdynamixel"
  LICENSE "GNU Lesser General Public License (LGPLv3)"
  PREFIX dynamixel-
  INSTALL /usr/
  EXTRA_CXX_FLAGS "-std=c++0x -Wall -Wextra"
)

remake_doc(
  html
  latex
)

remake_pack_deb(
  DESCRIPTION "Main library"
)

remake_pack_deb(
  COMPONENT dev
  DESCRIPTION "Development headers"
  DEPENDS libdynamixel
)

remake_pack_deb(
  COMPONENT console-tools
  DESCRIPTION "Console tools"
  DEPENDS libdynamixel
)

remake_pack_deb(
  COMPONENT doc
  DESCRIPTION "Documentation"
  DEPENDS libdynamixel
)

remake_distribute_deb(
  DISTRIBUTION lucid
  SECTION libs
  UPLOAD ppa:jmaye/ethz
  DEPENDS remake doxygen pkg-config
  PASS CMAKE_BUILD_TYPE LIBDYNAMIXEL_GIT_REVISION
)

remake_distribute_deb(
  DISTRIBUTION precise
  SECTION libs
  UPLOAD ppa:jmaye/ethz
  DEPENDS remake doxygen pkg-config
  PASS CMAKE_BUILD_TYPE LIBDYNAMIXEL_GIT_REVISION
)

remake_distribute_deb(
  DISTRIBUTION trusty
  SECTION libs
  UPLOAD ppa:jmaye/ethz
  DEPENDS remake doxygen pkg-config
  PASS CMAKE_BUILD_TYPE LIBDYNAMIXEL_GIT_REVISION
)
