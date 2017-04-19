cmake_minimum_required(VERSION 3.6)
project(Chasing17)

include(CMakeForceCompiler)

# specify what environment we're working with
# valid options: david_desktop, david_laptop
set(COMPILE_ENV david_laptop)

# specify the cross compiler
set(TOOLCHAIN_PREFIX arm-none-eabi-)
set(CC ${TOOLCHAIN_PREFIX}gcc)
set(CXX ${TOOLCHAIN_PREFIX}g++)
set(AR ${TOOLCHAIN_PREFIX}ar)
set(CMAKE_C_COMPILER ${CC})
SET(CMAKE_CXX_COMPILER ${CXX})
CMAKE_FORCE_C_COMPILER(${CC} GNU)
CMAKE_FORCE_CXX_COMPILER(${CXX} GNU)

# find libsccc
if (${COMPILE_ENV} STREQUAL david_desktop)
    set(LIBSCCC_PATH E:/David/Dropbox/ENGG4950S/libsccc)
elseif (${COMPILE_ENV} STREQUAL david_laptop)
    set(LIBSCCC_PATH C:/Users/david/Dropbox/ENGG4950S/libsccc)
endif ()

# define the c++ standard and includes to resolve *.h
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)
include_directories(inc)
include_directories(${LIBSCCC_PATH}/inc)

# set build type according to what we have in CLion
if (${CMAKE_BUILD_TYPE} STREQUAL Release)
    set(MAKE_BUILD_TYPE RELEASE)
elseif (${CMAKE_BUILD_TYPE} STREQUAL Debug)
    set(MAKE_BUILD_TYPE DEBUG)
else (${CMAKE_BUILD_TYPE} STREQUAL Release)
    message(WARNING "No build type configured. Defaulting to Debug")
    set(MAKE_BUILD_TYPE DEBUG)
endif ()

# set make parameters
set(MAKE_PATH "C:/Program Files (x86)/GnuWin32/bin/make.exe")
set(MAKE_ARGS -f ${Chasing17_SOURCE_DIR}/Makefile -k)
if (${COMPILE_ENV} STREQUAL david_desktop)
    set(MAKE_ARGS ${MAKE_ARGS} -j8)
else ()
    set(MAKE_ARGS ${MAKE_ARGS} -j4)
endif ()
set(MAKE_VARS SCCC_BUILD=${MAKE_BUILD_TYPE} PWD=${Chasing17_SOURCE_DIR} CWD=${PWD}
        LIBSCCC_BIN_PATH=${LIBSCCC_PATH}/lib
        LIBSCCC_INC_PATH=${LIBSCCC_PATH}/inc)

# buiders
add_custom_target(make_once
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=${SCCC_CONFIG} all
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(INNO_make
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=INNO all
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2014_CAMERA_make
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2014_CAMERA all
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2014_CCD_make
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2014_CCD all
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2014_INNO_make
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2014_INNO all
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2014_INNO_FX_make
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2014_INNO_FX all
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2014_MAGNETIC_make
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2014_MAGNETIC all
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(VCAN_FX15DEV_make
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=VCAN_FX15DEV all
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2015_CAMERA_make
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2015_CAMERA all
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2015_MAGNETIC_make
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2015_MAGNETIC all
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2015_CCD_make
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2015_CCD all
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(VCAN_Z4DEV_make
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=VCAN_Z4DEV all
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2015_CCD2_make
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2015_CCD2 all
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2015_M2_make
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2015_M2 all
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2016_INNO_make
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2016_INNO all
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2016_CAMERA_make
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2016_CAMERA all
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2016_CCD_make
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2016_CCD all
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2016_RAIL_make
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2016_RAIL all
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2016_MAGNETIC_make
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2016_MAGNETIC all
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2016_MAGNETIC_YANG_make
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2016_MAGNETIC_YANG all
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2017_INNO_make
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2017_INNO all
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2017_CHASING_make
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2017_CHASING all
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})

# cleaners
add_custom_target(clean_only
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=${SCCC_CONFIG} clean
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(INNO_clean
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=INNO clean
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2014_CAMERA_clean
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2014_CAMERA clean
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2014_CCD_clean
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2014_CCD clean
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2014_INNO_clean
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2014_INNO clean
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2014_INNO_FX_clean
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2014_INNO_FX clean
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2014_MAGNETIC_clean
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2014_MAGNETIC clean
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(VCAN_FX15DEV_clean
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=VCAN_FX15DEV clean
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2015_CAMERA_clean
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2015_CAMERA clean
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2015_MAGNETIC_clean
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2015_MAGNETIC clean
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2015_CCD_clean
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2015_CCD clean
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(VCAN_Z4DEV_clean
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=VCAN_Z4DEV clean
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2015_CCD2_clean
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2015_CCD2 clean
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2015_M2_clean
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2015_M2 clean
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2016_INNO_clean
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2016_INNO clean
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2016_CAMERA_clean
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2016_CAMERA clean
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2016_CCD_clean
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2016_CCD clean
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2016_RAIL_clean
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2016_RAIL clean
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2016_MAGNETIC_clean
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2016_MAGNETIC clean
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2016_MAGNETIC_YANG_clean
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2016_MAGNETIC_YANG clean
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2017_INNO_clean
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2017_INNO clean
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})
add_custom_target(2017_CHASING_clean
        COMMAND ${MAKE_PATH} ${MAKE_ARGS} ${MAKE_VARS} SCCC_CONFIG=2017_CHASING clean
        WORKING_DIRECTORY ${Chasing17_SOURCE_DIR})

# configuration enabling all resolutions
file(GLOB_RECURSE SOURCES "src/*.cpp")
add_executable(CurrentConfigSymbols ${SOURCES})
