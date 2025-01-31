#!/usr/bin/env bash
# Pre build script that connects generated C code with
# user written C++ code and copies neccessary files from
# microROS submodule to Core/Src directory.
# Project is configured to run this script automatically before
# each build.

FILEPATH=Core/Src/main.c
INCLUDE='#include "user-code\/main_cpp.h"'
MAINCALL='MainCpp();'

# Copy nesseccary files from microROS submodule to Core/Src directory
cp micro_ros_stm32cubemx_utils/extra_sources/custom_memory_manager.c Core/Src/custom_memory_manager.c
cp micro_ros_stm32cubemx_utils/extra_sources/microros_allocators.c Core/Src/microros_allocators.c
cp micro_ros_stm32cubemx_utils/extra_sources/microros_time.c Core/Src/microros_time.c
cp micro_ros_stm32cubemx_utils/extra_sources/microros_transports/dma_transport.c Core/Src/dma_transport.c 

# Add call to MainCpp() in main.c if it's missing
if grep -q ${MAINCALL} ${FILEPATH}
then
  :
else
  sed -i "/USER CODE BEGIN 5/s/$/\n  ${MAINCALL}/" ${FILEPATH}
fi

# Add include of main_cpp.h in main.c if it's missing
if grep -q "${INCLUDE}" ${FILEPATH}
then
  :
else
  sed -i "/USER CODE BEGIN Includes/s/$/\n${INCLUDE}/" ${FILEPATH}
fi
