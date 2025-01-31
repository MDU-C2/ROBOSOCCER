/* main_cpp.cc
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-11-11
 * Last modified: 2024-11-22 by Emil Åberg
 * Description: Function that is called within main.c and serves as the main
 * entry point of the  user defined c++ code.
 * License: See LICENSE file for license details.
 *==============================================================================
 */

/* Related .h files */
#include "../../Src/user-code/main_cpp.h"

/* Project .h files */
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"

extern "C"
{

/* Main entry point for the user defined C++ code */
void MainCpp()
{

  for(;;)
  {
    osDelay(10);
  }
}

}
