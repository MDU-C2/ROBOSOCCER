// main_test.cc
//==============================================================================
// Author: Carl Larsson
// Creation date: 2024-09-23
// Last modified: 2024-10-27 by Carl Larsson
// Description: Main test file which initiates and runs all tests.
// License: See LICENSE file for license details.
//==============================================================================


// Other .h files
#include "gtest/gtest.h"


// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

//==============================================================================
