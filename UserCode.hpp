#pragma once

#include "MainLoopTypes.hpp"

/* Main loop function
 *
 * This is where the action happens. This is run at 500Hz, i.e. every 2ms.
 * The input is a list of sensor measurements, etc., and the outputs are the
 * desired motor commands and the states of the LEDs.
 */
MainLoopOutput MainLoop(MainLoopInput const &in);

/* A debugging function, that we can call from the shell, to print information
 * about the system state.
 */
void PrintStatus();

