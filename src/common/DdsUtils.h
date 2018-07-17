/*
 * Copyright 2018 Real-Time Innovations, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <csignal>
#include <iostream>

/**
 * This global variable keeps the main thread sleeping until a signal changes
 * its value to false.
 */
bool exit_application = false;
/**
 * This method changes sets exit_service to true. It is triggered by
 * @param signal Unused int parameters that identifies the captured signal.
 */
inline void signal_handler(int signal)
{
    // Log registration of Web Server
    std::cout << "Received " << std::to_string(signal) << " signal" << std::endl;
    exit_application = true;
}

/**
 * This method registers the signal_handler() with all the exit signals, which
 * triggers a call to the method if any event happens.
 */
inline void setup_signal_handler()
{
#ifndef RTI_WIN32
    signal(SIGHUP,  signal_handler); //Terminal is closed
    signal(SIGQUIT, signal_handler); //Quit
#endif
    signal(SIGTERM, signal_handler); //Terminate
    signal(SIGINT,  signal_handler); //Interrupt
    signal(SIGABRT, signal_handler); //Abort
}

#endif // UTILS_H_
