/****************************************************************************
 *
 *   Copyright (C) 2023 Marcelo Jacinto. All rights reserved.
 *   Author: Marcelo Jacinto <marcelo.jacinto@tecnico.ulisboa.pt>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name Pegasus nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#ifndef AUTOPILOT_MODES__VISIBILITY_CONTROL_H_
#define AUTOPILOT_MODES__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define AUTOPILOT_MODES_EXPORT __attribute__ ((dllexport))
    #define AUTOPILOT_MODES_IMPORT __attribute__ ((dllimport))
  #else
    #define AUTOPILOT_MODES_EXPORT __declspec(dllexport)
    #define AUTOPILOT_MODES_IMPORT __declspec(dllimport)
  #endif
  #ifdef AUTOPILOT_MODES_BUILDING_LIBRARY
    #define AUTOPILOT_MODES_PUBLIC AUTOPILOT_MODES_EXPORT
  #else
    #define AUTOPILOT_MODES_PUBLIC AUTOPILOT_MODES_IMPORT
  #endif
  #define AUTOPILOT_MODES_PUBLIC_TYPE AUTOPILOT_MODES_PUBLIC
  #define AUTOPILOT_MODES_LOCAL
#else
  #define AUTOPILOT_MODES_EXPORT __attribute__ ((visibility("default")))
  #define AUTOPILOT_MODES_IMPORT
  #if __GNUC__ >= 4
    #define AUTOPILOT_MODES_PUBLIC __attribute__ ((visibility("default")))
    #define AUTOPILOT_MODES_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define AUTOPILOT_MODES_PUBLIC
    #define AUTOPILOT_MODES_LOCAL
  #endif
  #define AUTOPILOT_MODES_PUBLIC_TYPE
#endif

#endif  // AUTOPILOT_MODES__VISIBILITY_CONTROL_H_
