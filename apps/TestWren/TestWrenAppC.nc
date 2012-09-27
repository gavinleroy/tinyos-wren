/*
 * Copyright (c) 2012 University of Utah.  
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the University of California nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *
 * @author Thomas Schmid
 */

#define NEW_PRINTF_SEMANTICS
#include "printf.h"

#include "RadioReflector.h"
#include "StorageVolumes.h"

configuration TestWrenAppC{}
implementation {
  components MainC, TestWrenP as App, LedsC, NoLedsC;

  MainC.Boot <- App;
  App.Leds -> LedsC;

  components PrintfC, SerialStartC;


  /* Power Test */
  components BQ25010C;
  App.BQ25010 -> BQ25010C;
  BQ25010C.Leds -> NoLedsC;

  /* Radio Test */
  components new AMSenderC(AM_RADIO_REFLECTOR_MSG);
  components new AMReceiverC(AM_RADIO_REFLECTOR_MSG);
  components ActiveMessageC;
  components new TimerMilliC() as ReflectionTimer;

  App.Receive -> AMReceiverC;
  App.AMSend -> AMSenderC;
  App.AMControl -> ActiveMessageC;
  App.Packet -> AMSenderC;
  App.ReflectionTimer -> ReflectionTimer;

  /* Accelerometer */
  components Lis3dhC;
  App.Lis3dh -> Lis3dhC;
  App.Lis3dhAccel -> Lis3dhC;

  /* RTC */
  components Pcf2127aC, BusyWait32khzC;
  App.Pcf2127a -> Pcf2127aC;
  App.Pcf2127aRtc -> Pcf2127aC;
  App.BusyWait -> BusyWait32khzC;

  /* Flash Storage */
  components new LogStorageC(VOLUME_LOGTEST, TRUE);
  App.LogRead -> LogStorageC;
  App.LogWrite -> LogStorageC;
}

