/** 
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
 * - Neither the name of the copyright holder nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Test code for LIS331DL Accelerometer.
 *
 * @author Thomas Schmid <thomas.schmid@utha.edu>
 */

//#include "printf.h"

module TestLis3dhC {
  uses {
    interface Boot;
    interface Leds;
    interface Lis3dh;
    interface Lis3dhAccel;
    interface Timer<TMilli> as Timer;
  }
}
implementation {

  event void Boot.booted() 
  {
    uint8_t chipId;
    call Leds.led0Off();
    call Leds.led1Off();
    call Leds.led2Off();

    call Lis3dhAccel.lowPower10Hz();

    if(call Lis3dhAccel.getChipId(&chipId) == SUCCESS) {
      printf("Accel ID: 0x%x\n", chipId);
    } else {
      printf("Chip ID failed\n");
    }
    printfflush();

    call Lis3dhAccel.enableDataReadyInt();
    call Lis3dhAccel.enableClickInt();

    call Timer.startOneShot(1024);
  }


  task void dataReadyTask()
  {
      accel_xyz_t a;
      if(call Lis3dhAccel.getAccel(&a) == SUCCESS)
      {
          printf("X: %d Y: %d Z: %d\n", a.x, a.y, a.z);
          //printf("abs: %d\n", a.x*a.x + a.y*a.y + a.z*a.z);
      } else {
          printf("Accel not working\n");
      }
      printfflush();
  }

  event void Timer.fired()
  {
      //printf("timer: ");
      //post dataReadyTask();
  }

  async event void Lis3dhAccel.dataReady()
  {
      call Leds.led0Toggle();
      post dataReadyTask();
  }

  async event void Lis3dhAccel.click()
  {
      call Leds.led1Toggle();
  }
}
