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

/**
 * Basic application that tests the SPI.
 *
 * @author Thomas Schmid
 **/

configuration TestSpiAppC
{
}
implementation
{
  components MainC, TestSpiC, LedsC;
#ifdef DEBUG_PRINTF
  components SerialStartC, PrintfC;
#endif

  TestSpiC -> MainC.Boot;
  TestSpiC.Leds -> LedsC;

  components PlatformSpiC as SpiC;
  TestSpiC.SpiByte -> SpiC;
  TestSpiC.SpiPacket -> SpiC;
  TestSpiC.SpiResource -> SpiC;
  TestSpiC.FastSpiByte -> SpiC;

  components HplMsp430GeneralIOC as IO;

  components new Msp430GpioC() as SELN_IO;
  SELN_IO.HplGeneralIO -> IO.Port27;
  TestSpiC.SELN -> SELN_IO.GeneralIO;
}

