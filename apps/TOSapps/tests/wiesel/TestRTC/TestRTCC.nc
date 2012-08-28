/* 
 */

/*
 *
 */

//#include "printf.h"

module TestRTCC {
  uses {
    interface Boot;
    interface Leds;
    interface Pcf2127a;
    interface Pcf2127aRtc;
    interface GpioInterrupt      as Int;
    interface HplMsp430GeneralIO as IntIO;
  }
}
implementation {

  event void Boot.booted() 
  {
    call Leds.led0Off();
    call Leds.led1Off();
    call Leds.led2Off();

    call IntIO.setResistor(MSP430_PORT_RESISTOR_PULLUP);
    call IntIO.makeInput();

    call Int.enableFallingEdge();

    if(call Pcf2127aRtc.setSecondInterrupt() != SUCCESS)
        call Leds.led0On();
    else
        call Leds.led1On();
    call Pcf2127aRtc.setPulsedInterrupt();
  }


  async event void Int.fired()
  {
      call Leds.led2Toggle();
  }
}
