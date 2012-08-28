/* 
 */

/**
 */

//#define NEW_PRINTF_SEMANTICS
//#include "printf.h"

configuration TestRTCAppC {
}
implementation {
  components MainC, TestRTCC, LedsC, NoLedsC;

  TestRTCC.Boot -> MainC;
  TestRTCC.Leds -> LedsC;

  components HplMsp430GeneralIOC;
  TestRTCC.IntIO -> HplMsp430GeneralIOC.Port14;

  components HplMsp430InterruptC;
  components new Msp430InterruptC() as INTC;
  INTC.HplInterrupt -> HplMsp430InterruptC.Port14;
  TestRTCC.Int -> INTC.Interrupt;

  components Pcf2127aC;
  TestRTCC.Pcf2127a -> Pcf2127aC;
  TestRTCC.Pcf2127aRtc -> Pcf2127aC;

//  components PrintfC, SerialStartC;
}
