/*                                                                     tab:2
 *
 *
 */

/**
 *
 * @author Kyeong T. Min
 * @date   Oct 8, 2012
 */
#include <Timer.h>
#include "CUSPSerial.h"

configuration CUSPBaseRecorderC {
}
implementation {
  components MainC;
  components LedsC;
  components CUSPBaseRecorderP as App;
  components ActiveMessageC;
  components RF233ActiveMessageC;
  components RF233TimeSyncMessageC;
  components TimeSyncC;

  App.Boot -> MainC;
  App.Leds -> LedsC;

  App.Packet           -> RF233TimeSyncMessageC;
  App.AMPacket         -> RF233TimeSyncMessageC;
  App.AMControl        -> ActiveMessageC;
  App.RadioReceive          -> RF233TimeSyncMessageC.Receive[AM_RSSI_MSG];
  App.RadioSend         -> RF233TimeSyncMessageC.TimeSyncAMSendMilli[AM_RSSI_MSG];

  App.LowPowerListening -> RF233TimeSyncMessageC;
  App.RadioChannel -> ActiveMessageC;
  App.PacketTransmitPower -> RF233ActiveMessageC.PacketTransmitPower;
  App.GlobalTime -> TimeSyncC;
    
  #ifdef MOTE_DEBUG
    components DiagMsgC;
    App.DiagMsg -> DiagMsgC;
  #endif
}
