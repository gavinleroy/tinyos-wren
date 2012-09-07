/*                                                                     tab:2
 *
 *
 */

/**
 *
 * @author Thomas Schmid
 * @date   Apr 19, 2012
 */
#include <Timer.h>
#include "StorageVolumes.h"
#include "CUSPSerial.h"

configuration CUSPBaseRecorderC {
}
implementation {
  components MainC;
  components LedsC;
  components CUSPBaseRecorderP as App;
  components ActiveMessageC;
  components new TimerMilliC() as Timer0;
  components new TimerMilliC() as RandomTimer2;

  components SerialActiveMessageC as SAM;
  components CC2420ActiveMessageC;
  components CC2420ControlC;
  
  components TimeSyncC;
  components TimeSyncMessageC;
  
  MainC.SoftwareInit -> TimeSyncC;
  TimeSyncC.Boot -> MainC;

  App.GlobalTimeSet -> TimeSyncC;

  App.Boot -> MainC;
  App.Leds -> LedsC;

  App.RadioPacket      -> TimeSyncMessageC;
  App.RadioAMPacket    -> TimeSyncMessageC;
  App.RadioControl     -> ActiveMessageC;
  App.TimeSyncPacket   -> TimeSyncMessageC;

  App.CMDReceive       -> TimeSyncMessageC.Receive[AM_CMD_MSG];
  App.RssiLogReceive   -> TimeSyncMessageC.Receive[AM_RSSI_SERIAL_MSG];
  App.CMDSend          -> TimeSyncMessageC.TimeSyncAMSendMilli[AM_CMD_MSG];
  
  App.BaseCMDReceive   -> TimeSyncMessageC.Receive[AM_BASE_MSG];
  App.BaseStatusSend   -> TimeSyncMessageC.TimeSyncAMSendMilli[AM_BASE_STATUS_MSG];

  App.UartSend         -> SAM.AMSend[AM_RSSI_SERIAL_MSG];
  App.SerialControl    -> SAM;
  App.UartPacket       -> SAM;
  App.UartAMPacket     -> SAM;  
  //App.Snoop -> ActiveMessageC.Snoop;

  App.Timer0         -> Timer0;
  App.RandomTimer2   -> RandomTimer2;

  components RandomC;
  App.Random -> RandomC;
  MainC.SoftwareInit -> RandomC.Init;
    
  App.GlobalTime -> TimeSyncC;

  App.LowPowerListening -> TimeSyncMessageC;

  App -> CC2420ActiveMessageC.CC2420Packet;
  App -> CC2420ControlC.CC2420Config;

  #ifdef MOTE_DEBUG
    components DiagMsgC;
    App.DiagMsg -> DiagMsgC;
  #endif

}
