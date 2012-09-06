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

  components SerialActiveMessageC as SAM;
  components RF233ActiveMessageC;
  
  components TimeSyncC;
  components RF233TimeSyncMessageC;

  MainC.SoftwareInit -> TimeSyncC;
  TimeSyncC.Boot -> MainC;

  App.GlobalTimeSet -> TimeSyncC;

  App.Boot -> MainC;
  App.Leds -> LedsC;

  App.RadioPacket      -> RF233TimeSyncMessageC;
  App.RadioAMPacket    -> RF233TimeSyncMessageC;
  App.RadioControl     -> ActiveMessageC;
  App.TimeSyncPacket   -> RF233TimeSyncMessageC;

  App.CMDReceive       -> RF233TimeSyncMessageC.Receive[AM_CMD_MSG];
  App.RssiLogReceive   -> RF233TimeSyncMessageC.Receive[AM_RSSI_SERIAL_MSG];
  App.CMDSend          -> RF233TimeSyncMessageC.TimeSyncAMSendMilli[AM_CMD_MSG];
  
  App.BaseCMDReceive   -> RF233TimeSyncMessageC.Receive[AM_BASE_MSG];
  App.BaseStatusSend   -> RF233TimeSyncMessageC.TimeSyncAMSendMilli[AM_BASE_STATUS_MSG];

  App.UartSend         -> SAM.AMSend[AM_RSSI_SERIAL_MSG];
  App.SerialStatusSend -> SAM.AMSend[AM_SERIAL_STATUS_MSG];
  App.SerialReceive    -> SAM.Receive[AM_CMD_SERIAL_MSG];
  App.SerialControl    -> SAM;
  App.UartPacket       -> SAM;
  App.UartAMPacket     -> SAM;  
  //App.Snoop -> ActiveMessageC.Snoop;

  App.Timer0         -> Timer0;
  App.GlobalTime -> TimeSyncC;

  App.LowPowerListening -> RF233TimeSyncMessageC;

  #ifdef MOTE_DEBUG
    components DiagMsgC;
    App.DiagMsg -> DiagMsgC;
  #endif

}
