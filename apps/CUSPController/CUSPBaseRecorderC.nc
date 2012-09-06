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
  components CC2420ActiveMessageC;
  
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
  App.CMDSend          -> TimeSyncMessageC.TimeSyncAMSendMilli[AM_CMD_MSG];
  
  App.BaseCMDSend      -> TimeSyncMessageC.TimeSyncAMSendMilli[AM_BASE_MSG];
  App.BaseStatusReceive -> TimeSyncMessageC.Receive[AM_BASE_STATUS_MSG];

  App.UartSend         -> SAM.AMSend[AM_SERIAL_STATUS_MSG];
  App.SerialBaseStatusSend -> SAM.AMSend[AM_BASE_STATUS_MSG];
  App.SerialReceive    -> SAM.Receive[AM_CMD_SERIAL_MSG];
  App.SerialControl    -> SAM;
  App.UartPacket       -> SAM;
  App.UartAMPacket     -> SAM;  
  //App.Snoop -> ActiveMessageC.Snoop;

  App.Timer0         -> Timer0;
  App.GlobalTime -> TimeSyncC;

  App.LowPowerListening -> TimeSyncMessageC;

  #ifdef MOTE_DEBUG
    components DiagMsgC;
    App.DiagMsg -> DiagMsgC;
  #endif

}
