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
  components new LogStorageC(VOLUME_LOGTEST, TRUE);
  components new ConfigStorageC(VOLUME_CONFIGTEST);
  components new TimerMilliC() as Timer0;
  components new TimerMilliC() as SensingTimer;
  components new TimerMilliC() as HeartbeatTimer;
  components new TimerMilliC() as LedOffTimer;
  components new TimerMilliC() as RandomTimer;
  components new TimerMilliC() as StatusRandomTimer;
  components new TimerMilliC() as BatteryTimer;
  components new TimerMilliC() as DownloadTimer;
  components new TimerMilliC() as WRENRandomTimer;

  components SerialActiveMessageC as SAM;
  
  components TimeSyncC;
  
  components RF233ActiveMessageC;
  components RF233TimeSyncMessageC;
  /* Power Test */
  components BQ25010C;
  App.BQ25010 -> BQ25010C;
  BQ25010C.Leds -> LedsC;

  
  MainC.SoftwareInit -> TimeSyncC;
  TimeSyncC.Boot -> MainC;

  App.GlobalTimeSet -> TimeSyncC;

  App.Boot -> MainC;
  App.Leds -> LedsC;

  App.Packet           -> RF233TimeSyncMessageC;
  App.AMPacket         -> RF233TimeSyncMessageC;
  App.AMControl        -> ActiveMessageC;
  App.Receive          -> RF233TimeSyncMessageC.Receive[AM_RSSI_MSG];
  App.RssiSend         -> RF233TimeSyncMessageC.TimeSyncAMSendMilli[AM_RSSI_MSG];
  App.TimeSyncPacket   -> RF233TimeSyncMessageC;

  App.CMDReceive       -> RF233TimeSyncMessageC.Receive[AM_CMD_MSG];
  App.CMDSend          -> RF233TimeSyncMessageC.TimeSyncAMSendMilli[AM_CMD_MSG];
  App.WRENSend         -> RF233TimeSyncMessageC.TimeSyncAMSendMilli[AM_WREN_STATUS_MSG];
  App.RssiLogSend      -> RF233TimeSyncMessageC.TimeSyncAMSendMilli[AM_RSSI_SERIAL_MSG];

  App.AMSend           -> SAM.AMSend[AM_RSSI_SERIAL_MSG];
  App.SerialStatusSend -> SAM.AMSend[AM_SERIAL_STATUS_MSG];
  App.SerialReceive    -> SAM.Receive[AM_CMD_SERIAL_MSG];
  App.SerialControl    -> SAM;
  //App.Snoop -> ActiveMessageC.Snoop;
  App.LogRead -> LogStorageC;
  App.LogWrite -> LogStorageC;
 
  App.Config -> ConfigStorageC.ConfigStorage;
  App.Mount  -> ConfigStorageC.Mount;

  App.Timer0         -> Timer0;
  App.SensingTimer   -> SensingTimer;
  App.HeartbeatTimer -> HeartbeatTimer;
  App.LedOffTimer    -> LedOffTimer;
  App.RandomTimer    -> RandomTimer;
  App.StatusRandomTimer    -> StatusRandomTimer;
  App.BatteryTimer   -> BatteryTimer;
  App.DownloadTimer  -> DownloadTimer;
  App.WRENRandomTimer -> WRENRandomTimer;
  
  components RandomC;
  App.Random -> RandomC;
  MainC.SoftwareInit -> RandomC.Init;


  App.PacketRSSI -> RF233ActiveMessageC.PacketRSSI;
  App.GlobalTime -> TimeSyncC;

  components new MoteBatteryLevelC() as BatteryLevel; 
  App.BatteryRead -> BatteryLevel; 
  App.BatteryReadNow -> BatteryLevel; 
  App.BatteryReadStream -> BatteryLevel; 
  App.BatteryResource -> BatteryLevel; 

  App.LowPowerListening -> RF233TimeSyncMessageC;

  App.RadioChannel -> ActiveMessageC;
  
  components Pcf2127aC;
  App.Pcf2127a -> Pcf2127aC;
  App.Pcf2127aRtc -> Pcf2127aC;

  App.PacketTransmitPower -> RF233ActiveMessageC.PacketTransmitPower;
    
  #ifdef MOTE_DEBUG
    components DiagMsgC;
    App.DiagMsg -> DiagMsgC;
  #endif

}
