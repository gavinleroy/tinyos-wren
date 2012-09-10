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
  components new TimerMilliC() as BatteryTimer;

  components SerialActiveMessageC as SAM;
  components CC2420ActiveMessageC;
  
  components TimeSyncC;
  components TimeSyncMessageC;

  MainC.SoftwareInit -> TimeSyncC;
  TimeSyncC.Boot -> MainC;

  App.GlobalTimeSet -> TimeSyncC;

  App.Boot -> MainC;
  App.Leds -> LedsC;

  App.Packet           -> TimeSyncMessageC;
  App.AMPacket         -> TimeSyncMessageC;
  App.AMControl        -> ActiveMessageC;
  App.Receive          -> TimeSyncMessageC.Receive[AM_RSSI_MSG];
  App.RssiSend         -> TimeSyncMessageC.TimeSyncAMSendMilli[AM_RSSI_MSG];
  App.TimeSyncPacket   -> TimeSyncMessageC;

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
  App.BatteryTimer   -> BatteryTimer;

  components RandomC;
  App.Random -> RandomC;
  MainC.SoftwareInit -> RandomC.Init;


  App -> CC2420ActiveMessageC.CC2420Packet;
  App.GlobalTime -> TimeSyncC;

  components new MoteBatteryLevelC() as BatteryLevel; 
  App.BatteryRead -> BatteryLevel; 
  App.BatteryReadNow -> BatteryLevel; 
  App.BatteryReadStream -> BatteryLevel; 
  App.BatteryResource -> BatteryLevel; 

  App.LowPowerListening -> CC2420ActiveMessageC;

}
