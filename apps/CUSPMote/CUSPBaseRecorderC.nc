/*
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
 * - Neither the name of the University of California nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
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
  components NoLedsC;
  
  components CUSPBaseRecorderP as App;
  components ActiveMessageC;
  components new LogStorageC(VOLUME_LOGTEST, TRUE);
  components new ConfigStorageC(VOLUME_CONFIGTEST);
  components new TimerMilliC() as LogSendTimer;
  components new TimerMilliC() as SensingTimer;
  components new TimerMilliC() as HeartbeatTimer;
  components new TimerMilliC() as LedOffTimer;
  components new TimerMilliC() as RandomTimer;
  components new TimerMilliC() as StatusRandomTimer;
  components new TimerMilliC() as BatteryTimer;
  components new TimerMilliC() as DownloadTimer;
  components new TimerMilliC() as WRENStatusTimer;
  components new TimerMilliC() as ConnectionTimer;
  components new TimerMilliC() as AckTimer;
  components new TimerMilliC() as TransmitTimer;
  components new TimerMilliC() as ReTransmitTimer;

  

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

  App.ConnectionReceive       -> RF233TimeSyncMessageC.Receive[AM_CONNECTION_MSG];
  App.ConnectionSend          -> RF233TimeSyncMessageC.TimeSyncAMSendMilli[AM_CONNECTION_MSG];

  App.AckReceive       -> RF233TimeSyncMessageC.Receive[AM_SWP_MSG];

  App.AMSend           -> SAM.AMSend[AM_RSSI_SERIAL_MSG];
  App.SerialStatusSend -> SAM.AMSend[AM_SERIAL_STATUS_MSG];
  App.SerialReceive    -> SAM.Receive[AM_CMD_SERIAL_MSG];
  App.SerialControl    -> SAM;
  //App.Snoop -> ActiveMessageC.Snoop;
  App.LogRead -> LogStorageC;
  App.LogWrite -> LogStorageC;
 
  App.Config -> ConfigStorageC.ConfigStorage;
  App.Mount  -> ConfigStorageC.Mount;

  App.LogSendTimer   -> LogSendTimer;
  App.SensingTimer   -> SensingTimer;
  App.HeartbeatTimer -> HeartbeatTimer;
  App.LedOffTimer    -> LedOffTimer;
  App.RandomTimer    -> RandomTimer;
  App.StatusRandomTimer    -> StatusRandomTimer;
  App.BatteryTimer   -> BatteryTimer;
  App.DownloadTimer  -> DownloadTimer;
  App.WRENStatusTimer -> WRENStatusTimer;
  App.ConnectionTimer -> ConnectionTimer;
  App.AckTimer -> AckTimer;
  App.TransmitTimer -> TransmitTimer;
  App.ReTransmitTimer -> ReTransmitTimer;
  
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

  #ifdef DISSEMINATION_ON
  components DisseminationC;
  App.DisseminationControl -> DisseminationC;

  components new DisseminatorC(uint16_t, 0x1234) as CommandObjectC;
  App.CommandValue -> CommandObjectC;
  App.CommandUpdate -> CommandObjectC;
  #endif
  
  components BlinkC;
  App.Blink -> BlinkC;
  
  components new TimerMilliC() as TimeoutTimer;
  App.TimeoutTimer -> TimeoutTimer;
  
  //components new VirtualizeTimerC(TMilli, WINDOWSIZE) as VirtualizeTimer;
  //App.VirtualizeTimer -> VirtualizeTimer;

}
