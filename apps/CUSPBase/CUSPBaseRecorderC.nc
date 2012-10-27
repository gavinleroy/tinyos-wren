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
  components CUSPBaseRecorderP as App;
  components ActiveMessageC;
  components new TimerMilliC() as Timer0;
  components new TimerMilliC() as RandomTimer2;
  components new TimerMilliC() as AckTimer;
  components new TimerMilliC() as ConnectionTimer;

  components SerialActiveMessageC as SAM;
  components new SerialAMSenderC(AM_CLOSE_MSG);
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
  
  App.ConnectionReceive       -> TimeSyncMessageC.Receive[AM_CONNECTION_MSG];
  App.ConnectionSend          -> TimeSyncMessageC.TimeSyncAMSendMilli[AM_CONNECTION_MSG];
  
  App.BaseCMDReceive   -> TimeSyncMessageC.Receive[AM_BASE_MSG];
  App.BaseStatusSend   -> TimeSyncMessageC.TimeSyncAMSendMilli[AM_BASE_STATUS_MSG];

  App.WRENReceive      -> TimeSyncMessageC.Receive[AM_WREN_STATUS_MSG];
  App.WRENSerialSend   -> SAM.AMSend[AM_WREN_STATUS_MSG];

  App.AckSend          -> TimeSyncMessageC.TimeSyncAMSendMilli[AM_SWP_MSG];

  App.UartSend         -> SAM.AMSend[AM_RSSI_SERIAL_MSG];
  App.SerialControl    -> SAM;
  App.SerialBaseStatusSend -> SAM.AMSend[AM_BASE_STATUS_MSG];
  App.SerialReceive    -> SAM.Receive[AM_CMD_SERIAL_MSG];
//  App.SerialCloseSend  -> SAM.AMSend[AM_CLOSE_MSG];
  App.SerialCloseSend  -> SerialAMSenderC;
  App.UartPacket       -> SAM;
  App.UartAMPacket     -> SAM;  
  //App.Snoop -> ActiveMessageC.Snoop;

  App.Timer0         -> Timer0;
  App.RandomTimer2   -> RandomTimer2;
  App.AckTimer   -> AckTimer;
  App.ConnectionTimer   -> ConnectionTimer;

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
