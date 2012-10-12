/*                                                                      tab:2
 *
 * Copyright (c) 2000-2007 The Regents of the University of
 * California.  All rights reserved.
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
 * - Neither the name of the copyright holders nor the names of
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
 *
 */

/**
 *
 * @author Kyeong T. Min
 * @date   Oct 8, 2012
 */

#include "CUSPSerial.h"

module CUSPBaseRecorderP {
    uses {
        interface Boot;
        interface Leds;
        interface Packet;
        interface AMPacket;

        interface TimeSyncAMSend<TMilli,uint32_t> as RadioSend;
        interface Receive as RadioReceive; // rssi receive
        interface SplitControl as AMControl;

        interface LowPowerListening;
        interface RadioChannel;
        interface PacketField<uint8_t> as PacketTransmitPower;
        interface GlobalTime<TMilli>;

#ifdef MOTE_DEBUG
        interface DiagMsg;
#endif        
    }
}
implementation {
  enum {
    UART_QUEUE_LEN = 12,
    RADIO_QUEUE_LEN = 12,
  };

  message_t  radioQueueBufs[RADIO_QUEUE_LEN];
  message_t  * ONE_NOK radioQueue[RADIO_QUEUE_LEN];
  uint8_t    radioIn, radioOut;
  bool       radioBusy, radioFull;

    bool m_busy = TRUE;

    message_t packet;
    message_t rssipacket;

    bool sensing = FALSE;
    bool rssilocked = FALSE;
    bool statuslocked = FALSE;
    uint16_t counter = 0;

    enum {
        DEF = 0,
        SECOND = 1,
    };

    uint8_t channel = DEF;
  task void radioSendTask();

  void dropBlink() {
    call Leds.led2Toggle();
  }

  void failBlink() {
    call Leds.led2Toggle();
  }

    event void Boot.booted() {
	    uint8_t i;
	
	    for (i = 0; i < RADIO_QUEUE_LEN; i++)
	      radioQueue[i] = &radioQueueBufs[i];
	    radioIn = radioOut = 0;
	    radioBusy = FALSE;
	    radioFull = TRUE;
	    if (call AMControl.start() == EALREADY)
	      radioFull = FALSE;
    }

    event void AMControl.startDone(error_t err) {
        if (err == SUCCESS) {
            m_busy = FALSE;
            call LowPowerListening.setLocalWakeupInterval(LOCAL_WAKEUP_INTERVAL);
        }
        else {
            call AMControl.start();
        }
    }

    event void AMControl.stopDone(error_t err) {
    }


  event message_t *RadioReceive.receive(message_t *msg,
                            void *payload,
                            uint8_t len) {
    message_t *ret = msg;
    error_t err;
    
    atomic
      if (!radioFull)
	{
	  ret = radioQueue[radioIn];
	  radioQueue[radioIn] = msg;
	  if (++radioIn >= RADIO_QUEUE_LEN)
	    radioIn = 0;
	  if (radioIn == radioOut)
	    radioFull = TRUE;

	}
      else {
		dropBlink();
       if (!radioBusy)
	    {
            radioBusy = TRUE;
	        err = call RadioChannel.setChannel(RF233_SECOND_CHANNEL);
	        channel = SECOND;
	
	        if (err == SUCCESS || err == EALREADY) {
	        }
	        else {
	        	radioBusy = FALSE;
	        	channel = DEF;
	        }	        
	    }
    }
    
    return ret;
  }

    event void RadioChannel.setChannelDone() {
        switch(channel){
            case DEF:

                break;

            case SECOND:
                post radioSendTask();
                break;
        }
    }

  task void radioSendTask() {
    uint8_t len;
    //am_id_t id;
    am_addr_t addr,source;
    message_t* msg;
    uint32_t time;
    time  = call GlobalTime.getLocalTime();
    
    atomic
      if (radioIn == radioOut && !radioFull)
	{
		radioBusy = FALSE;

		// Change back to the listening mode after all rssis are sent. 
		// There could be some rssi drop offs during this time.
	    call RadioChannel.setChannel(RF233_DEF_CHANNEL);
	    channel = DEF;
	
	  return;
	}

    msg = radioQueue[radioOut];
    len = call Packet.payloadLength(msg);
    addr = call AMPacket.destination(msg);
    source = call AMPacket.source(msg);
    //id = call AMPacket.type(msg);

    call Packet.clear(msg);
    call AMPacket.setSource(msg, source);

    call LowPowerListening.setRemoteWakeupInterval(msg, 3);
    
    if (call RadioSend.send(addr, msg, len, time) == SUCCESS)
      call Leds.led0Toggle();
    else
	  {
		failBlink();
		post radioSendTask();
	  }
  }

  event void RadioSend.sendDone(message_t* msg, error_t error) {
    if (error != SUCCESS)
      failBlink();
    else
      atomic
	if (msg == radioQueue[radioOut])
	  {
	    if (++radioOut >= RADIO_QUEUE_LEN)
	      radioOut = 0;
	    if (radioFull)
	      radioFull = FALSE;
	  }

    post radioSendTask();
  }
}
