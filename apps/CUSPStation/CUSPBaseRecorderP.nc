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
 * @author Kyeong T. Min
 * @date   Oct 8, 2012
 */

#include "CUSPSerial.h"

module CUSPBaseRecorderP {
    uses {
        interface Boot;
        interface Leds;
        interface Packet as RadioPacket;
        interface AMPacket as RadioAMPacket;
        
        interface AMSend as UartSend; // serial rssi send
        interface Packet as UartPacket;
        interface AMPacket as UartAMPacket;
        
        interface Receive as RadioReceive; // log receive
        
        //interface Receive as Snoop[uint8_t id];
        interface SplitControl as RadioControl;
        interface SplitControl as SerialControl;

#ifdef MOTE_DEBUG
        interface DiagMsg;
#endif                
    }
}
implementation {

	enum {
	  UART_QUEUE_LEN = 24,
	  RADIO_QUEUE_LEN = 12,
	};
      
  message_t  uartQueueBufs[UART_QUEUE_LEN];
  message_t  * ONE_NOK uartQueue[UART_QUEUE_LEN];
  uint8_t    uartIn, uartOut;
  bool       uartBusy, uartFull;

  message_t  radioQueueBufs[RADIO_QUEUE_LEN];
  message_t  * ONE_NOK radioQueue[RADIO_QUEUE_LEN];
  uint8_t    radioIn, radioOut;
  bool       radioBusy, radioFull;

  task void uartSendTask();

    void dropBlink() {
	   call Leds.led2Toggle();
	}
	
	void failBlink() {
	   call Leds.led2Toggle();
	}
    
    event void Boot.booted() {
	    uint8_t i;
	
	    for (i = 0; i < UART_QUEUE_LEN; i++)
	      uartQueue[i] = &uartQueueBufs[i];
	    uartIn = uartOut = 0;
	    uartBusy = FALSE;
	    uartFull = TRUE;
	
	    for (i = 0; i < RADIO_QUEUE_LEN; i++)
	      radioQueue[i] = &radioQueueBufs[i];
	    radioIn = radioOut = 0;
	    radioBusy = FALSE;
	    radioFull = TRUE;
	
	    if (call RadioControl.start() == EALREADY)
	      radioFull = FALSE;
	    if (call SerialControl.start() == EALREADY)
	      uartFull = FALSE;
    }

    event void RadioControl.startDone(error_t err) {
        if (err == SUCCESS) {
	      radioFull = FALSE;
        }
        else {
            call RadioControl.start();
        }
    }


    event void RadioControl.stopDone(error_t err) {
    }

    event void SerialControl.startDone(error_t err) {
        if (err == SUCCESS) {
			     uartFull = FALSE;
			  }
        else {
            call SerialControl.start();
        }
    }

    event void SerialControl.stopDone(error_t err) {
    }

    event void UartSend.sendDone(message_t* msg, error_t error) {
	    if (error != SUCCESS)
	      failBlink();
	    else
	      atomic
	    if (msg == uartQueue[uartOut])
	      {
	        if (++uartOut >= UART_QUEUE_LEN)
	          uartOut = 0;
	        if (uartFull)
	          uartFull = FALSE;
	      }
	    post uartSendTask();
    }

     event message_t * RadioReceive.receive(message_t *msg,
                                void *payload,
                                uint8_t len) {
	    message_t *ret = msg;
        rssi_serial_msg_t* rssim = (rssi_serial_msg_t*)payload;
	
	    #ifdef MOTE_DEBUG_MESSAGES
	    
	        if (call DiagMsg.record())
	        {
	            call DiagMsg.str("rlr:1");
                call DiagMsg.uint16(rssim->dst);
                call DiagMsg.uint32(rssim->size);
	            call DiagMsg.send();
	        }
	    #endif
		    atomic {
		      if (!uartFull)
		    {
		      ret = uartQueue[uartIn];
		      uartQueue[uartIn] = msg;
		
		      uartIn = (uartIn + 1) % UART_QUEUE_LEN;
		    
		      if (uartIn == uartOut)
		        uartFull = TRUE;
		
		      if (!uartBusy)
		        {
		          post uartSendTask();
		          uartBusy = TRUE;
		        }
		    }
		      else
		    dropBlink();
		    }
		    
		    return ret;
	
	  }

	  uint8_t tmpLen;

	  task void uartSendTask() {
	    uint8_t len;
	    //am_id_t id;
	    am_addr_t addr, src;
	    message_t* msg;
	    am_group_t grp;
	    atomic
	      if (uartIn == uartOut && !uartFull)
	    {
	      uartBusy = FALSE;
	      return;
	    }
	
	    msg = uartQueue[uartOut];
	    tmpLen = len = call RadioPacket.payloadLength(msg);
	    //id = call RadioAMPacket.type(msg);
	    addr = call RadioAMPacket.destination(msg);
	    src = call RadioAMPacket.source(msg);
	    grp = call RadioAMPacket.group(msg);
	    call UartPacket.clear(msg);
	    call UartAMPacket.setSource(msg, src);
	    call UartAMPacket.setGroup(msg, grp);
	
	    if (call UartSend.send(addr, uartQueue[uartOut], len) == SUCCESS)
	      call Leds.led1Toggle();
	    else
	      {
	    failBlink();
	    post uartSendTask();
	      }
	  }
}
