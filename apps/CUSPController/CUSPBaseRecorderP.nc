/*                                                                      tab:2
 *
 * Copyright (c) 2000-2012 The Regents of the University of
 * Utah.  All rights reserved.
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
*/

#include "CUSPSerial.h"

module CUSPBaseRecorderP {
    uses {
        interface Boot;
        interface Leds;
        interface Packet as RadioPacket;
        interface AMPacket as RadioAMPacket;
        
        interface AMSend as UartSend; // serial rssi send
        interface AMSend as SerialStatusSend; // serial status send
        interface AMSend as SerialBaseStatusSend; //base serial status send
        interface Receive as SerialReceive;
        interface Packet as UartPacket;
        interface AMPacket as UartAMPacket;
        
        interface TimeSyncAMSend<TMilli,uint32_t> as CMDSend;
        interface Receive as CMDReceive; // cmd receive

        interface TimeSyncAMSend<TMilli,uint32_t> as BaseCMDSend;
        interface Receive as BaseStatusReceive; // base receive
        
        interface Receive as RssiLogReceive; // log receive
        
        //interface Receive as Snoop[uint8_t id];
        interface SplitControl as RadioControl;
        interface SplitControl as SerialControl;
        interface Timer<TMilli> as Timer0;
        interface LowPowerListening;
        interface GlobalTime<TMilli>;
        interface TimeSyncPacket<TMilli,uint32_t>;

        interface GlobalTimeSet;

#ifdef MOTE_DEBUG
        interface DiagMsg;
#endif                
    }
}
implementation {

    enum {
        INTER_PACKET_INTERVAL = 25
    };

		enum {
		  UART_QUEUE_LEN = 12,
		  RADIO_QUEUE_LEN = 12,
		};
      
    typedef nx_struct logentry_t {
        nx_uint8_t len;
        rssi_serial_msg_t msg;
    } logentry_t;

	  message_t  uartQueueBufs[UART_QUEUE_LEN];
	  message_t  * ONE_NOK uartQueue[UART_QUEUE_LEN];
	  uint8_t    uartIn, uartOut;
	  bool       uartBusy, uartFull;
	
	  message_t  radioQueueBufs[RADIO_QUEUE_LEN];
	  message_t  * ONE_NOK radioQueue[RADIO_QUEUE_LEN];
	  uint8_t    radioIn, radioOut;
	  bool       radioBusy, radioFull;

    bool m_busy = TRUE;
    logentry_t m_entry;

    message_t packet;
    message_t statuspacket;
    message_t rssipacket;
    message_t cmdpacket;

    bool statuslocked = FALSE;
    bool cmdlocked = FALSE;
    bool amsendlocked = FALSE;
    bool basestatuslocked = FALSE;

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
            m_busy = FALSE;
            // Set the local wakeup interval
            call LowPowerListening.setLocalWakeupInterval(LOCAL_WAKEUP_INTERVAL);


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

    event void SerialStatusSend.sendDone(message_t* msg, error_t err) {
        statuslocked = FALSE;
    }

    event void SerialBaseStatusSend.sendDone(message_t* msg, error_t err) {
        basestatuslocked = FALSE;
    }

    event void Timer0.fired() {
        rssi_serial_msg_t* rcm = (rssi_serial_msg_t*)call RadioPacket.getPayload(&packet, sizeof(rssi_serial_msg_t));
        call UartSend.send(0, &packet, m_entry.len);
    }

    void process_command(cmd_serial_msg_t* rcm) {

        switch(rcm->cmd)
        {
            case CMD_DOWNLOAD:
                break;

            case CMD_ERASE:
                break;

            case CMD_START_SENSE:
                break;

            case CMD_STOP_SENSE:
                break;

            case CMD_STATUS:
                break;
            
            case CMD_START_BLINK:
                call Leds.led1Toggle();
                break;
                                
            default:
                break;
        }

    }

    event void BaseCMDSend.sendDone(message_t* msg, error_t err) {
        #ifdef MOTE_DEBUG_MESSAGES
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("b_s:d");
                call DiagMsg.send();
            }
        #endif
        cmdlocked = FALSE;
    }

    event void CMDSend.sendDone(message_t* msg, error_t err) {
        #ifdef MOTE_DEBUG_MESSAGES
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("m_s:d");
                call DiagMsg.send();
            }
        #endif
        cmdlocked = FALSE;
    }

     event message_t * CMDReceive.receive(message_t *msg,
                                void *payload,
                                uint8_t len) {
        if (len != sizeof(serial_status_msg_t)) {
            call Leds.led0Toggle();
            return msg;
        }
        else {
	        if(!statuslocked) {
	            if (call SerialStatusSend.send(AM_BROADCAST_ADDR, msg, sizeof(serial_status_msg_t)) == SUCCESS) {
	                statuslocked = TRUE;
	            }
	        }
            
        }
        return msg;
      }

     event message_t * BaseStatusReceive.receive(message_t *msg,
                                void *payload,
                                uint8_t len) {
        if (len != sizeof(base_status_msg_t)) {
            call Leds.led0Toggle();
            return msg;
        }
        else {
            if(!basestatuslocked) {
                if (call SerialBaseStatusSend.send(AM_BROADCAST_ADDR, msg, sizeof(base_status_msg_t)) == SUCCESS) {
                    basestatuslocked = TRUE;
                }
            }
            
        }
        return msg;
      }
      
     event message_t * RssiLogReceive.receive(message_t *msg,
                                void *payload,
                                uint8_t len) {
	    message_t *ret = msg;
	
	    #ifdef MOTE_DEBUG_MESSAGES
	    
	        if (call DiagMsg.record())
	        {
	            call DiagMsg.str("rlr:1");
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
	    am_id_t id;
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
	    id = call RadioAMPacket.type(msg);
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
            
    event message_t* SerialReceive.receive(message_t* msg,
            void* payload, uint8_t len) {

        uint32_t time;
        time  = call GlobalTime.getLocalTime();

        if (len != sizeof(cmd_serial_msg_t)) {
            //call Leds.led1Toggle();
            return msg;
        }
        else {
            //cmd_serial_msg_t* rcm = (cmd_serial_msg_t*)call Packet.getPayload(&cmdpacket, sizeof(rssi_msg_t));
            cmd_serial_msg_t* rcm = (cmd_serial_msg_t*)payload;

            process_command(rcm);

	        #ifdef MOTE_DEBUG_MESSAGES
	        
	            if (call DiagMsg.record())
	            {
	                call DiagMsg.str("m_s:1");
	                call DiagMsg.send();
	            }
	        #endif

		       call LowPowerListening.setRemoteWakeupInterval(msg, REMOTE_WAKEUP_INTERVAL);
        
		       if (rcm->cmd == CMD_BASESTATUS) {
	//            if (call CMDSend.send(AM_BROADCAST_ADDR, msg, sizeof(cmd_serial_msg_t), time) == SUCCESS) {
	            if (call BaseCMDSend.send(rcm->dst, msg, sizeof(cmd_serial_msg_t), time) == SUCCESS) {
	            #ifdef MOTE_DEBUG_MESSAGES
	            
	                if (call DiagMsg.record())
	                {
	                    call DiagMsg.str("b_s:2");
	                    call DiagMsg.send();
	                }
	            #endif
	                cmdlocked = TRUE;
	            }
	            else {
	            #ifdef MOTE_DEBUG_MESSAGES
	            
	                if (call DiagMsg.record())
	                {
	                    call DiagMsg.str("b_s:e");
	                    call DiagMsg.send();
	                }
	            #endif
	                
	            }
		       }
		       else {
			
			//            if (call CMDSend.send(AM_BROADCAST_ADDR, msg, sizeof(cmd_serial_msg_t), time) == SUCCESS) {
				        if (call CMDSend.send(rcm->dst, msg, sizeof(cmd_serial_msg_t), time) == SUCCESS) {
				        #ifdef MOTE_DEBUG_MESSAGES
				        
				            if (call DiagMsg.record())
				            {
				                call DiagMsg.str("m_s:2");
				                call DiagMsg.send();
				            }
				        #endif
				            cmdlocked = TRUE;
				        }
				        else {
				        #ifdef MOTE_DEBUG_MESSAGES
				        
				            if (call DiagMsg.record())
				            {
				                call DiagMsg.str("m_s:e");
				                call DiagMsg.send();
				            }
				        #endif
				            
				        }
		       }
        }
        return msg;
    }
}
