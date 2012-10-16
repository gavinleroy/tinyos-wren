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
*/

#include "CUSPSerial.h"

module CUSPBaseRecorderP {
    uses {
        interface Boot;
        interface Leds;
        interface Packet as RadioPacket;
        interface AMPacket as RadioAMPacket;
        
        interface AMSend as UartSend; // serial rssi send
        interface AMSend as WRENSend; // serial rssi send
        interface AMSend as SerialBaseStatusSend; //base serial status send
        interface AMSend as SerialConnectionSend; //base serial status send
        interface Receive as SerialReceive;
        interface Packet as UartPacket;
        interface AMPacket as UartAMPacket;
        
        interface TimeSyncAMSend<TMilli,uint32_t> as CMDSend;
        interface Receive as CMDReceive; // cmd receive
        interface Receive as WRENReceive; // wren receive
        interface Receive as ConnectionReceive; // cmd receive

        interface TimeSyncAMSend<TMilli,uint32_t> as BaseCMDSend;
        interface Receive as BaseStatusReceive; // base receive
        
        //interface Receive as Snoop[uint8_t id];
        interface SplitControl as RadioControl;
        interface SplitControl as SerialControl;

        interface Timer<TMilli> as Timer0;
        interface Timer<TMilli> as RandomTimer2;
        interface Random;

        interface LowPowerListening;
        interface GlobalTime<TMilli>;
        interface TimeSyncPacket<TMilli,uint32_t>;

        interface GlobalTimeSet;

        interface CC2420Packet;
        interface CC2420Config;

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
          WREN_QUEUE_LEN = 12,
		  RADIO_QUEUE_LEN = 12,
		};

    enum {
        DEF = 0,
        SECOND = 1,
    };

    uint8_t channel = DEF;
          
    typedef nx_struct logentry_t {
        nx_uint8_t len;
        rssi_serial_msg_t msg;
    } logentry_t;

	  message_t  uartQueueBufs[UART_QUEUE_LEN];
	  message_t  * ONE_NOK uartQueue[UART_QUEUE_LEN];
	  uint8_t    uartIn, uartOut;
	  bool       uartBusy, uartFull;
	
      message_t  wrenQueueBufs[WREN_QUEUE_LEN];
      message_t  * ONE_NOK wrenQueue[WREN_QUEUE_LEN];
      uint8_t    wrenIn, wrenOut;
      bool       wrenBusy, wrenFull;

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
    bool connectionlocked = FALSE;

    uint16_t currentCommand;
    uint8_t currentChannel;
    uint16_t currentdst;

    task void changeChannelTask();
    task void uartSendTask();
    task void wrenSendTask();
    void sendCommand();
    task void radioSendTask();

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

        for (i = 0; i < WREN_QUEUE_LEN; i++)
          wrenQueue[i] = &wrenQueueBufs[i];
        wrenIn = wrenOut = 0;
        wrenBusy = FALSE;
        wrenFull = TRUE;
	
	    for (i = 0; i < RADIO_QUEUE_LEN; i++)
	      radioQueue[i] = &radioQueueBufs[i];
	    radioIn = radioOut = 0;
	    radioBusy = FALSE;
	    radioFull = TRUE;
	
	    if (call RadioControl.start() == EALREADY)
	      radioFull = FALSE;
	    if (call SerialControl.start() == EALREADY) {
	      uartFull = FALSE;
	      wrenFull = FALSE;
	    }
		      
        currentCommand = CMD_NONE;
        currentChannel = CC2420_DEF_CHANNEL;
		currentdst = AM_BROADCAST_ADDR;     
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
			     wrenFull = FALSE;
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

    event void WRENSend.sendDone(message_t* msg, error_t error) {
        if (error != SUCCESS)
          failBlink();
        else
          atomic
        if (msg == wrenQueue[wrenOut])
          {
            if (++wrenOut >= WREN_QUEUE_LEN)
              wrenOut = 0;
            if (wrenFull)
              wrenFull = FALSE;
          }
        post wrenSendTask();
    }

    event void SerialConnectionSend.sendDone(message_t* msg, error_t err) {
        connectionlocked = FALSE;
    }

    event void SerialBaseStatusSend.sendDone(message_t* msg, error_t err) {
        basestatuslocked = FALSE;
    }

    event void Timer0.fired() {
        rssi_serial_msg_t* rcm = (rssi_serial_msg_t*)call RadioPacket.getPayload(&packet, sizeof(rssi_serial_msg_t));
        call UartSend.send(0, &packet, m_entry.len);
    }

    event void BaseCMDSend.sendDone(message_t* msg, error_t err) {
        #ifdef MOTE_DEBUG_MESSAGES
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("b_s:d");
                call DiagMsg.send();
            }
        #endif

       if ( (err == SUCCESS) && (msg == &cmdpacket) ) {
            call UartPacket.clear(&cmdpacket);
        }
        call Leds.led0Off();
        m_busy = FALSE;
        
        
        // restore old channel
        /*
        call CC2420Config.setChannel(CC2420_DEF_CHANNEL);
        channel = DEF;
        call CC2420Config.sync();
        */
        
        cmdlocked = FALSE;
    }

    event void CMDSend.sendDone(message_t* msg, error_t error) {
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

     event message_t * ConnectionReceive.receive(message_t *msg,
                                void *payload,
                                uint8_t len) {

        if (call RadioAMPacket.isForMe(msg)) {
            if (len != sizeof(wren_connection_msg_t)) {
                call Leds.led0Toggle();
                return msg;
            }
            else {
	            if(!connectionlocked) {
		       #ifdef MOTE_DEBUG_MESSAGES
		        
		            if (call DiagMsg.record())
		            {
		                call DiagMsg.str("bsr:3");
		                call DiagMsg.send();
		            }
		        #endif
		                if (call SerialConnectionSend.send(AM_BROADCAST_ADDR, msg, sizeof(wren_connection_msg_t)) == SUCCESS) {
		                    connectionlocked = TRUE;
		                }
		            }
	            }
        }
        return msg;
    }
     event message_t * BaseStatusReceive.receive(message_t *msg,
                                void *payload,
                                uint8_t len) {
        #ifdef MOTE_DEBUG_MESSAGES
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("bsr:1");
                call DiagMsg.send();
            }
        #endif

        if (len != sizeof(base_status_msg_t)) {
	       #ifdef MOTE_DEBUG_MESSAGES
	        
	            if (call DiagMsg.record())
	            {
	                call DiagMsg.str("bsr:2");
	                call DiagMsg.send();
	            }
	        #endif
            call Leds.led0Toggle();
            return msg;
        }
        else {
//            if(!basestatuslocked) {
       #ifdef MOTE_DEBUG_MESSAGES
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("bsr:3");
                call DiagMsg.send();
            }
        #endif
                if (call SerialBaseStatusSend.send(AM_BROADCAST_ADDR, msg, sizeof(base_status_msg_t)) == SUCCESS) {
                    basestatuslocked = TRUE;
                }
 //           }
            
        }
        return msg;
      }
      
     event message_t * CMDReceive.receive(message_t *msg,
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

     event message_t * WRENReceive.receive(message_t *msg,
                                void *payload,
                                uint8_t len) {
        message_t *ret = msg;
    
        #ifdef MOTE_DEBUG_MESSAGES
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("wrr:1");
                call DiagMsg.send();
            }
        #endif
        
            atomic {
              if (!wrenFull)
            {
              ret = wrenQueue[wrenIn];
              wrenQueue[wrenIn] = msg;
        
              wrenIn = (wrenIn + 1) % WREN_QUEUE_LEN;
            
              if (wrenIn == wrenOut)
                wrenFull = TRUE;
        
              if (!wrenBusy)
                {
                  post wrenSendTask();
                  wrenBusy = TRUE;
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

      task void wrenSendTask() {
        uint8_t len;
        am_id_t id;
        am_addr_t addr, src;
        message_t* msg;
        am_group_t grp;
        atomic
          if (wrenIn == wrenOut && !wrenFull)
        {
          wrenBusy = FALSE;
          return;
        }
    
        msg = wrenQueue[wrenOut];
        tmpLen = len = call RadioPacket.payloadLength(msg);
        id = call RadioAMPacket.type(msg);
        addr = call RadioAMPacket.destination(msg);
        src = call RadioAMPacket.source(msg);
        grp = call RadioAMPacket.group(msg);
        call UartPacket.clear(msg);
        call UartAMPacket.setSource(msg, src);
        call UartAMPacket.setGroup(msg, grp);
    
        if (call WRENSend.send(addr, wrenQueue[wrenOut], len) == SUCCESS)
          call Leds.led1Toggle();
        else
          {
        failBlink();
        post wrenSendTask();
          }
      }
    event void CC2420Config.syncDone(error_t err) { 
        //call RandomTimer2.startOneShot((call Random.rand32()%50));
    }

    event void RandomTimer2.fired() {
        //void sendCommand();
    }
    
    void sendCommand() {
       uint32_t time;
        time  = call GlobalTime.getLocalTime();

        // set the power for this packet to high
        call CC2420Packet.setPower(&cmdpacket, CC2420_SECOND_RFPOWER); 

       call LowPowerListening.setRemoteWakeupInterval(&cmdpacket, REMOTE_WAKEUP_INTERVAL);

//        switch(currentCommand)
//        {
//            case CMD_DOWNLOAD:
//                // This can be specific node that needs to be downloaded
//                break;
//
//            case CMD_ERASE:
//                break;
//
//            case CMD_START_SENSE:
//                break;
//
//            case CMD_STOP_SENSE:
//                break;
//
//            case CMD_STATUS:
//                break;
//            
//            case CMD_START_BLINK:
//                call Leds.led1Toggle();
//                break;
//                                
//            default:
//                break;
//        }

       if (currentCommand == CMD_BASESTATUS) {
            if (call BaseCMDSend.send(AM_BROADCAST_ADDR, &cmdpacket, sizeof(cmd_serial_msg_t), time) != SUCCESS) {
//            if (call BaseCMDSend.send(rcm->dst, msg, sizeof(cmd_serial_msg_t), time) == SUCCESS) {
            #ifdef MOTE_DEBUG_MESSAGES
            
                if (call DiagMsg.record())
                {
                    call DiagMsg.str("b_s:e");
                    call DiagMsg.send();
                }
            #endif
                cmdlocked = TRUE;
            }
            else {
            #ifdef MOTE_DEBUG_MESSAGES
            
                if (call DiagMsg.record())
                {
                    call DiagMsg.str("b_s:s");
                    call DiagMsg.send();
                }
            #endif
                
            }
       }
       else {
            if (currentCommand == CMD_DOWNLOAD) {
              if (call CMDSend.send(currentdst, &cmdpacket, sizeof(cmd_serial_msg_t), time) == SUCCESS) {

                #ifdef MOTE_DEBUG_MESSAGES
                
                    if (call DiagMsg.record())
                    {
                        call DiagMsg.str("m_s:e1");
                        call DiagMsg.send();
                    }
                #endif
                    cmdlocked = TRUE;
                }
                else {
                #ifdef MOTE_DEBUG_MESSAGES
                
                    if (call DiagMsg.record())
                    {
                        call DiagMsg.str("m_s:s1");
                        call DiagMsg.send();
                    }
                #endif
                    
                }
            }
            else {
                if (call CMDSend.send(AM_BROADCAST_ADDR, &cmdpacket, sizeof(cmd_serial_msg_t), time) != SUCCESS) {
    //          if (call CMDSend.send(rcm->dst, msg, sizeof(cmd_serial_msg_t), time) == SUCCESS) {

                #ifdef MOTE_DEBUG_MESSAGES
                
                    if (call DiagMsg.record())
                    {
                        call DiagMsg.str("m_s:e2");
                        call DiagMsg.send();
                    }
                #endif
                    cmdlocked = TRUE;
                }
                else {
                #ifdef MOTE_DEBUG_MESSAGES
                
                    if (call DiagMsg.record())
                    {
                        call DiagMsg.str("m_s:s2");
                        call DiagMsg.send();
                    }
                #endif
                    
                }
           }
       }
    }

    task void changeChannelTask() {
        call CC2420Config.setChannel(currentChannel);

        if(call CC2420Config.sync() != SUCCESS) {
            call Leds.led0Toggle();
            post changeChannelTask();
        }
        else {
            call Leds.led1Toggle();
        }
    }                    
                    
    event message_t* SerialReceive.receive(message_t* msg,
            void* payload, uint8_t len) {

	    message_t *ret = msg;
	    bool reflectToken = FALSE;
	
    #ifdef MOTE_DEBUG_MESSAGES
    
        if (call DiagMsg.record())
        {
            call DiagMsg.str("srr:1");
            call DiagMsg.send();
        }
    #endif

	    atomic
	      if (!radioFull)
	    {
	      reflectToken = TRUE;
	      ret = radioQueue[radioIn];
	      radioQueue[radioIn] = msg;
	      if (++radioIn >= RADIO_QUEUE_LEN)
	        radioIn = 0;
	      if (radioIn == radioOut)
	        radioFull = TRUE;
	
	      if (!radioBusy)
	        {
	          post radioSendTask();
	          radioBusy = TRUE;
	        }
	    }
	      else
	    dropBlink();
	
	    if (reflectToken) {
	      //call UartTokenReceive.ReflectToken(Token);
	    }
    
        return ret;
    }
    
  task void radioSendTask() {
    uint8_t len;
    am_id_t id;
    am_addr_t addr,source;
    message_t* msg;

    uint32_t time;
    time  = call GlobalTime.getLocalTime();

    #ifdef MOTE_DEBUG_MESSAGES
    
        if (call DiagMsg.record())
        {
            call DiagMsg.str("rst:1");
            call DiagMsg.send();
        }
    #endif
    
    atomic
      if (radioIn == radioOut && !radioFull)
    {
      radioBusy = FALSE;
      return;
    }

    msg = radioQueue[radioOut];
    len = call UartPacket.payloadLength(msg);
    addr = call UartAMPacket.destination(msg);
    source = call UartAMPacket.source(msg);
    id = call UartAMPacket.type(msg);

    call RadioPacket.clear(msg);
    call RadioAMPacket.setSource(msg, source);
    
    #ifdef MOTE_DEBUG_MESSAGES
    
        if (call DiagMsg.record())
        {
            call DiagMsg.str("rst:2");
            call DiagMsg.uint16(addr);
            call DiagMsg.send();
        }
    #endif

    call LowPowerListening.setRemoteWakeupInterval(msg, REMOTE_WAKEUP_INTERVAL);
    if (call CMDSend.send(addr, msg, len, time) == SUCCESS)
      call Leds.led0Toggle();
    else
      {
	    #ifdef MOTE_DEBUG_MESSAGES
	    
	        if (call DiagMsg.record())
	        {
	            call DiagMsg.str("rst:e");
	            call DiagMsg.send();
	        }
	    #endif
    failBlink();
    post radioSendTask();
      }
  }    
}
