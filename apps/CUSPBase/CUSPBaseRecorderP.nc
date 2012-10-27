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
        interface AMSend as SerialBaseStatusSend; //base serial status send
        interface Receive as SerialReceive;
        interface Packet as UartPacket;
        interface AMPacket as UartAMPacket;
        
        interface TimeSyncAMSend<TMilli,uint32_t> as CMDSend;
        interface Receive as CMDReceive; // cmd receive

        interface TimeSyncAMSend<TMilli,uint32_t> as ConnectionSend;
        interface Receive as ConnectionReceive; // cmd receive
        interface AMSend as SerialCloseSend; 

        interface TimeSyncAMSend<TMilli,uint32_t> as AckSend;

        interface Receive as BaseCMDReceive; // base receive
        interface TimeSyncAMSend<TMilli,uint32_t> as BaseStatusSend;

        interface Receive as WRENReceive; // wren receive
        interface AMSend as WRENSerialSend; // serial rssi send
        
        interface Receive as RssiLogReceive; // log receive
        
        //interface Receive as Snoop[uint8_t id];
        interface SplitControl as RadioControl;
        interface SplitControl as SerialControl;

        interface Timer<TMilli> as Timer0;
        interface Timer<TMilli> as RandomTimer2;
        interface Timer<TMilli> as AckTimer;
        interface Timer<TMilli> as ConnectionTimer;
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
          UART_QUEUE_LEN = WINDOWSIZE,
          WREN_QUEUE_LEN = 12,
          RADIO_QUEUE_LEN = 12,
          CONNECTION_QUEUE_LEN = 2,
        };
      
    typedef nx_struct logentry_t {
        nx_uint8_t len;
        rssi_serial_msg_t msg;
    } logentry_t;

    enum {
        DEF = 0,
        SECOND = 1,
    };

    uint8_t channel = DEF;

      message_t  uartQueueBufs[UART_QUEUE_LEN];
      message_t  * ONE_NOK uartQueue[UART_QUEUE_LEN];
      uint8_t    uartIn, uartOut;
      bool       uartBusy, uartFull;
    
      message_t  radioQueueBufs[RADIO_QUEUE_LEN];
      message_t  * ONE_NOK radioQueue[RADIO_QUEUE_LEN];
      uint8_t    radioIn, radioOut;
      bool       radioBusy, radioFull;

      message_t  wrenQueueBufs[WREN_QUEUE_LEN];
      message_t  * ONE_NOK wrenQueue[WREN_QUEUE_LEN];
      uint8_t    wrenIn, wrenOut;
      bool       wrenBusy, wrenFull;

    bool m_busy = TRUE;
    logentry_t m_entry;

    message_t packet;
    message_t statuspacket;
    message_t basestatuspacket;
    message_t rssipacket;
    message_t cmdpacket;
    message_t connectionpacket;
    message_t serialclosepacket;
    message_t swppacket;

    bool statuslocked = FALSE;
    bool cmdlocked = FALSE;
    bool amsendlocked = FALSE;
    bool basestatuslocked = FALSE;
    bool connectionlocked = FALSE;
    bool serialcloselocked = FALSE;
    bool acklocked = FALSE;

    uint16_t currentCMD;
    uint8_t currentChannel;
    uint16_t currentDST;
    uint32_t currentClientLogSize;

    uint8_t swLowIndex;
    uint8_t swUpperIndex;
    uint32_t lastUartedFrameNo;
    
    uint32_t lastReceivedFrameNo;
    uint8_t lastReceivedFrameIndex;
    
    uint32_t lastAcceptableFrameNo;
    uint8_t lastAcceptableFrameIndex;
    
    uint16_t ackDst;
    uint32_t ackseqno;
    
    uint8_t lastUartedFrameIndex;
    uint32_t recordCount;
    
    uint8_t ackAttempt;
    uint16_t connectionClient;
    uint8_t downloadDone;

    uint8_t connectionAttempt;
    
    FrameItem frameTable[UART_QUEUE_LEN];

    task void changeChannelTask();
    task void uartSendTask();
    task void sendStatusToBase();
    task void sendCommand();
    task void sendConnection();
    task void sendAck();
    task void wrenSendTask();
    task void stopAckTimer();
    task void closeConnection();
    
    uint8_t getIndexFromLastFrameReceived(uint32_t frameno);
    uint32_t findFrameIndex(uint32_t frameno);
	uint8_t getIndexNumber(uint32_t input);
	uint32_t slideDownWindows(uint32_t input);
	uint32_t slideUpWindows(uint32_t input);
    void adjustLastReceivedFrame(uint32_t frameno, uint8_t idx);
    void adjustAcceptableSlidingWindows(uint32_t frameno, uint8_t idx);
    void initializeDownload();
    
    void clearFrameTable()
    {
        int8_t i;
        for(i = 0; i < UART_QUEUE_LEN; ++i) {
            frameTable[i].dst = 0;
            frameTable[i].state = ENTRY_EMPTY;
            frameTable[i].frameno = 0;
            frameTable[i].timeout = 0;
            frameTable[i].index = 0;
            frameTable[i].attempt = 0;
        }
    }

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

        for (i = 0; i < WREN_QUEUE_LEN; i++)
          wrenQueue[i] = &wrenQueueBufs[i];
        wrenIn = wrenOut = 0;
        wrenBusy = FALSE;
        wrenFull = TRUE;

        if (call RadioControl.start() == EALREADY)
          radioFull = FALSE;
        if (call SerialControl.start() == EALREADY)
          uartFull = FALSE;
    
        currentCMD = CMD_NONE;
        currentChannel = CC2420_DEF_CHANNEL;
        currentClientLogSize = 0;
        
        swLowIndex = swUpperIndex = 0;
        lastReceivedFrameNo = 0;
        lastUartedFrameNo = 0;
        lastAcceptableFrameNo = 0;
        recordCount = 0;
        
        connectionClient = 0;
        
        ackseqno = 0;
        ackAttempt = 0;
        downloadDone = 0;

        connectionAttempt = 0;
        
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

    event void ConnectionTimer.fired() {
        if (connectionAttempt < CONNECTION_ATTEMPT) {
            connectionAttempt++;
            post sendConnection();
        } else {
            // give up now after # of trials            
            call ConnectionTimer.stop();
            // what to do now after we don't get data with # of ack attempts
            // maybe move on??
            // close connection and start a new download
            post closeConnection();
        }
    }

    event void AckTimer.fired() {
        
        if (ACK_ATTEMPT == 0) {
            post sendAck();
        }
        else {
	        if (ackAttempt <= ACK_ATTEMPT && !downloadDone) {
	            post sendAck();
	            ackAttempt++;
	        } else {
	            ackAttempt = 0;
	            call AckTimer.stop();
	            // what to do now after we don't get data with # of ack attempts
	            // maybe move on??
	            // close connection and start a new download
                post closeConnection();
	        }
        }
    }

    uint8_t getIndexFromLastFrameReceived(uint32_t frameno) {
        uint8_t i;
        uint8_t index;
         
        i = (lastReceivedFrameNo - frameno);
        index = getIndexNumber(lastReceivedFrameIndex + i);           
                       
        return index;
    }

    uint32_t findFrameIndex(uint32_t frameno) {
        uint8_t i;
        for (i = 0; i < UART_QUEUE_LEN; i++) {
          if (frameTable[i].frameno == frameno) {
            break;
          }
        }       
        
        return i;
    }

	uint8_t getIndexNumber(uint32_t input) {
		return input % UART_QUEUE_LEN;
	}
	
	uint32_t slideDownWindows(uint32_t input) {
		if (input <= UART_QUEUE_LEN) {
          return 0;
		} else {
		  return input - (UART_QUEUE_LEN - 1);
		}	
	}

	uint32_t slideUpWindows(uint32_t input) {
		return input + (UART_QUEUE_LEN - 1);	
	}
	
    void adjustLastReceivedFrame(uint32_t frameno, uint8_t idx) {
        uint8_t i;
        uint8_t index;
        #ifdef MOTE_DEBUG_MESSAGE_DETAIL
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("adj:0");
                call DiagMsg.uint8(lastUartedFrameIndex);
                call DiagMsg.uint8(lastReceivedFrameIndex);
                call DiagMsg.send();
            }
        #endif
        for (i = lastReceivedFrameIndex; i < slideUpWindows(lastUartedFrameIndex); i++) {
          index = i % UART_QUEUE_LEN;
	        #ifdef MOTE_DEBUG_MESSAGE_DETAIL
	        
	            if (call DiagMsg.record())
	            {
	                call DiagMsg.str("adj:1");
                    call DiagMsg.uint8(index);
	                call DiagMsg.uint8(frameTable[index].state);
                    call DiagMsg.uint32(lastReceivedFrameNo);
                    call DiagMsg.uint8(lastReceivedFrameIndex);
	                call DiagMsg.send();
	            }
	        #endif
          if (frameTable[index].state == ENTRY_UART || frameTable[index].state == ENTRY_RADIO) {
	          lastReceivedFrameNo = frameTable[index].frameno;
	          lastReceivedFrameIndex = index;
          }
          else {
              break;
          }
        }  
        
        #ifdef MOTE_DEBUG_MESSAGE
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("adj:2");
                call DiagMsg.uint32(lastReceivedFrameNo);
                call DiagMsg.uint8(lastReceivedFrameIndex);
                call DiagMsg.send();
            }
        #endif
        
             
    }

    void adjustAcceptableSlidingWindows(uint32_t frameno, uint8_t idx) {
        lastAcceptableFrameNo = slideDownWindows(frameno);
        lastAcceptableFrameIndex = getIndexNumber(slideUpWindows(idx));
        
        #ifdef MOTE_DEBUG_MESSAGE_DETAIL
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("asw:0");
                call DiagMsg.uint32(lastAcceptableFrameNo);
                call DiagMsg.uint8(lastAcceptableFrameIndex);
                call DiagMsg.send();
            }
        #endif
        
             
    }

    void printFrameTable(uint8_t index) {
        #ifdef MOTE_DEBUG_MESSAGE_DETAIL
            if (call DiagMsg.record())
            {
                call DiagMsg.str("frame:");
                call DiagMsg.uint16(frameTable[index].dst);
                call DiagMsg.uint8(frameTable[index].state);
                call DiagMsg.uint32(frameTable[index].frameno);
                call DiagMsg.uint32(frameTable[index].index);
                call DiagMsg.send();
            }
        #endif
        
    }
    
     event message_t * RssiLogReceive.receive(message_t *msg,
                                void *payload,
                                uint8_t len) {
        message_t *ret = msg;
        rssi_serial_msg_t* rssim = (rssi_serial_msg_t*)payload;

        #ifdef MOTE_DEBUG_MESSAGE_DETAIL
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("rlr:1");
                call DiagMsg.uint16(rssim->dst);
                call DiagMsg.uint32(rssim->size);
                call DiagMsg.uint32(lastAcceptableFrameNo);
                call DiagMsg.send();
            }
        #endif
        
        if (len == sizeof(rssi_serial_msg_t)) {
            
            ++recordCount;
            // We need to set default values when the download starts.
			// Somehow the connection is established, the numbers delivered don't match our first data arrived here.
            if (recordCount == 1) {
	            call ConnectionTimer.stop();
                
                // We have size that starts the largest and decrement by one from there, give 1 pad
                //lastReceivedFrameNo = rssim->size + 1;
                lastReceivedFrameNo = rssim->size;
                
                // decide which buffer index will be the first one to be used
                lastReceivedFrameIndex = 0;
                
                // We need to have a default value for the first uart  
                lastUartedFrameNo = lastReceivedFrameNo;
                
                // set the default index for uart
                lastUartedFrameIndex = 0;

                // since the number is from the largest to the smallest, we need to do minus
                lastAcceptableFrameNo = slideDownWindows(lastUartedFrameNo);
                
                // But the buffer is incrementing by one on the other hand, confusing...
                lastAcceptableFrameIndex = getIndexNumber(slideUpWindows(lastUartedFrameIndex));
                
            }
            
	        // Filtering (if the receiving packet has greater number than or smaller number than we are expecting, discard)
	        if (rssim->size > lastReceivedFrameNo || rssim->size < lastAcceptableFrameNo)
	        {
	            
                // ackDst = rssim->dst;
	            // post sendAck();
	            // Discard: Don't send ACK
	            return ret;            
	        }           
            
			// important to run following in the atomic action
            atomic {
              //uartIn = rssim->frameindex;
              
              // This line should return % Buffer Size index of the size within the limit of the buffer size
              uartIn = getIndexFromLastFrameReceived(rssim->size);

        #ifdef MOTE_DEBUG_MESSAGE
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("rlr:2");
                call DiagMsg.uint16(rssim->dst);
                call DiagMsg.uint32(lastUartedFrameNo);
                call DiagMsg.uint32(lastAcceptableFrameNo);
                call DiagMsg.uint32(rssim->size);
                call DiagMsg.uint8(uartIn);
                call DiagMsg.send();
            }
        #endif

              // Filter out packets already received 
              //if (frameTable[uartIn].state == ENTRY_EMPTY) {
                  // stop now. we got something. 
                  call AckTimer.stop();
			      ackAttempt = 0;
                  
	              ret = uartQueue[uartIn];
	              uartQueue[uartIn] = msg;
	                
                  frameTable[uartIn].dst = rssim->dst;
		          frameTable[uartIn].state = ENTRY_RADIO;
		          frameTable[uartIn].frameno = rssim->size;
		          frameTable[uartIn].timeout = 0;
	              frameTable[uartIn].index = uartIn;
	
	              //printFrameTable(uartIn);

				  // now adjust the sfr and others using the current received data
	              adjustLastReceivedFrame(rssim->size, uartIn);
	              
	              if (rssim->size == 0) {
	                downloadDone = 1;
	              }
                  // send ack for reception
                  ackDst = rssim->dst;
                  post sendAck();

                  // send ack for reception
	              //ackDst = rssim->dst;
	              //post sendAck();

			       #ifdef MOTE_DEBUG_MESSAGE_DETAIL
			        
			            if (call DiagMsg.record())
			            {
			                call DiagMsg.str("rlr:4");
			                call DiagMsg.uint8(lastReceivedFrameIndex);
			                call DiagMsg.send();
			            }
			        #endif
                  if (!uartBusy) {
                    post uartSendTask();
                    uartBusy = TRUE;
                  }
                  //uartIn = (uartIn + 1) % UART_QUEUE_LEN;
               //}
            }
        }
        return ret;
      }

      task void uartSendTask() {
        uint8_t len;
        am_id_t id;
        am_addr_t addr, src;
        message_t* msg;
        am_group_t grp;
        uint8_t tmpLen;

       #ifdef MOTE_DEBUG_MESSAGE_DETAIL
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("ust:0");
                call DiagMsg.uint8(lastReceivedFrameIndex);
                call DiagMsg.uint8(uartOut);
                call DiagMsg.send();
            }
        #endif

	    atomic
	      if (lastReceivedFrameIndex == uartOut)
	    {
	      uartBusy = FALSE;
	      return;
	    }

       #ifdef MOTE_DEBUG_MESSAGE_DETAIL
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("ust:1");
                call DiagMsg.send();
            }
        #endif

        //if (frameTable[uartOut].state == ENTRY_RADIO) {
	        msg = uartQueue[uartOut];
	        tmpLen = len = call RadioPacket.payloadLength(msg);
	        id = call RadioAMPacket.type(msg);
	        addr = call RadioAMPacket.destination(msg);
	        src = call RadioAMPacket.source(msg);
	        grp = call RadioAMPacket.group(msg);
	        call UartPacket.clear(msg);
	        call UartAMPacket.setSource(msg, src);
	        call UartAMPacket.setGroup(msg, grp);
	        
	        atomic {
	          frameTable[uartOut].state = ENTRY_UART;
			}
				
	        if (call UartSend.send(addr, uartQueue[uartOut], len) == SUCCESS) {
		          call Leds.led1Toggle();
	        }
	        else
	          {
		        failBlink();
		        post uartSendTask();
	        }
		//}
    }
    
    event void UartSend.sendDone(message_t* msg, error_t error) {
        if (error != SUCCESS) {
          failBlink();
        }
        else
          atomic
        if (msg == uartQueue[uartOut])
          {
            lastUartedFrameIndex = uartOut;
            lastUartedFrameNo = frameTable[uartOut].frameno;

            // now adjust the sfr and others using the current received data
            adjustAcceptableSlidingWindows(lastUartedFrameNo, lastUartedFrameIndex);

            frameTable[uartOut].state = ENTRY_EMPTY;
            frameTable[uartOut].frameno = 0;
            frameTable[uartOut].timeout = 0;
            frameTable[uartOut].index = 0;
            
            if (++uartOut >= UART_QUEUE_LEN)
              uartOut = 0;
          }
          
        post uartSendTask();
    }

    task void sendAck()
    {
        uint32_t time;
        wren_ack_msg_t* sm;
        if(!acklocked) {
            
            time  = call GlobalTime.getLocalTime();
            sm = (wren_ack_msg_t*)call RadioPacket.getPayload(&swppacket, sizeof(wren_ack_msg_t));
    
            #ifdef MOTE_DEBUG_MESSAGE_DETAIL
                if (call DiagMsg.record())
                {
                    call DiagMsg.str("ack:1");
                    call DiagMsg.send();
                }
            #endif


            if (sm == NULL) {
                return;
            }
            
            sm->seqno = ++ackseqno;
            sm->ackNumber = lastReceivedFrameNo; 
            //sm->ackNumber = lastUartedFrameNo; 
            
        #ifdef MOTE_DEBUG_MESSAGE
            if (call DiagMsg.record())
            {
                call DiagMsg.str("ack:2");
                call DiagMsg.uint16(ackDst);
                call DiagMsg.uint32(sm->seqno);
                call DiagMsg.uint32(sm->ackNumber);
                call DiagMsg.send();
            }
        #endif
            
            #ifdef RF233_USE_SECOND_RFPOWER
                call PacketTransmitPower.set(&swppacket, RF233_SECOND_RFPOWER);
            #endif        
    
            call LowPowerListening.setRemoteWakeupInterval(&swppacket, REMOTE_WAKEUP_INTERVAL);
    
//                if (call CMDSend.send(AM_BROADCAST_ADDR, &statuspacket, sizeof(serial_status_msg_t), time) == SUCCESS) {
            if (call AckSend.send(ackDst, &swppacket, sizeof(wren_ack_msg_t), time) == SUCCESS) {
                acklocked = TRUE;
            } else {
		        acklocked = FALSE;
                post sendAck();
            
            }
        }
    }

    event void AckSend.sendDone(message_t* msg, error_t err) {
        #ifdef MOTE_DEBUG_MESSAGE_DETAIL
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("ack:sd");
                call DiagMsg.send();
            }
        #endif
        acklocked = FALSE;
        
        /*
        if (ackAttempt == 0 && lastUartedFrameNo > 0) {
          // In case ack packet gets lost
          call AckTimer.startPeriodic(ACK_INTERVAL);
        }
        */

        if (err != SUCCESS) {
          post sendAck();
        }
        else {
	        if (ackAttempt == 0 && lastUartedFrameNo > 1) {
	          // In case ack packet gets lost
	          call AckTimer.startPeriodic(ACK_INTERVAL);
	        }
        }
      // need to see if we can uart data
      //if (lastReceivedFrameNo < lastUartedFrameNo) {
      //  uartOut = lastUartedFrameIndex + 1;
      //  post uartSendTask();
      //}
    }


    event void SerialBaseStatusSend.sendDone(message_t* msg, error_t err) {
        basestatuslocked = FALSE;
    }

    event void Timer0.fired() {
        call RadioPacket.getPayload(&packet, sizeof(rssi_serial_msg_t));
//        rssi_serial_msg_t* rcm = (rssi_serial_msg_t*)call RadioPacket.getPayload(&packet, sizeof(rssi_serial_msg_t));
        call UartSend.send(0, &packet, m_entry.len);
    }

    event void BaseStatusSend.sendDone(message_t* msg, error_t err) {
        #ifdef MOTE_DEBUG_MESSAGE_DETAIL
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("b_s:d");
                call DiagMsg.send();
            }
        #endif
        basestatuslocked = FALSE;
    }

    task void sendStatusToBase()
    {
        uint32_t time;
        base_status_msg_t* sm = (base_status_msg_t*)call RadioPacket.getPayload(&basestatuspacket, sizeof(base_status_msg_t));

        if(!basestatuslocked) {

            if (sm == NULL) {
                return;
            }
            sm->src = TOS_NODE_ID;
            time  = call GlobalTime.getLocalTime();
            sm->localtime = time;
            sm->isSynced = call GlobalTime.local2Global(&time); 
            sm->globaltime = time;
            sm->channel = call CC2420Config.getChannel();
            
            if (call SerialBaseStatusSend.send(AM_BROADCAST_ADDR, &basestatuspacket, sizeof(base_status_msg_t)) == SUCCESS) {
                basestatuslocked = TRUE;
            }
            else {
                call LowPowerListening.setRemoteWakeupInterval(&basestatuspacket, 0);
                if (call BaseStatusSend.send(AM_BROADCAST_ADDR, &basestatuspacket, sizeof(base_status_msg_t), time) == SUCCESS) {
                    basestatuslocked = TRUE;
                }
            }
        }
    }

    event void CC2420Config.syncDone(error_t err) { 
        call RandomTimer2.startOneShot((call Random.rand32()%50));
    }

    event void RandomTimer2.fired() {
        post sendCommand();
    }
    
    task void sendCommand() {
       uint32_t time;
        time  = call GlobalTime.getLocalTime();

        // set the power for this packet to high
        // call CC2420Packet.setPower(&cmdpacket, CC2420_SECOND_RFPOWER); 

       // call LowPowerListening.setRemoteWakeupInterval(msg, REMOTE_WAKEUP_INTERVAL);

        call Leds.led2Toggle();
        
        switch(currentCMD)
        {
            case CMD_DOWNLOAD:
               recordCount = 0;
		       ackAttempt = 0;
		       downloadDone = 0;
		       connectionAttempt = 0;
		       
	          // In case ack packet gets lost
	          // call AckTimer.startPeriodic(ACK_INTERVAL);
               
               /*
               call LowPowerListening.setRemoteWakeupInterval(&cmdpacket, REMOTE_WAKEUP_INTERVAL);
        
                #ifdef MOTE_DEBUG_MESSAGE_DETAIL
                
                    if (call DiagMsg.record())
                    {
                        call DiagMsg.str("m_s:1");
                        call DiagMsg.uint16(currentDST);
                        call DiagMsg.send();
                    }
                #endif

    //            if (call CMDSend.send(AM_BROADCAST_ADDR, msg, sizeof(cmd_serial_msg_t), time) == SUCCESS) {
                if (call CMDSend.send(currentDST, &cmdpacket, sizeof(cmd_serial_msg_t), time) == SUCCESS) {
                #ifdef MOTE_DEBUG_MESSAGE_DETAIL
                
                    if (call DiagMsg.record())
                    {
                        call DiagMsg.str("m_s:2");
                        call DiagMsg.send();
                    }
                #endif
                    cmdlocked = TRUE;
                }
                else {
                #ifdef MOTE_DEBUG_MESSAGE_DETAIL
                
                    if (call DiagMsg.record())
                    {
                        call DiagMsg.str("m_s:e");
                        call DiagMsg.send();
                    }
                #endif
                    
                }
                break;
*/

                break;
            case CMD_ERASE:
                break;

            case CMD_START_SENSE:
                break;

            case CMD_STOP_SENSE:
                break;

            case CMD_STATUS:
                break;
            
            case CMD_BASESTATUS:
                post sendStatusToBase();
                break;

            case CMD_START_BLINK:
                call Leds.led1Toggle();
                break;
                                
            case CMD_NONE:

                break;
            case CMD_CHANNEL_RESET:
                
                call AckTimer.stop();
                
                call LowPowerListening.setRemoteWakeupInterval(&cmdpacket, REMOTE_WAKEUP_INTERVAL);
        
                if (call CMDSend.send(AM_BROADCAST_ADDR, &cmdpacket, sizeof(cmd_serial_msg_t), time) == SUCCESS) {
                #ifdef MOTE_DEBUG_MESSAGE_DETAIL
                
                    if (call DiagMsg.record())
                    {
                        call DiagMsg.str("m_s:2");
                        call DiagMsg.send();
                    }
                #endif
                    cmdlocked = TRUE;
                }
                else {
                #ifdef MOTE_DEBUG_MESSAGE_DETAIL
                
                    if (call DiagMsg.record())
                    {
                        call DiagMsg.str("m_s:e");
                        call DiagMsg.send();
                    }
                #endif
                    
                }
                
                break;
            default:
                break;
        }

    }

    task void changeChannelTask() {
        call CC2420Config.setChannel(currentChannel);

        if(call CC2420Config.sync() != SUCCESS) {
            call Leds.led0Toggle();
            post changeChannelTask();
        }
        else {
            post sendStatusToBase();
            call Leds.led1Toggle();
        }
    }                    

    task void stopAckTimer() 
    {
        call AckTimer.stop();
    }
    
     event message_t * WRENReceive.receive(message_t *msg,
                                void *payload,
                                uint8_t len) {
        message_t *ret = msg;
        #ifdef MOTE_DEBUG_MESSAGES
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("wrr:0");
                call DiagMsg.send();
            }
        #endif

        wren_status_msg_t* rssim = (wren_status_msg_t*)payload;

        #ifdef MOTE_DEBUG_MESSAGES
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("wrr:1");
                call DiagMsg.uint8(rssim->download);
                call DiagMsg.send();
            }
        #endif

        if (len == sizeof(wren_status_msg_t)) {
            if (rssim->download == 1) {
                post stopAckTimer();
            }
        }
        #ifdef MOTE_DEBUG_MESSAGES
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("wrr:2");
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

      task void wrenSendTask() {
        uint8_t len;
        uint8_t tmpLen;
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
    
        if (call WRENSerialSend.send(addr, wrenQueue[wrenOut], len) == SUCCESS)
          call Leds.led1Toggle();
        else
          {
        failBlink();
        post wrenSendTask();
          }
      }

    event void WRENSerialSend.sendDone(message_t* msg, error_t error) {
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

     event message_t * BaseCMDReceive.receive(message_t *msg,
                                void *payload,
                                uint8_t len) {

        if (call RadioAMPacket.isForMe(msg)) {
            if (len != sizeof(cmd_serial_msg_t)) {
                call Leds.led0Toggle();
                return msg;
            }
            else {
                cmd_serial_msg_t* rkcm = (cmd_serial_msg_t*)call RadioPacket.getPayload(&cmdpacket, sizeof(rssi_msg_t));
                cmd_serial_msg_t* rcm = (cmd_serial_msg_t*)payload;

                rkcm->cmd   = rcm->cmd;
                rkcm->dst   = rcm->dst;
                rkcm->channel = rcm->channel;
                
                currentCMD = rcm->cmd;
                currentChannel = rcm->channel;
                currentDST     = rcm->dst;
                
                if (currentCMD == CMD_CHANNEL) { 
                    post changeChannelTask();
                }
                else {
                    post sendCommand();
                } 
                /*
                if (currentCMD == CMD_DOWNLOAD || currentCMD == CMD_CHANNEL) { 
                    post changeChannelTask();
                }
                else {
                    post sendCommand();
                } 
                */
            }
        }
        return msg;
    }

    void initializeDownload(){
        lastReceivedFrameNo = 0;
        lastUartedFrameNo = 0;
        lastAcceptableFrameNo = 0;
        recordCount = 0;

        ackseqno = 0;
        downloadDone = 0;
        ackAttempt = 0;
        connectionAttempt = 0;
        
        clearFrameTable();
    }
    
     event message_t * ConnectionReceive.receive(message_t *msg,
                                void *payload,
                                uint8_t len) {
        message_t *ret = msg;

        #ifdef MOTE_DEBUG_MESSAGE_DETAIL
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("crr:0");
                call DiagMsg.uint8(call CC2420Config.getChannel());
                call DiagMsg.send();
            }
        #endif
        
        // call AckTimer.stop();
//        post stopAckTimer();
        
        #ifdef MOTE_DEBUG_MESSAGE_DETAIL
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("crr:1");
                call DiagMsg.send();
            }
        #endif
        call ConnectionTimer.stop();
        post stopAckTimer();

        if (len != sizeof(wren_connection_msg_t)) {
            call Leds.led0Toggle();
            return ret;
        }
        else {
	        atomic {
	            wren_connection_msg_t* rkcm = (wren_connection_msg_t*)call RadioPacket.getPayload(&connectionpacket, sizeof(wren_connection_msg_t));
	            wren_connection_msg_t* rcm = (wren_connection_msg_t*)payload;
	            
	
	            #ifdef MOTE_DEBUG_MESSAGE
	            
	                if (call DiagMsg.record())
	                {
	                    call DiagMsg.str("crr:2");
	                    call DiagMsg.uint16(rcm->src);
	                    call DiagMsg.uint8(rcm->isAcked);
	                    call DiagMsg.uint8(rcm->action);
	                    call DiagMsg.send();
	                }
	            #endif
	
	            rkcm->src   = rcm->src;
	            
	            
	            rkcm->cmd   = rcm->cmd;
	            rkcm->dst   = rcm->dst;
	            rkcm->logsize = rcm->logsize;
	            rkcm->action = rcm->action;
	            
	            //lastReceivedFrameNo = rkcm->logsize;
	            connectionClient = rcm->src;
	            currentClientLogSize = rkcm->logsize;

                // Initialize all variables and buffers
                initializeDownload();
                
	//                if (rkcm->logsize > 0)
	//                    lastAcceptableFrameNo = rkcm->logsize - 1;
	//                else
	//                    lastAcceptableFrameNo = rkcm->logsize;
	                
	            rkcm->channel = rcm->channel;
	            rkcm->isAcked = COM_ACK;
	
	
	            if (rcm->isAcked == COM_ACK && rcm->action == CONNECTION_CLOSE) {
			        #ifdef MOTE_DEBUG_MESSAGE
			        
			            if (call DiagMsg.record())
			            {
			                call DiagMsg.str("crr:3");
			                call DiagMsg.uint8(rcm->isAcked);
			                call DiagMsg.uint8(rcm->action);
			                call DiagMsg.send();
			            }
			        #endif
	                // we got the close connection ack from the mote
	                post closeConnection();
	                // post sendConnection();
	                #ifdef MOTE_DEBUG_MESSAGE
	                
	                    if (call DiagMsg.record())
	                    {
	                        call DiagMsg.str("crr:4");
	                        call DiagMsg.uint8(rcm->isAcked);
	                        call DiagMsg.uint8(rcm->action);
	                        call DiagMsg.send();
	                    }
	                #endif
	            }
	            else {
	                post sendConnection();
	            }
	            
	        }
        }
        return ret;
    }

    task void closeConnection() {
        wren_close_msg_t* wcm;
        #ifdef MOTE_DEBUG_MESSAGE
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("clo:0");
                call DiagMsg.uint8(serialcloselocked);
                call DiagMsg.send();
            }
        #endif
        if (!serialcloselocked) {
			wcm = (wren_close_msg_t*)call UartPacket.getPayload(&serialclosepacket, sizeof(wren_close_msg_t));
	        if (!serialcloselocked) {
	            
	            if (wcm == NULL) {
	                return;
	            }
	            wcm->src   = connectionClient;
	            wcm->channel = TOS_NODE_ID;
	            wcm->close = 1;
	            
	        #ifdef MOTE_DEBUG_MESSAGE
	        
	            if (call DiagMsg.record())
	            {
	                call DiagMsg.str("clo:1");
	                call DiagMsg.uint8(serialcloselocked);
	                call DiagMsg.send();
	            }
	        #endif
		        if (call SerialCloseSend.send(AM_BROADCAST_ADDR, &serialclosepacket, sizeof(wren_close_msg_t)) == SUCCESS) {
		            serialcloselocked = TRUE;
		        } else {
		            serialcloselocked = FALSE;
		            post closeConnection();
		        }
	        }
        }
    }

    event void SerialCloseSend.sendDone(message_t* msg, error_t err) {
        serialcloselocked = FALSE;
       if (err != SUCCESS) {
            post closeConnection();    
       }
       else {
           call UartPacket.clear(&serialclosepacket);
       }
    }

    task void sendConnection()
    {
        uint32_t time;
        time  = call GlobalTime.getLocalTime();

        if(!connectionlocked) {

            //if (rkcm == NULL) {
            //    return;
            //}
            // rkcm->isAcked = COM_ACK;

                #ifdef MOTE_DEBUG_MESSAGE
                
                    if (call DiagMsg.record())
                    {
                        call DiagMsg.str("sc:1");
                        call DiagMsg.uint16(connectionClient);
                        call DiagMsg.send();
                    }
                #endif
            
                #ifdef RF233_USE_SECOND_RFPOWER
                    call PacketTransmitPower.set(&connectionpacket, RF233_SECOND_RFPOWER);
                #endif        
        
                call LowPowerListening.setRemoteWakeupInterval(&connectionpacket, REMOTE_WAKEUP_INTERVAL);
        
//                if (call CMDSend.send(AM_BROADCAST_ADDR, &statuspacket, sizeof(serial_status_msg_t), time) == SUCCESS) {
//                if (call ConnectionSend.send(rkcm->src, &connectionpacket, sizeof(wren_connection_msg_t), time) == SUCCESS) {
                if (call ConnectionSend.send(connectionClient, &connectionpacket, sizeof(wren_connection_msg_t), time) == SUCCESS) {
                    connectionlocked = TRUE;
                } else {
                    connectionlocked = FALSE;
                    post sendConnection();
                }
            }
    }

    event void ConnectionSend.sendDone(message_t* msg, error_t err) {
        #ifdef MOTE_DEBUG_MESSAGE_DETAIL
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("m_s:d");
                call DiagMsg.send();
            }
        #endif
        connectionlocked = FALSE;
       if (err != SUCCESS) {
            post sendConnection();    
       }
       else {
           call ConnectionTimer.startPeriodic(CONNECTION_TIMEOUT);                
           //call RadioPacket.clear(&connectionpacket);
       }
        
    }
    
    event void CMDSend.sendDone(message_t* msg, error_t err) {
        #ifdef MOTE_DEBUG_MESSAGE_DETAIL
        
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
            }
            
        }
        return msg;
      }

    event message_t* SerialReceive.receive(message_t* msg,
            void* payload, uint8_t len) {

        if (len != sizeof(cmd_serial_msg_t)) {
            call Leds.led0Toggle();
            return msg;
        }
        else {
            cmd_serial_msg_t* rkcm = (cmd_serial_msg_t*)call UartPacket.getPayload(&cmdpacket, sizeof(rssi_msg_t));
            cmd_serial_msg_t* rcm = (cmd_serial_msg_t*)payload;

            currentCMD = rcm->cmd;
            currentChannel = rcm->channel;
            currentDST     = rcm->dst;

            #ifdef MOTE_DEBUG_MESSAGE_DETAIL
            
                if (call DiagMsg.record())
                {
                    call DiagMsg.str("srr:1");
                    call DiagMsg.uint16(currentDST);
                    call DiagMsg.uint8(currentChannel);
                    call DiagMsg.send();
                }
            #endif

            rkcm->cmd   = rcm->cmd;
            // rkcm->dst   = rcm->dst;
            rkcm->dst   = TOS_NODE_ID;
            rkcm->channel = rcm->channel;


            /*            
            if (currentCMD == CMD_DOWNLOAD || currentCMD == CMD_CHANNEL) { 
                if (currentChannel != call CC2420Config.getChannel()) {
                    post changeChannelTask();
                }
                else {
                    post sendCommand();
                }
            }
            else {
                post sendCommand();
            } 
            */

            if (currentCMD == CMD_CHANNEL) { 
                post changeChannelTask();
            }
            else {
                post sendCommand();
            } 
        
        }
        return msg;
    }      
}
