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

        interface TimeSyncAMSend<TMilli,uint32_t> as AckSend;

        interface Receive as BaseCMDReceive; // base receive
        interface TimeSyncAMSend<TMilli,uint32_t> as BaseStatusSend;
        
        interface Receive as RssiLogReceive; // log receive
        
        //interface Receive as Snoop[uint8_t id];
        interface SplitControl as RadioControl;
        interface SplitControl as SerialControl;

        interface Timer<TMilli> as Timer0;
        interface Timer<TMilli> as RandomTimer2;
        interface Timer<TMilli> as AckTimer;
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
          RADIO_QUEUE_LEN = 12,
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

    bool m_busy = TRUE;
    logentry_t m_entry;

    message_t packet;
    message_t statuspacket;
    message_t basestatuspacket;
    message_t rssipacket;
    message_t cmdpacket;
    message_t connectionpacket;
    message_t swppacket;

    bool statuslocked = FALSE;
    bool cmdlocked = FALSE;
    bool amsendlocked = FALSE;
    bool basestatuslocked = FALSE;
    bool connectionlocked = FALSE;
    bool acklocked = FALSE;

    uint16_t currentCMD;
    uint8_t currentChannel;
    uint16_t currentDST;
    uint32_t currentClientLogSize;

    uint8_t swLowIndex;
    uint8_t swUpperIndex;
    uint32_t lastFrameUarted;
    
    uint32_t largestFrameReceived;
    uint8_t largestFrameReceivedIndex;
    
    uint32_t largestAcceptableFrame;
    uint8_t largestAcceptableFrameIndex;
    
    uint16_t ackDst;
    uint8_t lastUARTIndex;
    uint8_t waitForAck;
    uint8_t recordCount;
    
    FrameItem frameTable[UART_QUEUE_LEN];

    task void changeChannelTask();
    task void uartSendTask();
    void sendStatusToBase();
    void sendCommand();
    void sendConnection(wren_connection_msg_t* rkcm);
    task void sendAck();
    uint8_t getFrameNo(uint32_t logsize);
    uint32_t getLastFrameReceived(uint32_t frameno);

    void clearFrameTable()
    {
        int8_t i;
        for(i = 0; i < UART_QUEUE_LEN; ++i) {
            frameTable[i].state = ENTRY_EMPTY;
            frameTable[i].frameno = 0;
            frameTable[i].timeout = 0;
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

        if (call RadioControl.start() == EALREADY)
          radioFull = FALSE;
        if (call SerialControl.start() == EALREADY)
          uartFull = FALSE;
    
        currentCMD = CMD_NONE;
        currentChannel = CC2420_DEF_CHANNEL;
        currentClientLogSize = 0;
        
        swLowIndex = swUpperIndex = 0;
        largestFrameReceived = 0;
        lastFrameUarted = 0;
        largestAcceptableFrame = 0;
        waitForAck = 0;
        recordCount = 0;
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

    event void AckTimer.fired() {
        post sendAck();
    }

    uint8_t getIndexFromFrameReceived(uint32_t frameno) {
        uint8_t i;
        uint8_t index;
         
        i = (largestFrameReceived - frameno);
        index = (largestAcceptableFrameIndex + i) % UART_QUEUE_LEN;           
                       
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

    void calculateLargestFrameReceived(uint32_t frameno, uint8_t idx) {
        uint8_t i;
        uint8_t index;
        for (i = largestFrameReceivedIndex; i < (largestFrameReceivedIndex + (UART_QUEUE_LEN - 1)); i++) {
          index = i % UART_QUEUE_LEN;
          if (frameTable[index].state == ENTRY_RADIO) {
	          largestFrameReceived = frameTable[index].frameno;
	          largestFrameReceivedIndex = index;
          }
          else {
              break;
          }
        }  
        
        largestAcceptableFrame = largestFrameReceived - (WINDOWSIZE - 1);
        largestAcceptableFrameIndex = largestFrameReceivedIndex + (WINDOWSIZE - 1);
             
    }

     event message_t * RssiLogReceive.receive(message_t *msg,
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
                call DiagMsg.uint32(largestAcceptableFrame);
                call DiagMsg.send();
            }
        #endif
        if (len == sizeof(rssi_serial_msg_t)) {
            recordCount++;
            if (recordCount == 1) {
                largestFrameReceived = rssim->size;
                largestFrameReceivedIndex = uartIn;
                largestAcceptableFrame = largestFrameReceived - (WINDOWSIZE - 1);
                largestAcceptableFrameIndex = largestFrameReceivedIndex + (WINDOWSIZE - 1);
            }
            
	        // Filtering
	        if (rssim->size > largestFrameReceived || rssim->size < largestAcceptableFrame)
	        {
	            
                ackDst = rssim->dst;
	            post sendAck();
	            // Discard: Don't send ACK
	            return ret;            
	        }           
            
            
        #ifdef MOTE_DEBUG_MESSAGES
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("rlr:2");
                call DiagMsg.uint32(largestAcceptableFrame);
                call DiagMsg.uint8(uartIn);
                call DiagMsg.send();
            }
        #endif

            atomic {
              uartIn = getIndexFromFrameReceived(rssim->size);

              ret = uartQueue[uartIn];
              uartQueue[uartIn] = msg;
                
	          frameTable[uartIn].state = ENTRY_RADIO;
	          frameTable[uartIn].frameno = rssim->size;
	          frameTable[uartIn].timeout = 0;
              frameTable[uartIn].index = uartIn;

              calculateLargestFrameReceived(rssim->size, uartIn);
              
              uartOut = uartIn;
            
              post uartSendTask();

              ackDst = rssim->dst;
              post sendAck();

              uartIn = (uartIn + 1) % UART_QUEUE_LEN;

            }
        }
        return ret;
      }

    uint8_t getFrameNo(uint32_t logsize) {
        uint8_t frameidx;
        frameidx = (currentClientLogSize - logsize) % UART_QUEUE_LEN;
        #ifdef MOTE_DEBUG_MESSAGES
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("rlr:3");
                call DiagMsg.uint8(frameidx);
                call DiagMsg.send();
            }
        #endif
        return frameidx; 
    }

    uint32_t getLastFrameReceived(uint32_t frameno) {
        uint8_t i;
        uint8_t index;
        uint32_t ackno = frameno;
        uint8_t lastindex = getFrameNo(largestFrameReceived);
        for (i = lastindex; i < (lastindex + UART_QUEUE_LEN); i++) {
            index = i % UART_QUEUE_LEN;

        #ifdef MOTE_DEBUG_MESSAGES
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("rlr:3");
                call DiagMsg.uint8(index);
                call DiagMsg.uint32(frameTable[index].frameno);
                call DiagMsg.send();
            }
        #endif

              if (frameTable[index].frameno == 0) {
                return ackno;
              } else {
                // no not here
                ackno = frameTable[index].frameno;
              }
        }       
    }

      task void uartSendTask() {
        uint8_t len;
        am_id_t id;
        am_addr_t addr, src;
        message_t* msg;
        am_group_t grp;
        uint8_t tmpLen;
        
        msg = uartQueue[uartOut];
        tmpLen = len = call RadioPacket.payloadLength(msg);
        id = call RadioAMPacket.type(msg);
        addr = call RadioAMPacket.destination(msg);
        src = call RadioAMPacket.source(msg);
        grp = call RadioAMPacket.group(msg);
        call UartPacket.clear(msg);
        call UartAMPacket.setSource(msg, src);
        call UartAMPacket.setGroup(msg, grp);

        frameTable[uartOut].state = ENTRY_UART;
    
        if (call UartSend.send(addr, uartQueue[uartOut], len) == SUCCESS)
          call Leds.led1Toggle();
        else
          {
        failBlink();
        post uartSendTask();

      }
    }
    
    event void UartSend.sendDone(message_t* msg, error_t error) {
        if (error != SUCCESS) {
          failBlink();
          post uartSendTask();
        }
        else
          atomic
        if (msg == uartQueue[uartOut])
          {
 	        frameTable[uartOut].state = ENTRY_EMPTY;
            frameTable[uartOut].frameno = 0;
            frameTable[uartOut].timeout = 0;
            frameTable[uartOut].index = 0;
          }
    }

    task void sendAck()
    {
        uint32_t time;
        wren_ack_msg_t* sm;
        if(!acklocked) {
            time  = call GlobalTime.getLocalTime();
            sm = (wren_ack_msg_t*)call RadioPacket.getPayload(&swppacket, sizeof(wren_ack_msg_t));
    
            #ifdef MOTE_DEBUG_MESSAGES
                if (call DiagMsg.record())
                {
                    call DiagMsg.str("ack:1");
                    call DiagMsg.send();
                }
            #endif


            if (sm == NULL) {
                return;
            }
            
            sm->ackNumber = largestFrameReceived; 
            sm->index = largestFrameReceivedIndex;
            sm->waitForAck = waitForAck;
            
        #ifdef MOTE_DEBUG_MESSAGES
            if (call DiagMsg.record())
            {
                call DiagMsg.str("ack:2");
                call DiagMsg.uint16(ackDst);
                call DiagMsg.uint32(sm-> ackNumber);
                call DiagMsg.uint8(sm->index);
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
            }
        }
    }

    event void AckSend.sendDone(message_t* msg, error_t err) {
        #ifdef MOTE_DEBUG_MESSAGES
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("swp:sd");
                call DiagMsg.send();
            }
        #endif
        acklocked = FALSE;

      // need to see if we can uart data
      if (largestFrameReceived < lastFrameUarted) {
        uartOut = lastUARTIndex + 1;
        post uartSendTask();
      }
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
        #ifdef MOTE_DEBUG_MESSAGES
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("b_s:d");
                call DiagMsg.send();
            }
        #endif
        basestatuslocked = FALSE;
    }

    void sendStatusToBase()
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
        sendCommand();
    }
    
    void sendCommand() {
       uint32_t time;
        time  = call GlobalTime.getLocalTime();

        // set the power for this packet to high
        // call CC2420Packet.setPower(&cmdpacket, CC2420_SECOND_RFPOWER); 

       // call LowPowerListening.setRemoteWakeupInterval(msg, REMOTE_WAKEUP_INTERVAL);

        call Leds.led2Toggle();
        
        switch(currentCMD)
        {
            case CMD_DOWNLOAD:
            
               call LowPowerListening.setRemoteWakeupInterval(&cmdpacket, REMOTE_WAKEUP_INTERVAL);
        
                #ifdef MOTE_DEBUG_MESSAGES
                
                    if (call DiagMsg.record())
                    {
                        call DiagMsg.str("m_s:1");
                        call DiagMsg.uint16(currentDST);
                        call DiagMsg.send();
                    }
                #endif

    //            if (call CMDSend.send(AM_BROADCAST_ADDR, msg, sizeof(cmd_serial_msg_t), time) == SUCCESS) {
                if (call CMDSend.send(currentDST, &cmdpacket, sizeof(cmd_serial_msg_t), time) == SUCCESS) {
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
                sendStatusToBase();
                break;

            case CMD_START_BLINK:
                call Leds.led1Toggle();
                break;
                                
            case CMD_NONE:

                break;
            case CMD_CHANNEL_RESET:
               call LowPowerListening.setRemoteWakeupInterval(&cmdpacket, REMOTE_WAKEUP_INTERVAL);
        
                if (call CMDSend.send(AM_BROADCAST_ADDR, &cmdpacket, sizeof(cmd_serial_msg_t), time) == SUCCESS) {
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
            sendStatusToBase();
            call Leds.led1Toggle();
        }
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
                    sendCommand();
                } 
                /*
                if (currentCMD == CMD_DOWNLOAD || currentCMD == CMD_CHANNEL) { 
                    post changeChannelTask();
                }
                else {
                    sendCommand();
                } 
                */
            }
        }
        return msg;
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
                wren_connection_msg_t* rkcm = (wren_connection_msg_t*)call RadioPacket.getPayload(&connectionpacket, sizeof(wren_connection_msg_t));
                wren_connection_msg_t* rcm = (wren_connection_msg_t*)payload;
                
                rkcm->src   = rcm->src;
                rkcm->cmd   = rcm->cmd;
                rkcm->dst   = rcm->dst;
                rkcm->logsize = rcm->logsize;
                
                
                currentClientLogSize = rkcm->logsize;
                //largestFrameReceived = rkcm->logsize;
                
                recordCount = 0;
                
//                if (rkcm->logsize > 0)
//                    largestAcceptableFrame = rkcm->logsize - 1;
//                else
//                    largestAcceptableFrame = rkcm->logsize;
                    
                rkcm->channel = rcm->channel;

                #ifdef MOTE_DEBUG_MESSAGES
                
                    if (call DiagMsg.record())
                    {
                        call DiagMsg.str("crr:0");
                        call DiagMsg.uint16(rkcm->src);
                        call DiagMsg.send();
                    }
                #endif
                
                sendConnection(rkcm);
            }
        }
        return msg;
    }

    void sendConnection(wren_connection_msg_t* rkcm)
    {
        uint32_t time;
        time  = call GlobalTime.getLocalTime();

        if(!connectionlocked) {

            if (rkcm == NULL) {
                return;
            }
            rkcm->isAcked = 1;

                #ifdef MOTE_DEBUG_MESSAGES
                
                    if (call DiagMsg.record())
                    {
                        call DiagMsg.str("crr:1");
                        call DiagMsg.uint16(rkcm->src);
                        call DiagMsg.send();
                    }
                #endif
            
                #ifdef RF233_USE_SECOND_RFPOWER
                    call PacketTransmitPower.set(&connectionpacket, RF233_SECOND_RFPOWER);
                #endif        
        
                call LowPowerListening.setRemoteWakeupInterval(&connectionpacket, REMOTE_WAKEUP_INTERVAL);
        
//                if (call CMDSend.send(AM_BROADCAST_ADDR, &statuspacket, sizeof(serial_status_msg_t), time) == SUCCESS) {
                if (call ConnectionSend.send(rkcm->src, &connectionpacket, sizeof(wren_connection_msg_t), time) == SUCCESS) {
                    connectionlocked = TRUE;
                }
            }
    }

    event void ConnectionSend.sendDone(message_t* msg, error_t err) {
        #ifdef MOTE_DEBUG_MESSAGES
        
            if (call DiagMsg.record())
            {
                call DiagMsg.str("m_s:d");
                call DiagMsg.send();
            }
        #endif
        connectionlocked = FALSE;
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
                    sendCommand();
                }
            }
            else {
                sendCommand();
            } 
            */

            if (currentCMD == CMD_CHANNEL) { 
                post changeChannelTask();
            }
            else {
                sendCommand();
            } 
        
        }
        return msg;
    }      
}
