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
#include "BQ25010.h"

module CUSPBaseRecorderP {
    uses {
        interface Boot;
        interface Leds;

        interface SplitControl as AMControl;
        interface Packet;
        interface AMPacket;
        interface TimeSyncAMSend<TMilli,uint32_t> as RssiSend;
        interface Receive; // rssi receive
        interface TimeSyncPacket<TMilli,uint32_t>;

        interface TimeSyncAMSend<TMilli,uint32_t> as CMDSend;
        interface Receive as CMDReceive; // cmd receive
        interface TimeSyncAMSend<TMilli,uint32_t> as RssiLogSend;
        interface TimeSyncAMSend<TMilli,uint32_t> as WRENSend;

        interface TimeSyncAMSend<TMilli,uint32_t> as ConnectionSend;
        interface Receive as ConnectionReceive; // cmd receive

        interface Receive as SWPAckReceive; // cmd receive

        interface SplitControl as SerialControl;
        interface AMSend; // serial rssi send
        interface AMSend as SerialStatusSend; // serial status send
        interface Receive as SerialReceive;
        
        //interface Receive as Snoop[uint8_t id];
        interface LogRead;
        interface LogWrite;
        interface ConfigStorage as Config;
        interface Mount as Mount;
        
        interface Timer<TMilli> as LogSendTimer;
        interface Timer<TMilli> as SensingTimer;
        interface Timer<TMilli> as HeartbeatTimer;
        interface Timer<TMilli> as LedOffTimer;
        interface Timer<TMilli> as RandomTimer;
        interface Timer<TMilli> as BatteryTimer;
        interface Timer<TMilli> as DownloadTimer;
        interface Timer<TMilli> as StatusRandomTimer;
        interface Timer<TMilli> as WRENRandomTimer;
        interface Timer<TMilli> as WRENConnectionTimer;
        interface Timer<TMilli> as WRENTimeoutTimer;
        
        interface Random;
        interface PacketField<uint8_t> as PacketRSSI;
        interface LowPowerListening;
        interface GlobalTime<TMilli>;

        interface Read<uint16_t> as BatteryRead;
        interface ReadStream<uint16_t> as BatteryReadStream;
        interface Resource as BatteryResource;
        interface ReadNow<uint16_t> as BatteryReadNow;

        interface GlobalTimeSet;
        
        // Real Time Clock
        interface Pcf2127a;
        interface Pcf2127aRtc;
        
        // MessageBufferLayerP.nc 
        interface RadioChannel;
        interface PacketField<uint8_t> as PacketTransmitPower;

        /* Power */
        interface BQ25010;
        
#ifdef MOTE_DEBUG
        interface DiagMsg;
#endif        

#ifdef DISSEMINATION_ON
        interface StdControl as DisseminationControl;
        interface DisseminationValue<uint16_t> as CommandValue;
        interface DisseminationUpdate<uint16_t> as CommandUpdate;
#endif        

        interface Blink;

    }
}
implementation {

    enum {
        INTER_PACKET_INTERVAL = 25,
    };
    
    typedef struct config_t {
        uint16_t version;
        uint8_t sensing;
        uint32_t globaltime;
        uint16_t reboot;
        uint8_t halt;
        
    } config_t;

    enum {
        CONFIG_ADDR = 0,
        CONFIG_VERSION = 7,
    };

    enum {
        DEFAULT_SENSING    = 0,
        DEFAULT_GLOBALTIME = 0,
        DEFAULT_REBOOT     = 0,
        DEFAULT_HALT       = 0,
    };

    enum {
        LOG_QUEUE_LEN = 12,
        WINDOW_BUFFER_LEN = WINDOWSIZE,
    };

    typedef uint8_t cmd_status_t;

    typedef nx_struct logentry_t {
        nx_uint8_t len;
        rssi_serial_msg_t msg;
    } logentry_t;

    logentry_t m_entry;

    message_t packet;
    message_t statuspacket;
    message_t rssipacket;
    message_t cmdpacket;
    message_t wrenpacket;
    message_t connectionpacket;

    bool sensing = FALSE;
    bool rssilocked = FALSE;
    bool statuslocked = FALSE;
    bool cmdlocked = FALSE;
    bool wrenlocked = FALSE;
    bool connectionlocked = FALSE;

    bool sleep = FALSE;
    bool halt = FALSE;
    bool stopDownload = FALSE;
    
    uint16_t counter = 0;
    config_t conf;
    uint16_t batteryLevel = 0xffff;
    uint16_t batteryLevelVal = 0xffff;
    uint8_t isErased = 0;
    uint8_t download = 0;
    uint16_t rssiMaxUnlockCounter = 0;
    uint8_t smartSensingCounter = 0;
    
    uint16_t messageCount = 0;
    
    logentry_t logQueueBufs[LOG_QUEUE_LEN];
    logentry_t *logQueue[LOG_QUEUE_LEN];
    uint8_t logIn, logOut;
    bool logBusy, logFull;

    message_t  radioQueueBufs[WINDOW_BUFFER_LEN];
    message_t  * ONE_NOK radioQueue[WINDOW_BUFFER_LEN];
    uint8_t    radioIn, radioOut;
    bool       radioBusy, radioFull;

    uint16_t currentCMD;
    uint8_t currentChannel;
    uint16_t currentDST;
	uint32_t currentClientLogSize;
	
    uint8_t swLowIndex;
    uint8_t swUpperIndex;
    uint32_t lastACKReceived;
    uint32_t lastFrameSent;
    uint32_t nextFrameToSend;
    
    FrameItem frameTable[WINDOW_BUFFER_LEN];
    
    uint32_t frameno;
    
    uint8_t connectionAttempt;
    
    task void logWriteTask();
    void sendStatus();
    void sendStatusToController();
    void sendWRENStatus();
    void shutdown(bool all);
    void saveSensing(bool sense);
    task void changeChannelTask();
    void process_command();
    void sendConnection();
    task void radioSendTask();
    task void rssiLogRead();

    void clearFrameTable()
    {
        int8_t i;
        for(i = 0; i < WINDOW_BUFFER_LEN; ++i) {
            frameTable[i].state = ENTRY_EMPTY;
            frameTable[i].frameno = 0;
            frameTable[i].timeout = 0;
        }
    }
    
    event void Boot.booted() {
        uint8_t i;
        // Create Log Queue memory storage            
        for (i = 0; i < LOG_QUEUE_LEN; i++)
          logQueue[i] = &logQueueBufs[i];
          
        logIn = logOut = 0;
        logBusy = FALSE;
        logFull = TRUE;

        for (i = 0; i < WINDOW_BUFFER_LEN; i++)
          radioQueue[i] = &radioQueueBufs[i];
        radioIn = radioOut = 0;
        radioBusy = FALSE;
        radioFull = TRUE;

        
        currentCMD = CMD_NONE;
        currentChannel = RF233_DEF_CHANNEL;
        currentDST = AM_BROADCAST_ADDR; //default download base station in case
        currentClientLogSize = 0;
        
        sleep = FALSE;
        messageCount = 0;

        lastACKReceived = 0;
        lastFrameSent = 0;
        nextFrameToSend = 0;
        frameno = 0;
        
        connectionAttempt = 0;
        
        clearFrameTable();
        
  #ifdef DISSEMINATION_ON
        call CommandValue.set(&currentCMD);
  #endif
        
        call AMControl.start();
        call SerialControl.start();
        call HeartbeatTimer.startPeriodic(HEARTBEAT_INTERVAL);
//        call LogWrite.sync();
        if(call Mount.mount() != SUCCESS) {
            call Leds.led0On();
            //uohhh... handle this how?
        }
    }

    event void Mount.mountDone(error_t error) {
        if (error == SUCCESS) {
            if (call Config.valid() == TRUE) {
                if (call Config.read(CONFIG_ADDR, &conf, sizeof(conf)) != SUCCESS) {
                    // Handle failure
                }
            }
            else {
                // Invalid volume.  Commit to make valid.
                call Leds.led1On();
                if (call Config.commit() == SUCCESS) {
                    call Leds.led0On();
                }
                else {
                    // Handle failure
                }
            }

            logFull = FALSE; // indicate that the log queue is ready
            
        }
        else{
            // Handle failure
        }
    }

    event void Config.readDone(storage_addr_t addr, void* buf, 
            storage_len_t len, error_t err) __attribute__((noinline)) {

        if (err == SUCCESS) {
            memcpy(&conf, buf, len);
            if (conf.version == CONFIG_VERSION) {
                // do we have to start sensing?
                if (conf.sensing)
                {
                    sensing = TRUE;
                    call RandomTimer.startOneShot((call Random.rand32()%SENSING_INTERVAL));
                } else {
                    sensing = FALSE;
                }
                
                halt = conf.halt;
                
                conf.reboot += 1;
                
                // restore global time
//                call GlobalTimeSet.set(conf.globaltime);
            }
            else {
                // Version mismatch. Restore default.
                call Leds.led1On();
                conf.version    = CONFIG_VERSION;
                conf.sensing    = DEFAULT_SENSING;
                conf.globaltime = DEFAULT_GLOBALTIME;
                conf.reboot     = DEFAULT_REBOOT;
                conf.halt        = DEFAULT_HALT;
                
            }
            call Leds.led0On();
            call Config.write(CONFIG_ADDR, &conf, sizeof(conf));
        }
        else {
            // Handle failure.
        }
    }

    event void Config.writeDone(storage_addr_t addr, void *buf, 
            storage_len_t len, error_t err) {
        // Verify addr and len

        if (err == SUCCESS) {
            if (call Config.commit() != SUCCESS) {
                // Handle failure
            }
        }
        else {
            // Handle failure
        }
    }

    event void Config.commitDone(error_t err) {
        call Leds.led0Off();
        if (err == SUCCESS) {
            // Handle failure
        }
    }

    event void AMControl.startDone(error_t err) {
        if (err == SUCCESS) {
            // Set the local wakeup interval
            call LowPowerListening.setLocalWakeupInterval(LOCAL_WAKEUP_INTERVAL);

          #ifdef DISSEMINATION_ON
            call DisseminationControl.start();
          #endif

          radioFull = FALSE;
          
        }
        else {
            call AMControl.start();
        }
    }

    event void RadioChannel.setChannelDone() {
        // Start downloading ...
        process_command();
    }

    event void AMControl.stopDone(error_t err) {
    }

    event void SerialControl.startDone(error_t err) {
        if (err == SUCCESS) {
            call BatteryRead.read();
        }
        else {
            call SerialControl.start();
        }
    }

    event void SerialControl.stopDone(error_t err) {
    }


    event void RandomTimer.fired() {
        call SensingTimer.startPeriodic(SENSING_INTERVAL);
    }

    event void HeartbeatTimer.fired() {
//        uint32_t time;

        call Leds.led1On();
        call LedOffTimer.startOneShot(LED_INTERVAL);
        if(TOS_NODE_ID == TIMESYNC_NODEID)
        {
            // we should send a status update to keep time.
            // sendStatus();
            sendStatusToController();

        }
        
        call BatteryRead.read();
/*
        // safe global time in configuration
        time  = call GlobalTime.getLocalTime();
        if(call GlobalTime.local2Global(&time) != SUCCESS) {
            // not synced... set global time to 0
            //time = 0;
            // we ignore this for now. the local storage will show is
            // this node is synced or not.
        }
        conf.globaltime = time;
        call Config.write(CONFIG_ADDR, &conf, sizeof(conf));
*/
    }

    event void LedOffTimer.fired() {
        call Leds.led0Off();
        call Leds.led1Off();
        call Leds.led2Off();
    }

    event void SensingTimer.fired() {
        // send rssi message
        uint32_t time;
        rssi_msg_t* rm = (rssi_msg_t*)call Packet.getPayload(&rssipacket, sizeof(rssi_msg_t));
        
        if(!sensing) {
            call SensingTimer.stop();
        } else {
            if(!rssilocked && !sleep) {
                rssiMaxUnlockCounter = 0;

                call Leds.led2On();

                call LedOffTimer.startOneShot(LED_INTERVAL);

                if (rm == NULL) {
                    return;
                }

                rm->counter = counter++;
                rm->src = TOS_NODE_ID;
                time  = call GlobalTime.getLocalTime();
                rm->localtime = time;
                if(call GlobalTime.local2Global(&time) != SUCCESS) {
                    // not synced... set global time to 0
                    //time = 0;
                    // we ignore this for now. the local storage will show is
                    // this node is synced or not.
                }
                rm->globaltime = time;
                // we do this in heartbeat now
                //conf.globaltime = time;
                //call Config.write(CONFIG_ADDR, &conf, sizeof(conf));

                call LowPowerListening.setRemoteWakeupInterval(&rssipacket, REMOTE_WAKEUP_INTERVAL);

                if (call RssiSend.send(AM_BROADCAST_ADDR, &rssipacket, sizeof(rssi_msg_t), rm->localtime) == SUCCESS) {
                    rssilocked = TRUE;
                }
            }
            else {
                rssiMaxUnlockCounter++;
                if (rssiMaxUnlockCounter > 1) {
                    rssilocked = FALSE;
                    rssiMaxUnlockCounter = 0;
                    
                }
            }
        }
        sleep = FALSE;
    }

    event void BatteryTimer.fired() {
        call Leds.led0On();

        call LedOffTimer.startOneShot(LED_INTERVAL);
    }

    event void StatusRandomTimer.fired() {
        sendStatusToController();
    }

    event void WRENRandomTimer.fired() {
        sendWRENStatus();
    }

    event void WRENConnectionTimer.fired() {
		if (connectionAttempt < CONNECTION_ATTEMPT) {
		    connectionAttempt++;
        	sendConnection();
        } else {
			// give up now after # of trials        	
        	call WRENConnectionTimer.stop();
        }
    }

    event void WRENTimeoutTimer.fired() {

        if (connectionAttempt < TRANSMIT_ATTEMPT) {
            connectionAttempt++;
            sendConnection();
        } else {
            // give up now after # of trials            
            call WRENConnectionTimer.stop();
        }
    }

    event void LogSendTimer.fired() {
        post radioSendTask();
    }

    event void DownloadTimer.fired() {
        #ifdef MOTE_DEBUG_MESSAGES
            if (call DiagMsg.record())
            {
                call DiagMsg.str("dt-f:1");
                call DiagMsg.send();
            }
        #endif
                
        if (logIn == logOut && !logFull)
        {
            post rssiLogRead();
        }
        else {
            call DownloadTimer.startOneShot(DOWNLOAD_BUSYWAIT_INTERVAL);
        }
    }

    task void rssiLogRead() 
    {
        if (!stopDownload) {            
            if (call LogRead.read(&m_entry, sizeof(logentry_t)) != SUCCESS) {
            }
        }            
    }

	uint8_t getFrameNo(uint32_t logsize) {
		uint8_t frameidx;
		frameidx = (currentClientLogSize - logsize) % WINDOW_BUFFER_LEN;
		return frameidx; 
	}
	
    event void LogRead.readDone(void* buf, storage_len_t len, error_t err) {
        rssi_serial_msg_t* rcm;
        
        #ifdef MOTE_DEBUG_MESSAGES
            if (call DiagMsg.record())
            {
                call DiagMsg.str("lr-rd:0");
                call DiagMsg.send();
            }
        #endif

        if ( (len == sizeof(logentry_t)) && (buf == &m_entry) ) {
            #ifdef MOTE_DEBUG_MESSAGES
            
                if (call DiagMsg.record())
                {
                    call DiagMsg.str("lr-rd:1");
                    call DiagMsg.send();
                }
            #endif
            atomic {
            #ifdef MOTE_DEBUG_MESSAGES
            
                if (call DiagMsg.record())
                {
                    call DiagMsg.str("lr-rd:2");
                    call DiagMsg.send();
                }
            #endif
//                rcm = (rssi_serial_msg_t*)call Packet.getPayload(&packet, sizeof(rssi_serial_msg_t));
	            rcm = (rssi_serial_msg_t*)call Packet.getPayload(radioQueue[radioIn], sizeof(rssi_serial_msg_t));

                memcpy(rcm, &(m_entry.msg), m_entry.len);
//                memcpy(radioQueue[radioIn], buf, len);
                rcm->size = (call LogWrite.currentOffset() - call LogRead.currentOffset()) / sizeof(logentry_t);

	            frameTable[radioIn].state = ENTRY_EMPTY;
	            frameTable[radioIn].frameno = rcm->size;
	            frameTable[radioIn].timeout = 0;
            				
            #ifdef MOTE_DEBUG_MESSAGES
            
                if (call DiagMsg.record())
                {
                    call DiagMsg.str("lr-rd:3");
                    call DiagMsg.uint32(rcm->size);
                    call DiagMsg.send();
                }
            #endif
				
				radioOut = radioIn;				

                if (++radioIn >= WINDOW_BUFFER_LEN) {
                    radioIn = 0;
                }                    

                post radioSendTask();
            }

            #ifdef MOTE_DEBUG_MESSAGES
            
                if (call DiagMsg.record())
                {
                    call DiagMsg.str("lr-rd:5");
                    call DiagMsg.uint32(nextFrameToSend);
                    call DiagMsg.uint32(lastFrameSent);
                    call DiagMsg.send();
                }
            #endif

            if (!stopDownload && (nextFrameToSend < lastFrameSent)) {            
                post rssiLogRead();
            }
        }
        else {
            // log is empty..., set the busy flag to false
            download = 1;
            
            // reset to default
            logIn = logOut = 0;
            logBusy = FALSE;
            logFull = FALSE;
                        
            sendStatus();
            
            sendStatusToController();
            
/* don't erase log            
            if (call LogWrite.erase() == SUCCESS) {
            // Reached to the end of log. Let's erase so that LogRead and LogWrite.currentOffset become equal
                isErased = 1;
                sendStatus();
            }
*/                        
        }
    }

    task void radioSendTask() {
        uint32_t time;
        
        time  = call GlobalTime.getLocalTime();
        call Leds.led2On();
        
        //call Packet.clear(radioQueue[radioOut]);
        
        #ifdef RF233_USE_SECOND_RFPOWER
            call PacketTransmitPower.set(radioQueue[radioOut], RF233_SECOND_RFPOWER);
        #endif        
        call LowPowerListening.setRemoteWakeupInterval(radioQueue[radioOut], 3);

        #ifdef MOTE_DEBUG_MESSAGES
            if (call DiagMsg.record())
            {
                call DiagMsg.str("rst:0");
                call DiagMsg.send();
            }
        #endif
                
        // Start ack timer
        if (call RssiLogSend.send(currentDST, radioQueue[radioOut], sizeof(rssi_serial_msg_t), time) == SUCCESS) {
            call Leds.led0Toggle();
        }
        else {
            // failed. try again
            post radioSendTask();
        }
    }

    event void RssiLogSend.sendDone(message_t* msg, error_t err) {

#ifdef MOTE_DEBUG_MESSAGES
    if (call DiagMsg.record())
    {
        call DiagMsg.str("rls-sd:1");
        call DiagMsg.send();
    }
#endif

        call Leds.led2Off();
        if (err == SUCCESS) {
            //call Packet.clear(&packet);
		      atomic
            if (msg == radioQueue[radioOut]) {
                
            	if (lastFrameSent > frameTable[radioOut].frameno) {
            		lastFrameSent = frameTable[radioOut].frameno;
            	}
            }

#ifdef MOTE_DEBUG_MESSAGES
    if (call DiagMsg.record())
    {
        call DiagMsg.str("rls-sd:2");
        call DiagMsg.uint32(lastFrameSent);
        call DiagMsg.send();
    }
#endif
            
            /*
		      atomic
		    if (msg == radioQueue[radioOut])
		      {
		        if (++radioOut >= WINDOW_BUFFER_LEN) {
		          radioOut = 0;
		        }
		        if (radioFull)
		          radioFull = FALSE;
		      }
		      */
		      
        }
        else {
	        post radioSendTask();
        }

        
//        if (!radioFull && !stopDownload) {
//            post rssiLogRead();
//        }        
    }
    
  event message_t * SWPAckReceive.receive(message_t *msg,
                            void *payload,
                            uint8_t len) {
    wren_swp_msg_t* rcm;

        #ifdef MOTE_DEBUG_MESSAGES
            if (call DiagMsg.record())
            {
                call DiagMsg.str("ack:0");
                call DiagMsg.send();
            }
        #endif
    
    if (call AMPacket.isForMe(msg)) {
        if (len != sizeof(wren_swp_msg_t)) {
            call Leds.led0Toggle();
            return msg;
        }
        else {
            // wren_swp_msg_t* rcm = (wren_swp_msg_t*)call Packet.getPayload(&cmdpacket, sizeof(wren_swp_msg_t));
            rcm = (wren_swp_msg_t*)payload;
            
        #ifdef MOTE_DEBUG_MESSAGES
            if (call DiagMsg.record())
            {
                call DiagMsg.str("ack:1");
                call DiagMsg.uint32(lastACKReceived);
                call DiagMsg.uint32(rcm->seqNumToAck);
                call DiagMsg.send();
            }
        #endif

            if (rcm->seqNumToAck > lastACKReceived) // ignore, we already got this ACK.
            	return msg;
            	
        #ifdef MOTE_DEBUG_MESSAGES
            if (call DiagMsg.record())
            {
                call DiagMsg.str("ack:2");
                call DiagMsg.send();
            }
        #endif

            lastACKReceived = rcm->seqNumToAck;
            // ex) seqNum = 112, next should be 111
            
            nextFrameToSend = lastACKReceived - WINDOW_BUFFER_LEN;
            // stopDownload = FALSE;

        #ifdef MOTE_DEBUG_MESSAGES
            if (call DiagMsg.record())
            {
                call DiagMsg.str("ack:3");
                call DiagMsg.uint32(lastACKReceived);
                call DiagMsg.uint32(nextFrameToSend);
                call DiagMsg.send();
            }
        #endif

			// Now read more data and send    
			if (!stopDownload && (nextFrameToSend < lastFrameSent)) 
			{        
            	post rssiLogRead();
            }
            
            //lastFrameSent = lastACKReceived + WINDOW_BUFFER_LEN;
        #ifdef MOTE_DEBUG_MESSAGES
            if (call DiagMsg.record())
            {
                call DiagMsg.str("ack:1");
                call DiagMsg.uint32(lastACKReceived);
                call DiagMsg.send();
            }
        #endif
        }
    }
    return msg;
  }

    event void RssiSend.sendDone(message_t* msg, error_t err) {
        rssilocked = FALSE;
    }

    event void SerialStatusSend.sendDone(message_t* msg, error_t err) {
        statuslocked = FALSE;
    }

    void sendStatus()
    {
        uint32_t time;
        serial_status_msg_t* sm = (serial_status_msg_t*)call Packet.getPayload(&statuspacket, sizeof(serial_status_msg_t));

        if(!statuslocked) {

            if (sm == NULL) {
                return;
            }
            sm->src = TOS_NODE_ID;
            sm->sensing = sensing;
            time  = call GlobalTime.getLocalTime();
            sm->localtime = time;
            sm->isSynced = call GlobalTime.local2Global(&time); 
            sm->globaltime = time;
            sm->buffersize = (call LogWrite.currentOffset() - call LogRead.currentOffset()) / sizeof(logentry_t);
            sm->reboots = conf.reboot;
            sm->isErased = isErased;
            sm->download = download;
            sm->channel = call RadioChannel.getChannel();
            
            atomic sm->bat            = batteryLevelVal;

            if (call SerialStatusSend.send(AM_BROADCAST_ADDR, &statuspacket, sizeof(serial_status_msg_t)) == SUCCESS) {
                statuslocked = TRUE;
            }
        }
    }

    event void LogWrite.eraseDone(error_t err) {
        if (err == SUCCESS) {
            isErased = 0;
            download = 0;
            
            // reset to default
            logIn = logOut = 0;
            logBusy = FALSE;
            logFull = FALSE;
            
            call Blink.start();
            
            // let cmdcenter know that erase is finished.
            // sendStatus();
            call StatusRandomTimer.startOneShot((call Random.rand32()%STATUS_INTERVAL));
            
        }
        else {
            // Handle error.
            isErased = 2;
        }
        
        call Leds.led0Off();
        conf.reboot = 0;
        call Config.write(CONFIG_ADDR, &conf, sizeof(conf));

    }

    void saveSensing(bool sense)
    {
        conf.sensing = sense;
        conf.halt = halt;
        call Config.write(CONFIG_ADDR, &conf, sizeof(conf));
    }

    void startSensing()
    {
        download = 0;
        sleep = FALSE;
        
        call Blink.stop();

        if (conf.sensing != TRUE) {
            call Leds.led2On();
            sensing = TRUE;
            if(TOS_NODE_ID == TIMESYNC_NODEID) {
                call RandomTimer.startOneShot((call Random.rand32()%SENSING_INTERVAL));
            } else {
                call RandomTimer.startOneShot(2*SENSING_INTERVAL + (call Random.rand32()%SENSING_INTERVAL));
            }
            saveSensing(sensing);
        } 
        else {
            sensing = TRUE;
        }       
    }

    task void changeChannelTask() {
        error_t err;
        #ifdef MOTE_DEBUG_MESSAGES
            if (call DiagMsg.record())
            {
                call DiagMsg.str("cct:1");
                call DiagMsg.send();
            }
        #endif

        err = call RadioChannel.setChannel(currentChannel);
        if (err == SUCCESS || err == EALREADY) {
            call Leds.led1Toggle();
            
            if (err == EALREADY) {
                //signal RadioChannel.setChannelDone();
            }
            
            process_command();
        }
        else {
            call Leds.led0Toggle();
            post changeChannelTask();
        }
    }

    task void startDownload() {
        download = 0;
        stopDownload = FALSE;
        sleep = FALSE;
        //call LowPowerListening.setLocalWakeupInterval(0);
        
        // When the download command comes, then change the channel and start sending 
        // rssi log message by using the channel
        // First get the channel it needs to use
        
        #ifdef MOTE_DEBUG_MESSAGES
            if (call DiagMsg.record())
            {
                call DiagMsg.str("d:");
                call DiagMsg.send();
            }
        #endif
    
        sensing = FALSE;
        smartSensingCounter = 0;
    
        // Halt all including the smart sensing
        halt = TRUE;
    
        call SensingTimer.stop();
        saveSensing(sensing);
    
        // Wait until all queued log entires are stored into log storage                    
        call DownloadTimer.startOneShot(DOWNLOAD_BUSYWAIT_INTERVAL);
    
    
    }
    
    void process_command() {
        #ifdef MOTE_DEBUG_MESSAGES
            if (call DiagMsg.record())
            {
                call DiagMsg.str("pc-r:0");
                call DiagMsg.uint16(currentCMD);
                
                call DiagMsg.send();
            }
        #endif
        
		connectionAttempt = 0;
        switch(currentCMD)
        {
            case CMD_DOWNLOAD:
                //call WRENConnectionTimer.startPeriodic(CONNECTION_TIMEOUT);
                sendConnection();
                // post startDownload();
                break;

            case CMD_ERASE:
                download = 0;
                sensing = FALSE;
                smartSensingCounter = 0;
                sleep = FALSE;                
                
                // Halt all including the smart sensing
                halt = TRUE;
                
                call SensingTimer.stop();
                saveSensing(sensing);
                
                if (call LogWrite.erase() != SUCCESS) {
                    // handle error
                } else {
                    isErased = 1;
                }
                break;

            case CMD_START_SENSE:
                // Halt all including the smart sensing
                halt = FALSE;
                startSensing();

                /*
                if(TOS_NODE_ID != TIMESYNC_NODEID) {
                    //call SerialControl.stop();
                    if (call LogWrite.erase() != SUCCESS) {
                        // handle error
                    } else {
                        isErased = 1;
                    }
                }
                */
                
                #ifdef DISSEMINATION_ON
                call CommandUpdate.change(&currentCMD);
                #endif

                break;

            case CMD_STOP_SENSE:
                call Leds.led0On();

                download = 0;
                sensing = FALSE;
                sleep = FALSE;
                                
                call SensingTimer.stop();
                saveSensing(sensing);

                #ifdef DISSEMINATION_ON
                call CommandUpdate.change(&currentCMD);
                #endif
        
                break;

            case CMD_STATUS:
                // sendStatus();
                call StatusRandomTimer.startOneShot((call Random.rand32()%STATUS_INTERVAL));

                // Send to Base
                // sendStatusToController();
                
                break;
                
            case CMD_LOGSYNC:
                download = 0;
                sleep = FALSE;
                                
                call LogWrite.sync();

                break;

            case CMD_START_BLINK:
                call Blink.start();
                break;

            case CMD_STOP_BLINK:
                call Blink.stop();
                break;

            case CMD_CHANNEL_RESET:
                stopDownload = TRUE;
                sleep = FALSE;
                
                #ifdef DISSEMINATION_ON
                call CommandUpdate.change(&currentCMD);
                #endif

                // Done before hitting here
                break;       
            case CMD_WREN_STATUS:
                call WRENRandomTimer.startOneShot((call Random.rand32()%STATUS_INTERVAL));
                
                break;         
            default:
                break;
        }
        
    }

    #ifdef DISSEMINATION_ON
    event void CommandValue.changed() {
        const uint16_t* newVal = call CommandValue.get();
        call Leds.led1Toggle();
        currentCMD = *newVal;
        process_command();            
    }
    #endif
 
    void sendWRENStatus()
    {
        if(TOS_NODE_ID != TIMESYNC_NODEID) {
            uint32_t time = call GlobalTime.getLocalTime();
            wren_status_msg_t* sm = (wren_status_msg_t*)call Packet.getPayload(&wrenpacket, sizeof(wren_status_msg_t));
    
            if(!wrenlocked) {
    
                if (sm == NULL) {
                    return;
                }
                sm->src = TOS_NODE_ID;
                sm->sensing = sensing;
                sm->buffersize = (call LogWrite.currentOffset() - call LogRead.currentOffset()) / sizeof(logentry_t);
    
    //            if (call WRENSend.send(AM_BROADCAST_ADDR, &wrenpacket, sizeof(wren_status_msg_t), time) == SUCCESS) {
                // 0 address: Controller

                #ifdef RF233_USE_SECOND_RFPOWER
                    call PacketTransmitPower.set(&wrenpacket, RF233_SECOND_RFPOWER);
                #endif        
        
                call LowPowerListening.setRemoteWakeupInterval(&wrenpacket, 0);
                if (call WRENSend.send(CONTROLLER_NODEID, &wrenpacket, sizeof(wren_status_msg_t), time) == SUCCESS) {
                    wrenlocked = TRUE;
                }
            }
        }
    }

    void sendStatusToController()
    {
        uint32_t time;
        serial_status_msg_t* sm = (serial_status_msg_t*)call Packet.getPayload(&statuspacket, sizeof(serial_status_msg_t));

        if(!cmdlocked) {

            if (sm == NULL) {
                return;
            }
            sm->src = TOS_NODE_ID;
            sm->sensing = sensing;
            time  = call GlobalTime.getLocalTime();
            sm->localtime = time;
            sm->isSynced = call GlobalTime.local2Global(&time); 
            sm->globaltime = time;
            sm->buffersize = (call LogWrite.currentOffset() - call LogRead.currentOffset()) / sizeof(logentry_t);
            sm->reboots = conf.reboot;
            sm->isErased = isErased;
            sm->download = download;
            sm->channel = call RadioChannel.getChannel();
            
            atomic sm->bat            = batteryLevelVal;

                #ifdef RF233_USE_SECOND_RFPOWER
                    call PacketTransmitPower.set(&statuspacket, RF233_SECOND_RFPOWER);
                #endif        
        
                call LowPowerListening.setRemoteWakeupInterval(&statuspacket, 0);
        
//                if (call CMDSend.send(AM_BROADCAST_ADDR, &statuspacket, sizeof(serial_status_msg_t), time) == SUCCESS) {
                if (call CMDSend.send(CONTROLLER_NODEID, &statuspacket, sizeof(serial_status_msg_t), time) == SUCCESS) {
                    cmdlocked = TRUE;
                }
            }
    }

    void sendConnection()
    {
        uint32_t time;
        wren_connection_msg_t* sm;
        time  = call GlobalTime.getLocalTime();
        sm = (wren_connection_msg_t*)call Packet.getPayload(&connectionpacket, sizeof(wren_connection_msg_t));

        #ifdef MOTE_DEBUG_MESSAGES
            if (call DiagMsg.record())
            {
                call DiagMsg.str("shs:1");
                call DiagMsg.send();
            }
        #endif

		currentClientLogSize = 0;
		
        if(!connectionlocked) {

            if (sm == NULL) {
                return;
            }
            sm->src = TOS_NODE_ID;
            sm->cmd = currentCMD;
            sm->dst = currentDST;
            sm->logsize = (call LogWrite.currentOffset() - call LogRead.currentOffset()) / sizeof(logentry_t);
            sm->channel = call RadioChannel.getChannel();
            sm->isAcked = 0;
            
            currentClientLogSize = sm->logsize;
            lastACKReceived = sm->logsize;
            lastFrameSent = sm->logsize;
            nextFrameToSend = sm->logsize;
            
            #ifdef RF233_USE_SECOND_RFPOWER
                call PacketTransmitPower.set(&connectionpacket, RF233_SECOND_RFPOWER);
            #endif        
    
            call LowPowerListening.setRemoteWakeupInterval(&connectionpacket, 3);

        #ifdef MOTE_DEBUG_MESSAGES
            if (call DiagMsg.record())
            {
                call DiagMsg.str("shs:1");
                call DiagMsg.uint16(currentDST);
                call DiagMsg.send();
            }
        #endif
    
//                if (call CMDSend.send(AM_BROADCAST_ADDR, &statuspacket, sizeof(serial_status_msg_t), time) == SUCCESS) {
            if (call ConnectionSend.send(currentDST, &connectionpacket, sizeof(wren_connection_msg_t), time) == SUCCESS) {
                connectionlocked = TRUE;
            }
        }
    }

    event void ConnectionSend.sendDone(message_t* msg, error_t err) {
        connectionlocked = FALSE;
    }

  event message_t * ConnectionReceive.receive(message_t *msg,
                            void *payload,
                            uint8_t len) {
    wren_connection_msg_t* rcm;
    
    #ifdef MOTE_DEBUG_MESSAGES
        if (call DiagMsg.record())
        {
            call DiagMsg.str("hsr:0");
            call DiagMsg.send();
        }
    #endif

    if (call AMPacket.isForMe(msg)) {
        #ifdef MOTE_DEBUG_MESSAGES
            if (call DiagMsg.record())
            {
                call DiagMsg.str("hsr:1");
                call DiagMsg.send();
            }
        #endif

        if (len != sizeof(wren_connection_msg_t)) {
            #ifdef MOTE_DEBUG_MESSAGES
                if (call DiagMsg.record())
                {
                    call DiagMsg.str("hsr:e");
                    call DiagMsg.send();
                }
            #endif
            
            call Leds.led0Toggle();
            return msg;
        }
        else {
            // stop the connection timer 
            call WRENConnectionTimer.stop();

            #ifdef MOTE_DEBUG_MESSAGES
                if (call DiagMsg.record())
                {
                    call DiagMsg.str("hsr:2");
                    call DiagMsg.send();
                }
            #endif

            // cmd_serial_msg_t* rcm = (cmd_serial_msg_t*)call Packet.getPayload(&cmdpacket, sizeof(rssi_msg_t));
    
            rcm = (wren_connection_msg_t*)payload;
            
            call Leds.led1On();
            
            currentCMD = rcm->cmd;
            currentChannel = rcm->channel;        
            currentDST = rcm->dst;
            
                #ifdef MOTE_DEBUG_MESSAGES
                
                    if (call DiagMsg.record())
                    {
                        call DiagMsg.str("hsr:3");
                        call DiagMsg.uint16(currentCMD);
                        call DiagMsg.uint16(currentChannel);
                        call DiagMsg.uint16(currentDST);
                        call DiagMsg.send();
                    }
                #endif
            
            
            if (rcm->isAcked == 1) {
                post startDownload();
            }
            else {
                // change back to normal channel 11 or random backoff and try again
            }
        }
    }
    return msg;
  }

    event void CMDSend.sendDone(message_t* msg, error_t err) {
        cmdlocked = FALSE;
    }

    event void WRENSend.sendDone(message_t* msg, error_t err) {
        wrenlocked = FALSE;
    }


  event message_t * CMDReceive.receive(message_t *msg,
                            void *payload,
                            uint8_t len) {
    cmd_serial_msg_t* rcm;
    
    #ifdef MOTE_DEBUG_MESSAGES
        if (call DiagMsg.record())
        {
            call DiagMsg.str("cmdR:0");
            call DiagMsg.send();
        }
    #endif

    if (call AMPacket.isForMe(msg)) {
        #ifdef MOTE_DEBUG_MESSAGES
            if (call DiagMsg.record())
            {
                call DiagMsg.str("cmdR:1");
                call DiagMsg.send();
            }
        #endif

        if (len != sizeof(cmd_serial_msg_t)) {
            #ifdef MOTE_DEBUG_MESSAGES
                if (call DiagMsg.record())
                {
                    call DiagMsg.str("cmdR:e");
                    call DiagMsg.send();
                }
            #endif
            
            call Leds.led0Toggle();
            return msg;
        }
        else {
            #ifdef MOTE_DEBUG_MESSAGES
                if (call DiagMsg.record())
                {
                    call DiagMsg.str("cmdR:2");
                    call DiagMsg.send();
                }
            #endif

            // cmd_serial_msg_t* rcm = (cmd_serial_msg_t*)call Packet.getPayload(&cmdpacket, sizeof(rssi_msg_t));
    
            rcm = (cmd_serial_msg_t*)payload;
            
            call Leds.led1On();
            
            currentCMD = rcm->cmd;
            currentChannel = rcm->channel;        
            currentDST = rcm->dst;

                #ifdef MOTE_DEBUG_MESSAGES
                
                    if (call DiagMsg.record())
                    {
                        call DiagMsg.str("cmdR:3");
                        call DiagMsg.uint16(currentCMD);
                        call DiagMsg.uint16(currentChannel);
                        call DiagMsg.uint16(currentDST);
                        call DiagMsg.send();
                    }
                #endif
            
            if (currentCMD == CMD_DOWNLOAD || currentCMD == CMD_CHANNEL_RESET) {    
                post changeChannelTask();
            }
            else {
               process_command();
            }
            
        }
    }
    return msg;
  }
    
    event message_t* SerialReceive.receive(message_t* msg,
            void* payload, uint8_t len) {

        if (len != sizeof(cmd_serial_msg_t)) {
            //call Leds.led1Toggle();
            return msg;
        }
        else {
            cmd_serial_msg_t* rcm = (cmd_serial_msg_t*)payload;

            call Leds.led2Toggle();

            currentCMD = rcm->cmd;
            currentChannel = rcm->channel;        
            currentDST = rcm->dst;
            
            if (currentCMD == CMD_DOWNLOAD) {    
                post changeChannelTask();
            }
            else {
               process_command();
            }

        }
        return msg;
    }

    void smartSensing()
    {
        // Better start sensing when receiving messages although not configured to sense
        if (!sensing && !halt) {
            smartSensingCounter++;
            if (smartSensingCounter > SMART_SENSING_COUNT) {
                smartSensingCounter = 0;
                startSensing();
            }
        }        
    }

    event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
        if (len != sizeof(rssi_msg_t)) {
            call Leds.led1Toggle(); 
            return msg;
        }

        #ifdef MOTE_DEBUG_MESSAGES
            if (call DiagMsg.record())
            {
                call DiagMsg.str("rec:1");
                call DiagMsg.send();
            }
        #endif
            
        atomic {
            messageCount++;
            
            if (!logFull) {

                #ifdef SMART_SENSING
                    smartSensing();
                #endif        
                
                // check if we have still space in the log
                // note, 60000 is a magic number. getSize reports too big of a
                // number. 60000 seems like a save margine
                //if ((call LogWrite.currentOffset() - call LogRead.currentOffset()) < (call LogRead.getSize() - 30000)) {
//                if ((call LogWrite.currentOffset() - call LogRead.currentOffset()) < (120000L*sizeof(logentry_t))) {
                if ((call LogWrite.currentOffset() - call LogRead.currentOffset()) < (120000L*sizeof(logentry_t))) {
                    rssi_serial_msg_t rssi_serial_m;
                    uint32_t time;
                    rssi_msg_t* rssim = (rssi_msg_t*)payload;

                    if(call TimeSyncPacket.isValid(msg)) {
                        time = call TimeSyncPacket.eventTime(msg);
                    } else {
                        time  = call GlobalTime.getLocalTime();
                    }

                    // If this message is from node 1, then delay sensing
                    if(rssim->src == TIMESYNC_NODEID || rssim->src == SLEEP_NODEID) {
                        sleep = TRUE;                        
                        call SensingTimer.stop();
                        call RandomTimer.startOneShot(5*SENSING_INTERVAL + (call Random.rand32()%SENSING_INTERVAL));
                        return msg;
                    } 
                    
                    if (sleep) {
                        return msg;
                    }

                    #ifdef MOTE_DEBUG_MESSAGES
                        if (call DiagMsg.record())
                        {
                            call DiagMsg.str("rec:2");
                            call DiagMsg.uint16(rssim->src);
                            call DiagMsg.send();
                        }
                    #endif
                    
                    rssi_serial_m.counter        = rssim->counter;
                    rssi_serial_m.dst            = TOS_NODE_ID;
                    rssi_serial_m.rssi           = call PacketRSSI.get(msg);
                    rssi_serial_m.src            = rssim->src;
                    rssi_serial_m.srclocaltime   = rssim->localtime;
                    rssi_serial_m.srcglobaltime  = rssim->globaltime;
                    rssi_serial_m.localtime      = time;
                    rssi_serial_m.isSynced       = call GlobalTime.local2Global(&time);
                    rssi_serial_m.globaltime     = time;
                    rssi_serial_m.reboot         = conf.reboot;
                    atomic rssi_serial_m.bat            = batteryLevelVal;

                    logQueue[logIn]->len = sizeof(rssi_serial_m);
                    memcpy(&(logQueue[logIn]->msg), &rssi_serial_m, logQueue[logIn]->len);
                
                    if (++logIn >= LOG_QUEUE_LEN) {
                        logIn = 0;
                    }                    

                    if (logIn == logOut)
                        logFull = TRUE;
                    
                    if (!logBusy)
                    {
                        post logWriteTask();
                        logBusy = TRUE;
                    }                
                }
            }
        }
        return msg;
    }

    task void logWriteTask() {
        // Check to see if all logs are already appended         
        atomic {
          if (logIn == logOut && !logFull)
                {
                    logBusy = FALSE;
                    return;
                }
        }

/*
        #ifdef DMOTE_DEBUG_QUEUE
            if (call DiagMsg.record())
            {
                call DiagMsg.str("l:w");
                call DiagMsg.uint16((logQueue[logOut]->msg).counter);
                call DiagMsg.uint16((logQueue[logOut]->msg).dst);
                call DiagMsg.int8((logQueue[logOut]->msg).rssi);
                call DiagMsg.uint16((logQueue[logOut]->msg).src);
//                call DiagMsg.uint32((logQueue[logOut]->msg).srclocaltime);
//                call DiagMsg.uint32((logQueue[logOut]->msg).srcglobaltime);
//                call DiagMsg.uint32((logQueue[logOut]->msg).localtime);
//                call DiagMsg.uint32((logQueue[logOut]->msg).globaltime);
//                call DiagMsg.uint8((logQueue[logOut]->msg).isSynced);
//                call DiagMsg.uint8((logQueue[logOut]->msg).reboot);
//                call DiagMsg.uint16((logQueue[logOut]->msg).bat);
                //call DiagMsg.uint32(); // size
                call DiagMsg.send();
            }
        #endif
*/
        
            if (call LogWrite.append(logQueue[logOut], sizeof(logentry_t)) == SUCCESS) {
            }
            else {
                // failed. try again
                post logWriteTask();
            }
        }

    /*
       event message_t* Snoop.receive[uint8_t id](message_t* msg, void* payload, uint8_t len) {
       return signal Receive.receive[id](msg, payload, len);
       }
       */

    event void LogWrite.appendDone(void* buf, storage_len_t len, 
            bool recordsLost, error_t err) {

        call Leds.led2Off();
        
        if (err != SUCCESS) {
        }
        else
        {
            atomic {
                if (buf == logQueue[logOut]) {
                    if (++logOut >= LOG_QUEUE_LEN) {
                        logOut = 0;
                    }
                    
                    if (logFull)
                        logFull = FALSE;
                }
            }
        }

        post logWriteTask();
    }

    event void LogRead.seekDone(error_t err) {
    }

    event void LogWrite.syncDone(error_t err) {
        if (err == SUCCESS) {
            call StatusRandomTimer.startOneShot((call Random.rand32()%STATUS_INTERVAL));
//            sendStatus();
        }
        else {
            call LogWrite.sync();
        }        
    }

    void shutdown(bool all) {
        // CRITICAL! Shut down.else {
        if (all) {
            call AMControl.stop();
            call SerialControl.stop();
        }

        call LogSendTimer.stop();
        call SensingTimer.stop();
        call HeartbeatTimer.stop();
        call LedOffTimer.stop();
        call RandomTimer.stop();
        call StatusRandomTimer.stop();
        call BatteryTimer.stop();
        call WRENRandomTimer.stop();
    }

    task void setBatteryLevel()
    {
        float b;
        atomic
        {
            b = ((float)batteryLevelVal)*4.5/4096.0;
        }

        if(b > BAT_HIGH)
        {
            batteryLevel = BAT_HIGH;
            call BatteryTimer.stop();
        } else if (b > BAT_MID) {
            if(batteryLevel != BAT_MID) {
                batteryLevel = BAT_MID;
                call BatteryTimer.startPeriodic(BAT_MID_INTERVAL);
            }
        } else if (b > BAT_LOW) {
            if(batteryLevel != BAT_LOW) {
                batteryLevel = BAT_LOW;
                call BatteryTimer.startPeriodic(BAT_LOW_INTERVAL);
            }
        } else {
            // CRITICAL! Shut down.else {
            shutdown(TRUE);
        }
    }

    event void BatteryRead.readDone(error_t result, uint16_t val){
        if (result == SUCCESS)
        {
            atomic batteryLevelVal = val;
            post setBatteryLevel();
        }
    }

    event void BatteryReadStream.readDone(error_t result, uint32_t usActualPeriod){

    }

    event void BatteryReadStream.bufferDone(error_t result, uint16_t *buf, uint16_t streamCount){

    }

    event void BatteryResource.granted(){

    }

    async event void BatteryReadNow.readDone(error_t result, uint16_t val){
        if (result == SUCCESS)
        {
            atomic batteryLevelVal = val;
            post setBatteryLevel();
        }
    }

    event void AMSend.sendDone(message_t* msg, error_t err) {

#ifdef MOTE_DEBUG_MESSAGES
    if (call DiagMsg.record())
    {
        call DiagMsg.str("as");
        call DiagMsg.send();
    }
#endif

        call Leds.led1Off();
        if ( (err == SUCCESS) && (msg == &packet) ) {
            call Packet.clear(&packet);
            if (call LogRead.read(&m_entry, sizeof(logentry_t)) != SUCCESS) {
                // Handle error.
            }
        }
        else {
            call LogSendTimer.startOneShot(INTER_PACKET_INTERVAL);
        }
    }

    
}
