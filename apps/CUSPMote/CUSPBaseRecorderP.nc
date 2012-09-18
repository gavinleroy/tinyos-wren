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

        interface SplitControl as SerialControl;
        interface AMSend; // serial rssi send
        interface AMSend as SerialStatusSend; // serial status send
        interface Receive as SerialReceive;
        
        //interface Receive as Snoop[uint8_t id];
        interface LogRead;
        interface LogWrite;
        interface ConfigStorage as Config;
        interface Mount as Mount;
        
        interface Timer<TMilli> as Timer0;
        interface Timer<TMilli> as SensingTimer;
        interface Timer<TMilli> as HeartbeatTimer;
        interface Timer<TMilli> as LedOffTimer;
        interface Timer<TMilli> as RandomTimer;
        interface Timer<TMilli> as BatteryTimer;
        interface Timer<TMilli> as DownloadTimer;
        interface Timer<TMilli> as StatusRandomTimer;
        interface Timer<TMilli> as WRENRandomTimer;
        
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

    bool sensing = FALSE;
    bool rssilocked = FALSE;
    bool statuslocked = FALSE;
    bool cmdlocked = FALSE;
    bool wrenlocked = FALSE;

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
    
    logentry_t logQueueData[LOG_QUEUE_LEN];
    logentry_t *logQueueDataPt[LOG_QUEUE_LEN];
    uint8_t logQueueStart, logQueueEnd;
    bool logWriteBusy, logQueueFull;

    uint16_t currentCommand;
    uint8_t currentChannel;
    uint16_t currentdst;
    
    task void logWriteTask();
    void sendStatus();
    void sendStatusToBase();
    void sendWRENStatus();
    void shutdown(bool all);
    void saveSensing(bool sense);
    task void changeChannelTask();
    void process_command();
    
    event void Boot.booted() {
        uint8_t i;
        // Create Log Queue memory storage            
        for (i = 0; i < LOG_QUEUE_LEN; i++)
          logQueueDataPt[i] = &logQueueData[i];
          
        logQueueStart = logQueueEnd = 0;
        logWriteBusy = FALSE;
        logQueueFull = TRUE;
        
        currentCommand = CMD_NONE;
        currentChannel = RF233_DEF_CHANNEL;
        currentdst = AM_BROADCAST_ADDR; //default download base station in case
        
        sleep = FALSE;
        messageCount = 0;
        
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

            logQueueFull = FALSE; // indicate that the log queue is ready
            
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

    event void LogRead.readDone(void* buf, storage_len_t len, error_t err) {
        uint32_t time;
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
        
            rcm = (rssi_serial_msg_t*)call Packet.getPayload(&packet, sizeof(rssi_serial_msg_t));
            
            memcpy(rcm, &(m_entry.msg), m_entry.len);
            
            rcm->size = (call LogWrite.currentOffset() - call LogRead.currentOffset()) / sizeof(logentry_t);
            //rcm->size = call LogRead.getSize() / sizeof(logentry_t);

            time  = call GlobalTime.getLocalTime();

/*
                call Leds.led1On();
                if (call AMSend.send(0, &packet, m_entry.len) != SUCCESS) {
                    // uohhh. bad bad bad
                    // retry
                }
*/
                
                call Leds.led1On();
                
                call LowPowerListening.setRemoteWakeupInterval(&packet, 0);

                if (call RssiLogSend.send(currentdst, &packet, m_entry.len, time) == SUCCESS) {
		            #ifdef MOTE_DEBUG_MESSAGES
		            
		                if (call DiagMsg.record())
		                {
		                    call DiagMsg.str("rls-s:s");
		                    call DiagMsg.send();
		                }
		            #endif
                }
                else {
		            #ifdef MOTE_DEBUG_MESSAGES
		            
		                if (call DiagMsg.record())
		                {
		                    call DiagMsg.str("rls-s:e");
		                    call DiagMsg.send();
		                }
		            #endif
                }
        
//                if (call RssiLogSend.send(AM_BROADCAST_ADDR, &packet, sizeof(rssi_serial_msg_t), time) == SUCCESS) {
//                }
                
        }
        else {
            // log is empty..., set the busy flag to false
            download = 1;
            
            // reset to default
            logQueueStart = logQueueEnd = 0;
            logWriteBusy = FALSE;
            logQueueFull = FALSE;
                        
            sendStatus();
            
            sendStatusToBase();
            
/* don't erase log            
            if (call LogWrite.erase() == SUCCESS) {
            // Reached to the end of log. Let's erase so that LogRead and LogWrite.currentOffset become equal
                isErased = 1;
                sendStatus();
            }
*/                        
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

        call Leds.led1Off();
        if ( (err == SUCCESS) && (msg == &packet) ) {
            call Packet.clear(&packet);

            if (!stopDownload) {            
	            if (call LogRead.read(&m_entry, sizeof(logentry_t)) != SUCCESS) {
	                // Handle error.
			        #ifdef MOTE_DEBUG_MESSAGES
			            if (call DiagMsg.record())
			            {
			                call DiagMsg.str("rls-sd:e");
			                call DiagMsg.send();
			            }
			        #endif
	            }
           }
        }
        else {
            if (!stopDownload) {
                call Timer0.startOneShot(INTER_PACKET_INTERVAL);
            }
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
            call Timer0.startOneShot(INTER_PACKET_INTERVAL);
        }
    }

    event void RssiSend.sendDone(message_t* msg, error_t err) {
        rssilocked = FALSE;
    }

    event void SerialStatusSend.sendDone(message_t* msg, error_t err) {
        statuslocked = FALSE;
    }

    event void Timer0.fired() {
        uint32_t time;
        rssi_serial_msg_t* rcm;
        
        time  = call GlobalTime.getLocalTime();
        rcm = (rssi_serial_msg_t*)call Packet.getPayload(&packet, sizeof(rssi_serial_msg_t));
//        memcpy(rcm, &(m_entry.msg), m_entry.len);
//        rcm->size = (call LogWrite.currentOffset() - call LogRead.currentOffset()) / sizeof(logentry_t);
//        call AMSend.send(0, &packet, m_entry.len);

        call LowPowerListening.setRemoteWakeupInterval(&packet, 0);
        if (call RssiLogSend.send(currentdst, &packet, sizeof(rssi_serial_msg_t), time) == SUCCESS) {
        }      
    }

    event void RandomTimer.fired() {
        call SensingTimer.startPeriodic(SENSING_INTERVAL);
    }

    event void SensingTimer.fired() {
        // send rssi message
        uint32_t time;
        rssi_msg_t* rm = (rssi_msg_t*)call Packet.getPayload(&rssipacket, sizeof(rssi_msg_t));
        
        if(!sensing) {
            call SensingTimer.stop();
        } else {
            if(!rssilocked) {
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
    }

    event void BatteryTimer.fired() {
        call Leds.led0On();

        call LedOffTimer.startOneShot(LED_INTERVAL);
    }

    event void StatusRandomTimer.fired() {
        sendStatusToBase();
    }

    event void WRENRandomTimer.fired() {
        sendWRENStatus();
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

    event void HeartbeatTimer.fired() {
//        uint32_t time;

        call Leds.led1On();
        call LedOffTimer.startOneShot(LED_INTERVAL);
        if(TOS_NODE_ID == TIMESYNC_NODEID)
        {
            // we should send a status update to keep time.
            // sendStatus();
            sendStatusToBase();

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

    event void LogWrite.eraseDone(error_t err) {
        if (err == SUCCESS) {
            isErased = 0;
            download = 0;
            
            // reset to default
            logQueueStart = logQueueEnd = 0;
            logWriteBusy = FALSE;
            logQueueFull = FALSE;
            
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

    event void DownloadTimer.fired() {
        #ifdef MOTE_DEBUG_MESSAGES
            if (call DiagMsg.record())
            {
                call DiagMsg.str("dt-f:1");
                call DiagMsg.send();
            }
        #endif
                
        if (logQueueStart == logQueueEnd && !logQueueFull)
        {
            if (call LogRead.read(&m_entry, sizeof(logentry_t)) != SUCCESS) {
                // need to do somthing
            }
        }
        else {
            call DownloadTimer.startOneShot(DOWNLOAD_BUSYWAIT_INTERVAL);
        }
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

        if (conf.sensing != TRUE) {
            sensing = TRUE;
            if(TOS_NODE_ID == TIMESYNC_NODEID) {
                call RandomTimer.startOneShot((call Random.rand32()%SENSING_INTERVAL));
            } else {
                call RandomTimer.startOneShot(2*SENSING_INTERVAL + (call Random.rand32()%SENSING_INTERVAL));
            }
            saveSensing(sensing);
        }        
    }

    task void changeChannelTask() {
        error_t err;
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

    void process_command() {
        #ifdef MOTE_DEBUG_MESSAGES
            if (call DiagMsg.record())
            {
                call DiagMsg.str("pc:0");
                call DiagMsg.uint16(currentCommand);
                
                call DiagMsg.send();
            }
        #endif
        
        switch(currentCommand)
        {
            case CMD_DOWNLOAD:
                download = 0;
                stopDownload = FALSE;
	            call LowPowerListening.setLocalWakeupInterval(0);
                
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
            
                break;

            case CMD_ERASE:
                download = 0;
                sensing = FALSE;
                smartSensingCounter = 0;
                
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
                
                break;

            case CMD_STOP_SENSE:
                download = 0;
                sensing = FALSE;
                call SensingTimer.stop();
                saveSensing(sensing);

                break;

            case CMD_STATUS:
                // sendStatus();
                call StatusRandomTimer.startOneShot((call Random.rand32()%STATUS_INTERVAL));

		        // Send to Base
		        // sendStatusToBase();
                
                break;
                
            case CMD_LOGSYNC:
                download = 0;
                call LogWrite.sync();

                break;

            case CMD_START_BLINK:
                call Leds.led1Toggle();
                break;

            case CMD_CHANNEL_RESET:
                stopDownload = TRUE;
                // Done before hitting here
                break;       
            case CMD_WREN_STATUS:
                call WRENRandomTimer.startOneShot((call Random.rand32()%STATUS_INTERVAL));
                
                break;         
            default:
                break;
        }
        
    }

    void sendWRENStatus()
    {
        if(TOS_NODE_ID != TIMESYNC_NODEID) {
	        uint32_t time = call GlobalTime.getLocalTime();
	        wren_status_msg_t* sm = (wren_status_msg_t*)call Packet.getPayload(&wrenpacket, sizeof(wren_status_msg_t));
	
	//        call PacketTransmitPower.set(&wrenpacket, RF233_SECOND_RFPOWER);
	
	        if(!wrenlocked) {
	
	            if (sm == NULL) {
	                return;
	            }
	            sm->src = TOS_NODE_ID;
	            sm->sensing = sensing;
	            sm->buffersize = (call LogWrite.currentOffset() - call LogRead.currentOffset()) / sizeof(logentry_t);
	
	//            if (call WRENSend.send(AM_BROADCAST_ADDR, &wrenpacket, sizeof(wren_status_msg_t), time) == SUCCESS) {
	            // 0 address: Controller
                call LowPowerListening.setRemoteWakeupInterval(&wrenpacket, 0);
	            if (call WRENSend.send(0, &wrenpacket, sizeof(wren_status_msg_t), time) == SUCCESS) {
	                wrenlocked = TRUE;
	            }
	        }
        }
    }

    void sendStatusToBase()
    {
        uint32_t time;
        serial_status_msg_t* sm = (serial_status_msg_t*)call Packet.getPayload(&statuspacket, sizeof(serial_status_msg_t));

//        call PacketTransmitPower.set(&statuspacket, RF233_SECOND_RFPOWER);

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

		        call LowPowerListening.setRemoteWakeupInterval(&statuspacket, 0);
		
//                if (call CMDSend.send(AM_BROADCAST_ADDR, &statuspacket, sizeof(serial_status_msg_t), time) == SUCCESS) {
		        if (call CMDSend.send(0, &statuspacket, sizeof(serial_status_msg_t), time) == SUCCESS) {
		            cmdlocked = TRUE;
		        }
	        }
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
    
    call Leds.led1Toggle();

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
	        
	        call Leds.led2Toggle();
	        
	        currentCommand = rcm->cmd;
	        currentChannel = rcm->channel;        
	        currentdst = rcm->dst;
	        
	        if (currentCommand == CMD_DOWNLOAD || currentCommand == CMD_CHANNEL_RESET) {    
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

            currentCommand = rcm->cmd;
            currentChannel = rcm->channel;        
            currentdst = rcm->dst;
            
            if (currentCommand == CMD_DOWNLOAD) {    
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
            
            if (!logQueueFull && !sleep) {

                #ifdef SMART_SENSING
                    smartSensing();
                #endif        
                
                // check if we have still space in the log
                // note, 60000 is a magic number. getSize reports too big of a
                // number. 60000 seems like a save margine
                //if ((call LogWrite.currentOffset() - call LogRead.currentOffset()) < (call LogRead.getSize() - 30000)) {
//                if ((call LogWrite.currentOffset() - call LogRead.currentOffset()) < (60000L*sizeof(logentry_t))) {
                if ((call LogWrite.currentOffset() - call LogRead.currentOffset()) < (60000L*sizeof(logentry_t))) {
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
                        call SensingTimer.stop();
                        call RandomTimer.startOneShot(2*SENSING_INTERVAL + (call Random.rand32()%SENSING_INTERVAL));
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

                    logQueueDataPt[logQueueStart]->len = sizeof(rssi_serial_m);
                    memcpy(&(logQueueDataPt[logQueueStart]->msg), &rssi_serial_m, logQueueDataPt[logQueueStart]->len);
                
                    if (++logQueueStart >= LOG_QUEUE_LEN) {
                        logQueueStart = 0;
                    }                    

                    if (logQueueStart == logQueueEnd)
                        logQueueFull = TRUE;
                    
                    if (!logWriteBusy)
                    {
                        post logWriteTask();
                        logWriteBusy = TRUE;
                    }                
                }
            }
        }
        return msg;
    }

    task void logWriteTask() {
        // Check to see if all logs are already appended         
        atomic {
          if (logQueueStart == logQueueEnd && !logQueueFull)
                {
                    logWriteBusy = FALSE;
                    return;
                }
        }

/*
        #ifdef DMOTE_DEBUG_QUEUE
            if (call DiagMsg.record())
            {
                call DiagMsg.str("l:w");
                call DiagMsg.uint16((logQueueDataPt[logQueueEnd]->msg).counter);
                call DiagMsg.uint16((logQueueDataPt[logQueueEnd]->msg).dst);
                call DiagMsg.int8((logQueueDataPt[logQueueEnd]->msg).rssi);
                call DiagMsg.uint16((logQueueDataPt[logQueueEnd]->msg).src);
//                call DiagMsg.uint32((logQueueDataPt[logQueueEnd]->msg).srclocaltime);
//                call DiagMsg.uint32((logQueueDataPt[logQueueEnd]->msg).srcglobaltime);
//                call DiagMsg.uint32((logQueueDataPt[logQueueEnd]->msg).localtime);
//                call DiagMsg.uint32((logQueueDataPt[logQueueEnd]->msg).globaltime);
//                call DiagMsg.uint8((logQueueDataPt[logQueueEnd]->msg).isSynced);
//                call DiagMsg.uint8((logQueueDataPt[logQueueEnd]->msg).reboot);
//                call DiagMsg.uint16((logQueueDataPt[logQueueEnd]->msg).bat);
                //call DiagMsg.uint32(); // size
                call DiagMsg.send();
            }
        #endif
*/
        
            if (call LogWrite.append(logQueueDataPt[logQueueEnd], sizeof(logentry_t)) == SUCCESS) {
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
                if (buf == logQueueDataPt[logQueueEnd]) {
                    if (++logQueueEnd >= LOG_QUEUE_LEN) {
                        logQueueEnd = 0;
                    }
                    
                    if (logQueueFull)
                        logQueueFull = FALSE;
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

        call Timer0.stop();
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
}
