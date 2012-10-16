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
        interface Packet;
        interface AMPacket;
        interface AMSend; // serial rssi send
        interface AMSend as SerialStatusSend; // serial status send
        interface Receive as SerialReceive;
        interface TimeSyncAMSend<TMilli,uint32_t> as RssiSend;
        interface Receive; // rssi receive
        //interface Receive as Snoop[uint8_t id];
        interface SplitControl as AMControl;
        interface SplitControl as SerialControl;
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
        interface Random;
        interface CC2420Packet;
        interface LowPowerListening;
        interface GlobalTime<TMilli>;
        interface TimeSyncPacket<TMilli,uint32_t>;

        interface Read<uint16_t> as BatteryRead;
        interface ReadStream<uint16_t> as BatteryReadStream;
        interface Resource as BatteryResource;
        interface ReadNow<uint16_t> as BatteryReadNow;

        interface GlobalTimeSet;
    }
}
implementation {

    enum {
        INTER_PACKET_INTERVAL = 25
    };
    
    typedef struct config_t {
        uint16_t version;
        uint8_t sensing;
        uint32_t globaltime;
        uint16_t reboot;
    } config_t;

    enum {
        CONFIG_ADDR = 0,
        CONFIG_VERSION = 7,
    };

    enum {
        DEFAULT_SENSING    = 0,
        DEFAULT_GLOBALTIME = 0,
        DEFAULT_REBOOT     = 0,
    };


    typedef nx_struct logentry_t {
        nx_uint8_t len;
        rssi_serial_msg_t msg;
    } logentry_t;

    bool m_busy = TRUE;
    logentry_t m_entry;

    message_t packet;
    message_t statuspacket;
    message_t rssipacket;

    bool sensing = FALSE;
    bool rssilocked = FALSE;
    bool statuslocked = FALSE;
    uint16_t counter = 0;
    config_t conf;
    uint16_t batteryLevel = 0xffff;
    uint16_t batteryLevelVal = 0xffff;
	uint8_t isErased = 0;
    uint8_t download = 0;

    event void Boot.booted() {
        call AMControl.start();
        call SerialControl.start();
        call HeartbeatTimer.startPeriodic(HEARTBEAT_INTERVAL);
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
                // do we have to start sensing? This is a sleep sensor. Just start
//                if (conf.sensing)
//                {
                    sensing = TRUE;
                    call RandomTimer.startOneShot((call Random.rand32()%SENSING_INTERVAL));
//                } else {
//                    sensing = FALSE;
//                }
                conf.reboot += 1;
                // restore global time
//                call GlobalTimeSet.set(conf.globaltime);
            }
            else {
                // Version mismatch. Restore default.
                call Leds.led1On();
                conf.version    = CONFIG_VERSION;
                conf.sensing    = TRUE;
                conf.globaltime = DEFAULT_GLOBALTIME;
                conf.reboot     = DEFAULT_REBOOT;
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
            m_busy = FALSE;
            // Set the local wakeup interval
            call LowPowerListening.setLocalWakeupInterval(LOCAL_WAKEUP_INTERVAL);


        }
        else {
            call AMControl.start();
        }
    }


    event void AMControl.stopDone(error_t err) {
    }

    event void SerialControl.startDone(error_t err) {
        if (err == SUCCESS) {
        }
        else {
            call SerialControl.start();
        }
    }

    event void SerialControl.stopDone(error_t err) {
    }

    event void LogRead.readDone(void* buf, storage_len_t len, error_t err) {
        if ( (len == sizeof(logentry_t)) && (buf == &m_entry) ) {
            rssi_serial_msg_t* rcm = (rssi_serial_msg_t*)call Packet.getPayload(&packet, sizeof(rssi_serial_msg_t));
            memcpy(rcm, &(m_entry.msg), m_entry.len);
            rcm->size = (call LogWrite.currentOffset() - call LogRead.currentOffset()) / sizeof(logentry_t);
            //rcm->size = call LogRead.getSize() / sizeof(logentry_t);
            call Leds.led1On();
            if (call AMSend.send(0, &packet, m_entry.len) != SUCCESS) {
                // uohhh. bad bad bad
                m_busy = FALSE;
            }
        }
        else {
            m_busy = FALSE;
            // log is empty...
        }
    }


    event void AMSend.sendDone(message_t* msg, error_t err) {
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
        rssi_serial_msg_t* rcm = (rssi_serial_msg_t*)call Packet.getPayload(&packet, sizeof(rssi_serial_msg_t));
        memcpy(rcm, &(m_entry.msg), m_entry.len);
        rcm->size = (call LogWrite.currentOffset() - call LogRead.currentOffset()) / sizeof(logentry_t);
        call AMSend.send(0, &packet, m_entry.len);
    }

    event void RandomTimer.fired() {
        call SensingTimer.startPeriodic(SENSING_INTERVAL);
    }

    event void SensingTimer.fired() {
        // send rssi message
        uint32_t time;
        rssi_msg_t* rm = (rssi_msg_t*)call Packet.getPayload(&rssipacket, sizeof(rssi_msg_t));

        if(!rssilocked) {

            call Leds.led2Toggle();

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
    }

    event void BatteryTimer.fired() {
        call Leds.led0On();

        call LedOffTimer.startOneShot(LED_INTERVAL);
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
            sm->channel = CC2420_DEF_CHANNEL;
            
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
            sendStatus();

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
            m_busy = FALSE;
            isErased = 0;
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
        call Config.write(CONFIG_ADDR, &conf, sizeof(conf));
    }

    event message_t* SerialReceive.receive(message_t* msg,
            void* payload, uint8_t len) {

        if (len != sizeof(cmd_serial_msg_t)) {
            //call Leds.led1Toggle();
            return msg;
        }
        else {
            cmd_serial_msg_t* rcm = (cmd_serial_msg_t*)payload;

            switch(rcm->cmd)
            {
                case CMD_DOWNLOAD:
                    //call Leds.led0Toggle();
                    if (call LogRead.read(&m_entry, sizeof(logentry_t)) != SUCCESS) {
                        m_busy = FALSE;
                    } else {
                        m_busy = TRUE;
                    }
                    break;

                case CMD_ERASE:
                    if (call LogWrite.erase() != SUCCESS) {
                        // handle error
                    } else {
                        m_busy = TRUE;
                        isErased = 1;
                    }
                    break;

                case CMD_START_SENSE:
                    sensing = TRUE;
                    if(TOS_NODE_ID == TIMESYNC_NODEID) {
                        call RandomTimer.startOneShot((call Random.rand32()%SENSING_INTERVAL));
                    } else {
                        call RandomTimer.startOneShot(2*SENSING_INTERVAL + (call Random.rand32()%SENSING_INTERVAL));
                    }
                    saveSensing(sensing);
                    if(TOS_NODE_ID != TIMESYNC_NODEID) {
                        call SerialControl.stop();
                    }
                    
                    break;

                case CMD_STOP_SENSE:
                    sensing = FALSE;
                    call SensingTimer.stop();
                    saveSensing(sensing);

                    break;

                case CMD_STATUS:
                    sendStatus();
                    break;
                    
                default:
                    break;
            }

        }
        return msg;
    }

    event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
        if (len != sizeof(rssi_msg_t)) {
            call Leds.led1Toggle(); 
            return msg;
        }

        if (!m_busy) {
            rssi_serial_msg_t rssi_serial_m;
            uint32_t time;
            rssi_msg_t* rssim = (rssi_msg_t*)payload;
            if(call TimeSyncPacket.isValid(msg)) {
                time = call TimeSyncPacket.eventTime(msg);
            } else {
                time  = call GlobalTime.getLocalTime();
            }

            // If this message is from node 1, then delay sensing
            if(rssim->src == TIMESYNC_NODEID) {
                call SensingTimer.stop();
                call RandomTimer.startOneShot(2*SENSING_INTERVAL + (call Random.rand32()%SENSING_INTERVAL));
                return msg;
            }
            m_busy = TRUE;

            rssi_serial_m.counter        = rssim->counter;
            rssi_serial_m.dst            = TOS_NODE_ID;
            rssi_serial_m.rssi           = call CC2420Packet.getRssi(msg);
            rssi_serial_m.src            = rssim->src;
            rssi_serial_m.srclocaltime   = rssim->localtime;
            rssi_serial_m.srcglobaltime  = rssim->globaltime;
            rssi_serial_m.localtime      = time;
            rssi_serial_m.isSynced       = call GlobalTime.local2Global(&time);
            rssi_serial_m.globaltime     = time;
            rssi_serial_m.reboot         = conf.reboot;
            atomic rssi_serial_m.bat            = batteryLevelVal;

            m_entry.len = sizeof(rssi_serial_m);
            memcpy(&(m_entry.msg), &rssi_serial_m, m_entry.len);
            if (call LogWrite.append(&m_entry, sizeof(logentry_t)) != SUCCESS) {
                m_busy = FALSE;
            }
        }
        return msg;
    }

    /*
       event message_t* Snoop.receive[uint8_t id](message_t* msg, void* payload, uint8_t len) {
       return signal Receive.receive[id](msg, payload, len);
       }
       */

    event void LogWrite.appendDone(void* buf, storage_len_t len, 
            bool recordsLost, error_t err) {
        m_busy = FALSE;
        call Leds.led2Off();
    }

    event void LogRead.seekDone(error_t err) {
    }

    event void LogWrite.syncDone(error_t err) {
    }


    task void setBatteryLevel()
    {
        float b;
        atomic
        {
            b = ((float)batteryLevelVal)*5/4096.0;
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
        		call AMControl.stop();
        		call SerialControl.stop();
        		call Timer0.stop();
        		call SensingTimer.stop();
        		call HeartbeatTimer.stop();
        		call LedOffTimer.stop();
        		call RandomTimer.stop();
        		call BatteryTimer.stop();
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
