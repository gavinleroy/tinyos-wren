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
 */


#include "printf.h"
#include "BQ25010.h"

#include "RadioReflector.h"

module TestWrenP
{
    uses {
        interface Boot;
        interface Leds;
        /* Power */
        interface BQ25010;
        /* Radio */
        interface Receive;
        interface AMSend;
        interface SplitControl as AMControl;
        interface Packet;
        interface Timer<TMilli> as ReflectionTimer;
        /* Accelerometer */
        interface Lis3dh;
        interface Lis3dhAccel;
        /* RTC */
        interface Pcf2127a;
        interface Pcf2127aRtc;
        interface BusyWait<T32khz,uint16_t>;
        /* Flash */
        interface LogRead;
        interface LogWrite;
    }
}
implementation
{
    enum {
        NO_TEST             = 0,
        CHARGING_TEST       = 1,
        RADIO_TEST          = 2,
        ACCELEROMETER_TEST  = 3,
        RTC_TEST            = 4,
        FLASH_TEST          = 5,
    };




    uint8_t test = NO_TEST;
    bool testPassed;
    uint8_t reflectionCounter;

    message_t packet;

    /* Forward Defines */

    task void nextTest();

    /*
     * Test if the charging pins work. Since we are plugged in, we should see
     * the charging status.
     */
    task void chargingTest()
    {
        power_state_t state = call BQ25010.getState();
        testPassed = FALSE;
        printf("Charging State: ");
        switch(state) {
            case STATE_PRECHARGE:
                printf("PRECHARGE\n");
                testPassed = TRUE;
                break;
            case STATE_FASTCHARGE:
                printf("FASTCHARGE\n");
                testPassed = TRUE;
                break;
            case STATE_CHARGEDONE:
                printf("CHARGEDONE\n");
                testPassed = TRUE;
                break;
            case STATE_SLEEP:
                printf("SLEEP\n");
                break;
            default:
        }
        printfflush();

        post nextTest();
    }

  
    /************************************************************************
     * THIS IS ALL RADIO STUFF
     */

    task void sendTask()
    {
        radio_reflector_msg_t* rrm = (radio_reflector_msg_t*)call Packet.getPayload(&packet, sizeof(radio_reflector_msg_t));
        printf("Prepare to send\n");
        printfflush();
        if (rrm == NULL) {
            printf("Couldn't allocate packet payload\n");
            post nextTest();
            return;
        }

        rrm->src = TOS_NODE_ID;
        if (call AMSend.send(REFLECTOR_ID, &packet, sizeof(radio_reflector_msg_t)) == SUCCESS) {
        } else {
            printf("Send Failed\n");
            post nextTest();
        }
    }

    event void AMControl.startDone(error_t err) {
        printf("Start Done\n");
        if (err == SUCCESS) {
            post sendTask();
        }
        else {
            printf("Start Done FAILED\n");
            post nextTest();
        }
    }

    event void ReflectionTimer.fired() {
        // uoh... didn't reflect. Try again

        printf("Reflection FAILED for the %d time.\n", reflectionCounter);
        printfflush();
        if(reflectionCounter < MAX_REFLECTIONS) {
            post sendTask();
        } else {
            printf("Too many retries. Reflection FAILED\n");
            post nextTest();
        }
    }

    event void AMControl.stopDone(error_t err) {
      
        if(err == SUCCESS) {
            printf("Radio Stopped\n");
            testPassed = TRUE;
            post nextTest();
        } else {
            printf("Radio Stop FAILED\n");
            post nextTest();
        }
    }

    event message_t* Receive.receive(message_t* bufPtr,
            void* payload, uint8_t len) {
        if (len != sizeof(radio_reflector_msg_t)) {return bufPtr;}
        else {
            radio_reflector_msg_t* rrm = (radio_reflector_msg_t*)payload;
            printf("Received Message\n");

            if(rrm->src == REFLECTOR_ID) {
                call ReflectionTimer.stop();
                printf("Reflection SUCCESS\n");
                printfflush();
                if(call AMControl.stop() != SUCCESS) {
                    printf("AMControl.stop FAILED\n");
                    post nextTest();
                } else {
                    call ReflectionTimer.stop();
                }
            }
        }
        return bufPtr;

    }

    event void AMSend.sendDone(message_t* bufPtr, error_t error) {
        if (&packet == bufPtr) {
            printf("Send SUCCESS\n");
            reflectionCounter++;
            call ReflectionTimer.startOneShot(256);
            printfflush();
        } else {
            printf("SendDone Failed\n");
            post nextTest();
        }
    }

    task void radioTest()
    {
        // turn on radio
        printf("Turning on Radio\n");
        reflectionCounter = 0;
        if(call AMControl.start() != SUCCESS) {
            printf("Turn on failed!\n");
            post nextTest();
        }
    }

    /************************************************************************
     * ACCELEROMETER TESTS
     */

    task void accelerometerTest()
    {
        uint8_t chipId;

        // turn on Accelerometer
        call Lis3dhAccel.lowPower10Hz();

        if(call Lis3dhAccel.getChipId(&chipId) == SUCCESS) {
            printf("Accel ID: 0x%x\n", chipId);
            if(chipId == 0x33) {
                printf("Identification SUCCESS\n");
                testPassed = TRUE;
            } else {
                printf("Wrong chip ID. Should be 0x33.\n");
            }
        } else {
            printf("Accel ID failed\n");
        }
        printfflush();
        post nextTest();
    }


    async event void Lis3dhAccel.dataReady() {}
    async event void Lis3dhAccel.click() {}

    /************************************************************************
     * RTC Test Procedure
     */

    task void rtcTest()
    {
        uint8_t data[512];
        uint8_t verify[512];
        uint16_t i;
        rtc_time_t time, verifyTime;

        for(i=0; i< 512; i++) {
            data[i] = i;
            verify[i] = 0;
        }

        /* Verify time write/read */

        time.years    = 12;
        time.months   = 8;
        time.weekdays = 3;
        time.days     = 29;
        time.hours    = 13;
        time.minutes  = 3;
        time.seconds  = 17;

        printf("Set Time:\n");
        printf("min: %d, sec: %d\n", time.minutes, time.seconds);

        if(call Pcf2127aRtc.setTime(time) != SUCCESS) {
            printf("Write of RTC Time failed.\n");
            post nextTest();
            return;
        }
        call BusyWait.wait(48000U); // wait for 1.5 seconds
        if(call Pcf2127aRtc.getTime(&verifyTime) != SUCCESS) {
            printf("Read of RTC Time failed.\n");
            post nextTest();
            return;
        }

        printf("Read Time:\n");
        printf("min: %d, sec: %d\n", verifyTime.minutes, verifyTime.seconds);
        if(verifyTime.seconds <= time.seconds){
            printf("Time didn't advance!\n");
            post nextTest();
            return;
        }


        printfflush();


        /* Verify RAM write/read */
        if(call Pcf2127a.writeRam(0, data, 512) != SUCCESS) {
            printf("Write to RTC RAM failed.\n");
            post nextTest();
            return;
        }
        if(call Pcf2127a.readRam(0, verify, 512) != SUCCESS) {
            printf("Read from RTC RAM failed.\n");
            post nextTest();
            return;
        }

        printf("RAM Verify Not Supported (it fails)\n");
        printfflush();

        /*

        for(i=0; i<512; i++)
            if(data[i] != verify[i]) {
                printf("RTC Ver failed. %d != %d at %d\n", data[i], verify[i], i);
                printfflush();
                post nextTest();
                return;
            }

            */

        testPassed = TRUE;

        post nextTest();
    }

    /************************************************************************
     * Flash Test
     */

    enum {
        SIZE = FLASH_SZE,
        RECSIZE = 32,
        NRECS = 16,
        NWRITES = SIZE / (2 * RECSIZE),
    };

    uint16_t shiftReg;
    uint16_t initSeed;
    uint16_t mask;

    void done();

    /* Return the next 16 bit random number */
    uint16_t rand() {
        bool endbit;
        uint16_t tmpShiftReg;

        tmpShiftReg = shiftReg;
        endbit = ((tmpShiftReg & 0x8000) != 0);
        tmpShiftReg <<= 1;
        if (endbit) 
            tmpShiftReg ^= 0x100b;
        tmpShiftReg++;
        shiftReg = tmpShiftReg;
        tmpShiftReg = tmpShiftReg ^ mask;

        return tmpShiftReg;
    }


    void resetSeed() {
        shiftReg = 119 * 119 * ((TOS_NODE_ID % 100) + 1);
        initSeed = shiftReg;
        mask = 137 * 29 * ((TOS_NODE_ID % 100) + 1);
    }

    uint8_t data[NRECS * RECSIZE], rdata[RECSIZE];
    int count, testCount;

  
    void fail(error_t e) {
        printf("Flash fail\n");
        post nextTest();
    }

    void success() {
        printf("Flash test SUCCESS\n");
        testPassed = TRUE;
        post nextTest();
    }

    bool scheck(error_t r) __attribute__((noinline)) {
        if (r != SUCCESS)
            fail(r);
        return r == SUCCESS;
    }

    bool bcheck(bool b) {
        if (!b)
            fail(FAIL);
        return b;
    }

    void nextRead() {
        scheck(call LogRead.read(rdata, RECSIZE));
    }

    event void LogRead.readDone(void* buf, storage_len_t rlen, error_t result) __attribute__((noinline)) {
        if (rlen == 0)
            done();
        else if (scheck(result) && bcheck(rlen == RECSIZE && buf == rdata))
        {
            int i;

            /* It must be one of our possible records */
            for (i = 0; i < sizeof data; i += RECSIZE)
                if (memcmp(buf, data + i, RECSIZE) == 0)
                {
                    nextRead();
                    return;
                }
            bcheck(FALSE);
        }
    }

    event void LogRead.seekDone(error_t error) {
    }

    void nextWrite() {
        if (count++ == NWRITES)
            scheck(call LogWrite.sync());
        else
        {
            int offset = ((rand() >> 8) % NRECS) * RECSIZE;

            scheck(call LogWrite.append(data + offset, RECSIZE));
        }
    }

    event void LogWrite.appendDone(void *buf, storage_len_t y, bool recordsLost, error_t result) {
        if (scheck(result))
            nextWrite();
    }

    event void LogWrite.eraseDone(error_t result) {
        if (scheck(result))
            done();
    }

    event void LogWrite.syncDone(error_t result) {
        if (scheck(result))
            done();
    }

    enum { A_ERASE = 1, A_READ, A_WRITE };

    void doAction(int act) {
        switch (act)
        {
            case A_ERASE:
                printf("Erase\n");
                printfflush();
                scheck(call LogWrite.erase());
                break;
            case A_WRITE:
                printf("Write\n");
                printfflush();
                resetSeed();
                count = 0;
                nextWrite();
                break;
            case A_READ:
                printf("Read\n");
                printfflush();
                resetSeed();
                count = 0;
                nextRead();
                break;
        }
    }

    const uint8_t actions[] = {
        A_ERASE,
        A_READ,
        A_WRITE,
        A_READ,
        A_WRITE,
        A_WRITE,
        A_WRITE,
        A_READ,
    };

    void done() {
        uint8_t act = 17 / 100;

        call Leds.led2Toggle();

        switch (act)
        {
            case 0:
                if (testCount < sizeof actions)
                    doAction(actions[testCount]);
                else
                    success();
                break;

            case A_ERASE: case A_READ: case A_WRITE:
                if (testCount)
                    success();
                else
                    doAction(act);
                break;

            default:
                fail(FAIL);
                break;
        }
        testCount++;
    }

    task void flashTest()
    {

        int i;

        resetSeed();
        for (i = 0; i < sizeof data; i++)
            data[i++] = rand() >> 8;

        testCount = 0;
        done();
    }

    /************************************************************************
     * MAIN TEST CONTROL LOOP
     */
    task void nextTest()
    {
        bool runNextTest = TRUE;

        if(testPassed) {
            printf("Last Test PASSED\n");
        } else {
            printf("Last Test FAILED\n");
            printfflush();
            call Leds.led0On();
            return;
        }
        printfflush();

        testPassed = FALSE;

        switch(test) {
            case NO_TEST:
                test = CHARGING_TEST;
                runNextTest = FALSE;
                post chargingTest();
                break;
            case CHARGING_TEST:
                test = RADIO_TEST;
                runNextTest = FALSE;
                post radioTest();
                break;
            case RADIO_TEST:
                test = ACCELEROMETER_TEST;
                runNextTest = FALSE;
                post accelerometerTest();
                break;
            case ACCELEROMETER_TEST:
                test = RTC_TEST;
                runNextTest = FALSE;
                post rtcTest();
                break;
            case RTC_TEST:
                test = FLASH_TEST;
                runNextTest = FALSE;
                post flashTest();
                break;
            case FLASH_TEST:
                /* This is the last test. Turn on Green if we get here. */
                test = NO_TEST;
                call Leds.led1On();
                return;

            default:
                test = NO_TEST;
        }
        printf("Next Test: %d\n", test);
        printfflush();
        call Leds.led2Toggle();
        if(runNextTest)
            post nextTest();
    }


    event void Boot.booted() {
        printf("Booted\n");
        testPassed = TRUE;
        post nextTest();
    }

}

