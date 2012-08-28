/**
 * Copyright (c) 2009 The Regents of the University of California.
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
 * - Neither the name of the copyright holder nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @author Thomas Schmid
 **/

#ifdef DEBUG_PRINTF
#define NEW_PRINTF_SEMANTICS
#include <printf.h>
#endif

module TestSpiC
{
    uses interface Leds;
    uses interface Boot;
    uses interface SpiByte;
    uses interface FastSpiByte;
    uses interface SpiPacket;
    uses interface Resource as SpiResource;
    uses interface GeneralIO as SELN;

}
implementation
{

    enum {
        SPI_BYTE,
        SPI_PACKET,
        FAST_SPI_BYTE,
    };

    uint8_t state;

    void nextState();

    task void transferFastTask()
    {
        uint8_t tx_buf[10];
        uint8_t rx_buf[10];
        uint8_t i;

#ifdef DEBUG_PRINTF
        printf("transfer fast task\n");
        printfflush();
#endif

        for(i=0; i<10; i++)
        {
            tx_buf[i] = 0xA0 + i;
            rx_buf[i] = 0;
        }

        call SELN.clr();
        call FastSpiByte.splitWrite(tx_buf[0]);

        for(i=0; i<9; i++) {
            rx_buf[i] = call FastSpiByte.splitReadWrite(tx_buf[i+1]);
        }
        rx_buf[10] = call FastSpiByte.splitRead();

        for(i=0; i<10; i++)
        {
            if(rx_buf[i] != tx_buf[i]){
#ifdef DEBUG_PRINTF
                printf("Received 0x%x, expected 0x%x\n", rx_buf[i], tx_buf[i]);
                printfflush();
#endif
            }
        }

        nextState();
    }


    task void transferPacketTask()
    {
        uint8_t tx_buf[10];
        uint8_t rx_buf[10];
        uint8_t i;

#ifdef DEBUG_PRINTF
        printf("transfer packet task\n");
        printfflush();
#endif
        for(i=0; i<10; i++)
        {
            tx_buf[i] = 0xA0 + i;
            rx_buf[i] = 0;
        }

#ifdef DEBUG_PRINTF
        printf("send packet\n");
        printfflush();
#endif

        call SELN.clr();

        if(call SpiPacket.send(tx_buf, rx_buf, 10)!= SUCCESS)
        {
#ifdef DEBUG_PRINTF
            printf("SpiPacket.send FAILED\n");
            printfflush();
#endif
            nextState();
        }
    }

    task void transferTask()
    {
        uint8_t byte;

#ifdef DEBUG_PRINTF
        printf("Transfer Task\n");

        printf("w\n");
        printfflush();
#endif

        call SELN.clr();

        byte = call SpiByte.write(0xCD);
        if(byte == 0xCD)
        {
            call Leds.led0Toggle();
        } else {
            call Leds.led1Toggle();
        }
        call SELN.set();

#ifdef DEBUG_PRINTF
        printf("w\n");
        printfflush();
#endif

        call SELN.clr();
        byte = call SpiByte.write(0xAB);
        if(byte == 0xAB)
        {
            call Leds.led0Toggle();
        } else {
            call Leds.led1Toggle();
        }

        call SELN.set();
        nextState();

    }

    event void Boot.booted()
    {
#ifdef DEBUG_PRINTF
        printf("Boot\n");
        printfflush();
#endif

        call SELN.makeOutput();
        call SELN.set();

        call SpiResource.request();

    }

    event void SpiResource.granted()
    {
#ifdef DEBUG_PRINTF
        printf("Resource Granted\n");
        printfflush();
#endif
        nextState();
    }

    async event void SpiPacket.sendDone(uint8_t* tx_buf, uint8_t* rx_buf, uint16_t len, error_t error)
    {
        uint8_t i;

        call SELN.set();

#ifdef DEBUG_PRINTF
        printf("Packet Send Done\n");
        printfflush();
#endif

        if(error == SUCCESS)
        {
            if(len == 10)
            {
                for(i=0; i<10; i++){
                    if(rx_buf[i] != 0xA0 + i)
                    {
#ifdef DEBUG_PRINTF
                        printf("SpiPacket.sendDone received wrong byte: 0x%x instead of 0x%x\n", rx_buf[i], 0xA0+i);
                        printfflush();
#endif
                        call Leds.led1Toggle();
                    } else {
                        call Leds.led2Toggle();
                    }
                }
                call Leds.led0Toggle();
            }
        }
        call Leds.led1Toggle();
        nextState();
    }

    void nextState()
    {

        switch(state) {
        case SPI_BYTE:
            post transferTask();
            state = SPI_PACKET;
            break;

        case SPI_PACKET:
            post transferPacketTask();
            state = FAST_SPI_BYTE;
            break;

        case FAST_SPI_BYTE:
            post transferFastTask();
            state = SPI_BYTE;
            break;

        default:
        }

    }

}
