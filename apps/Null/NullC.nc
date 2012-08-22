/*
 * Copyright (c) 2000-2005 The Regents of the University  of California.  
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
/*
 * Copyright (c) 2002-2005 Intel Corporation
 * All rights reserved.
 *
 * This file is distributed under the terms in the attached INTEL-LICENSE     
 * file. If you do not find these files, copies can be found by writing to
 * Intel Research Berkeley, 2150 Shattuck Avenue, Suite 1300, Berkeley, CA, 
 * 94704.  Attention:  Intel License Inquiry.
 */

/**
 * Null is an empty skeleton application.  It is useful to test that the
 * build environment is functional in its most minimal sense, i.e., you
 * can correctly compile an application. It is also useful to test the
 * minimum power consumption of a node when it has absolutely no 
 * interrupts or resources active.
 *
 * @author Cory Sharp <cssharp@eecs.berkeley.edu>
 * @date February 4, 2006
 */
module NullC @safe()
{
  uses {
  		interface Boot;
        interface ConfigStorage as Config;
        interface Mount as Mount;
  }
  
}
implementation
{
	
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
        DEFAULT_HALT	   = 0,
    };

    config_t conf;
        
  event void Boot.booted() {
    // Do nothing.
        if(call Mount.mount() != SUCCESS) {
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
                if (call Config.commit() == SUCCESS) {
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
                conf.sensing    = DEFAULT_SENSING;
                conf.reboot = 0;
                conf.halt		= DEFAULT_HALT;
            }
            else {
                conf.version    = CONFIG_VERSION;
                conf.sensing    = DEFAULT_SENSING;
                conf.globaltime = DEFAULT_GLOBALTIME;
                conf.reboot     = DEFAULT_REBOOT;
                conf.halt		= DEFAULT_HALT;
            }
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
        if (err == SUCCESS) {
            // Handle failure
        }
    }
}

