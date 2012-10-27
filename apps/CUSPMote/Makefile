COMPONENT=CUSPBaseRecorderC

BUILD_EXTRA_DEPS += CmdSerialMsg.py RssiSerialMsg.py SerialStatusMsg.py BaseStatusMsg.py WRENStatusMsg.py WRENConnectionMsg.py
CLEAN_EXTRA = CmdSerialMsg.py RssiSerialMsg.py SerialStatusMsg.py BaseStatusMsg.py WRENStatusMsg.py WRENConnectionMsg.py

CmdSerialMsg.py: CUSPSerial.h
	    mig python -target=null $(CFLAGS) -python-classname=CmdSerialMsg CUSPSerial.h cmd_serial_msg -o $@

RssiSerialMsg.py: CUSPSerial.h
	    mig python -target=null $(CFLAGS) -python-classname=RssiSerialMsg CUSPSerial.h rssi_serial_msg -o $@
		
SerialStatusMsg.py: CUSPSerial.h
	    mig python -target=null $(CFLAGS) -python-classname=SerialStatusMsg CUSPSerial.h serial_status_msg -o $@

BaseStatusMsg.py: CUSPSerial.h
	    mig python -target=null $(CFLAGS) -python-classname=BaseStatusMsg CUSPSerial.h base_status_msg -o $@

WRENStatusMsg.py: CUSPSerial.h
	    mig python -target=null $(CFLAGS) -python-classname=WRENStatusMsg CUSPSerial.h wren_status_msg -o $@

WRENConnectionMsg.py: CUSPSerial.h
	    mig python -target=null $(CFLAGS) -python-classname=WRENConnectionMsg CUSPSerial.h wren_connection_msg -o $@

PFLAGS += -DTIMESYNC_RATE=10

PFLAGS += -I$(TOSDIR)/lib/ftsp
PFLAGS += -I$(TOSDIR)/lib/diagmsg

CFLAGS += -DCC2420_DEF_CHANNEL=11
CFLAGS += -DRF233_DEF_CHANNEL=11

CFLAGS += -DRF233_RSSI_ENERGY
CFLAGS += -DRF233_RX_PDT_LEVEL_VAL=0x03
CFLAGS += -DRF233_DEF_RFPOWER=0x0F
CFLAGS += -DRF233_SECOND_RFPOWER=0x00
#CFLAGS += -DRF233_USE_SECOND_RFPOWER
    
CFLAGS += -DLOW_POWER_LISTENING
CFLAGS += -DLPL_INTERVAL=512
CFLAGS += -DLPL_DEF_LOCAL_WAKEUP=512
CFLAGS += -DLPL_DEF_REMOTE_WAKEUP=512
CFLAGS += -DDELAY_AFTER_RECEIVE=20
CFLAGS += -DREMOTE_WAKEUP_INTERVAL=512
CFLAGS += -DLOCAL_WAKEUP_INTERVAL=512

CFLAGS += -DTOSH_DATA_LENGTH=50
CFLAGS += -DCONTROLLER_NODEID=0
CFLAGS += -DTIMESYNC_NODEID=1
CFLAGS += -DSLEEP_NODEID=2
CFLAGS += -DHEARTBEAT_INTERVAL=15000
CFLAGS += -DLED_INTERVAL=5
CFLAGS += -DSENSING_INTERVAL=20000L
#CFLAGS += -DSENSING_INTERVAL=1000L
CFLAGS += -DDOWNLOAD_BUSYWAIT_INTERVAL=1000
CFLAGS += -DSTATUS_INTERVAL=2000L

#==== Auto-Triggering Auto-Sensing ==
CFLAGS += -DSMART_SENSING
CFLAGS += -DSMART_SENSING_COUNT=10

# Depending on # of Sensors, 
#CFLAGS += -DSMART_SLEEP

#====================================

#=== Dissemination ===
#CFLAGS += -DDISSEMINATION_ON
### For Drip:
CFLAGS += -I$(TOSDIR)/lib/net -I%T/lib/net/drip

### For DIP:
#CFLAGS += -I$(TOSDIR)/lib/net -I%T/lib/net/dip -I%T/lib/net/dip/interfaces

### For Dhv
#CFLAGS += -I$(TOSDIR)/lib/net -I%T/lib/net/dhv -I%T/lib/net/dhv/interfaces

#==== Connection & Sliding Window Protocol ===
CFLAGS += -DCONNECTION_TIMEOUT=2000
CFLAGS += -DCONNECTION_ATTEMPT=15

CFLAGS += -DACK_INTERVAL=1000
CFLAGS += -DACK_ATTEMPT=20

CFLAGS += -DTRANSMIT_INTERVAL=250
CFLAGS += -DTRANSMIT_ATTEMPT=3

CFLAGS += -DRETRANSMIT_INTERVAL=3000
CFLAGS += -DTIMEOUT_INTERVAL=2000

CFLAGS += -DWINDOWSIZE=50

#================================

CFLAGS += -DBAT_HIGH=3.6
CFLAGS += -DBAT_MID=3.3
CFLAGS += -DBAT_MID_INTERVAL=5000
CFLAGS += -DBAT_LOW=3.0
CFLAGS += -DBAT_LOW_INTERVAL=1000

# ==== DEBUG PARAMETERS ====== 
#CFLAGS += -DDIAGMSG_SERIAL_START
#CFLAGS += -DMOTE_DEBUG
#CFLAGS += -DMOTE_DEBUG_LOGREAD
#CFLAGS += -DMOTE_DEBUG_STATUS
#CFLAGS += -DMOTE_DEBUG_MESSAGE
#CFLAGS += -DMOTE_DEBUG_MESSAGE_DETAIL
#CFLAGS += -DMOTE_DEBUG_QUEUE
CFLAGS += -DMOTE_DEBUG_MOTECOUNTING
# ============================

include $(MAKERULES)
