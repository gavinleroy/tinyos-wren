COMPONENT=CUSPBaseRecorderC

BUILD_EXTRA_DEPS += CmdSerialMsg.py RssiSerialMsg.py SerialStatusMsg.py
CLEAN_EXTRA = CmdSerialMsg.py RssiSerialMsg.py SerialStatusMsg.py

CmdSerialMsg.py: CUSPSerial.h
	    mig python -target=null $(CFLAGS) -python-classname=CmdSerialMsg CUSPSerial.h cmd_serial_msg -o $@

RssiSerialMsg.py: CUSPSerial.h
	    mig python -target=null $(CFLAGS) -python-classname=RssiSerialMsg CUSPSerial.h rssi_serial_msg -o $@
		
SerialStatusMsg.py: CUSPSerial.h
	    mig python -target=null $(CFLAGS) -python-classname=SerialStatusMsg CUSPSerial.h serial_status_msg -o $@

PFLAGS += -DTIMESYNC_RATE=10

PFLAGS += -I$(TOSDIR)/lib/ftsp
PFLAGS += -I$(TOSDIR)/lib/diagmsg

CFLAGS += -DCC2420_DEF_CHANNEL=11

CFLAGS += -DCC2420_DEF_RFPOWER=2

CFLAGS += -DLOW_POWER_LISTENING
CFLAGS += -DLPL_INTERVAL=512
CFLAGS += -DLPL_DEF_LOCAL_WAKEUP=512
CFLAGS += -DLPL_DEF_REMOTE_WAKEUP=512
CFLAGS += -DDELAY_AFTER_RECEIVE=20
CFLAGS += -DREMOTE_WAKEUP_INTERVAL=512
CFLAGS += -DLOCAL_WAKEUP_INTERVAL=512

CFLAGS += -DTOSH_DATA_LENGTH=50
CFLAGS += -DTIMESYNC_NODEID=1
CFLAGS += -DSLEEP_NODEID=2
CFLAGS += -DHEARTBEAT_INTERVAL=15000
CFLAGS += -DLED_INTERVAL=5
CFLAGS += -DSENSING_INTERVAL=20000L
#CFLAGS += -DSENSING_INTERVAL=1000L
CFLAGS += -DDOWNLOAD_BUSYWAIT_INTERVAL=1000

#==== Auto-Triggering Auto-Sensing ==
#CFLAGS += -DSMART_SENSING
CFLAGS += -DSMART_SENSING_COUNT=10

# Depending on # of Sensors, 
#CFLAGS += -DSMART_SLEEP

#====================================

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
#CFLAGS += -DMOTE_DEBUG_MESSAGES
#CFLAGS += -DMOTE_DEBUG_QUEUE
CFLAGS += -DMOTE_DEBUG_MOTECOUNTING
# ============================

include $(MAKERULES)
