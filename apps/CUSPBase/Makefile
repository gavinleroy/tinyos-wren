COMPONENT=CUSPBaseRecorderC

BUILD_EXTRA_DEPS += CmdSerialMsg.py RssiSerialMsg.py SerialStatusMsg.py BaseStatusMsg.py WRENStatusMsg.py WRENConnectionMsg.py WRENCloseMsg.py
CLEAN_EXTRA = CmdSerialMsg.py RssiSerialMsg.py SerialStatusMsg.py BaseStatusMsg.py WRENStatusMsg.py WRENConnectionMsg.py WRENCloseMsg.py

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

WRENCloseMsg.py: CUSPSerial.h
	    mig python -target=null $(CFLAGS) -python-classname=WRENCloseMsg CUSPSerial.h wren_close_msg -o $@

PFLAGS += -DTIMESYNC_RATE=10

PFLAGS += -I$(TOSDIR)/lib/ftsp
PFLAGS += -I$(TOSDIR)/lib/diagmsg

CFLAGS += -DCC2420_DEF_CHANNEL=11

CFLAGS += -DCC2420_DEF_RFPOWER=31
CFLAGS += -DCC2420_SECOND_RFPOWER=31

CFLAGS += -DLOW_POWER_LISTENING
CFLAGS += -DLPL_INTERVAL=512
CFLAGS += -DLPL_DEF_LOCAL_WAKEUP=0
CFLAGS += -DLPL_DEF_REMOTE_WAKEUP=512
CFLAGS += -DDELAY_AFTER_RECEIVE=20
CFLAGS += -DREMOTE_WAKEUP_INTERVAL=512
CFLAGS += -DLOCAL_WAKEUP_INTERVAL=0

CFLAGS += -DTOSH_DATA_LENGTH=50
CFLAGS += -DCONTROLLER_NODEID=0
CFLAGS += -DTIMESYNC_NODEID=1
CFLAGS += -DHEARTBEAT_INTERVAL=15000
CFLAGS += -DLED_INTERVAL=5
CFLAGS += -DSENSING_INTERVAL=20000L

CFLAGS += -DBAT_HIGH=2.7
CFLAGS += -DBAT_MID=2.5
CFLAGS += -DBAT_MID_INTERVAL=5000
CFLAGS += -DBAT_LOW=2.0
CFLAGS += -DBAT_LOW_INTERVAL=1000

#==== Sliding Window Protocol ===
CFLAGS += -DWINDOWSIZE=50
CFLAGS += -DTIMEOUT=200
CFLAGS += -DACK_INTERVAL=250
CFLAGS += -DACK_ATTEMPT=100
CFLAGS += -DCONNECTION_TIMEOUT=2000
CFLAGS += -DCONNECTION_ATTEMPT=5
#================================

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
