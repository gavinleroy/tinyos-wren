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

CFLAGS += -DRF233_DEF_CHANNEL=11
CFLAGS += -DRF233_SECOND_CHANNEL=20

CFLAGS += -DRF233_RSSI_ENERGY
CFLAGS += -DRF233_RX_PDT_LEVEL_VAL=0x03
CFLAGS += -DRF233_DEF_RFPOWER=0x00
CFLAGS += -DRF233_SECOND_RFPOWER=0x0F

CFLAGS += -DLOW_POWER_LISTENING
CFLAGS += -DLPL_INTERVAL=512
CFLAGS += -DLPL_DEF_LOCAL_WAKEUP=512
CFLAGS += -DLPL_DEF_REMOTE_WAKEUP=512
CFLAGS += -DDELAY_AFTER_RECEIVE=20
CFLAGS += -DREMOTE_WAKEUP_INTERVAL=512
CFLAGS += -DLOCAL_WAKEUP_INTERVAL=512

CFLAGS += -DTOSH_DATA_LENGTH=50
CFLAGS += -DTIMESYNC_NODEID=1
CFLAGS += -DHEARTBEAT_INTERVAL=15000
CFLAGS += -DLED_INTERVAL=4500L
CFLAGS += -DSENSING_INTERVAL=2000L

CFLAGS += -DBAT_HIGH=3.6
CFLAGS += -DBAT_MID=3.3
CFLAGS += -DBAT_MID_INTERVAL=5000
CFLAGS += -DBAT_LOW=3.0
CFLAGS += -DBAT_LOW_INTERVAL=1000

# ==== DEBUG PARAMETERS ====== 
#CFLAGS += -DDIAGMSG_SERIAL_START
CFLAGS += -DMOTE_DEBUG
#CFLAGS += -DMOTE_DEBUG_LOGREAD
#CFLAGS += -DMOTE_DEBUG_STATUS
CFLAGS += -DMOTE_DEBUG_MESSAGES
#CFLAGS += -DMOTE_DEBUG_QUEUE
CFLAGS += -DMOTE_DEBUG_MOTECOUNTING
# ============================

include $(MAKERULES)
