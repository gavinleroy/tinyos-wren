from mni import mni

import os
import sys
import time
import optparse
import threading

from rtTimer import ResettableTimer

from mni import managedsubproc as msp

from tinyos.message import *
from tinyos.message.Message import *
from tinyos.message.SerialPacket import *
from tinyos.packet.Serial import Serial
from collections import deque
from array import *
from collections import defaultdict

import CmdSerialMsg
import RssiSerialMsg
import SerialStatusMsg
import BaseStatusMsg
import WRENStatusMsg
import WRENConnectionMsg
import WRENCloseMsg
from logger import Logger

CMD_DOWNLOAD        = 0
CMD_ERASE           = 1
CMD_START_SENSE     = 2
CMD_STOP_SENSE      = 3
CMD_STATUS          = 4
CMD_START_BLINK     = 5
CMD_STOP_BLINK      = 6
CMD_LOGSYNC         = 7
CMD_BASESTATUS      = 8
CMD_NONE            = 9
CMD_CHANNEL         = 10
CMD_CHANNEL_RESET   = 11
CMD_WREN_STATUS     = 12

DOWNLOAD_ALL        = 0
DOWNLOAD_SINGLE     = 1
DOWNLOAD_START      = 2
DOWNLOAD_STOP       = 3
DOWNLOAD_END        = -1

DOWNLOAD_MAX_TRY    = 10
DOWNLOAD_MAX_RETRY  = 10

basedir = time.strftime("%m-%d-%Y", time.localtime())
if not os.path.exists(basedir):
    os.mkdir(basedir)



class CmdCenter:

    mif = {}
    sfprocess = {}
    tos_source = {}

    msgs     = {}
    logSize  = {}
    
    lock = threading.RLock()
    progressLock = threading.RLock()
    
    f = {}
    motes = [] # LIST of motes
    
    basemsgs = {}
    wrenmsgs = {}
    wrenconnectionmsgs = {}
    basestations = {} # Dictionary of {downloader, nodeid} pairs. Means downloader is currently working on nodeid
    
    moteLogSize = {}
    downloadMaxTry = defaultdict(int)
    downloadMaxRetry = defaultdict(int)
    
    
    def __init__(self):
        print "init"

        self.dl = 0
        self.downloadMode = DOWNLOAD_ALL
        self.downloadState = DOWNLOAD_START
        
        self.isBusy = False
        self.m = mni.MNI()
        self.basemsgTimer = ResettableTimer(2, self.printBaseStatus)
        self.msgTimer = ResettableTimer(3, self.printStatus)
#        self.wrenTimer = ResettableTimer(2, self.printWRENStatus)
        self.wrenTimer = ResettableTimer(2, self.printMoteQueues)
        self.wrenConnectionTimer = ResettableTimer(2, self.printConnection)
        self.downloadTimer = ResettableTimer(4, self.downloadData)

        self.logger = Logger(basedir+"/download_log_stat.txt")
        

        numberOfMotes = 0
        for n in self.m.get_nodes():
            sys.stdout.write("%d,%s "%(n.id, n.serial))
            numberOfMotes = numberOfMotes + 1
        sys.stdout.write("\n number of motes connected: %d\n" %(numberOfMotes))
        sys.stdout.flush()

        # starting mote interfaces
        for n in self.m.get_nodes():
            baseFileName = "/tmp/sf"
            p = msp.ManagedSubproc("/usr/bin/java net.tinyos.sf.SerialForwarder -no-gui -port %d -comm serial@%s:tmote"%(20000+n.id, n.serial),
                    stdout_disk = baseFileName + ".%d.log"%(n.id,),
                    stderr_disk = baseFileName + ".%d.stderr.log"%(n.id,),
                    stdout_fns = [ ])
            p.start()
            self.sfprocess[n.id] = p

        # give it some time to establish all the serial forwarders
        time.sleep(4) # I also changed this time from 3 to 4 Gavin

        for n in self.m.get_nodes():
            #print n.id
            self.mif[n.id] = MoteIF.MoteIF()
            self.tos_source[n.id] = self.mif[n.id].addSource("sf@localhost:%d"%(20000+n.id))


        #add us as listener for the different messages
        for n in self.m.get_nodes():
            self.mif[n.id].addListener(self, RssiSerialMsg.RssiSerialMsg)
            self.mif[n.id].addListener(self, SerialStatusMsg.SerialStatusMsg)
            self.mif[n.id].addListener(self, BaseStatusMsg.BaseStatusMsg)
            self.mif[n.id].addListener(self, WRENStatusMsg.WRENStatusMsg)
            self.mif[n.id].addListener(self, WRENConnectionMsg.WRENConnectionMsg)
            self.mif[n.id].addListener(self, WRENCloseMsg.WRENCloseMsg)


    # Whenever a message is received from a mote this function is called.
    def receive(self, src, msg):

        if msg.get_amType() == RssiSerialMsg.AM_TYPE:
            m = RssiSerialMsg.RssiSerialMsg(msg.dataGet())
            if m.get_dst() == 0:
                return;
            
            with self.lock:
                if not self.downloadTimer.isAlive():
                    self.downloadTimer.start()
                self.downloadTimer.reset()
                
                self.logSize[m.get_dst()] = m.get_size()

                if (self.dl%200) == 0:
                    sys.stdout.write(".")
                    sys.stdout.flush()
                self.dl += 1


            if m.get_dst() not in self.f.keys():
                sys.stdout.write(basedir+"/node_%d.log\n"%(m.get_dst()))
                self.f[m.get_dst()] = open(basedir+"/node_%d.log"%(m.get_dst()), "a+")
            self.f[m.get_dst()].write("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %.2f, %d\n"%(m.get_dst(), m.get_src(), m.get_counter(), m.get_rssi(), m.get_srclocaltime(), m.get_srcglobaltime(), m.get_localtime(), m.get_globaltime(), m.get_isSynced(), m.get_reboot(), m.get_bat()/4096.0*5, m.get_size()))
            self.f[m.get_dst()].flush()
                
        if msg.get_amType() == SerialStatusMsg.AM_TYPE:
            with self.lock:
                if not self.msgTimer.isAlive():
                    self.msgTimer.start()
                self.msgTimer.reset()

                m = SerialStatusMsg.SerialStatusMsg(msg.dataGet())
                if m.get_src() not in self.msgs.keys():
                    self.msgs[m.get_src()] = m

            if m.get_src() == 1:
                # this is the base mote. Store it's time for sync in file
                f = open(basedir+"/timesync.log", "a+")
                f.write("%.3f, %d\n"%(time.time(), m.get_globaltime()))
                f.close()
            

        if msg.get_amType() == WRENStatusMsg.AM_TYPE:
            with self.lock:
                if not self.wrenTimer.isAlive():
                    self.wrenTimer.start()
                self.wrenTimer.reset()

                m = WRENStatusMsg.WRENStatusMsg(msg.dataGet())
                self.wrenmsgs[m.get_src()] = m

            if self.motes.count(m.get_src()) == 0 and m.get_buffersize() > 0:
                self.motes.append(m.get_src())

        if msg.get_amType() == WRENConnectionMsg.AM_TYPE:
            with self.lock:
                if not self.wrenConnectionTimer.isAlive():
                    self.wrenConnectionTimer.start()
                self.wrenConnectionTimer.reset()

                m = WRENConnectionMsg.WRENConnectionMsg(msg.dataGet())
                self.wrenconnectionmsgs[m.get_src()] = m

        if msg.get_amType() == WRENCloseMsg.AM_TYPE:
            with self.lock:
                m = WRENCloseMsg.WRENCloseMsg(msg.dataGet())

            if m.get_close() == 1:
                self.printWrite("connection and file closed: %s, nodeid: %s\n"%(time.ctime(), m.get_src()))
                self.printFlush()
                if m.get_src() in self.f.keys():
                    self.f[m.get_src()].flush()
                    self.f[m.get_src()].close()
                    del self.f[m.get_src()]
                self.finishDownload(m.get_channel(), m.get_src())
                                
        if msg.get_amType() == BaseStatusMsg.AM_TYPE:
            with self.lock:
                if not self.basemsgTimer.isAlive():
                    self.basemsgTimer.start()
                self.basemsgTimer.reset()

                m = BaseStatusMsg.BaseStatusMsg(msg.dataGet())
                self.basemsgs[m.get_src()] = m

            # collect all base stations out there
            if m.get_src() not in self.basestations.keys():
                self.basestations[m.get_src()] = 0

    def clearDownloadByNodeId(self, nodeid):
        cleared = False
        for baseid, value in self.basestations.iteritems(): # we can optimize this lookup
            if value == nodeid:
                self.basestations[baseid] = 0
                
                cleared = True
                break
        return cleared

    def clearDownloadByBaseId(self, baseid):
        self.basestations[baseid] = 0

    def existDownload(self, nodeid):
        exist = False
	# For every downloader
        for baseid in self.basestations:
	    # If the downloader id maps to the nodeid
            if self.basestations[baseid] == nodeid:
                exist = True
                break
        return exist

    def getNextDownload(self):
        nodeid = DOWNLOAD_END 
        if self.downloadState == DOWNLOAD_START: #Make sure we're still downloading
	    #Note that self.motes is a list of all the motes connected
            if len(self.motes) > 0:
                nodeid = self.motes.pop() #Pop the next node off the stack
            else:
		# If there aren't any nodes in the stack
		#    do a quick scan to verify we have tried all of them.
                if self.downloadMode == DOWNLOAD_ALL:
                    self.scanMotes() #find all motes out there
                    time.sleep(4) # Wait for responses, Gavin modified this to be 4 seconds
                    if len(self.motes) > 0: #if we have motes to download ...
                        nodeid = self.motes.pop()  # go ahead and pop one
        if nodeid == DOWNLOAD_END: # If the node ID didn't change we don't have any left
            print "Download finished ! Please wait until the remaining process finishes..."
        return nodeid

    def checkDownloadAll(self):
        print "checking progress ex..."
        for baseid, nodeid in self.basestations.iteritems(): # we can optimize this lookup
            print "checking ", baseid, nodeid
            if self.downloadState == DOWNLOAD_STOP:
                break
            if nodeid > 0:
                self.checkCurrentDownload(baseid, nodeid)


    def checkCurrentDownload(self, baseid, nodeid):
        if nodeid not in self.logSize.keys(): #This means not started yet. 
            self.downloadCommandController(baseid, nodeid)
        else:
            if self.logSize[nodeid] > 0:
                self.downloadCommandBaseStation(baseid, nodeid)
            else:
                if nodeid in self.f.keys():
                    self.f[nodeid].flush()
                    self.f[nodeid].close()
                    del self.f[nodeid]
                sys.stdout.write("%d Download Done\n" % (nodeid,))
                sys.stdout.flush()
                self.startNextDownload(baseid)                

    def startNextDownload(self, baseid):
	# If there could be another node to download, get it's id
        nodeid = self.getNextDownload()

	# If it's a valid nodeid proceed.
	#    note: that we can't get an id of 0
        if nodeid > 0: 
            if not self.existDownload(nodeid): #if it is not already downloading...
                self.basestations[baseid] = nodeid # Map the downloader to the new nodeid
                self.downloadMaxTry[nodeid] = 0 # Set download tries to 0
                self.moteLogSize[nodeid] = 0
                self.downloadCommandController(baseid, nodeid) # send the download command to Controller
            else:
                if self.downloadState == DOWNLOAD_STOP:
                    return;

                if nodeid != DOWNLOAD_END:
                    self.startNextDownload(baseid)
	elif nodeid == DOWNLOAD_END: # Note that DOWNLOAD_END == -1
	    return;

    def downloadData(self):

        print "downloadData mapping..."
        self.printDownloadMapping()

        if not self.isBusy:
            self.isBusy = True
	    
	    # For every downloader, and corresponding node ready for download.
            for baseid, nodeid in self.basestations.iteritems(): # we can optimize this lookup
		# If we ever get a DOWNLOAD_STOP return right away.
                if self.downloadState == DOWNLOAD_STOP:
                    return
		# If the node is a commander (nodeid == 0) then startNextDownload
                if nodeid == 0:
                    self.startNextDownload(baseid)
                else:
		    # If not the commander check on the current download in progress
                    self.checkCurrentDownload(baseid, nodeid) #check to see if the download is still in progress

            self.isBusy = False    

	# Reset the timer to re-call the downloadData() function
        with self.lock:
            self.downloadTimer.reset()
	
                
    def downloadCommandController(self, channel, nodeid):

	# If the max try is still less than MAX_TRY (10)
        if self.downloadMaxTry[nodeid] < DOWNLOAD_MAX_TRY:
            if self.downloadState == DOWNLOAD_STOP:
                self.resetDownloadMaxTry(nodeid)
                return;

            if nodeid not in self.logSize.keys(): # If nodeid hasn't been logged
                print("send download command to Controller ..") # Then send a message!
                msg = CmdSerialMsg.CmdSerialMsg()
                msg.set_cmd(CMD_DOWNLOAD)
                msg.set_dst(channel)
                msg.set_channel(channel)
                
                print (channel, nodeid)
		# Send download command from commander to nodeid
		#Self.mif[0].sendMsg(self.tos_source[0], nodeid, CmdSerialMsg.AM_TYPE, 0x22, msg)

            else: # If the message was already sent
                self.printWrite("download start: %s, channel: %s, nodeid: %s\n"%(time.ctime(), channel, nodeid))
                self.printFlush()
                self.resetDownloadMaxTry(nodeid)
                return
            
	    # Temporary placement here, to see if it works better... Gavin
            self.mif[0].sendMsg(self.tos_source[0], nodeid, CmdSerialMsg.AM_TYPE, 0x22, msg)
            self.downloadMaxTry[nodeid] += 1
        else:
            self.resetDownloadMaxTry(nodeid) # Reset the max try for the nodeid
            self.startNextDownload(channel) # Note that channel was the downloader id


    # Gavin Observation: I've never actually seen this function run before...
    def downloadCommandBaseStation(self, baseid, nodeid):
	# If less than max try for download attempts
        if self.downloadMaxRetry[nodeid] < DOWNLOAD_MAX_RETRY:
            if self.downloadState == DOWNLOAD_STOP:
                self.resetDownloadMaxReTry(nodeid)
                return;

            print("send command directly to base ..")
            msg = CmdSerialMsg.CmdSerialMsg()
            msg.set_cmd(CMD_DOWNLOAD)
            #msg.set_dst(baseid)
            msg.set_dst(nodeid)
            msg.set_channel(baseid)
            
            print (baseid, nodeid)

            if self.downloadMaxRetry[nodeid] == 0:
                self.moteLogSize[nodeid] = self.logSize[nodeid]
                #self.mif[baseid].sendMsg(self.tos_source[baseid], baseid, CmdSerialMsg.AM_TYPE, 0x22, msg)
            else:
                if self.moteLogSize[nodeid] == self.logSize[nodeid]: # this means still downloading...
                    if self.downloadMaxRetry[nodeid] > 5:
                        self.printWrite("download stop: %s, channel: %s, nodeid: %s\n"%(time.ctime(), baseid, nodeid))
                        self.printFlush()
                        self.resetDownloadMaxReTry(nodeid)
                        self.startNextDownload(baseid)
                        return
                    else:
                        self.mif[baseid].sendMsg(self.tos_source[baseid], baseid, CmdSerialMsg.AM_TYPE, 0x22, msg)
                else:
                    self.moteLogSize[nodeid] = self.logSize[nodeid]
                    self.printWrite("download restart: %s, channel: %s, nodeid: %s\n"%(time.ctime(), baseid, nodeid))
                    self.printFlush()
                    self.resetDownloadMaxReTry(nodeid)
                    #self.downloadTimer.reset()
                    return
            
            self.downloadMaxRetry[nodeid] += 1


    def finishDownload(self, baseid, nodeid):
        print "download done for node id", nodeid
        self.basestations[baseid] = 0
        self.printWrite("download end: %s, channel: %s, nodeid: %s\n"%(time.ctime(), baseid, nodeid))
        self.printFlush()
        with self.lock:
            self.downloadTimer.reset()

        if self.downloadMode == DOWNLOAD_ALL:
            self.downloadData()

    def clearDownloadAll(self):
        print "clear mapping..."
        for baseid, nodeid in self.basestations.iteritems(): 
            self.basestations[baseid] = 0
        self.printDownloadMapping()

    def scanMotes(self):
        msg = CmdSerialMsg.CmdSerialMsg()
        msg.set_cmd(CMD_WREN_STATUS)
        msg.set_dst(102)
        for n in self.m.get_nodes():
            self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)

    def scanBaseStations(self):
        msg.set_cmd(CMD_BASESTATUS)
        msg.set_dst(102)
        for n in self.m.get_nodes():
            msg.set_channel(n.id)
            self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)
    
    def resetDownloadMaxTry(self, nodeid):
        self.downloadMaxTry[nodeid] = 0
        #del self.downloadMaxTry[nodeid]

    def resetDownloadMaxReTry(self, nodeid):
        #self.downloadMaxRetry[nodeid] = 0
        del self.downloadMaxRetry[nodeid]
        
    def resetChannel(self):
        # set base channels
        msg = CmdSerialMsg.CmdSerialMsg()
        msg.set_cmd(CMD_CHANNEL_RESET)
        msg.set_dst(0xffff)
        for n in self.m.get_nodes():
            msg.set_channel(11)
            self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)

    def stopSensing(self, nodeid):
        # stop sensing
        msg = CmdSerialMsg.CmdSerialMsg()
        msg.set_cmd(CMD_STOP_SENSE)
        #msg.set_dst(0xffff)
        for n in self.m.get_nodes():
            self.mif[n.id].sendMsg(self.tos_source[n.id], nodeid, CmdSerialMsg.AM_TYPE, 0x22, msg)

    def startBlink(self, nodeid):
        # stop sensing
        msg = CmdSerialMsg.CmdSerialMsg()
        msg.set_cmd(CMD_START_BLINK)
        #msg.set_dst(0xffff)
        for n in self.m.get_nodes():
            self.mif[n.id].sendMsg(self.tos_source[n.id], nodeid, CmdSerialMsg.AM_TYPE, 0x22, msg)

    def stopBlink(self, nodeid):
        # stop sensing
        msg = CmdSerialMsg.CmdSerialMsg()
        msg.set_cmd(CMD_STOP_BLINK)
        #msg.set_dst(0xffff)
        for n in self.m.get_nodes():
            self.mif[n.id].sendMsg(self.tos_source[n.id], nodeid, CmdSerialMsg.AM_TYPE, 0x22, msg)


    def setBaseStationChannel(self):
        msg = CmdSerialMsg.CmdSerialMsg()
        msg.set_cmd(CMD_CHANNEL)
        msg.set_dst(0xffff)
        for n in self.m.get_nodes():
            msg.set_channel(n.id)
            self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)

    def resetDownloadTimer(self):
        with self.lock:
            if not self.downloadTimer.isAlive():
                self.downloadTimer.start()
            self.downloadTimer.reset()
            #self.downloadTimer.run()

            
    def printLogSize(self):
        for key, value in self.logSize.iteritems(): # we can optimize this lookup
            print "logSize:", key, value
            
    def printDownloadMapping(self):
	#print every downloader, and the node it will download 
        for baseid, nodeid in self.basestations.iteritems(): # we can optimize this lookup
            print "mapping:", baseid, nodeid

    def printStatus(self):
        keys = self.msgs.keys()
        keys.sort()
        for id in keys:
            m = self.msgs[id]
            self.printWrite("id: %4d, sensing: %d, local: %d, global: %d, isSync: %d, reboot: %d, bat: %.2f, logsize: %d, isErased: %d, download: %d, channel: %d,\n"%(m.get_src(), m.get_sensing(), m.get_localtime(), m.get_globaltime(), m.get_isSynced(), m.get_reboots(), m.get_bat()/4096.0*5, m.get_buffersize(), m.get_isErased(), m.get_download(), m.get_channel()))
        
        self.printWrite("%d messages received!\n"%(len(self.msgs)))
        self.printFlush()

        self.msgs = {}

    def printBaseStatus(self):
        basekeys = self.basemsgs.keys()
        basekeys.sort()
        for id in basekeys:
            m = self.basemsgs[id]
            sys.stdout.write("id: %4d, local: %d, global: %d, isSync: %d, channel: %d\n"%(m.get_src(), m.get_localtime(), m.get_globaltime(), m.get_isSynced(), m.get_channel()))

        sys.stdout.write("%d messages received !\n"%(len(self.basemsgs)))
        sys.stdout.flush()

        self.basemsgs = {}

    def printWRENStatus(self):
        wrenkeys = self.wrenmsgs.keys()
        wrenkeys.sort()
        for id in wrenkeys:
            m = self.wrenmsgs[id]
            sys.stdout.write("id: %4d, sensing: %d\n"%(m.get_src(), m.get_sensing()))

        sys.stdout.write("%d messages received !\n"%(len(self.wrenmsgs)))
        sys.stdout.flush()

        self.wrenmsgs = {}

    def printConnection(self):
        wrenkeys = self.wrenconnectionmsgs.keys()
        wrenkeys.sort()
        for id in wrenkeys:
            m = self.wrenconnectionmsgs[id]
            sys.stdout.write("id: %4d, is Acked: %d\n"%(m.get_src(), m.get_isAcked()))

        sys.stdout.write("%d messages received !\n"%(len(self.wrenconnectionmsgs)))
        sys.stdout.flush()

        self.wrenconnectionmsgs = {}

    def printMoteQueues(self):
        print "***", len(self.motes), "client motes have data and queued for download"
        print "***", len(self.basestations), "download base stations are ready for download"

    def printMoteQueueDetail(self):
        print "***", len(self.motes), "client motes have data and queued for download"
        print "client node id:"
        for elem in self.motes:
            print elem
        
        print "***", len(self.basestations), "download base stations are ready for download"
        print "base station id:"
        for elem in self.basestations:
            print elem

    def printWrite(self, line):
        self.logger.write(line)
        sys.stdout.write(line)

    def printWriteLine(self, line):
        self.logger.writeLine(line)
        sys.stdout.write(line)

    def printFlush(self):
        self.logger.flush()
        sys.stdout.flush()
        
    def help(self):

        print "Hit 'q' to exit"
        print "Hit 's' to get status"
        print "Hit 'g' to start sensing (go)"
        print "Hit 'b' to stop sensing  (break)"
        print "Hit 'd' to start downloading"
        print "Hit 'e' to erase"
        print "Hit 'r' to restore log"
        print "Hit 't' to start blink"
        print "Hit 'a' to stop blink"
        print "Hit 'h' for help"
        print "Hit 'x' for radio channel reset"
        #print "Hit 'w' to get ready for download"
        print "Hit 's <nodeid>' to get status of one specific node"
        print "Hit 'g <nodeid>' to start sensing a specific node"
        print "Hit 'b <nodeid>' to stop sensing a specific node"
        print "Hit 'd <nodeid>' to start downloading a specific node"
        print "Hit 'e <nodeid>' to erase a specific node"
        print "Hit 'r <nodeid>' to restore log of one specific node"
        print "Hit 't <nodeid>' to start blinking a specific node"
        print "Hit 'a <nodeid>' to stop blinking a specific node"
	if not __debug__:
	    print "____DEBUGGING ON____"

    
    def sendMessage(self, MSG_TYPE, nodeid=0xffff, channel=None, dst=None):

	msg = CmdSerialMsg.CmdSerialMsg()
	msg.set_cmd(MSG_TYPE)
	msg.set_dst(nodeid)
	if(dst is not None):
	    msg.set_dst(dst)
	for n in self.m.get_nodes():
	    if(channel is not None):
		msg.set_channel(channel)
	    elif(channel is -1):
		msg.set_channel(n.id)
	    self.mif[n.id].sendMsg(self.tos_source[n.id], nodeid, CmdSerialMsg.AM_TYPE, 0x22, msg)

    def main_loop(self):

        self.help()
        
        while 1:
	    c = raw_input()
            self.downloadState = DOWNLOAD_START
            
            self.printWrite("%s, command: %s\n"%(time.ctime(), c))
            self.printFlush()
            
            if c == 'p':
                self.printMoteQueueDetail()
            elif c == 'q':
                self.resetChannel()
                for k in self.sfprocess.keys():
                    self.sfprocess[k].stop()
                    while self.sfprocess[k].is_dead():
                        time.sleep(1)
                
                if self.logger is not None:
                    self.logger.close()
                    
                break
            elif c == 's':
                # get status
                self.f.clear()
       		self.sendMessage(CMD_STATUS)         
            elif c == 'w':
                self.scanMotes()
                self.setBaseStationChannel()
            elif c == 'f':
                # get base status
		self.sendMessage(channel=-1, dst=102)
                msg = CmdSerialMsg.CmdSerialMsg()
                msg.set_cmd(CMD_BASESTATUS)
                msg.set_dst(102)
                for n in self.m.get_nodes():
                    msg.set_channel(n.id)
                    self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)
            elif c == 'x':
                self.downloadState = DOWNLOAD_STOP
                self.resetChannel()
            elif c == 'g':
                # start sensing
		self.sendMessage(CMD_START_SENSE);
            elif c == 'b':
                self.downloadState = DOWNLOAD_STOP
		self.sendMessage(CMD_STOP_SENSE)
            elif c == 't':
		self.sendMessage(CMD_START_BLINK)
            elif c == 'a':
		self.sendMessage(CMD_STOP_BLINK)
            elif c == 'e':
                c = raw_input("Are you sure you want to erase the data [y/n]")
                if c != 'y':
                    print "Abort erase"
                    continue
		self.sendMessage(CMD_ERASE);
            elif c == 'd':
		# Set mode to download all
                self.downloadMode = DOWNLOAD_ALL

                # start download
                # stop all motes first
                self.scanMotes()
                self.setBaseStationChannel()

		# Give some time for the motes to settle down.
                time.sleep(3)
		# Make sure that sensing is off.
		self.sendMessage(CMD_STOP_SENSE)

		# Reset the download timer. When the timer expires (every 4 seconds)
		# it will call the downloadData() function.
		self.resetDownloadTimer()
		self.downloadData()

            elif c == 'r':
                # restore log
                c = raw_input("Are you sure you want to restore log? [y/n]")
                if c != 'y':
                    print "Abort restore"
                    continue
		self.sendMessage(CMD_LOGSYNC)
            elif len(c) > 1:
                if c[0] == 's' and c[1] == ' ':
                    cs = c.strip().split(" ")
                    nodeid = int(cs[1])
                    # get status
		    self.sendMessage(CMD_STATUS, nodeid=nodeid)
                elif c[0] == 'b' and c[1] == ' ':
                    cs = c.strip().split(" ")
                    nodeid = int(cs[1])
		    self.sendMessage(CMD_STOP_SENSE, nodeid=nodeid)
                elif c[0] == 't' and c[1] == ' ':
                    cs = c.strip().split(" ")
                    nodeid = int(cs[1])
		    self.sendMessage(CMD_START_BLINK, nodeid=nodeid)
                elif c[0] == 'a' and c[1] == ' ':
                    cs = c.strip().split(" ")
                    nodeid = int(cs[1])
		    self.sendMessage(CMD_STOP_BLINK, nodeid=nodeid)
                elif c[0] == 'g' and c[1] == ' ':
                    cs = c.strip().split(" ")
                    nodeid = int(cs[1])
                    # get status
		    self.sendMessage(CMD_START_SENSE, nodeid=nodeid)
                elif c[0] == 'e' and c[1] == ' ':
                    cs = c.strip().split(" ")
                    nodeid = int(cs[1])
                    # get status
                    c = raw_input("Are you sure you want to erase the data for node " + str(nodeid) + " [y/n]")
                    if c != 'y':
                        print "Abort erase"
                        continue
		    self.sendMessage(CMD_ERASE, nodeid=nodeid)
                elif c[0] == 'd':
                    self.downloadMode = DOWNLOAD_SINGLE
                    # start download
                    cs = c.strip().split(" ")
                    nodeid = int(cs[1])

                    # get the first group of motes for download and set channels for base stations
                    #self.scanMotes()
                    # set base channels
                    self.setBaseStationChannel()

                    time.sleep(3) #give sometime to settle down                
		    self.sendMessage(CMD_START_SENSE, nodeid=nodeid)
                    time.sleep(3) #give sometime to settle down                

                    if self.motes.count(nodeid) == 0:
                        self.motes.append(nodeid)

                    self.downloadData()
                    
                elif c[0] == 'r':
                    cs = c.strip().split(" ")
                    nodeid = int(cs[1])
		    self.sendMessage(CMD_LOGSYNC, nodeid=nodeid)

            elif c == 'h':
                self.help()


def main():
    cc = CmdCenter()
    cc.main_loop()  # don't expect this to return...

if __name__ == "__main__":
    main()

