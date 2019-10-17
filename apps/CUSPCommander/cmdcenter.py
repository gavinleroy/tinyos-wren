#!/usr/bin/env python

# Last edited by Gavin Gray at the University of Utah
#	September 22, 2019
#

from mni import mni

import os
import sys
import time
import optparse
import threading

import time # Added for timing the motes.

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

# Create directory for LOG file. All commands are written to this file.
# Directories are named by the day of use.
basedir = time.strftime("%m-%d-%Y", time.localtime())
if not os.path.exists(basedir):
    os.mkdir(basedir)


class CmdCenter:
    
    avg_resp_times = []

    # Dictionary id:MoteIF instance. MoteIF can be found in MoteIF.py creates a mote instance
    mif = {}

    sfprocess = {}
    # Dictionary that maps id's to their source instances
    tos_source = {}

    # Dictionary, mapping SerialStatusMSG sources to their messages. Used only for SerialStatus Messages
    msgs     = {}
    # Dictionary mapping moteid's to their logsizes
    logSize  = {} 
    
    lock = threading.RLock()
    progressLock = threading.RLock()
    
    f = {}
    # List of motes connected
    motes = [] 
    # List of motes that failed to download, created by Gavin for the new downloadData(int x) method
    failed_download = []
    # Dictionary, mapping BaseStatus MSG sources to their messages. Used only for BaseStatus Messages

    basemsgs = {}
    # Dictionary, mapping WREN MSG sources to their messages. Used only for WRENStatus Messages
    wrenmsgs = {}
    # Dictionary, mapping WREN MSG sources to their messages. Used only for WRENConnection Messages
    wrenconnectionmsgs = {}
    # Dictionary of {downloader, nodeid} pairs. Means downloader is currently working on nodeid
    basestations = {} 
    # Dictionary of baseid : moteidStatus to indicate how it's doing on the download.
    download_status = {}
    
    moteLogSize = {}
    downloadMaxTry = defaultdict(int)
    downloadMaxRetry = defaultdict(int)
    
    
    def __init__(self):
        print "init"

	self.StartTime = 0
	
        self.dl = 0
        self.downloadMode = DOWNLOAD_ALL
        self.downloadState = DOWNLOAD_START
        
        self.isBusy = False

	# Class found in mni.py ... use locate mni.py to find filepath on local machine.
        self.m = mni.MNI() # Initialize a Managed Node Infrastructure.

	# Timers
        self.basemsgTimer = ResettableTimer(2, self.printBaseStatus)
        self.msgTimer = ResettableTimer(3, self.printStatus)
        self.wrenTimer = ResettableTimer(2, self.printMoteQueues)
        self.wrenConnectionTimer = ResettableTimer(2, self.printConnection)
        self.downloadTimer = ResettableTimer(3, self.downloadData)

	# Log file for all commands run.
        self.logger = Logger(basedir+"/download_log_stat.txt")
        
	# Count number of motes in self.m 
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
            p.start() # Start the Sub process
            self.sfprocess[n.id] = p

        # give it some time to establish all the serial forwarders
        time.sleep(3) 

	# Initialize mote instances with their ID's and save the source to the id.
        for n in self.m.get_nodes():
            self.mif[n.id] = MoteIF.MoteIF()
	    # Add the source of each node to its id.
            self.tos_source[n.id] = self.mif[n.id].addSource("sf@localhost:%d"%(20000+n.id))


        #add us as listener for the different types of messages
        for n in self.m.get_nodes():
            self.mif[n.id].addListener(self, RssiSerialMsg.RssiSerialMsg)
            self.mif[n.id].addListener(self, SerialStatusMsg.SerialStatusMsg)
            self.mif[n.id].addListener(self, BaseStatusMsg.BaseStatusMsg)
            self.mif[n.id].addListener(self, WRENStatusMsg.WRENStatusMsg)
            self.mif[n.id].addListener(self, WRENConnectionMsg.WRENConnectionMsg)
            self.mif[n.id].addListener(self, WRENCloseMsg.WRENCloseMsg)


    # Whenever a message is received from a mote this function is called.
    def receive(self, src, msg):

        # Message to start a download of a mote.
        if msg.get_amType() == RssiSerialMsg.AM_TYPE:
	    print "RssiSerialMsg Received"  # Gavin
            m = RssiSerialMsg.RssiSerialMsg(msg.dataGet())
            if m.get_dst() == 0:
                return;
            
            with self.lock:
#                if not self.downloadTimer.isAlive():
#                    self.downloadTimer.start()
#                self.downloadTimer.reset()
                
		# Add logsize to the dictionary, indicating that the downloadstarted
                self.logSize[m.get_dst()] = m.get_size()

                if (self.dl%200) == 0:
                    sys.stdout.write(".")
                    sys.stdout.flush()
                self.dl += 1


            if m.get_dst() not in self.f.keys():
		self.openMoteLog(m.get_dst())
#                sys.stdout.write(basedir+"/node_%d.log\n"%(m.get_dst()))
#                self.f[m.get_dst()] = open(basedir+"/node_%d.log"%(m.get_dst()), "a+")
            self.f[m.get_dst()].write("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %.2f, %d\n"%(m.get_dst(), m.get_src(), m.get_counter(), m.get_rssi(), m.get_srclocaltime(), m.get_srcglobaltime(), m.get_localtime(), m.get_globaltime(), m.get_isSynced(), m.get_reboot(), m.get_bat()/4096.0*5, m.get_size()))
            self.f[m.get_dst()].flush()
                
	# Message is received when the status of the motes is requested
        if msg.get_amType() == SerialStatusMsg.AM_TYPE:
            with self.lock:
		self.avg_resp_times.append(time.time()-self.StartTime)
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
#	    self.avg_resp_times.append(time.time()-self.StartTime)
	    print "WRENStatusMsg Received"  # Gavin
            with self.lock:
                if not self.wrenTimer.isAlive():
                    self.wrenTimer.start()
                self.wrenTimer.reset()

                m = WRENStatusMsg.WRENStatusMsg(msg.dataGet())
                self.wrenmsgs[m.get_src()] = m

            if self.motes.count(m.get_src()) == 0 and m.get_buffersize() > 0:
                self.motes.append(m.get_src())

        if msg.get_amType() == WRENConnectionMsg.AM_TYPE:
	    print "WRENConnectionMsg Received"  # Gavin
       	    self.avg_resp_times.append(time.time()-self.StartTime)
            with self.lock:
                if not self.wrenConnectionTimer.isAlive():
                    self.wrenConnectionTimer.start()
                self.wrenConnectionTimer.reset()

                m = WRENConnectionMsg.WRENConnectionMsg(msg.dataGet())
                self.wrenconnectionmsgs[m.get_src()] = m

        if msg.get_amType() == WRENCloseMsg.AM_TYPE:
	    print "WRENCloseMsg Received"  # Gavin
       	    self.avg_resp_times.append(time.time()-self.StartTime)
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
	    print "BaseStatusMsg Received"  # Gavin
            with self.lock:
                if not self.basemsgTimer.isAlive():
                    self.basemsgTimer.start()
                self.basemsgTimer.reset()

                m = BaseStatusMsg.BaseStatusMsg(msg.dataGet())
                self.basemsgs[m.get_src()] = m

            # collect all base stations out there
            if m.get_src() not in self.basestations.keys():
                self.basestations[m.get_src()] = 0

    def openMoteLog(self, nodeid):
	# Remove concurrency to make sure that the self.f object always stays up-to-date
	with self.lock:
	    sys.stdout.write(basedir+"/node_%d.log\n"%(nodeid))
	    self.f[m.get_dst()] = open(basedir+"/node_%d.log"%(nodeid), "a+")
	    # Change the status of the mote to logfile opened 

    def clearDownloadByNodeId(self, nodeid):
        cleared = False
        for baseid, value in self.basestations.iteritems(): # we can optimize this lookup
            if value == nodeid:
                self.basestations[baseid] = 0
                
                cleared = True
                break
        return cleared

    # Unassociate mote with specified baseid
    def clearDownloadByBaseId(self, baseid):
        self.basestations[baseid] = 0

    # Find if there exists a current download for nodeid
    def existDownload(self, nodeid):
        exist = False
	# For every downloader
        for baseid in self.basestations:
	    # If the downloader id maps to the nodeid
            if self.basestations[baseid] == nodeid:
                exist = True # Then the download does exist
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
                    time.sleep(3) # Wait for responses
                    if len(self.motes) > 0: #if we have motes to download ...
                        nodeid = self.motes.pop()  # go ahead and pop one
        if nodeid == DOWNLOAD_END: # If the node ID didn't change we don't have any left
            print "Download finished ! Please wait until the remaining process finishes..."
        return nodeid

    def checkDownloadAll(self):
        print "checking progress ex..."
	# For each connected downloader.
        for baseid, nodeid in self.basestations.iteritems(): # we can optimize this lookup
            print "checking ", baseid, nodeid
            if self.downloadState == DOWNLOAD_STOP:
                break
	    # If the node isn't the downloader then check the download state of it.
            if nodeid > 0:
                self.checkCurrentDownload(baseid, nodeid)


    def checkCurrentDownload(self, baseid, nodeid):
        if nodeid not in self.logSize.keys(): #This means not started yet. 
            self.downloadCommandController(baseid, nodeid) #If it hasn't started then start the download of nodeid
        else:
            if self.logSize[nodeid] > 0: #If there is still data left on the mote to download.
                self.downloadCommandBaseStation(baseid, nodeid)
            else:
                if nodeid in self.f.keys(): # Else the download is done. 
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


    def test_threading(self, baseid, x):
	print "thread started: " + str(baseid) + " : " + str(x)
	time.sleep(10)

    def handleMoteDownload(self, baseid, nodeid):
	'''
	The current status codes are:
		- DOWN_NOT_STARTED
		- DOWN_CURRENT
		- DOWN_FINISHED
	if the logSize of the nodeid is greater than zero, this means that the process has not completed yet
	and we need to continue sending download signals to the mote.
	'''
	self.basestations[baseid] = nodeid # Associate Baseid with Nodeid that will be downloaded 
	self.download_status[baseid] = DOWN_NOT_STARTED # Start the status as NOT STARTED

	self.downloadMaxTry[nodeid] = 0 # Initialize the try count to 0 because nothing has been tried yet.
	self.moteLogSize[nodeid] = 0 # Initialize LogSize to 0 because nothing has been downloaded yet.

	# While the try count is less than MAX_TRY and status isn't FINISHED
	# We will try to download, 
	while self.downloadMaxTry[nodeid] < DOWNLOAD_MAX_TRY and self.download_status[baseid] != DOWN_FINISHED:
	    # Complete action based on the current status of the mote.
	    if self.download_status[baseid] == DOWN_NOT_STARTED:
		# Need to implement logic
	    elif self.download_status[baseid] == DOWN_CURRENT:
		# Need to implement logic 
	    elif self.download_status[baseid] == DOWN_FINISHED:
		break
	    else: # the download status was not recognized as a valid option, throw an error.
		raise Exception("Download Status not recognized: Fatal Error.")
	
	    time.sleep(10) # Wait 10 seconds in order to ensure that everything was completed okay
	    self.downloadMaxTry[nodeid] += 1 # Increase the try count for the node
	 

# This needs to happen with every mote to ENSURE that the logfile is closed 
#	self.f[nodeid].flush()
#	self.f[nodeid].close()
#	del self.f[nodeid]
	return

    def _downloadData(self):
	'''
	This download method was created by Gavin Gray 10/2019,
	any questions can be sent to gavinleroy6@gmail.com

	This was created to ensure that all motes were downloaded and that the program
	would never stall. It processes the downloads concurrently with as many threads as
	there are downloaders connected. 

	At the end of the function a report of which motes were NOT successfully downloaded is generated.
	'''
	# Find the basestations that are ready to download
        self.setBaseStationChannel()
	# Set up the mote queue
	self.scanMotes()	
	# Wait for previous processes to finish 
	time.sleep(10)
	#WHILE the queue is not empty
	while self.motes:
	    _threads = list()
	    # Prepare the individual threads for downloading
	    for baseid in self.basestations:
		# Make sure that the queue isn't empty
		if self.motes:
		    nodeid = 0
		    while nodeid == 0: # Make sure that we don't get a commander
			# Save the nodeid that will be downloaded
			nodeid = self.motes[-1]
			# Pop that mote off the stack, because it will be downloaded.
			self.motes.pop()
		    # Add a thread to our current list of threads
		    _threads.append(threading.Thread(target=self.handleMoteDownload, args=(baseid,nodeid,)))
	    # Start all the threads
	    for t in _threads:
		t.start()
	    # Join all the threads
	    for t in _threads:
		t.join()
	print "RETURNING FROM TEST DOWNLOAD"
	return

    def downloadData(self):

        print "downloadData mapping..."
        self.printDownloadMapping()

        if not self.isBusy:
            self.isBusy = True
	    
	    # For every downloader, and corresponding node ready for download.
            for baseid in self.basestations: # we can optimize this lookup
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

            if nodeid not in self.logSize.keys(): # If download for particular node hasn't started yet. 
                print("send download command to Controller ..") # Then send a message!
                msg = CmdSerialMsg.CmdSerialMsg()
                msg.set_cmd(CMD_DOWNLOAD)
                msg.set_dst(channel)
                msg.set_channel(channel)
                
                print (channel, nodeid)
		# Send download command from commander to nodeid
		self.mif[0].sendMsg(self.tos_source[0], nodeid, CmdSerialMsg.AM_TYPE, 0x22, msg)

            else: # If download has been started (meaning message was sent.) 
                self.printWrite("download start: %s, channel: %s, nodeid: %s\n"%(time.ctime(), channel, nodeid))
                self.printFlush()
                self.resetDownloadMaxTry(nodeid)
                return
            
            self.downloadMaxTry[nodeid] += 1 # Increment the try count for the node.
        else:
            self.resetDownloadMaxTry(nodeid) # Reset the max try for the nodeid
            self.startNextDownload(channel) # Note that channel was the downloader id


    # Gavin Observation: I've never actually seen this function run before...
    # I am not totally positive however this function seems to have the same functionality as 
    #	the one above, but instead the downloader sends the message.
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
                self.mif[baseid].sendMsg(self.tos_source[baseid], baseid, CmdSerialMsg.AM_TYPE, 0x22, msg)
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

    # Send WREN status message to the motes
    # WREN status is used for downloading
    def scanMotes(self):
	self.sendMessage(CMD_WREN_STATUS, dst=102);

    def scanBaseStations(self):
	self.sendMessage(CMD_BASESTATUS, channel=-1, dst=102)
    
    def resetDownloadMaxTry(self, nodeid):
        self.downloadMaxTry[nodeid] = 0
        #del self.downloadMaxTry[nodeid]

    def resetDownloadMaxReTry(self, nodeid):
        self.downloadMaxRetry[nodeid] = 0
#        del self.downloadMaxRetry[nodeid]
        
    def resetChannel(self):
	self.sendMessage(CMD_CHANNEL_RESET, channel=11, dst=0xffff)

    def stopSensing(self, nodeid):
	self.sendMessage(CMD_STOP_SENSE)

    def startBlink(self, nodeid):
	self.sendMessage(CMD_START_BLINK)

    def stopBlink(self, nodeid):
	self.sendMessage(CMD_STOP_BLINK)

    def setBaseStationChannel(self):
	self.sendMessage(CMD_CHANNEL, channel=-1, dst=0xffff)

    def resetDownloadTimer(self):
        with self.lock:
            if not self.downloadTimer.isAlive():
                self.downloadTimer.start()
            self.downloadTimer.reset()

    # Print the log size (amount of data) on each mote.         
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

# Function commented out because it is not being used. - Gavin
#    def printWRENStatus(self):
#        wrenkeys = self.wrenmsgs.keys()
#        wrenkeys.sort()
#        for id in wrenkeys:
#            m = self.wrenmsgs[id]
#            sys.stdout.write("id: %4d, sensing: %d\n"%(m.get_src(), m.get_sensing()))
#
#        sys.stdout.write("%d messages received !\n"%(len(self.wrenmsgs)))
#        sys.stdout.flush()
#
#        self.wrenmsgs = {}

    # 
    def printConnection(self):
        wrenkeys = self.wrenconnectionmsgs.keys()
        wrenkeys.sort()
        for id in wrenkeys:
            m = self.wrenconnectionmsgs[id]
            sys.stdout.write("id: %4d, is Acked: %d\n"%(m.get_src(), m.get_isAcked()))

        sys.stdout.write("%d messages received !\n"%(len(self.wrenconnectionmsgs)))
        sys.stdout.flush()

    	self.wrenconnectionmsgs = {}

    def sendMessage(self, MSG_TYPE, nodeid=0xffff, channel=None, dst=None):
	'''
		Method sends the message of MSG_TYPE to all the nodes.
	
		If the message is only supposed to be sent to a specific node the 
			parameter nodeid should be set accordingly. 
	'''

	msg = CmdSerialMsg.CmdSerialMsg() # Create message instance
	msg.set_cmd(MSG_TYPE) 		  # Set the type of message
	msg.set_dst(nodeid)		  # Set destination of message
	if(dst is not None):
	    msg.set_dst(dst)
	for n in self.m.get_nodes():	  # For all the motes available in the mni
	    if(channel is -1):		  
		msg.set_channel(n.id)	  # Channel specification of -1 sets the channel to n.id for all specific nodes
	    elif(channel is not None):	  # If a channel was specified and it wasn't -1 then set it.
		msg.set_channel(channel)
	    # Send the message
	    # Print out the n.id to make sure it is the commander sending the message
	    self.mif[n.id].sendMsg(self.tos_source[n.id], nodeid, CmdSerialMsg.AM_TYPE, 0x22, msg) 

    # Print the length of each mote queue. WREN motes that have data to download and Downloaders that are connected
    def printMoteQueues(self):
        print "***", len(self.motes), "client motes have data and queued for download"
        print "***", len(self.basestations), "download base stations are ready for download"

    # Not only print the length of the mote queues but the contents as well.
    def printMoteQueueDetail(self):
        print "***", len(self.motes), "client motes have data and queued for download"
        print "client node id:"
        for elem in self.motes:
            print elem
        
        print "***", len(self.basestations), "download base stations are ready for download"
        print "base station id:"
        for elem in self.basestations:
            print elem

    # Print information to log file and stdout
    def printWrite(self, line):
        self.logger.write(line)
        sys.stdout.write(line)

    # Print line to both the log file and stdout
    def printWriteLine(self, line):
        self.logger.writeLine(line)
        sys.stdout.write(line)

    # Flushes both the logger and stdout output
    def printFlush(self):
        self.logger.flush()
        sys.stdout.flush()
        
    # Basic menu that shows the users options
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

    def test_average_timing(self, msg_type, itrs=1):
	self.downloadMode = DOWNLOAD_ALL
	self.scanMotes()
	self.setBaseStationChannel()
	time.sleep(10)
	for _ in range(itrs):
		print "first"
		for nodeid in self.motes:
			print "second"
			for baseid in self.basestations:
				print "third"
				self.StartTime = time.time()
				msg = CmdSerialMsg.CmdSerialMsg()
				msg.set_cmd(msg_type)
				msg.set_dst(baseid)
				msg.set_channel(baseid)
				self.mif[0].sendMsg(self.tos_source[0], nodeid, CmdSerialMsg.AM_TYPE, 0x22, msg)
				time.sleep(35)	
	max_t = -1 
	print "----- Printing Response Times -----"
	for t in self.avg_resp_times:
		max_t = t > max_t and t or max_t
		print "\t" + str(t)
	print "\nMax time: " + str(max_t)
	print "Avg num of mote responses: " + str(len(self.avg_resp_times) / itrs)
	print "-----------------------------------"
	self.avg_resp_times = []
    
    def main_loop(self):

        self.help()
        
        while 1:
	    c = raw_input() # Get user input
            self.downloadState = DOWNLOAD_START
            
	    # Write the input to the log for records
            self.printWrite("%s, command: %s\n"%(time.ctime(), c))
            self.printFlush()
            
            if c == 'p':
		# Reports the number of available Downloaders
		# 	and how many motes are ready for download.
                self.printMoteQueueDetail()
	    elif c == 'z':
		self.test_average_timing(CMD_DOWNLOAD)
            elif c == 'q':
		print("A QUIT was requested")
		# quits and safely exits the program
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
		self.sendMessage(CMD_BASESTATUS, channel=-1, dst=102)
            elif c == 'x':
		# Reset channel
                self.downloadState = DOWNLOAD_STOP
                self.resetChannel()
            elif c == 'g':
                # start sensing
		self.sendMessage(CMD_START_SENSE);
            elif c == 'b':
		# Stop sensing
                self.downloadState = DOWNLOAD_STOP
		self.sendMessage(CMD_STOP_SENSE)
            elif c == 't':
		# Start blinking
		self.sendMessage(CMD_START_BLINK)
            elif c == 'a':
		# Stop blinking
		self.sendMessage(CMD_STOP_BLINK)
            elif c == 'e':
		# Erase data from the motes
                c = raw_input("Are you sure you want to erase the data [y/n]")
                if c != 'y':
                    print "Abort erase"
                    continue
		self.sendMessage(CMD_ERASE);
            elif c == 'd':
		# Testing for New DownlaodFunction Gavin
		if not __debug__:
		    self._downloadData()
		    continue
			
		
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
		    # Get status of individual mote
                    cs = c.strip().split(" ")
                    nodeid = int(cs[1])
                    # get status
		    self.sendMessage(CMD_STATUS, nodeid=nodeid)
		elif c[0] == 'z' and c[1] == ' ':
		    cs = c.strip().split(" ")
		    self.test_average_timing(CMD_DOWNLOAD, itrs=int(cs[1]))
                elif c[0] == 'b' and c[1] == ' ':
		    # Stop sensing particular mote
                    cs = c.strip().split(" ")
                    nodeid = int(cs[1])
		    self.sendMessage(CMD_STOP_SENSE, nodeid=nodeid)
                elif c[0] == 't' and c[1] == ' ':
		    # Start blinking particular mote
                    cs = c.strip().split(" ")
                    nodeid = int(cs[1])
		    self.sendMessage(CMD_START_BLINK, nodeid=nodeid)
                elif c[0] == 'a' and c[1] == ' ':
		    # Stop blinking particular mote
                    cs = c.strip().split(" ")
                    nodeid = int(cs[1])
		    self.sendMessage(CMD_STOP_BLINK, nodeid=nodeid)
                elif c[0] == 'g' and c[1] == ' ':
		    # Start sensing particular mote
                    cs = c.strip().split(" ")
                    nodeid = int(cs[1])
                    # get status
		    self.sendMessage(CMD_START_SENSE, nodeid=nodeid)
                elif c[0] == 'e' and c[1] == ' ':
		    # Erase particular mote
                    cs = c.strip().split(" ")
                    nodeid = int(cs[1])
                    # get status
                    c = raw_input("Are you sure you want to erase the data for node " + str(nodeid) + " [y/n]")
                    if c != 'y':
                        print "Abort erase"
                        continue
		    self.sendMessage(CMD_ERASE, nodeid=nodeid)
                elif c[0] == 'd':
		    # Download particular mote
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
		    # Restore a particular mote
                    cs = c.strip().split(" ")
                    nodeid = int(cs[1])
		    self.sendMessage(CMD_LOGSYNC, nodeid=nodeid)
            elif c == 'h':
		# Print the help menu 
                self.help()


def main():
    # Create an instance of CmdCenter
    cc = CmdCenter()
    # Run the loop
    # This shouldn't return unless the user presses q
    cc.main_loop()

if __name__ == "__main__":
    main()

