#!/usr/bin/env python

# Last edited by Gavin Gray at the University of Utah
#	December 19, 2019
#

from mni import mni

import os
import sys
import time
import optparse

import threading # For new download process threads
import tqdm # For the progress bars
import time # Added for timing the motes and date

from itertools import islice


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

DOWN_NOT_STARTED    = 42
DOWN_CURRENT        = 43
DOWN_FINISHED       = 44
DOWN_ERROR 	    = 45

# Path for the failed download mote logs
FAIL_PATH = "/failed_downloads_"
# Path for the log file
LOG_PATH = "/download_log_stat.txt"
# Create directory for LOG files. All commands are written to this file.
# Directories are named by the day of use.
BASE_DIR = time.strftime("%m-%d-%Y", time.localtime())
if not os.path.exists(BASE_DIR):
    os.mkdir(BASE_DIR)


class CmdCenter:
    
    # Dictionary id:MoteIF instance. MoteIF can be found in MoteIF.py creates a mote instance
    mif = {}

    sfprocess = {}

    # Dictionary that maps id's to their source instances
    tos_source = {}

    # Dictionary, mapping SerialStatusMSG sources to their messages. Used only for SerialStatus Messages
    msgs = {}

    # Dictionary mapping moteid's to their logsizes
    logSize = {} 
    
    lock = threading.RLock()

    progressLock = threading.RLock()
    
    # Dictionary, mapping nodeid's to their file objects.
    f = {}

    # List of motes connected
    motes = [] 

    # List of motes to be downloaded
    motes_to_download = []

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

    # Progress bars for the download process
    prog_bars = {}
    mote_base = {}
    
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

	# Log file for all commands run.
        self.logger = Logger(BASE_DIR+LOG_PATH)
        
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
	# 	altered to 5 seconds because it was always behind when
	#	more than 2 downloaders were connected
        time.sleep(5) 

	# Initialize node instances with their ID's and save the source to the id.
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

    def receive_BaseStatusMsg(self, src, msg):
	m = BaseStatusMsg.BaseStatusMsg(msg.dataGet())
        with self.lock:
	    if not self.basemsgTimer.isAlive():
	        self.basemsgTimer.start()
	    self.basemsgTimer.reset()
	    self.basemsgs[m.get_src()] = m

            # collect all base stations out there
            if m.get_src() not in self.basestations.keys():
                self.basestations[m.get_src()] = 0

    def receive_WRENStatusMsg(self, src, msg):
	m = WRENStatusMsg.WRENStatusMsg(msg.dataGet())
	with self.lock:
	    if not self.wrenTimer.isAlive():
	        self.wrenTimer.start()
	    self.wrenTimer.reset()
	    self.wrenmsgs[m.get_src()] = m
	    if self.motes.count(m.get_src()) == 0 and m.get_buffersize() > 0:
	        self.motes.append(m.get_src())

    def receive_SerialStatusMsg(self, src, msg):
	m = SerialStatusMsg.SerialStatusMsg(msg.dataGet())
        with self.lock:
	    if not self.msgTimer.isAlive():
	        self.msgTimer.start()
	    self.msgTimer.reset()

	    if m.get_src() not in self.msgs.keys():
	        self.msgs[m.get_src()] = m

        if m.get_src() == 1:
	    # this is the base mote. Store it's time for sync in file
	    f = open(BASE_DIR+"/timesync.log", "a+")
	    f.write("%.3f, %d\n"%(time.time(), m.get_globaltime()))
	    f.close()


    def receive_WRENConnectionMsg(self, src, msg):
	m = WRENConnectionMsg.WRENConnectionMsg(msg.dataGet())
        with self.lock:
	    if not self.wrenConnectionTimer.isAlive():
	        self.wrenConnectionTimer.start()
	    self.wrenConnectionTimer.reset()
	    self.wrenconnectionmsgs[m.get_src()] = m

    def receive_WRENCloseMsg(self, src, msg):
	m = WRENCloseMsg.WRENCloseMsg(msg.dataGet())
        with self.lock:
	    if m.get_close() == 1:
	        # SET THE DOWNLOAD STATUS TO FINISHED
	        self.download_status[m.get_src()] = DOWN_FINISHED
		self.prog_bars[m.get_channel()].set_description('Terminating processes : %d'%m.get_src())

    def receive_RssiSerialMsg(self, src, msg):
	m = RssiSerialMsg.RssiSerialMsg(msg.dataGet())
	if m.get_dst() == 0 or m.get_dst() not in self.basestations.values():
	    return

	with self.lock:
	    # Add logsize to the dictionary, indicating that the download started
	    self.logSize[m.get_dst()] = m.get_size() # now we add the node to the logsize dictionary

	    if m.get_size() > 0:
	        self.downloadMaxTry[m.get_dst()] = 0 # Reset the download tries
	    elif m.get_size() == 0:
	        self.download_status[m.get_dst()] = DOWN_FINISHED

	if (self.dl%200) == 0:
	    sys.stdout.write(".")
	    sys.stdout.flush()
	self.dl += 1
	# If we don't have a log for the given node, first we must open a file.
	if m.get_dst() not in self.f.keys():
	    self.prog_bars[self.mote_base[m.get_dst()]].reset(total=m.get_size())
	    self.prog_bars[self.mote_base[m.get_dst()]].set_description('Downloading : %d'%m.get_dst())
	    self.openMoteLog(m.get_dst())
	# Update the progress bar
	self.prog_bars[self.mote_base[m.get_dst()]].update()

	# WRITE THE DATA TO THE LOG FILE
	self.f[m.get_dst()].write("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %.2f, %d\n"%(m.get_dst(), m.get_src(), m.get_counter(), m.get_rssi(), m.get_srclocaltime(), m.get_srcglobaltime(), m.get_localtime(), m.get_globaltime(), m.get_isSynced(), m.get_reboot(), m.get_bat()/4096.0*5, m.get_size()))
	self.f[m.get_dst()].flush() # Flush the pipe
	

    # Whenever a message is received from a mote this function is called.
    def receive(self, src, msg):
	'''
	Method is called whenever a message is received on one of the channels (downloaders)
	'''
        # Message download packet from the motes
        if msg.get_amType() == RssiSerialMsg.AM_TYPE:
	    self.receive_RssiSerialMsg(src, msg)
                
	# Message is received when the status of the motes is requested
        elif msg.get_amType() == SerialStatusMsg.AM_TYPE:
	    self.receive_SerialStatusMsg(src, msg)
            
        elif msg.get_amType() == WRENStatusMsg.AM_TYPE:
	    self.receive_WRENStatusMsg(src, msg)

        elif msg.get_amType() == WRENConnectionMsg.AM_TYPE:
	    self.receive_WRENConnectionMsg(src, msg)

	# Downloading Finished
        elif msg.get_amType() == WRENCloseMsg.AM_TYPE:
	    self.receive_WRENCloseMsg(src, msg)
                                
        elif msg.get_amType() == BaseStatusMsg.AM_TYPE:
	    self.receive_BaseStatusMsg(src, msg)

    def openMoteLog(self, nodeid):
	'''
	Method opens a file for the given nodeid in order for us to write the downloaded data.
	'''
	with self.lock:
	    if nodeid in self.basestations.values():
		self.download_status[nodeid] = DOWN_CURRENT
		self.f[nodeid] = open(BASE_DIR+"/node_%d.log"%(nodeid), "a+")

    def closeMoteLog(self, nodeid):
	'''
	Method closes the file for the given nodeid IF it existed. 
	After closing we want to delete it from the dictionary to keep small quantities.
	'''
	with self.lock:
	    if nodeid in self.f.keys(): # if the id has a log open
		self.f[nodeid].flush() # Clear the pipe
		self.f[nodeid].close() # Close the file
		del self.f[nodeid] # Delete the object from the dictionary

    def findNextFile(self, fp):
	''' 
	Takes in a file path as a parameter. Appends a number to the end of the file path 
	until the file does not exist.
	'''
	n = 0
	while os.path.exists(BASE_DIR+fp+str(n)):
	    n += 1
	return BASE_DIR+fp+str(n)

    def clearDownloadByNodeId(self, nodeid):
        cleared = False
        for baseid, value in self.basestations.iteritems(): 
            if value == nodeid:
                self.basestations[baseid] = 0
                
                cleared = True
                break
        return cleared

    def finishDownload(self, baseid, nodeid):
	'''
	Cleans up the download of a particular mote.
	unassociates the downloader from the mote and closes the specific log file.
	'''
        self.logger.write("download end: %s, channel: %s, nodeid: %s\n"%(time.ctime(), baseid, nodeid))
	with self.lock:
	    self.basestations[baseid] = 0
	    del self.mote_base[nodeid]
        self.printFlush()
	self.closeMoteLog(nodeid)

    def startDownload(self, baseid, nodeid):
	'''
	Method to initiate the download of a particular mote.
	Associates the downloader with the mote and vice-versa
	Sets status of download to DOWN_NOT_STARTED
	Sets number of tries to 0
	'''
        self.logger.write("download start: %s, channel: %s, nodeid: %s\n"%(time.ctime(), baseid, nodeid))
	with self.lock:
	    self.basestations[baseid] = nodeid # Associate Baseid with Nodeid that will be downloaded 
	    self.mote_base[nodeid] = baseid
	    self.download_status[nodeid] = DOWN_NOT_STARTED # Start the status as NOT STARTED
	    self.downloadMaxTry[nodeid] = 0 # Initialize the try count to 0. 

    def moteDownloadCommander(self, baseid, nodeid):
	'''
	moteDownloadCommaner sends a download message to the mote:nodeid from the commander
	in which it requests the return be sent to baseid. 

	This method has proven to be
	more stable when downloading however I doubt its ability to handle concurrent downloads
	well.
	'''
	msg = CmdSerialMsg.CmdSerialMsg()
	msg.set_cmd(CMD_DOWNLOAD)
	msg.set_dst(baseid)
	msg.set_channel(baseid)
	with self.lock:
	    self.mif[0].sendMsg(self.tos_source[0], nodeid, CmdSerialMsg.AM_TYPE, 0x22, msg)

    # Sends message to mote from downloader that requests data
    def moteDownloadDownloader(self, baseid, nodeid):
	'''
	This method was copied from the original download process and I am still unsure what it is supposed to do.
		I AM CURRENTLY NOT USING THIS METHOD.
	'''
	msg = CmdSerialMsg.CmdSerialMsg()
	msg.set_cmd(CMD_DOWNLOAD)
	msg.set_dst(nodeid)
	msg.set_channel(baseid)
	with self.lock:
	    self.mif[baseid].sendMsg(self.tos_source[baseid], baseid, CmdSerialMsg.AM_TYPE, 0x22, msg)

    def handleMoteDownload(self, baseid, nodeid):
	'''
	The current status codes are:
		- DOWN_NOT_STARTED
		- DOWN_CURRENT
		- DOWN_FINISHED
		- DOWN_ERROR
	if the logSize of the nodeid is greater than zero, this means that the process has not completed yet
	and we need to continue sending download signals to the mote.
	'''
	self.startDownload(baseid, nodeid)	
	# Created to just simplify the conditions for the while loop
	a = self.downloadMaxTry[nodeid]
	b = self.download_status[nodeid] 

	# While the try count is less than MAX_TRY and status isn't FINISHED
	# We will try to download, 
	while a < DOWNLOAD_MAX_TRY and b != DOWN_FINISHED and b != DOWN_ERROR:
	    try:
	        # Complete action based on the current status of the mote.
	        if self.download_status[nodeid] == DOWN_NOT_STARTED:
		    self.prog_bars[baseid].update() # udpate saying that we sent a download requeset to mote.
		    self.moteDownloadCommander(baseid, nodeid)
	        elif self.download_status[nodeid] == DOWN_CURRENT:
		    # For current downloads the progress bar is updated in the receive method
		    #	when messages are received with data.
		    self.moteDownloadCommander(baseid, nodeid)
	        elif self.download_status[nodeid] == DOWN_FINISHED or self.download_status[nodeid] == DOWN_ERROR:
		    break # Break if download is finished or if ERROR was received.
	        else: # the download status was not recognized as a valid option, throw an error.
		    raise Exception("Download Status not recognized: Fatal Error.")
	    except:
		with self.lock:
		    self.download_status[nodeid] = DOWN_ERROR	

	    time.sleep(10) # 10 Second buffer for sending messages
	    with self.lock:
	        self.downloadMaxTry[nodeid] += 1 # Increase the try count for the node

	    # Update our conditions for the while loop
	    a = self.downloadMaxTry[nodeid]
	    b = self.download_status[nodeid]

	# If the download wasn't successful let's write it to the file and save it into the list
	if self.download_status[nodeid] != DOWN_FINISHED:
	    with self.lock:
		msg = ""
		if self.download_status[nodeid] == DOWN_CURRENT:
		    msg = "Download Started and unfinished : "
		self.download_status[nodeid] = DOWN_ERROR
		self.failed_download.append(nodeid)
		self.f[-1].write(msg + "%d\n"%(nodeid))
	    
	# No matter if the download was successful we want to make sure to close the log file 
	# 	as to not corrupt it.
	time.sleep(10) # Make sure all processes finish
	self.finishDownload(baseid, nodeid)

    def moteThreadStart(self, baseid, motes):
	'''
	This method is the outer layer of each downloading thread.
	It will be responsible for associating a baseid with a nodeid and
	ensuring that all the motes get downloaded successfully or if not they
	are reported.
	'''
	while motes: # For all the given motes
	    with self.lock: # We need to lock because this list is shared amongst all threads.	
		nodeid = 0
		while nodeid == 0: # Make sure that we don't get a commander
		    nodeid = motes[-1] # Save the nodeid to be downloaded
		    motes.pop() # pop from the stack
	    # Set the progress bar for the next download
	    self.prog_bars[baseid].reset(total=10)
	    self.prog_bars[baseid].set_description('Initiaing Download %d : %d'%(baseid,nodeid))
	    self.handleMoteDownload(baseid, nodeid) # Handle the download of the nodeid
	    with self.lock:
		self.prog_bars[0].update() # The total download progress bar should be updated
					   # 	to reflect the completed mote, whether successful or not.
	    if self.download_status[nodeid] == DOWN_ERROR:
		continue # We need to do something for the errors that will possibly fix the system
        self.prog_bars[baseid].reset()
        self.prog_bars[baseid].set_description('Download Finished')

    def _downloadData(self, motes):
	'''
	This was created to ensure that all motes were downloaded and that the program
	would never stall. It processes the downloads concurrently with as many threads as
	there are downloaders connected. 

	parameters:
	-    motes: a list of all the motes that should be downloaded. The list is passed as a parameter
		    which allows the download of a specific subset of the moets.
	'''
	# Open a log file to store the failed mote downloads	
	self.f[-1] = open(self.findNextFile(FAIL_PATH), "a+")
	# Progress bar for the total download
	self.prog_bars[0] = tqdm.tqdm(total=len(motes),desc='Total Motes',position=0)
	_threads = list()
	# Prepare the individual threads for downloading
	for baseid, pos in zip(self.basestations.keys(), range(1,len(self.basestations)+1)):
	    # Add a progress bar for each downloader
	    self.prog_bars[baseid] = tqdm.tqdm(desc='Download Channel : %d'%baseid, position=pos)
	    # Add a thread to our current list of threads
	    _threads.append(threading.Thread(target=self.moteThreadStart, args=(baseid,motes,)))
	# Start all the threads
	for t in _threads:
	    t.start()
	# Join all the threads
	for t in _threads:
	    t.join()
	# Close all the progress bars.
	for pb in self.prog_bars.values():
	    pb.close()

	self.f[-1].write("Number of failed downloads: %d\n\n\n\n"%(len(self.failed_download)))
	self.closeMoteLog(-1) # Close the log file for failed mote downloads

    def fullDownload(self, motes, itrs=10):
	'''
	Method fullDownload will iterate over the _downloadData method until all motes are successful,
		or until the number of desired iterations is complete. The default number of iterations is 10.
		After each iteration the channel is reset, and the basestations re-contacted to ensure
		that the connection of the next iteration will be strong.

	If the pipe to one of the downloaders, or the commander, is broken then this 
		method will continue to run because the error is raised from the MOTEIF.py file.
		In this case none of the motes will be downloaded, the program will need to be restarted
		and the nodes (all telosB motes) will need to be reconnected.
	'''
	# Used for timing the download process
	print("Motes to be downloaded")
	print motes
	print("With %d iterations"%itrs)
	start_time = time.time()
	_motes = motes
	for _ in range(itrs):
	    if not _motes: # If no motes failed to download, then break
		break
	    self.printWriteLine("\nRESETTING CHANNELS BEFORE NEXT ITERATION\n\n")
	    # Prepare for next iteration
	    # Reset the Channel
	    self.resetChannel()
	    time.sleep(15) # Wait for channel reset
	    # Get all available downloaders
	    # I do this every iteration because it will filter out downloaders that have stopped responding
	    self.setBaseStationChannel()
	    time.sleep(15)
	    self._downloadData(_motes)
	    _motes = self.failed_download # Swap lists.
	    self.failed_download = []	
	end_time = time.time()	
	self.printWrite("\nCalculating download time for %d iterations\n"%(itrs))
	self.time_elapsed(start_time, end_time) # Calculate and print the time elapsed

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

    def resetDownloadMaxReTry(self, nodeid):
        self.downloadMaxRetry[nodeid] = 0
        
    def resetChannel(self):
	self.sendMessage(CMD_CHANNEL_RESET, channel=11, dst=0xffff)

    def stopSensing(self, nodeid):
	self.sendMessage(CMD_STOP_SENSE)

    def startBlink(self, nodeid):
	self.sendMessage(CMD_START_BLINK)

    def stopBlink(self, nodeid):
	self.sendMessage(CMD_STOP_BLINK)

    def setBaseStationChannel(self):
	self.basestations = {}
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

    def printConnection(self):
        wrenkeys = self.wrenconnectionmsgs.keys()
        wrenkeys.sort()
        for id in wrenkeys:
            m = self.wrenconnectionmsgs[id]
            sys.stdout.write("id: %4d, is Acked: %d\n"%(m.get_src(), m.get_isAcked()))

        sys.stdout.write("%d messages received !\n"%(len(self.wrenconnectionmsgs)))
        sys.stdout.flush()

    	self.wrenconnectionmsgs = {}


    def time_elapsed(self,start_time, end_time):
	'''
	Calculates the time elapsed and displays in Days, Hours, Minutes, Seconds.
	First parameter is the start time, second is the end time.
	'''
	total_time = end_time - start_time

	days = total_time // 86400
	total_time = total_time%86400

	hours = total_time // 3600
	minutes = (total_time % 3600) // 60
	seconds = ((total_time % 3600)%60)

	self.printWrite("\nTime Elapsed: \nD:%d, \nH:%d, \nM:%d, \nS:%d\n"%(days,hours,minutes,seconds))
	self.printFlush()

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
	for n in self.m.get_nodes():	  # For all the nodes available in the mni
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
		start_time = time.time()
                # get status
                self.f.clear()
       		self.sendMessage(CMD_STATUS)         
		end_time = time.time()

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
		# Set mode to download all
                self.downloadMode = DOWNLOAD_ALL
                # start download
                # stop all motes first
                self.scanMotes()
		# Give some time for the motes to settle down.
                time.sleep(10)
		# Make sure that sensing is off.
		self.sendMessage(CMD_STOP_SENSE)
		time.sleep(10)
		
		if not __debug__:
		    self.fullDownload(self.motes)
		else:
		    self.fullDownload(self.motes)

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
                    if c != 'y' or c.lower() != 'yes':
                        print "Abort erase"
                        continue
		    self.sendMessage(CMD_ERASE, nodeid=nodeid)
                elif c[0] == 'd':
		    cs = c[1:].strip().split(' ') # get the full input array
		    itrs=10
		    # check to see if the itrs was set
		    if cs[0] == '-i': # -i is the flag to set the iterations
			if len(cs) >= 2 and int(cs[1]) <= 30:
       			    itrs = int(cs[1]) # If it was and is reasonable then set it.
			cs.remove(cs[0]) # delete the flag
			cs.remove(cs[0]) # and the number of iterations
	  	    if not cs: # if the rest of the input is empty that means all the motes should be downloaded
	       	        self.scanMotes()
			time.sleep(10)
			self.fullDownload(self.motes, itrs=itrs)
		    elif not __debug__:
		        self.fullDownload([int(x) for x in cs], itrs=itrs)
		    else: # else we just want to download the motes specified
		        self.fullDownload([int(x) for x in cs], itrs=itrs)
                elif c[0] == 'r':
		    # Restore a particular mote
                    cs = c.strip().split(" ")
                    nodeid = int(cs[1])
		    self.sendMessage(CMD_LOGSYNC, nodeid=nodeid)
            elif c == 'h':
		# Print the help menu 
                self.help()

def main(): # Create an instance of CmdCenter
    cc = CmdCenter()
    # Run the loop
    # This shouldn't return unless the user presses q
    cc.main_loop()

if __name__ == "__main__":
    main()

