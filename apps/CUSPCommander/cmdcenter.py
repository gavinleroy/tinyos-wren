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
    motes = []
#    downloadingMotes = []
    
    basemsgs = {}
    wrenmsgs = {}
    wrenconnectionmsgs = {}
    basestations = {}
    
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
        
        #self.downloadTimer = ResettableTimer(3, self.checkDownloadAll)

        # connecting serial forwarder for all nodes
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
        time.sleep(3)

        for n in self.m.get_nodes():
            #print n.id
            self.mif[n.id] = MoteIF.MoteIF()
            self.tos_source[n.id] = self.mif[n.id].addSource("sf@localhost:%d"%(20000+n.id))


        #add us as listener for the different messages
        for n in self.m.get_nodes():
            #self.mif[n.id].addListener(self, CmdSerialMsg.CmdSerialMsg)
            #self.mif[n.id].addListener(self, MoteStatusMsg.MoteStatusMsg)
            #self.mif[n.id].addListener(self, MoteRssiMsg.MoteRssiMsg)
            self.mif[n.id].addListener(self, RssiSerialMsg.RssiSerialMsg)
            self.mif[n.id].addListener(self, SerialStatusMsg.SerialStatusMsg)
            self.mif[n.id].addListener(self, BaseStatusMsg.BaseStatusMsg)
            self.mif[n.id].addListener(self, WRENStatusMsg.WRENStatusMsg)
            self.mif[n.id].addListener(self, WRENConnectionMsg.WRENConnectionMsg)
            self.mif[n.id].addListener(self, WRENCloseMsg.WRENCloseMsg)


    def receive(self, src, msg):
        #print time.time(), msg.addr
        #if msg.get_amType() == CmdSerialMsg.AM_TYPE:
        #    m = CmdSerialMsg.CmdSerialMsg(msg.dataGet())
        #    print m

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

#            if m.get_dst() not in self.downloadingMotes:
#                self.downloadingMotes.append(m.get_dst())

            #sys.stdout.write("dst: %d, src, %d, c: %d, rssi, %d, sloc: %d, sglob: %d, dstloc: %d, dstglob: %d, Sync: %d, reboot: %d, bat: %.2f, size: %d\n"%(m.get_dst(), m.get_src(), m.get_counter(), m.get_rssi(), m.get_srclocaltime(), m.get_srcglobaltime(), m.get_localtime(), m.get_globaltime(), m.get_isSynced(), m.get_reboot(), m.get_bat()/4096.0*5, m.get_size()))
            #sys.stdout.write("dst: %d, src, %d, size: %d\n"%(m.get_dst(), m.get_src(), m.get_size()))

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
            
            # collect all client motes out there
            # print("exist", m.get_src(), self.motes.count(m.get_src()))
            #if self.motes.count(m.get_src()) == 0:
            #    self.motes.append(m.get_src())

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

            # collect all client motes out there
            # print("exist", m.get_src(), self.motes.count(m.get_src()))
#            if self.motes.count(m.get_src()) == 0 and m.get_logsize() > 0:
#                self.motes.append(m.get_src())

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
        for baseid in self.basestations:
            if self.basestations[baseid] == nodeid:
                exist = True
                break
        return exist

    def getNextDownload(self):
        nodeid = DOWNLOAD_END
        if self.downloadState == DOWNLOAD_START:
            if len(self.motes) > 0:
                nodeid = self.motes.pop()
            else:
                if self.downloadMode == DOWNLOAD_ALL:
                    self.scanMotes() #find all motes out there
                    time.sleep(3)
                    if len(self.motes) > 0: #if we have motes to download ...
                        nodeid = self.motes.pop()  # go ahead and pop one
        if nodeid == DOWNLOAD_END:
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

#    def checkCurrentDownload(self, baseid, nodeid):
#        if nodeid not in self.logSize.keys(): #This means not started yet. 
#            self.downloadCommandController(baseid, nodeid)
#        else:
#            if self.logSize[nodeid] > 0:
#                if self.moteLogSize[nodeid] != self.logSize[nodeid]: # this means still downloading...
#                    self.downloadMaxTry[nodeid] = 0
#                    self.moteLogSize[nodeid] = self.logSize[nodeid]
#                else:
#                    self.downloadMaxTry[nodeid] += 1
#                    if self.downloadMaxTry[nodeid] > 2: # greater than 3 time trials, give up for the mote
#                        self.startNextDownload(baseid) #give up and move onto the next
#                    else:
#                        self.downloadCommandBaseStation(baseid, nodeid)
#            else:
#                if nodeid in self.f.keys():
#                    self.f[nodeid].flush()
#                    self.f[nodeid].close()
#                    del self.f[nodeid]
#                sys.stdout.write("%d Download Done\n" % (nodeid,))
#                sys.stdout.flush()
#                self.startNextDownload(baseid)                

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
        nodeid = self.getNextDownload()
        if nodeid > 0:
            if not self.existDownload(nodeid): #if it is not already downloading...
                self.basestations[baseid] = nodeid
                self.downloadMaxTry[nodeid] = 0
                self.moteLogSize[nodeid] = 0
                self.downloadCommandController(baseid, nodeid) # send the download command to Controller
            else:
                if self.downloadState == DOWNLOAD_STOP:
                    return;

                if nodeid != DOWNLOAD_END:
                    self.startNextDownload(baseid)
                
    def downloadData(self):
        print "downloadData mapping..."
        self.printDownloadMapping()
        
        if not self.isBusy:
            self.isBusy = True
            for baseid, nodeid in self.basestations.iteritems(): # we can optimize this lookup
                if self.downloadState == DOWNLOAD_STOP:
                    return

                if nodeid == 0:
                    self.startNextDownload(baseid)
                else:
                    self.checkCurrentDownload(baseid, nodeid) #check to see if the download is still in progress

            self.isBusy = False    
        
        with self.lock:
            self.downloadTimer.reset()
                
    def downloadCommandController(self, channel, nodeid):
        #self.resetDownloadTimer()
        if self.downloadMaxTry[nodeid] < DOWNLOAD_MAX_TRY:
            if self.downloadState == DOWNLOAD_STOP:
                self.resetDownloadMaxTry(nodeid)
                return;

            if nodeid not in self.logSize.keys():
                print("send download command to Controller ..")
                msg = CmdSerialMsg.CmdSerialMsg()
                msg.set_cmd(CMD_DOWNLOAD)
                msg.set_dst(channel)
                msg.set_channel(channel)
                
                print (channel, nodeid)
        #        for n in self.m.get_nodes():
                #self.resetDownloadTimer()
        #            self.mif[n.id].sendMsg(self.tos_source[n.id], nodeid, CmdSerialMsg.AM_TYPE, 0x22, msg)
                self.mif[0].sendMsg(self.tos_source[0], nodeid, CmdSerialMsg.AM_TYPE, 0x22, msg)
                #self.downloadTimer.reset()
                #time.sleep(1)
                #self.downloadTimer.reset()
            else:
                self.printWrite("download start: %s, channel: %s, nodeid: %s\n"%(time.ctime(), channel, nodeid))
                self.printFlush()
                self.resetDownloadMaxTry(nodeid)
                #time.sleep(1)
                #self.downloadTimer.reset()
                return
            
            self.downloadMaxTry[nodeid] += 1
        else:
            self.resetDownloadMaxTry(nodeid)
            self.startNextDownload(channel)

            #self.basestations[channel] = 0
                #self.downloadCommandController(channel, nodeid)


    def downloadCommandBaseStation(self, baseid, nodeid):
        #self.resetDownloadTimer()
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
                        #self.resetDownloadTimer()
                        #self.mif[0].sendMsg(self.tos_source[0], nodeid, CmdSerialMsg.AM_TYPE, 0x22, msg)
                        self.mif[baseid].sendMsg(self.tos_source[baseid], baseid, CmdSerialMsg.AM_TYPE, 0x22, msg)
                        #self.downloadTimer.reset()
                        #time.sleep(1)
                        #self.mif[baseid].sendMsg(self.tos_source[baseid], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)
                        #self.mif[baseid].sendMsg(self.tos_source[baseid], nodeid, CmdSerialMsg.AM_TYPE, 0x22, msg)
                        #self.downloadTimer.reset()
                else:
                    self.moteLogSize[nodeid] = self.logSize[nodeid]
                    self.printWrite("download restart: %s, channel: %s, nodeid: %s\n"%(time.ctime(), baseid, nodeid))
                    self.printFlush()
                    self.resetDownloadMaxReTry(nodeid)
                    #self.downloadTimer.reset()
                    return
            
            self.downloadMaxRetry[nodeid] += 1

        #time.sleep(1)

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
        #self.downloadMaxTry[nodeid] = 0
        del self.downloadMaxTry[nodeid]

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
        for baseid, nodeid in self.basestations.iteritems(): # we can optimize this lookup
            print "mapping:", baseid, nodeid
            
#    def printStatus(self):
#        keys = self.msgs.keys()
#        keys.sort()
#        for id in keys:
#            m = self.msgs[id]
#            sys.stdout.write("id: %4d, sensing: %d, local: %d, global: %d, isSync: %d, reboot: %d, bat: %.2f, logsize: %d, isErased: %d, download: %d, channel: %d,\n"%(m.get_src(), m.get_sensing(), m.get_localtime(), m.get_globaltime(), m.get_isSynced(), m.get_reboots(), m.get_bat()/4096.0*5, m.get_buffersize(), m.get_isErased(), m.get_download(), m.get_channel()))
#        
#        sys.stdout.write("%d messages received!\n"%(len(self.msgs)))
#        sys.stdout.flush()
#
#        self.msgs = {}

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

    def main_loop(self):

        self.help()
        
        #self.scanMotes()
        
        while 1:
            c = raw_input()
            self.downloadState = DOWNLOAD_START
            
            #self.printWrite("%.3f, command: %s\n"%(time.time(), c))
            self.printWrite("%s, command: %s\n"%(time.ctime(), c))
            self.printFlush()
            
            if c == 'p':
                self.printMoteQueueDetail()
            
            if c == 'q':
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
                
                msg = CmdSerialMsg.CmdSerialMsg()
                msg.set_cmd(CMD_STATUS)
                msg.set_dst(102)
                #msg.set_nodeId()
                #msg.set_deploymentId()
                #msg.set_syncPeriod()
                for n in self.m.get_nodes():
                    self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)
            elif c == 'w':
                self.scanMotes()
                self.setBaseStationChannel()
            elif c == 'f':
                # get base status
                msg = CmdSerialMsg.CmdSerialMsg()
                msg.set_cmd(CMD_BASESTATUS)
                msg.set_dst(102)
                #msg.set_dst(0xffff)
                #msg.set_nodeId()
                #msg.set_deploymentId()
                #msg.set_syncPeriod()
                for n in self.m.get_nodes():
                    msg.set_channel(n.id)
                    self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)
            elif c == 'x':
                self.downloadState = DOWNLOAD_STOP
                self.resetChannel()
            elif c == 'g':
                # start sensing
                msg = CmdSerialMsg.CmdSerialMsg()
                msg.set_cmd(CMD_START_SENSE)
                #msg.set_dst(0xffff)
                for n in self.m.get_nodes():
                    self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)

                #self.stopBlink(0xffff)

            elif c == 'b':
                self.downloadState = DOWNLOAD_STOP
                self.stopSensing(0xffff)
            elif c == 't':
                self.startBlink(0xffff)
            elif c == 'a':
                self.stopBlink(0xffff)
            elif c == 'e':
                c = raw_input("Are you sure you want to erase the data [y/n]")
                if c != 'y':
                    print "Abort erase"
                    continue
                msg = CmdSerialMsg.CmdSerialMsg()
                msg.set_cmd(CMD_ERASE)
                #msg.set_dst(0xffff)
                for n in self.m.get_nodes():
                    self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)
            elif c == 'd':
                self.downloadMode = DOWNLOAD_ALL
#                if not self.downloadTimer.isAlive():
#                    self.downloadTimer.start()
#                self.downloadTimer.reset()

                # start download
                # stop all motes first
                self.scanMotes()
                self.setBaseStationChannel()

                time.sleep(3) #give sometime to settle down                
                self.stopSensing(0xffff)
#                # get the first group of motes for download and set channels for base stations
#                self.scanMotes()
#                # set base channels
#                self.setBaseStationChannel()
                # start download now
                
                self.resetDownloadTimer()
                self.downloadData()
            elif c == 'r':
                # restore log
                c = raw_input("Are you sure you want to restore log? [y/n]")
                if c != 'y':
                    print "Abort restore"
                    continue
                msg = CmdSerialMsg.CmdSerialMsg()
                msg.set_cmd(CMD_LOGSYNC)
                #msg.set_dst(0xffff)
                for n in self.m.get_nodes():
                    self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)
            elif len(c) > 1:
                if c[0] == 's' and c[1] == ' ':
                    cs = c.strip().split(" ")
                    nodeid = int(cs[1])
                    # get status
                    msg = CmdSerialMsg.CmdSerialMsg()
                    msg.set_cmd(CMD_STATUS)
                    msg.set_dst(nodeid)
                    for n in self.m.get_nodes():
                        self.mif[n.id].sendMsg(self.tos_source[n.id], nodeid, CmdSerialMsg.AM_TYPE, 0x22, msg)
                elif c[0] == 'b' and c[1] == ' ':
                    cs = c.strip().split(" ")
                    nodeid = int(cs[1])
                    self.stopSensing(nodeid)
                elif c[0] == 't' and c[1] == ' ':
                    cs = c.strip().split(" ")
                    nodeid = int(cs[1])
                    self.startBlink(nodeid)
                elif c[0] == 'a' and c[1] == ' ':
                    cs = c.strip().split(" ")
                    nodeid = int(cs[1])
                    self.stopBlink(nodeid)
                elif c[0] == 'g' and c[1] == ' ':
                    cs = c.strip().split(" ")
                    nodeid = int(cs[1])
                    # get status
                    msg = CmdSerialMsg.CmdSerialMsg()
                    msg.set_cmd(CMD_START_SENSE)
                    msg.set_dst(nodeid)
                    for n in self.m.get_nodes():
                        self.mif[n.id].sendMsg(self.tos_source[n.id], nodeid, CmdSerialMsg.AM_TYPE, 0x22, msg)
                elif c[0] == 'e' and c[1] == ' ':
                    cs = c.strip().split(" ")
                    nodeid = int(cs[1])
                    # get status
                    c = raw_input("Are you sure you want to erase the data for node " + str(nodeid) + " [y/n]")
                    if c != 'y':
                        print "Abort erase"
                        continue
                    msg = CmdSerialMsg.CmdSerialMsg()
                    msg.set_cmd(CMD_ERASE)
                    msg.set_dst(nodeid)
                    for n in self.m.get_nodes():
                        self.mif[n.id].sendMsg(self.tos_source[n.id], nodeid, CmdSerialMsg.AM_TYPE, 0x22, msg)
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
                    self.stopSensing(nodeid)
                    time.sleep(3) #give sometime to settle down                

                    if self.motes.count(nodeid) == 0:
                        self.motes.append(nodeid)

                    self.downloadData()
                    
#                    msg = CmdSerialMsg.CmdSerialMsg()
#                    msg.set_cmd(CMD_DOWNLOAD)
#                    msg.set_dst(nodeid)
#                    msg.set_channel(16)
#                    
#                    # now send command to the controller
#                    self.mif[0].sendMsg(self.tos_source[0], nodeid, CmdSerialMsg.AM_TYPE, 0x22, msg)
#                    for n in self.m.get_nodes():
#                        self.mif[n.id].sendMsg(self.tos_source[n.id], nodeid, CmdSerialMsg.AM_TYPE, 0x22, msg)
                                        
                elif c[0] == 'r':
                    cs = c.strip().split(" ")
                    nodeid = int(cs[1])
                    msg = CmdSerialMsg.CmdSerialMsg()
                    msg.set_cmd(CMD_LOGSYNC)
                    msg.set_dst(nodeid)
                    for n in self.m.get_nodes():
                        self.mif[n.id].sendMsg(self.tos_source[n.id], nodeid, CmdSerialMsg.AM_TYPE, 0x22, msg)

            elif c == 'h':
                self.help()


def main():

    cc = CmdCenter()
    cc.main_loop()  # don't expect this to return...


if __name__ == "__main__":
    main()

