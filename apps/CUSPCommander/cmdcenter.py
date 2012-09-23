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

    f = {}
    motes = []
    
    basemsgs = {}
    wrenmsgs = {}
    downloaders = {}
    
    moteLogSize = {}
    downloadTrials = defaultdict(int)
    
    
    def __init__(self):
        print "init"

        self.dl = 0

        self.m = mni.MNI()
        self.basemsgTimer = ResettableTimer(2, self.printBaseStatus)
        self.msgTimer = ResettableTimer(3, self.printStatus)
        self.downloadTimer = ResettableTimer(5, self.check_Download)
#        self.wrenTimer = ResettableTimer(2, self.printWRENStatus)
        self.wrenTimer = ResettableTimer(2, self.printMoteQueues)

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
                
                if (self.dl%1000) == 0:
                    sys.stdout.write(".")
                    sys.stdout.flush()
                self.dl += 1

            #sys.stdout.write("dst: %d, src, %d, c: %d, rssi, %d, sloc: %d, sglob: %d, dstloc: %d, dstglob: %d, Sync: %d, reboot: %d, bat: %.2f, size: %d\n"%(m.get_dst(), m.get_src(), m.get_counter(), m.get_rssi(), m.get_srclocaltime(), m.get_srcglobaltime(), m.get_localtime(), m.get_globaltime(), m.get_isSynced(), m.get_reboot(), m.get_bat()/4096.0*5, m.get_size()))
            #sys.stdout.write("dst: %d, src, %d, size: %d\n"%(m.get_dst(), m.get_src(), m.get_size()))

            if m.get_dst() not in self.f.keys():
                sys.stdout.write(basedir+"/node_%d.log\n"%(m.get_dst()))
                self.f[m.get_dst()] = open(basedir+"/node_%d.log"%(m.get_dst()), "a+")
            self.f[m.get_dst()].write("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %.2f, %d\n"%(m.get_dst(), m.get_src(), m.get_counter(), m.get_rssi(), m.get_srclocaltime(), m.get_srcglobaltime(), m.get_localtime(), m.get_globaltime(), m.get_isSynced(), m.get_reboot(), m.get_bat()/4096.0*5, m.get_size()))
            self.f[m.get_dst()].flush()
            if m.get_size() == 0:
                # Start a new one now
                self.f[m.get_dst()].flush()
                self.f[m.get_dst()].close()
                del self.f[m.get_dst()]
                self.done_Mote(m.get_dst())
                
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

            # collect all client motes out there
            # print("exist", m.get_src(), self.motes.count(m.get_src()))
            if self.motes.count(m.get_src()) == 0 and m.get_buffersize() > 0:
                self.motes.append(m.get_src())
                                
        if msg.get_amType() == BaseStatusMsg.AM_TYPE:
            with self.lock:
                if not self.basemsgTimer.isAlive():
                    self.basemsgTimer.start()
                self.basemsgTimer.reset()

                m = BaseStatusMsg.BaseStatusMsg(msg.dataGet())
                self.basemsgs[m.get_src()] = m

            # collect all base stations out there
            if m.get_src() not in self.downloaders.keys():
                self.downloaders[m.get_src()] = 0


    def clearDownloading(self, nodeid):
        cleared = False
        for baseid, value in self.downloaders.iteritems(): # we can optimize this lookup
            if value == nodeid:
                self.downloaders[baseid] = 0
                cleared = True
                break
        return cleared


    def assign_Mote(self):
        if len(self.motes) > 0:
            return self.motes.pop()
        else:
            self.queryWREN()
            time.sleep(3)
            if len(self.motes) > 0:
                return self.motes.pop()
            else:
                # just end here
                return 0

    def monitor_Mote(self, baseid, nodeid):
        while True:
            if nodeid not in self.logSize.keys():
                print "node id", nodeid
                self.downloadTrials[nodeid] += 1
                
                if self.downloadTrials[nodeid] > 2: # greater than 3 time trials, give up for the mote
                    self.done_Mote(nodeid)
                    #self.downloaders[baseid] = 0
                    break
                else:
                    # try to download again
                    self.sendDownloadCmdToDownloader(baseid, nodeid)
                    time.sleep(2)
            else:
                # mote download started. Now check to see if the mote is still downloading....
                if self.moteLogSize[nodeid] != self.logSize[nodeid]:
                    self.downloadTrials[nodeid] = 0
                    self.moteLogSize[nodeid] = self.logSize[nodeid]
                    break
                else:
                    self.downloadTrials[nodeid] += 1
                    if self.downloadTrials[nodeid] > 2: # greater than 3 time trials, give up for the mote
                        #self.downloaders[baseid] = 0
                        self.done_Mote(nodeid)
                        break
                    else:
                        # try to download again
                        self.sendDownloadCmdToDownloader(baseid, nodeid)
                        time.sleep(2)

        #self.downloadTimer.reset()

#    def run_Mote(self):
#        for baseid in self.downloaders:
#            if self.downloaders[baseid] == 0:
#                nodeid = self.assign_Mote()
#                if nodeid > 0:
#                    self.downloadTrials[nodeid] = 0
#                    self.moteLogSize[nodeid] = 0
#
#                    self.downloaders[baseid] = nodeid
#                    self.sendDownloadCmdToController(baseid, nodeid) 
#                    time.sleep(1)
#            else:
#                self.monitor_Mote(baseid, self.downloaders[baseid])
#        #self.downloadTimer.reset()
        
    def run_Mote(self):
        for baseid, nodeid in self.downloaders.iteritems(): # we can optimize this lookup
            if nodeid == 0:
                nodeid = self.assign_Mote()
                if nodeid > 0:
                    self.downloadTrials[nodeid] = 0
                    self.moteLogSize[nodeid] = 0

                    self.downloaders[baseid] = nodeid
                    self.sendDownloadCmdToController(baseid, nodeid) 
                    time.sleep(1)
            else:
                self.monitor_Mote(baseid, self.downloaders[baseid])
                time.sleep(1)
        #self.downloadTimer.reset()
                
    def start(self):
        self.resetDownloadTimer()
        self.run_Mote()


    def done_Mote(self, nodeid):
        self.clearDownloading(nodeid)
        self.run_Mote()

    
    def check_Download(self):
        self.run_Mote()
        self.downloadTimer.reset()


    def sendDownloadCmdToController(self, channel, nodeid):
        print("send download command to Controller ..")
        #self.resetDownloadTimer()
        msg = CmdSerialMsg.CmdSerialMsg()
        msg.set_cmd(CMD_DOWNLOAD)
        msg.set_dst(channel)
        msg.set_channel(channel)
        
        print (nodeid, channel)
#        for n in self.m.get_nodes():
        #self.resetDownloadTimer()
#            self.mif[n.id].sendMsg(self.tos_source[n.id], nodeid, CmdSerialMsg.AM_TYPE, 0x22, msg)
        self.mif[0].sendMsg(self.tos_source[0], nodeid, CmdSerialMsg.AM_TYPE, 0x22, msg)


    def sendDownloadCmdToDownloader(self, baseid, nodeid):
        msg = CmdSerialMsg.CmdSerialMsg()
        msg.set_cmd(CMD_DOWNLOAD)
        #msg.set_dst(baseid)
        msg.set_dst(nodeid)
        msg.set_channel(baseid)
        
        print (nodeid, baseid)
        #self.resetDownloadTimer()
        #self.mif[0].sendMsg(self.tos_source[0], nodeid, CmdSerialMsg.AM_TYPE, 0x22, msg)
        self.mif[baseid].sendMsg(self.tos_source[baseid], baseid, CmdSerialMsg.AM_TYPE, 0x22, msg)
        #self.mif[baseid].sendMsg(self.tos_source[baseid], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)
        #self.mif[baseid].sendMsg(self.tos_source[baseid], nodeid, CmdSerialMsg.AM_TYPE, 0x22, msg)
        print("send command directly to base ..")


        #time.sleep(1)

    def stopDownloadMsg(self):
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


    def setDownloadBaseStationChannel(self):
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
            
    def split_list(alist, wanted_parts=1):
        length = len(alist)
        return [ alist[i*length // wanted_parts: (i+1)*length // wanted_parts] 
                 for i in range(wanted_parts) ]

    def printStatus(self):
        keys = self.msgs.keys()
        keys.sort()
        for id in keys:
            m = self.msgs[id]
            sys.stdout.write("id: %4d, sensing: %d, local: %d, global: %d, isSync: %d, reboot: %d, bat: %.2f, logsize: %d, isErased: %d, download: %d, channel: %d,\n"%(m.get_src(), m.get_sensing(), m.get_localtime(), m.get_globaltime(), m.get_isSynced(), m.get_reboots(), m.get_bat()/4096.0*5, m.get_buffersize(), m.get_isErased(), m.get_download(), m.get_channel()))
        
        sys.stdout.write("%d messages received!\n"%(len(self.msgs)))
        sys.stdout.flush()

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

    def printMoteQueues(self):
        print "***", len(self.motes), "client motes have data and queued for download"
        print "***", len(self.downloaders), "download base stations are ready for download"

    def printMoteQueueDetail(self):
        print "***", len(self.motes), "client motes have data and queued for download"
        print "client node id:"
        for elem in self.motes:
            print elem
        
        print "***", len(self.downloaders), "download base stations are ready for download"
        print "base station id:"
        for elem in self.downloaders:
            print elem


    def queryWREN(self):
        msg = CmdSerialMsg.CmdSerialMsg()
        msg.set_cmd(CMD_WREN_STATUS)
        msg.set_dst(102)
        for n in self.m.get_nodes():
            self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)

    def queryBaseStation(self):
        msg.set_cmd(CMD_BASESTATUS)
        msg.set_dst(102)
        for n in self.m.get_nodes():
            msg.set_channel(n.id)
            self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)
                    
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

    def main_loop(self):

        self.help()
        
        #self.queryWREN()
        
        while 1:
            c = raw_input()
            if c == 'p':
                self.printMoteQueueDetail()
            
            if c == 'q':
                self.stopDownloadMsg()
                for k in self.sfprocess.keys():
                    self.sfprocess[k].stop()
                    while self.sfprocess[k].is_dead():
                        time.sleep(1)
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
                self.queryWREN()
                self.setDownloadBaseStationChannel()
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
                self.stopDownloadMsg()
            elif c == 'g':
                # start sensing
                c = raw_input("Are you sure you want to start? [y/n]")
                if c != 'y':
                    print "Abort"
                    continue
                msg = CmdSerialMsg.CmdSerialMsg()
                msg.set_cmd(CMD_START_SENSE)
                #msg.set_dst(0xffff)
                for n in self.m.get_nodes():
                    self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)

                #self.stopBlink(0xffff)

            elif c == 'b':
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
#                if not self.downloadTimer.isAlive():
#                    self.downloadTimer.start()
#                self.downloadTimer.reset()

                # start download
                # stop all motes first
                self.queryWREN()
                self.setDownloadBaseStationChannel()

                time.sleep(3) #give sometime to settle down                
                self.stopSensing(0xffff)
#                # get the first group of motes for download and set channels for base stations
#                self.queryWREN()
#                # set base channels
#                self.setDownloadBaseStationChannel()
                # start download now
                
                self.start()
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
                    # start download
                    cs = c.strip().split(" ")
                    nodeid = int(cs[1])

                    # get the first group of motes for download and set channels for base stations
                    #self.queryWREN()
                    # set base channels
                    self.setDownloadBaseStationChannel()

                    time.sleep(3) #give sometime to settle down                
                    self.stopSensing(0xffff)

                    if self.motes.count(nodeid) == 0:
                        self.motes.append(nodeid)

                    self.start()
                    
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
                    # restore log
                    c = raw_input("Are you sure you want to restore log for node " + cs[1] + " ? [y/n]")
                    if c != 'y':
                        print "Abort restore"
                        continue
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

