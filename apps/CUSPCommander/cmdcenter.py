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

import CmdSerialMsg
import RssiSerialMsg
import SerialStatusMsg
import BaseStatusMsg

CMD_DOWNLOAD    = 0
CMD_ERASE       = 1
CMD_START_SENSE = 2
CMD_STOP_SENSE  = 3
CMD_STATUS      = 4
CMD_LOGSYNC     = 7
CMD_START_BLINK = 5
CMD_BASESTATUS  = 8
CMD_NONE        = 9
CMD_CHANNEL     = 10

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
    basemotes = {}
    
    def __init__(self):
        print "init"

        self.dl = 0

        self.m = mni.MNI()
        self.basemsgTimer = ResettableTimer(2, self.printBaseStatus)
        self.msgTimer = ResettableTimer(2, self.printStatus)
        self.downloadTimer = ResettableTimer(1, self.checkDownload)

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
            print n.id
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


    def receive(self, src, msg):
        print time.time(), msg.addr
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

            sys.stdout.write("dst: %d, src, %d, c: %d, rssi, %d, sloc: %d, sglob: %d, dstloc: %d, dstglob: %d, Sync: %d, reboot: %d, bat: %.2f, size: %d\n"%(m.get_dst(), m.get_src(), m.get_counter(), m.get_rssi(), m.get_srclocaltime(), m.get_srcglobaltime(), m.get_localtime(), m.get_globaltime(), m.get_isSynced(), m.get_reboot(), m.get_bat()/4096.0*5, m.get_size()))
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
                self.startDownload(m.get_dst())
                
        if msg.get_amType() == SerialStatusMsg.AM_TYPE:
            with self.lock:
                if not self.msgTimer.isAlive():
                    self.msgTimer.start()
                self.msgTimer.reset()

                m = SerialStatusMsg.SerialStatusMsg(msg.dataGet())
                self.msgs[m.get_src()] = m

            if m.get_src() == 1:
                # this is the base mote. Store it's time for sync in file
                f = open(basedir+"/timesync.log", "a+")
                f.write("%.3f, %d\n"%(time.time(), m.get_globaltime()))
                f.close()
            
            # collect all client motes out there
            # print("exist", m.get_src(), self.motes.count(m.get_src()))
            if self.motes.count(m.get_src()) == 0:
                self.motes.append(m.get_src())
                
        if msg.get_amType() == BaseStatusMsg.AM_TYPE:
            with self.lock:
                if not self.basemsgTimer.isAlive():
                    self.basemsgTimer.start()
                self.basemsgTimer.reset()

                m = BaseStatusMsg.BaseStatusMsg(msg.dataGet())
                self.basemsgs[m.get_src()] = m

            if m.get_src() == 1:
                # this is the base mote. Store it's time for sync in file
                f = open(basedir+"/timesync.log", "a+")
                f.write("%.3f, %d\n"%(time.time(), m.get_globaltime()))
                f.close()

            # collect all base stations out there
            if m.get_src() not in self.basemotes.keys():
                self.basemotes[m.get_src()] = 0
            
    def startDownload(self, cur):
        print "downloading ..."
        
        # find the next dst for the base station
#        self.mif[0].sendMsg(self.tos_source[0], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)
        
        # find the base station
        # find the next dst (client id)
        # hand them to controller
        # The controller sends a download command to the client mote
        # and start downloading from the base station

        # clear the assignment once the download is finished for the current mote
        for key, value in self.basemotes.iteritems():
            if value == cur:
                self.basemotes[key] = 0

        # So, the controller needs to know which base station is free...
        if len(self.motes) == 0:  # check to see if the client mote list is empty
            print "download finished ..."
        else:
            print ("else")
            for item in self.basemotes:
                print("inside2")
                if len(self.motes) == 0:
                    break
                
                if self.basemotes[item] == 0:
                    print ("inside3")
                    self.basemotes[item] = self.motes.pop()
                    
                    msg = CmdSerialMsg.CmdSerialMsg()
                    msg.set_cmd(CMD_DOWNLOAD)
                    msg.set_dst(self.basemotes[item])
                    msg.set_channel(item)

                    print (self.basemotes[item], item)

                    # now send command to the controller
                    self.mif[0].sendMsg(self.tos_source[0], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)
            
#        print id, dst

#            self.mif[id].sendMsg(self.tos_source[id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)
            
#        for n in self.m.get_nodes():
#            self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)

    def checkDownload(self):
        logSize = 0
        for k in self.logSize.keys():
            if self.logSize[k] > logSize:
                logSize = self.logSize[k]

        if logSize > 0:
            msg = CmdSerialMsg.CmdSerialMsg()
            msg.set_cmd(CMD_DOWNLOAD)
            for n in self.m.get_nodes():
                self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)
            sys.stdout.write("**Restart Download**\n")
            sys.stdout.flush()
            self.downloadTimer.reset()
        else:
            # we are done. make sure everything is closed
            for k in self.f.keys():
                self.f[k].flush()
                self.f[k].close()
                del self.f[k]
            sys.stdout.write("Download Done\n")
            sys.stdout.flush()


    def split_list(alist, wanted_parts=1):
        length = len(alist)
        return [ alist[i*length // wanted_parts: (i+1)*length // wanted_parts] 
                 for i in range(wanted_parts) ]

    def printStatus(self):
        keys = self.msgs.keys()
        keys.sort()
        for id in keys:
            m = self.msgs[id]
            sys.stdout.write("id: %4d, sensing: %d, local: %d, global: %d, isSync: %d, reboot: %d, bat: %.2f, logsize: %d, isErased: %d, download: %d,\n"%(m.get_src(), m.get_sensing(), m.get_localtime(), m.get_globaltime(), m.get_isSynced(), m.get_reboots(), m.get_bat()/4096.0*5, m.get_buffersize(), m.get_isErased(), m.get_download()))

        self.msgs = {}

    def printBaseStatus(self):
        basekeys = self.basemsgs.keys()
        basekeys.sort()
        for id in basekeys:
            m = self.basemsgs[id]
            sys.stdout.write("id: %4d, local: %d, global: %d, isSync: %d, channel: %d\n"%(m.get_src(), m.get_localtime(), m.get_globaltime(), m.get_isSynced(), m.get_channel()))

        self.basemsgs = {}

    def printMoteQueues(self):
        print "printing client motes ..."
        for elem in self.motes:
            print elem
        
        print "printing base motes ..."
        
        for elem in self.basemotes:
            print elem
            
    def help(self):

        print "Hit 'q' to exit"
        print "Hit 's' to get status"
        print "Hit 's <nodeid>' to get status of one specific node"
        print "Hit 'g' to start sensing (go)"
        print "Hit 'b' to stop sensing  (break)"
        print "Hit 'd' to start downloading"
        print "Hit 'd <nodeid>' to start downloading a specific node"
        print "Hit 'e' to erase"
        print "Hit 'r' to restore log"
        print "Hit 'r <nodeid>' to restore log of one specific node"
        print "Hit 'f' to find all base stations"
        print "Hit 'h' for help"
        print "Hit 'c' for channel"

    def main_loop(self):

        self.help()

        while 1:
            c = raw_input()
            if c == 'p':
                self.printMoteQueues()
            
            if c == 'q':
                for k in self.sfprocess.keys():
                    self.sfprocess[k].stop()
                    while self.sfprocess[k].is_dead():
                        time.sleep(1)
                break
            elif c == 's':
                # get status
                self.f.clear()
                del self.motes[0:len(self.motes)]
                self.basemotes.clear()
                
                msg = CmdSerialMsg.CmdSerialMsg()
                msg.set_cmd(CMD_STATUS)
                msg.set_dst(102)
                #msg.set_nodeId()
                #msg.set_deploymentId()
                #msg.set_syncPeriod()
                for n in self.m.get_nodes():
                    self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)

                msg.set_cmd(CMD_BASESTATUS)
                msg.set_dst(102)
                #msg.set_nodeId()
                #msg.set_deploymentId()
                #msg.set_syncPeriod()
                for n in self.m.get_nodes():
                    msg.set_channel(n.id)
                    self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)
            
            elif c == 'f':
                self.basemotes.clear()

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
            elif c == 'c':
                # set base channels
                msg = CmdSerialMsg.CmdSerialMsg()
                msg.set_cmd(CMD_CHANNEL)
                msg.set_dst(102)
                #msg.set_dst(0xffff)
                #msg.set_nodeId()
                #msg.set_deploymentId()
                #msg.set_syncPeriod()
                for n in self.m.get_nodes():
                    msg.set_channel(n.id)
                    self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)
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
            elif c == 'b':
                # stop sensing
                msg = CmdSerialMsg.CmdSerialMsg()
                msg.set_cmd(CMD_STOP_SENSE)
                #msg.set_dst(0xffff)
                for n in self.m.get_nodes():
                    self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)
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
                # start download
                
                # split the clients for each base station
#                mote_list = split_list(self.motes, wanted_parts=len(self.basemotes))
#                for basemote in self.basemotes:
#                    assignment[basemote] = mote_list[index]
#                    index += 1
                    
                
                # start download now
                
                self.startDownload(0)
                
                
#                msg = CmdSerialMsg.CmdSerialMsg()
#                msg.set_cmd(CMD_DOWNLOAD)
#                msg.set_dst(102)
#                msg.set_channel(16)
#                for n in self.m.get_nodes():
#                    self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)
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
                        self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)
                elif c[0] == 'd':
                    # start download
                    cs = c.strip().split(" ")
                    nodeid = int(cs[1])
                    msg = CmdSerialMsg.CmdSerialMsg()
                    msg.set_cmd(CMD_DOWNLOAD)
                    msg.set_dst(nodeid)
                    for n in self.m.get_nodes():
                        self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)
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
                        self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)

            elif c == 'h':
                self.help()


def main():

    cc = CmdCenter()
    cc.main_loop()  # don't expect this to return...


if __name__ == "__main__":
    main()

