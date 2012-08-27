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

import CmdSerialMsg
import RssiSerialMsg
import SerialStatusMsg

CMD_DOWNLOAD    = 0
CMD_ERASE       = 1
CMD_START_SENSE = 2
CMD_STOP_SENSE  = 3
CMD_STATUS      = 4
CMD_LOGSYNC     = 7

basedir = time.strftime("%m-%d-%Y", time.localtime())
if not os.path.exists(basedir):
    os.mkdir(basedir)



class WrenCmdCenter:

    mif = {}
    sfprocess = {}
    tos_source = {}

    msgs     = {}
    logSize  = {}
    lock = threading.RLock()

    f = {}

    def __init__(self):
        print "init"

        self.dl = 0

        self.m = mni.MNI()
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
            self.mif[n.id] = MoteIF.MoteIF()
            self.tos_source[n.id] = self.mif[n.id].addSource("sf@localhost:%d"%(20000+n.id))


        #add us as listener for the different messages
        for n in self.m.get_nodes():
            #self.mif[n.id].addListener(self, CmdSerialMsg.CmdSerialMsg)
            #self.mif[n.id].addListener(self, MoteStatusMsg.MoteStatusMsg)
            #self.mif[n.id].addListener(self, MoteRssiMsg.MoteRssiMsg)
            self.mif[n.id].addListener(self, RssiSerialMsg.RssiSerialMsg)
            self.mif[n.id].addListener(self, SerialStatusMsg.SerialStatusMsg)


    def receive(self, src, msg):
        #print time.time(), msg.addr
        #if msg.get_amType() == CmdSerialMsg.AM_TYPE:
        #    m = CmdSerialMsg.CmdSerialMsg(msg.dataGet())
        #    print m

        if msg.get_amType() == RssiSerialMsg.AM_TYPE:
            m = RssiSerialMsg.RssiSerialMsg(msg.dataGet())
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
                self.f[m.get_dst()] = open(basedir+"/node_%d.log"%(m.get_dst()), "a+")
            self.f[m.get_dst()].write("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %.2f, %d\n"%(m.get_dst(), m.get_src(), m.get_counter(), m.get_rssi(), m.get_srclocaltime(), m.get_srcglobaltime(), m.get_localtime(), m.get_globaltime(), m.get_isSynced(), m.get_reboot(), m.get_bat()/4096.0*5, m.get_size()))
            self.f[m.get_dst()].flush()
            if m.get_size() == 0:
                self.f[m.get_dst()].flush()
                self.f[m.get_dst()].close()
                del self.f[m.get_dst()]

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



    def printStatus(self):
        keys = self.msgs.keys()
        keys.sort()
        for id in keys:
            m = self.msgs[id]
            sys.stdout.write("id: %4d, sensing: %d, local: %d, global: %d, isSync: %d, reboot: %d, bat: %.2f, logsize: %d, isErased: %d, download: %d,\n"%(m.get_src(), m.get_sensing(), m.get_localtime(), m.get_globaltime(), m.get_isSynced(), m.get_reboots(), m.get_bat()/4096.0*5, m.get_buffersize(), m.get_isErased(), m.get_download()))

        self.msgs = {}

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
        print "Hit 'h' for help"

    def main_loop(self):

        self.help()

        while 1:
            c = raw_input()
            if c == 'q':
                for k in self.sfprocess.keys():
                    self.sfprocess[k].stop()
                    while self.sfprocess[k].is_dead():
                        time.sleep(1)
                break
            elif c == 's':
                # get status
                msg = CmdSerialMsg.CmdSerialMsg()
                msg.set_cmd(CMD_STATUS)
                #msg.set_nodeId()
                #msg.set_deploymentId()
                #msg.set_syncPeriod()
                for n in self.m.get_nodes():
                    self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)
            elif c == 'g':
                # start sensing
                c = raw_input("Are you sure you want to start? [y/n]")
                if c != 'y':
                    print "Abort"
                    continue
                msg = CmdSerialMsg.CmdSerialMsg()
                msg.set_cmd(CMD_START_SENSE)
                for n in self.m.get_nodes():
                    self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)
            elif c == 'b':
                # stop sensing
                msg = CmdSerialMsg.CmdSerialMsg()
                msg.set_cmd(CMD_STOP_SENSE)
                for n in self.m.get_nodes():
                    self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)
            elif c == 'e':
                c = raw_input("Are you sure you want to erase the data [y/n]")
                if c != 'y':
                    print "Abort erase"
                    continue
                msg = CmdSerialMsg.CmdSerialMsg()
                msg.set_cmd(CMD_ERASE)
                for n in self.m.get_nodes():
                    self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)
            elif c == 'd':
                # start download
                msg = CmdSerialMsg.CmdSerialMsg()
                msg.set_cmd(CMD_DOWNLOAD)
                for n in self.m.get_nodes():
                    self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)
            elif c == 'r':
                # restore log
                c = raw_input("Are you sure you want to restore log? [y/n]")
                if c != 'y':
                    print "Abort restore"
                    continue
                msg = CmdSerialMsg.CmdSerialMsg()
                msg.set_cmd(CMD_LOGSYNC)
                for n in self.m.get_nodes():
                    self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)
            elif len(c) > 1:
                if c[0] == 's' and c[1] == ' ':
                    cs = c.strip().split(" ")
                    nodeid = int(cs[1])
                    # get status
                    msg = CmdSerialMsg.CmdSerialMsg()
                    msg.set_cmd(CMD_STATUS)
                    for n in self.m.get_nodes():
                        if nodeid == n.id:
                            self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)
                elif c[0] == 'd':
                    # start download
                    cs = c.strip().split(" ")
                    nodeid = int(cs[1])
                    msg = CmdSerialMsg.CmdSerialMsg()
                    msg.set_cmd(CMD_DOWNLOAD)
                    for n in self.m.get_nodes():
                        if nodeid == n.id:
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
                    for n in self.m.get_nodes():
                        if nodeid == n.id:
                            self.mif[n.id].sendMsg(self.tos_source[n.id], 0xffff, CmdSerialMsg.AM_TYPE, 0x22, msg)

            elif c == 'h':
                self.help()


def main():

    cc = WrenCmdCenter()
    cc.main_loop()  # don't expect this to return...


if __name__ == "__main__":
    main()

