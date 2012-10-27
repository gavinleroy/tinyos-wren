import os
import sys
import time
import threading
import os.path

class Logger:
    
    def __init__(self, filename):
        print "logger init"
        self.filename = filename
        self.f = open(filename, "a+")

    def write(self, line):
        self.f.write(line)
        
    def writeLine(self, line):
        self.f.write(line + "\n")
        
    def flush(self):
        self.f.flush()
    
    def close(self):
        self.f.flush()
        self.f.close()
                        
    def readInputFile(self, filename):
        lines = open(filename, 'r').readlines()
        setting = []
        for line in lines:
            #print line
            line = line.strip()
            if line:
                setting.append(line)
        return setting        

    def output(self, filename):
        f = open(filename, "a+")
        for elem in self.deployingsensors:
            f.write(str(elem) + "\n")
        f.close()

    def getDeployableSensors(self, howmany):
        i = 0
        giveup = 0
        for sensorid in self.sensors:
            msg = CmdSerialMsg.CmdSerialMsg()
            msg.set_cmd(CMD_WREN_STATUS)
            msg.set_dst(102)
            giveup = 0
            for n in self.m.get_nodes():
                if self.deployingsensors.count(sensorid) == 0:
                    print "pinging ", sensorid
                    self.mif[n.id].sendMsg(self.tos_source[n.id], sensorid, CmdSerialMsg.AM_TYPE, 0x22, msg)
                    giveup += 1
                    time.sleep(2)
                    if giveup > 5:
                        print "not ready ", sensorid
                        break;
                else:
                    print "Is Ready ", sensorid
                    i += 1
                    break
                
            if i == howmany:
                break
