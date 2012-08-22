#!/usr/bin/env python
from mni import mni
import sys
import optparse
import ConfigParser
import subprocess
import time

config = ConfigParser.RawConfigParser()
config.readfp(open("config.ini"))


m = mni.MNI(addIgnore=True)

currentid = -1

while 1:
    proc = subprocess.Popen("motelist -c", shell=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    proc.wait()

    s = proc.stdout.readlines()


    for l in s:
        ls = l.strip().split(',')
        serialid = ls[0]

        found = False
        for n in m.get_nodes():
            if n.serialid == serialid:
                found = True
                if n.id != currentid:
                    currentid = n.id
                    sys.stdout.write("\n\r%d: %s\r\n"%(n.id, n.serialid))
                    sys.stdout.flush()
        if len(ls) > 1 and not found:
            if currentid != -1:
                sys.stdout.write("\n\rUnknown Node: %s\r\n"%(serialid,))
                currentid = -1

    time.sleep(0.1)




