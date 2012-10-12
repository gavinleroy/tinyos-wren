from mni import mni

import os
import sys
import time
import optparse

import random
import matplotlib.pyplot as plt
import networkx as nx

from mni import managedsubproc as msp

from tinyos.message import *
from tinyos.message.Message import *
from tinyos.message.SerialPacket import *
from tinyos.packet.Serial import Serial

import CmdSerialMsg
import RssiSerialMsg
import SerialStatusMsg

class CmdCenter:

    mif = {}
    sfprocess = {}
    tos_source = {}

    def __init__(self):
        print "init"

        self.m = mni.MNI(addIgnore=True)

        # connecting serial forwarder to basestation
        baseFileName = "/tmp/sf"
        p = msp.ManagedSubproc("/usr/bin/java net.tinyos.sf.SerialForwarder -no-gui -port %d -comm serial@%s:tmote"%(20000, sys.argv[1]),
                stdout_disk = baseFileName + ".%d.log"%(1),
                stderr_disk = baseFileName + ".%d.stderr.log"%(1),
                stdout_fns = [ ])
        p.start()
        self.sfprocess = p

        # give it some time to establish all the serial forwarders
        time.sleep(3)

        self.mif = MoteIF.MoteIF()
        self.tos_source = self.mif.addSource("sf@localhost:%d"%(20000))

        #add us as listener for the different messages
        self.mif.addListener(self, RssiSerialMsg.RssiSerialMsg)



    def receive(self, src, msg):
        if msg.get_amType() == RssiSerialMsg.AM_TYPE:
            m = RssiSerialMsg.RssiSerialMsg(msg.dataGet())
            sys.stdout.write("dst: %d, src, %d, c: %d, rssi, %d\n"%(
                m.get_dst(),
                m.get_src(),
                m.get_counter(),
                m.get_rssi()))
            self.G.add_edge(m.get_dst(),m.get_src(), weight=-1.0/m.get_rssi())
            #self.G.add_edge(m.get_dst(),m.get_src())

    def main_loop(self):

        #activeNode = [201, 170, 190, 145, 298]
        plt.show()
        self.G = nx.Graph()
        for n in self.m.nodes:
            #if n.id in activeNode:
                self.G.add_node(n.id)
        print self.G.nodes()
        self.pos = nx.circular_layout(self.G)
        #nx.draw(self.G, self.pos)
        plt.axis('off')
        #plt.draw()

        while 1:
            print "Redraw"

            springpos=nx.spring_layout(self.G, pos=self.pos)
            nx.draw_networkx_nodes(self.G,springpos,
                    node_color='b',
                    node_size=3000,
                    alpha=0.8)
            plt.hold(True)
            nx.draw_networkx_edges(self.G,springpos,
                    width=2,alpha=0.5,edge_color='r')
            nx.draw_networkx_labels(self.G, springpos,
                    font_color='w',
                    font_size=20)
            #nx.draw(self.G, springpos)
            #nx.draw_networkx_edge_labels(self.G, springpos)
            plt.hold(False)
            #nx.draw(self.G, springpos)
            plt.axis('off')
            plt.draw()

            # remove all edges
            self.G.remove_edges_from(self.G.edges())
            time.sleep(5)

def main():

    plt.figure(figsize=(18,10))
    plt.axis('off')
    plt.ion()
    plt.draw()
    plt.show()
    plt.hold(False)

    if len(sys.argv) != 2:
        print "Usage: %s <serialport>"%(sys.argv[0])
        sys.exit(1)


    cc = CmdCenter()
    cc.main_loop()  # don't expect this to return...


if __name__ == "__main__":
    main()

