#
# This class is automatically generated by mig. DO NOT EDIT THIS FILE.
# This class implements a Python interface to the 'WRENConnectionMsg'
# message type.
#

import tinyos.message.Message

# The default size of this message type in bytes.
DEFAULT_MESSAGE_SIZE = 13

# The Active Message type associated with this message.
AM_TYPE = 80

class WRENConnectionMsg(tinyos.message.Message.Message):
    # Create a new WRENConnectionMsg of size 13.
    def __init__(self, data="", addr=None, gid=None, base_offset=0, data_length=13):
        tinyos.message.Message.Message.__init__(self, data, addr, gid, base_offset, data_length)
        self.amTypeSet(AM_TYPE)
    
    # Get AM_TYPE
    def get_amType(cls):
        return AM_TYPE
    
    get_amType = classmethod(get_amType)
    
    #
    # Return a String representation of this message. Includes the
    # message type name and the non-indexed field values.
    #
    def __str__(self):
        s = "Message <WRENConnectionMsg> \n"
        try:
            s += "  [src=0x%x]\n" % (self.get_src())
        except:
            pass
        try:
            s += "  [dst=0x%x]\n" % (self.get_dst())
        except:
            pass
        try:
            s += "  [cmd=0x%x]\n" % (self.get_cmd())
        except:
            pass
        try:
            s += "  [logsize=0x%x]\n" % (self.get_logsize())
        except:
            pass
        try:
            s += "  [channel=0x%x]\n" % (self.get_channel())
        except:
            pass
        try:
            s += "  [isAcked=0x%x]\n" % (self.get_isAcked())
        except:
            pass
        try:
            s += "  [action=0x%x]\n" % (self.get_action())
        except:
            pass
        return s

    # Message-type-specific access methods appear below.

    #
    # Accessor methods for field: src
    #   Field type: int
    #   Offset (bits): 0
    #   Size (bits): 16
    #

    #
    # Return whether the field 'src' is signed (False).
    #
    def isSigned_src(self):
        return False
    
    #
    # Return whether the field 'src' is an array (False).
    #
    def isArray_src(self):
        return False
    
    #
    # Return the offset (in bytes) of the field 'src'
    #
    def offset_src(self):
        return (0 / 8)
    
    #
    # Return the offset (in bits) of the field 'src'
    #
    def offsetBits_src(self):
        return 0
    
    #
    # Return the value (as a int) of the field 'src'
    #
    def get_src(self):
        return self.getUIntElement(self.offsetBits_src(), 16, 1)
    
    #
    # Set the value of the field 'src'
    #
    def set_src(self, value):
        self.setUIntElement(self.offsetBits_src(), 16, value, 1)
    
    #
    # Return the size, in bytes, of the field 'src'
    #
    def size_src(self):
        return (16 / 8)
    
    #
    # Return the size, in bits, of the field 'src'
    #
    def sizeBits_src(self):
        return 16
    
    #
    # Accessor methods for field: dst
    #   Field type: int
    #   Offset (bits): 16
    #   Size (bits): 16
    #

    #
    # Return whether the field 'dst' is signed (False).
    #
    def isSigned_dst(self):
        return False
    
    #
    # Return whether the field 'dst' is an array (False).
    #
    def isArray_dst(self):
        return False
    
    #
    # Return the offset (in bytes) of the field 'dst'
    #
    def offset_dst(self):
        return (16 / 8)
    
    #
    # Return the offset (in bits) of the field 'dst'
    #
    def offsetBits_dst(self):
        return 16
    
    #
    # Return the value (as a int) of the field 'dst'
    #
    def get_dst(self):
        return self.getUIntElement(self.offsetBits_dst(), 16, 1)
    
    #
    # Set the value of the field 'dst'
    #
    def set_dst(self, value):
        self.setUIntElement(self.offsetBits_dst(), 16, value, 1)
    
    #
    # Return the size, in bytes, of the field 'dst'
    #
    def size_dst(self):
        return (16 / 8)
    
    #
    # Return the size, in bits, of the field 'dst'
    #
    def sizeBits_dst(self):
        return 16
    
    #
    # Accessor methods for field: cmd
    #   Field type: int
    #   Offset (bits): 32
    #   Size (bits): 16
    #

    #
    # Return whether the field 'cmd' is signed (False).
    #
    def isSigned_cmd(self):
        return False
    
    #
    # Return whether the field 'cmd' is an array (False).
    #
    def isArray_cmd(self):
        return False
    
    #
    # Return the offset (in bytes) of the field 'cmd'
    #
    def offset_cmd(self):
        return (32 / 8)
    
    #
    # Return the offset (in bits) of the field 'cmd'
    #
    def offsetBits_cmd(self):
        return 32
    
    #
    # Return the value (as a int) of the field 'cmd'
    #
    def get_cmd(self):
        return self.getUIntElement(self.offsetBits_cmd(), 16, 1)
    
    #
    # Set the value of the field 'cmd'
    #
    def set_cmd(self, value):
        self.setUIntElement(self.offsetBits_cmd(), 16, value, 1)
    
    #
    # Return the size, in bytes, of the field 'cmd'
    #
    def size_cmd(self):
        return (16 / 8)
    
    #
    # Return the size, in bits, of the field 'cmd'
    #
    def sizeBits_cmd(self):
        return 16
    
    #
    # Accessor methods for field: logsize
    #   Field type: long
    #   Offset (bits): 48
    #   Size (bits): 32
    #

    #
    # Return whether the field 'logsize' is signed (False).
    #
    def isSigned_logsize(self):
        return False
    
    #
    # Return whether the field 'logsize' is an array (False).
    #
    def isArray_logsize(self):
        return False
    
    #
    # Return the offset (in bytes) of the field 'logsize'
    #
    def offset_logsize(self):
        return (48 / 8)
    
    #
    # Return the offset (in bits) of the field 'logsize'
    #
    def offsetBits_logsize(self):
        return 48
    
    #
    # Return the value (as a long) of the field 'logsize'
    #
    def get_logsize(self):
        return self.getUIntElement(self.offsetBits_logsize(), 32, 1)
    
    #
    # Set the value of the field 'logsize'
    #
    def set_logsize(self, value):
        self.setUIntElement(self.offsetBits_logsize(), 32, value, 1)
    
    #
    # Return the size, in bytes, of the field 'logsize'
    #
    def size_logsize(self):
        return (32 / 8)
    
    #
    # Return the size, in bits, of the field 'logsize'
    #
    def sizeBits_logsize(self):
        return 32
    
    #
    # Accessor methods for field: channel
    #   Field type: short
    #   Offset (bits): 80
    #   Size (bits): 8
    #

    #
    # Return whether the field 'channel' is signed (False).
    #
    def isSigned_channel(self):
        return False
    
    #
    # Return whether the field 'channel' is an array (False).
    #
    def isArray_channel(self):
        return False
    
    #
    # Return the offset (in bytes) of the field 'channel'
    #
    def offset_channel(self):
        return (80 / 8)
    
    #
    # Return the offset (in bits) of the field 'channel'
    #
    def offsetBits_channel(self):
        return 80
    
    #
    # Return the value (as a short) of the field 'channel'
    #
    def get_channel(self):
        return self.getUIntElement(self.offsetBits_channel(), 8, 1)
    
    #
    # Set the value of the field 'channel'
    #
    def set_channel(self, value):
        self.setUIntElement(self.offsetBits_channel(), 8, value, 1)
    
    #
    # Return the size, in bytes, of the field 'channel'
    #
    def size_channel(self):
        return (8 / 8)
    
    #
    # Return the size, in bits, of the field 'channel'
    #
    def sizeBits_channel(self):
        return 8
    
    #
    # Accessor methods for field: isAcked
    #   Field type: short
    #   Offset (bits): 88
    #   Size (bits): 8
    #

    #
    # Return whether the field 'isAcked' is signed (False).
    #
    def isSigned_isAcked(self):
        return False
    
    #
    # Return whether the field 'isAcked' is an array (False).
    #
    def isArray_isAcked(self):
        return False
    
    #
    # Return the offset (in bytes) of the field 'isAcked'
    #
    def offset_isAcked(self):
        return (88 / 8)
    
    #
    # Return the offset (in bits) of the field 'isAcked'
    #
    def offsetBits_isAcked(self):
        return 88
    
    #
    # Return the value (as a short) of the field 'isAcked'
    #
    def get_isAcked(self):
        return self.getUIntElement(self.offsetBits_isAcked(), 8, 1)
    
    #
    # Set the value of the field 'isAcked'
    #
    def set_isAcked(self, value):
        self.setUIntElement(self.offsetBits_isAcked(), 8, value, 1)
    
    #
    # Return the size, in bytes, of the field 'isAcked'
    #
    def size_isAcked(self):
        return (8 / 8)
    
    #
    # Return the size, in bits, of the field 'isAcked'
    #
    def sizeBits_isAcked(self):
        return 8
    
    #
    # Accessor methods for field: action
    #   Field type: short
    #   Offset (bits): 96
    #   Size (bits): 8
    #

    #
    # Return whether the field 'action' is signed (False).
    #
    def isSigned_action(self):
        return False
    
    #
    # Return whether the field 'action' is an array (False).
    #
    def isArray_action(self):
        return False
    
    #
    # Return the offset (in bytes) of the field 'action'
    #
    def offset_action(self):
        return (96 / 8)
    
    #
    # Return the offset (in bits) of the field 'action'
    #
    def offsetBits_action(self):
        return 96
    
    #
    # Return the value (as a short) of the field 'action'
    #
    def get_action(self):
        return self.getUIntElement(self.offsetBits_action(), 8, 1)
    
    #
    # Set the value of the field 'action'
    #
    def set_action(self, value):
        self.setUIntElement(self.offsetBits_action(), 8, value, 1)
    
    #
    # Return the size, in bytes, of the field 'action'
    #
    def size_action(self):
        return (8 / 8)
    
    #
    # Return the size, in bits, of the field 'action'
    #
    def sizeBits_action(self):
        return 8
    
