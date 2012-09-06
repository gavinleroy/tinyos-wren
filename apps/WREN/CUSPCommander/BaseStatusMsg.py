#
# This class is automatically generated by mig. DO NOT EDIT THIS FILE.
# This class implements a Python interface to the 'BaseStatusMsg'
# message type.
#

import tinyos.message.Message

# The default size of this message type in bytes.
DEFAULT_MESSAGE_SIZE = 11

# The Active Message type associated with this message.
AM_TYPE = 8

class BaseStatusMsg(tinyos.message.Message.Message):
    # Create a new BaseStatusMsg of size 11.
    def __init__(self, data="", addr=None, gid=None, base_offset=0, data_length=11):
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
        s = "Message <BaseStatusMsg> \n"
        try:
            s += "  [src=0x%x]\n" % (self.get_src())
        except:
            pass
        try:
            s += "  [localtime=0x%x]\n" % (self.get_localtime())
        except:
            pass
        try:
            s += "  [globaltime=0x%x]\n" % (self.get_globaltime())
        except:
            pass
        try:
            s += "  [isSynced=0x%x]\n" % (self.get_isSynced())
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
    # Accessor methods for field: localtime
    #   Field type: long
    #   Offset (bits): 16
    #   Size (bits): 32
    #

    #
    # Return whether the field 'localtime' is signed (False).
    #
    def isSigned_localtime(self):
        return False
    
    #
    # Return whether the field 'localtime' is an array (False).
    #
    def isArray_localtime(self):
        return False
    
    #
    # Return the offset (in bytes) of the field 'localtime'
    #
    def offset_localtime(self):
        return (16 / 8)
    
    #
    # Return the offset (in bits) of the field 'localtime'
    #
    def offsetBits_localtime(self):
        return 16
    
    #
    # Return the value (as a long) of the field 'localtime'
    #
    def get_localtime(self):
        return self.getUIntElement(self.offsetBits_localtime(), 32, 1)
    
    #
    # Set the value of the field 'localtime'
    #
    def set_localtime(self, value):
        self.setUIntElement(self.offsetBits_localtime(), 32, value, 1)
    
    #
    # Return the size, in bytes, of the field 'localtime'
    #
    def size_localtime(self):
        return (32 / 8)
    
    #
    # Return the size, in bits, of the field 'localtime'
    #
    def sizeBits_localtime(self):
        return 32
    
    #
    # Accessor methods for field: globaltime
    #   Field type: long
    #   Offset (bits): 48
    #   Size (bits): 32
    #

    #
    # Return whether the field 'globaltime' is signed (False).
    #
    def isSigned_globaltime(self):
        return False
    
    #
    # Return whether the field 'globaltime' is an array (False).
    #
    def isArray_globaltime(self):
        return False
    
    #
    # Return the offset (in bytes) of the field 'globaltime'
    #
    def offset_globaltime(self):
        return (48 / 8)
    
    #
    # Return the offset (in bits) of the field 'globaltime'
    #
    def offsetBits_globaltime(self):
        return 48
    
    #
    # Return the value (as a long) of the field 'globaltime'
    #
    def get_globaltime(self):
        return self.getUIntElement(self.offsetBits_globaltime(), 32, 1)
    
    #
    # Set the value of the field 'globaltime'
    #
    def set_globaltime(self, value):
        self.setUIntElement(self.offsetBits_globaltime(), 32, value, 1)
    
    #
    # Return the size, in bytes, of the field 'globaltime'
    #
    def size_globaltime(self):
        return (32 / 8)
    
    #
    # Return the size, in bits, of the field 'globaltime'
    #
    def sizeBits_globaltime(self):
        return 32
    
    #
    # Accessor methods for field: isSynced
    #   Field type: short
    #   Offset (bits): 80
    #   Size (bits): 8
    #

    #
    # Return whether the field 'isSynced' is signed (False).
    #
    def isSigned_isSynced(self):
        return False
    
    #
    # Return whether the field 'isSynced' is an array (False).
    #
    def isArray_isSynced(self):
        return False
    
    #
    # Return the offset (in bytes) of the field 'isSynced'
    #
    def offset_isSynced(self):
        return (80 / 8)
    
    #
    # Return the offset (in bits) of the field 'isSynced'
    #
    def offsetBits_isSynced(self):
        return 80
    
    #
    # Return the value (as a short) of the field 'isSynced'
    #
    def get_isSynced(self):
        return self.getUIntElement(self.offsetBits_isSynced(), 8, 1)
    
    #
    # Set the value of the field 'isSynced'
    #
    def set_isSynced(self, value):
        self.setUIntElement(self.offsetBits_isSynced(), 8, value, 1)
    
    #
    # Return the size, in bytes, of the field 'isSynced'
    #
    def size_isSynced(self):
        return (8 / 8)
    
    #
    # Return the size, in bits, of the field 'isSynced'
    #
    def sizeBits_isSynced(self):
        return 8
    
