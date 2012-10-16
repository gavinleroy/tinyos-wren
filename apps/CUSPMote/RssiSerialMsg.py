#
# This class is automatically generated by mig. DO NOT EDIT THIS FILE.
# This class implements a Python interface to the 'RssiSerialMsg'
# message type.
#

import tinyos.message.Message

# The default size of this message type in bytes.
DEFAULT_MESSAGE_SIZE = 31

# The Active Message type associated with this message.
AM_TYPE = 144

class RssiSerialMsg(tinyos.message.Message.Message):
    # Create a new RssiSerialMsg of size 31.
    def __init__(self, data="", addr=None, gid=None, base_offset=0, data_length=31):
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
        s = "Message <RssiSerialMsg> \n"
        try:
            s += "  [counter=0x%x]\n" % (self.get_counter())
        except:
            pass
        try:
            s += "  [dst=0x%x]\n" % (self.get_dst())
        except:
            pass
        try:
            s += "  [rssi=0x%x]\n" % (self.get_rssi())
        except:
            pass
        try:
            s += "  [src=0x%x]\n" % (self.get_src())
        except:
            pass
        try:
            s += "  [srclocaltime=0x%x]\n" % (self.get_srclocaltime())
        except:
            pass
        try:
            s += "  [srcglobaltime=0x%x]\n" % (self.get_srcglobaltime())
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
        try:
            s += "  [reboot=0x%x]\n" % (self.get_reboot())
        except:
            pass
        try:
            s += "  [bat=0x%x]\n" % (self.get_bat())
        except:
            pass
        try:
            s += "  [size=0x%x]\n" % (self.get_size())
        except:
            pass
        return s

    # Message-type-specific access methods appear below.

    #
    # Accessor methods for field: counter
    #   Field type: int
    #   Offset (bits): 0
    #   Size (bits): 16
    #

    #
    # Return whether the field 'counter' is signed (False).
    #
    def isSigned_counter(self):
        return False
    
    #
    # Return whether the field 'counter' is an array (False).
    #
    def isArray_counter(self):
        return False
    
    #
    # Return the offset (in bytes) of the field 'counter'
    #
    def offset_counter(self):
        return (0 / 8)
    
    #
    # Return the offset (in bits) of the field 'counter'
    #
    def offsetBits_counter(self):
        return 0
    
    #
    # Return the value (as a int) of the field 'counter'
    #
    def get_counter(self):
        return self.getUIntElement(self.offsetBits_counter(), 16, 1)
    
    #
    # Set the value of the field 'counter'
    #
    def set_counter(self, value):
        self.setUIntElement(self.offsetBits_counter(), 16, value, 1)
    
    #
    # Return the size, in bytes, of the field 'counter'
    #
    def size_counter(self):
        return (16 / 8)
    
    #
    # Return the size, in bits, of the field 'counter'
    #
    def sizeBits_counter(self):
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
    # Accessor methods for field: rssi
    #   Field type: short
    #   Offset (bits): 32
    #   Size (bits): 8
    #

    #
    # Return whether the field 'rssi' is signed (False).
    #
    def isSigned_rssi(self):
        return False
    
    #
    # Return whether the field 'rssi' is an array (False).
    #
    def isArray_rssi(self):
        return False
    
    #
    # Return the offset (in bytes) of the field 'rssi'
    #
    def offset_rssi(self):
        return (32 / 8)
    
    #
    # Return the offset (in bits) of the field 'rssi'
    #
    def offsetBits_rssi(self):
        return 32
    
    #
    # Return the value (as a short) of the field 'rssi'
    #
    def get_rssi(self):
        return self.getUIntElement(self.offsetBits_rssi(), 8, 1)
    
    #
    # Set the value of the field 'rssi'
    #
    def set_rssi(self, value):
        self.setUIntElement(self.offsetBits_rssi(), 8, value, 1)
    
    #
    # Return the size, in bytes, of the field 'rssi'
    #
    def size_rssi(self):
        return (8 / 8)
    
    #
    # Return the size, in bits, of the field 'rssi'
    #
    def sizeBits_rssi(self):
        return 8
    
    #
    # Accessor methods for field: src
    #   Field type: int
    #   Offset (bits): 40
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
        return (40 / 8)
    
    #
    # Return the offset (in bits) of the field 'src'
    #
    def offsetBits_src(self):
        return 40
    
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
    # Accessor methods for field: srclocaltime
    #   Field type: long
    #   Offset (bits): 56
    #   Size (bits): 32
    #

    #
    # Return whether the field 'srclocaltime' is signed (False).
    #
    def isSigned_srclocaltime(self):
        return False
    
    #
    # Return whether the field 'srclocaltime' is an array (False).
    #
    def isArray_srclocaltime(self):
        return False
    
    #
    # Return the offset (in bytes) of the field 'srclocaltime'
    #
    def offset_srclocaltime(self):
        return (56 / 8)
    
    #
    # Return the offset (in bits) of the field 'srclocaltime'
    #
    def offsetBits_srclocaltime(self):
        return 56
    
    #
    # Return the value (as a long) of the field 'srclocaltime'
    #
    def get_srclocaltime(self):
        return self.getUIntElement(self.offsetBits_srclocaltime(), 32, 1)
    
    #
    # Set the value of the field 'srclocaltime'
    #
    def set_srclocaltime(self, value):
        self.setUIntElement(self.offsetBits_srclocaltime(), 32, value, 1)
    
    #
    # Return the size, in bytes, of the field 'srclocaltime'
    #
    def size_srclocaltime(self):
        return (32 / 8)
    
    #
    # Return the size, in bits, of the field 'srclocaltime'
    #
    def sizeBits_srclocaltime(self):
        return 32
    
    #
    # Accessor methods for field: srcglobaltime
    #   Field type: long
    #   Offset (bits): 88
    #   Size (bits): 32
    #

    #
    # Return whether the field 'srcglobaltime' is signed (False).
    #
    def isSigned_srcglobaltime(self):
        return False
    
    #
    # Return whether the field 'srcglobaltime' is an array (False).
    #
    def isArray_srcglobaltime(self):
        return False
    
    #
    # Return the offset (in bytes) of the field 'srcglobaltime'
    #
    def offset_srcglobaltime(self):
        return (88 / 8)
    
    #
    # Return the offset (in bits) of the field 'srcglobaltime'
    #
    def offsetBits_srcglobaltime(self):
        return 88
    
    #
    # Return the value (as a long) of the field 'srcglobaltime'
    #
    def get_srcglobaltime(self):
        return self.getUIntElement(self.offsetBits_srcglobaltime(), 32, 1)
    
    #
    # Set the value of the field 'srcglobaltime'
    #
    def set_srcglobaltime(self, value):
        self.setUIntElement(self.offsetBits_srcglobaltime(), 32, value, 1)
    
    #
    # Return the size, in bytes, of the field 'srcglobaltime'
    #
    def size_srcglobaltime(self):
        return (32 / 8)
    
    #
    # Return the size, in bits, of the field 'srcglobaltime'
    #
    def sizeBits_srcglobaltime(self):
        return 32
    
    #
    # Accessor methods for field: localtime
    #   Field type: long
    #   Offset (bits): 120
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
        return (120 / 8)
    
    #
    # Return the offset (in bits) of the field 'localtime'
    #
    def offsetBits_localtime(self):
        return 120
    
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
    #   Offset (bits): 152
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
        return (152 / 8)
    
    #
    # Return the offset (in bits) of the field 'globaltime'
    #
    def offsetBits_globaltime(self):
        return 152
    
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
    #   Offset (bits): 184
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
        return (184 / 8)
    
    #
    # Return the offset (in bits) of the field 'isSynced'
    #
    def offsetBits_isSynced(self):
        return 184
    
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
    
    #
    # Accessor methods for field: reboot
    #   Field type: short
    #   Offset (bits): 192
    #   Size (bits): 8
    #

    #
    # Return whether the field 'reboot' is signed (False).
    #
    def isSigned_reboot(self):
        return False
    
    #
    # Return whether the field 'reboot' is an array (False).
    #
    def isArray_reboot(self):
        return False
    
    #
    # Return the offset (in bytes) of the field 'reboot'
    #
    def offset_reboot(self):
        return (192 / 8)
    
    #
    # Return the offset (in bits) of the field 'reboot'
    #
    def offsetBits_reboot(self):
        return 192
    
    #
    # Return the value (as a short) of the field 'reboot'
    #
    def get_reboot(self):
        return self.getUIntElement(self.offsetBits_reboot(), 8, 1)
    
    #
    # Set the value of the field 'reboot'
    #
    def set_reboot(self, value):
        self.setUIntElement(self.offsetBits_reboot(), 8, value, 1)
    
    #
    # Return the size, in bytes, of the field 'reboot'
    #
    def size_reboot(self):
        return (8 / 8)
    
    #
    # Return the size, in bits, of the field 'reboot'
    #
    def sizeBits_reboot(self):
        return 8
    
    #
    # Accessor methods for field: bat
    #   Field type: int
    #   Offset (bits): 200
    #   Size (bits): 16
    #

    #
    # Return whether the field 'bat' is signed (False).
    #
    def isSigned_bat(self):
        return False
    
    #
    # Return whether the field 'bat' is an array (False).
    #
    def isArray_bat(self):
        return False
    
    #
    # Return the offset (in bytes) of the field 'bat'
    #
    def offset_bat(self):
        return (200 / 8)
    
    #
    # Return the offset (in bits) of the field 'bat'
    #
    def offsetBits_bat(self):
        return 200
    
    #
    # Return the value (as a int) of the field 'bat'
    #
    def get_bat(self):
        return self.getUIntElement(self.offsetBits_bat(), 16, 1)
    
    #
    # Set the value of the field 'bat'
    #
    def set_bat(self, value):
        self.setUIntElement(self.offsetBits_bat(), 16, value, 1)
    
    #
    # Return the size, in bytes, of the field 'bat'
    #
    def size_bat(self):
        return (16 / 8)
    
    #
    # Return the size, in bits, of the field 'bat'
    #
    def sizeBits_bat(self):
        return 16
    
    #
    # Accessor methods for field: size
    #   Field type: long
    #   Offset (bits): 216
    #   Size (bits): 32
    #

    #
    # Return whether the field 'size' is signed (False).
    #
    def isSigned_size(self):
        return False
    
    #
    # Return whether the field 'size' is an array (False).
    #
    def isArray_size(self):
        return False
    
    #
    # Return the offset (in bytes) of the field 'size'
    #
    def offset_size(self):
        return (216 / 8)
    
    #
    # Return the offset (in bits) of the field 'size'
    #
    def offsetBits_size(self):
        return 216
    
    #
    # Return the value (as a long) of the field 'size'
    #
    def get_size(self):
        return self.getUIntElement(self.offsetBits_size(), 32, 1)
    
    #
    # Set the value of the field 'size'
    #
    def set_size(self, value):
        self.setUIntElement(self.offsetBits_size(), 32, value, 1)
    
    #
    # Return the size, in bytes, of the field 'size'
    #
    def size_size(self):
        return (32 / 8)
    
    #
    # Return the size, in bits, of the field 'size'
    #
    def sizeBits_size(self):
        return 32
    
