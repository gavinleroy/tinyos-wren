COMPONENT=NullWrenAppC
CFLAGS += -I$(TOSDIR)/lib/printf -I$(TOSDIR)/../apps/tests/wiesel/RadioReflector
CFLAGS += -DREFLECTOR_ID=666
CFLAGS += -DMAX_REFLECTIONS=5
# Flash size junk to be read/written
CFLAGS += -DFLASH_SZE=1024L*10

CFLAGS += -DCC2420_DEF_CHANNEL=11
CFLAGS += -DRF233_DEF_CHANNEL=11

include $(MAKERULES)

