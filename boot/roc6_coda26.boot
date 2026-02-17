# Boot file for CODA ROC 2.6
# PowerPC version

#loginUserAdd "abbottd","yzzbdbccd"

# Add route to outside world (from 29 subnet to 120 subnet)
mRouteAdd("129.57.120.0","129.57.29.1",0xfffffc00,0,0)

# Load host table
< /daqfs/home/abbottd/VXKERN/vxhosts.boot

# Setup environment to load coda_roc
putenv "MSQL_TCP_HOST=dafarm29"
putenv "EXPID=experiment_0"
putenv "TCL_LIBRARY=/daqfs/coda/2.6/common/lib/tcl7.4"
putenv "ITCL_LIBRARY=/daqfs/coda/2.6/common/lib/itcl2.0"
putenv "DP_LIBRARY=/daqfs/coda/2.6/common/lib/dp"
putenv "SESSION=davetest"


# Load Tempe DMA Library (for MV6100)
cd "/daqfs/mizar/home/abbottd/vxWorks/tempeDma"
ld < usrTempeDma.o
usrVmeDmaConfig(2,2)

# Load FADC Library
cd "/daqfs/mizar/home/abbottd/vxWorks/fadc/v2.0"
ld<fadcLib.o
faInit(0xed0000,0,1,0)


# Load cMsg Stuff
cd "/daqfs/coda/3.0b/vxworks-ppc"
ld< lib/libcmsgRegex.so
ld< lib/libcmsg.so


cd "/daqfs/coda/2.6/VXWORKSPPC55/bin"
ld < coda_roc_rc3


# Spawn tasks
taskSpawn ("ROC",200,8,250000,coda_roc,"","-s","davetest","-objects","ROC6 ROC")





