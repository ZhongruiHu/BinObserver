#======================================================================== 
# Package	: Kalman Filter
# Authors	: Vilas kumar Chitrakaran  
# Start Date	: Wed Jan 01 11:06:55 GMT 2005
# ----------------------------------------------------------------------  
# File: makefile (makefile of examples)
#========================================================================  

LDFLAGS = g++ -Wall -fexceptions -O2 -o
INCLUDEHEADERS = -I ../ -I /usr/local/include/QMath -I /usr/local/include \
                 -I /usr/qrts/include
INCLUDELIB = -L ../ -L /usr/local/lib -L /usr/qrts/lib -lQMath 
TARGETS = BinObserver.t
targets:	$(TARGETS)
CLEAN = rm -rf $(TARGETS) *.dat *.o


# ----- BinObserver -----
BinObserver.t :	BinObserver.t.cpp
	$(LDFLAGS) $@ $? $(INCLUDEHEADERS) $(INCLUDELIB)


clean:
	$(CLEAN)

