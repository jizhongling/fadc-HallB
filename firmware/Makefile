# File:
#    Makefile
#
# Description:
#	Makefile for fADC250 firmware updating code (Linux ony)
#
#
DEBUG	?= 1
QUIET	?= 1
#
ifeq ($(QUIET),1)
        Q = @
else
        Q =
endif

#Check Operating system we are using
ifndef OSNAME
  OSNAME := $(subst -,_,$(shell uname))
endif

# Check for CODA 3 environment
ifdef CODA_VME

INC_CODA	= -I${CODA_VME}/include
LIB_CODA	= -L${CODA_VME_LIB}

endif

# Safe defaults
LINUXVME_LIB		?= ../../lib
LINUXVME_INC		?= ../../include
LINUXVME_BIN		?= ../../bin

CFLAGS			= -L. -L${LINUXVME_LIB} ${LIB_CODA} -DLinux_vme -DLinux -ljvme -lfadc
INCS			= -I. -I${LINUXVME_INC} ${INC_CODA}

ifdef DEBUG
CFLAGS	+= -Wall -g
else
CFLAGS	+= -O2
endif

SRC			= $(wildcard *.c)
DEPS			= $(SRC:.c=.d)
PROGS			= $(SRC:.c=)

all: $(PROGS)

%: %.c
	@echo " CC     $@"
	${Q}$(CC) $(CFLAGS) $(INCS) -o $@ $<

%.o: %.c
	@echo " CC     $@"
	${Q}$(CC) $(CFLAGS) $(INCS) -c -o $@ $<

install: $(PROGS)
	@echo " CP     ${PROGS}"
	${Q}cp ${PWD}/$(PROGS) $(LINUXVME_BIN)/

coda_install: $(PROGS)
	@echo " CODACP ${PROGS}"
	${Q}cp $(PROGS) $(CODA_VME_BIN)/

%.d: %.c
	@echo " DEP    $@"
	${Q}set -e; rm -f $@; \
	$(CC) -MM -shared  -DLinux_vme -DLinux $(INCS) $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$

-include $(DEPS)

clean:
	rm -f *~ $(PROGS)
