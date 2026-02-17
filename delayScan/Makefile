#
# File:
#    Makefile
#
# Description:
#    Makefile for compilation of delay_scan_code program
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

CROSS_COMPILE		=
CC			= $(CROSS_COMPILE)gcc
AR                      = ar
RANLIB                  = ranlib
INCS			= -I. -I${LINUXVME_INC}
CFLAGS			= -L. -L${LINUXVME_LIB} -lrt -ljvme -lfadc
ifeq ($(DEBUG),1)
	CFLAGS		+= -Wall -g
endif
SRC			= $(wildcard *.c)
DEPS			= $(SRC:.c=.d)
PROGS			= $(SRC:.c=)

all: $(PROGS)

clean distclean:
	@rm -f $(PROGS) *~ $(OBJS) $(DEPS)

%: %.c
	@echo " CC     $@"
	${Q}$(CC) $(CFLAGS) $(INCS) \
		-o $@ $<

%.d: %.c
	@echo " DEP    $@"
	@set -e; rm -f $@; \
	$(CC) -MM -shared $(INCS) $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1 $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$

-include $(DEPS)

.PHONY: all clean distclean
