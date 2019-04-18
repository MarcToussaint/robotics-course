BASE = rai

DEPEND = Core Algo Geo Plot Kin Gui Operate ry

build: $(DEPEND:%=inPath_makeLib/%)

installUbuntuAll: $(DEPEND:%=inPath_installUbuntu/%)

printUbuntuAll: $(DEPEND:%=inPath_printUbuntu/%) printUbuntu

clean: $(DEPEND:%=inPath_clean/%)

depend: $(DEPEND:%=inPath_depend/%)

include $(BASE)/build/generic.mk

.NOTPARALLEL:
