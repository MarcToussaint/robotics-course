BASE = rai

DEPEND = Core Algo Geo Plot Kin Gui ry

build: $(DEPEND:%=inPath_makeLib/%)

initUbuntuPackages: $(DEPEND:%=inPath_installUbuntu/%)

printUbuntu: $(DEPEND:%=inPath_printUbuntuPackages/%) printUbuntuPackages

clean: $(DEPEND:%=inPath_clean/%)

depend: $(DEPEND:%=inPath_depend/%) $(test_paths:%=inPath_depend/%)

include $(BASE)/build/generic.mk

.NOTPARALLEL:
