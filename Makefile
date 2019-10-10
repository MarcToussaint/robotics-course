BASE = rai

target: build bin

DEPEND = Core Algo Geo Plot Kin Gui Optim KOMO LGP RosCom Operate Perception ry

exercise_paths =  $(shell find cpp -mindepth 1 -maxdepth 1 -type d -not -name 'retired' -printf "%f ")

build: $(DEPEND:%=inPath_makeLib/%)

exercises: $(exercise_paths:%=inPath_make/cpp/%)

bin:
	+make -C rai bin

installUbuntuAll: $(DEPEND:%=inPath_installUbuntu/%)

printUbuntuAll: $(DEPEND:%=inPath_printUbuntu/%) printUbuntu

clean: $(DEPEND:%=inPath_clean/%)

dependAll: cleanLocks cleanDepends $(DEPEND:%=inPath_depend/%)

include $(BASE)/build/generic.mk

.NOTPARALLEL:
