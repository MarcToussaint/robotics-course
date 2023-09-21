SRCS = $(shell find . -maxdepth 1 -name '*.tex' -not -name 'rst-header.tex' -not -name 'z.tex')
OBJS = $(SRCS:%.tex=%.rst)

default: $(OBJS)

.NOTPARALLEL:

%.rst: %.tex force
	+-../make-rst.sh $<

force: ;
