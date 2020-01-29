# Makefile for drvtool

PGM=drvtool
OBJS=drvtool.o
CFLAGS=-Wall -g -O

all: $(PGM)

$(PGM): $(OBJS)
	$(CC) -o $(PGM) $(OBJS) -lmd

clean distclean:
	-rm -f $(PGM) *.o *~ \#* core *.core

push: clean
	git add -A && git commit -a && git push
