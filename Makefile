# Makefile for drvtool

DEST=/usr/local
BINDEST=$(DEST)/bin

PGM=drvtool
OBJS=drvtool.o digest.o buffer.o rate.o blocks.o
CFLAGS=-Wall -g -O
LIBS=-lcam -lmd -lz

all: $(PGM)

drvtool.o: drvtool.c drvtool.h digest.h rate.h blocks.h
digest.o:  digest.c digest.h
buffer.o:  buffer.c buffer.h
rate.o:    rate.c rate.h
blocks.o:  blocks.c blocks.h

$(PGM): $(OBJS)
	$(CC) -o $(PGM) $(OBJS) $(LIBS)

clean distclean:
	-rm -f $(PGM) *.o *~ \#* core *.core

pull:
	git pull

push: clean
	git add -A && git commit -a && git push

install: $(PGM)
	install -m 755 $(PGM) $(BINDEST)
