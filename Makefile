# Makefile for drvtool

PGM=drvtool
OBJS=drvtool.o digest.o buffer.o
CFLAGS=-Wall -g -O
LIBS=-lcam -lmd -lz

all: $(PGM)

drvtool.o: drvtool.c drvtool.h digest.h
digest.o:  digest.c digest.h
buffer.o:  buffer.c buffer.h

$(PGM): $(OBJS)
	$(CC) -o $(PGM) $(OBJS) $(LIBS)

clean distclean:
	-rm -f $(PGM) *.o *~ \#* core *.core

pull:
	git pull

push: clean
	git add -A && git commit -a && git push
