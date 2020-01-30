# Makefile for drvtool

PGM=drvtool
OBJS=drvtool.o
CFLAGS=-Wall -g -O
LIBS=-lcam -lmd -lz

all: $(PGM)

drvtool.o: drvtool.c drvtool.h

$(PGM): $(OBJS)
	$(CC) -o $(PGM) $(OBJS) $(LIBS)

clean distclean:
	-rm -f $(PGM) *.o *~ \#* core *.core

push: clean
	git add -A && git commit -a && git push
