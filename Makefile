# Makefile for drvtool

DEST=/usr/local
BINDEST=$(DEST)/bin

PGM=drvtool
OBJS=drvtool.o buffer.o rate.o blocks.o digest.o transform.o seq.o strval.o
CFLAGS=-Wall -g -O -I/usr/local/include
LIBS=-L/usr/local/lib -lreadline -lcam -lmd -lz

all: $(PGM) tests/seq-test

drvtool.o:   drvtool.c drvtool.h digest.h rate.h blocks.h strval.h
buffer.o:    buffer.c buffer.h
rate.o:      rate.c rate.h
blocks.o:    blocks.c blocks.h
digest.o:    digest.c digest.h
transform.o: transform.c transform.h
seq.o:       seq.c seq.h
strval.o:    strval.c strval.h

$(PGM): $(OBJS)
	$(CC) -o $(PGM) $(OBJS) $(LIBS)

clean distclean:
	-rm -f $(PGM) *.o *~ \#* core *.core
	-(cd tests && rm -f seq-test *.o *~ \#* core *.core)

pull:
	git pull

push: clean
	git add -A && git commit -a && git push

install: $(PGM)
	install -m 755 $(PGM) $(BINDEST)

tests/seq-test:	tests/seq-test.o seq.o strval.o
	$(CC) -DDEBUG=1 -g -Wall -o tests/seq-test tests/seq-test.o seq.o strval.o
