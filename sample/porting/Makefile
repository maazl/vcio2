CFLAGS:=-Wall -c -I../../include -I.

all: smitest

.SUFFIXES:
.SUFFIXES: .c.o

%.o : %.c
	gcc $(CFLAGS) -o $@ $<

smitest: smitest.o mailbox.o ../shader.o
	gcc -o $@ -Wall $^

mailbox.c: mailbox.h Makefile

smitest.o: smitest.c ../shader.h mailbox.h Makefile

../shader.o: ../shader.c ../shader.h

clean:
	rm *.o smitest
