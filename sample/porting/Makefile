CFLAGS:=-Wall -c -I../../include -I.

all: smitest

.SUFFIXES:
.SUFFIXES: .c.o

%.o : %.c
	gcc $(CFLAGS) -o $@ $<

smitest: smitest.o mailbox.o ../shader.o
	gcc -o $@ -Wall $^

mailbox.o: mailbox.c mailbox.h Makefile ../../include/soc/bcm2835/vcio2.h

smitest.o: smitest.c ../shader.h mailbox.h Makefile

../shader.o: ../shader.c ../shader.h

clean:
	rm *.o smitest
