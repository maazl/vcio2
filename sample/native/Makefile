CFLAGS:=-Wall -c -I../../include

all: smitest

.SUFFIXES:
.SUFFIXES: .c.o

%.o : %.c
	gcc $(CFLAGS) -o $@ $<

smitest: smitest.o ../shader.o
	gcc -o $@ -Wall $^

smitest.o: smitest.c ../shader.h ../../include/soc/bcm2835/vcio2.h Makefile

../shader.o: ../shader.c ../shader.h

clean:
	rm *.o smitest
