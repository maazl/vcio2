all : smitest

%.o : %.c
	gcc -Wall -c -o $@ $<

smitest : smitest.o mailbox.o
	gcc -o $@ -Wall $^

smitest.o : smitest.c mailbox.h smitest.hex

mailbox.c : mailbox.h

clean :
	rm *.o smitest
