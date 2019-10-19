all : smitest

%.o : %.c
	gcc -Wall -c -o $@ -I../include $<

smitest : smitest.o mailbox.o shader.o
	gcc -o $@ -Wall $^

smitest.o : smitest.c mailbox.h shader.h

mailbox.c : mailbox.h

shader.o : shader.c shader.h

clean :
	rm *.o smitest
