all:
	gcc -g -O2 -Wall kincone.c -o kincone -lfreenect -lm

debug:
	gcc -g -O0 -Wall kincone.c -o kincone -lfreenect -lm

clean:
	rm -f kincone


