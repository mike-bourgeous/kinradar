all:
	gcc -g -O2 -Wall kinradar.c -o kinradar -lfreenect -lm

debug:
	gcc -g -O0 -Wall kinradar.c -o kinradar -lfreenect -lm

clean:
	rm -f kinradar


