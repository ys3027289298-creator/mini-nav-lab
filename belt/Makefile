CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: belt

belt: src/belt.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: belt
	./belt

csv: belt
	./belt csv

clean:
	rm -f belt
