CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: preheat

preheat: src/preheat.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: preheat
	./preheat

clean:
	rm -f preheat
