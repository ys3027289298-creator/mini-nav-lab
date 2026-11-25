CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: concurrent

concurrent: src/concurrent.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: concurrent
	./concurrent

clean:
	rm -f concurrent
