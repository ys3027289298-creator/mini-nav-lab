CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: cache

cache: src/cache.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: cache
	./cache

clean:
	rm -f cache
