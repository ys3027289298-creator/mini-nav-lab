CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: consistency

consistency: src/consistency.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: consistency
	./consistency

clean:
	rm -f consistency
