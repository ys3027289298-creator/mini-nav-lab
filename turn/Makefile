CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: turn

turn: src/turn.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: turn
	./turn

clean:
	rm -f turn
