CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: event

event: src/event.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: event
	./event

clean:
	rm -f event
