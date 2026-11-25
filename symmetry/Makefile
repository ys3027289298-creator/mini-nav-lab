CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: symmetry

symmetry: src/symmetry.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: symmetry
	./symmetry

clean:
	rm -f symmetry

.PHONY: all test clean
