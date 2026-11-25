CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: grid

grid: src/grid.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: grid
	./grid

clean:
	rm -f grid
