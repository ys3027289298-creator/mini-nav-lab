CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: grid

grid: src/grid.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: grid
	./grid

fuzz:
	python3 fuzz/run_experiment.py

clean:
	rm -f grid

.PHONY: all test fuzz clean
