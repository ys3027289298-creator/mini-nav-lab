CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
ART ?= ../../artifacts

all: bench

bench: bench.c
	$(CC) $(CFLAGS) $< -o $@ -lm

# headline scenarios: repo LOS (no divergence), strict-LOS divergence,
# strict-LOS forward-degeneration limit
run: bench
	./bench $(ART) random plain
	./bench $(ART) random40 strict
	./bench $(ART) staircase strict
	./bench $(ART) stair2 strict 0 1

report:
	python3 report.py

clean:
	rm -f bench bench_asan

.PHONY: run report clean
