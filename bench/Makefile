CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: bench

bench: src/bench.c
	$(CC) $(CFLAGS) $< -o $@ -lm

# single sweep -> bench_raw.csv
run: bench
	./bench | tee bench_raw.csv

# robust sweep: 10 independent runs, then median-aggregate analysis
runs: bench
	for i in 1 2 3 4 5 6 7 8 9 10; do ./bench > bench_run_$$i.csv; done
	$(MAKE) analyze

analyze:
	python3 -P tools/analyze.py

# BFS oracle: optimality + identical search traces for all open lists
verify:
	$(CC) $(CFLAGS) -D_POSIX_C_SOURCE=200809L tests/verify.c -o /tmp/verify -lm
	/tmp/verify

clean:
	rm -f bench bench_raw.csv bench_run_*.csv
