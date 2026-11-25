CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: bench demo

bench: src/bench.c
	$(CC) $(CFLAGS) $< -o $@ -lm

demo: src/demo.c
	$(CC) $(CFLAGS) $< -o $@ -lm

# calibration tables consumed by scripts/preheat_sim.py
measure: bench
	./bench open  > data/bench_open.csv
	./bench choke > data/bench_choke.csv
	./bench hit   > data/bench_hit.csv
	@echo "== open ==";  cat data/bench_open.csv
	@echo "== choke =="; cat data/bench_choke.csv
	@echo "== hit ==";   cat data/bench_hit.csv

# end-to-end check: measured full-preheat wall time vs model n(n-1)*t_q
demo-run: demo
	./demo 16 12 open 100  | tee data/demo_192_open.txt
	./demo 32 24 open 100  | tee data/demo_768_open.txt
	./demo 32 24 choke 100 | tee data/demo_768_choke.txt
	./demo 48 36 open 80   | tee data/demo_1728_open.txt

clean:
	rm -f bench demo data/bench_*.csv data/demo_*.txt

.PHONY: all measure demo-run clean
