CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
ROUNDS ?= 6

all: bench

bench: bench.c
	$(CC) $(CFLAGS) $< -o $@ -lm

# independent rounds (different seeds) then pool every kind into one CSV so
# heavy-tail moments are estimated from ROUNDS*4000 samples, not a single run
run: bench
	@for i in $$(seq 0 $$(($(ROUNDS)-1))); do \
		mkdir -p results/r$$i; ./bench $$i results/r$$i; \
	done
	@for kind in full miss cache; do \
		cat results/r*/times_$$kind.csv > results/times_$$kind.csv; \
	done
	@wc -l results/times_full.csv | awk '{print "pooled samples per kind:", $$1}'

clean:
	rm -f bench
