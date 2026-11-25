CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: batch

batch: src/batch.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: batch
	./batch

plot: test
	python3 plot_curves.py

clean:
	rm -f batch curves.csv tiers.csv
