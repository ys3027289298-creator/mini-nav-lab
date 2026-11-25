CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -D_POSIX_C_SOURCE=200809L

all: hier

hier: src/hier.c
	$(CC) $(CFLAGS) $< -o $@ -lm

# build + benchmark, keep the CSV output for the analysis script
test: hier
	./hier | tee results/bench.csv

# refresh the fitted model, summary.json and chart data
chart: test
	python3 -I tools/analyze.py

clean:
	rm -f hier
