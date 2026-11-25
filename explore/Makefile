CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

.PHONY: all test calibrate chart clean

all: explore

explore: src/explore.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: explore
	./explore
	python3 tools/calibrate.py --check

calibrate:
	python3 tools/calibrate.py

chart: docs/explore-curves.png

docs/explore-curves.png: tools/plot_curves.py docs/curves.csv docs/summary.json
	python3 tools/plot_curves.py docs/curves.csv docs/summary.json docs/explore-curves.png

docs/curves.csv docs/summary.json:
	python3 tools/calibrate.py

clean:
	rm -f explore
