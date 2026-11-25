CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: request

request: src/request.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: request
	./request

docs/curves.csv: request
	./request --csv > docs/curves.csv

csv: docs/curves.csv

chart: docs/curves.csv tools/gen_curves.py
	python3 tools/gen_curves.py docs/curves.csv docs/slice-curves.png

clean:
	rm -f request

.PHONY: all test clean csv chart
