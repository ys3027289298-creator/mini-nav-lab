CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
PYTHON ?= python3

all: migrate

migrate: src/migrate.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: migrate
	./migrate

curves: test
	PYTHONPATH=$(HOME)/.local/lib/python3.12/site-packages $(PYTHON) -P plot_curves.py curves.csv results.csv

clean:
	rm -f migrate results.csv segments.csv curves.csv curves_light.png curves_dark.png

.PHONY: all test curves clean
