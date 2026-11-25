CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

.PHONY: all test csv clean

all: tube

tube: src/tube.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: tube
	./tube

csv: tube
	mkdir -p ../artifacts
	./tube --csv > ../artifacts/tube_curves.csv

clean:
	rm -f tube
