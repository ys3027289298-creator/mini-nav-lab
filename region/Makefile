CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: region

region: src/region.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: region
	./region

clean:
	rm -f region
