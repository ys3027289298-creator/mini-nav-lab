CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: pursuit

pursuit: src/pursuit.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: pursuit
	./pursuit

clean:
	rm -f pursuit
