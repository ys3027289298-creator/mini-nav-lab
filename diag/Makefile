CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: diag

diag: src/diag.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: diag
	./diag

clean:
	rm -f diag
