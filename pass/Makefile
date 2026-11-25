CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: pass

pass: src/pass.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: pass
	./pass

clean:
	rm -f pass
