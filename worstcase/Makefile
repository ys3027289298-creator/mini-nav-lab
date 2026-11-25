CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: worstcase

worstcase: src/worstcase.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: worstcase
	./worstcase

clean:
	rm -f worstcase
