CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: degrade

degrade: src/degrade.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: degrade
	./degrade

clean:
	rm -f degrade
