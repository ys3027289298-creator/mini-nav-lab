CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: replay

replay: src/replay.c
	$(CC) $(CFLAGS) $< -o $@ -lm

run: replay
	./replay --json results.json

test: replay
	./replay --json results.json

clean:
	rm -f replay results.json
