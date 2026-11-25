CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: resume

resume: src/resume.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: resume
	./resume

clean:
	rm -f resume
