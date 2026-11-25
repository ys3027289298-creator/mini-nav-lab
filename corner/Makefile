CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: corner

corner: src/corner.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: corner
	./corner

csv: corner
	./corner --csv > curves.csv

clean:
	rm -f corner curves.csv
