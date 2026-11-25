CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

PLOT ?= tools/plot_origin_tradeoff.py
CSV ?= origin_curve.csv
PNG ?= origin_tradeoff.png

all: switch

switch: src/switch.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: switch
	./switch

$(CSV): switch
	./switch --csv > $(CSV)

plot: $(PNG)

$(PNG): $(CSV) $(PLOT)
	python3 $(PLOT) $(CSV) $(PNG)

clean:
	rm -f switch $(CSV) $(PNG)

.PHONY: all test plot clean
