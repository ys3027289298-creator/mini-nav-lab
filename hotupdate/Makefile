CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: hotupdate

hotupdate: src/hotupdate.c
	$(CC) $(CFLAGS) $< -o $@

test: hotupdate
	./hotupdate

plot: hotupdate
	PYTHONPATH=$(shell python3 -c 'import site; print(site.getusersitepackages())') \
	python3 -s tools/plot_curves.py

clean:
	rm -f hotupdate artifacts/*.png
