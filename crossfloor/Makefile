CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: crossfloor

crossfloor: src/crossfloor.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: crossfloor
	./crossfloor selftest data/floors.txt data/survey.txt

# canonical one-run-per-case acceptance report
report: crossfloor
	mkdir -p docs
	./crossfloor selftest data/floors.txt data/survey.txt > docs/selftest.txt
	./crossfloor accept data/floors.txt data/survey.txt > docs/accept.txt
	{ echo '$$ make test'; cat docs/selftest.txt; echo; echo '$$ ./crossfloor accept data/floors.txt data/survey.txt'; cat docs/accept.txt; } > docs/test-output.txt
	cat docs/selftest.txt
	echo
	cat docs/accept.txt

docs/curves.csv: crossfloor data/floors.txt
	./crossfloor sweep data/floors.txt 0.25 20 0.25 > docs/curves.csv

csv: docs/curves.csv

chart: docs/curves.csv tools/gen_curves.py
	python3 tools/gen_curves.py docs/curves.csv data/survey.txt docs/selection-length-curves.png

docs/routes.txt: crossfloor data/floors.txt data/survey.txt
	./crossfloor routes data/floors.txt data/survey.txt > docs/routes.txt

map: docs/routes.txt tools/gen_floor_map.py
	python3 tools/gen_floor_map.py data/floors.txt docs/routes.txt docs/floor-paths.png

docs: test report csv chart map

clean:
	rm -f crossfloor

.PHONY: all test clean report csv chart map docs
