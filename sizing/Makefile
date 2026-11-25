PYTHON ?= python3

.PHONY: all test chart clean

all: test chart

test:
	$(PYTHON) -I sizing.py

chart: results/curves.csv
	$(PYTHON) -I make_chart.py

results/curves.csv: sizing.py
	$(PYTHON) -I sizing.py

clean:
	rm -rf results __pycache__ chart.html
