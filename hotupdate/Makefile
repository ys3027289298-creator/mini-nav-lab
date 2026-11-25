PY ?= python3

.PHONY: all test sim plots clean

all: sim

sim:
	$(PY) src/run_experiments.py

plots: sim
	$(PY) src/make_plots.py

test: sim
	$(PY) src/selfcheck.py

clean:
	rm -f results/sim_results.json results/misjudgement_diagnostics.csv
	rm -f figures/*.png
	rm -rf src/__pycache__
