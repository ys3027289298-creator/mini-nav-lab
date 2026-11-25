PY ?= python3

.PHONY: help test exp-u exp-o exp-r mystery mirror tradeoff

help:
	@echo "make test      mirror self-test + all three injections + 340/352 case"
	@echo "make exp-u     isolated pocket  -> U term"
	@echo "make exp-o     border map       -> O term"
	@echo "make exp-r     duplicate fill   -> R term"
	@echo "make mystery   340 vs 352 reverse solve"
	@echo "make tradeoff  precision/memory curves + CSV/PNG under artifacts/"

test: mirror exp-u exp-o exp-r mystery
	@echo "reconcile: all checks passed"

mirror:
	$(PY) reconcile.py mirror-selftest

exp-u:
	$(PY) reconcile.py exp-u

exp-o:
	$(PY) reconcile.py exp-o

exp-r:
	$(PY) reconcile.py exp-r

mystery:
	$(PY) reconcile.py mystery

tradeoff:
	$(PY) tradeoff.py
