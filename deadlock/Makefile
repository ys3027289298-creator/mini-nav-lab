PYTHON ?= python3

.PHONY: test report clean check

test: report check

report:
	$(PYTHON) src/deadlock_lab.py run --out ../artifacts/deadlock-lab

check:
	$(PYTHON) -m py_compile src/deadlock_lab.py

clean:
	rm -rf ../artifacts/deadlock-lab src/__pycache__
