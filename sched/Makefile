# Preemptive priority scheduler SLA analysis.
PY ?= python3
# -P keeps CWD off sys.path (guards against planted modules); site-packages
# are passed explicitly because isolated mode would drop them too.
SITE := $(shell $(PY) -c "import numpy,os;print(os.path.dirname(os.path.dirname(numpy.__file__)))" 2>/dev/null)
PP := PYTHONPATH=$(SITE)

.PHONY: all calib sweep analyze report clean test

all: report

calib/calib.csv: calib/astar_calib.c
	cd calib && gcc -O2 -std=c11 -Wall -Wextra astar_calib.c -o astar_calib -lm \
	  && ./astar_calib 3000 > calib.csv

calib: calib/calib.csv

sweep: calib/calib.csv
	$(PP) $(PY) -P sim.py

analyze: sweep
	$(PP) $(PY) -P analyze.py
	$(PP) $(PY) -P emit_chart_data.py

report: analyze
	$(PP) $(PY) -P build_report.py

test: calib/calib.csv
	$(PP) $(PY) -P -c "import sys,os; sys.path.insert(0,os.getcwd()); import sim; \
	w=sim.load_work(); \
	r=sim.simulate(4,'bg_heavy',7.0,8000,1000,1,'reserve',True,w); \
	assert r['preempt_events']>=0 and 0<=r['P0']['one_frame_hit_rate']<=1; \
	print('sim smoke OK; P0 hit=%.4f preempts=%d'%(r['P0']['one_frame_hit_rate'],r['preempt_events']))"

clean:
	rm -f calib/astar_calib sweep.json report.html
	rm -rf results
