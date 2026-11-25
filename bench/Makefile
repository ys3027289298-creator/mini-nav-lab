CC ?= cc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
ROOT := ..
MODS := grid smooth region

all: $(addprefix navbench_,$(MODS)) control cache

# One binary per module; the module's source is textually included by navbench.c
# with its main() renamed, so each round re-runs the real self-test in-process.
define MODRULE
navbench_$(1): navbench.c allocwatch.c allocwatch.h $(ROOT)/$(1)/src/$(1).c
	$$(CC) $$(CFLAGS) -I. -DMOD_SRC='"$(ROOT)/$(1)/src/$(1).c"' \
		navbench.c allocwatch.c -o $$@ -lm
endef
$(foreach m,$(MODS),$(eval $(call MODRULE,$(m))))

# Synthetic control with a known answer: a nav-cache-shaped workload whose
# leak, retention and per-entry size are set on the command line. This is how
# the method is validated -- the analyzer must recover the injected truth.
control: control.c allocwatch.c allocwatch.h
	$(CC) $(CFLAGS) -I. control.c allocwatch.c -o $@ -lm

# Release-timing sweep: cache capacity and idle-TTL over a deterministic replay.
# Emits one row per policy; pick_release.py reads the peak/hit-rate pair.
cache: cache.c allocwatch.c allocwatch.h
	$(CC) $(CFLAGS) -I. cache.c allocwatch.c -o $@ -lm

# Three bracket runs per module, each a separate process so one run's heap
# cannot contaminate the next.
run: all
	@mkdir -p measurements
	@printf 'module,run,rounds_planned,round,rss_kb,live_bytes,live_blocks,' \
		> measurements/repo.csv
	@printf 'peak_bytes,allocs,frees,selftest_rc\n' >> measurements/repo.csv
	@for m in $(MODS); do for n in 3 10 30; do \
		./navbench_$$m $$m $$n 2>/dev/null | tail -n +2; \
	done; done >> measurements/repo.csv
	@echo "wrote measurements/repo.csv"

# Controls: a true unbounded leak, a bounded cache, and a partial-reclaim
# workload, all with known parameters. Validates the analyzer end to end.
controls: control
	@mkdir -p measurements
	@printf 'module,run,rounds_planned,round,rss_kb,live_bytes,live_blocks,' \
		> measurements/controls.csv
	@printf 'peak_bytes,allocs,frees,selftest_rc\n' >> measurements/controls.csv
	@./control leak --rounds 12 --entries 40 --entry-bytes 72 --retire 0 \
		--run leak | tail -n +2 >> measurements/controls.csv
	@./control cache --rounds 12 --entries 40 --entry-bytes 72 --retire 1 \
		--run cache | tail -n +2 >> measurements/controls.csv
	@./control decay --rounds 50 --entries 40 --entry-bytes 72 --retire 0.25 \
		--run decay | tail -n +2 >> measurements/controls.csv
	@echo "wrote measurements/controls.csv"

# A control at the rate the monitoring show reported: 2 MiB per round, so the
# slope -> entry-count conversion can be checked against a known 2 MiB truth.
two-mib: control
	@mkdir -p measurements
	@./control two_mib --rounds 10 --entries 29127 --entry-bytes 72 --retire 0 \
		--run two_mib > measurements/two_mib.csv
	@echo "wrote measurements/two_mib.csv"

cache-run: cache
	@mkdir -p measurements
	@./cache > measurements/cache.csv
	@echo "wrote measurements/cache.csv"

# Everything the report quotes, from a clean build.
report: run controls two-mib cache-run
	@for m in $(MODS); do echo "===== $$m ====="; \
		python3 analyze.py --csv measurements/repo.csv --module $$m --entry-bytes 72; \
	done
	@echo "===== controls ====="; \
	for m in leak cache decay; do \
		python3 analyze.py --csv measurements/controls.csv --module $$m --entry-bytes 72; \
	done
	@echo "===== two_mib ====="; \
	python3 analyze.py --csv measurements/two_mib.csv --module two_mib --entry-bytes 72
	@echo "===== release timing ====="; \
	python3 pick_release.py --csv measurements/cache.csv
	@echo "===== analyzer self-tests ====="; python3 test_analyze.py >/dev/null 2>&1 && echo "9/9 analyzer tests OK" || (echo "ANALYZER TESTS FAILED"; exit 1)

clean:
	rm -f $(addprefix navbench_,$(MODS)) control cache \
		measurements/repo.csv measurements/controls.csv \
		measurements/two_mib.csv measurements/cache.csv

.PHONY: all run controls two-mib cache-run report clean
