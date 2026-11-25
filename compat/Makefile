CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -Wpedantic
CPPFLAGS := -Iinclude -Itests
LDLIBS := -lm

BUILD := build
AWK ?= awk

# One object per generation, so a generation's cost is measured from its own
# file and nothing else. format.c is the dispatch every build pays whatever
# generations it keeps; nav.c is the comparison harness and is not part of the
# shipped reader.
LIB_SRC := src/format.c src/gen2.c src/gen1.c src/gen0.c src/reexport.c src/retention.c
LIB_OBJ := $(LIB_SRC:src/%.c=$(BUILD)/%.o)

FX_OBJ := $(BUILD)/fixtures.o

TOOLS := $(BUILD)/test_mapfile $(BUILD)/reexport $(BUILD)/bench

# Reader size, measured at build time. The bench prints these next to the
# retention curve, so the curve cannot drift from the sources it describes.
LOC_GEN0 := $(shell $(AWK) -f tools/loc.awk src/gen0.c)
LOC_GEN1 := $(shell $(AWK) -f tools/loc.awk src/gen1.c)
LOC_DISPATCH := $(shell $(AWK) -f tools/loc.awk src/format.c)
LOC_DEFS := -DLOC_GEN0=$(LOC_GEN0) -DLOC_GEN1=$(LOC_GEN1) -DLOC_DISPATCH=$(LOC_DISPATCH)

all: $(TOOLS)

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/%.o: src/%.c include/mapfile.h src/internal.h | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

$(BUILD)/nav.o: src/nav.c include/nav.h include/mapfile.h | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

$(BUILD)/fixtures.o: tests/fixtures.c tests/fixtures.h include/mapfile.h | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

$(BUILD)/test_mapfile: tests/test_mapfile.c $(LIB_OBJ) $(BUILD)/nav.o $(FX_OBJ) | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) $^ -o $@ $(LDLIBS)

$(BUILD)/reexport: tools/reexport.c $(LIB_OBJ) | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) $^ -o $@ $(LDLIBS)

$(BUILD)/bench: tools/bench.c $(LIB_OBJ) $(BUILD)/nav.o $(FX_OBJ) | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LOC_DEFS) $^ -o $@ $(LDLIBS)

loc:
	@printf '%-16s %s\n' src/gen0.c "$(LOC_GEN0)"
	@printf '%-16s %s\n' src/gen1.c "$(LOC_GEN1)"
	@printf '%-16s %s\n' src/gen2.c "$(shell $(AWK) -f tools/loc.awk src/gen2.c)"
	@printf '%-16s %s\n' src/format.c "$(LOC_DISPATCH)"

test: $(BUILD)/test_mapfile
	./$(BUILD)/test_mapfile

bench: $(BUILD)/bench
	./$(BUILD)/bench

# The same sources with one legacy reader compiled out, proving the retention
# switch removes it rather than merely hiding it. Such a build must still
# recognise the dropped generation's shape and answer MAP_ERR_VERSION, not
# MAP_ERR_FORMAT: "we do not read this any more" is not "this is not a map".
# Dropping gen 0 is the realistic end state (it is the only generation with
# files today), so both switches are exercised.
test-drop-v1:
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -DMAP_KEEP_V1=0 tests/test_mapfile.c $(LIB_SRC) \
		tests/fixtures.c src/nav.c -o $(BUILD)/test_drop_v1 $(LDLIBS)
	./$(BUILD)/test_drop_v1 --expect-no-gen1

test-drop-v0:
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -DMAP_KEEP_V0=0 tests/test_mapfile.c $(LIB_SRC) \
		tests/fixtures.c src/nav.c -o $(BUILD)/test_drop_v0 $(LDLIBS)
	./$(BUILD)/test_drop_v0 --expect-no-gen0

test-drop-both:
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -DMAP_KEEP_V0=0 -DMAP_KEEP_V1=0 \
		tests/test_mapfile.c $(LIB_SRC) tests/fixtures.c src/nav.c \
		-o $(BUILD)/test_drop_both $(LDLIBS)
	./$(BUILD)/test_drop_both --expect-no-gen0 --expect-no-gen1

check: test bench test-drop-v1 test-drop-v0 test-drop-both
	@echo "all checks passed"

clean:
	rm -rf $(BUILD)

.PHONY: all clean test bench loc test-drop-v1 test-drop-v0 test-drop-both check
