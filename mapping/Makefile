PY ?= python3
OUT ?= ../artifacts/mapping

.PHONY: all test run clean

all: test

run: migrate_map_labels.py
	$(PY) migrate_map_labels.py --out $(OUT)

# 脚本会自检全部门限；覆盖率不为 100% 或任一档准确率 < 95% 时退出码为 2。
test: run

clean:
	rm -rf $(OUT) __pycache__
