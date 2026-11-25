#!/usr/bin/env python3
"""Inject chart_data.json into report_template.html -> report.html."""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
data = open(os.path.join(HERE, "results", "chart_data.json")).read()
tpl = open(os.path.join(HERE, "report_template.html")).read()
# embed as JSON inside a <script type=application/json> block; avoid breaking
# out of it (no </script> sequence appears in the data)
assert "</script>" not in data
out = tpl.replace("__DATA__", data)
path = os.path.join(HERE, "report.html")
open(path, "w").write(out)
print("wrote", path, f"({len(out)} bytes)")
