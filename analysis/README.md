# One-off log-forensics scripts

Each script re-derives a specific finding of a debug_log.md entry straight
from `catch_logs/*.jsonl` + the transform JSONs — see the docstring at the
top of each for what question it answers and which entry it backs. Written
for the 2026-07-18 night-shift analysis (debug_log.md 2026-07-18); they
hardcode the 2026-07-17 log glob but are trivially retargetable at newer
sessions. `analyze_catch_logs.py` in the repo root is the older (2026-07-17)
general-purpose one.
