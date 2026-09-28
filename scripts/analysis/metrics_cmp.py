import json, sys
for r in sys.argv[1:]:
    try:
        d = json.load(open(f"/home/project/MY-LIVO/results/{r}/evaluation/metrics.json")); o = d["trajectory"]["optimized"]["xy"]; n = d["trajectory"]["native"]["xy"]
        print(f"{r:24s} optimized xy median/p95/max {o['median_m']*100:.2f}/{o['p95_m']*100:.2f}/{o['max_m']*100:.2f} cm | native median/max {n['median_m']*100:.1f}/{n['max_m']*100:.1f} cm")
    except Exception as e:
        print(r, "no metrics:", e)
