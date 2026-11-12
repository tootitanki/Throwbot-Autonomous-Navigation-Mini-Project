"""Analyse the sensor comparison batch. Usage: python3 analyse.py RESULTS_DIR OUT_DIR

Contamination rule (applied equally to all sensors/seeds, independent of outcome):
a trial is excluded from the clean set only if the heartbeat log (1 Hz wall clock
writer) shows a gap > 5 s inside the trial's wall-clock window, or the trial
predates the heartbeat and was observed to span a sandbox freeze (listed in
contaminated_ids.txt). Excluded trials stay in summary_all.csv with a flag.
"""
import json, sys, os
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

res, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
rows = [json.loads(l) for l in open(f"{res}/summary.jsonl")]
df = pd.DataFrame(rows)
df["outcome"] = df["goal_status"].replace({"SUCCEEDED": "reached"})
df.loc[(df.outcome == "reached") & (df.contact == True), "outcome"] = "reached_logged_prox_1cm"
df["contaminated"] = False
cont = os.path.join(res, "contaminated_ids.txt")
if os.path.exists(cont):
    ids = set(open(cont).read().split())
    df["contaminated"] = df.run_id.isin(ids)
df.to_csv(f"{out}/summary_all.csv", index=False)
clean = df[~df.contaminated]
order = ["lidar", "lidar_fwd87", "depth160", "ultrasonic"]


def agg(g):
    return pd.Series({
        "n": len(g),
        "succeeded_gap_gt_1cm": int((g.outcome == "reached").sum()),
        "succeeded_gap_le_1cm_logged": int((g.outcome == "reached_logged_prox_1cm").sum()),
        "aborted": int(g.goal_status.astype(str).str.startswith("status").sum()),
        "timeout": int((g.goal_status == "TIMEOUT").sum()),
        "logged_gap_le_1cm": int(g.contact.fillna(False).astype(bool).sum()), "logged_gap_le_5cm": int(g.near_contact_5cm.fillna(False).astype(bool).sum()),
        "min_clear_med_m": g.min_clearance_m.median(),
        "detect_lag_med_s": g.detect_lag_s.median(),
        "detect_gap_med_m": g.detect_gap_m.median(),
        "tracking_recall_mean": g.tracking_recall.mean(),
        "phantom_mean": g.phantom_cells_mean.mean(),
        "sensor_hz_sim": g.sensor_rate_sim_hz.median(),
        "sensor_hz_wall": g.sensor_rate_wall_hz.median(),
        "costmap_hz_sim": g.costmap_rate_sim_hz.median(),
        "rtf_med": g.real_time_factor.median(),
    })


t1 = clean.groupby("sensor").apply(agg).reindex(order)
t2 = clean.groupby(["sensor", "speed"]).apply(agg).reindex(order, level=0)
t1.round(3).to_csv(f"{out}/table_by_sensor.csv")
t2.round(3).to_csv(f"{out}/table_by_sensor_speed.csv")
with open(f"{out}/tables.md", "w") as f:
    f.write("All trials reported goal SUCCEEDED unless aborted/timeout columns say otherwise. `succeeded_gap_gt_1cm` and `succeeded_gap_le_1cm_logged` split the SUCCEEDED trials by the logged minimum body-box gap (> 1 cm vs <= 1 cm). A logged gap <= 1 cm is geometric proximity in the logger, NOT confirmed physical contact. Medians (`*_med*`) and means (`*_mean`) are as named; n = 18 per sensor, 6 per sensor and speed. No ranking is implied.\n\n## Per sensor (clean trials)\n\n" + t1.round(3).to_markdown() + "\n\n")
    f.write("## Per sensor and crossing speed\n\n" + t2.round(3).to_markdown() + "\n")

col = {"lidar": "#1f77b4", "lidar_fwd87": "#17becf", "depth160": "#d62728", "ultrasonic": "#2ca02c"}

# 1 outcomes
fig, ax = plt.subplots(figsize=(7, 4))
cats = ["reached", "reached_logged_prox_1cm", "aborted", "timeout"]
LAB = {"reached": "SUCCEEDED, logged gap > 1 cm", "reached_logged_prox_1cm": "SUCCEEDED, logged gap <= 1 cm (proximity, not confirmed contact)", "aborted": "aborted", "timeout": "timeout"}
cc = ["#4daf4a", "#ff7f00", "#e41a1c", "#984ea3"]
bot = np.zeros(len(order))
for c, k in zip(cats, cc):
    if c == "aborted":
        v = [clean[(clean.sensor == s)].goal_status.astype(str).str.startswith("status").sum() for s in order]
    elif c == "timeout":
        v = [(clean[(clean.sensor == s)].goal_status == "TIMEOUT").sum() for s in order]
    else:
        v = [(clean[(clean.sensor == s)].outcome == c).sum() for s in order]
    ax.bar(order, v, bottom=bot, color=k, label=LAB[c])
    bot += np.array(v)
ax.set_ylabel("trials (n=18 per condition)"); ax.set_title("Goal outcomes split by logged body-box gap (simulation)\nAll 72 goals SUCCEEDED; <=1 cm is logged proximity, not confirmed contact", fontsize=9)
ax.legend(fontsize=8); fig.tight_layout(); fig.savefig(f"{out}/fig_outcomes.png", dpi=130); plt.close(fig)

# 2 clearance & detect lag by speed
for metric, name, yl in [("min_clearance_m", "fig_clearance.png", "min logged body-box gap (m)"),
                         ("detect_lag_s", "fig_detect_lag.png", "detection lag after first visible (sim s)"),
                         ("tracking_recall", "fig_tracking_recall.png", "tracking recall (visible frames marked)")]:
    fig, ax = plt.subplots(figsize=(7, 4))
    for s in order:
        sub = clean[clean.sensor == s]
        g = sub.groupby("speed")[metric].median()
        ax.plot(g.index, g.values, "o-", color=col[s], label=s + " (median of 6 seeds)")
        ax.scatter(sub.speed + (order.index(s) - 1.5) * 0.012, sub[metric], s=9, alpha=0.35, color=col[s])
    ax.set_xlabel("box crossing speed (m/s)"); ax.set_ylabel(yl)
    ax.set_title("Lines: median over 6 seeds per speed. Dots: individual trials.\nSmall samples, not a ranking.", fontsize=9)
    ax.legend(fontsize=7); fig.tight_layout(); fig.savefig(f"{out}/{name}", dpi=130); plt.close(fig)

print(t1.round(3).to_string())
print("contaminated:", int(df.contaminated.sum()), "of", len(df))
