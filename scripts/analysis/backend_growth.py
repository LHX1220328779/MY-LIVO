#!/usr/bin/env python3
"""Audit actual saved frontend/backend diagnostics, including interrupted CSV tails."""
import argparse
import csv
import json
import pathlib

import numpy as np


def fusion(path):
    p = pathlib.Path(path)
    return p if (p/'diagnostics.csv').exists() else sorted((p/'Log/fusion').glob('*'))[-1]


def table(path, comments=False):
    with open(path) as f:
        lines = [line for line in f if line.strip() and (not comments or not line.startswith('#'))]
    rows = list(csv.reader(lines)); names = rows[0]
    complete = [row for row in rows[1:] if len(row)==len(names)]
    values = np.array([[float(v) for v in row] for row in complete])
    return {name:values[:, i] for i,name in enumerate(names)}, len(rows)-1-len(complete)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('runs', nargs='+');ap.add_argument('--out', required=True)
    args=ap.parse_args();out=pathlib.Path(args.out);out.mkdir(parents=True,exist_ok=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(3,1,figsize=(12,11),sharex=True)
    report={}
    for run in args.runs:
        p=fusion(run);d,partial=table(p/'diagnostics.csv');t=d['stamp']-d['stamp'][0]
        name=pathlib.Path(run).name; windows=[]
        for lo in range(0,int(t[-1])+1,60):
            m=(t>=lo)&(t<lo+60)
            if not m.any():continue
            entry={'start_s':lo,'frames':int(m.sum()),'keyframes_at_end':int(d['keyframes'][m][-1]),
                   'backend_ms_mean':float(np.mean(d['processing_ms'][m])),
                   'backend_ms_p95':float(np.percentile(d['processing_ms'][m],95)),
                   'position_residual_max_m':float(np.nanmax(d['rtk_residual_m'][m])),
                   'native_reference_max_m':float(np.nanmax(d['frontend_rtk_residual_m'][m]))}
            for c in ['prepare_ms','association_ms','solve_ms','refresh_ms','relinearized','reeliminated']:
                entry[c+'_mean']=float(np.mean(d[c][m]))
            windows.append(entry)
        report[name]={'frames':len(t),'span_s':float(t[-1]),'partial_tail_rows_excluded':partial,
                      'windows':windows,'max_online_position_residual_m':float(np.nanmax(d['rtk_residual_m'])),
                      'reference_failure_count':int(np.count_nonzero(d['reference_failure']))}
        ax[0].plot(t,d['keyframes'],label=name)
        ax[1].plot([v['start_s']+30 for v in windows],[v['backend_ms_mean'] for v in windows],'.-',label=name)
        ax[2].plot(t[::5],d['rtk_residual_m'][::5],label=name,linewidth=.7)
    ax[0].set_ylabel('Backend keyframes');ax[1].set_ylabel('Backend mean ms / scan')
    ax[2].set_ylabel('Online rear-axle residual to RTK (m)');ax[2].set_xlabel('Data time since first mapped scan (s)')
    ax[2].axhline(.2,color='k',linestyle='--',linewidth=.8)
    for a in ax:a.legend();a.grid(alpha=.2)
    fig.tight_layout();fig.savefig(out/'backend_growth.png',dpi=150)
    (out/'backend_growth.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:{x:v[x] for x in ['frames','span_s','max_online_position_residual_m']} for k,v in report.items()},indent=2))


if __name__=='__main__':main()
