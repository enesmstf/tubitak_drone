"""tune.py - grid search of angle-law gains (normalised error, window criterion).
Result is copied into TUNED in run_experiments.py."""
import itertools, json
from multiprocessing import Pool
import itertools as _it
import numpy as np
from dataclasses import replace
from sim_core import *


def one_center(cfg, ch, n=20, seed=0, dist=(60,80)):
    cfg = replace(cfg, plan="paper", centers=((0.0,0.0,15.0),))
    res = monte_carlo(cfg, ch, n_runs=n, seed=seed, r_min=dist[0], r_max=dist[1])
    a = np.array([r.lines[0].ang_err_deg for r in res]); c=np.mean([r.lines[0].converged for r in res]); t=np.mean([r.lines[0].t_used for r in res])
    return np.median(a), np.mean(a), c, t

QUIET=ChannelCfg(sigma_t=1.2, rho=0.3, sigma_s=1.0, corr_len=20, bias_std=0.5)
PAPER=ChannelCfg(sigma_t=0.17, rho=0.0, sigma_s=0, bias_std=0, quant=0)
base=dict(signal_filter="kalman",normalize=True,criterion="window",average_angles=True,t_min=300,t_max=600,window=30,tol_db=0.15)
grids={
 "pid":[dict(kp=a,ki=b,kd=0.0) for a,b in itertools.product([0.025,0.05,0.1],[0.0005,0.0016,0.005])],
 "ismf2024":[dict(ks=a,ki=b,lam=l,phi=1.0) for a,b,l in itertools.product([0.05,0.1,0.2],[0.0005,0.0016,0.005],[0.2,1.0])],
 "sismf2026":[dict(k=a,kappa=b,lam=l,phi=1.0) for a,b,l in itertools.product([0.05,0.1,0.2],[0.0005,0.0016,0.005],[0.01,0.1])],
}
def job(args):
    law,kw=args
    q=one_center(SphereCfg(R=20,alt=25,centers=((0,0,25),),law=law,law_kw=kw,**base),QUIET,n=24)
    p=one_center(SphereCfg(R=5,alt=15,centers=((0,0,15),),law=law,law_kw=kw,**base),PAPER,n=24)
    return law,kw,q[0],q[1],p[0],p[1]
if __name__=="__main__":
    jobs=[(l,k) for l,g in grids.items() for k in g]
    with Pool() as pool: res=pool.map(job,jobs)
    best={}
    for law,kw,qm,qa,pm,pa in res:
        score=qm+pm
        if law not in best or score<best[law][0]: best[law]=(score,kw,qm,qa,pm,pa)
    for law,kw,qm,qa,pm,pa in sorted(res,key=lambda r:(r[0],r[2]+r[4]))[:]:
        pass
    for law,(sc,kw,qm,qa,pm,pa) in best.items(): print(law,kw,"quietR20 med %.1f mean %.1f | paperR5 med %.1f mean %.1f"%(qm,qa,pm,pa))
    json.dump({l:b[1] for l,b in best.items()},open("tuned_gains.json","w"),indent=1)
