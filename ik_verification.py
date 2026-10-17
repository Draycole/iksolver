import numpy as np, time, json, math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pathlib import Path

L = 5.0
OUT = Path(__file__).resolve().parent / "results"
OUT.mkdir(parents=True, exist_ok=True)

def ik(x, y, L=L, elbow="up"):
    d = np.hypot(x, y)
    th1 = np.arccos(np.clip(d / (2 * L), -1, 1))          # eq 7
    b1 = np.arctan2(y, x)                                  # eq 8
    th2 = np.arccos(np.clip(1 - d**2 / (2 * L**2), -1, 1)) # eq 14
    if elbow == "up":
        a1 = th1 + b1                                      # eq 9
        a2 = np.pi - a1 - th2                              # eq 16
    else:
        a1 = b1 - th1
        a2 = -(b1 + th1)
    return a1, a2, th1, b1, th2, d

def fk(a1, a2, L=L):
    # alpha2 clockwise from horizontal through B
    return L*np.cos(a1) + L*np.cos(a2), L*np.sin(a1) - L*np.sin(a2)

def wrap(a):
    return (a + np.pi) % (2*np.pi) - np.pi

res = {}
# --- Table: representative targets
targets = [(7,3),(5,5),(0,8),(-4,6),(-6,-2),(3,-7),(9.9,0),(2,1),(0.5,0.2),(10,0)]
rows=[]
for (x,y) in targets:
    a1,a2,th1,b1,th2,d = ik(x,y)
    xe,ye = fk(a1,a2)
    err = math.hypot(xe-x, ye-y)
    # identity check
    ident = abs(wrap(a2 - (th1-b1)))
    rows.append(dict(x=x,y=y,d=round(float(d),3),th1=round(math.degrees(th1),2),b1=round(math.degrees(b1),2),
                     th2=round(math.degrees(th2),2),a1=round(math.degrees(a1),2),a2=round(math.degrees(wrap(a2)),2),
                     err=float(err), ident=float(ident)))
res["table"]=rows

# --- compare against repo implementation (atan2 recomputation of alpha2)
def repo_ik(x,y):
    t1 = np.arccos(np.sqrt(x**2+y**2)/(2*L))
    a1 = math.degrees(t1 + math.atan2(y,x))
    x1 = L*math.cos(math.radians(a1)); y1 = L*math.sin(math.radians(a1))
    a2 = -math.degrees(math.atan2(y-y1, x-x1))
    return a1,a2
mx=0
for (x,y) in targets:
    a1,a2,*_ = ik(x,y); r1,r2 = repo_ik(x,y)
    mx=max(mx,abs(math.degrees(a1)-r1),abs(math.degrees(wrap(a2))-((r2+180)%360-180)))
res["repo_maxdiff_deg"]=mx

# --- Monte Carlo
rng=np.random.default_rng(2025)
N=1_000_000
r = 2*L*np.sqrt(rng.random(N)); phi = rng.uniform(-np.pi,np.pi,N)
X,Y = r*np.cos(phi), r*np.sin(phi)
for elbow in ["up","down"]:
    a1,a2,th1,b1,th2,d = ik(X,Y,elbow=elbow)
    xe,ye=fk(a1,a2); e=np.hypot(xe-X,ye-Y)
    res[f"mc_{elbow}"]=dict(mean=float(e.mean()),max=float(e.max()),p99=float(np.percentile(e,99)))
    if elbow=="up":
        res["ident_max"]=float(np.abs(wrap(a2-(th1-b1))).max())
        eup=e; dup=d
# timing vectorised
ts=[]
for _ in range(10):
    t=time.perf_counter(); ik(X,Y); ts.append(time.perf_counter()-t)
res["vec_ns_per_target"]=min(ts)/N*1e9
# scalar timing
xs,ys=X[:20000],Y[:20000]
def ik_scalar(x,y):
    d=math.hypot(x,y); th1=math.acos(min(1.0,d/(2*L))); b1=math.atan2(y,x)
    th2=math.acos(max(-1.0,1-d*d/(2*L*L))); a1=th1+b1; return a1, math.pi-a1-th2
t=time.perf_counter()
for x,y in zip(xs.tolist(),ys.tolist()): ik_scalar(x,y)
res["scalar_us_per_target"]=(time.perf_counter()-t)/len(xs)*1e6

# unreachable
res["unreachable_flag"]= bool(np.isnan(np.arccos(10.5/10)))

# --- Damped least squares iterative baseline
def jac(a1,a2):
    # derivative of fk wrt (a1,a2) with a2 CW
    return np.array([[-L*np.sin(a1), -L*np.sin(a2)],
                     [ L*np.cos(a1), -L*np.cos(a2)]])
def dls(x,y,q0=(0.3,0.3),lam=0.5,tol=1e-9,maxit=500):
    q=np.array(q0,float)
    for k in range(maxit):
        xe,ye=fk(*q); e=np.array([x-xe,y-ye])
        if np.hypot(*e)<tol: return q,k,np.hypot(*e),True
        J=jac(*q)
        q=q+J.T@np.linalg.solve(J@J.T+lam**2*np.eye(2),e)
    return q,maxit,np.hypot(*e),False
M=2000
its=[];errs=[];conv=0
t=time.perf_counter()
for x,y in zip(X[:M],Y[:M]):
    q,k,e,c=dls(x,y); its.append(k); errs.append(e); conv+=c
tdls=(time.perf_counter()-t)/M
res["dls"]=dict(us_per_target=tdls*1e6,mean_it=float(np.mean(its)),median_it=float(np.median(its)),
                max_it=int(np.max(its)),conv_rate=conv/M,mean_err=float(np.mean(errs)),max_err=float(np.max(errs)))

# repeated with closed-form, same targets, scalar
t=time.perf_counter()
for x,y in zip(X[:M].tolist(),Y[:M].tolist()): ik_scalar(x,y)
res["cf_us_same"]=(time.perf_counter()-t)/M*1e6

# sensitivity near boundary
res["sens"]=[(s, 1/(2*L*math.sin(math.acos(s)))) for s in [0.5,0.9,0.99,0.999]]
print(json.dumps(res,indent=1))
json.dump(res,open(OUT / "results.json","w"),indent=1)

# Figure 4: configurations
plt.rcParams.update({"font.family":"serif","font.size":10})
fig,axs=plt.subplots(1,2,figsize=(7.2,3.6))
sel=[(7,3),(-4,6),(3,-7),(2,1)]
cols=["#1f4e9a","#c0392b","#2e8b57","#8e44ad"]
for ax,elbow,title in zip(axs,["up","down"],["(a) Elbow-up branch","(b) Elbow-down branch"]):
    th=np.linspace(0,2*np.pi,300)
    ax.plot(2*L*np.cos(th),2*L*np.sin(th),"--",color="0.6",lw=0.8)
    for (x,y),c in zip(sel,cols):
        a1,a2,*_=ik(x,y,elbow=elbow)
        bx,by=L*np.cos(a1),L*np.sin(a1); cx,cy=fk(a1,a2)
        ax.plot([0,bx,cx],[0,by,cy],"-o",color=c,lw=2,ms=4)
        ax.plot(x,y,"x",color="k",ms=8,mew=1.5)
    ax.plot(0,0,"ks",ms=6)
    ax.set_aspect("equal"); ax.set_xlim(-11,11); ax.set_ylim(-11,11)
    ax.grid(alpha=0.3); ax.set_title(title,fontsize=9); ax.set_xlabel("x"); ax.set_ylabel("y")
fig.tight_layout(); fig.savefig(OUT / "fig4.png",dpi=300)

# --- Figure 5: error vs normalised distance
fig,ax=plt.subplots(figsize=(6,3.2))
idx=rng.choice(N,20000,replace=False)
ax.scatter(dup[idx]/(2*L),np.maximum(eup[idx],1e-17),s=2,alpha=0.4,color="#1f4e9a")
ax.set_yscale("log"); ax.set_xlabel(r"Normalised reach $d/2L$"); ax.set_ylabel("Position error (units)")
ax.set_ylim(1e-17,1e-12); ax.grid(alpha=0.3,which="both")
fig.tight_layout(); fig.savefig(OUT / "fig5.png",dpi=300)

print(f"\nResults saved to: {OUT}")