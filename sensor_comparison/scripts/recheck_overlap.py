"""Recompute min clearance with the true oriented 0.40x0.30 body vs the 0.5x0.5 box (SAT distance, via sampled polygon)."""
import csv, json, math, sys, glob, os
OBS_X=5.0; BH=0.25; L,W=0.20,0.15
def poly(x,y,yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return [(x+c*a-s*b,y+s*a+c*b) for a,b in ((L,W),(L,-W),(-L,-W),(-L,W))]
def pdist_rect(p,cx,cy,h):
    dx=max(abs(p[0]-cx)-h,0); dy=max(abs(p[1]-cy)-h,0); return math.hypot(dx,dy)
def inside_rect_poly(pts,cx,cy,h):
    return any(abs(p[0]-cx)<=h and abs(p[1]-cy)<=h for p in pts)
def seg_pts(a,b,n=20): return [(a[0]+(b[0]-a[0])*i/n,a[1]+(b[1]-a[1])*i/n) for i in range(n+1)]
def clearance(x,y,yaw,by):
    P=poly(x,y,yaw); best=1e9
    for i in range(4):
        for q in seg_pts(P[i],P[(i+1)%4]): best=min(best,pdist_rect(q,OBS_X,by,BH))
    # box corners vs robot rect (box poking in)
    cs=[(OBS_X+sx*BH,by+sy*BH) for sx in(-1,1) for sy in(-1,1)]
    c,s=math.cos(-yaw),math.sin(-yaw)
    for q in cs:
        lx=c*(q[0]-x)-s*(q[1]-y); ly=s*(q[0]-x)+c*(q[1]-y)
        if abs(lx)<L-0.02 and abs(ly)<W-0.02: return -1.0
    return best
rows=[]
for l in open('summary.jsonl'):
    d=json.loads(l); f='traces/%s.csv'%d['run_id']
    if not os.path.exists(f): continue
    m=1e9
    for r in csv.DictReader(open(f)):
        m=min(m,clearance(float(r['robot_x']),float(r['robot_y']),float(r['robot_yaw']),float(r['box_y'])))
    rows.append((d['run_id'],d['sensor'],d['speed'],d['min_clearance_m'],round(m,3),m<0.0 or m<=1e-9 and False))
import collections
agg=collections.defaultdict(lambda:[0,0,0])
for rid,s,sp,old,new,_ in rows:
    a=agg[s]; a[0]+=1; a[1]+= (old<0); a[2]+= (new<=0.02)
for s,a in agg.items(): print(s,'n',a[0],'circle-overlap',a[1],'true-footprint contact(<=2cm or penetrating)',a[2], 'penetrating>2cm', sum(1 for x in rows if x[1]==s and x[4]<0))
json.dump(rows,open('/tmp/study/ws/footprint_recheck.json','w'))
