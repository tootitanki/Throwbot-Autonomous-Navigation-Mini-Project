"""Compare Gazebo world-pose ground truth to the odometry pose used by the logger; recompute clearance from ground truth."""
import re, csv, math, sys, json
D=sys.argv[1]; RID=sys.argv[2]
txt=open(f'{D}/pose_stream.txt').read()
msgs=txt.split('header {')[1:]
gt={}  # t -> (rx,ry,ryaw,my)
for m in msgs:
    ts=re.search(r'sec: (\d+)',m); ns=re.search(r'nsec: (\d+)',m)
    t=int(ts.group(1)) if ts else 0
    if ns: t+=int(ns.group(1))*1e-9
    d={}
    for blk in m.split('pose {')[1:]:
        n=re.search(r'name: "([^"]+)"',blk).group(1)
        g=lambda k,b=blk:(lambda r: float(r.group(1)) if r else 0.0)(re.search(k+r': ([-\d.e+]+)',b))
        pos=blk.split('position {')[1].split('}')[0] if 'position {' in blk else ''
        ori=blk.split('orientation {')[1].split('}')[0] if 'orientation {' in blk else ''
        gx=lambda s,k:(lambda r: float(r.group(1)) if r else 0.0)(re.search(r'\b'+k+r': ([-\d.e+]+)',s))
        x,y=gx(pos,'x'),gx(pos,'y'); qz,qw=gx(ori,'z'),gx(ori,'w')
        d[n]=(x,y,2*math.atan2(qz,qw if qw else 1e-9))
    if 'throwbot' in d and 'mover' in d: gt[round(t,1)]=(d['throwbot'],d['mover'])
tr={round(float(r['t_sim']),1):r for r in csv.DictReader(open(f'{D}/traces/{RID}.csv'))}
L,W,BH=0.20,0.15,0.25
def rect_clear(x,y,yaw,bx,by):
    c,s=math.cos(yaw),math.sin(yaw); best=1e9; pen=0.0
    pts=[]
    for i in range(41):
        for (a,b),(a2,b2) in (((L,W),(L,-W)),((L,-W),(-L,-W)),((-L,-W),(-L,W)),((-L,W),(L,W))):
            u=i/40; px=a+(a2-a)*u; py=b+(b2-b)*u
            pts.append((x+c*px-s*py,y+s*px+c*py))
    dm=1e9
    for p in pts:
        dx=max(abs(p[0]-bx)-BH,0); dy=max(abs(p[1]-by)-BH,0); dm=min(dm,math.hypot(dx,dy))
        if abs(p[0]-bx)<BH and abs(p[1]-by)<BH: pen=max(pen,min(BH-abs(p[0]-bx),BH-abs(p[1]-by)))
    return dm,pen
mind=1e9; maxpen=0; maxdiv=0; n=0; rows=[]
for t,((rx,ry,ryaw),(mx,my,_)) in sorted(gt.items()):
    mx_w=0.5+mx
    dm,pen=rect_clear(rx,ry,ryaw,mx_w,my)
    mind=min(mind,dm); maxpen=max(maxpen,pen)
    r=tr.get(t)
    if r:
        ox=float(r['robot_x'])-4.5; oy=float(r['robot_y'])
        div=math.hypot(ox-rx,oy-ry); maxdiv=max(maxdiv,div); n+=1
        rows.append((t,round(rx,3),round(ry,3),round(ox,3),round(oy,3),round(my,3),round(dm,3),round(pen,3)))
print('gt samples',len(gt),'matched to trace',n)
print('GROUND-TRUTH min body-box distance %.3f m, max penetration %.3f m'%(mind,maxpen))
print('max |odom - gazebo truth| position diff %.3f m'%maxdiv)
json.dump(rows,open(f'{D}/gt_vs_odom.json','w'))
for r in rows[::max(1,len(rows)//12)]: print(r)
