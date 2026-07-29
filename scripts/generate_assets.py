#!/usr/bin/env python3
from pathlib import Path
import csv, math, yaml
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.interpolate import splprep, splev
from shapely.geometry import LineString, Point
from shapely.affinity import translate
import cv2

ROOT = Path(__file__).resolve().parents[1]
TRACK = ROOT/'track'; CONFIG=ROOT/'config'; SIGNS=ROOT/'signs'
for p in (TRACK,CONFIG,SIGNS): p.mkdir(parents=True,exist_ok=True)

RES=0.05
W_M,H_M=30.0,24.0
W,H=int(W_M/RES),int(H_M/RES)
# Smooth closed centreline control points, designed to include straight, chicane and hairpin-like ends.
ctrl=np.array([
 [3.0,4.0],[8.0,3.0],[14.0,3.2],[22.0,3.0],[27.0,6.0],
 [26.0,11.0],[22.0,13.0],[25.5,18.0],[21.0,21.0],[14.5,20.0],
 [10.5,17.0],[7.0,20.0],[3.0,18.0],[4.0,13.0],[7.5,11.0],[3.0,8.0]
],dtype=float)
tck,_=splprep([ctrl[:,0],ctrl[:,1]],s=1.2,per=True,k=3)
u=np.linspace(0,1,900,endpoint=False)
x,y=splev(u,tck)
pts=np.c_[x,y]
line=LineString(pts.tolist()+[pts[0].tolist()])
track_poly=line.buffer(1.15,cap_style=1,join_style=1)
inner=line.buffer(0.12)

# occupancy map: free=254, occupied=0, unknown not used
arr=np.zeros((H,W),dtype=np.uint8)
for py in range(H):
    ym=(H-1-py)*RES
    for px in range(W):
        xm=px*RES
        if track_poly.contains(Point(xm,ym)):
            arr[py,px]=254
Image.fromarray(arr).save(TRACK/'mushr_training_track.pgm')

with open(TRACK/'mushr_training_track.yaml','w') as f:
    f.write('image: mushr_training_track.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')

# waypoints spaced approximately every 0.25 m
coords=list(line.coords)
dists=[0.0]
for a,b in zip(coords[:-1],coords[1:]): dists.append(dists[-1]+math.dist(a,b))
length=line.length
samples=np.arange(0,length,0.25)
way=[]
for i,s in enumerate(samples):
    p=line.interpolate(float(s)); p2=line.interpolate(float((s+0.05)%length))
    yaw=math.atan2(p2.y-p.y,p2.x-p.x)
    way.append((i,p.x,p.y,yaw))
with open(TRACK/'centerline_waypoints.csv','w',newline='') as f:
    w=csv.writer(f); w.writerow(['id','x','y','yaw']); w.writerows(way)

# Named sectors for instructor / scoring
sectors={
 'start_finish': {'x':3.4,'y':4.0,'yaw':0.0},
 'long_straight': {'from_waypoint':8,'to_waypoint':54},
 'east_hairpin': {'from_waypoint':62,'to_waypoint':96},
 'upper_chicane': {'from_waypoint':115,'to_waypoint':158},
 'west_complex': {'from_waypoint':190,'to_waypoint':245}
}
with open(CONFIG/'track_sectors.yaml','w') as f: yaml.safe_dump(sectors,f,sort_keys=False)

obstacle_sets={
 'development': [
  {'name':'straight_box','type':'box','x':13.2,'y':3.65,'yaw':0.0,'size':[0.60,0.45]},
  {'name':'corner_barrel','type':'cylinder','x':25.2,'y':11.8,'yaw':0.0,'radius':0.28},
  {'name':'gate_left','type':'box','x':17.0,'y':20.55,'yaw':0.15,'size':[0.45,0.35]},
  {'name':'gate_right','type':'box','x':17.15,'y':19.35,'yaw':0.15,'size':[0.45,0.35]},
 ],
 'evaluation_A': [
  {'name':'box_A1','type':'box','x':18.8,'y':3.25,'yaw':0.05,'size':[0.55,0.45]},
  {'name':'barrel_A2','type':'cylinder','x':23.3,'y':18.5,'yaw':0.0,'radius':0.30},
 ],
 'evaluation_B': [
  {'name':'box_B1','type':'box','x':5.5,'y':12.2,'yaw':-0.4,'size':[0.55,0.40]},
  {'name':'box_B2','type':'box','x':11.2,'y':19.0,'yaw':0.2,'size':[0.50,0.40]},
 ],
 'evaluation_C': [
  {'name':'barrel_C1','type':'cylinder','x':26.0,'y':8.4,'yaw':0.0,'radius':0.30},
  {'name':'box_C2','type':'box','x':7.1,'y':4.0,'yaw':0.0,'size':[0.65,0.40]},
 ]
}
with open(CONFIG/'obstacle_sets.yaml','w') as f: yaml.safe_dump(obstacle_sets,f,sort_keys=False)

sign_layouts={
 'development': [
  {'marker_id':10,'meaning':'SLOW','x':24.4,'y':7.5,'yaw':2.6,'speed_limit':1.0},
  {'marker_id':20,'meaning':'NORMAL','x':22.0,'y':14.0,'yaw':-1.8,'speed_limit':1.8},
  {'marker_id':30,'meaning':'BOOST','x':10.5,'y':3.9,'yaw':1.57,'speed_limit':2.5},
 ],
 'evaluation_A': [
  {'marker_id':10,'meaning':'SLOW','x':20.8,'y':19.0,'yaw':-1.3,'speed_limit':1.0},
  {'marker_id':30,'meaning':'BOOST','x':8.0,'y':3.6,'yaw':1.57,'speed_limit':2.5},
 ],
 'evaluation_B': [
  {'marker_id':10,'meaning':'SLOW','x':5.2,'y':16.0,'yaw':0.2,'speed_limit':1.0},
  {'marker_id':20,'meaning':'NORMAL','x':15.5,'y':20.8,'yaw':-1.57,'speed_limit':1.8},
 ],
 'evaluation_C': [
  {'marker_id':30,'meaning':'BOOST','x':14.0,'y':3.9,'yaw':1.57,'speed_limit':2.5},
  {'marker_id':10,'meaning':'SLOW','x':25.0,'y':12.8,'yaw':-2.4,'speed_limit':1.0},
 ]
}
with open(CONFIG/'vision_sign_layouts.yaml','w') as f: yaml.safe_dump(sign_layouts,f,sort_keys=False)

# ArUco sign boards
aruco=cv2.aruco
D=aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
colors={10:(245,191,66),20:(72,180,95),30:(63,137,230)}
labels={10:'SLOW',20:'NORMAL',30:'BOOST'}
for mid in (10,20,30):
    marker=aruco.generateImageMarker(D,mid,360)
    board=Image.new('RGB',(520,620),'white'); draw=ImageDraw.Draw(board)
    draw.rectangle((0,0,519,619),outline='black',width=10)
    draw.rectangle((0,0,519,100),fill=colors[mid])
    draw.text((260,50),labels[mid],anchor='mm',fill='black',font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',48))
    board.paste(Image.fromarray(marker).convert('RGB'),(80,155))
    draw.text((260,555),f'MARKER {mid}',anchor='mm',fill='black',font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',30))
    board.save(SIGNS/f'aruco_{mid}_{labels[mid].lower()}.png')

# Preview
scale=32
pw,ph=int(W_M*scale),int(H_M*scale)
preview=Image.new('RGB',(pw,ph),(42,46,52)); d=ImageDraw.Draw(preview)
def pix(p): return (int(p[0]*scale),int((H_M-p[1])*scale))
# draw track wide then centerline
ppts=[pix(p) for p in pts]
d.line(ppts+[ppts[0]],fill=(220,220,220),width=int(2.3*scale),joint='curve')
d.line(ppts+[ppts[0]],fill=(70,70,75),width=int(1.95*scale),joint='curve')
# dashed centerline
for i in range(0,len(ppts)-6,12): d.line(ppts[i:i+6],fill=(235,210,80),width=2)
# obstacles
for o in obstacle_sets['development']:
    cx,cy=pix((o['x'],o['y']))
    if o['type']=='cylinder':
        r=int(o['radius']*scale); d.ellipse((cx-r,cy-r,cx+r,cy+r),fill=(210,70,55),outline='white',width=2)
    else:
        sx,sy=o['size']; d.rectangle((cx-int(sx*scale/2),cy-int(sy*scale/2),cx+int(sx*scale/2),cy+int(sy*scale/2)),fill=(210,70,55),outline='white',width=2)
# signs
for s in sign_layouts['development']:
    cx,cy=pix((s['x'],s['y'])); col=colors[s['marker_id']]
    d.rectangle((cx-10,cy-10,cx+10,cy+10),fill=col,outline='black',width=2)
    d.text((cx+14,cy-8),labels[s['marker_id']],fill='white',font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',16))
# start line
sx,sy=pix((3.4,4.0)); d.line((sx,sy-35,sx,sy+35),fill='white',width=5)
d.text((sx+8,sy-55),'START / FINISH',fill='white',font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',18))
preview.save(TRACK/'training_track_preview.png')
print('Generated assets in',ROOT)
