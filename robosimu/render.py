"""3D software renderer for the arm and its trajectory."""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .kinematics import forward_kinematics

FONT = Path(r"C:\Windows\Fonts\arial.ttf")
INK = (30, 30, 30)
GREY = (130, 130, 130)
TEAL = (62, 124, 123)
TERRA = (180, 85, 63)
OCHRE = (201, 154, 60)
SLATE = (79, 93, 107)


def font(size: int, bold: bool = False):
    name = "arialbd.ttf" if bold else "arial.ttf"
    try:
        return ImageFont.truetype(str(FONT.parent / name), size)
    except Exception:
        return ImageFont.load_default()


def joint_points(model, q):
    _, frames = forward_kinematics(model, q)
    return [frames[i][:3, 3].copy() for i in range(model.n + 1)]


class View:
    def __init__(self, box, points, yaw=math.radians(35), pitch=math.radians(22), margin=0.78):
        self.yaw, self.pitch = yaw, pitch
        xs, ys = zip(*(self.raw(p) for p in points))
        x0, y0, x1, y1 = box
        w, h = x1 - x0, y1 - y0
        dx, dy = max(max(xs) - min(xs), 1e-6), max(max(ys) - min(ys), 1e-6)
        self.s = min(w / dx, h / dy) * margin
        self.cx = (x0 + x1) / 2 - self.s * (min(xs) + max(xs)) / 2
        self.cy = (y0 + y1) / 2 - self.s * (min(ys) + max(ys)) / 2

    def raw(self, p):
        x, y, z = p
        xr = math.cos(self.yaw) * x - math.sin(self.yaw) * y
        yr = math.sin(self.yaw) * x + math.cos(self.yaw) * y
        zr = math.cos(self.pitch) * z - math.sin(self.pitch) * yr
        return xr, -zr

    def p(self, point):
        x, y = self.raw(point)
        return (self.cx + self.s * x, self.cy + self.s * y)


def draw_arm(img, box, model, q, target, trail=None, title=None, subtitle=None):
    d = ImageDraw.Draw(img)
    pts = joint_points(model, q)
    view = View(box, pts + [target], margin=0.72)

    # ground grid
    for v in range(-1000, 1001, 250):
        a = view.p((v / 1000, -1.0, 0.0))
        b = view.p((v / 1000, 1.0, 0.0))
        d.line([a, b], fill=(236, 236, 232), width=1)
        a = view.p((-1.0, v / 1000, 0.0))
        b = view.p((1.0, v / 1000, 0.0))
        d.line([a, b], fill=(236, 236, 232), width=1)

    # base
    base = view.p((0, 0, 0))
    d.ellipse([base[0] - 26, base[1] - 10, base[0] + 26, base[1] + 18],
              fill=(210, 205, 195), outline=(90, 90, 90), width=2)

    # trail
    if trail:
        for a, b in zip(trail[:-1], trail[1:]):
            d.line([view.p(a), view.p(b)], fill=(190, 205, 220), width=3)

    # links
    widths = [18, 15, 13, 9, 8, 7]
    for i in range(model.n):
        a, b = view.p(pts[i]), view.p(pts[i + 1])
        d.line([a, b], fill=SLATE, width=widths[min(i, len(widths) - 1)])
        d.line([a, b], fill=(120, 135, 150), width=max(2, widths[min(i, len(widths) - 1)] - 8))
    for i, p in enumerate(pts):
        x, y = view.p(p)
        r = 14 if i == 0 else 10
        d.ellipse([x - r, y - r, x + r, y + r], fill=OCHRE, outline=(80, 60, 20), width=2)

    # tool and target
    tool = view.p(pts[-1])
    tg = view.p(target)
    d.line([tool, tg], fill=TERRA, width=3)
    d.ellipse([tg[0] - 9, tg[1] - 9, tg[0] + 9, tg[1] + 9], outline=(60, 140, 60), width=3)
    d.ellipse([tg[0] - 3, tg[1] - 3, tg[0] + 3, tg[1] + 3], fill=(60, 140, 60))

    if title:
        d.text((box[0] + 6, box[1] + 2), title, font=font(22, True), fill=INK)
    if subtitle:
        d.text((box[0] + 6, box[1] + 30), subtitle, font=font(18), fill=(80, 80, 80))
    return view


def render_frame(model, q, target, size=520, trail=None, title=None, subtitle=None):
    img = Image.new("RGB", (size, size), (250, 250, 248))
    draw_arm(img, (20, 50, size - 10, size - 20), model, q, target, trail, title, subtitle)
    return img


def save_gif(model, result, path, every=2, size=520):
    frames = []
    trail = []
    for i in range(0, len(result.q), every):
        q = result.q[i]
        p = result.p[i]
        trail.append(p.copy())
        frames.append(render_frame(
            model, q, result.target, size=size,
            trail=trail[-200:],
            title=f"{result.controller}  t={result.t[i]:.2f} s",
            subtitle=f"|e| = {result.error[i]*1000:.1f} mm   tau_peak = {np.max(np.abs(result.tau[i])):.1f} Nm",
        ))
    if not frames:
        return
    frames[0].save(path, save_all=True, append_images=frames[1:],
                   duration=int(1000 * result.config.control_dt * every), loop=0, optimize=True)


def save_figure(model, result, path):
    W, H = 1800, 620
    img = Image.new("RGB", (W, H), (255, 255, 255))
    d = ImageDraw.Draw(img)
    idx = [0, len(result.q) // 2, len(result.q) - 1]
    labels = ["start", "mid-reach", "end"]
    for k, (i, lab) in enumerate(zip(idx, labels)):
        box = (20 + k * 580, 30, 580 + k * 580, 600)
        draw_arm(img, box, model, result.q[i], result.target,
                 trail=[tuple(p) for p in result.p[:i + 1]],
                 title=f"({chr(97 + k)}) {lab}, t={result.t[i]:.2f} s",
                 subtitle=f"|e| = {result.error[i]*1000:.1f} mm")
    img.save(path, dpi=(300, 300))
    return path


def save_error_plot(results, path, size=(760, 420)):
    """Cartesian error against time for two or more runs."""
    img = Image.new("RGB", size, (255, 255, 255))
    d = ImageDraw.Draw(img)
    x0, y0, x1, y1 = 80, 30, size[0] - 30, size[1] - 70
    d.rectangle([x0, y0, x1, y1], outline=INK, width=2)
    tmax = max(float(r.t[-1]) for r in results)
    emax = max(float(np.max(r.error)) for r in results) * 1000.0
    colors = [SLATE, TERRA, TEAL]
    for k, r in enumerate(results):
        col = colors[k % len(colors)]
        pts = [(x0 + (float(t) / tmax) * (x1 - x0),
                y1 - (float(e) * 1000.0 / emax) * (y1 - y0)) for t, e in zip(r.t, r.error)]
        d.line(pts, fill=col, width=4)
        d.text((x1 - 160, y0 + 10 + 24 * k), r.controller, font=font(18, True), fill=col)
    for k in range(5):
        frac = k / 4
        y = y1 - frac * (y1 - y0)
        d.text((x0 - 60, y - 10), f"{frac*emax:5.1f}", font=font(16), fill=GREY)
    d.text((x0, y1 + 16), "time (s)", font=font(18), fill=INK)
    d.text((x0, y0 - 22), "Cartesian error (mm)", font=font(18), fill=INK)
    d.text((x1 - 60, y1 + 16), f"{tmax:.1f}", font=font(16), fill=GREY)
    img.save(path, dpi=(300, 300))
    return path


def save_html(model, result, path, every=2):
    """Write a self-contained interactive HTML viewer.

    The file embeds the model DH parameters and a subsampled trajectory; the
    browser recomputes forward kinematics, so the six joint sliders move the
    arm live and the trajectory can be scrubbed or played back.
    """
    idx = list(range(0, len(result.q), every))
    if idx[-1] != len(result.q) - 1:
        idx.append(len(result.q) - 1)
    data = {
        "name": model.name,
        "links": [
            {
                "name": l.name,
                "a": l.a,
                "alpha_deg": math.degrees(l.alpha),
                "d": l.d,
                "theta_offset_deg": math.degrees(l.theta_offset),
                "limits_deg": [math.degrees(l.limits[0]), math.degrees(l.limits[1])],
            }
            for l in model.links
        ],
        "q": [result.q[i].tolist() for i in idx],
        "p": [result.p[i].tolist() for i in idx],
        "t": [float(result.t[i]) for i in idx],
        "error": [float(result.error[i]) for i in idx],
        "tau": [result.tau[i].tolist() for i in idx],
        "power": [float(result.power[i]) for i in idx],
        "target": result.target.tolist(),
        "p0": result.p0.tolist(),
        "controller": result.controller,
        "energy": float(result.energy),
    }
    html = _HTML_TEMPLATE.replace("__TITLE__", f"{model.name} — {result.controller}")
    html = html.replace("/*__DATA__*/", json.dumps(data))
    Path(path).write_text(html, encoding="utf-8")
    return path


_HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>robosimu — __TITLE__</title>
<style>
  body { font-family: Arial, Helvetica, sans-serif; background:#f4f4f2; margin:0; padding:16px; color:#222; }
  h1 { font-size:18px; margin:0 0 10px; font-weight:bold; }
  .wrap { display:flex; gap:16px; align-items:flex-start; }
  canvas { background:#fafaf8; border:1px solid #ccc; border-radius:6px; }
  .panel { width:320px; background:#fff; border:1px solid #ddd; border-radius:6px; padding:12px; }
  .row { margin:8px 0; font-size:12px; }
  .row label { display:block; margin-bottom:2px; color:#444; }
  .val { float:right; color:#666; }
  input[type=range] { width:100%; }
  button { padding:6px 12px; margin:2px 4px 2px 0; cursor:pointer; }
  #info { font-family: Consolas, monospace; font-size:12px; line-height:1.6; color:#333; }
</style>
</head>
<body>
<h1>robosimu — __TITLE__</h1>
<div class="wrap">
  <canvas id="cv" width="920" height="640"></canvas>
  <div class="panel">
    <div class="row">
      <button id="play">&#9654; play</button>
      <button id="fit">fit view</button>
    </div>
    <div class="row"><label>trajectory scrub</label>
      <input id="scrub" type="range" min="0" max="100" value="0" step="1"></div>
    <div class="row" style="margin-top:10px"><b>joint sliders (live forward kinematics)</b></div>
    <div id="sliders"></div>
    <div class="row" id="info" style="margin-top:10px"></div>
  </div>
</div>
<script>
const D = /*__DATA__*/;
const cv = document.getElementById('cv'), ctx = cv.getContext('2d');
const INK='#1e1e1e', SLATE='#4f5d6b', OCHRE='#c99a3c', TERRA='#b4553f', GRID='#ecece8';

function dhT(a, alpha, d, th){
  const ct=Math.cos(th), st=Math.sin(th), ca=Math.cos(alpha), sa=Math.sin(alpha);
  return [[ct,-st*ca,st*sa,a*ct],[st,ct*ca,-ct*sa,a*st],[0,sa,ca,d],[0,0,0,1]];
}
function mm(A,B){
  const C=Array.from({length:4},()=>[0,0,0,0]);
  for(let i=0;i<4;i++)for(let j=0;j<4;j++){let s=0;for(let k=0;k<4;k++)s+=A[i][k]*B[k][j];C[i][j]=s;}
  return C;
}
const I4=[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]];
function fk(q){
  let T=I4.map(r=>r.slice());
  const org=[[0,0,0]];
  for(let i=0;i<D.links.length;i++){
    const L=D.links[i];
    T=mm(T, dhT(L.a, L.alpha_deg*Math.PI/180, L.d, q[i]+L.theta_offset_deg*Math.PI/180));
    org.push([T[0][3],T[1][3],T[2][3]]);
  }
  return org;
}
const yaw=35*Math.PI/180, pitch=22*Math.PI/180;
function raw(p){
  const x=p[0],y=p[1],z=p[2];
  const xr=Math.cos(yaw)*x-Math.sin(yaw)*y;
  const yr=Math.sin(yaw)*x+Math.cos(yaw)*y;
  const zr=Math.cos(pitch)*z-Math.sin(pitch)*yr;
  return [xr,-zr];
}
let view=null;
function computeView(pts){
  const rp=pts.map(raw);
  const xs=rp.map(p=>p[0]), ys=rp.map(p=>p[1]);
  const x0=Math.min.apply(null,xs), x1=Math.max.apply(null,xs);
  const y0=Math.min.apply(null,ys), y1=Math.max.apply(null,ys);
  const box=[40,50,cv.width-40,cv.height-40];
  const w=box[2]-box[0], h=box[3]-box[1];
  const dx=Math.max(x1-x0,1e-6), dy=Math.max(y1-y0,1e-6);
  const s=Math.min(w/dx,h/dy)*0.72;
  return {s:s, cx:(box[0]+box[2])/2 - s*(x0+x1)/2, cy:(box[1]+box[3])/2 - s*(y0+y1)/2};
}
function proj(p){ const r=raw(p); return [view.cx+view.s*r[0], view.cy+view.s*r[1]]; }

function draw(q, frameIdx){
  ctx.fillStyle='#fafaf8'; ctx.fillRect(0,0,cv.width,cv.height);
  const org=fk(q);
  ctx.strokeStyle=GRID; ctx.lineWidth=1;
  for(let v=-1000;v<=1000;v+=250){
    let a=proj([v/1000,-1,0]), b=proj([v/1000,1,0]);
    ctx.beginPath();ctx.moveTo(a[0],a[1]);ctx.lineTo(b[0],b[1]);ctx.stroke();
    let c=proj([-1,v/1000,0]), d=proj([1,v/1000,0]);
    ctx.beginPath();ctx.moveTo(c[0],c[1]);ctx.lineTo(d[0],d[1]);ctx.stroke();
  }
  const base=proj([0,0,0]);
  ctx.fillStyle='#d2cdc3'; ctx.strokeStyle='#5a5a5a'; ctx.lineWidth=2;
  ctx.beginPath();ctx.ellipse(base[0],base[1],26,12,0,0,Math.PI*2);ctx.fill();ctx.stroke();
  if(frameIdx>0){
    ctx.strokeStyle='#becdd9'; ctx.lineWidth=3; ctx.beginPath();
    for(let k=0;k<=frameIdx;k++){const p=proj(D.p[k]); if(k)ctx.lineTo(p[0],p[1]);else ctx.moveTo(p[0],p[1]);}
    ctx.stroke();
  }
  const widths=[18,15,13,9,8,7];
  for(let i=0;i<D.links.length;i++){
    const a=proj(org[i]), b=proj(org[i+1]);
    const wdt=widths[Math.min(i,widths.length-1)];
    ctx.strokeStyle=SLATE; ctx.lineWidth=wdt;
    ctx.beginPath();ctx.moveTo(a[0],a[1]);ctx.lineTo(b[0],b[1]);ctx.stroke();
    ctx.strokeStyle='#788796'; ctx.lineWidth=Math.max(2,wdt-8);
    ctx.beginPath();ctx.moveTo(a[0],a[1]);ctx.lineTo(b[0],b[1]);ctx.stroke();
  }
  for(let i=0;i<org.length;i++){
    const p=proj(org[i]); const r=i?10:14;
    ctx.fillStyle=OCHRE; ctx.strokeStyle='#503c14'; ctx.lineWidth=2;
    ctx.beginPath();ctx.arc(p[0],p[1],r,0,Math.PI*2);ctx.fill();ctx.stroke();
  }
  const t=proj(D.target);
  ctx.strokeStyle='#3c8c3c'; ctx.lineWidth=3;
  ctx.beginPath();ctx.arc(t[0],t[1],9,0,Math.PI*2);ctx.stroke();
  ctx.fillStyle='#3c8c3c'; ctx.beginPath();ctx.arc(t[0],t[1],3,0,Math.PI*2);ctx.fill();
  const tool=proj(org[org.length-1]);
  ctx.strokeStyle=TERRA; ctx.lineWidth=3;
  ctx.beginPath();ctx.moveTo(tool[0],tool[1]);ctx.lineTo(t[0],t[1]);ctx.stroke();
  ctx.fillStyle=INK; ctx.font='16px Arial';
  if(frameIdx>=0){
    const e=D.error[frameIdx]*1000;
    const tau=Math.max.apply(null,D.tau[frameIdx].map(Math.abs));
    ctx.fillText('t = '+D.t[frameIdx].toFixed(2)+' s    |e| = '+e.toFixed(1)+' mm    tau_peak = '+tau.toFixed(1)+' Nm', 16, 26);
    ctx.fillText(D.controller+'    energy = '+D.energy.toFixed(1)+' J', 16, 48);
  } else {
    ctx.fillText('custom pose (drag the joint sliders)', 16, 26);
  }
}

const sliders=document.getElementById('sliders');
D.links.forEach(function(L,i){
  const row=document.createElement('div'); row.className='row';
  row.innerHTML='<label>'+L.name+' <span class="val" id="v'+i+'"></span></label>'+
    '<input type="range" id="s'+i+'" min="'+L.limits_deg[0]+'" max="'+L.limits_deg[1]+'" step="0.1">';
  sliders.appendChild(row);
});
const inputs=D.links.map(function(_,i){return document.getElementById('s'+i);});
function setQ(q){
  for(let i=0;i<inputs.length;i++){
    inputs[i].value=q[i]*180/Math.PI;
    document.getElementById('v'+i).textContent=(q[i]*180/Math.PI).toFixed(1)+' deg';
  }
}
function readQ(){
  return inputs.map(function(inp){return parseFloat(inp.value)*Math.PI/180;});
}
function info(i){
  if(i<0){ document.getElementById('info').innerHTML='custom pose'; return; }
  const tau=D.tau[i].map(function(x,j){return 'j'+(j+1)+': '+x.toFixed(1);}).join(' &nbsp; ');
  document.getElementById('info').innerHTML='joint torque (Nm): '+tau+'<br>electrical power: '+D.power[i].toFixed(1)+' W';
}
let idx=0, playing=false, timer=null;
function show(i){ idx=i; setQ(D.q[i]); draw(D.q[i], i); info(i); document.getElementById('scrub').value=i; }
function step(){ if(idx>=D.q.length-1){ pause(); return; } show(idx+1); }
function play(){ if(playing)return; playing=true; document.getElementById('play').innerHTML='&#10074;&#10074; pause'; timer=setInterval(step,40); }
function pause(){ playing=false; document.getElementById('play').innerHTML='&#9654; play'; if(timer)clearInterval(timer); timer=null; }
document.getElementById('play').addEventListener('click',function(){ playing?pause():play(); });
document.getElementById('fit').addEventListener('click',function(){ view=computeView(D.p.concat([D.target])); show(idx); });
document.getElementById('scrub').addEventListener('input',function(e){ pause(); show(parseInt(e.target.value,10)); });
inputs.forEach(function(inp,i){
  inp.addEventListener('input',function(){
    pause(); const q=readQ(); draw(q,-1); info(-1);
  });
});
view=computeView(D.p.concat([D.target]));
show(0);
</script>
</body>
</html>
"""
