// Week 14 · Radar SLAM: FMCW ranging and Doppler, CA-CFAR, Doppler ego-velocity, a radar Scan Context.
const C0=3e8, FM=77e9, LAM=C0/FM, TC=40e-6, NS=128;   // speed of light, start frequency, wavelength, chirp time, complex samples per chirp
const lgd=items=>h("div",{class:"legend"},...items.map(([c,t])=>h("span",{},h("i",{style:"background:"+c}),t)));
const pl=(pts,a)=>sv("path",Object.assign({d:pts.map((p,i)=>(i?"L":"M")+p[0].toFixed(1)+" "+p[1].toFixed(1)).join(" "),fill:"none","stroke-width":2,"stroke-linejoin":"round"},a));

// ---- Day 1: FMCW. Beat signal of chirp m for targets [[range, amplitude]] with radial speed v (positive = receding).
// phase = 2*pi*f0*t + 4*pi*d(t)/lambda, with f0*t = (d/dr)*(n/NS) and d(t) = d + v*t
const fmcwChirp=(tg,v,B,m,r)=>{ const dr=C0/(2*B), re=new Float64Array(NS), im=new Float64Array(NS);
  for(let n=0;n<NS;n++){ let a=r.n()*0.15, b=r.n()*0.15;
    for(const [d,amp] of tg){ const ph=2*Math.PI*(d/dr)*n/NS+4*Math.PI*(d+v*(m*TC+n*TC/NS))/LAM; a+=amp*Math.cos(ph); b+=amp*Math.sin(ph); }
    re[n]=a; im[n]=b; }
  return {re,im}; };
// discrete-time Fourier transform of one chirp, evaluated at the beat frequency that belongs to range rg
const fmcwAt=(s,B,rg)=>{ const w=2*Math.PI*(rg/(C0/(2*B)))/NS; let a=0,b=0;
  for(let n=0;n<NS;n++){ const c=Math.cos(w*n), q=Math.sin(w*n); a+=s.re[n]*c+s.im[n]*q; b+=s.im[n]*c-s.re[n]*q; }
  return [a/NS,b/NS]; };

VIZ.w14_fmcw=el=>{
  let d=12, v=5, B=300e6, two=false, seed=1;
  const W=600, H=396, G=320, RMAX=32, K=100, X0=56, X1=584;
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Three stacked plots: transmitted and received chirp frequency against time, the beat signal over one chirp, and the spectrum of the beat signal on a range axis with its peak marked"});
  const st=h("div",{class:"stats"});
  const draw=()=>{ const r=rng(seed), tg=two?[[d,1],[d+1.5,0.8]]:[[d,1]], dr=C0/(2*B), s0=fmcwChirp(tg,v,B,0,r), s1=fmcwChirp(tg,v,B,1,r);
    const mag=[]; for(let g=0;g<G;g++){ const z=fmcwAt(s0,B,g*RMAX/G); mag.push(Math.hypot(z[0],z[1])); }
    let k=0; for(let g=1;g<G;g++) if(mag[g]>mag[k]) k=g;
    let p=0; if(k>0&&k<G-1){ const den=mag[k-1]-2*mag[k]+mag[k+1]; if(den<0) p=0.5*(mag[k-1]-mag[k+1])/den; }
    const rh=(k+p)*RMAX/G, a0=fmcwAt(s0,B,rh), a1=fmcwAt(s1,B,rh), dphi=Math.atan2(a1[1]*a0[0]-a1[0]*a0[1],a1[0]*a0[0]+a1[1]*a0[1]), vh=LAM*dphi/(4*Math.PI*TC), vmax=LAM/(4*TC);
    svg.innerHTML="";
    // panel A: frequency against time for two chirps; the echo delay is drawn K times too long so it can be seen
    const tx=t=>X0+(X1-X0)*t/(2*TC), fy=f=>112-84*f, tau=K*2*d/C0, tm=0.62*TC;
    svg.append(txt(X0,16,"frequency against time, two chirps (echo delay drawn ×"+K+")",{class:"mut","font-size":12}),
      txt(X0-6,32,"fₘ+B",{"text-anchor":"end","font-size":12,class:"mut"}),txt(X0-6,116,"fₘ",{"text-anchor":"end","font-size":12,class:"mut"}),
      sv("line",{x1:X0,y1:112,x2:X1,y2:112,stroke:"var(--line2)"}),
      pl([[tx(0),fy(0)],[tx(TC),fy(1)],[tx(TC),fy(0)],[tx(2*TC),fy(1)]],{stroke:"var(--c5)"}),
      pl([[tx(tau),fy(0)],[tx(TC+tau),fy(1)]],{stroke:"var(--c2)"}),pl([[tx(TC+tau),fy(0)],[tx(2*TC),fy(1-tau/TC)]],{stroke:"var(--c2)"}),
      sv("line",{x1:tx(tm),y1:fy(tm/TC),x2:tx(tm),y2:fy((tm-tau)/TC),stroke:"var(--ink)","stroke-width":2}),txt(tx(tm)+6,fy((tm-tau/2)/TC)+14,"f₀",{"font-size":13}),
      txt(tx(TC),128,"T",{"text-anchor":"middle","font-size":12,class:"mut"}),txt(tx(2*TC),128,"2T",{"text-anchor":"end","font-size":12,class:"mut"}));
    // panel B: real part of the beat signal of the first chirp
    svg.append(txt(X0,150,"beat (IF) signal over one chirp, real part, "+NS+" samples",{class:"mut","font-size":12}),
      sv("line",{x1:X0,y1:196,x2:X1,y2:196,stroke:"var(--line2)"}),pl(Array.from(s0.re,(y,n)=>[X0+(X1-X0)*n/(NS-1),196-16*Math.max(-2.3,Math.min(2.3,y))]),{stroke:"var(--c4)","stroke-width":1.5}));
    // panel C: spectrum on a range axis
    const rx=x=>X0+(X1-X0)*x/RMAX, mx=Math.max(...mag,1e-9), my=m=>372-90*m/mx;
    svg.append(txt(X0,262,"spectrum of the beat signal on a range axis [m], using d = c f₀ / (2S)",{class:"mut","font-size":12}),sv("line",{x1:X0,y1:372,x2:X1,y2:372,stroke:"var(--line2)"}));
    for(let x=0;x<=RMAX;x+=4) svg.append(sv("line",{x1:rx(x),y1:372,x2:rx(x),y2:377,stroke:"var(--line2)"}),txt(rx(x),390,String(x),{"text-anchor":"middle","font-size":12,class:"mut"}));
    tg.forEach(t=>{ if(t[0]<=RMAX) svg.append(sv("line",{x1:rx(t[0]),y1:372,x2:rx(t[0]),y2:278,stroke:"var(--muted)","stroke-dasharray":"3 4"})); });
    svg.append(pl(mag.map((m,g)=>[rx(g*RMAX/G),my(m)]),{stroke:"var(--c1)"}),sv("circle",{cx:rx(rh),cy:my(mag[k]),r:4.5,fill:"var(--c1)",stroke:"var(--surface)","stroke-width":1.5}),
      txt(rh>24?rx(rh)-8:rx(rh)+8,Math.max(286,my(mag[k])+4),"peak "+rh.toFixed(2)+" m",{"text-anchor":rh>24?"end":"start","font-size":12}));
    st.innerHTML=""; st.append(stat("Beat frequency f₀",(2*B*d/(TC*C0)/1e3).toFixed(0)+" kHz","2Bd/(Tc), first target"),stat("Size of a range bin",dr.toFixed(2)+" m","c/(2B); unambiguous to "+(NS*dr).toFixed(0)+" m"),
      stat("Range from the peak",rh.toFixed(2)+" m","true "+d.toFixed(1)+" m"),stat("Phase step between chirps",(dphi*180/Math.PI).toFixed(1)+"°","4πvT/λ = "+(4*Math.PI*v*TC/LAM*180/Math.PI).toFixed(1)+"°"),
      stat("Velocity from the phase",vh.toFixed(2)+" m/s",Math.abs(v)>=vmax?"aliased: |v| is above "+vmax.toFixed(1)+" m/s":"true "+v.toFixed(1)+" m/s")); };
  const tb=button("Add a second target 1.5 m behind",()=>{ two=!two; tb.classList.toggle("on",two); draw(); });
  el.append(h("div",{class:"vt"},"FMCW radar: from a chirp to a range and a velocity"),h("p",{class:"vd"},"A 77 GHz radar sends two 40 µs chirps and takes 128 complex samples of the beat signal in each. Range comes from the peak of the spectrum, velocity from the phase change between the chirps."),
    h("div",{class:"ctl"},range("Target range d",2,30,0.1,d,x=>{ d=x; draw(); },x=>x.toFixed(1)+" m").el,range("Radial velocity v",-30,30,0.5,v,x=>{ v=x; draw(); },x=>x.toFixed(1)+" m/s").el,range("Bandwidth B",50,600,10,B/1e6,x=>{ B=x*1e6; draw(); },x=>x+" MHz").el),
    h("div",{class:"ctl"},tb,button("New noise",()=>{ seed++; draw(); })),svg,lgd([["var(--c5)","transmitted chirp"],["var(--c2)","received echo"],["var(--c4)","beat signal"],["var(--c1)","spectrum"]]),st);
  draw();
};

// ---- Day 2: cell-averaging CFAR on one azimuth of a radargram
const CF_T=[[30,15],[62,14],[68,18],[135,21],[185,13]];        // [cell, target power in dB above the quiet noise level]
const cfarAlpha=(n,pfa)=>n*(Math.pow(pfa,-1/n)-1);             // threshold multiplier for n training cells and exponential noise power
const cfarProfile=seed=>{ const r=rng(seed), x=[];
  for(let i=0;i<200;i++){ const mu=i>=110&&i<165?8:1; let p=0;
    for(const [c,db] of CF_T){ const k=Math.abs(i-c); if(k<=1) p+=Math.pow(10,db/10)*(k?0.3:1); }
    const a=Math.sqrt(p)+r.n()*Math.sqrt(mu/2), b=r.n()*Math.sqrt(mu/2); x.push(a*a+b*b); }
  return x; };
const cfarThr=(x,nt,ng,pfa)=>x.map((_,i)=>{ let s=0,c=0;
  for(let k=ng+1;k<=ng+nt;k++){ if(i-k>=0){ s+=x[i-k]; c++; } if(i+k<x.length){ s+=x[i+k]; c++; } }
  return cfarAlpha(c,pfa)*s/c; });

VIZ.w14_cfar=el=>{
  let nt=10, ng=2, lp=-3, seed=4;
  const W=600, H=300, db=p=>10*Math.log10(Math.max(p,1e-3)), X=i=>46+i*540/199, Y=q=>262-242*(Math.max(-12,Math.min(27,q))+12)/39;
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Power against range along one radar azimuth, with the adaptive CA-CFAR threshold, a fixed threshold, and the cells each one detects"});
  const st=h("div",{class:"stats"});
  const near=i=>CF_T.findIndex(t=>Math.abs(t[0]-i)<=1);
  const score=det=>{ const hit=new Set(); let fa=0; det.forEach((on,i)=>{ if(!on) return; const t=near(i); if(t<0) fa++; else hit.add(t); }); return [hit.size,fa]; };
  const draw=()=>{ const pfa=Math.pow(10,lp), x=cfarProfile(seed), T=cfarThr(x,nt,ng,pfa), fix=-Math.log(pfa), dc=x.map((p,i)=>p>T[i]), df=x.map(p=>p>fix), sc=score(dc), sf=score(df);
    svg.innerHTML="";
    [-10,0,10,20].forEach(q=>svg.append(sv("line",{x1:46,y1:Y(q),x2:586,y2:Y(q),stroke:"var(--line)"}),txt(40,Y(q)+4,String(q),{"text-anchor":"end","font-size":12,class:"mut"})));
    [0,25,50,75,100].forEach(m=>svg.append(txt(X(Math.min(199,m*2)),280,String(m),{"text-anchor":"middle","font-size":12,class:"mut"})));
    svg.append(txt(46,14,"power [dB above the quiet noise level]",{class:"mut","font-size":12}),txt(586,296,"range [m]",{"text-anchor":"end","font-size":12,class:"mut"}),
      sv("rect",{x:X(110),y:20,width:X(165)-X(110),height:242,fill:"var(--soft)"}),txt(586,14,"shaded: clutter, 9 dB higher",{"text-anchor":"end","font-size":11,class:"mut"}),
      pl(x.map((p,i)=>[X(i),Y(db(p))]),{stroke:"var(--muted)","stroke-width":1.2}),sv("line",{x1:46,y1:Y(db(fix)),x2:586,y2:Y(db(fix)),stroke:"var(--c2)","stroke-width":2,"stroke-dasharray":"6 4"}),
      pl(T.map((p,i)=>[X(i),Y(db(p))]),{stroke:"var(--c5)"}));
    CF_T.forEach(t=>svg.append(sv("path",{d:"M"+(X(t[0])-5)+" 20 L"+(X(t[0])+5)+" 20 L"+X(t[0])+" 29 z",fill:"var(--ink)"})));
    dc.forEach((on,i)=>{ if(on) svg.append(sv("circle",{cx:X(i),cy:Y(db(x[i])),r:4,fill:near(i)<0?"var(--c3)":"var(--c1)",stroke:"var(--surface)","stroke-width":1})); });
    const a=cfarAlpha(2*nt,pfa);
    st.innerHTML=""; st.append(stat("Threshold multiplier",a.toFixed(2)+" ("+db(a).toFixed(1)+" dB)","N = "+2*nt+" training cells"),stat("CA-CFAR finds",sc[0]+" of 5 targets",sc[1]+" false alarm cells"),
      stat("Fixed threshold finds",sf[0]+" of 5 targets",sf[1]+" false alarm cells"),stat("False alarms expected",(pfa*185).toFixed(2),"P_fa × 185 noise cells")); };
  el.append(h("div",{class:"vt"},"CA-CFAR on one azimuth"),h("p",{class:"vd"},"200 range cells of 0.5 m. Five targets (black markers) sit in exponentially distributed noise whose level is 8 times higher between 55 m and 82 m. The fixed threshold is set for the quiet noise level."),
    h("div",{class:"ctl"},range("Training cells per side",2,20,1,nt,v=>{ nt=v; draw(); }).el,range("Guard cells per side",0,4,1,ng,v=>{ ng=v; draw(); }).el,range("False alarm probability",-5,-1,0.5,lp,v=>{ lp=v; draw(); },v=>"1 in "+Math.round(Math.pow(10,-v))).el),
    h("div",{class:"ctl"},button("New noise",()=>{ seed++; draw(); })),svg,
    lgd([["var(--muted)","measured power"],["var(--c5)","CA-CFAR threshold"],["var(--c2)","fixed threshold"],["var(--c1)","CFAR detection on a target"],["var(--c3)","CFAR false alarm"]]),st);
  draw();
};

// ---- Day 3: ego-velocity from Doppler. Each point: azimuth th, measured closing speed y = (v_sensor - w_target) . u(th) + noise
const egoScene=(o,seed)=>{ const r=rng(seed), N=60, nm=Math.round(N*o.frac), F=o.fov*Math.PI/180, vx=o.sp*Math.cos(o.dir), vy=o.sp*Math.sin(o.dir), obj=[{c:0.25*F,w:[-12,0]},{c:-0.2*F,w:[4,-8]}], P=[];
  for(let i=0;i<N;i++){ const u1=r.u(), u2=r.u(), g=r.n(), mv=i<nm, ob=obj[i%2], th=mv?ob.c+(u1-0.5)*0.12*F:(u1-0.5)*F, w=mv?ob.w:[0,0];
    P.push({th,rad:4+8*u2,mv,y:(vx-w[0])*Math.cos(th)+(vy-w[1])*Math.sin(th)+o.sig*g}); }
  return P; };
// least squares for y_i = [cos th_i, sin th_i] v over the points in idx: solves the 2x2 normal equations; cov = (A'A)^-1
const egoFit=(P,idx)=>{ let a=0,b=0,d=0,p=0,q=0;
  for(const i of idx){ const c=Math.cos(P[i].th), s=Math.sin(P[i].th); a+=c*c; b+=c*s; d+=s*s; p+=c*P[i].y; q+=s*P[i].y; }
  const det=a*d-b*b; return det<1e-9?null:{v:[(d*p-b*q)/det,(a*q-b*p)/det],cov:[[d/det,-b/det],[-b/det,a/det]]}; };
const egoRes=(P,i,v)=>P[i].y-v[0]*Math.cos(P[i].th)-v[1]*Math.sin(P[i].th);
// RANSAC with 2-point samples (two unknowns in 2D); returns the largest consensus set
const egoRansac=(P,tol,seed)=>{ const r=rng(seed), all=P.map((_,m)=>m); let best=[];
  for(let k=0;k<80;k++){ const i=Math.floor(r.u()*P.length), j=Math.floor(r.u()*P.length); if(i===j) continue; const f=egoFit(P,[i,j]); if(!f) continue;
    const inl=all.filter(m=>Math.abs(egoRes(P,m,f.v))<tol); if(inl.length>best.length) best=inl; }
  return best.length>=2?best:all; };

VIZ.w14_egovel=el=>{
  const o={sp:10,dir:20*Math.PI/180,fov:120,frac:0.25,sig:0.1}; let seed=2, rob=false;
  const W=600, H=300, cx=150, cy=150, PX=x=>330+255*(x+180)/360, PY=y=>144-118*Math.max(-34,Math.min(34,y))/34;
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Left: top view of a radar with detected points and velocity arrows. Right: measured closing speed of every point against its azimuth, with the fitted cosine curves"});
  const st=h("div",{class:"stats"}), fm=v=>"("+v[0].toFixed(2)+", "+v[1].toFixed(2)+")";
  const arrow=(v,c,wd)=>sv("line",{x1:cx,y1:cy,x2:cx+5*v[0],y2:cy-5*v[1],stroke:c,"stroke-width":wd,"stroke-linecap":"round"});
  const curve=(v,a)=>pl(Array.from({length:73},(_,i)=>{ const t=-180+5*i, q=t*Math.PI/180; return [PX(t),PY(v[0]*Math.cos(q)+v[1]*Math.sin(q))]; }),a);
  const draw=()=>{ const P=egoScene(o,seed), all=P.map((_,i)=>i), vt=[o.sp*Math.cos(o.dir),o.sp*Math.sin(o.dir)], f0=egoFit(P,all), inl=egoRansac(P,Math.max(0.25,3*o.sig),seed+500), f1=egoFit(P,inl), keep=new Set(inl), err=f=>f?Math.hypot(f.v[0]-vt[0],f.v[1]-vt[1]).toFixed(2)+" m/s off":"not solvable";
    svg.innerHTML="";
    if(o.fov<360) [-1,1].forEach(s=>{ const a=s*o.fov*Math.PI/360; svg.append(sv("line",{x1:cx,y1:cy,x2:cx+128*Math.cos(a),y2:cy-128*Math.sin(a),stroke:"var(--line2)","stroke-dasharray":"4 4"})); });
    P.forEach((p,i)=>{ const x=cx+10*p.rad*Math.cos(p.th), y=cy-10*p.rad*Math.sin(p.th), c=p.mv?"var(--c2)":"var(--c5)", out=rob&&!keep.has(i);
      svg.append(sv("circle",{cx:x.toFixed(1),cy:y.toFixed(1),r:3.2,fill:out?"none":c,stroke:c,"stroke-width":1.5}),sv("circle",{cx:PX(p.th*180/Math.PI).toFixed(1),cy:PY(p.y).toFixed(1),r:3,fill:out?"none":c,stroke:c,"stroke-width":1.5})); });
    svg.append(sv("rect",{x:cx-9,y:cy-6,width:18,height:12,rx:3,fill:"var(--surface)",stroke:"var(--ink)","stroke-width":1.5}),arrow(vt,"var(--ink)",5));
    if(f0) svg.append(arrow(f0.v,"var(--c3)",2.5),curve(f0.v,{stroke:"var(--c3)","stroke-dasharray":"6 4"}));
    if(rob&&f1) svg.append(arrow(f1.v,"var(--c1)",2.5),curve(f1.v,{stroke:"var(--c1)"}));
    svg.append(txt(8,16,"top view (sensor looks right)",{class:"mut","font-size":12}),txt(330,16,"closing speed [m/s] against azimuth θ [°]",{class:"mut","font-size":12}),sv("line",{x1:330,y1:144,x2:585,y2:144,stroke:"var(--line2)"}));
    [-180,-90,0,90,180].forEach(t=>svg.append(sv("line",{x1:PX(t),y1:26,x2:PX(t),y2:262,stroke:"var(--line)"}),txt(PX(t),280,String(t),{"text-anchor":t<0?"start":t>0?"end":"middle","font-size":12,class:"mut"})));
    [-30,-15,15,30].forEach(y=>svg.append(txt(326,PY(y)+4,String(y),{"text-anchor":"end","font-size":12,class:"mut"})));
    const use=rob?f1:f0, sd=use?[o.sig*Math.sqrt(use.cov[0][0]),o.sig*Math.sqrt(use.cov[1][1])]:null;
    st.innerHTML=""; st.append(stat("True velocity (forward, left)",fm(vt)+" m/s"),stat("Least squares, all points",f0?fm(f0.v):"—",err(f0)),
      stat("After outlier rejection",rob&&f1?fm(f1.v):"—",rob?err(f1)+", kept "+inl.length+" of "+P.length:"press Reject outliers"),
      stat("One-sigma uncertainty",sd?sd[0].toFixed(3)+", "+sd[1].toFixed(3)+" m/s":"—","forward, left: σ√diag((AᵀA)⁻¹)")); };
  const rb=button("Reject outliers (RANSAC)",()=>{ rob=!rob; rb.classList.toggle("on",rob); draw(); });
  el.append(h("div",{class:"vt"},"Ego-velocity from one radar scan"),h("p",{class:"vd"},"60 detections, each with an azimuth and a Doppler closing speed. Static points lie on one cosine; points on two moving cars do not. The fit solves a 2×2 least-squares problem."),
    h("div",{class:"ctl"},range("Sensor speed",0,20,0.5,o.sp,v=>{ o.sp=v; draw(); },v=>v.toFixed(1)+" m/s").el,range("Direction of travel",-90,90,5,20,v=>{ o.dir=v*Math.PI/180; draw(); },v=>v+"°").el,range("Field of view",60,360,10,o.fov,v=>{ o.fov=v; draw(); },v=>v+"°").el),
    h("div",{class:"ctl"},range("Points on moving cars",0,40,5,25,v=>{ o.frac=v/100; draw(); },v=>v+"%").el,range("Doppler noise σ",0,0.5,0.05,o.sig,v=>{ o.sig=v; draw(); },v=>v.toFixed(2)+" m/s").el),
    h("div",{class:"ctl"},rb,button("New scene",()=>{ seed++; draw(); })),svg,
    lgd([["var(--c5)","static point"],["var(--c2)","point on a moving car"],["var(--ink)","true velocity"],["var(--c3)","least squares, all points"],["var(--c1)","after outlier rejection"]]),st);
  draw();
};

// ---- Day 4: a Scan Context descriptor for radar (sum of intensities per ring-sector bin) and its column-shift distance
const SC_R=8, SC_S=36, SC_MAX=80;
const scPlace=seed=>{ const r=rng(seed), P=[];
  for(let k=0;k<14;k++){ const rr=10+62*r.u(), a=2*Math.PI*r.u(), s=0.4+0.6*r.u(), n=8+Math.floor(6*r.u()), sp=1.5+2.5*r.u();
    for(let i=0;i<n;i++) P.push([rr*Math.cos(a)+sp*r.n(),rr*Math.sin(a)+sp*r.n(),s*(0.6+0.4*r.u())]); }
  return P; };
// descriptor seen from the origin with heading yaw and field of view fov; with r given, the scan is degraded (dropout, jitter, speckle)
const scDesc=(P,yaw,fov,r)=>{ const D=M.zeros(SC_R,SC_S);
  const add=(x,y,w)=>{ const d=Math.hypot(x,y); let a=Math.atan2(y,x)-yaw; a=Math.atan2(Math.sin(a),Math.cos(a)); if(d>=SC_MAX||Math.abs(a)>fov/2) return;
    D[Math.floor(d/SC_MAX*SC_R)][Math.min(SC_S-1,Math.floor((a+Math.PI)/(2*Math.PI)*SC_S))]+=w; };
  for(const [x,y,w] of P){ if(!r){ add(x,y,w); continue; } const keep=r.u()>0.2, jx=r.n(), jy=r.n(), g=r.n(); if(keep) add(x+jx,y+jy,Math.max(0,w*(1+0.2*g))); }
  if(r) for(let i=0;i<25;i++){ const d=SC_MAX*Math.sqrt(r.u()), a=2*Math.PI*r.u(); add(d*Math.cos(a),d*Math.sin(a),0.3); }
  return D; };
// mean over sectors of (1 - cosine similarity) between query sector j and database sector j+s.
// A sector that is empty in both is skipped; a sector that is empty in only one counts as a full mismatch (1).
const scDist=(Q,C,s)=>{ let sum=0,cnt=0;
  for(let j=0;j<SC_S;j++){ const k=((j+s)%SC_S+SC_S)%SC_S; let qq=0,cc=0,qc=0;
    for(let i=0;i<SC_R;i++){ qq+=Q[i][j]*Q[i][j]; cc+=C[i][k]*C[i][k]; qc+=Q[i][j]*C[i][k]; }
    if(qq===0&&cc===0) continue; cnt++; sum+=qq&&cc?1-qc/Math.sqrt(qq*cc):1; }
  return cnt?sum/cnt:1; };

VIZ.w14_sc=el=>{
  let yaw=60, fov=360, seed=1; const A=scPlace(11), Bp=scPlace(29);
  const W=600, H=330, CX=y=>46+534*(y+180)/350, CY=d=>300-140*d;
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Two ring-by-sector descriptors drawn as heat maps, the database scan and the rotated noisy query scan, above a plot of descriptor distance against the column shift for the same place and for a different place"});
  const st=h("div",{class:"stats"});
  const heat=(D,x0,mx)=>{ for(let i=0;i<SC_R;i++) for(let j=0;j<SC_S;j++) svg.append(sv("rect",{x:x0+7.5*j,y:93-9*i,width:7,height:8.5,fill:D[i][j]?"var(--c5)":"var(--soft)","fill-opacity":D[i][j]?(0.25+0.75*Math.sqrt(D[i][j]/mx)).toFixed(2):1})); };
  const draw=()=>{ const f=fov*Math.PI/180, C=scDesc(A,0,f), C2=scDesc(Bp,0,f), Q=scDesc(A,yaw*Math.PI/180,f,rng(seed)), mx=Math.max(1e-9,...C.flat(),...Q.flat());
    const ys=Array.from({length:SC_S},(_,i)=>-180+10*i), dA=ys.map(y=>scDist(Q,C,y/10)), dB=ys.map(y=>scDist(Q,C2,y/10));
    let k=0; dA.forEach((d,i)=>{ if(d<dA[k]) k=i; }); const mB=Math.min(...dB);
    svg.innerHTML=""; svg.append(txt(20,16,"database scan, heading 0°",{class:"mut","font-size":12}),txt(310,16,"query scan, heading "+yaw+"°, degraded",{class:"mut","font-size":12}),
      txt(20,118,"columns: 36 sectors of 10° in the sensor frame · rows: 8 rings of 10 m, nearest at the bottom",{class:"mut","font-size":11}));
    heat(C,20,mx); heat(Q,310,mx);
    [0,0.5,1].forEach(d=>svg.append(sv("line",{x1:46,y1:CY(d),x2:580,y2:CY(d),stroke:"var(--line)"}),txt(40,CY(d)+4,String(d),{"text-anchor":"end","font-size":12,class:"mut"})));
    [-180,-90,0,90,170].forEach(y=>svg.append(txt(CX(y),318,String(y),{"text-anchor":y<0?"start":y>90?"end":"middle","font-size":12,class:"mut"})));
    svg.append(txt(46,146,"descriptor distance against the column shift, written as a heading change [°]",{class:"mut","font-size":12}),
      sv("line",{x1:CX(yaw>170?yaw-360:yaw),y1:CY(0),x2:CX(yaw>170?yaw-360:yaw),y2:CY(1),stroke:"var(--muted)","stroke-dasharray":"3 4"}),
      pl(ys.map((y,i)=>[CX(y),CY(Math.min(1,dB[i]))]),{stroke:"var(--c3)"}),pl(ys.map((y,i)=>[CX(y),CY(Math.min(1,dA[i]))]),{stroke:"var(--c1)"}),
      sv("circle",{cx:CX(ys[k]),cy:CY(Math.min(1,dA[k])),r:5,fill:"var(--c1)",stroke:"var(--surface)","stroke-width":1.5}));
    st.innerHTML=""; st.append(stat("Best shift",((ys[k]/10+36)%36)+" sectors → "+ys[k]+"°","true heading change "+yaw+"°"),stat("Distance, same place",dA[k].toFixed(2),"at the best shift"),
      stat("Distance, different place",mB.toFixed(2),"best of all 36 shifts"),stat("Margin",(mB-dA[k]).toFixed(2),mB-dA[k]>0.1?"the right place wins":"too close to call")); };
  el.append(h("div",{class:"vt"},"A Scan Context descriptor for radar"),h("p",{class:"vd"},"Each bin holds the sum of the intensities of the points that fall in it. The query is the same place seen with a different heading, with 20% of the points dropped, 1 m of jitter and 25 speckle points. The distance is the mean over sectors of one minus the cosine similarity."),
    h("div",{class:"ctl"},range("Heading of the query scan",-180,180,10,yaw,v=>{ yaw=v; draw(); },v=>v+"°").el,range("Field of view of both scans",60,360,30,fov,v=>{ fov=v; draw(); },v=>v+"°").el),
    h("div",{class:"ctl"},button("New noise",()=>{ seed++; draw(); })),svg,lgd([["var(--c1)","query against the same place"],["var(--c3)","query against a different place"],["var(--muted)","true heading change"]]),st);
  draw();
};
