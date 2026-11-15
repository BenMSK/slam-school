// Week 13 · LiDAR SLAM. Every widget is computed from one 2-D room and a ray-casting LiDAR.
const ROOM=[[0,0],[8,0],[8,5],[5,5],[5,3.8],[3.5,3.8],[3.5,5],[0,5]];          // wall corners in metres: a room with a block on the far wall
const TAU=2*Math.PI, DEG=Math.PI/180;
const edges=f=>ROOM.forEach((A,i)=>{ const B=ROOM[(i+1)%ROOM.length]; f(A,B[0]-A[0],B[1]-A[1]); });
// range from (x,y) along world direction a to the nearest wall (ray-segment intersection)
const cast=(x,y,a)=>{ const c=Math.cos(a), s=Math.sin(a); let best=Infinity;
  edges((A,ex,ey)=>{ const den=c*ey-s*ex; if(Math.abs(den)<1e-12) return; const t=((A[0]-x)*ey-(A[1]-y)*ex)/den, u=((A[0]-x)*s-(A[1]-y)*c)/den; if(t>1e-9&&u>=0&&u<=1&&t<best) best=t; });
  return best; };
const wallDist=(x,y)=>{ let b=Infinity; edges((A,ex,ey)=>{ const u=Math.max(0,Math.min(1,((x-A[0])*ex+(y-A[1])*ey)/(ex*ex+ey*ey))); b=Math.min(b,Math.hypot(x-A[0]-u*ex,y-A[1]-u*ey)); }); return b; };
// pose [x,y,heading] after t seconds of constant speed v (along the heading) and constant yaw rate w, starting from s
const poseAt=(s,v,w,t)=>{ const th=s[2]+w*t; return Math.abs(w)<1e-9? [s[0]+v*t*Math.cos(s[2]),s[1]+v*t*Math.sin(s[2]),th] : [s[0]+v/w*(Math.sin(th)-Math.sin(s[2])),s[1]-v/w*(Math.cos(th)-Math.cos(s[2])),th]; };
const onPose=(p,x,y)=>{ const c=Math.cos(p[2]), s=Math.sin(p[2]); return [p[0]+c*x-s*y,p[1]+s*x+c*y]; };
// one sweep of n beams over T seconds. Beam k leaves at time T·k/n in sensor direction 2πk/n, from wherever the sensor is by then.
const sweep=(s,n,sig,seed,v=0,w=0,T=0.1)=>{ const r=rng(seed), out=[];
  for(let k=0;k<n;k++){ const t=T*k/n, o=poseAt(s,v,w,t), a=TAU*k/n, d=cast(o[0],o[1],o[2]+a)+sig*r.n(); if(isFinite(d)&&d>0) out.push({a,d,t,o,x:d*Math.cos(a),y:d*Math.sin(a)}); }
  return out; };
const roomPath=(X,Y,a)=>sv("path",Object.assign({d:ROOM.map((p,i)=>(i?"L":"M")+X(p[0]).toFixed(1)+" "+Y(p[1]).toFixed(1)).join(" ")+" Z",fill:"none",stroke:"var(--line2)","stroke-width":2},a));
const tri=(x,y,th,col,r=9)=>sv("path",{d:"M"+[[r,0],[-r*0.7,r*0.6],[-r*0.7,-r*0.6]].map(([a,b])=>(x+a*Math.cos(th)+b*Math.sin(th)).toFixed(1)+" "+(y-a*Math.sin(th)+b*Math.cos(th)).toFixed(1)).join(" L")+" Z",fill:col,stroke:"var(--surface)","stroke-width":1.5});
const legend=items=>h("div",{class:"legend"},...items.map(([c,t])=>h("span",{},h("i",{style:"background:"+c}),t)));
const wrap=a=>{ a=(a+Math.PI)%TAU; return (a<0?a+TAU:a)-Math.PI; };

// ---------- Day 1 and Day 3: one sweep of a 2-D LiDAR, with the sensor moving during the sweep ----------
const scanViz=(el,comp)=>{
  const W=600, H=372, sc=56, ox=76, oy=312, S0=[1.6,1.5,0], T=0.1, X=x=>ox+x*sc, Y=y=>oy-y*sc;
  let n=comp?180:120, sig=comp?1:2, v=comp?6:0, w=comp?90:0, err=0, on=false;
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Top view of a room. A LiDAR at the lower left sends beams in all directions; each dot is one measured point, drawn where the sensor believes it is."});
  const st=h("div",{class:"stats"}), chart=h("div"), lg=h("div");
  const draw=()=>{ const S=sweep(S0,n,sig/100,5,v,w*DEG,T), every=Math.ceil(S.length/60), vh=v*(1+err/100), wh=w*DEG*(1+err/100);
    const raw=S.map(p=>onPose(S0,p.x,p.y)), fix=S.map(p=>onPose(poseAt(S0,vh,wh,p.t),p.x,p.y)), rms=Q=>Math.sqrt(Q.reduce((a,q)=>a+wallDist(q[0],q[1])**2,0)/Math.max(1,Q.length));
    svg.innerHTML=""; svg.append(roomPath(X,Y));
    S.forEach((p,k)=>{ if(k%every) return; const e=onPose(p.o,p.x,p.y); svg.append(sv("line",{x1:X(p.o[0]),y1:Y(p.o[1]),x2:X(e[0]),y2:Y(e[1]),stroke:"var(--c2)","stroke-width":1,opacity:0.35})); });
    raw.forEach(q=>svg.append(sv("circle",{cx:X(q[0]).toFixed(1),cy:Y(q[1]).toFixed(1),r:2.6,fill:"var(--c3)",opacity:comp&&on?0.3:1})));
    if(comp&&on) fix.forEach(q=>svg.append(sv("circle",{cx:X(q[0]).toFixed(1),cy:Y(q[1]).toFixed(1),r:2.6,fill:"var(--c1)"})));
    const path=[]; for(let i=0;i<=20;i++){ const p=poseAt(S0,v,w*DEG,T*i/20); path.push((i?"L":"M")+X(p[0]).toFixed(1)+" "+Y(p[1]).toFixed(1)); }
    const E=poseAt(S0,v,w*DEG,T);
    svg.append(sv("path",{d:path.join(" "),fill:"none",stroke:"var(--ink)","stroke-width":2.5}),tri(X(E[0]),Y(E[1]),E[2],"var(--muted)",7),tri(X(S0[0]),Y(S0[1]),S0[2],"var(--ink)"));
    const dth=360/n, ds=S.map(p=>p.d), lo=Math.min(...ds), hi=Math.max(...ds); st.innerHTML=""; lg.innerHTML="";
    if(comp){ st.append(stat("Sensor motion in one sweep",(v*T).toFixed(2)+" m, "+(w*T).toFixed(0)+"°","0.1 s at "+v+" m/s and "+w+"°/s"),
        stat("Points off the walls, raw",(rms(raw)*100).toFixed(1)+" cm","RMS distance to the nearest wall"),
        stat("Points off the walls, compensated",on?(rms(fix)*100).toFixed(1)+" cm":"—",on?"with the velocity estimate "+(err>0?"+":"")+err+"% off":"press Motion compensation"));
      lg.append(legend([["var(--line2)","true walls"],["var(--ink)","sensor path during the sweep"],["var(--c3)","raw scan (all points placed from the start pose)"]].concat(on?[["var(--c1)","motion-compensated scan"]]:[]))); }
    else { st.append(stat("Points per sweep",S.length,"one every "+dth.toFixed(2)+"°"),
        stat("Gap to the next point, nearest return",(lo*dth*DEG*100).toFixed(1)+" cm","r·Δθ at range "+lo.toFixed(2)+" m"),
        stat("Gap to the next point, farthest return",(hi*dth*DEG*100).toFixed(1)+" cm","r·Δθ at range "+hi.toFixed(2)+" m"),
        stat("Sensor moved during the sweep",(v*T*100).toFixed(0)+" cm","0.1 s at "+v+" m/s"));
      lg.append(legend([["var(--line2)","true walls"],["var(--c2)","beams"],["var(--c3)","measured points"]]));
      chart.innerHTML=""; chart.append(lineChart({w:600,h:190,aria:"Measured range against beam angle for this sweep",x:{min:0,max:360,ticks:[[0,"0°"],[90,"90°"],[180,"180°"],[270,"270°"],[360,"360°"]],label:"beam angle"},
        y:{min:0,max:8,ticks:[[0,"0"],[2,"2"],[4,"4"],[6,"6"],[8,"8"]],label:"range (m)"},series:[{name:"range",color:"var(--c3)",dots:false,points:S.map(p=>({x:p.a/DEG,y:Math.min(8,Math.max(0,p.d))}))}]})); } };
  const cb=button("Motion compensation: off",()=>{ on=!on; cb.classList.toggle("on",on); cb.textContent="Motion compensation: "+(on?"on":"off"); draw(); });
  el.append(...[h("div",{class:"vt"},comp?"Motion distortion and its correction":"One sweep of a 2-D LiDAR"),
    h("p",{class:"vd"},comp?"The sensor drives and turns while its beam sweeps 360° in 0.1 s. Compensation moves each point by the pose the constant-velocity model predicts for its own time stamp."
      :"A beam rotates once in 0.1 s inside an 8 m by 5 m room. Each range is the distance to the first wall the beam meets, plus Gaussian noise."),
    comp? h("div",{class:"ctl"},range("Speed",0,10,0.5,v,x=>{ v=x; draw(); },x=>x+" m/s").el,range("Yaw rate",0,180,10,w,x=>{ w=x; draw(); },x=>x+"°/s").el,range("Error of the velocity estimate",-50,50,5,err,x=>{ err=x; draw(); },x=>(x>0?"+":"")+x+"%").el)
        : h("div",{class:"ctl"},range("Beams per sweep",30,720,30,n,x=>{ n=x; draw(); }).el,range("Range noise (1σ)",0,10,0.5,sig,x=>{ sig=x; draw(); },x=>x+" cm").el,range("Sensor speed",0,10,0.5,v,x=>{ v=x; draw(); },x=>x+" m/s").el),
    comp?h("div",{class:"ctl"},cb):null, svg, lg, comp?null:chart, st].filter(Boolean));
  draw();
};
VIZ.w13_scan=el=>scanViz(el,false);
VIZ.w13_deskew=el=>scanViz(el,true);

// ---------- Day 2: ICP between two scans of the room ----------
// Scan P is taken at pose A, scan Q after the sensor turned by `turn` degrees and moved `shift` metres. Unknown: (R,t) with p ≈ R q + t,
// stored as est=[tx,ty,theta]. It is also the pose of the second sensor in the frame of the first. The first guess is "no motion".
const icpMake=(turn,shift,mode)=>{
  const A=[2.5,2,0], N=72, B=[A[0]+shift*Math.cos(0.35),A[1]+shift*Math.sin(0.35),turn*DEG], P=sweep(A,N,0.01,11), Q=sweep(B,N,0.01,12);
  // unit normal of the fixed scan at each point, from its two neighbours in the sweep; none across a depth jump
  const nrm=P.map((p,i)=>{ const a=P[(i+P.length-1)%P.length], b=P[(i+1)%P.length], dx=b.x-a.x, dy=b.y-a.y, L=Math.hypot(dx,dy); return L>1||L<1e-9?null:[-dy/L,dx/L]; });
  const o={P,Q,truth:[B[0]-A[0],B[1]-A[1],turn*DEG],est:[0,0,0],hist:[],pairs:null,iters:0,done:false};
  o.moved=()=>{ const c=Math.cos(o.est[2]), s=Math.sin(o.est[2]); return Q.map(q=>[c*q.x-s*q.y+o.est[0],s*q.x+c*q.y+o.est[1]]); };
  // equation (8.3): for every p, the nearest point of the moved scan
  o.match=()=>{ const Qm=o.moved(); let ss=0; o.pairs=P.map((p,i)=>{ let b=0, bd=Infinity; Qm.forEach((q,j)=>{ const d=(p.x-q[0])**2+(p.y-q[1])**2; if(d<bd){ bd=d; b=j; } }); ss+=bd; return [i,b]; }); o.hist.push(Math.sqrt(ss/P.length)); };
  o.align=()=>{ const old=o.est, c=Math.cos(old[2]), s=Math.sin(old[2]);
    if(mode==="p2p"){ // closed form: centroids, then the angle that maximizes sum p'·R q'
      let px=0, py=0, qx=0, qy=0, a=0, b=0; const n=o.pairs.length; o.pairs.forEach(([i,j])=>{ px+=P[i].x/n; py+=P[i].y/n; qx+=Q[j].x/n; qy+=Q[j].y/n; });
      o.pairs.forEach(([i,j])=>{ const ux=Q[j].x-qx, uy=Q[j].y-qy, vx=P[i].x-px, vy=P[i].y-py; a+=ux*vx+uy*vy; b+=ux*vy-uy*vx; });
      const th=Math.atan2(b,a), C=Math.cos(th), S=Math.sin(th); o.est=[px-(C*qx-S*qy),py-(S*qx+C*qy),th]; }
    else { // one Gauss-Newton step on r = nᵀ(p − R q − t), right perturbation T = T⁰ Exp(ξ), ξ = (ρ, θ)
      const Hm=M.zeros(3,3), g=[0,0,0];
      o.pairs.forEach(([i,j])=>{ const n=nrm[i]; if(!n) return; const q=Q[j], m=[c*n[0]+s*n[1],-s*n[0]+c*n[1]], r=n[0]*(P[i].x-(c*q.x-s*q.y+old[0]))+n[1]*(P[i].y-(s*q.x+c*q.y+old[1])), J=[-m[0],-m[1],m[0]*q.y-m[1]*q.x];
        for(let u=0;u<3;u++){ g[u]-=J[u]*r; for(let k=0;k<3;k++) Hm[u][k]+=J[u]*J[k]; } });
      const d=M.solve(Hm,g); if(d) o.est=[old[0]+c*d[0]-s*d[1],old[1]+s*d[0]+c*d[1],old[2]+d[2]]; }
    o.iters++; o.pairs=null; if((Math.hypot(o.est[0]-old[0],o.est[1]-old[1])<1e-4&&Math.abs(o.est[2]-old[2])<1e-4)||o.iters>=60) o.done=true; };
  o.step=()=>{ if(o.done) return; if(!o.pairs) o.match(); else { o.align(); if(o.done) o.match(); } };
  o.rotErr=()=>Math.abs(wrap(o.est[2]-o.truth[2]))/DEG; o.posErr=()=>Math.hypot(o.est[0]-o.truth[0],o.est[1]-o.truth[1]);
  o.match(); return o; };
VIZ.w13_icp=el=>{
  const W=600, H=385, sc=36, ox=235, oy=212, X=x=>ox+x*sc, Y=y=>oy-y*sc;
  let turn=20, shift=0.8, mode="p2p", o, raf=0, last=0;
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Two LiDAR scans of the same room drawn in the frame of the first sensor: the fixed scan P and the moving scan Q under the current estimate, with a line for each correspondence"});
  const st=h("div",{class:"stats"}), chart=h("div"), out=h("div",{class:"out"});
  const draw=()=>{ const Qm=o.moved(); svg.innerHTML="";
    if(o.pairs) o.pairs.forEach(([i,j])=>svg.append(sv("line",{x1:X(o.P[i].x).toFixed(1),y1:Y(o.P[i].y).toFixed(1),x2:X(Qm[j][0]).toFixed(1),y2:Y(Qm[j][1]).toFixed(1),stroke:"var(--muted)","stroke-width":1})));
    o.P.forEach(p=>svg.append(sv("circle",{cx:X(p.x).toFixed(1),cy:Y(p.y).toFixed(1),r:3.2,fill:"var(--c5)"})));
    Qm.forEach(q=>svg.append(sv("circle",{cx:X(q[0]).toFixed(1),cy:Y(q[1]).toFixed(1),r:3.2,fill:"var(--c3)"})));
    svg.append(sv("circle",{cx:X(o.truth[0]),cy:Y(o.truth[1]),r:9,fill:"none",stroke:"var(--c1)","stroke-width":2.5}),tri(X(0),Y(0),0,"var(--c5)"),tri(X(o.est[0]),Y(o.est[1]),o.est[2],"var(--c3)"));
    const hi=Math.max(0.1,...o.hist)*1.1, xm=Math.max(10,o.hist.length-1);
    chart.innerHTML=""; chart.append(lineChart({w:600,h:190,aria:"RMS distance between matched points after each ICP iteration",x:{min:0,max:xm,ticks:[0,0.25,0.5,0.75,1].map(f=>[Math.round(f*xm),""+Math.round(f*xm)]),label:"iteration"},
      y:{min:0,max:hi,ticks:[0,0.5,1].map(f=>[f*hi,(f*hi).toFixed(2)]),label:"RMS distance (m)"},series:[{name:"RMS distance of the pairs",color:"var(--c4)",points:o.hist.map((y,x)=>({x,y,label:"iteration "+x,fmt:z=>z.toFixed(3)+" m"}))}]}));
    out.textContent=o.done? (o.iters>=60?"Stopped after 60 iterations.":"Converged: the pairs no longer change the estimate.") : o.pairs? "Pairs found (8.3). Next step: solve for (R, t) from these pairs." : "Estimate updated. Next step: find the pairs again.";
    st.innerHTML=""; st.append(stat("Iterations",o.iters,o.done?"converged":"alignment steps so far"),stat("RMS distance of the pairs",(o.hist[o.hist.length-1]*100).toFixed(1)+" cm","point-to-point, latest matching"),
      stat("Rotation error",o.rotErr().toFixed(1)+"°","estimate against the true turn of "+turn+"°"),stat("Translation error",(o.posErr()*100).toFixed(1)+" cm","estimate against the true move")); };
  const stop=()=>{ if(raf) cancelAnimationFrame(raf); raf=0; runB.textContent="Run"; };
  const tick=ts=>{ if(ts-last>220){ last=ts; o.step(); draw(); } if(o.done){ stop(); return; } raf=requestAnimationFrame(tick); };
  const runB=button("Run",()=>{ if(raf){ stop(); return; } if(o.done) return; runB.textContent="Pause"; last=0; raf=requestAnimationFrame(tick); },"primary");
  const reset=()=>{ stop(); o=icpMake(turn,shift,mode); draw(); };
  el.append(h("div",{class:"vt"},"ICP, one half-step at a time"),
    h("p",{class:"vd"},"Two 72-beam scans of the same room, taken before and after the sensor moved. The first guess is that it did not move. Each Step either pairs the points or solves for the motion."),
    h("div",{class:"ctl"},select("Distance",[["p2p","point-to-point"],["p2l","point-to-plane (a line in 2-D)"]],x=>{ mode=x; reset(); }).el,
      range("Sensor turned by",-180,180,5,turn,x=>{ turn=x; reset(); },x=>x+"°").el,range("Sensor moved by",0,2,0.1,shift,x=>{ shift=x; reset(); },x=>x.toFixed(1)+" m").el),
    h("div",{class:"ctl"},button("Step",()=>{ stop(); o.step(); draw(); }),runB,button("Reset",reset,"ghost")),svg,
    legend([["var(--c5)","fixed scan P and sensor A"],["var(--c3)","moving scan Rq + t and estimated sensor B"],["var(--muted)","pairs"],["var(--c1)","true position of sensor B (ring)"]]),out,chart,st);
  reset();
};

// ---------- Day 4: a ring-and-sector descriptor compared under rotation by a column shift ----------
const NR=6, NS=24, RMAX=6.6;
const scDesc=S=>{ const D=M.zeros(NR,NS); S.forEach(p=>{ D[Math.min(NR-1,Math.floor(p.d/RMAX*NR))][Math.floor(p.a/TAU*NS)%NS]++; }); return D; };
// mean cosine distance between column j of D and column j+sh of E, over the columns that are non-empty in both
const scDist=(D,E,sh)=>{ let sum=0, k=0; for(let j=0;j<NS;j++){ const jj=(j+sh)%NS; let ab=0, aa=0, bb=0; for(let i=0;i<NR;i++){ ab+=D[i][j]*E[i][jj]; aa+=D[i][j]**2; bb+=E[i][jj]**2; } if(aa&&bb){ sum+=1-ab/Math.sqrt(aa*bb); k++; } } return k?sum/k:1; };
const scBest=(D,E)=>{ let b=0; const ds=[]; for(let s=0;s<NS;s++){ ds.push(scDist(D,E,s)); if(ds[s]<ds[b]) b=s; } return {ds,b}; };
VIZ.w13_sc=el=>{
  const W=600, H=500, R=128, A=[2.5,2.2,0], C=[6.6,1.2,0], D0=scDesc(sweep(A,360,0.01,21)), DC=scDesc(sweep(C,360,0.01,23));
  let yaw=90, off=0.3;
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Two scans of the same room in polar bins, the stored one and the query taken with a different heading, and below them the two descriptors as grids of rings by sectors, plus the query grid shifted to the best-matching column offset"});
  const st=h("div",{class:"stats"}), chart=h("div");
  const polar=(cx,S,col,label)=>{ const g=sv("g",{}); for(let i=1;i<=NR;i++) g.append(sv("circle",{cx,cy:150,r:R*i/NR,fill:"none",stroke:"var(--line)"}));
    for(let j=0;j<NS;j++) g.append(sv("line",{x1:cx,y1:150,x2:(cx+R*Math.cos(TAU*j/NS)).toFixed(1),y2:(150-R*Math.sin(TAU*j/NS)).toFixed(1),stroke:"var(--line)"}));
    S.forEach(p=>g.append(sv("circle",{cx:(cx+p.x*R/RMAX).toFixed(1),cy:(150-p.y*R/RMAX).toFixed(1),r:1.8,fill:col})));
    g.append(sv("line",{x1:cx,y1:150,x2:cx+R+10,y2:150,stroke:"var(--ink)","stroke-width":2}),tri(cx,150,0,"var(--ink)",7),txt(cx,14,label,{"text-anchor":"middle","data-top":1})); return g; };
  const grid=(y,D,col,label,sh)=>{ const g=sv("g",{}), mx=Math.max(1,...D.flat()); g.append(txt(8,y+32,label,{class:"mut","font-size":12}));
    for(let i=0;i<NR;i++) for(let j=0;j<NS;j++) g.append(sv("rect",{x:150+j*18,y:y+(NR-1-i)*9,width:17,height:8,fill:col,opacity:(0.06+0.94*D[i][(j+sh)%NS]/mx).toFixed(2)})); return g; };
  const draw=()=>{ const Sq=sweep([A[0]+off,A[1],yaw*DEG],360,0.01,22), Dq=scDesc(Sq), m=scBest(D0,Dq), other=scBest(D0,DC), est=(NS-m.b)%NS*360/NS;
    svg.innerHTML=""; svg.append(polar(150,sweep(A,360,0.01,21),"var(--c5)","stored scan (sensor frame)"),polar(450,Sq,"var(--c3)","query scan (sensor frame)"),
      grid(300,D0,"var(--c5)","stored descriptor",0),grid(366,Dq,"var(--c3)","query descriptor",0),grid(432,Dq,"var(--c3)","query, shifted "+m.b+" col.",m.b));
    const hi=Math.max(0.2,...m.ds)*1.1;
    chart.innerHTML=""; chart.append(lineChart({w:600,h:190,aria:"Descriptor distance between the stored scan and the query for each of the 24 column shifts",x:{min:0,max:NS-1,ticks:[0,6,12,18,23].map(s=>[s,""+s]),label:"column shift"},
      y:{min:0,max:hi,ticks:[0,0.5,1].map(f=>[f*hi,(f*hi).toFixed(2)]),label:"distance"},series:[{name:"distance",color:"var(--c4)",points:m.ds.map((y,x)=>({x,y,label:"shift "+x,fmt:z=>z.toFixed(3)}))}],marker:{x:m.b,y:m.ds[m.b]}}));
    st.innerHTML=""; st.append(stat("Best column shift",m.b+" of "+NS,"each column is "+360/NS+"°"),stat("Heading change, estimated",est+"°","true: "+yaw+"°"),
      stat("Distance to the same place",m.ds[m.b].toFixed(3),"at the best shift"),stat("Distance to another place",other.ds[other.b].toFixed(3),"a scan taken 4 m away, at its best shift")); };
  el.append(h("div",{class:"vt"},"A polar descriptor and its column shift"),
    h("p",{class:"vd"},"Each scan is binned into "+NR+" rings by "+NS+" sectors around the sensor; a cell stores how many points fell in it. Turning the sensor shifts the columns, so the two grids are compared at every shift."),
    h("div",{class:"ctl"},range("Heading of the query",0,345,15,yaw,x=>{ yaw=x; draw(); },x=>x+"°").el,range("Position offset of the query",0,1.5,0.1,off,x=>{ off=x; draw(); },x=>x.toFixed(1)+" m").el),
    svg,chart,st);
  draw();
};
