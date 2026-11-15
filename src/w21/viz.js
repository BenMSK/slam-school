// Week 21 · Dynamic and deformable SLAM.
// w21_rate: rate of change vs rate of observation (Day 1).   w21_track: constant-velocity motion factors (Day 2).
// w21_ghost: point accumulation vs free-space clearing (Day 3).   w21_persist: is the object still there? (Day 4).
// w21_arap: a deformation graph with an as-rigid-as-possible regularizer (Day 5).
// The simulations are plain functions (rateSim, trackSolve, ghostScan, persistRun, arapAll); the VIZ entries only draw them.
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
const legend=items=>h("div",{class:"legend"},...items.map(([c,t])=>h("span",{},h("i",{style:"background:"+c+";box-shadow:0 0 0 1px var(--line2)"}),t)));
const tri=(a,L)=>{ const m=((a%(2*L))+2*L)%(2*L); return m<=L?m:2*L-m; };       // fold a onto [0, L]: straight motion with bounces
const rms=a=>Math.sqrt(a.reduce((s,v)=>s+v*v,0)/(a.length||1));

// ---------- Day 1: eight look-alike objects in a 10 m × 5 m room, observed every d = v/f metres of travel ----------
const RB={W:10,H:5,K:8,NP:60};
const rateObjs=()=>{ const r=rng(11); return Array.from({length:RB.K},()=>{ const a=r.u()*2*Math.PI; return {x:r.u()*RB.W,y:r.u()*RB.H,ux:Math.cos(a),uy:Math.sin(a)}; }); };
const ratePos=(o,s)=>[tri(o.x+o.ux*s,RB.W),tri(o.y+o.uy*s,RB.H)];                // position after travelling a path length s
// nearest-neighbour association between consecutive observations, scored over NP pairs
const rateSim=(O,d)=>{ let ok=0, gap=0, first=null;
  for(let k=0;k<RB.NP;k++){ const A=O.map(o=>ratePos(o,k*d)), B=O.map(o=>ratePos(o,(k+1)*d));
    const nn=A.map(a=>{ let b=0, bd=1e9; B.forEach((q,j)=>{ const e=Math.hypot(q[0]-a[0],q[1]-a[1]); if(e<bd){ bd=e; b=j; } }); return b; });
    nn.forEach((j,i)=>{ if(j===i) ok++; let m=1e9; A.forEach((q,l)=>{ if(l!==i) m=Math.min(m,Math.hypot(q[0]-A[i][0],q[1]-A[i][1])); }); gap+=m; });
    if(!k) first={A,B,nn}; }
  return {ok:ok/(RB.NP*RB.K),gap:gap/(RB.NP*RB.K),first}; };

VIZ.w21_rate=el=>{
  const RATES=[0.2,0.5,1,2,5,10,20,50], O=rateObjs(), sc=56, X=p=>(20+p[0]*sc).toFixed(1), Y=p=>(10+p[1]*sc).toFixed(1);
  let v=1.5, ri=5;
  const svg=sv("svg",{viewBox:"0 0 600 300",role:"img","aria-label":"Top view of a room with eight identical moving objects, drawn at two consecutive observations, with a link from each old position to the nearest new one"});
  const st=h("div",{class:"stats"});
  const draw=()=>{ const d=v/RATES[ri], R=rateSim(O,d), F=R.first;
    svg.innerHTML=""; svg.append(sv("rect",{x:20,y:10,width:RB.W*sc,height:RB.H*sc,rx:6,fill:"var(--soft)",stroke:"var(--line2)"}));
    O.forEach((o,i)=>{ const a=F.A[i], b=F.B[F.nn[i]], good=F.nn[i]===i;
      if(d<=3){ const n=Math.max(2,Math.ceil(d/0.1)); svg.append(sv("path",{d:Array.from({length:n+1},(_,k)=>(k?"L":"M")+X(ratePos(o,k*d/n))+" "+Y(ratePos(o,k*d/n))).join(" "),fill:"none",stroke:"var(--muted)","stroke-width":1.5,"stroke-dasharray":"3 4"})); }
      svg.append(sv("line",{x1:X(a),y1:Y(a),x2:X(b),y2:Y(b),stroke:good?"var(--c1)":"var(--c3)","stroke-width":good?3:2.5}),
        sv("circle",{cx:X(a),cy:Y(a),r:7,fill:"var(--surface)",stroke:"var(--ink)","stroke-width":2})); });
    F.B.forEach(b=>svg.append(sv("circle",{cx:X(b),cy:Y(b),r:5.5,fill:"var(--c5)",stroke:"var(--surface)","stroke-width":1.5})));
    st.innerHTML=""; st.append(stat("Motion between observations",(d<10?d.toFixed(2):d.toFixed(0))+" m","d = v / f"),
      stat("Compared with the spacing",(d/R.gap).toFixed(2)+" ×","nearest other object is "+R.gap.toFixed(2)+" m away on average"),
      stat("Correct matches",(100*R.ok).toFixed(0)+"%","nearest neighbour, over 60 pairs of observations")); };
  el.append(h("div",{class:"vt"},"Same motion, different observer"),h("p",{class:"vd"},"Eight identical objects move in straight lines and bounce off the walls. The robot matches each object to the nearest detection in its next observation."),
    h("div",{class:"ctl"},range("Object speed v",0.5,10,0.5,v,x=>{ v=x; draw(); },x=>x.toFixed(1)+" m/s").el,range("Observation rate f",0,RATES.length-1,1,ri,x=>{ ri=x; draw(); },x=>RATES[x]+" Hz").el),
    svg,legend([["var(--ink)","observation k (rings)"],["var(--c5)","observation k+1 (dots)"],["var(--c1)","match is right"],["var(--c3)","match is wrong"]]),st);
  draw();
};

// ---------- Day 2: one tracked object, positions p_i and velocities v_i, joined by constant-velocity factors ----------
const TK={N:30,dt:0.2,g0:12,sc:0.02};
const trackTruth=()=>Array.from({length:TK.N},(_,i)=>{ const t=i*TK.dt; return [0.6+1.5*t,2.6+1.3*Math.sin(0.8*t)]; });
// Linear least squares in x = [p_0..p_{N-1}, v_0..v_{N-1}], the same matrix for both axes. Three kinds of factor:
//   measurement  (p_i - z_i)/sz            only where the object was seen
//   velocity     (v_{i+1} - v_i)/sa        the 2-D analogue of equation (15.2)
//   coupling     (p_{i+1} - p_i - v_i dt)/sc   the 2-D, rotation-free analogue of equations (15.3)-(15.4)
const trackSolve=(Z,vis,sz,sa)=>{ const N=TK.N, A=M.zeros(2*N,2*N), b=[new Array(2*N).fill(0),new Array(2*N).fill(0)];
  const add=(J,r,w)=>J.forEach(([i,c])=>{ J.forEach(([j,e])=>{ A[i][j]+=w*c*e; }); b[0][i]+=w*c*r[0]; b[1][i]+=w*c*r[1]; });
  for(let i=0;i<N;i++){ if(vis[i]) add([[i,1]],Z[i],1/(sz*sz));
    if(i<N-1){ add([[N+i+1,1],[N+i,-1]],[0,0],1/(sa*sa)); add([[i+1,1],[i,-1],[N+i,-TK.dt]],[0,0],1/(TK.sc*TK.sc)); } }
  const x=M.solve(A,b[0]), y=M.solve(A,b[1]); return x&&y?x.slice(0,N).map((u,i)=>[u,y[i]]):null; };

VIZ.w21_track=el=>{
  let sz=0.25, sa=0.15, G=6, on=true, seed=4;
  const T=trackTruth(), X=p=>(10+p[0]*58).toFixed(1), Y=p=>(302-p[1]*58).toFixed(1);
  const svg=sv("svg",{viewBox:"0 0 600 310",role:"img","aria-label":"Top view of one moving object: its true path, noisy position measurements with a stretch where it is hidden, and the path estimated with constant-velocity factors"});
  const st=h("div",{class:"stats"});
  const line=(P,a)=>sv("path",Object.assign({d:P.map((p,i)=>(i?"L":"M")+X(p)+" "+Y(p)).join(" "),fill:"none","stroke-width":2.5,"stroke-linejoin":"round"},a));
  const draw=()=>{ const r=rng(seed), Z=T.map(p=>[p[0]+sz*r.n(),p[1]+sz*r.n()]), vis=T.map((_,i)=>i<TK.g0||i>=TK.g0+G), E=on?trackSolve(Z,vis,sz,sa):null;
    const err=(P,keep)=>P.map((p,i)=>keep(i)?Math.hypot(p[0]-T[i][0],p[1]-T[i][1]):null).filter(e=>e!=null);
    svg.innerHTML="";
    if(G>0){ const x0=+X(T[TK.g0])-8, x1=+X(T[TK.g0+G-1])+8; svg.append(sv("rect",{x:x0,y:6,width:x1-x0,height:298,fill:"var(--soft)"}),txt((x0+x1)/2,22,G>2?"hidden":"",{"text-anchor":"middle",class:"mut","font-size":12})); }
    svg.append(line(T,{stroke:"var(--muted)","stroke-dasharray":"6 5","stroke-width":2}));
    if(E) svg.append(line(E,{stroke:"var(--c1)"}),...E.map(p=>sv("circle",{cx:X(p),cy:Y(p),r:2.5,fill:"var(--c1)"})));
    Z.forEach((z,i)=>{ if(vis[i]) svg.append(sv("circle",{cx:X(z),cy:Y(z),r:4,fill:"var(--c2)",stroke:"var(--surface)","stroke-width":1.5})); });
    st.innerHTML=""; st.append(stat("RMS error of the measurements",rms(err(Z,i=>vis[i])).toFixed(2)+" m","where the object was seen"),
      stat("RMS error of the estimate",E?rms(err(E,i=>vis[i])).toFixed(2)+" m":"—",E?"at the same instants":"motion factors are off"),
      stat("Largest error while hidden",E&&G>0?Math.max(...err(E,i=>!vis[i])).toFixed(2)+" m":"—",E?(G>0?G+" steps with no measurement":"nothing is hidden"):"no estimate there at all")); };
  const tb=button("Motion factors: on",()=>{ on=!on; tb.classList.toggle("on",on); tb.textContent="Motion factors: "+(on?"on":"off"); draw(); }); tb.classList.add("on");
  el.append(h("div",{class:"vt"},"Tracking with constant-velocity factors"),h("p",{class:"vd"},"30 object positions and 30 velocities, 0.2 s apart, solved as one linear least-squares problem: position measurements, velocity factors, and factors that tie each position step to the velocity."),
    h("div",{class:"ctl"},range("Measurement noise",0.05,0.6,0.05,sz,x=>{ sz=x; draw(); },x=>x.toFixed(2)+" m").el,range("Velocity change allowed per step",0.02,1,0.01,sa,x=>{ sa=x; draw(); },x=>x.toFixed(2)+" m/s").el,
      range("Steps hidden",0,14,1,G,x=>{ G=x; draw(); },x=>x+"").el),
    h("div",{class:"ctl"},tb,button("New noise",()=>{ seed++; draw(); })),svg,
    legend([["var(--muted)","true path"],["var(--c2)","position measurements"],["var(--c1)","estimate with motion factors"]]),st);
  draw();
};

// ---------- Day 3: a fixed 2-D lidar, a room, and a 3×3 object crossing it ----------
const GW=29, GH=21, GS={x:14.5,y:17.5}, NB=180, LMIN=-2, LMAX=3.5, TMAX=160;
const gStatic=(x,y)=>x<=0||y<=0||x>=GW-1||y>=GH-1||(x>=19&&x<=20&&y>=9&&y<=10);   // walls and one pillar
const gObjX=(t,sp)=>2+Math.round(tri(sp*t,GW-7));                                 // left column of the object (rows 3..5)
const gObj=(S,x,y)=>x>=S.ox&&x<S.ox+3&&y>=3&&y<=5;
const ghostNew=()=>({t:0,ox:2,N:new Uint8Array(GW*GH),L:new Float64Array(GW*GH),M:new Int32Array(GW*GH)});
// One scan. Every beam is marched until it meets an occupied cell. N keeps every hit for ever. L is a log-odds grid:
// the end cell gains lh, each cell the beam crossed loses lm, clamped to [LMIN, LMAX]; a cell is updated at most once per scan.
const ghostScan=(S,p)=>{ S.t++; S.ox=gObjX(S.t,p.sp);
  for(let b=0;b<NB;b++){ const a=2*Math.PI*(b+0.5)/NB, ux=Math.cos(a), uy=Math.sin(a);
    for(let r=0.4;r<40;r+=0.4){ const x=Math.floor(GS.x+ux*r), y=Math.floor(GS.y+uy*r); if(x<0||y<0||x>=GW||y>=GH) break;
      const c=y*GW+x, o=gStatic(x,y)||gObj(S,x,y);
      if(S.M[c]!==S.t){ S.M[c]=S.t; if(o){ S.N[c]=1; S.L[c]=Math.min(LMAX,S.L[c]+p.lh); } else S.L[c]=Math.max(LMIN,S.L[c]-p.lm); }
      if(o) break; } } };
// ghost = a cell the map calls occupied although neither the building nor the object is there now
const ghostCount=S=>{ let gn=0, go=0, sn=0, so=0;
  for(let y=0;y<GH;y++) for(let x=0;x<GW;x++){ const c=y*GW+x;
    if(gStatic(x,y)){ sn+=S.N[c]; if(S.L[c]>0) so++; } else if(!gObj(S,x,y)){ gn+=S.N[c]; if(S.L[c]>0) go++; } }
  return {gn,go,sn,so}; };

VIZ.w21_ghost=el=>{
  const p={sp:1,lh:0.85,lm:0.4}, C=10, OY=26, PX=[0,310]; let S=ghostNew(), playing=false, raf=0, tick=0;
  const svg=sv("svg",{viewBox:"0 0 600 240",role:"img","aria-label":"Two maps of the same room built from the same lidar scans while an object crosses it. Left: every point is kept. Right: an occupancy grid that also uses the free space along each beam"});
  const st=h("div",{class:"stats"}), out=h("div",{class:"out"});
  const replay=n=>{ S=ghostNew(); for(let k=0;k<n;k++) ghostScan(S,p); };
  const draw=()=>{ const d=["","","","",""], cell=(x,y)=>"M"+x*C+" "+(OY+y*C)+"h"+C+"v"+C+"h-"+C+"z";       // naive: solid, ghost · grid: free, solid, ghost
    for(let y=0;y<GH;y++) for(let x=0;x<GW;x++){ const c=y*GW+x, real=gStatic(x,y)||gObj(S,x,y);
      if(S.N[c]) d[real?0:1]+=cell(x,y);
      if(S.L[c]>0) d[real?3:4]+=cell(x,y); else if(S.L[c]<0) d[2]+=cell(x,y); }
    svg.innerHTML="";
    [["Keep every point",[[d[0],"var(--ink)"],[d[1],"var(--c3)"]]],["Occupancy grid: hits and misses",[[d[2],"var(--surface)"],[d[3],"var(--ink)"],[d[4],"var(--c3)"]]]].forEach(([t,L],k)=>{
      const g=sv("g",{transform:"translate("+PX[k]+" 0)"},sv("rect",{x:0,y:OY,width:GW*C,height:GH*C,fill:"var(--soft)"}));
      L.forEach(([dd,f])=>{ if(dd) g.append(sv("path",{d:dd,fill:f})); });
      g.append(sv("rect",{x:S.ox*C,y:OY+3*C,width:3*C,height:3*C,fill:"none",stroke:"var(--c2)","stroke-width":2.5}),sv("circle",{cx:GS.x*C,cy:OY+GS.y*C,r:5,fill:"var(--c5)"}));
      svg.append(g,txt(PX[k],17,t,{"font-weight":700})); });
    const n=ghostCount(S); let nh=0, nm=0;
    for(let l=LMIN;l<=0;l+=p.lh) nh++;                                            // hits that turn a long-free cell solid
    if(p.lm>0) for(let l=LMAX;l>0;l-=p.lm) nm++;                                  // misses that clear a saturated solid cell
    out.textContent="Scan "+S.t+" of "+TMAX+" · walls and pillar: "+n.sn+" cells in the point map, "+n.so+" in the grid";
    st.innerHTML=""; st.append(stat("Ghost cells, keep every point",n.gn+"","marked solid, but empty now"),stat("Ghost cells, occupancy grid",n.go+"","log-odds above 0, but empty now"),
      stat("Hits to accept · misses to clear",nh+" · "+(nm||"never"),"a long-free cell turns solid · a saturated solid cell is cleared")); };
  const setPlay=on=>{ playing=on; pb.textContent=on?"Pause":"Play"; pb.classList.toggle("on",on); cancelAnimationFrame(raf); if(on){ if(S.t>=TMAX) replay(0); raf=requestAnimationFrame(loop); } };
  const loop=()=>{ if(!playing) return; if(++tick%3===0){ ghostScan(S,p); draw(); if(S.t>=TMAX){ setPlay(false); return; } } raf=requestAnimationFrame(loop); };
  const pb=button("Play",()=>setPlay(!playing),"primary"), set=k=>x=>{ p[k]=x; replay(S.t); draw(); };
  el.append(h("div",{class:"vt"},"Ghost trail, and how free space erases it"),h("p",{class:"vd"},"A fixed lidar (blue dot) fires 180 beams per scan while a box crosses the room. Both maps get the same scans. A cell is updated at most once per scan, and its log-odds stay between −2 and +3.5."),
    h("div",{class:"ctl"},range("Object speed",0.2,3,0.1,p.sp,set("sp"),x=>x.toFixed(1)+" cells/scan").el,range("Hit update",0.2,2,0.05,p.lh,set("lh"),x=>"+"+x.toFixed(2)).el,range("Miss update",0,2,0.05,p.lm,set("lm"),x=>"−"+x.toFixed(2)).el),
    h("div",{class:"ctl"},pb,button("Step",()=>{ if(S.t<TMAX){ ghostScan(S,p); draw(); } }),button("Restart",()=>{ setPlay(false); replay(0); draw(); })),svg,
    legend([["var(--ink)","solid, and really there"],["var(--c3)","ghost: solid in the map, empty in the world"],["var(--c2)","where the object is now"],["var(--soft)","never observed"],["var(--surface)","observed free"]]),out,st);
  replay(30); draw();
};

// ---------- Day 4: a two-state Bayes filter for "is this object still where the map says?" ----------
// each visit: first the object survives with probability s, then D = detected, M = looked and not detected, N = not in view
const persistRun=(ev,pd,s,pf)=>{ let p=0.5; const out=[p];
  ev.forEach(e=>{ p*=s; if(e!=="N"){ const a=e==="D"?pd:1-pd, b=e==="D"?pf:1-pf; p=p*a/(p*a+(1-p)*b); } out.push(p); }); return out; };

VIZ.w21_persist=el=>{
  const PF=0.05, START="DDNNNMM", NAME={D:"detected",M:"looked, not detected",N:"not in view"}, SYM={D:"●",M:"○",N:"–"};
  let pd=0.8, s=0.95, ev=START.split("");
  const box=h("div"), out=h("div",{class:"out"}), st=h("div",{class:"stats"});
  const draw=()=>{ const P=persistRun(ev,pd,s,PF), n=Math.max(10,ev.length), dx=n>20?5:n>10?2:1, last=P[P.length-1];
    box.innerHTML=""; box.append(lineChart({h:250,aria:"Probability that the object is still there after each visit",x:{min:0,max:n,ticks:Array.from({length:Math.floor(n/dx)+1},(_,i)=>[i*dx,""+i*dx]),label:"visit"},
      y:{min:0,max:1,ticks:[[0,"0"],[0.5,"0.5"],[1,"1"]],label:"P(still there)"},series:[{name:"persistence",color:"var(--c5)",points:P.map((y,i)=>({x:i,y,label:i?"visit "+i+", "+NAME[ev[i-1]]:"before any visit",fmt:x=>x.toFixed(3)}))}]}));
    out.textContent="Visits: "+(ev.map(e=>SYM[e]).join(" ")||"none yet")+"     (● detected   ○ looked, not detected   – not in view)";
    st.innerHTML=""; st.append(stat("P(still there)",last.toFixed(3),"after "+ev.length+" visits"),stat("Map decision",last>=0.5?"keep":"remove","threshold 0.5"),
      stat("One miss multiplies the odds by",((1-pd)/(1-PF)).toFixed(2),"one detection by "+(pd/PF).toFixed(1))); };
  const add=e=>()=>{ if(ev.length<40) ev.push(e); draw(); };
  el.append(h("div",{class:"vt"},"Is the object still there?"),h("p",{class:"vd"},"A two-state Bayes filter. Before each visit the object stays with the survival probability. The detector fires on a present object with the recall below, and on an empty spot 5% of the time."),
    h("div",{class:"ctl"},range("Detector recall",0.5,0.99,0.01,pd,x=>{ pd=x; draw(); },x=>x.toFixed(2)).el,range("Survival between visits",0.8,1,0.01,s,x=>{ s=x; draw(); },x=>x.toFixed(2)).el),
    h("div",{class:"ctl"},button("Detected",add("D")),button("Looked, not detected",add("M")),button("Not in view",add("N")),button("Clear",()=>{ ev=[]; draw(); },"ghost"),button("Example",()=>{ ev=START.split(""); draw(); },"ghost")),box,out,st);
  draw();
};

// ---------- Day 5: a 9×2 deformation graph, left column pinned, one handle node moved by hand ----------
const arapSetup=(C,R)=>{ const q=[], nb=[];
  for(let r=0;r<R;r++) for(let c=0;c<C;c++) q.push([c,r]);
  q.forEach(([c,r],i)=>{ nb.push([]); if(c>0) nb[i].push(i-1); if(c<C-1) nb[i].push(i+1); if(r>0) nb[i].push(i-C); if(r<R-1) nb[i].push(i+C); });
  const hd=C*Math.ceil(R/2)-1, fix=i=>i%C===0||i===hd, free=q.map((_,i)=>i).filter(i=>!fix(i)), at=new Map(free.map((i,k)=>[i,k]));
  const L=M.zeros(free.length,free.length); free.forEach((i,k)=>{ L[k][k]=nb[i].length; nb[i].forEach(j=>{ if(at.has(j)) L[k][at.get(j)]=-1; }); });
  return {C,q,nb,hd,fix,free,Li:M.inv(L)}; };
// local step: the rotation (cos, sin) at each node that best maps its rest edges onto its current edges
const arapLocal=(G,p)=>G.q.map((qi,i)=>{ let d=0, x=0;
  G.nb[i].forEach(j=>{ const ex=qi[0]-G.q[j][0], ey=qi[1]-G.q[j][1], fx=p[i][0]-p[j][0], fy=p[i][1]-p[j][1]; d+=ex*fx+ey*fy; x+=ex*fy-ey*fx; });
  const m=Math.hypot(d,x); return m>1e-12?[d/m,x/m]:[1,0]; });
// global step: with the rotations held (null = no rotation), the free nodes solve  sum_j (p_i - p_j) = sum_j (R_i + R_j)/2 (q_i - q_j)
const arapGlobal=(G,p,rot)=>{ const bx=[], by=[];
  G.free.forEach(i=>{ let sx=0, sy=0; G.nb[i].forEach(j=>{ const ex=G.q[i][0]-G.q[j][0], ey=G.q[i][1]-G.q[j][1], c=rot?(rot[i][0]+rot[j][0])/2:1, s=rot?(rot[i][1]+rot[j][1])/2:0;
      sx+=c*ex-s*ey; sy+=s*ex+c*ey; if(G.fix(j)){ sx+=p[j][0]; sy+=p[j][1]; } }); bx.push(sx); by.push(sy); });
  const X=M.mv(G.Li,bx), Y=M.mv(G.Li,by); G.free.forEach((i,k)=>{ p[i]=[X[k],Y[k]]; }); };
// E[i][j] = | (p_i - p_j) - R_i (q_i - q_j) |: how far edge i→j is from a rigid motion of its rest edge. The as-rigid-as-possible
// energy is the sum of their squares; rms is its square root per edge. len = [mean, worst] change of edge length (rest length 1).
const arapMis=(G,p)=>{ const R=arapLocal(G,p), E=G.q.map(()=>({})); let s=0, n=0, ls=0, lw=0;
  G.nb.forEach((l,i)=>l.forEach(j=>{ const ex=G.q[i][0]-G.q[j][0], ey=G.q[i][1]-G.q[j][1], fx=p[i][0]-p[j][0], fy=p[i][1]-p[j][1], [c,sn]=R[i];
    const m=Math.hypot(fx-(c*ex-sn*ey),fy-(sn*ex+c*ey)), e=Math.abs(Math.hypot(fx,fy)-1); E[i][j]=m; s+=m*m; n++; ls+=e; lw=Math.max(lw,e); }));
  return {rms:Math.sqrt(s/n),E,len:[ls/n,lw]}; };
const arapAll=(G,hx,hy,warm,its)=>{ const base=()=>{ const p=G.q.map(v=>v.slice()); p[G.hd]=[hx,hy]; return p; }, none=base(), smooth=base();
  arapGlobal(G,smooth,null); const arap=warm||smooth.map(v=>v.slice()); arap[G.hd]=[hx,hy];
  for(let k=0;k<its;k++) arapGlobal(G,arap,arapLocal(G,arap));
  return {none,smooth,arap}; };

VIZ.w21_arap=el=>{
  const G=arapSetup(9,2), sc=52, X=p=>60+p[0]*sc, Y=p=>70+p[1]*sc, H0=[7.2,4];
  let hx=H0[0], hy=H0[1], mode="arap", R=arapAll(G,hx,hy,null,400);
  const svg=sv("svg",{viewBox:"0 0 600 340",role:"img","aria-label":"A ladder-shaped strip of 18 graph nodes, pinned at its left end, deformed by moving one handle node at its right end. The rest shape is shown dashed"});
  const g=sv("g"), dot=sv("circle",{r:11,fill:"var(--c2)",stroke:"var(--surface)","stroke-width":2,"pointer-events":"none"}), grab=sv("circle",{r:30,fill:"transparent"}), st=h("div",{class:"stats"});
  svg.append(g,dot,grab);
  const draw=()=>{ const p=R[mode], Q=arapMis(G,p); g.innerHTML="";
    G.nb.forEach((l,i)=>l.forEach(j=>{ if(j>i) g.append(sv("line",{x1:X(G.q[i]),y1:Y(G.q[i]),x2:X(G.q[j]),y2:Y(G.q[j]),stroke:"var(--line2)","stroke-dasharray":"4 4"})); }));
    G.nb.forEach((l,i)=>l.forEach(j=>{ if(j>i) g.append(sv("line",{x1:X(p[i]).toFixed(1),y1:Y(p[i]).toFixed(1),x2:X(p[j]).toFixed(1),y2:Y(p[j]).toFixed(1),stroke:Math.max(Q.E[i][j],Q.E[j][i])>0.1?"var(--c3)":"var(--c1)","stroke-width":3})); }));
    p.forEach((v,i)=>{ if(i%G.C===0) g.append(sv("rect",{x:X(v)-6,y:Y(v)-6,width:12,height:12,fill:"var(--ink)"})); else if(i!==G.hd) g.append(sv("circle",{cx:X(v).toFixed(1),cy:Y(v).toFixed(1),r:4.5,fill:"var(--ink)"})); });
    [dot,grab].forEach(c=>{ c.setAttribute("cx",X([hx,hy]).toFixed(1)); c.setAttribute("cy",Y([hx,hy]).toFixed(1)); });
    st.innerHTML=""; [["none","No regularizer"],["smooth","Neighbours move alike"],["arap","As rigid as possible"]].forEach(([k,t])=>{ const q=k===mode?Q:arapMis(G,R[k]);
      st.append(stat(t,(100*q.rms).toFixed(0)+"%","departure from rigid · edge lengths change "+(100*q.len[0]).toFixed(1)+"% on average")); }); };
  drag(svg,grab,(x,y)=>{ hx=clamp((x-60)/sc,1.5,10); hy=clamp((y-70)/sc,-1,4.6); R=arapAll(G,hx,hy,R.arap,40); draw(); });
  const sel=select("Regularizer drawn",[["arap","as rigid as possible"],["smooth","neighbours move alike"],["none","none"]],v=>{ mode=v; draw(); });
  el.append(h("div",{class:"vt"},"A deformation graph you can pull"),h("p",{class:"vd"},"18 nodes, 25 edges of rest length 1. The two squares are pinned. Drag the round handle: the other 15 nodes are placed by the chosen regularizer. The big number is the root-mean-square mismatch between an edge and the best local rotation of its rest edge."),
    h("div",{class:"ctl"},sel.el,button("Reset",()=>{ hx=H0[0]; hy=H0[1]; R=arapAll(G,hx,hy,null,400); draw(); })),svg,
    legend([["var(--c1)","edge moves rigidly with its neighbours (mismatch under 10%)"],["var(--c3)","edge does not"],["var(--line2)","rest shape"]]),st);
  draw();
};
