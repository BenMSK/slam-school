// Week 6: outliers and robust estimation. Every line-fit widget fits y = a·x + b; a residual is the vertical distance r = y − a·x − b.
// True line: y = 0.5x + 1, inlier noise σ = 0.15. Outliers: three in four lie near a second line ("a look-alike wall"), one in four anywhere.
const fitData=(seed,N,frac)=>{ const r=rng(seed), P=[], nOut=Math.round(N*frac);
  for(let i=0;i<N-nOut;i++){ const x=0.4+9.2*r.u(); P.push({x,y:0.5*x+1+0.15*r.n(),out:false}); }
  for(let i=0;i<nOut;i++){ const x=0.4+9.2*r.u(); P.push({x,y:i%4===3?0.3+7.4*r.u():6.8-0.55*x+0.25*r.n(),out:true}); }
  return P; };
// weighted least-squares line: minimise Σ w_i (y_i − a x_i − b)²; null if the weighted points do not determine a line
const wls=(P,w)=>{ let s=0,sx=0,sy=0,sxx=0,sxy=0; P.forEach((p,i)=>{ const v=w?w[i]:1; s+=v; sx+=v*p.x; sy+=v*p.y; sxx+=v*p.x*p.x; sxy+=v*p.x*p.y; });
  const d=s*sxx-sx*sx; if(!(Math.abs(d)>1e-9)) return null; const a=(s*sxy-sx*sy)/d; return [a,(sy-a*sx)/s]; };
const res=(L,p)=>p.y-L[0]*p.x-L[1];
// robust losses in the book's scaling (quadratic loss is r², not r²/2); weight w = ψ/(2r), influence ψ = 2·r·w
const LOSS={
  quad:{name:"Quadratic",rho:r=>r*r,w:()=>1},
  huber:{name:"Huber",rho:(r,b)=>Math.abs(r)<=b?r*r:2*b*Math.abs(r)-b*b,w:(r,b)=>Math.abs(r)<=b?1:b/Math.abs(r)},
  gm:{name:"Geman-McClure",rho:(r,b)=>b*b*r*r/(b*b+r*r),w:(r,b)=>(b*b/(b*b+r*r))**2},
  tukey:{name:"Tukey's biweight",rho:(r,b)=>Math.abs(r)<=b?b*b/3*(1-(1-r*r/(b*b))**3):b*b/3,w:(r,b)=>Math.abs(r)<=b?(1-r*r/(b*b))**2:0},
  tls:{name:"Truncated quadratic",rho:(r,b)=>Math.min(r*r,b*b),w:(r,b)=>Math.abs(r)<=b?1:0}
};
// GNC surrogates, equations (3.24) and (3.25), with the weights that minimise w·r² + Φ_ρμ(w)
const GNC={
  tls:{rho:(r,b,mu)=>{ const q=r*r; return q<=mu/(mu+1)*b*b?q:q>=(mu+1)/mu*b*b?b*b:2*b*Math.abs(r)*Math.sqrt(mu*(mu+1))-mu*(b*b+q); },
    w:(r,b,mu)=>{ const q=r*r; return q<=mu/(mu+1)*b*b?1:q>=(mu+1)/mu*b*b?0:b*Math.sqrt(mu*(mu+1))/Math.abs(r)-mu; },
    mu0:(rm,b)=>rm<=b?100:b*b/(2*rm*rm-b*b),next:mu=>mu*1.4,done:mu=>mu>=100},
  gm:{rho:(r,b,mu)=>mu*b*b*r*r/(mu*b*b+r*r),w:(r,b,mu)=>(mu*b*b/(mu*b*b+r*r))**2,
    mu0:(rm,b)=>Math.max(1,2*rm*rm/(b*b)),next:mu=>Math.max(1,mu/1.4),done:mu=>mu<=1}
};
const irlsRun=(P,key,b,T)=>{ let L=wls(P); for(let t=0;t<T&&L;t++){ const n=wls(P,P.map(p=>LOSS[key].w(res(L,p),b))); if(!n) break; L=n; } return L; };

// shared drawing for the line-fit plots: world x in [0,10], y in [0,8]
const FX=x=>30+x*55, FY=y=>322-y*38, f2=v=>v.toFixed(2), lineStr=L=>L?"y = "+f2(L[0])+"x "+(L[1]<0?"− ":"+ ")+f2(Math.abs(L[1])):"—";
const fitSeg=(L,a)=>{ if(!L) return null; const [m,b]=L; let x0=0,x1=10;
  if(Math.abs(m)>1e-9){ const xa=-b/m, xb=(8-b)/m; x0=Math.max(x0,Math.min(xa,xb)); x1=Math.min(x1,Math.max(xa,xb)); } else if(b<0||b>8) return null;
  if(!(x0<x1)) return null;
  return sv("line",Object.assign({x1:FX(x0).toFixed(1),y1:FY(m*x0+b).toFixed(1),x2:FX(x1).toFixed(1),y2:FY(m*x1+b).toFixed(1),"stroke-width":2.5,"stroke-linecap":"round"},a)); };
const fitSvg=aria=>sv("svg",{viewBox:"0 0 600 340",role:"img","aria-label":aria});
const fitFrame=svg=>{ svg.innerHTML=""; svg.append(sv("rect",{x:FX(0),y:FY(8),width:550,height:304,fill:"none",stroke:"var(--line)"}));
  for(let x=2;x<=8;x+=2) svg.append(txt(FX(x),336,String(x),{"text-anchor":"middle",class:"mut","font-size":11}));
  for(let y=2;y<=6;y+=2) svg.append(txt(24,FY(y)+4,String(y),{"text-anchor":"end",class:"mut","font-size":11})); };
const add=(g,...els)=>els.forEach(e=>e&&g.append(e));            // fitSeg returns null when a line misses the plot
const legend=(lg,items)=>{ lg.innerHTML=""; items.forEach(([c,t])=>lg.append(h("span",{},h("i",{style:"background:"+c}),t))); };

// Day 1: one draggable bad point against N good ones
VIZ.w06_drag=el=>{
  let N=12, bad={x:8.5,y:1.2};
  const svg=fitSvg("Scatter plot of good points along a line and one bad point that can be dragged, with the least-squares line fitted with and without the bad point");
  const st=h("div",{class:"stats"}), lg=h("div",{class:"legend"}), dot=sv("circle",{r:11,fill:"var(--c3)",stroke:"var(--surface)","stroke-width":2}), layer=sv("g");
  svg.append(layer,dot);                                                  // the dot stays in the DOM so a drag is never interrupted by a redraw
  const good=()=>{ const r=rng(11), P=[]; for(let i=0;i<N;i++){ const x=0.8+8.4*(i+0.5)/N; P.push({x,y:0.5*x+1+0.15*r.n()}); } return P; };
  const draw=()=>{ const G=good(), A=G.concat([bad]), L0=wls(G), L1=wls(A), T=[0.5,1];
    fitFrame(layer); add(layer,fitSeg(L0,{stroke:"var(--c1)","stroke-dasharray":"7 5"}),fitSeg(L1,{stroke:"var(--c3)"}));
    G.forEach(p=>layer.append(sv("circle",{cx:FX(p.x),cy:FY(p.y),r:4.5,fill:"var(--c5)"})));
    dot.setAttribute("cx",FX(bad.x)); dot.setAttribute("cy",FY(bad.y));
    const rb=res(T,bad), sg=G.reduce((s,p)=>s+res(T,p)**2,0);
    st.innerHTML=""; st.append(stat("Least squares, good points only",lineStr(L0),"true line: y = 0.50x + 1.00"),stat("Least squares, all points",lineStr(L1),"one bad point among "+N+" good"),
      stat("Bad point's residual",f2(rb),"to the true line; inlier σ = 0.15"),stat("Its share of the cost",(100*rb*rb/(rb*rb+sg)).toFixed(1)+"%","of Σ r² evaluated at the true line")); };
  drag(svg,dot,(x,y)=>{ bad={x:Math.min(10,Math.max(0,(x-30)/55)),y:Math.min(8,Math.max(0,(322-y)/38))}; draw(); });
  legend(lg,[["var(--c5)","good points"],["var(--c3)","bad point (drag it) and the fit to all points"],["var(--c1)","fit to the good points only (dashed)"]]);
  el.append(h("div",{class:"vt"},"One bad point against many good ones"),h("p",{class:"vd"},"Least-squares line through N good points plus one point you can drag anywhere."),
    h("div",{class:"ctl"},range("Good points N",6,40,1,N,v=>{ N=v; draw(); }).el,button("Reset the bad point",()=>{ bad={x:8.5,y:1.2}; draw(); })),svg,lg,st);
  draw();
};

// Day 2: RANSAC for a line. Minimal set n = 2, minimal solver = the line through two points.
VIZ.w06_ransac=el=>{
  const N=40, MAXIT=30; let frac=0.4, g=0.4, seed=4, P, r, it, cur, best, raf=0, last=0;
  const svg=fitSvg("Scatter plot with inliers and outliers. Each RANSAC iteration draws a line through two sampled points, a band of half-width gamma around it, and the best line so far");
  const st=h("div",{class:"stats"}), lg=h("div",{class:"legend"});
  const score=L=>P.map((p,i)=>Math.abs(res(L,p))<=g?i:-1).filter(i=>i>=0);
  const reset=()=>{ P=fitData(seed,N,frac); r=rng(seed+77); it=0; cur=null; best=null; };
  const step=()=>{ const i=Math.floor(r.u()*N); let j=Math.floor(r.u()*(N-1)); if(j>=i) j++;
    if(Math.abs(P[j].x-P[i].x)<1e-9) return;                              // a vertical pair has no line y = ax + b: skip the draw
    const a=(P[j].y-P[i].y)/(P[j].x-P[i].x), L=[a,P[i].y-a*P[i].x]; cur={i,j,L,S:score(L)}; it++; if(!best||cur.S.length>best.S.length) best=cur; };
  const draw=()=>{ fitFrame(svg); const S=new Set(cur?cur.S:[]), nIn=P.filter(p=>!p.out).length, om=nIn/N;
    add(svg,fitSeg(wls(P),{stroke:"var(--c3)","stroke-dasharray":"7 5"}));
    if(cur){ [g,-g].forEach(d=>add(svg,fitSeg([cur.L[0],cur.L[1]+d],{stroke:"var(--c2)","stroke-width":1,"stroke-dasharray":"3 4"}))); add(svg,fitSeg(cur.L,{stroke:"var(--c2)","stroke-width":1.5})); }
    if(best) add(svg,fitSeg(best.L,{stroke:"var(--c1)"}));
    P.forEach((p,i)=>svg.append(sv("circle",{cx:FX(p.x),cy:FY(p.y),r:4.5,fill:S.has(i)?"var(--c2)":p.out?"var(--surface)":"var(--c5)",stroke:p.out?"var(--muted)":"none","stroke-width":1.5})));
    if(cur) [cur.i,cur.j].forEach(i=>svg.append(sv("circle",{cx:FX(P[i].x),cy:FY(P[i].y),r:10,fill:"none",stroke:"var(--ink)","stroke-width":2})));
    const fit=best?wls(best.S.map(i=>P[i])):null, wrong=best?best.S.filter(i=>P[i].out).length:0;
    st.innerHTML=""; st.append(stat("Iteration",it+" of "+MAXIT,cur?"this sample agrees with "+cur.S.length+" points":"press Step"),
      stat("Best consensus set",best?best.S.length+" of "+N:"—",best?wrong+" of them true outliers":"largest set so far"),
      stat("Refit on the consensus set",lineStr(fit),"true line: y = 0.50x + 1.00"),
      stat("Expected tries",om>0?(1/(om*om)).toFixed(1):"∞","1/ω² with ω = "+nIn+"/"+N+" inliers, minimal set n = 2")); };
  const pb=button("Play",()=>{ if(raf){ stop(); return; } if(it>=MAXIT){ reset(); } pb.textContent="Pause"; last=0; raf=requestAnimationFrame(tick); });
  const stop=()=>{ if(raf) cancelAnimationFrame(raf); raf=0; pb.textContent="Play"; };
  const tick=t=>{ if(t-last>350){ last=t; step(); draw(); } if(it>=MAXIT){ stop(); return; } raf=requestAnimationFrame(tick); };
  legend(lg,[["var(--c5)","true inlier"],["var(--muted)","true outlier (hollow)"],["var(--c2)","this sample's line, band ±γ, and the points inside it"],["var(--c1)","best line so far"],["var(--c3)","least squares on everything (dashed)"]]);
  el.append(h("div",{class:"vt"},"RANSAC on a line"),h("p",{class:"vd"},"Sample 2 points, draw the line through them, count the points within γ of it, keep the line with the largest count."),
    h("div",{class:"ctl"},range("Outliers",0,70,5,frac*100,v=>{ frac=v/100; stop(); reset(); draw(); },v=>v+"%").el,range("Inlier threshold γ",0.1,1.5,0.05,g,v=>{ g=v; if(cur) cur.S=score(cur.L); if(best) best.S=score(best.L); draw(); },f2).el),
    h("div",{class:"ctl"},button("Step",()=>{ stop(); if(it<MAXIT) step(); draw(); },"primary"),pb,button("Reset",()=>{ stop(); reset(); draw(); }),button("New data",()=>{ stop(); seed++; reset(); draw(); })),svg,lg,st);
  reset(); draw();
};

// Day 3: pairwise consistency graph and its maximum clique for 2-D point correspondences
VIZ.w06_pcm=el=>{
  const N=10; let g=0.15, nOut=4, seed=7;
  const svg=sv("svg",{viewBox:"0 0 600 300",role:"img","aria-label":"Left: ten numbered points seen in frame k minus 1 and their putative matches in frame k. Right: the consistency graph with one node per correspondence, an edge between each consistent pair, and the maximum clique highlighted"});
  const st=h("div",{class:"stats"}), lg=h("div",{class:"legend"});
  const draw=()=>{ const r=rng(seed*13+5), th=0.6, c=Math.cos(th), s=Math.sin(th), rot=p=>[c*p[0]-s*p[1],s*p[0]+c*p[1]], C=[];
    // points are drawn at least 0.55 apart in each frame so that their labels never overlap
    const pick=L=>{ let p; for(let k=0;k<80;k++){ p=[0.3+3.4*r.u(),0.3+3.4*r.u()]; if(L.every(q=>Math.hypot(p[0]-q[0],p[1]-q[1])>0.55)) break; } L.push(p); return p; };
    const As=[], Bs=[]; for(let i=0;i<N;i++) pick(As); As.forEach((a,i)=>{ if(i>=nOut) Bs.push(a); });
    for(let i=0;i<N;i++){ const a=As[i], out=i<nOut, q=rot(out?pick(Bs):a); C.push({a,b:out?q:[q[0]+0.03*r.n(),q[1]+0.03*r.n()],out}); }
    for(let i=N-1;i>0;i--){ const j=Math.floor(r.u()*(i+1)); [C[i],C[j]]=[C[j],C[i]]; }           // shuffle so the labels say nothing
    const d=(p,q)=>Math.hypot(p[0]-q[0],p[1]-q[1]), adj=new Array(N).fill(0); let edges=0;
    for(let i=0;i<N;i++) for(let j=i+1;j<N;j++) if(Math.abs(d(C[i].a,C[j].a)-d(C[i].b,C[j].b))<=g){ adj[i]|=1<<j; adj[j]|=1<<i; edges++; }
    // maximum clique by checking all 2^10 subsets (fine for 10 nodes; the general problem is NP-hard)
    let bestM=0, bestC=0, ties=0;
    for(let m=1;m<1<<N;m++){ let ok=true, cnt=0; for(let i=0;i<N&&ok;i++) if(m>>i&1){ cnt++; if(m&~adj[i]&~(1<<i)) ok=false; } if(!ok) continue; if(cnt>bestC){ bestC=cnt; bestM=m; ties=1; } else if(cnt===bestC) ties++; }
    const inQ=i=>bestM>>i&1, A=p=>[12+p[0]*34,228-p[1]*34], B=p=>[232+p[0]*34,254.5-p[1]*34];
    svg.innerHTML=""; svg.append(txt(80,16,"frame k − 1",{"text-anchor":"middle",class:"mut","font-size":12}),txt(250,16,"frame k",{"text-anchor":"middle",class:"mut","font-size":12}),txt(475,16,"consistency graph",{"text-anchor":"middle",class:"mut","font-size":12}));
    C.forEach((q,i)=>{ const a=A(q.a), b=B(q.b); svg.append(sv("line",{x1:a[0],y1:a[1],x2:b[0],y2:b[1],stroke:inQ(i)?"var(--c1)":"var(--line2)","stroke-width":inQ(i)?1.5:1})); });
    C.forEach((q,i)=>[A(q.a),B(q.b)].forEach(p=>svg.append(sv("circle",{cx:p[0],cy:p[1],r:8,fill:inQ(i)?"var(--c1-soft)":"var(--surface)",stroke:q.out?"var(--c3)":"var(--c5)","stroke-width":2}),txt(p[0],p[1]+4,String(i+1),{"text-anchor":"middle","font-size":11}))));
    const G=i=>[475+100*Math.cos(2*Math.PI*i/N-Math.PI/2),160+100*Math.sin(2*Math.PI*i/N-Math.PI/2)];
    for(let i=0;i<N;i++) for(let j=i+1;j<N;j++) if(adj[i]>>j&1){ const a=G(i), b=G(j), q=inQ(i)&&inQ(j); svg.append(sv("line",{x1:a[0],y1:a[1],x2:b[0],y2:b[1],stroke:q?"var(--c1)":"var(--line2)","stroke-width":q?2.5:1})); }
    C.forEach((q,i)=>{ const a=G(i); svg.append(sv("circle",{cx:a[0],cy:a[1],r:13,fill:inQ(i)?"var(--c1-soft)":"var(--surface)",stroke:q.out?"var(--c3)":"var(--c5)","stroke-width":2.5}),txt(a[0],a[1]+4,String(i+1),{"text-anchor":"middle","font-size":12})); });
    const ids=f=>C.map((q,i)=>f(q,i)?i+1:0).filter(Boolean).join(", ")||"none";
    st.innerHTML=""; st.append(stat("Consistent pairs (edges)",edges+" of "+N*(N-1)/2,"pairs with |d(k−1) − d(k)| ≤ γ"),stat("Maximum clique",bestC+" nodes: "+ids((q,i)=>inQ(i)),ties>1?ties+" cliques tie at this size; the first is shown":"unique"),
      stat("True inliers",ids(q=>!q.out),(N-nOut)+" of "+N),stat("Mistakes",C.filter((q,i)=>q.out&&inQ(i)).length+" outliers kept, "+C.filter((q,i)=>!q.out&&!inQ(i)).length+" inliers dropped","clique compared with the truth")); };
  legend(lg,[["var(--c5)","blue ring: true inlier"],["var(--c3)","pink ring: true outlier"],["var(--c1)","green: in the maximum clique"],["var(--line2)","grey edge: consistent pair outside the clique"]]);
  el.append(h("div",{class:"vt"},"Pairwise consistency and the maximum clique"),h("p",{class:"vd"},"Frame k is frame k − 1 rotated by an unknown angle, plus 0.03 of noise. Two matches are consistent if the distance between their points is the same in both frames, to within γ."),
    h("div",{class:"ctl"},range("Threshold γ",0.02,1.5,0.01,g,v=>{ g=v; draw(); },f2).el,range("Outliers among 10 matches",0,8,1,nOut,v=>{ nOut=v; draw(); }).el),
    h("div",{class:"ctl"},button("New scene",()=>{ seed++; draw(); })),svg,lg,st);
  draw();
};

// Day 4: loss ρ, influence ψ = ρ′ and IRLS weight w = ψ/(2r), side by side
VIZ.w06_loss=el=>{
  let key="huber", b=1, r0=2.5; const R=4, W=176, H=150, Y0=24;
  const svg=sv("svg",{viewBox:"0 0 600 200",role:"img","aria-label":"Three plots against the residual r from minus 4 to 4: the loss rho, the influence function psi, and the weight w, for the selected robust loss, with the quadratic loss dashed for comparison"});
  const st=h("div",{class:"stats"});
  const panel=(k,title,lo,hi,f,q)=>{ const x0=14+k*196, X=r=>x0+(r+R)/(2*R)*W, Y=v=>Y0+H-(v-lo)/(hi-lo)*H;
    const path=fn=>{ let d="", pen=false; for(let i=0;i<=160;i++){ const r=-R+i*2*R/160, v=fn(r); if(v<lo-1e-9||v>hi+1e-9){ pen=false; continue; } d+=(pen?"L":"M")+X(r).toFixed(1)+" "+Y(v).toFixed(1)+" "; pen=true; } return d; };
    svg.append(sv("rect",{x:x0,y:Y0,width:W,height:H,fill:"none",stroke:"var(--line)"}),sv("line",{x1:X(0),y1:Y0,x2:X(0),y2:Y0+H,stroke:"var(--line)"}),sv("line",{x1:x0,y1:Y(0),x2:x0+W,y2:Y(0),stroke:"var(--line2)"}),
      txt(x0+W/2,15,title,{"text-anchor":"middle","font-size":12}),txt(x0,Y0+H+15,"−4",{class:"mut","font-size":11}),txt(x0+W,Y0+H+15,"4",{"text-anchor":"end",class:"mut","font-size":11}),txt(X(0),Y0+H+15,"r",{"text-anchor":"middle",class:"mut","font-size":11}),
      sv("path",{d:path(q),fill:"none",stroke:"var(--muted)","stroke-width":1.5,"stroke-dasharray":"4 4"}),sv("path",{d:path(f),fill:"none",stroke:"var(--acc)","stroke-width":2.5}));
    [-b,b].forEach(v=>svg.append(sv("line",{x1:X(v),y1:Y0+H-8,x2:X(v),y2:Y0+H,stroke:"var(--c2)","stroke-width":2})));
    const v=f(r0); if(v>=lo&&v<=hi) svg.append(sv("circle",{cx:X(r0),cy:Y(v),r:5,fill:"var(--c3)",stroke:"var(--surface)","stroke-width":2})); };
  const draw=()=>{ const L=LOSS[key], rho=r=>L.rho(r,b), w=r=>L.w(r,b), psi=r=>2*r*w(r); svg.innerHTML="";
    panel(0,"loss ρ(r)",0,8,rho,r=>r*r); panel(1,"influence ψ(r) = ρ′(r)",-6,6,psi,r=>2*r); panel(2,"weight w(r) = ψ/(2r)",0,1.15,w,()=>1);
    st.innerHTML=""; st.append(stat("Loss at r = "+f2(r0),"ρ = "+f2(rho(r0)),"quadratic: "+f2(r0*r0)),stat("Influence","ψ = "+f2(psi(r0)),"quadratic: "+f2(2*r0)),stat("Weight","w = "+w(r0).toFixed(3),"quadratic: 1")); };
  const sel=select("Loss",Object.keys(LOSS).filter(k=>k!=="quad").map(k=>[k,LOSS[k].name]),v=>{ key=v; draw(); });
  el.append(h("div",{class:"vt"},"Robust-loss gallery"),h("p",{class:"vd"},"Each loss with its influence function and its IRLS weight. Dashed: the quadratic loss. Amber ticks mark r = ±β."),
    h("div",{class:"ctl"},sel.el,range("Parameter β",0.5,3,0.1,b,v=>{ b=v; draw(); },v=>v.toFixed(1)).el,range("Residual r",-4,4,0.1,r0,v=>{ r0=v; draw(); },v=>v.toFixed(1)).el),svg,st);
  draw();
};

// Days 4 and 5: IRLS, and GNC (IRLS on a surrogate ρ_μ whose μ changes every iteration), on the same line-fit data
const robustFit=(el,gnc)=>{
  const N=40; let key=gnc?"tls":"huber", b=0.5, frac=0.4, seed=4, P, w, L, L0, t, mu, mus, rm, done;
  const svg=fitSvg("Scatter plot of the data. Each point is drawn larger the higher its current weight. Lines: least squares, the current robust estimate"+(gnc?", and plain IRLS without graduation":""));
  const top=gnc?sv("svg",{viewBox:"0 0 600 150",role:"img","aria-label":"The surrogate loss rho mu against the residual for the current mu, the target loss dashed, and a tick on the axis at each measurement's current residual"}):null;
  const st=h("div",{class:"stats"}), lg=h("div",{class:"legend"});
  const reset=()=>{ P=fitData(seed,N,frac); w=P.map(()=>1); L=L0=wls(P); t=0; done=false; rm=Math.max(...P.map(p=>Math.abs(res(L,p)))); mu=gnc?GNC[key].mu0(rm,b):0; mus=mu; };
  const step=()=>{ if(done) return; const w2=P.map(p=>gnc?GNC[key].w(res(L,p),b,mu):LOSS[key].w(res(L,p),b)), n=wls(P,w2);
    if(!n){ done=true; return; }                                           // every weight is zero: nothing left to fit
    const moved=Math.abs(n[0]-L[0])+Math.abs(n[1]-L[1]); w=w2; L=n; t++; mus=mu;
    if(gnc){ if(GNC[key].done(mu)) done=true; else mu=GNC[key].next(mu); } else if(moved<1e-6) done=true; };
  const drawTop=()=>{ const R=1.5*rm, hi=1.3*b*b, X=r=>40+(r+R)/(2*R)*540, Y=v=>126-v/hi*110, G=GNC[key];
    const path=fn=>{ let d="", pen=false; for(let i=0;i<=240;i++){ const r=-R+i*2*R/240, v=fn(r); if(v>hi){ pen=false; continue; } d+=(pen?"L":"M")+X(r).toFixed(1)+" "+Y(v).toFixed(1)+" "; pen=true; } return d; };
    top.innerHTML=""; top.append(sv("line",{x1:40,y1:126,x2:580,y2:126,stroke:"var(--line2)"}),sv("line",{x1:40,y1:Y(b*b),x2:580,y2:Y(b*b),stroke:"var(--line)","stroke-dasharray":"2 4"}),
      txt(34,Y(b*b)+4,"β²",{"text-anchor":"end",class:"mut","font-size":11}),txt(34,130,"0",{"text-anchor":"end",class:"mut","font-size":11}),txt(310,146,"residual r",{"text-anchor":"middle",class:"mut","font-size":11}),
      sv("path",{d:path(r=>LOSS[key].rho(r,b)),fill:"none",stroke:"var(--muted)","stroke-width":1.5,"stroke-dasharray":"4 4"}),sv("path",{d:path(r=>G.rho(r,b,mus)),fill:"none",stroke:"var(--acc)","stroke-width":2.5}));
    P.forEach(p=>{ const r=res(L,p); top.append(sv("line",{x1:X(r),y1:126,x2:X(r),y2:134,stroke:p.out?"var(--c3)":"var(--c5)","stroke-width":1.5})); }); };
  const draw=()=>{ fitFrame(svg); const plain=gnc?irlsRun(P,key,b,50):null, rho=l=>l?P.reduce((s,p)=>s+LOSS[key].rho(res(l,p),b),0):NaN;
    add(svg,fitSeg(L0,{stroke:"var(--c3)","stroke-dasharray":"7 5"}),fitSeg(plain,{stroke:"var(--c2)","stroke-dasharray":"2 5"}),fitSeg(L,{stroke:"var(--c1)"}));
    P.forEach((p,i)=>svg.append(sv("circle",{cx:FX(p.x),cy:FY(p.y),r:(1.5+4*w[i]).toFixed(2),fill:p.out?"var(--surface)":"var(--c5)",stroke:p.out?"var(--muted)":"var(--c5)","stroke-width":1.5})));
    if(gnc) drawTop();
    const nOut=P.filter(p=>p.out).length, dn=P.filter((p,i)=>p.out&&w[i]<0.5).length, kept=P.filter((p,i)=>!p.out&&w[i]>=0.5).length;
    st.innerHTML=""; st.append(stat("Iteration",t+(done?" (stopped)":""),gnc?"μ = "+(mus>=100?mus.toFixed(0):mus>=1?mus.toFixed(2):mus.toPrecision(2))+(key==="tls"?", grows ×1.4":", shrinks ÷1.4 down to 1"):"weights from the last line, then a weighted fit"),
      stat("Current line",lineStr(L),"true line: y = 0.50x + 1.00"),stat("Weights",dn+" of "+nOut+" outliers below 0.5",kept+" of "+(N-nOut)+" inliers at 0.5 or more"),
      stat("Robust cost",f2(rho(L)),"Σρ(r), "+LOSS[key].name+"; "+(gnc?"plain IRLS ends at "+f2(rho(plain))+", "+lineStr(plain):"least squares starts at "+f2(rho(L0))))); };
  const sel=select("Loss",(gnc?["tls","gm"]:["huber","gm","tls"]).map(k=>[k,LOSS[k].name]),v=>{ key=v; reset(); draw(); });
  legend(lg,[["var(--c5)","true inlier"],["var(--muted)","true outlier (hollow); size = weight"],["var(--c3)","least squares (dashed)"],["var(--c1)",gnc?"GNC estimate":"IRLS estimate"]].concat(gnc?[["var(--c2)","plain IRLS, run to convergence (dotted)"]]:[]));
  el.append(h("div",{class:"vt"},gnc?"Graduated non-convexity":"Iteratively reweighted least squares"),
    h("p",{class:"vd"},gnc?"Top: the surrogate loss for the current μ (solid) and the target loss (dashed). Each Step does a weight update, a weighted fit, and a μ update.":"Starts from the least-squares line. Each Step recomputes every weight from its residual, then solves the weighted line fit."),
    h("div",{class:"ctl"},sel.el,range("Parameter β",0.2,2,0.05,b,v=>{ b=v; reset(); draw(); },f2).el,range("Outliers",0,50,5,frac*100,v=>{ frac=v/100; reset(); draw(); },v=>v+"%").el),
    h("div",{class:"ctl"},button("Step",()=>{ step(); draw(); },"primary"),button("Run to the end",()=>{ for(let i=0;i<80&&!done;i++) step(); draw(); }),button("Reset",()=>{ reset(); draw(); }),button("New data",()=>{ seed++; reset(); draw(); })),...(top?[top]:[]),svg,lg,st);
  reset(); draw();
};
VIZ.w06_irls=el=>robustFit(el,false);
VIZ.w06_gnc=el=>robustFit(el,true);
