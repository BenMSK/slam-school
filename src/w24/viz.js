// Week 24: the computational structure of Spatial AI.
const w24leg=items=>h("div",{class:"legend"},...items.map(([c,t])=>h("span",{},h("i",{style:"background:"+c}),t)));
const w24tog=(label,get,set)=>{ const b=button(label,()=>{ set(!get()); b.classList.toggle("on",get()); }); b.classList.toggle("on",get()); return b; };

// ---- Day 1: frame-to-frame odometry against tracking on a persistent map (1-D corridor) ----
// The camera sweeps back and forth over landmarks 0.5 apart and sees those within 1.5 of itself, each as (landmark - camera) + noise.
const w24sweep=(n,sig,seed)=>{ const r=rng(seed), L=Array.from({length:21},(_,j)=>j*0.5), tx=k=>5-4*Math.cos(2*Math.PI*k/100);
  const map=new Map(); let vo=tx(0), cl=tx(0), prev=null; const eV=[], eC=[], seen=[];
  for(let k=0;k<n;k++){ const x=tx(k), z=new Map(); L.forEach((l,j)=>{ if(Math.abs(l-x)<=1.5) z.set(j,l-x+sig*r.n()); });
    if(prev){ let s=0, c=0; z.forEach((v,j)=>{ if(prev.has(j)){ s+=prev.get(j)-v; c++; } }); vo+=c?s/c:0;       // odometry: motion from the landmarks both frames share
      s=0; c=0; z.forEach((v,j)=>{ if(map.has(j)){ s+=map.get(j)-v; c++; } }); if(c) cl=s/c; }                   // closed loop: pose from the stored map
    z.forEach((v,j)=>{ if(!map.has(j)) map.set(j,cl+v); });                                                      // new landmarks enter the map once, at the current estimate
    prev=z; eV.push(Math.abs(vo-x)); eC.push(Math.abs(cl-x)); seen.push(map.size); }
  return {eV,eC,seen}; };
VIZ.w24_loop=el=>{ let sig=0.05, seed=1; const box=h("div"), st=h("div",{class:"stats"});
  const draw=()=>{ const n=400, R=w24sweep(n,sig,seed), top=Math.max(0.1,Math.ceil(Math.max(...R.eV,...R.eC)*10)/10), mean=(a,i0)=>a.slice(i0).reduce((s,v)=>s+v,0)/(a.length-i0);
    box.innerHTML=""; box.append(lineChart({h:260,aria:"Absolute position error against frame number for frame-to-frame odometry and for tracking against a persistent map",
      x:{min:0,max:n,ticks:[0,50,100,200,300,400].map(v=>[v,""+v]),label:"frame (the camera returns to the start every 100 frames)"},
      y:{min:0,max:top,ticks:[0,top/2,top].map(v=>[v,v.toFixed(2)]),label:"position error"},
      series:[{name:"frame-to-frame odometry (nothing kept)",color:"var(--c3)",dots:false,points:R.eV.map((y,x)=>({x,y}))},{name:"tracking against the persistent map",color:"var(--c1)",dots:false,points:R.eC.map((y,x)=>({x,y}))}]}));
    st.innerHTML=""; st.append(stat("Mean error, frames 300–399",mean(R.eV,300).toFixed(3),"odometry"),stat("Mean error, frames 300–399",mean(R.eC,300).toFixed(3),"persistent map"),stat("Map complete at frame",""+R.seen.findIndex(v=>v===21),"21 landmarks stored")); };
  el.append(h("div",{class:"vt"},"Incremental estimation against a closed loop"),h("p",{class:"vd"},"A camera sweeps a corridor four times. Both estimators get exactly the same noisy landmark measurements."),
    h("div",{class:"ctl"},range("Measurement noise",0.01,0.2,0.01,sig,v=>{ sig=v; draw(); },v=>v.toFixed(2)).el,button("New noise",()=>{ seed++; draw(); })),box,st); draw(); };

// ---- Day 2: what C V^2 f says about power as transistors shrink ----
VIZ.w24_dennard=el=>{ let g=6, mode="d"; const box=h("div"), st=h("div",{class:"stats"});
  // per generation every length shrinks by 1/sqrt(2): k = 2^(g/2). C ~ 1/k. Voltage ~ 1/k (Dennard) or fixed. Clock ~ k or frozen.
  const calc=(g,m)=>{ const k=Math.pow(2,g/2), V=m==="d"?1/k:1, f=m==="f"?1:k, P=(1/k)*V*V*f; return {k,V,f,P,dens:P*k*k}; };
  const names={d:"Dennard: voltage shrinks with size",v:"voltage stuck, clock still rising",f:"voltage stuck, clock frozen"}, x=v=>v===1?"×1":v>=10?"×"+v.toFixed(0):v>=1?"×"+v.toFixed(1):"×"+v.toPrecision(2);
  const draw=()=>{ const c=calc(g,mode); box.innerHTML=""; box.append(lineChart({h:250,aria:"Power density against the number of transistor generations for three scaling regimes, on a base-2 logarithmic axis",
      x:{min:0,max:10,ticks:[0,2,4,6,8,10].map(v=>[v,""+v]),label:"generations (each doubles transistor density)"},y:{min:-0.5,max:10,ticks:[0,5,10].map(v=>[v,"×"+Math.pow(2,v)])},
      series:["d","v","f"].map((m,i)=>({name:names[m],color:COL[[0,2,1][i]],dots:false,points:Array.from({length:11},(_,q)=>({x:q,y:Math.log2(calc(q,m).dens)}))})),marker:{x:g,y:Math.log2(c.dens)}}));
    st.innerHTML=""; st.append(stat("Transistors per mm²",x(c.k*c.k)),stat("Clock frequency",x(c.f)),stat("Power per transistor",x(c.P),"C·V²·f"),stat("Power per mm²",x(c.dens),c.dens>1.01?"heat to remove grows":"unchanged")); };
  el.append(h("div",{class:"vt"},"Shrinking transistors: where the power goes"),h("p",{class:"vd"},"Switching power per transistor is C·V²·f. The chart shows power per mm² (log scale), relative to generation 0."),
    h("div",{class:"ctl"},range("Generations",0,10,1,g,v=>{ g=v; draw(); }).el,select("Regime ",Object.entries(names),v=>{ mode=v; draw(); }).el),box,st); draw(); };

// ---- Day 3: a global estimate from purely local message passing (average consensus on a pixel grid) ----
const w24links=K=>{ const r=rng(11), out=[]; while(out.length<K){ const a=Math.floor(r.u()*64), b=Math.floor(r.u()*64); if(Math.abs(a%8-b%8)>=4) out.push([a,b]); } return out; };   // links that cross at least half the image width
const w24graph=K=>{ const nb=Array.from({length:64},()=>[]), link=(a,b)=>{ if(!nb[a].includes(b)){ nb[a].push(b); nb[b].push(a); } };
  for(let i=0;i<64;i++){ if(i%8<7) link(i,i+1); if(i<56) link(i,i+8); } w24links(K).forEach(([a,b])=>link(a,b)); return nb; };
const w24diam=nb=>{ let d=0; for(let s=0;s<nb.length;s++){ const dist=new Array(nb.length).fill(-1); dist[s]=0; const q=[s]; for(let i=0;i<q.length;i++) for(const j of nb[q[i]]) if(dist[j]<0){ dist[j]=dist[q[i]]+1; q.push(j); } d=Math.max(d,...dist); } return d; };
// one round: every pixel moves toward each neighbour by the Metropolis weight 1/(1+max degree). Symmetric weights keep the sum, so the fixed point is the mean.
const w24round=(x,nb)=>x.map((v,i)=>v+nb[i].reduce((s,j)=>s+(x[j]-v)/(1+Math.max(nb[i].length,nb[j].length)),0));
const w24run=(x0,nb,T)=>{ const sp=[], xs=[x0]; let x=x0; for(let t=0;t<=T;t++){ sp.push(Math.max(...x)-Math.min(...x)); if(t<T){ x=w24round(x,nb); xs.push(x); } } return {sp,xs}; };
VIZ.w24_consensus=el=>{ const T=300, r=rng(5), x0=Array.from({length:64},(_,i)=>2+0.4*(i%8-3.5)+0.4*r.n()), mean=x0.reduce((s,v)=>s+v,0)/64, lo=Math.min(...x0), hi=Math.max(...x0); let it=20, K=0;
  const base=w24run(x0,w24graph(0),T), svg=sv("svg",{viewBox:"0 0 600 300",role:"img","aria-label":"Left: an 8 by 8 grid of pixels, each showing its current estimate of the image shift. Right: the spread of the estimates against the number of message-passing rounds, on a logarithmic axis."}), st=h("div",{class:"stats"});
  const draw=()=>{ const nb=w24graph(K), R=K?w24run(x0,nb,T):base, x=R.xs[it], c=34, ox=14, oy=14, cx=i=>ox+(i%8)*c+c/2, cy=i=>oy+(i>>3)*c+c/2; svg.innerHTML="";
    x.forEach((v,i)=>{ svg.append(sv("rect",{x:ox+(i%8)*c,y:oy+(i>>3)*c,width:c-2,height:c-2,rx:4,fill:"var(--c5)","fill-opacity":(0.08+0.8*Math.min(1,Math.max(0,(v-lo)/(hi-lo)))).toFixed(2)}),txt(cx(i)-1,cy(i)+3,v.toFixed(1),{"text-anchor":"middle","font-size":11,fill:"var(--ink)"})); });
    w24links(K).forEach(([a,b])=>svg.append(sv("line",{x1:cx(a)-1,y1:cy(a)-1,x2:cx(b)-1,y2:cy(b)-1,stroke:"var(--c2)","stroke-width":1.5,opacity:0.8})));
    const X=t=>330+t/T*250, Y=s=>30+(1-Math.log10(Math.max(s,1e-4)))/5*220, line=(sp,col)=>sv("path",{d:sp.map((s,t)=>(t?"L":"M")+X(t).toFixed(1)+" "+Y(s).toFixed(1)).join(" "),fill:"none",stroke:col,"stroke-width":2});
    [1,-1,-3].forEach(e=>svg.append(sv("line",{x1:330,x2:580,y1:Y(Math.pow(10,e)),y2:Y(Math.pow(10,e)),stroke:"var(--line)"}),txt(324,Y(Math.pow(10,e))+4,e===1?"10":e===-1?"0.1":"0.001",{"text-anchor":"end","font-size":11,class:"mut"})));
    [0,100,200,300].forEach(t=>svg.append(txt(X(t),268,""+t,{"text-anchor":"middle","font-size":11,class:"mut"})));
    svg.append(txt(455,288,"rounds of message passing",{"text-anchor":"middle","font-size":11,class:"mut"}),txt(330,18,"spread = largest − smallest estimate",{"font-size":11,class:"mut"}),line(base.sp,"var(--line2)"));
    if(K) svg.append(line(R.sp,"var(--c2)")); svg.append(sv("circle",{cx:X(it),cy:Y(R.sp[it]),r:6,fill:"var(--acc)",stroke:"var(--surface)","stroke-width":2}));
    const need=R.xs.findIndex(v=>v.every(q=>Math.abs(q-mean)<0.05));
    st.innerHTML=""; st.append(stat("Global answer (mean of all 64)",mean.toFixed(3),"what a central sum gives"),stat("Spread after "+it+" rounds",R.sp[it].toFixed(3)),stat("Rounds until all within 0.05",need<0?"more than "+T:""+need),stat("Longest shortest path",w24diam(nb)+" hops",K?"with "+K+" long links":"grid only")); };
  el.append(h("div",{class:"vt"},"A global estimate from local messages"),h("p",{class:"vd"},"Each of 64 pixels starts with its own local estimate of one global number, the image shift. The estimates run low on the left and high on the right. Every round, each pixel averages with its neighbours."),
    h("div",{class:"ctl"},range("Rounds",0,T,1,it,v=>{ it=v; draw(); }).el,range("Long-range links",0,16,1,K,v=>{ K=v; draw(); }).el),svg,w24leg([["var(--line2)","grid links only"],["var(--c2)","with the long links"]]),st); draw(); };

// ---- Day 4: Gaussian belief propagation on a 1-D pose chain ----
const W24N=9, W24B=1.5;                                   // nine poses; Huber threshold in sigmas
const w24factors=(loop,outlier)=>{ const r=rng(4), truth=[0,1,2,3,4,3,2,1,0], F=[{v:[0],z:0,s:0.1}];
  for(let k=0;k<8;k++) F.push({v:[k,k+1],z:truth[k+1]-truth[k]+0.3*r.n(),s:0.3});
  const lc=0.1*r.n(); if(loop) F.push({v:[0,8],z:lc,s:0.1}); if(outlier) F.push({v:[2,6],z:2.5,s:0.4}); return {F,truth}; };
// exact answer: assemble the information matrix and vector, then M.solve. wts are the per-factor Huber weights (all 1 for plain least squares)
const w24exact=(F,wts)=>{ const L=M.zeros(W24N,W24N), e=new Array(W24N).fill(0);
  F.forEach((f,a)=>{ const w=wts[a]/(f.s*f.s); if(f.v.length===1){ L[f.v[0]][f.v[0]]+=w; e[f.v[0]]+=w*f.z; } else { const [i,j]=f.v; L[i][i]+=w; L[j][j]+=w; L[i][j]-=w; L[j][i]-=w; e[i]-=w*f.z; e[j]+=w*f.z; } });
  const mu=M.solve(L,e), S=M.inv(L); return {mu,sd:S.map((r,i)=>Math.sqrt(r[i]))}; };
const w24huber=(F,mu)=>F.map(f=>{ if(f.v.length===1) return 1; const m=Math.abs(mu[f.v[1]]-mu[f.v[0]]-f.z)/f.s; return m>W24B?W24B/m:1; });
const w24ref=(F,robust)=>{ let w=F.map(()=>1), X=w24exact(F,w); if(robust) for(let t=0;t<100;t++){ w=w24huber(F,X.mu); X=w24exact(F,w); } return X; };   // IRLS for the Huber problem
// GBP. msg[a][s] = [eta, lambda] sent by factor a to its s-th variable. A variable's belief is the sum of what it receives;
// what it sends back to factor a is its belief minus what a sent.
const w24gbp=(F,iters,random,robust,seed)=>{ const r=rng(seed), msg=F.map(f=>f.v.map(()=>[0,0])), hist=[];
  const belief=()=>{ const b=Array.from({length:W24N},()=>[0,0]); F.forEach((f,a)=>f.v.forEach((i,s)=>{ b[i][0]+=msg[a][s][0]; b[i][1]+=msg[a][s][1]; })); return b; };
  const send=(a,b)=>{ const f=F[a]; let w=1/(f.s*f.s); if(f.v.length===1) return [[w*f.z,w]];
    const [i,j]=f.v, vi=[b[i][0]-msg[a][0][0],b[i][1]-msg[a][0][1]], vj=[b[j][0]-msg[a][1][0],b[j][1]-msg[a][1][1]];
    if(robust&&b[i][1]>1e-9&&b[j][1]>1e-9){ const m=Math.abs(b[j][0]/b[j][1]-b[i][0]/b[i][1]-f.z)/f.s; if(m>W24B) w*=W24B/m; }
    // factor: eta = w z [-1, 1], Lambda = w [[1,-1],[-1,1]]. Add the other variable's message to its diagonal, then take the Schur complement.
    return [[-w*f.z+w*(vj[0]+w*f.z)/(w+vj[1]), w-w*w/(w+vj[1])], [w*f.z+w*(vi[0]-w*f.z)/(w+vi[1]), w-w*w/(w+vi[1])]]; };
  hist.push(belief());
  for(let t=0;t<iters;t++){
    if(random) for(let q=0;q<F.length;q++){ const a=Math.floor(r.u()*F.length); msg[a]=send(a,belief()); }
    else { const b=belief(), nu=F.map((_,a)=>send(a,b)); nu.forEach((m,a)=>{ msg[a]=m; }); }
    hist.push(belief()); }
  return hist; };
VIZ.w24_gbp=el=>{ const TMAX=60; let it=3, loop=true, outlier=false, robust=false, random=false, timer=0;
  const svg=sv("svg",{viewBox:"0 0 600 346",role:"img","aria-label":"Top: position of nine poses with the belief of Gaussian belief propagation as a dot with a one-sigma bar, next to the exact solution. Bottom: largest error of the belief means against iteration, on a logarithmic axis."}), st=h("div",{class:"stats"});
  const draw=()=>{ const {F,truth}=w24factors(loop,outlier), X=w24ref(F,robust), H=w24gbp(F,TMAX,random,robust,7), b=H[it];
    const mu=b.map(q=>q[1]>1e-9?q[0]/q[1]:null), sd=b.map(q=>q[1]>1e-9?1/Math.sqrt(q[1]):null);
    const err=H.map(B=>B.every(q=>q[1]>1e-9)?Math.max(...B.map((q,i)=>Math.abs(q[0]/q[1]-X.mu[i]))):null);
    const px=k=>70+k*60, py=v=>190-(v+1)*27, clampY=v=>Math.max(8,Math.min(200,py(v))); svg.innerHTML="";
    [0,2,4].forEach(v=>svg.append(sv("line",{x1:46,x2:580,y1:py(v),y2:py(v),stroke:"var(--line)"}),txt(40,py(v)+4,""+v,{"text-anchor":"end","font-size":11,class:"mut"})));
    for(let k=0;k<W24N;k++) svg.append(txt(px(k),214,"x"+k,{"text-anchor":"middle","font-size":11,class:"mut"}));
    svg.append(txt(46,12,"position (m)",{"font-size":11,class:"mut"}),sv("path",{d:truth.map((v,k)=>(k?"L":"M")+px(k)+" "+py(v)).join(" "),fill:"none",stroke:"var(--line2)","stroke-dasharray":"5 4","stroke-width":1.5}));
    for(let k=0;k<W24N;k++){ svg.append(sv("line",{x1:px(k)-9,x2:px(k)-9,y1:clampY(X.mu[k]-X.sd[k]),y2:clampY(X.mu[k]+X.sd[k]),stroke:"var(--c2)","stroke-width":2}),sv("circle",{cx:px(k)-9,cy:clampY(X.mu[k]),r:4,fill:"var(--surface)",stroke:"var(--c2)","stroke-width":2}));
      if(mu[k]!=null) svg.append(sv("line",{x1:px(k)+9,x2:px(k)+9,y1:clampY(mu[k]-sd[k]),y2:clampY(mu[k]+sd[k]),stroke:"var(--c5)","stroke-width":2}),sv("circle",{cx:px(k)+9,cy:clampY(mu[k]),r:4.5,fill:"var(--c5)"}));
      else svg.append(txt(px(k)+9,py(-0.6),"?",{"text-anchor":"middle","font-size":13,fill:"var(--c5)"})); }
    const ex=t=>70+t/TMAX*510, ey=e=>254+(0-Math.log10(Math.min(1,Math.max(e,1e-6))))/6*66;
    [0,-3,-6].forEach(p=>svg.append(sv("line",{x1:70,x2:580,y1:ey(Math.pow(10,p)),y2:ey(Math.pow(10,p)),stroke:"var(--line)"}),txt(64,ey(Math.pow(10,p))+4,p?"1e"+p:"1",{"text-anchor":"end","font-size":11,class:"mut"})));
    [0,20,40,60].forEach(t=>svg.append(txt(ex(t),338,""+t,{"text-anchor":"middle","font-size":11,class:"mut"})));
    svg.append(txt(325,240,"largest |mean − reference| over the nine poses, by iteration",{"text-anchor":"middle","font-size":11,class:"mut"}));
    let d=""; err.forEach((e,t)=>{ if(e!=null) d+=(d?"L":"M")+ex(t).toFixed(1)+" "+ey(e).toFixed(1)+" "; });
    if(d) svg.append(sv("path",{d,fill:"none",stroke:"var(--c5)","stroke-width":2}));
    svg.append(sv("line",{x1:ex(it),x2:ex(it),y1:254,y2:320,stroke:"var(--acc)","stroke-width":1.5}));
    const have=mu.filter(v=>v!=null).length;
    st.innerHTML=""; st.append(stat("Iteration",""+it,have+" of 9 poses have a belief"),stat("Largest mean error",err[it]==null?"—":err[it]<1e-6?"below 1e-6":err[it].toPrecision(2)+" m",robust?"against the Huber estimate":"against the exact solution"),
      stat("Std. dev. of x4: GBP / exact",(sd[4]==null?"—":sd[4].toFixed(3))+" / "+X.sd[4].toFixed(3),"metres"),stat("x8 mean: GBP / reference",(mu[8]==null?"—":mu[8].toFixed(3))+" / "+X.mu[8].toFixed(3),"true value 0")); };
  const sl=range("Iteration",0,TMAX,1,it,v=>{ it=v; draw(); }), setIt=v=>{ it=Math.max(0,Math.min(TMAX,v)); sl.input.value=it; sl.input.dispatchEvent(new Event("input")); };
  const play=button("Play",()=>{ if(timer){ cancelAnimationFrame(timer); timer=0; play.textContent="Play"; return; } if(it>=TMAX) setIt(0); let last=0; play.textContent="Pause";
    const tick=ts=>{ if(ts-last>140){ last=ts; setIt(it+1); } if(it<TMAX) timer=requestAnimationFrame(tick); else { timer=0; play.textContent="Play"; } }; timer=requestAnimationFrame(tick); },"primary");
  el.append(h("div",{class:"vt"},"Gaussian belief propagation on a pose chain"),h("p",{class:"vd"},"A robot drives 4 m down a corridor and back: nine poses, a prior on x0, eight odometry factors (σ = 0.3 m), and a loop closure x8 − x0 (σ = 0.1 m). One iteration is one update per factor."),
    h("div",{class:"ctl"},sl.el,select("Schedule ",[["s","synchronous: all factors at once"],["r","random: one factor at a time"]],v=>{ random=v==="r"; draw(); }).el),
    h("div",{class:"ctl"},button("Step",()=>setIt(it+1)),play,button("Reset",()=>setIt(0),"ghost"),w24tog("Loop closure",()=>loop,v=>{ loop=v; draw(); }),w24tog("Outlier x6 − x2 = 2.5",()=>outlier,v=>{ outlier=v; draw(); }),w24tog("Robust (Huber) factors",()=>robust,v=>{ robust=v; draw(); })),
    svg,w24leg([["var(--c5)","GBP belief, mean ± 1σ"],["var(--c2)","reference solution, mean ± 1σ"],["var(--line2)","true positions"]]),st); draw(); };

// ---- Day 5: choosing a system from a multi-objective benchmark (Pareto front) ----
VIZ.w24_pareto=el=>{ const r=rng(9), P=Array.from({length:40},()=>{ const p=Math.pow(10,-0.7+2*r.u()); return {p,e:Math.min(3.9,(0.35+0.9/Math.sqrt(p))*(1+0.9*r.u()))}; });
  const front=P.filter(a=>!P.some(b=>b!==a&&b.p<=a.p&&b.e<=a.e&&(b.p<a.p||b.e<a.e))).sort((a,b)=>a.p-b.p); let lb=0;
  const svg=sv("svg",{viewBox:"0 0 600 300",role:"img","aria-label":"Scatter plot of 40 synthetic system configurations: power on a logarithmic horizontal axis, drift error on the vertical axis, with the Pareto front highlighted and a movable power budget"}), st=h("div",{class:"stats"});
  const X=p=>50+(Math.log10(p)+0.7)/2*530, Y=e=>256-e/4*236;
  const draw=()=>{ const B=Math.pow(10,lb), ok=front.filter(a=>a.p<=B), best=ok[ok.length-1]; svg.innerHTML="";
    [0,1,2,3,4].forEach(v=>svg.append(sv("line",{x1:50,x2:580,y1:Y(v),y2:Y(v),stroke:"var(--line)"}),txt(44,Y(v)+4,v+"%",{"text-anchor":"end","font-size":11,class:"mut"})));
    [0.2,0.5,1,2,5,10,20].forEach(v=>svg.append(txt(X(v),274,v+" W",{"text-anchor":"middle","font-size":11,class:"mut"})));
    svg.append(txt(315,293,"power (log scale)",{"text-anchor":"middle","font-size":11,class:"mut"}),txt(54,14,"drift, % of distance travelled",{"font-size":11,class:"mut"}),
      sv("rect",{x:X(B),y:20,width:580-X(B),height:236,fill:"var(--c3-soft)",opacity:0.6}),sv("line",{x1:X(B),x2:X(B),y1:20,y2:256,stroke:"var(--c3)","stroke-width":2}),
      sv("path",{d:front.map((a,i)=>(i?"H"+X(a.p).toFixed(1)+" V":"M"+X(a.p).toFixed(1)+" ")+Y(a.e).toFixed(1)).join(" "),fill:"none",stroke:"var(--c1)","stroke-width":1.5}));
    P.forEach(a=>svg.append(sv("circle",{cx:X(a.p),cy:Y(a.e),r:front.includes(a)?5:3.5,fill:front.includes(a)?"var(--c1)":"var(--line2)"})));
    if(best) svg.append(sv("circle",{cx:X(best.p),cy:Y(best.e),r:10,fill:"none",stroke:"var(--acc)","stroke-width":2.5}));
    st.innerHTML=""; st.append(stat("Configurations",""+P.length,front.length+" on the Pareto front"),stat("Power budget",B.toFixed(2)+" W"),
      stat("Best choice within budget",best?best.e.toFixed(2)+"% drift":"none fits",best?"uses "+best.p.toFixed(2)+" W":"raise the budget"),stat("Dominated, never worth picking",""+(P.length-front.length))); };
  el.append(h("div",{class:"vt"},"Picking a system from a Pareto front"),h("p",{class:"vd"},"Forty made-up configurations of one SLAM system, each scored on two metrics. Lower and further left is better."),
    h("div",{class:"ctl"},range("Power budget",-0.7,1.3,0.05,lb,v=>{ lb=v; draw(); },v=>Math.pow(10,v).toFixed(2)+" W").el),svg,w24leg([["var(--c1)","Pareto front: not beaten on both metrics by anything"],["var(--line2)","dominated"],["var(--c3)","over budget"]]),st); draw(); };
