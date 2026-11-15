// Week 10 widgets: range-only cost surface, a one-variable moment/SOS relaxation, CRLB simulation,
// Fisher information of a position graph (the main one), and a spanning-tree counter.
const f2=v=>v.toFixed(2), f3=v=>(Math.abs(v)<5e-4?0:v).toFixed(3);
const legend=items=>h("div",{class:"legend"},...items.map(([c,t])=>h("span",{},h("i",{style:"background:"+c}),t)));

// ---- Day 1: range-only localization. f(t) = sum_i (|t - a_i| - r_i)^2 is not quadratic and has several minima ----
VIZ.w10_range=el=>{
  const W=600, H=360, sc=60, NX=40, NY=24;                       // world 10 x 6, y up
  const A=[[1.5,3.2],[5,2.8],[8.5,3.1]], P=[6,4.8];
  let nA=3, start=[7.5,1.2], sig=0, seed=1;
  const X=x=>x*sc, Y=y=>H-y*sc;
  const meas=()=>{ const r=rng(seed); return A.slice(0,nA).map(a=>Math.max(0.05,Math.hypot(P[0]-a[0],P[1]-a[1])+sig*r.n())); };
  const cost=(t,r)=>r.reduce((s,ri,i)=>s+(Math.hypot(t[0]-A[i][0],t[1]-A[i][1])-ri)**2,0);
  // damped Gauss-Newton on the residuals e_i = |t - a_i| - r_i, with step halving so the cost never goes up
  const solve=(t0,r)=>{ let t=[...t0]; const path=[t];
    for(let it=0;it<80;it++){ const Hm=[[1e-3,0],[0,1e-3]], g=[0,0];
      r.forEach((ri,i)=>{ const dx=t[0]-A[i][0], dy=t[1]-A[i][1], d=Math.hypot(dx,dy)||1e-9, jx=dx/d, jy=dy/d, e=d-ri; Hm[0][0]+=jx*jx; Hm[0][1]+=jx*jy; Hm[1][0]+=jx*jy; Hm[1][1]+=jy*jy; g[0]+=jx*e; g[1]+=jy*e; });
      const s=M.solve(Hm,g); if(!s) break; const c0=cost(t,r); let a=1, tn=t;
      for(;a>1e-4;a/=2){ tn=[t[0]-a*s[0],t[1]-a*s[1]]; if(cost(tn,r)<c0) break; }
      if(a<=1e-4) break; t=tn; path.push(t); if(Math.hypot(a*s[0],a*s[1])<1e-7) break; }
    return path; };
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Map of the range-only cost over the plane. Beacons with their measured range rings, the true position, a draggable start point, the path of a local solver, and the best point on the map."});
  const st=h("div",{class:"stats"});
  const draw=()=>{ const r=meas(); svg.innerHTML=""; let best=null, bc=1e9;
    for(let i=0;i<NX;i++) for(let j=0;j<NY;j++){ const t=[(i+0.5)*10/NX,(j+0.5)*6/NY], c=cost(t,r); if(c<bc){ bc=c; best=t; }
      svg.append(sv("rect",{x:i*W/NX,y:H-(j+1)*H/NY,width:W/NX+0.5,height:H/NY+0.5,fill:"var(--c5)",opacity:(0.8*Math.exp(-c/2.5)).toFixed(3)})); }
    const gp=solve(best,r), G=gp[gp.length-1], path=solve(start,r), E=path[path.length-1];
    A.slice(0,nA).forEach((a,i)=>{ svg.append(sv("circle",{cx:X(a[0]),cy:Y(a[1]),r:r[i]*sc,fill:"none",stroke:"var(--muted)","stroke-width":1,"stroke-dasharray":"4 4"}));
      const d=Math.hypot(E[0]-a[0],E[1]-a[1])||1e-9, q=[a[0]+r[i]*(E[0]-a[0])/d, a[1]+r[i]*(E[1]-a[1])/d];           // a_i + r_i b_i with the best unit vector b_i
      svg.append(sv("line",{x1:X(a[0]),y1:Y(a[1]),x2:X(q[0]),y2:Y(q[1]),stroke:"var(--c4)","stroke-width":1.5}),
        sv("line",{x1:X(q[0]),y1:Y(q[1]),x2:X(E[0]),y2:Y(E[1]),stroke:"var(--c3)","stroke-width":4,"stroke-linecap":"round"}),
        sv("rect",{x:X(a[0])-6,y:Y(a[1])-6,width:12,height:12,fill:"var(--ink)"}),txt(X(a[0]),Y(a[1])+(E[1]<a[1]?-12:22),"beacon "+(i+1),{"font-size":11,class:"mut","text-anchor":"middle"})); });
    svg.append(sv("path",{d:path.map((p,i)=>(i?"L":"M")+X(p[0]).toFixed(1)+" "+Y(p[1]).toFixed(1)).join(" "),fill:"none",stroke:"var(--c2)","stroke-width":2.5}),
      sv("circle",{cx:X(P[0]),cy:Y(P[1]),r:7,fill:"none",stroke:"var(--ink)","stroke-width":2}),
      sv("circle",{cx:X(G[0]),cy:Y(G[1]),r:4,fill:"var(--c1)"}),
      sv("circle",{cx:X(E[0]),cy:Y(E[1]),r:6,fill:"var(--c2)",stroke:"var(--surface)","stroke-width":2}));
    const hd=sv("circle",{cx:X(start[0]),cy:Y(start[1]),r:11,fill:"var(--surface)",stroke:"var(--c2)","stroke-width":3});
    svg.append(hd); drag(svg,hd,(x,y)=>{ start=[Math.max(0.1,Math.min(9.9,x/sc)),Math.max(0.1,Math.min(5.9,(H-y)/sc))]; draw(); });
    const cE=cost(E,r), cG=cost(G,r);
    st.innerHTML=""; st.append(stat("Cost where the solver stopped",f3(cE),"at ("+f2(E[0])+", "+f2(E[1])+") after "+(path.length-1)+" steps"),
      stat("Lowest cost on the map",f3(cG),"at ("+f2(G[0])+", "+f2(G[1])+")"),
      stat("Verdict",cE-cG<1e-4?"global minimum":"stuck in a local minimum","gap "+f3(Math.max(0,cE-cG)))); };
  el.append(h("div",{class:"vt"},"Where am I, from distances only?"),h("p",{class:"vd"},"One unknown position, beacons at known places, one range to each. More strongly coloured cells have lower cost. Drag the hollow orange start point; the solver is Gauss-Newton."),
    h("div",{class:"ctl"},select("Beacons",[["3","3 beacons"],["2","2 beacons"]],v=>{ nA=+v; draw(); }).el,range("Range noise",0,0.5,0.05,sig,v=>{ sig=v; draw(); },v=>f2(v)).el,button("New noise",()=>{ seed++; draw(); })),
    svg,legend([["var(--c2)","solver path and end point"],["var(--c1)","lowest-cost point on the map"],["var(--ink)","true position (ring)"],["var(--c4)","range r along the best unit vector b"],["var(--c3)","what is left: the residual"]]),st);
  draw();
};

// ---- Day 2: order-2 relaxation of min p(x) = x^4 - a x^2 + b x, solved through its sum-of-squares dual ----
// p(x) - g = [1 x x^2] Q [1 x x^2]^T with Q = [[-g, b/2, -l],[b/2, 2l - a, 0],[-l, 0, 1]].  Q >= 0 for some l  =>  p(x) >= g for all x.
// A Schur complement on the lower-right block gives the largest g for a given l:  g(l) = -l^2 - b^2 / (4 (2l - a)),  l > a/2.
// g(l) is concave, so a golden-section search finds the best bound.
VIZ.w10_moment=el=>{
  const W=600, H=320, P={l:44,r:14,t:14,b:34}, XM=2.6;
  let a=3, b=1, x0=1.5;
  const p=x=>x**4-a*x*x+b*x, dp=x=>4*x**3-2*a*x+b;
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Graph of the polynomial p(x), a horizontal line at the lower bound from the semidefinite relaxation, the start point, and the point where gradient descent stops."});
  const st=h("div",{class:"stats"}), out=h("div",{class:"out"});
  const draw=()=>{
    let x=x0; for(let i=0;i<4000;i++){ const s=0.01*dp(x); x-=s; if(Math.abs(s)<1e-12) break; }           // gradient descent, fixed step
    const g=l=>-l*l-b*b/(4*(2*l-a)); let lo=a/2+1e-9, hi=a/2+6; const gr=(Math.sqrt(5)-1)/2;
    for(let i=0;i<90;i++){ const m1=hi-gr*(hi-lo), m2=lo+gr*(hi-lo); if(g(m1)<g(m2)) lo=m1; else hi=m2; }
    const l=(lo+hi)/2, gam=g(l), Q=[[-gam,b/2,-l],[b/2,2*l-a,0],[-l,0,1]], eg=M.eigSym(Q), v=eg.vec.map(r=>r[0]), xs=Math.abs(v[0])>1e-9?v[1]/v[0]:0;
    const yMin=Math.min(gam,-1)-1.2, yMax=6, X=u=>P.l+(u+XM)/(2*XM)*(W-P.l-P.r), Y=u=>H-P.b-(Math.min(yMax+2,u)-yMin)/(yMax-yMin)*(H-P.t-P.b);
    svg.innerHTML=""; svg.append(sv("clipPath",{id:"w10clipm"},sv("rect",{x:P.l,y:P.t,width:W-P.l-P.r,height:H-P.t-P.b})));
    [-2,-1,0,1,2].forEach(u=>svg.append(sv("line",{class:"grid",x1:X(u),x2:X(u),y1:P.t,y2:H-P.b}),txt(X(u),H-P.b+16,u,{"text-anchor":"middle",class:"mut","font-size":11})));
    for(let u=Math.ceil(yMin);u<=yMax;u+=2) svg.append(sv("line",{class:"grid",x1:P.l,x2:W-P.r,y1:Y(u),y2:Y(u)}),txt(P.l-6,Y(u)+4,u,{"text-anchor":"end",class:"mut","font-size":11}));
    svg.append(txt(W-P.r,H-6,"x",{"text-anchor":"end",class:"mut","font-size":11}));
    let d=""; for(let i=0;i<=200;i++){ const u=-XM+2*XM*i/200; d+=(i?"L":"M")+X(u).toFixed(1)+" "+Y(p(u)).toFixed(1)+" "; }
    svg.append(sv("path",{d,fill:"none",stroke:"var(--c5)","stroke-width":2.5,"clip-path":"url(#w10clipm)"}),
      sv("line",{x1:P.l,x2:W-P.r,y1:Y(gam),y2:Y(gam),stroke:"var(--c1)","stroke-width":2,"stroke-dasharray":"7 5"}),
      sv("line",{x1:X(x0),x2:X(x0),y1:P.t,y2:H-P.b,stroke:"var(--c2)","stroke-width":1,"stroke-dasharray":"3 4"}),
      sv("circle",{cx:X(x),cy:Y(p(x)),r:7,fill:"var(--c2)",stroke:"var(--surface)","stroke-width":2}));
    const gap=p(x)-gam;
    st.innerHTML=""; st.append(stat("Descent stops at","x = "+f3(x),"p = "+f3(p(x))),stat("Lower bound d*",f3(gam),"holds for every x"),
      stat("Gap p − d*",f3(Math.max(0,gap)),gap<1e-4?"zero: certified global minimum":"positive: no certificate"));
    out.textContent="Q is positive semidefinite: eigenvalues "+eg.val.map(f3).join(", ")+".  "+(eg.val[1]<1e-6?"Two of them are zero: the minimum is reached at "+(a>0?"two points, x = ±"+f3(Math.sqrt(a/2)):"x = 0")+".":"Its null vector is proportional to [1, x, x²] with x = "+f3(xs)+", the global minimizer."); };
  el.append(h("div",{class:"vt"},"A relaxation you can check by hand"),h("p",{class:"vd"},"p(x) = x⁴ − a x² + b x. Gradient descent runs from the start you pick. The dashed line is the lower bound from the order-2 semidefinite relaxation."),
    h("div",{class:"ctl"},range("a (depth of the two wells)",0,6,0.25,a,v=>{ a=v; draw(); },f2).el,range("b (tilt)",-3,3,0.25,b,v=>{ b=v; draw(); },f2).el,range("Start x",-2.5,2.5,0.1,x0,v=>{ x0=v; draw(); },v=>v.toFixed(1)).el),
    svg,legend([["var(--c5)","p(x)"],["var(--c1)","relaxation bound d*"],["var(--c2)","start (thin line) and where descent stops (dot)"]]),st,out);
  draw();
};

// ---- Day 3: CRLB against the measured variance of three estimators of a scalar from N Gaussian measurements ----
VIZ.w10_crlb=el=>{
  const W=600, H=250, P={l:14,r:14,t:14,b:30}, XT=2, K=2000, NB=41;
  let N=10, sig=1, seed=5, which=0;
  const names=["Sample mean","Sample median","0.8 × sample mean"];
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Histogram of 2000 estimates from the chosen estimator, with the Gaussian whose variance equals the Cramér-Rao bound drawn on top and a vertical line at the true value."});
  const tb=h("div"), st=h("div",{class:"stats"});
  const draw=()=>{ const r=rng(seed), est=[[],[],[]];
    for(let k=0;k<K;k++){ const z=[]; let s=0; for(let i=0;i<N;i++){ const v=XT+sig*r.n(); z.push(v); s+=v; } z.sort((p,q)=>p-q);
      est[0].push(s/N); est[1].push(N%2?z[(N-1)/2]:(z[N/2-1]+z[N/2])/2); est[2].push(0.8*s/N); }
    const crlb=sig*sig/N, sd=Math.sqrt(crlb), res=est.map(e=>{ const m=e.reduce((p,q)=>p+q,0)/K, v=e.reduce((p,q)=>p+(q-m)**2,0)/(K-1), mse=e.reduce((p,q)=>p+(q-XT)**2,0)/K; return {bias:m-XT,v,mse}; });
    const lo=XT-4.2*sd, hi=XT+4.2*sd, X=u=>P.l+(u-lo)/(hi-lo)*(W-P.l-P.r), bw=(hi-lo)/NB, cnt=new Array(NB).fill(0);
    est[which].forEach(e=>{ const b=Math.floor((e-lo)/bw); if(b>=0&&b<NB) cnt[b]++; });
    const top=Math.max(1/(sd*Math.sqrt(2*Math.PI)),...cnt.map(c=>c/(K*bw)))*1.08, Y=u=>H-P.b-u/top*(H-P.t-P.b);
    svg.innerHTML=""; cnt.forEach((c,i)=>{ const y=Y(c/(K*bw)); svg.append(sv("rect",{x:X(lo+i*bw)+0.5,y,width:(W-P.l-P.r)/NB-1,height:H-P.b-y,fill:COL[[4,1,2][which]],opacity:0.55})); });
    let d=""; for(let i=0;i<=120;i++){ const u=lo+(hi-lo)*i/120; d+=(i?"L":"M")+X(u).toFixed(1)+" "+Y(Math.exp(-((u-XT)**2)/(2*crlb))/(sd*Math.sqrt(2*Math.PI))).toFixed(1)+" "; }
    svg.append(sv("path",{d,fill:"none",stroke:"var(--ink)","stroke-width":2}),sv("line",{x1:X(XT),x2:X(XT),y1:P.t,y2:H-P.b,stroke:"var(--muted)","stroke-dasharray":"4 4"}),
      sv("line",{class:"axis",x1:P.l,x2:W-P.r,y1:H-P.b,y2:H-P.b}));
    [-3,-2,-1,0,1,2,3].forEach(k=>svg.append(txt(X(XT+k*sd),H-P.b+16,k===0?"true x = 2":(k>0?"+":"−")+Math.abs(k)+"σ",{"text-anchor":"middle",class:"mut","font-size":11})));
    tb.innerHTML=""; tb.append(h("table",{},h("tr",{},...["Estimator","Bias","Variance","Variance ÷ bound","MSE"].map(t=>h("th",{},t))),
      ...res.map((q,i)=>h("tr",{},h("td",{},names[i]),h("td",{},f3(q.bias)),h("td",{},q.v.toFixed(4)),h("td",{},f2(q.v/crlb)),h("td",{},q.mse.toFixed(4))))));
    st.innerHTML=""; st.append(stat("Fisher information",f2(N/(sig*sig)),"N / σ²"),stat("Cramér-Rao bound",crlb.toFixed(4),"σ² / N, for unbiased estimators"),stat("Trials",K,"each with "+N+" fresh measurements")); };
  el.append(h("div",{class:"vt"},"Can any estimator beat the bound?"),h("p",{class:"vd"},"True value x = 2. Each trial draws N measurements z = x + noise and computes three estimates. The black curve is a Gaussian with the Cramér-Rao variance σ²/N."),
    h("div",{class:"ctl"},range("Measurements N",1,50,1,N,v=>{ N=v; draw(); }).el,range("Noise σ",0.2,3,0.1,sig,v=>{ sig=v; draw(); },v=>v.toFixed(1)).el),
    h("div",{class:"ctl"},select("Histogram shows",names.map((n,i)=>[i,n]),v=>{ which=+v; draw(); }).el,button("New trials",()=>{ seed++; draw(); })),svg,tb,st);
  draw();
};

// ---- Day 4 (main): Fisher information of a position-only pose graph. FIM = L_w (x) I_2, node 0 anchored ----
VIZ.w10_graph=el=>{
  const W=600, H=330, n=8, SC=17;
  const pos=[[90,100],[230,100],[370,100],[510,100],[510,240],[370,240],[230,240],[90,240]];
  const odo=[[0,1],[1,2],[2,3],[3,4],[4,5],[5,6],[6,7]], cand=[[7,0],[6,1],[5,2],[2,0],[6,4]], bend=[0,0,0,-46,46];
  let on=cand.map(()=>false), w=1;
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Pose graph with eight poses in a loop. Pose 0 is anchored. Solid lines are odometry, dashed lines are loop closures that can be switched on. A circle around each pose shows its one-sigma position uncertainty from the inverse Fisher information."});
  const st=h("div",{class:"stats"}), mat=h("div",{class:"panel"}), out=h("div",{class:"out"}), btns=h("div",{class:"ctl"});
  const lap=()=>{ const L=M.zeros(n,n); const add=([i,j],wt)=>{ L[i][i]+=wt; L[j][j]+=wt; L[i][j]-=wt; L[j][i]-=wt; };
    odo.forEach(e=>add(e,1)); cand.forEach((e,k)=>{ if(on[k]) add(e,w); }); return L; };
  const fm=v=>Math.abs(v-Math.round(v))<1e-9?String(Math.round(v)):v.toFixed(2);
  const edge=([i,j],bd,a)=>{ const p=pos[i], q=pos[j], mx=(p[0]+q[0])/2, my=(p[1]+q[1])/2; return sv("path",Object.assign({d:"M"+p[0]+" "+p[1]+" Q"+mx+" "+(my+2*bd)+" "+q[0]+" "+q[1],fill:"none"},a)); };
  const draw=()=>{ const L=lap(), Lr=L.slice(1).map(r=>r.slice(1)), C=M.inv(Lr), eg=M.eigSym(Lr), full=M.eigSym(L);
    const logdet=eg.val.reduce((s,v)=>s+Math.log(v),0), vars=C.map((r,i)=>r[i]), worst=vars.indexOf(Math.max(...vars));
    svg.innerHTML="";
    cand.forEach((e,k)=>svg.append(edge(e,bend[k],on[k]?{stroke:"var(--c1)","stroke-width":3}:{stroke:"var(--line2)","stroke-width":1.5,"stroke-dasharray":"5 5"})));
    odo.forEach(e=>svg.append(edge(e,0,{stroke:"var(--ink)","stroke-width":2})));
    vars.forEach((v,i)=>svg.append(covEllipse([[v,0],[0,v]],pos[i+1][0],pos[i+1][1],1,SC,{fill:"var(--c5-soft)",stroke:"var(--c5)","stroke-width":1.5,opacity:0.75})));
    pos.forEach((p,i)=>{ svg.append(i?sv("circle",{cx:p[0],cy:p[1],r:5,fill:"var(--ink)"}):sv("rect",{x:p[0]-7,y:p[1]-7,width:14,height:14,fill:"var(--c3)"}));
      svg.append(txt(p[0],p[1]+(i<4?-56:66),i?i+": σ² = "+f2(vars[i-1]):"0: anchored",{"text-anchor":"middle","font-size":12})); });
    btns.innerHTML=""; cand.forEach((e,k)=>{ const u=M.zeros(n-1,1).map((_,i)=>(i+1===e[0]?1:0)-(i+1===e[1]?1:0)), Cu=M.mv(C,u), R=u.reduce((s,ui,i)=>s+ui*Cu[i],0);   // effective resistance between the two ends
      btns.append(button(e[0]+"–"+e[1]+(on[k]?" ✓":"  +"+f2(Math.log(1+w*R))),()=>{ on[k]=!on[k]; draw(); },on[k]?"on":"")); });
    btns.append(button("All off",()=>{ on=on.map(()=>false); draw(); },"ghost"));
    st.innerHTML=""; st.append(stat("Weighted spanning trees",fm(Math.exp(logdet)),"det of the reduced Laplacian"),stat("log det (D-criterion)",f3(logdet),"tree connectivity"),
      stat("Smallest eigenvalue (E)",f3(eg.val[0]),"of the reduced Laplacian"),stat("Algebraic connectivity",f3(full.val[1]),"λ₂ of the full Laplacian"),
      stat("Mean variance (A)",f3(vars.reduce((s,v)=>s+v,0)/(n-1)),"trace of the inverse ÷ 7"),stat("Worst variance",f2(vars[worst]),"per axis, at pose "+(worst+1)));
    mat.innerHTML=""; mat.append(h("div",{},"Reduced weighted Laplacian (row and column of pose 0 removed):"),
      h("table",{},h("tr",{},h("th",{},""),...Lr.map((_,j)=>h("th",{},j+1))),...Lr.map((r,i)=>h("tr",{},h("th",{},i+1),...r.map(v=>h("td",{},v===0?"·":fm(v)))))));
    out.textContent="Eigenvalues of the reduced Laplacian: "+eg.val.map(f3).join(", "); };
  el.append(h("div",{class:"vt"},"Loop closures, the Laplacian, and uncertainty"),h("p",{class:"vd"},"Eight 2-D positions with known orientations. Odometry edges have weight 1. Each button switches a loop closure on or off; while it is off, the number on it is how much log det would rise if you added it."),
    h("div",{class:"ctl"},range("Loop-closure weight w (precision)",0.25,4,0.25,w,v=>{ w=v; draw(); },f2).el),btns,svg,
    legend([["var(--ink)","odometry"],["var(--c1)","loop closure, on"],["var(--line2)","loop closure, off"],["var(--c5)","1σ circle of each pose"]]),st,out,mat);
  draw();
};

// ---- Day 5: Kirchhoff's matrix-tree theorem on four nodes: count spanning trees by listing them, and by a determinant ----
VIZ.w10_trees=el=>{
  const W=600, H=300, E=[[0,1],[1,2],[2,3],[3,0],[0,2],[1,3]], pos=[[40,60],[180,60],[180,200],[40,200]];
  let on=[true,true,true,true,true,false];
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Left: a graph on four nodes with the chosen edges. Right: every spanning tree of that graph drawn as a small picture."});
  const st=h("div",{class:"stats"}), out=h("div",{class:"out"}), btns=h("div",{class:"ctl"});
  const isTree=es=>{ const p=[0,1,2,3], f=i=>p[i]===i?i:(p[i]=f(p[i])); for(const [i,j] of es){ const a=f(i), b=f(j); if(a===b) return false; p[a]=b; } return true; };   // 3 edges, no cycle => spanning tree
  const det3=A=>A[0][0]*(A[1][1]*A[2][2]-A[1][2]*A[2][1])-A[0][1]*(A[1][0]*A[2][2]-A[1][2]*A[2][0])+A[0][2]*(A[1][0]*A[2][1]-A[1][1]*A[2][0]);
  const draw=()=>{ const act=E.filter((_,k)=>on[k]), trees=[];
    for(let a=0;a<act.length;a++) for(let b=a+1;b<act.length;b++) for(let c=b+1;c<act.length;c++){ const s=[act[a],act[b],act[c]]; if(isTree(s)) trees.push(s); }
    const L=M.zeros(4,4); act.forEach(([i,j])=>{ L[i][i]++; L[j][j]++; L[i][j]--; L[j][i]--; }); const Lr=L.slice(1).map(r=>r.slice(1));
    svg.innerHTML=""; E.forEach(([i,j],k)=>svg.append(sv("line",{x1:pos[i][0],y1:pos[i][1],x2:pos[j][0],y2:pos[j][1],stroke:on[k]?"var(--ink)":"var(--line)","stroke-width":on[k]?2.5:1,"stroke-dasharray":on[k]?"none":"4 5"})));
    pos.forEach((p,i)=>svg.append(sv("circle",{cx:p[0],cy:p[1],r:13,fill:i?"var(--surface)":"var(--c3-soft)",stroke:i?"var(--ink)":"var(--c3)","stroke-width":2}),txt(p[0],p[1]+4.5,i,{"text-anchor":"middle","font-size":13})));
    svg.append(txt(110,250,"node 0 is the anchor",{"text-anchor":"middle",class:"mut","font-size":12}));
    trees.forEach((t,q)=>{ const ox=262+(q%4)*84, oy=16+Math.floor(q/4)*70, mp=i=>[ox+(pos[i][0]-40)*0.3,oy+(pos[i][1]-60)*0.3];
      t.forEach(([i,j])=>svg.append(sv("line",{x1:mp(i)[0],y1:mp(i)[1],x2:mp(j)[0],y2:mp(j)[1],stroke:"var(--c1)","stroke-width":2.5,"stroke-linecap":"round"})));
      [0,1,2,3].forEach(i=>svg.append(sv("circle",{cx:mp(i)[0],cy:mp(i)[1],r:3.5,fill:"var(--ink)"}))); });
    if(!trees.length) svg.append(txt(420,150,"no spanning tree: the graph is disconnected",{"text-anchor":"middle",class:"mut","font-size":12}));
    btns.innerHTML=""; E.forEach(([i,j],k)=>btns.append(button(i+"–"+j,()=>{ on[k]=!on[k]; draw(); },on[k]?"on":"")));
    st.innerHTML=""; st.append(stat("Edges",act.length,"of 6 possible"),stat("Spanning trees, listed",trees.length,"drawn on the right"),stat("det of reduced Laplacian",Math.round(det3(Lr)),"the same number"));
    out.textContent="Reduced Laplacian (nodes 1, 2, 3):  "+Lr.map(r=>"["+r.join(" ")+"]").join("  "); };
  el.append(h("div",{class:"vt"},"Counting spanning trees two ways"),h("p",{class:"vd"},"Switch edges of a four-node graph on and off. The widget lists every set of three edges that connects all four nodes, and separately computes a 3 × 3 determinant."),btns,svg,st,out);
  draw();
};
