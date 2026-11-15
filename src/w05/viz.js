// Week 5 widgets: B-spline basis viewer, spline fit, kernel-matrix sparsity, GP regression, SE(2) interpolation.

// ---- shared math ----
// uniform B-spline of degree d on the knots 0,1,…,d+1 (Cox-de Boor recursion); zero outside [0, d+1)
const bs=(d,x)=>d===0?(x>=0&&x<1?1:0):(x*bs(d-1,x)+(d+1-x)*bs(d-1,x-1))/d;
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
const rms=a=>Math.sqrt(a.reduce((s,v)=>s+v*v,0)/a.length);
const f2=v=>(Math.abs(v)<0.005?0:v).toFixed(2).replace("-","−");
// the 1-D trajectory both fitting widgets try to recover, and noisy measurements of it at irregular times with a gap
const truth=t=>1.2*Math.sin(0.9*t)+0.5*Math.sin(2.3*t+1)+0.15*t;
const mkData=(seed,sig)=>{ const r=rng(seed), D=[]; let t=0.2; while(t<8.8){ if(t<5.2||t>7) D.push({t,z:truth(t)+sig*r.n()}); t+=0.1+0.45*r.u(); } return D; };
// cubic B-spline least squares: minimise (1/sig²) Σ (z_i − Ψ(t_i) c)² + lam ‖c‖², knots every hk seconds
const splineFit=(D,hk,lam,sig)=>{ const k0=Math.floor(D[0].t/hk)-1, K=Math.floor(D[D.length-1].t/hk)+3-k0;
  const row=t=>Array.from({length:K},(_,j)=>bs(3,t/hk-(k0+j)+2));
  const Psi=D.map(d=>row(d.t)), Pt=M.T(Psi), w=1/(sig*sig);
  const c=M.solve(M.add(M.scale(M.mul(Pt,Psi),w),M.scale(M.eye(K),lam)),M.mv(Pt,D.map(d=>d.z*w)));
  return {K, ok:!!c&&c.every(v=>Math.abs(v)<1e4), p:t=>dot(row(t),c)}; };
// GP regression on position: posterior mean and variance at any time from the kernel k(t,t')
const gpFit=(D,kern,sg)=>{ const Ki=M.inv(D.map((a,i)=>D.map((b,j)=>kern(a.t,b.t)+(i===j?sg*sg:0)))); if(!Ki) return null;
  const al=M.mv(Ki,D.map(d=>d.z));
  return t=>{ const ks=D.map(d=>kern(t,d.t)); return {m:dot(ks,al), s:Math.sqrt(Math.max(0,kern(t,t)-dot(ks,M.mv(Ki,ks))))}; }; };
// kernel matrix K = Φ Q Φᵀ of an SDE prior at the times t (unit power spectral density, unit initial covariance)
const kernMat=(kind,t)=>{ const n=t.length;
  if(kind==="se") return t.map((a,i)=>t.map((b,j)=>Math.exp(-((a-b)**2)/2)+(i===j?1e-6:0)));
  const d=kind==="rw"?1:2, N=n*d, Phi=M.zeros(N,N), Q=M.zeros(N,N);
  for(let i=0;i<n;i++){ for(let j=0;j<=i;j++){ Phi[i*d][j*d]=1; if(d===2){ Phi[i*d][j*d+1]=t[i]-t[j]; Phi[i*d+1][j*d+1]=1; } }
    const dt=i?t[i]-t[i-1]:0;
    if(d===1) Q[i][i]=i?dt:1;
    else if(!i) Q[0][0]=Q[1][1]=1;
    else { Q[2*i][2*i]=dt**3/3; Q[2*i][2*i+1]=Q[2*i+1][2*i]=dt*dt/2; Q[2*i+1][2*i+1]=dt; } }
  const K=M.mul(M.mul(Phi,Q),M.T(Phi));
  return kind==="cvp"?K.filter((_,i)=>i%2===0).map(r=>r.filter((_,j)=>j%2===0)):K; };
// SE(2): pose matrix, and Exp/Log with ξ = (ρ1, ρ2, θ) ordered translation then rotation
const se2=(x,y,th)=>[[Math.cos(th),-Math.sin(th),x],[Math.sin(th),Math.cos(th),y],[0,0,1]];
const Vse2=th=>{ const s=Math.abs(th)<1e-6, a=s?1-th*th/6:Math.sin(th)/th, b=s?th/2:(1-Math.cos(th))/th; return [[a,-b],[b,a]]; };
const logSE2=T=>{ const th=Math.atan2(T[1][0],T[0][0]), r=M.solve(Vse2(th),[T[0][2],T[1][2]])||[0,0]; return [r[0],r[1],th]; };
const expSE2=xi=>{ const p=M.mv(Vse2(xi[2]),[xi[0],xi[1]]); return se2(p[0],p[1],xi[2]); };

// ---- shared plot frame for the two fitting widgets: t in [0,10] s, position in [−2.5, 3.5] ----
const fx=t=>46+t*53.8, fy=y=>290-(Math.max(-30,Math.min(30,y))+2.5)*45;
const pathD=P=>P.map((p,i)=>(i?"L":"M")+fx(p[0]).toFixed(1)+" "+fy(p[1]).toFixed(1)).join(" ");
const frame=aria=>{ const s=sv("svg",{viewBox:"0 0 600 330",role:"img","aria-label":aria});
  for(const v of [-2,0,2]) s.append(sv("line",{class:"grid",x1:46,x2:584,y1:fy(v),y2:fy(v)}),txt(40,fy(v)+4,String(v).replace("-","−"),{"text-anchor":"end",class:"mut","font-size":11}));
  for(let v=0;v<=10;v+=2) s.append(txt(fx(v),308,v,{"text-anchor":"middle",class:"mut","font-size":11}));
  s.append(sv("line",{class:"axis",x1:46,x2:584,y1:290,y2:290}),txt(315,326,"time t (s)",{"text-anchor":"middle",class:"mut","font-size":11}));
  const g=sv("svg",{x:40,y:4,width:548,height:294,viewBox:"40 4 548 294",style:"width:548px;height:294px"}); s.append(g);     // nested viewport: clips curves that leave the plot
  return [s,g]; };
const grid=(a,b,n)=>Array.from({length:n+1},(_,i)=>a+(b-a)*i/n);
const legend=items=>h("div",{class:"legend"},...items.map(([c,t])=>h("span",{},h("i",{style:"background:"+c}),t)));
const dataDots=(g,D)=>D.forEach(d=>g.append(sv("circle",{cx:fx(d.t),cy:fy(d.z),r:3.5,fill:"var(--ink)"})));

// ---- Day 1: B-spline basis functions, local support, and a curve built from draggable coefficients ----
VIZ.w05_basis=el=>{
  let d=3, t=4.3, sel=4; const c=[0.6,1.5,2.3,1.2,0.5,1.3,2.5,2.1,1.1,0.5], LO=1.5, HI=7.5;
  const X=u=>40+u*57.5, Y1=v=>132-v*96, Y2=v=>340-v*55;             // top panel: basis values 0..1; bottom panel: coefficient values 0..3
  const phi=(k,u)=>bs(d,u-k+(d+1)/2);                                // basis function k, centred on t = k
  const p=u=>c.reduce((s,ck,k)=>s+ck*phi(k,u),0);
  const svg=sv("svg",{viewBox:"0 0 600 372",role:"img","aria-label":"Top: ten B-spline basis functions over time with the ones that are nonzero at the chosen time highlighted. Bottom: ten draggable coefficients and the curve they produce."});
  const g=sv("g"), st=h("div",{class:"stats"}); svg.append(g);
  const vis=c.map((_,k)=>sv("circle",{cx:X(k),r:7,stroke:"var(--surface)","stroke-width":2}));
  const hit=c.map((_,k)=>{ const e=sv("circle",{cx:X(k),r:24,fill:"var(--ink)","fill-opacity":0}); drag(svg,e,(x,y)=>{ c[k]=Math.max(0,Math.min(3,(340-y)/55)); sel=k; draw(); }); return e; });
  svg.append(...vis,...hit);
  const draw=()=>{ g.innerHTML=""; const act=c.map((_,k)=>phi(k,t)>1e-12), hw=(d+1)/2, a=Math.max(LO,sel-hw), b=Math.min(HI,sel+hw);
    if(b>a) g.append(sv("rect",{x:X(a),y:170,width:X(b)-X(a),height:170,fill:"var(--acc-soft)"}));
    g.append(sv("line",{class:"axis",x1:X(0),x2:X(9),y1:Y1(0),y2:Y1(0)}),sv("line",{class:"axis",x1:X(0),x2:X(9),y1:Y2(0),y2:Y2(0)}));
    for(let k=0;k<10;k++){ g.append(txt(X(k),358,k,{"text-anchor":"middle",class:"mut","font-size":11}));
      const P=grid(Math.max(0,k-hw),Math.min(9,k+hw),80).map(u=>X(u).toFixed(1)+" "+Y1(phi(k,u)).toFixed(1));
      g.append(sv("path",{d:"M"+P.join(" L"),fill:"none",stroke:act[k]?COL[k%5]:"var(--line2)","stroke-width":act[k]?2.5:1.2})); }
    g.append(sv("line",{x1:X(t),x2:X(t),y1:26,y2:340,stroke:"var(--muted)","stroke-width":1,"stroke-dasharray":"4 4"}));
    c.forEach((ck,k)=>{ if(act[k]) g.append(sv("circle",{cx:X(t),cy:Y1(phi(k,t)),r:4.5,fill:COL[k%5],stroke:"var(--surface)","stroke-width":1.5}));
      vis[k].setAttribute("cy",Y2(ck)); hit[k].setAttribute("cy",Y2(ck)); vis[k].setAttribute("fill",act[k]?COL[k%5]:"var(--muted)"); });
    g.append(sv("path",{d:c.map((ck,k)=>(k?"L":"M")+X(k)+" "+Y2(ck).toFixed(1)).join(" "),fill:"none",stroke:"var(--line2)","stroke-width":1,"stroke-dasharray":"3 4"}),
      sv("path",{d:grid(LO,HI,240).map((u,i)=>(i?"L":"M")+X(u).toFixed(1)+" "+Y2(p(u)).toFixed(1)).join(" "),fill:"none",stroke:"var(--ink)","stroke-width":2.5}),
      sv("circle",{cx:X(t),cy:Y2(p(t)),r:5,fill:"var(--acc)",stroke:"var(--surface)","stroke-width":2}),
      txt(40,18,"basis functions Ψₖ(t)",{class:"mut","font-size":11}),txt(40,164,"coefficients cₖ (drag them) and the curve p(t)",{class:"mut","font-size":11}));
    const w=c.map((_,k)=>phi(k,t)).filter(v=>v>1e-12);
    st.innerHTML=""; st.append(stat("Nonzero basis functions",w.length,"at this time; degree "+d+" → at most "+(d+1)),stat("Their values",w.map(v=>v.toFixed(3)).join(", "),"sum = "+w.reduce((s,v)=>s+v,0).toFixed(3)),
      stat("Curve value",p(t).toFixed(3),"p(t) at t = "+t.toFixed(2)+" s, from "+w.length+" coefficient"+(w.length>1?"s":""))); };
  el.append(h("div",{class:"vt"},"B-spline basis functions"),h("p",{class:"vd"},"Ten basis functions on knots one second apart, built with the Cox–de Boor recursion, and the curve p(t) = Σ Ψₖ(t) cₖ they give. The curve is drawn where every basis function it needs exists."),
    h("div",{class:"ctl"},select("Degree ",[[3,"3 (cubic)"],[2,"2 (quadratic)"],[1,"1 (linear)"],[0,"0 (piecewise constant)"]],v=>{ d=+v; draw(); }).el,range("Time t",LO,HI,0.05,t,v=>{ t=v; draw(); },v=>v.toFixed(2)+" s").el),
    svg,st);
  draw();
};

// ---- Day 2: least-squares fit of a cubic B-spline: knot spacing and the regularizer ----
VIZ.w05_splinefit=el=>{
  const SIG=0.25, LAM=[0,0.001,0.01,0.1,1,10,100]; let hk=1, li=4, seed=1;
  const [svg,g]=frame("Noisy position measurements over time, the true trajectory, and a cubic B-spline fitted to the measurements by regularized least squares");
  const st=h("div",{class:"stats"}), wn=h("p",{class:"warn"});
  const draw=()=>{ const D=mkData(seed,SIG), F=splineFit(D,hk,LAM[li],SIG), T=grid(D[0].t,D[D.length-1].t,200); g.innerHTML="";
    for(let k=0;k*hk<=10+1e-9;k++) g.append(sv("line",{x1:fx(k*hk),x2:fx(k*hk),y1:284,y2:296,stroke:"var(--c2)","stroke-width":2}));
    g.append(sv("path",{d:pathD(grid(0,10,200).map(t=>[t,truth(t)])),fill:"none",stroke:"var(--line2)","stroke-width":2,"stroke-dasharray":"6 5"}));
    if(F.ok) g.append(sv("path",{d:pathD(T.map(t=>[t,F.p(t)])),fill:"none",stroke:"var(--c5)","stroke-width":2.5,"stroke-linejoin":"round"}));
    dataDots(g,D); wn.textContent=F.ok?"":"The normal equations are singular: with λ = 0 and knots this close, some coefficients are touched by no measurement. Add a regularizer or widen the knot spacing.";
    st.innerHTML=""; st.append(stat("Coefficients K",F.K,"for M = "+D.length+" measurements"),
      stat("RMS residual at the measurements",F.ok?rms(D.map(d=>d.z-F.p(d.t))).toFixed(3):"—","noise σ = "+SIG),
      stat("RMS error against the true curve",F.ok?rms(T.map(t=>truth(t)-F.p(t))).toFixed(3):"—","between first and last measurement")); };
  el.append(h("div",{class:"vt"},"Fit a spline: how many basis functions?"),h("p",{class:"vd"},"Solves (ΨᵀΨ/σ² + λI) c = Ψᵀz/σ² for the coefficients of a uniform cubic B-spline. λ = 1 is the regularizer of equation (2.43)."),
    h("div",{class:"ctl"},range("Knot spacing",0.4,3,0.1,hk,v=>{ hk=v; draw(); },v=>v.toFixed(1)+" s").el,range("Regularizer λ",0,6,1,li,v=>{ li=v; draw(); },v=>LAM[v]).el,button("New noise",()=>{ seed++; draw(); })),
    svg,legend([["var(--line2)","true trajectory"],["var(--ink)","measurements"],["var(--c5)","spline fit"],["var(--c2)","knots"]]),wn,st);
  draw();
};

// ---- Day 3: the kernel matrix of an SDE prior is dense, its inverse is (block-)tridiagonal ----
VIZ.w05_sparse=el=>{
  let kind="rw", n=6, irr=false;
  const svg=sv("svg",{viewBox:"0 0 600 310",role:"img","aria-label":"Two heat maps side by side: the kernel matrix K on the left and its inverse on the right. Coloured cells are nonzero entries."}), st=h("div",{class:"stats"});
  const heat=(A,x0,title,eps)=>{ const N=A.length, s=250/N, mx=Math.max(...A.flat().map(Math.abs)); svg.append(txt(x0+125,24,title,{"text-anchor":"middle","font-size":13}));
    A.forEach((r,i)=>r.forEach((v,j)=>{ const nz=Math.abs(v)>eps*mx;
      svg.append(sv("rect",{x:x0+j*s,y:40+i*s,width:s,height:s,stroke:"var(--line)","stroke-width":0.5,fill:nz?(v>0?"var(--c5)":"var(--c3)"):"var(--soft)","fill-opacity":nz?0.15+0.45*Math.sqrt(Math.abs(v)/mx):1}));
      if(N<=6) svg.append(txt(x0+(j+0.5)*s,40+(i+0.5)*s+4,f2(v),{"text-anchor":"middle","font-size":11})); })); };
  const draw=()=>{ const r=rng(5), t=[0]; for(let i=1;i<n;i++) t.push(t[i-1]+(irr?0.4+1.2*r.u():1));
    const K=kernMat(kind,t), Ki=M.inv(K)||K.map(x=>x.map(()=>0)), N=K.length, d=kind==="cv"?2:1, mx=Math.max(...Ki.flat().map(Math.abs)); let nz=0, bw=0;
    Ki.forEach((x,i)=>x.forEach((v,j)=>{ if(Math.abs(v)>1e-8*mx){ nz++; bw=Math.max(bw,Math.abs(Math.floor(i/d)-Math.floor(j/d))); } }));
    svg.innerHTML=""; heat(K,30,"kernel matrix K",1e-12); heat(Ki,320,"inverse kernel matrix K⁻¹",1e-8);
    st.innerHTML=""; st.append(stat("Size of K",N+" × "+N,n+" times"+(d===2?", state (p, v)":"")),stat("Nonzero entries of K⁻¹",nz+" of "+N*N,"K has "+N*N+" of "+N*N),
      stat("Pattern of K⁻¹",bw<=1&&n>2?(d===2?"block-tridiagonal":"tridiagonal"):"dense",bw<=1?"each state linked to its neighbours only":"every time linked to every other")); };
  const ib=button("Irregular times",()=>{ irr=!irr; ib.classList.toggle("on",irr); draw(); });
  el.append(h("div",{class:"vt"},"Dense kernel, sparse inverse"),h("p",{class:"vd"},"Builds K = ΦQΦᵀ for the chosen prior (unit power spectral density, unit initial covariance), inverts it numerically, and counts the entries of the inverse that are not zero. Blue is positive, pink is negative."),
    h("div",{class:"ctl"},select("Prior ",[["rw","random walk, state p"],["cv","constant velocity, state (p, v)"],["cvp","constant velocity, positions only"],["se","squared exponential, length scale 1"]],v=>{ kind=v; draw(); }).el,
      range("Number of times M",3,10,1,n,v=>{ n=v; draw(); }).el,ib),svg,st);
  draw();
};

// ---- Day 4: GP regression with a prior from an SDE: posterior mean and 2-sigma band ----
VIZ.w05_gpfit=el=>{
  const SIG=0.25, P0=100; let prior="cv", lq=1, sg=0.25, seed=1;
  // position kernel with the prior anchored at t = 0: constant velocity (white noise on acceleration) or random walk
  const kern=(a,b)=>{ const m=Math.min(a,b), q=10**lq; return prior==="cv"?P0+P0*a*b+q*(m*m*m/3+Math.abs(a-b)*m*m/2):P0+q*m; };
  const [svg,g]=frame("Noisy position measurements over time, the true trajectory, the Gaussian-process posterior mean, and a shaded band of plus and minus two posterior standard deviations");
  const st=h("div",{class:"stats"});
  const draw=()=>{ const D=mkData(seed,SIG), post=gpFit(D,kern,sg), T=grid(0,10,125); g.innerHTML="";
    const Pq=post?T.map(t=>Object.assign({t},post(t))):[];
    if(post) g.append(sv("path",{d:pathD(Pq.map(q=>[q.t,q.m+2*q.s]).concat(Pq.map(q=>[q.t,q.m-2*q.s]).reverse()))+" Z",fill:"var(--c5-soft)",stroke:"none"}));
    g.append(sv("path",{d:pathD(T.map(t=>[t,truth(t)])),fill:"none",stroke:"var(--line2)","stroke-width":2,"stroke-dasharray":"6 5"}));
    if(post) g.append(sv("path",{d:pathD(Pq.map(q=>[q.t,q.m])),fill:"none",stroke:"var(--c5)","stroke-width":2.5,"stroke-linejoin":"round"}));
    dataDots(g,D);
    const In=Pq.filter(q=>q.t>=D[0].t&&q.t<=D[D.length-1].t);
    st.innerHTML=""; st.append(stat("RMS error against the true curve",post?rms(In.map(q=>truth(q.t)-q.m)).toFixed(3):"—","between first and last measurement"),
      stat("Band half-width at 3 s",post?"±"+(2*post(3).s).toFixed(2):"—","2σ, among the measurements"),stat("Band half-width at 6.1 s",post?"±"+(2*post(6.1).s).toFixed(2):"—","2σ, middle of the gap"),
      stat("True curve inside the band",post?Math.round(100*Pq.filter(q=>Math.abs(truth(q.t)-q.m)<=2*q.s).length/Pq.length)+"%":"—","of 126 query times")); };
  el.append(h("div",{class:"vt"},"Gaussian-process regression with a motion prior"),h("p",{class:"vd"},"Computes the posterior mean and variance of p(t) at 126 query times from the same measurements as the spline widget. The prior has a wide initial covariance, so only its smoothness matters."),
    h("div",{class:"ctl"},select("Prior ",[["cv","constant velocity"],["rw","random walk"]],v=>{ prior=v; draw(); }).el,
      range("Power spectral density Q",-3,2,0.25,lq,v=>{ lq=v; draw(); },v=>+(10**v).toPrecision(2)).el,range("Assumed measurement σ",0.05,1,0.05,sg,v=>{ sg=v; draw(); },v=>v.toFixed(2)).el,button("New noise",()=>{ seed++; draw(); })),
    svg,legend([["var(--line2)","true trajectory"],["var(--ink)","measurements"],["var(--c5)","posterior mean"],["var(--c5-soft)","±2σ band"]]),st);
  draw();
};

// ---- Day 5: interpolating between two SE(2) poses through the Lie algebra ----
VIZ.w05_se2=el=>{
  let al=0.5, th1=120, p1=[10,7]; const T0=se2(2.5,3,0), SC=36;
  const X=x=>30+SC*x, Y=y=>322-SC*y;
  const svg=sv("svg",{viewBox:"0 0 600 340",role:"img","aria-label":"Top view of two poses drawn as triangles, the curved path given by interpolating between them on SE(2), the straight path given by interpolating coordinates, and the interpolated pose"});
  const g=sv("g"), st=h("div",{class:"stats"}); svg.append(g);
  const tri=(T,a)=>sv("polygon",Object.assign({points:[[0.75,0],[-0.4,0.38],[-0.4,-0.38]].map(([x,y])=>X(T[0][0]*x+T[0][1]*y+T[0][2]).toFixed(1)+","+Y(T[1][0]*x+T[1][1]*y+T[1][2]).toFixed(1)).join(" ")},a));
  const hd=sv("circle",{r:26,fill:"var(--ink)","fill-opacity":0}); drag(svg,hd,(x,y)=>{ p1=[Math.max(0.5,Math.min(15,(x-30)/SC)),Math.max(0.5,Math.min(8.4,(322-y)/SC))]; draw(); }); svg.append(hd);
  const draw=()=>{ const T1=se2(p1[0],p1[1],th1*Math.PI/180), xi=logSE2(M.mul(T1,M.inv(T0))), at=a=>M.mul(expSE2(xi.map(v=>v*a)),T0), Ta=at(al); g.innerHTML="";
    const lin=[T0[0][2]+al*(p1[0]-T0[0][2]),T0[1][2]+al*(p1[1]-T0[1][2])];
    g.append(sv("line",{x1:X(T0[0][2]),y1:Y(T0[1][2]),x2:X(p1[0]),y2:Y(p1[1]),stroke:"var(--c2)","stroke-width":2,"stroke-dasharray":"5 5"}),
      sv("path",{d:grid(0,1,60).map((a,i)=>{ const T=at(a); return (i?"L":"M")+X(T[0][2]).toFixed(1)+" "+Y(T[1][2]).toFixed(1); }).join(" "),fill:"none",stroke:"var(--c5)","stroke-width":2.5}));
    for(const a of [0.2,0.4,0.6,0.8]) g.append(tri(at(a),{fill:"var(--c5-soft)",stroke:"var(--c5)","stroke-width":1}));
    g.append(tri(T0,{fill:"var(--ink)"}),tri(T1,{fill:"var(--ink)"}),sv("circle",{cx:X(lin[0]),cy:Y(lin[1]),r:5,fill:"var(--surface)",stroke:"var(--c2)","stroke-width":2}),tri(Ta,{fill:"var(--acc)",stroke:"var(--surface)","stroke-width":1.5}),
      txt(X(T0[0][2])-8,Y(T0[1][2])+26,"Tₖ₋₁",{"text-anchor":"middle","font-size":12}),txt(X(p1[0]),Y(p1[1])-22,"Tₖ (drag)",{"text-anchor":"middle","font-size":12}));
    hd.setAttribute("cx",X(p1[0])); hd.setAttribute("cy",Y(p1[1]));
    const dR=(1-al)**2+al*al+2*al*(1-al)*Math.cos(xi[2]);
    st.innerHTML=""; st.append(stat("Log of the relative pose","("+f2(xi[0])+", "+f2(xi[1])+", "+f2(xi[2])+")","ξ = (ρ₁, ρ₂, θ), θ = "+Math.round(xi[2]*180/Math.PI)+"°"),
      stat("Interpolated pose","("+f2(Ta[0][2])+", "+f2(Ta[1][2])+"), "+Math.round(Math.atan2(Ta[1][0],Ta[0][0])*180/Math.PI)+"°","Exp(α ξ) Tₖ₋₁ at α = "+al.toFixed(2)),
      stat("Distance to the straight-line point",Math.hypot(Ta[0][2]-lin[0],Ta[1][2]-lin[1]).toFixed(2),"same α, coordinates interpolated"),
      stat("Determinant of the blended rotation",dR.toFixed(2),"(1−α)Rₖ₋₁ + αRₖ; a rotation needs 1")); };
  el.append(h("div",{class:"vt"},"Interpolating between two poses"),h("p",{class:"vd"},"Computes T(t) = Exp(α Log(Tₖ Tₖ₋₁⁻¹)) Tₖ₋₁ on SE(2), equation (2.61) with a linear spline, and compares it with interpolating x and y separately."),
    h("div",{class:"ctl"},range("α",0,1,0.05,al,v=>{ al=v; draw(); },v=>v.toFixed(2)).el,range("Heading of Tₖ",-180,180,5,th1,v=>{ th1=v; draw(); },v=>v+"°").el),
    svg,legend([["var(--c5)","interpolation on SE(2)"],["var(--c2)","straight line in x, y"],["var(--acc)","T(t)"]]),st);
  draw();
};
