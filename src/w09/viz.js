// Week 9: certifiably optimal SLAM.
//   w09_local, w09_cert, w09_sync : a rotation-only pose graph in the plane (book Problems 6.2–6.4 with d = 2, no translations)
//   w09_shor : a three-variable ±1 QCQP and its Shor relaxation
//   w09_elim : eliminating the translations of a planar pose graph, equations (6.16)–(6.17)
const DEG=Math.PI/180;
const arrow=(x,y,a,len,col,w)=>{ const c=Math.cos(a), s=-Math.sin(a), hl=5+2.5*w, hw=2.5+1.2*w, x1=x+len*c, y1=y+len*s, hx=x1-hl*c, hy=y1-hl*s;      // a = math angle, y up
  return sv("g",{},sv("line",{x1:x.toFixed(1),y1:y.toFixed(1),x2:hx.toFixed(1),y2:hy.toFixed(1),stroke:col,"stroke-width":w,"stroke-linecap":"round"}),
    sv("polygon",{points:[[x1,y1],[hx-hw*s,hy+hw*c],[hx+hw*s,hy-hw*c]].map(p=>p[0].toFixed(1)+","+p[1].toFixed(1)).join(" "),fill:col})); };
const z3=v=>(Math.abs(v)<5e-4?0:v).toFixed(3);

// ---------- core: synchronization of planar rotations ----------
// Y is r × 2n (rows). Block i = columns 2i, 2i+1 = an r × 2 matrix with orthonormal columns (a point of St(2, r)).
const SN=10, EDGES=[...Array.from({length:SN},(_,i)=>[i,(i+1)%SN]),[0,4]];
const mkGraph=(sigDeg,seed)=>{ const r=rng(seed), th=Array.from({length:SN},(_,i)=>2*Math.PI*i/SN+Math.PI/2);
  return {th,mz:EDGES.map(([i,j])=>th[j]-th[i]+r.n()*sigDeg*DEG)}; };
// connection Laplacian (6.13) with every κ = 1: the data matrix Q of the rotation-only problem
const conLap=(n,E,ang,kap)=>{ const Q=M.zeros(2*n,2*n); E.forEach(([i,j],e)=>{ const k=kap?kap[e]:1, c=k*Math.cos(ang[e]), s=k*Math.sin(ang[e]), R=[[c,-s],[s,c]];
  for(let a=0;a<2;a++){ Q[2*i+a][2*i+a]+=k; Q[2*j+a][2*j+a]+=k; for(let b=0;b<2;b++){ Q[2*i+a][2*j+b]-=R[a][b]; Q[2*j+b][2*i+a]-=R[a][b]; } } }); return Q; };
const cost=(Q,Y)=>{ let s=0; for(const y of Y){ const q=M.mv(Q,y); for(let k=0;k<y.length;k++) s+=y[k]*q[k]; } return s; };      // tr(Q YᵀY)
// Λ_i = sym(Y_iᵀ (YQ)_i) as [a, b, d], and the Riemannian gradient Y(Q − Λ) (the factor 2 is left out)
const rgrad=(Q,Y,n)=>{ const P=Y.map(y=>M.mv(Q,y)), L=[]; let g2=0;
  for(let i=0;i<n;i++){ let a=0,b=0,c=0,d=0; for(let k=0;k<Y.length;k++){ const y0=Y[k][2*i], y1=Y[k][2*i+1], p0=P[k][2*i], p1=P[k][2*i+1]; a+=y0*p0; b+=y0*p1; c+=y1*p0; d+=y1*p1; } L.push([a,(b+c)/2,d]); }
  const G=Y.map((y,k)=>{ const g=new Array(2*n); for(let i=0;i<n;i++){ const [a,b,d]=L[i]; g[2*i]=P[k][2*i]-y[2*i]*a-y[2*i+1]*b; g[2*i+1]=P[k][2*i+1]-y[2*i]*b-y[2*i+1]*d; g2+=g[2*i]*g[2*i]+g[2*i+1]*g[2*i+1]; } return g; });
  return {G,L,g2}; };
// retraction: step, then re-orthonormalize the two columns of every block (Gram-Schmidt)
const retract=(Y,G,a,n)=>{ const Z=Y.map((y,k)=>y.map((v,c)=>v-a*G[k][c])), r=Z.length;
  for(let i=0;i<n;i++){ let s=0; for(let k=0;k<r;k++) s+=Z[k][2*i]*Z[k][2*i]; s=Math.sqrt(s)||1; for(let k=0;k<r;k++) Z[k][2*i]/=s;
    let p=0; for(let k=0;k<r;k++) p+=Z[k][2*i]*Z[k][2*i+1]; s=0; for(let k=0;k<r;k++){ Z[k][2*i+1]-=p*Z[k][2*i]; s+=Z[k][2*i+1]*Z[k][2*i+1]; } s=Math.sqrt(s)||1; for(let k=0;k<r;k++) Z[k][2*i+1]/=s; }
  return Z; };
// Riemannian gradient descent with a backtracking (Armijo) line search
// ponytail: plain gradient descent, fine for 10 poses; SE-Sync uses a second-order trust-region method
const descend=(Q,Y,n,maxit)=>{ let f=cost(Q,Y), a=0.1, g;
  for(let it=0;it<maxit;it++){ g=rgrad(Q,Y,n); if(g.g2<1e-11) break; a*=2; let Yn, fn;
    for(;;){ Yn=retract(Y,g.G,a,n); fn=cost(Q,Yn); if(fn<=f-2e-4*a*g.g2||a<1e-10) break; a/=2; }
    if(a<1e-10) break; Y=Yn; f=fn; }
  g=rgrad(Q,Y,n); return {Y,f,L:g.L,g2:g.g2}; };
const certS=(Q,L)=>{ const S=Q.map(r=>[...r]); L.forEach(([a,b,d],i)=>{ S[2*i][2*i]-=a; S[2*i][2*i+1]-=b; S[2*i+1][2*i]-=b; S[2*i+1][2*i+1]-=d; }); return S; };     // S = Q − Λ
const fromTh=th=>{ const Y=[[],[]]; th.forEach(t=>{ Y[0].push(Math.cos(t),-Math.sin(t)); Y[1].push(Math.sin(t),Math.cos(t)); }); return Y; };            // R = (R_1 … R_n)
const randY=(r,n,seed)=>{ const g=rng(seed); if(r===2) return fromTh(Array.from({length:n},()=>2*Math.PI*g.u()));
  const Y=Array.from({length:r},()=>Array.from({length:2*n},()=>g.n())); return retract(Y,Y,0,n); };
// rounding (§6.1.2.4): best rank-2 approximation of Y from the top two eigenvectors of YYᵀ, then the nearest rotation to every 2 × 2 block
const roundY=(Y,n)=>{ const r=Y.length, e=M.eigSym(M.mul(Y,M.T(Y))), B=[0,1].map(c=>Y[0].map((_,col)=>{ let s=0; for(let k=0;k<r;k++) s+=e.vec[k][r-1-c]*Y[k][col]; return s; }));
  let neg=0; for(let i=0;i<n;i++) if(B[0][2*i]*B[1][2*i+1]-B[0][2*i+1]*B[1][2*i]<0) neg++; if(neg>n/2) B[1]=B[1].map(v=>-v);
  return {th:Array.from({length:n},(_,i)=>Math.atan2(B[1][2*i]-B[0][2*i+1],B[0][2*i]+B[1][2*i+1])), ev:e.val.slice().reverse().map(v=>v/n)}; };

const syncWidget=(el,mode)=>{ const n=SN, W=600, H=300, cx=150, cy=150, RR=100, full=mode==="full";
  let sig=10, nseed=78, r=full?3:2, start=8, tally=null;
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Left: ten poses on a ring with their true and estimated headings and the measurement edges. Right: a dot plot, either of the final costs of 20 runs or of the eigenvalues of the certificate matrix."});
  const st=h("div",{class:"stats"}), verdict=h("div",{class:"panel"});
  const run=(g,Q,s)=>descend(Q,randY(r,n,97*r+s),n,1500);
  // dots on a number line between x0 and x1; equal values stack upward
  const strip=(vals,lo,hi,y,title,colOf,ticks,dy)=>{ const x0=345, x1=580, X=v=>x0+(Math.max(lo,Math.min(hi,v))-lo)/(hi-lo)*(x1-x0), seen={};
    svg.append(txt(x0,y-dy,title,{"font-size":12,class:"mut"}),sv("line",{class:"axis",x1:x0,x2:x1,y1:y,y2:y}));
    ticks.forEach(v=>svg.append(sv("line",{class:"axis",x1:X(v),x2:X(v),y1:y,y2:y+5}),txt(X(v),y+19,String(v),{"text-anchor":"middle","font-size":11,class:"mut"})));
    vals.forEach(v=>{ const k=Math.round(X(v)/7); seen[k]=(seen[k]||0)+1; svg.append(sv("circle",{cx:7*k,cy:(y-6.5*seen[k]).toFixed(1),r:3,fill:colOf(v)})); }); };
  const draw=()=>{ const g=mkGraph(sig,nseed), Q=conLap(n,EDGES,g.mz), o=run(g,Q,start), ev=M.eigSym(certS(Q,o.L)).val, lm=ev[0], rd=roundY(o.Y,n), fr=cost(Q,fromTh(rd.th));
    const rk=rd.ev.filter(v=>v>1e-3).length, psd=lm>-1e-3;
    // align the estimate to the truth with the best single global rotation (the gauge freedom)
    let ss=0, cc=0; rd.th.forEach((t,i)=>{ ss+=Math.sin(g.th[i]-t); cc+=Math.cos(g.th[i]-t); }); const off=Math.atan2(ss,cc), est=rd.th.map(t=>t+off);
    const P=g.th.map((_,i)=>[cx+RR*Math.cos(2*Math.PI*i/n),cy-RR*Math.sin(2*Math.PI*i/n)]); let rmax=0;
    svg.innerHTML="";
    EDGES.forEach(([i,j],e)=>{ const res=Math.atan2(Math.sin(est[j]-est[i]-g.mz[e]),Math.cos(est[j]-est[i]-g.mz[e])), bad=Math.abs(res)>15*DEG; rmax=Math.max(rmax,Math.abs(res));
      const dx=P[j][0]-P[i][0], dy=P[j][1]-P[i][1], L=Math.hypot(dx,dy), ux=dx/L, uy=dy/L;
      svg.append(sv("line",Object.assign({x1:P[i][0]+12*ux,y1:P[i][1]+12*uy,x2:P[j][0]-12*ux,y2:P[j][1]-12*uy,stroke:bad?"var(--c3)":"var(--line2)","stroke-width":bad?3.5:2},e>=n?{"stroke-dasharray":"5 4"}:{}))); });
    P.forEach((p,i)=>{ svg.append(arrow(p[0],p[1],g.th[i],36,"var(--line2)",4),arrow(p[0],p[1],est[i],30,"var(--acc)",2.2),
      sv("circle",{cx:p[0],cy:p[1],r:10,fill:"var(--surface)",stroke:"var(--ink)","stroke-width":1.5}),txt(p[0],p[1]+4,String(i+1),{"text-anchor":"middle","font-size":11,"data-top":1})); });
    if(mode==="local"){ if(tally) strip(tally,0,25,230,"Final cost of each of 20 random starts",v=>v<tally.best+1e-3?"var(--c1)":"var(--c3)",[0,5,10,15,20,25],140);
      else svg.append(txt(345,150,"Press “Run 20 starts” to see",{"font-size":12,class:"mut"}),txt(345,167,"where 20 random starts end up.",{"font-size":12,class:"mut"})); }
    else strip(ev,-2,6,230,"The 20 eigenvalues of S = Q − Λ",v=>v<-1e-3?"var(--c3)":"var(--c1)",[-2,0,2,4,6],60);
    st.innerHTML=""; st.append(stat(full?"Cost f(Y) at rank r = "+r:"Cost at this critical point",o.f.toFixed(3),"gradient norm "+Math.sqrt(o.g2).toExponential(0)));
    if(mode==="local") st.append(stat("Largest edge residual",(rmax/DEG).toFixed(1)+"°","pink edges: above 15°"),
      stat("20 starts",tally?tally.filter(v=>v<tally.best+1e-3).length+" reach "+tally.best.toFixed(3):"—",tally?(20-tally.filter(v=>v<tally.best+1e-3).length)+" end higher":"not run yet"));
    else{ if(full) st.append(stat("Eigenvalues of YYᵀ / n",rd.ev.map(v=>(Math.abs(v)<5e-3?0:v).toFixed(2)).join(", "),"rank "+rk+(rk<r?" < r: rank-deficient":" = r: full rank")),stat("Rounded cost f(R̂)",fr.toFixed(3),"rotations in SO(2)"));
      st.append(stat("Min eigenvalue of S",z3(lm),psd?"S is positive semidefinite":"S is not positive semidefinite"));
      verdict.className="panel"+(psd&&rk===2?"":" warn");
      verdict.textContent= psd&&rk===2 ? "Certified. S ⪰ 0, so no feasible Z of the relaxation costs less than "+o.f.toFixed(3)+", and this solution uses rotations only (rank 2). It is a global optimum of the original problem."
        : psd ? "The relaxation is solved (S ⪰ 0) but its solution has rank "+rk+", not 2: the relaxation is not tight here. p*SDP = "+o.f.toFixed(3)+" is only a lower bound. The rounded estimate costs "+fr.toFixed(3)+", so it is at most "+(fr-o.f).toFixed(3)+" above the optimum."
        : full&&r>2 ? "No certificate: S has a negative eigenvalue, so YᵀY does not solve the relaxation. Y has "+(rk<r?"not converged to a second-order critical point":"full rank, so Theorem 6.2 does not apply")+". Climb the staircase: raise r."
        : "No certificate: S has a negative eigenvalue. Either this is a spurious local minimum or the relaxation is not tight at this noise; the test cannot say which. Try another start"+(full?" or raise r.":"."); } };
  const reset=()=>{ tally=null; draw(); };
  const ctl=h("div",{class:"ctl"},range("Rotation noise σ per measurement",0,90,5,sig,v=>{ sig=v; reset(); },v=>v+"°").el);
  if(full) ctl.append(range("Rank r of the factor Y",2,5,1,r,v=>{ r=v; draw(); },v=>v+(v===2?" (original problem)":" (relaxed)")).el);
  const btn=h("div",{class:"ctl"},button("New start",()=>{ start++; draw(); },"primary"),button("New noise",()=>{ nseed++; reset(); }));
  if(mode==="local") btn.append(button("Run 20 starts",()=>{ const g=mkGraph(sig,nseed), Q=conLap(n,EDGES,g.mz); tally=Array.from({length:20},(_,s)=>run(g,Q,s+1).f); tally.best=Math.min(...tally); draw(); }));
  el.append(h("div",{class:"vt"},mode==="local"?"Local search on a rotation-only pose graph":mode==="cert"?"The certificate matrix S = Q − Λ":"The Riemannian staircase on a rotation-only pose graph"),
    h("p",{class:"vd"},"Ten headings, ten ring measurements and one chord (dashed). "+(full?"Riemannian gradient descent on Problem 6.4 from a random start, then rounding and the test λmin(S) ≥ 0.":"Gradient descent on the headings from a random start"+(mode==="cert"?", then the test λmin(S) ≥ 0.":"."))),
    ctl,btn,svg,h("div",{class:"legend"},...[["var(--line2)","true heading"],["var(--acc)","estimate (best global rotation applied)"],["var(--c3)","edge residual above 15°"]].map(([c,t])=>h("span",{},h("i",{style:"background:"+c}),t))),st);
  if(mode!=="local") el.append(verdict);
  draw(); };
VIZ.w09_local=el=>syncWidget(el,"local");
VIZ.w09_cert=el=>syncWidget(el,"cert");
VIZ.w09_sync=el=>syncWidget(el,"full");

// ---------- Shor's relaxation of: min 2(c12 x1x2 + c13 x1x3 + c23 x2x3)  s.t.  x_i² = 1 ----------
VIZ.w09_shor=el=>{ let c=[1,1,1]; const W=600, H=310, CY=165;
  const f=(a,b)=>2*(c[0]*Math.cos(a)+c[1]*Math.cos(b)+c[2]*Math.cos(a-b));       // unit vectors at angles 0, a, b; ±1 means angle 0 or π
  const solve=()=>{ let p=Infinity, pt=[0,0]; for(const a of [0,Math.PI]) for(const b of [0,Math.PI]) if(f(a,b)<p-1e-12){ p=f(a,b); pt=[a,b]; }
    // three constraints ⇒ the SDP has a solution of rank ≤ 2, a Gram matrix of unit vectors in the plane: search the two angles (grid, then pattern search)
    let d=p, dt=pt; const N=120; for(let i=0;i<N;i++) for(let j=0;j<N;j++){ const v=f(2*Math.PI*i/N,2*Math.PI*j/N); if(v<d-1e-9){ d=v; dt=[2*Math.PI*i/N,2*Math.PI*j/N]; } }
    for(let s=2*Math.PI/N;s>1e-8;s/=2){ let imp=true; while(imp){ imp=false; for(const [da,db] of [[s,0],[-s,0],[0,s],[0,-s]]) if(f(dt[0]+da,dt[1]+db)<d-1e-13){ d=f(dt[0]+da,dt[1]+db); dt=[dt[0]+da,dt[1]+db]; imp=true; } } }
    if(p-d<1e-6){ d=p; dt=pt; } return {p,pt,d,dt}; };
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Two unit circles. Left: the best choice of three variables that must each be plus or minus one, drawn as arrows pointing right or left. Right: the best choice when each variable may be any unit vector in the plane."});
  const st=h("div",{class:"stats"}), mat=h("div",{class:"out"});
  const draw=()=>{ const s=solve(); svg.innerHTML="";
    [[150,[0,...s.pt],"QCQP: every x is +1 or −1"],[450,[0,...s.dt],"Relaxation: any unit vector"]].forEach(([x0,ang,title])=>{
      svg.append(txt(x0,22,title,{"text-anchor":"middle","font-size":13}),sv("circle",{cx:x0,cy:CY,r:90,fill:"none",stroke:"var(--line2)","stroke-width":1.5}),sv("line",{class:"grid",x1:x0-90,x2:x0+90,y1:CY,y2:CY}));
      ang.forEach((a,i)=>{ const oy=CY+(i-1)*9, rr=102+16*i; svg.append(arrow(x0,oy,a,90,COL[[0,1,4][i]],3),txt(x0+rr*Math.cos(a),oy-rr*Math.sin(a)+4,"x"+"₁₂₃"[i],{"text-anchor":"middle","font-size":13,fill:COL[[0,1,4][i]]})); }); });
    const X=[[1,Math.cos(s.dt[0]),Math.cos(s.dt[1])],[Math.cos(s.dt[0]),1,Math.cos(s.dt[0]-s.dt[1])],[Math.cos(s.dt[1]),Math.cos(s.dt[0]-s.dt[1]),1]], ev=M.eigSym(X).val, rk=ev.filter(v=>v>1e-6).length, n2=v=>(Math.abs(v)<5e-3?0:v).toFixed(2);
    st.innerHTML=""; st.append(stat("p* (QCQP)",s.p.toFixed(3),"best of the 4 sign patterns"),stat("d* (relaxation)",s.d.toFixed(3),"lower bound"),stat("Gap p* − d*",(s.p-s.d).toFixed(3),s.p-s.d<1e-6?"tight: the relaxation is exact":"not tight: a bound only"),stat("Eigenvalues of X*",ev.map(n2).join(", "),"rank "+rk));
    mat.innerHTML=""; mat.append(tex("\\mathbf{X}^\\star=\\begin{bmatrix}"+X.map(r=>r.map(n2).join(" & ")).join(" \\\\ ")+"\\end{bmatrix}")); };
  const sl=["c₁₂","c₁₃","c₂₃"].map((nm,i)=>range("Coupling "+nm,-2,2,0.25,c[i],v=>{ c[i]=v; draw(); },v=>v.toFixed(2)));
  const preset=v=>()=>sl.forEach((s,i)=>{ s.input.value=v[i]; s.input.dispatchEvent(new Event("input")); });
  el.append(h("div",{class:"vt"},"A tiny QCQP and its Shor relaxation"),h("p",{class:"vd"},"Minimize 2(c₁₂x₁x₂ + c₁₃x₁x₃ + c₂₃x₂x₃) subject to x₁² = x₂² = x₃² = 1. The relaxation replaces the products xᵢxⱼ by the entries of a positive semidefinite X with unit diagonal."),
    h("div",{class:"ctl"},...sl.map(s=>s.el)),h("div",{class:"ctl"},button("All three disagree (1, 1, 1)",preset([1,1,1])),button("Consistent (1, 1, −1)",preset([1,1,-1]))),svg,st,mat);
  draw(); };

// ---------- eliminating translations: t*(R) from (6.16), and tr(Q RᵀR) from (6.17) ----------
VIZ.w09_elim=el=>{ const n=6, POS=[[0,0],[2,0],[4,0],[4,2],[2,2],[0,2]], TH=[0,0,90,180,180,270].map(v=>v*DEG), E=[[0,1],[1,2],[2,3],[3,4],[4,5],[5,0],[1,4]], m=E.length, KAP=50, TAU=100;
  const g=rng(4), mz=[], mt=[];                                              // measurements from the noise model (6.8); angle noise ≈ N(0, 1/(2κ))
  E.forEach(([i,j])=>{ const dx=POS[j][0]-POS[i][0], dy=POS[j][1]-POS[i][1], c=Math.cos(TH[i]), s=Math.sin(TH[i]);
    mz.push(TH[j]-TH[i]+g.n()/Math.sqrt(2*KAP)); mt.push([c*dx+s*dy+g.n()/Math.sqrt(TAU),-s*dx+c*dy+g.n()/Math.sqrt(TAU)]); });
  // graph matrices, with the incidence matrix A as n × m (one column per edge), which is the shape (6.17b) needs
  const A=M.zeros(n,m), D=M.zeros(m,2*n); E.forEach(([i,j],e)=>{ A[i][e]=-1; A[j][e]=1; D[e][2*i]=-mt[e][0]; D[e][2*i+1]=-mt[e][1]; });
  const Lt=M.scale(M.mul(A,M.T(A)),TAU), eg=M.eigSym(Lt), Lp=M.zeros(n,n);                                            // L(W^τ) and its pseudo-inverse
  eg.val.forEach((v,k)=>{ if(v>1e-9) for(let a=0;a<n;a++) for(let b=0;b<n;b++) Lp[a][b]+=eg.vec[a][k]*eg.vec[b][k]/v; });
  const V=M.scale(M.mul(A,D),TAU), Pi=M.sub(M.eye(m),M.scale(M.mul(M.T(A),M.mul(Lp,A)),TAU));                           // Ṽ = AΩD̃ ;  Π = I − Ω½ Aᵀ L† A Ω½
  const Q=M.add(conLap(n,E,mz,E.map(()=>KAP)),M.scale(M.mul(M.T(D),M.mul(Pi,D)),TAU));                               // (6.17b)
  let k=1, tw=0;
  const eva=th=>{ const R=fromTh(th), T=M.scale(M.mul(R,M.mul(M.T(V),Lp)),-1); let fr=0, ft=0;                           // (6.16): columns of T are t_i
    E.forEach(([i,j],e)=>{ fr+=4*KAP*(1-Math.cos(th[j]-th[i]-mz[e])); const c=Math.cos(th[i]), s=Math.sin(th[i]);
      ft+=TAU*((T[0][j]-T[0][i]-c*mt[e][0]+s*mt[e][1])**2+(T[1][j]-T[1][i]-s*mt[e][0]-c*mt[e][1])**2); }); return {T,fr,ft,tr:cost(Q,R)}; };
  const ths=t=>TH.map((v,i)=>v+(i===k?t*DEG:0));
  const W=600, H=250, sc=48, X=x=>300+sc*x, Y=y=>125-sc*y, tri=(x,y,a,col,op)=>sv("polygon",{points:[[14,0],[-8,7],[-8,-7]].map(([u,v])=>(X(x)+u*Math.cos(a)-v*Math.sin(a)).toFixed(1)+","+(Y(y)-u*Math.sin(a)-v*Math.cos(a)).toFixed(1)).join(" "),fill:col,opacity:op});
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Top view of six poses: the true poses, and the poses whose rotations are set by the slider and whose positions are the optimal translations for those rotations"});
  const st=h("div",{class:"stats"}), chart=h("div");
  const draw=()=>{ const th=ths(tw), o=eva(th); svg.innerHTML="";
    E.forEach(([i,j],e)=>svg.append(sv("line",Object.assign({x1:X(o.T[0][i]),y1:Y(o.T[1][i]),x2:X(o.T[0][j]),y2:Y(o.T[1][j]),stroke:"var(--line2)","stroke-width":2},e>=n-1?{"stroke-dasharray":"5 4"}:{}))));
    POS.forEach((p,i)=>svg.append(tri(p[0]-2,p[1]-1,TH[i],"var(--muted)",0.35)));
    th.forEach((a,i)=>svg.append(tri(o.T[0][i],o.T[1][i],a,i===k?"var(--c3)":"var(--acc)",1),txt(X(o.T[0][i]),Y(o.T[1][i])-13,String(i+1),{"text-anchor":"middle","font-size":12,"data-top":1})));
    st.innerHTML=""; st.append(stat("Rotation term",o.fr.toFixed(2),"κ-weighted rotation residuals"),stat("Translation term",o.ft.toFixed(2),"τ-weighted residuals at t*(R)"),stat("Sum: objective (6.9)",(o.fr+o.ft).toFixed(2),"with t = t*(R)"),stat("tr(Q̃RᵀR), (6.17a)",o.tr.toFixed(2),"no translations needed"));
    const pts=[]; for(let t=-180;t<=180;t+=5) pts.push({x:t,y:cost(Q,fromTh(ths(t)))}); const ymax=Math.ceil(Math.max(...pts.map(p=>p.y))/200)*200;
    chart.innerHTML=""; chart.append(lineChart({h:220,aria:"Reduced cost as a function of the twist angle of the selected pose",x:{min:-180,max:180,ticks:[-120,-60,0,60,120].map(v=>[v,v+"°"]),label:"twist of pose "+(k+1)},
      y:{min:0,max:ymax,ticks:[0,ymax/2,ymax].map(v=>[v,String(v)]),label:"tr(Q̃RᵀR)"},series:[{name:"reduced cost",color:"var(--acc)",dots:false,points:pts}],marker:{x:tw,y:o.tr}})); };
  el.append(h("div",{class:"vt"},"Translations follow rotations"),h("p",{class:"vd"},"Six planar poses, seven noisy relative-pose measurements (κ = 50, τ = 100). You set the rotations; the positions are recomputed from (6.16)."),
    h("div",{class:"ctl"},select("Pose to twist",[1,2,3,4,5,6].map(i=>[i-1,"pose "+i]),v=>{ k=+v; draw(); }).el,range("Twist added to its true heading",-180,180,5,tw,v=>{ tw=v; draw(); },v=>v+"°").el),
    svg,h("div",{class:"legend"},...[["var(--muted)","true poses"],["var(--acc)","rotations you set, positions t*(R)"],["var(--c3)","the twisted pose"]].map(([c,t])=>h("span",{},h("i",{style:"background:"+c}),t))),st,chart);
  el.querySelector("select").value=String(k); draw(); };
