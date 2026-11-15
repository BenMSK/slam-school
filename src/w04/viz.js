// Week 4 widgets: Lie groups. A planar pose (member of SE(2)) is stored as [x, y, θ]; a tangent vector as
// [ρ1, ρ2, θ], translation first, as in the book's ξ = [ρ; θ].
const DEG=Math.PI/180, wrap=a=>Math.atan2(Math.sin(a),Math.cos(a)), f2=v=>(Math.abs(v)<0.005?0:v).toFixed(2);
// V(θ) = (sin θ/θ) I + ((1−cos θ)/θ) S turns ρ into the translation of Exp(ξ); vab returns its two coefficients
const vab=th=>Math.abs(th)<1e-6?[1-th*th/6,th/2]:[Math.sin(th)/th,(1-Math.cos(th))/th];
const se2Exp=([r1,r2,th])=>{ const [a,b]=vab(th); return [a*r1-b*r2,b*r1+a*r2,th]; };
const se2Log=([x,y,t])=>{ const th=wrap(t), [a,b]=vab(th), d=a*a+b*b; return [(a*x+b*y)/d,(a*y-b*x)/d,th]; };
const se2Mul=(A,B)=>{ const c=Math.cos(A[2]), s=Math.sin(A[2]); return [A[0]+c*B[0]-s*B[1],A[1]+s*B[0]+c*B[1],A[2]+B[2]]; };
const se2Inv=([x,y,th])=>{ const c=Math.cos(th), s=Math.sin(th); return [-(c*x+s*y),s*x-c*y,-th]; };
// SO(3): wedge, Rodrigues' formula, logarithm, and the left Jacobian J(θ)
const hat=v=>[[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]], nrm=v=>Math.hypot(...v);
const poly=(K,a,b)=>M.add(M.add(M.eye(3),M.scale(K,a)),M.scale(M.mul(K,K),b));        // I + a K + b K²
const so3Exp=v=>{ const p=nrm(v); return poly(hat(v),p<1e-6?1:Math.sin(p)/p,p<1e-6?0.5:(1-Math.cos(p))/(p*p)); };
const so3J=v=>{ const p=nrm(v); return poly(hat(v),p<1e-6?0.5:(1-Math.cos(p))/(p*p),p<1e-6?1/6:(p-Math.sin(p))/(p*p*p)); };
const so3Log=R=>{ const p=Math.acos(Math.max(-1,Math.min(1,(R[0][0]+R[1][1]+R[2][2]-1)/2))), s=Math.sin(p), k=p<1e-6?0.5:s<1e-9?0:p/(2*s);
  return [R[2][1]-R[1][2],R[0][2]-R[2][0],R[1][0]-R[0][1]].map(x=>x*k); };
// mean and covariance of a list of vectors
const meanCov=V=>{ const n=V[0].length, m=new Array(n).fill(0), C=M.zeros(n,n); V.forEach(v=>v.forEach((x,i)=>m[i]+=x/V.length));
  V.forEach(v=>{ for(let i=0;i<n;i++) for(let j=0;j<n;j++) C[i][j]+=(v[i]-m[i])*(v[j]-m[j])/(V.length-1); }); return {m,C}; };
// a robot drawn as an arrowhead at screen (X, Y) pointing along heading th (y up in the world, down on screen)
const robot=(X,Y,th,col,a={})=>{ const c=Math.cos(th), s=Math.sin(th), p=(u,v)=>(X+u*c-v*s).toFixed(1)+" "+(Y-u*s-v*c).toFixed(1);
  return sv("path",Object.assign({d:"M"+p(12,0)+" L"+p(-7,7)+" L"+p(-3,0)+" L"+p(-7,-7)+" Z",fill:col,stroke:"var(--surface)","stroke-width":1.5},a)); };
const legend=items=>h("div",{class:"legend"},...items.map(([c,t])=>h("span",{},h("i",{style:"background:"+c}),t)));
const poly2=(pts,a)=>sv("path",Object.assign({d:pts.map((p,i)=>(i?"L":"M")+p[0].toFixed(1)+" "+p[1].toFixed(1)).join(" "),fill:"none","stroke-width":2.5,"stroke-linejoin":"round"},a));

// Day 1: the entry-wise average of two rotation matrices is not a rotation matrix.
VIZ.w04_avg=el=>{
  let a1=20, a2=110; const cx=150, cy=158, r=105, qx=450, qy=165, sc=70;
  const svg=sv("svg",{viewBox:"0 0 600 300",role:"img","aria-label":"Left: the circle of planar rotations with two rotations marked, the midpoint of the straight chord between them, and the midpoint along the circle. Right: a unit square transformed by the entry-wise average matrix and by the midpoint on the circle."});
  const st=h("div",{class:"stats"});
  const draw=()=>{ const t1=a1*DEG, t2=a2*DEG, d=wrap(t2-t1), tm=t1+d/2, A=(Math.cos(t1)+Math.cos(t2))/2, B=(Math.sin(t1)+Math.sin(t2))/2, det=A*A+B*B;
    const P=t=>[cx+r*Math.cos(t),cy-r*Math.sin(t)], sq=(a,b)=>[[0,0],[1,0],[1,1],[0,1],[0,0]].map(([x,y])=>[qx+sc*(a*x-b*y),qy-sc*(b*x+a*y)]);
    svg.innerHTML="";
    svg.append(txt(cx,16,"each rotation is a point on this circle",{"text-anchor":"middle",class:"mut","font-size":12}),txt(qx,16,"what each matrix does to a unit square",{"text-anchor":"middle",class:"mut","font-size":12}),
      sv("circle",{cx,cy,r,fill:"none",stroke:"var(--line2)","stroke-width":2}),sv("circle",{cx,cy,r:2.5,fill:"var(--muted)"}),
      sv("line",{x1:P(t1)[0],y1:P(t1)[1],x2:P(t2)[0],y2:P(t2)[1],stroke:"var(--c3)","stroke-width":2,"stroke-dasharray":"4 4"}),
      sv("circle",{cx:P(t1)[0],cy:P(t1)[1],r:6,fill:"var(--c5)"}),sv("circle",{cx:P(t2)[0],cy:P(t2)[1],r:6,fill:"var(--c4)"}),
      sv("circle",{cx:P(tm)[0],cy:P(tm)[1],r:6,fill:"var(--c1)"}),sv("circle",{cx:cx+r*A,cy:cy-r*B,r:6,fill:"var(--c3)"}),
      poly2(sq(1,0),{stroke:"var(--line2)","stroke-dasharray":"5 4","stroke-width":1.5}),poly2(sq(Math.cos(tm),Math.sin(tm)),{stroke:"var(--c1)"}),poly2(sq(A,B),{stroke:"var(--c3)",fill:"var(--c3-soft)"}),
      sv("circle",{cx:qx,cy:qy,r:2.5,fill:"var(--muted)"}));
    st.innerHTML=""; st.append(stat("Entry-wise average ½(R₁ + R₂)","["+f2(A)+" "+f2(-B)+"; "+f2(B)+" "+f2(A)+"]","= "+f2(Math.sqrt(det))+" × a rotation"),
      stat("Its determinant",f2(det),"a rotation matrix needs exactly 1"),stat("Midpoint on the circle","R("+Math.round(wrap(tm)/DEG)+"°)","R₁ Exp(½ Log(R₁⁻¹R₂)), determinant 1")); };
  el.append(h("div",{class:"vt"},"Averaging two rotations"),h("p",{class:"vd"},"Adds the two 2×2 rotation matrices entry by entry, halves the sum, and tests whether the result is still a rotation."),
    h("div",{class:"ctl"},range("Angle of R₁",-180,180,1,a1,v=>{ a1=v; draw(); },v=>v+"°").el,range("Angle of R₂",-180,180,1,a2,v=>{ a2=v; draw(); },v=>v+"°").el),
    svg,legend([["var(--c5)","R₁"],["var(--c4)","R₂"],["var(--c3)","entry-wise average"],["var(--c1)","midpoint on the circle"],["var(--line2)","unit square"]]),st);
  draw();
};

// Day 2: Exp and Log on SE(2). The path s ↦ Exp(sξ), s in [0, 1], is a circular arc (a straight line when θ = 0).
VIZ.w04_exp=el=>{
  let r1=1.57, r2=0, th=90, split=false; const W=600, H=420, ox=300, oy=210, sc=70, X=x=>ox+x*sc, Y=y=>oy-y*sc;
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Top view of the plane with a robot at the origin, the arc traced by Exp of s times xi as s goes from 0 to 1, and the final pose Exp(xi)"});
  const st=h("div",{class:"stats"}), mat=h("div",{class:"panel"});
  const draw=()=>{ const xi=[r1,r2,th*DEG], T=se2Exp(xi), L=se2Log(T), Q=se2Mul(se2Exp([r1,r2,0]),se2Exp([0,0,xi[2]]));
    svg.innerHTML="";
    for(let i=-4;i<=4;i++) for(let j=-2;j<=2;j++) svg.append(sv("circle",{cx:X(i),cy:Y(j),r:i||j?1.3:0,fill:"var(--line2)"}));
    if(Math.abs(xi[2])>1e-3){ const c=[-r2/xi[2],r1/xi[2]]; if(Math.abs(c[0])<4.2&&Math.abs(c[1])<2.9){
      svg.append(sv("line",{x1:X(c[0]),y1:Y(c[1]),x2:X(0),y2:Y(0),stroke:"var(--line2)","stroke-dasharray":"3 4"}),sv("line",{x1:X(c[0]),y1:Y(c[1]),x2:X(T[0]),y2:Y(T[1]),stroke:"var(--line2)","stroke-dasharray":"3 4"}),
        sv("circle",{cx:X(c[0]),cy:Y(c[1]),r:4,fill:"var(--c2)"})); } }
    const pts=[]; for(let i=0;i<=60;i++){ const p=se2Exp(xi.map(v=>v*i/60)); pts.push([X(p[0]),Y(p[1])]); }
    svg.append(poly2(pts,{stroke:"var(--c5)"}));
    [0.25,0.5,0.75].forEach(s=>{ const p=se2Exp(xi.map(v=>v*s)); svg.append(robot(X(p[0]),Y(p[1]),p[2],"var(--c5)",{opacity:0.4})); });
    if(split) svg.append(poly2([[X(0),Y(0)],[X(Q[0]),Y(Q[1])]],{stroke:"var(--c3)","stroke-dasharray":"6 4"}),robot(X(Q[0]),Y(Q[1]),Q[2],"var(--c3)"));
    svg.append(robot(X(0),Y(0),0,"var(--muted)"),robot(X(T[0]),Y(T[1]),T[2],"var(--c5)"));
    const c=Math.cos(T[2]), s=Math.sin(T[2]), n=v=>f2(v).replace("-","{-}");
    mat.innerHTML=""; mat.append(tex("\\boldsymbol{\\xi}^\\wedge=\\begin{bmatrix}0 & "+n(-xi[2])+" & "+n(r1)+"\\\\ "+n(xi[2])+" & 0 & "+n(r2)+"\\\\ 0 & 0 & 0\\end{bmatrix}\\;\\xrightarrow{\\ \\exp\\ }\\;"),
      tex("\\mathbf{T}=\\begin{bmatrix}"+n(c)+" & "+n(-s)+" & "+n(T[0])+"\\\\ "+n(s)+" & "+n(c)+" & "+n(T[1])+"\\\\ 0 & 0 & 1\\end{bmatrix}"));
    st.innerHTML=""; st.append(stat("Translation of the pose","("+f2(T[0])+", "+f2(T[1])+")","V(θ)ρ, while ρ itself is ("+f2(r1)+", "+f2(r2)+")"),
      stat("Log of the pose","("+f2(L[0])+", "+f2(L[1])+", "+Math.round(L[2]/DEG)+"°)","the round trip returns ξ"),
      stat("Translate, then rotate","("+f2(Q[0])+", "+f2(Q[1])+")","Exp(ρ only) Exp(θ only), "+f2(Math.hypot(Q[0]-T[0],Q[1]-T[1]))+" away from Exp(ξ)")); };
  const sb=button("Show translate, then rotate",()=>{ split=!split; sb.classList.toggle("on",split); draw(); });
  el.append(h("div",{class:"vt"},"Exp and Log on SE(2)"),h("p",{class:"vd"},"Computes T = Exp(ξ) for ξ = (ρ₁, ρ₂, θ) and draws the poses Exp(sξ) for s from 0 to 1. Dots are one unit apart."),
    h("div",{class:"ctl"},range("ρ₁ (forward)",-3.14,3.14,0.01,r1,v=>{ r1=v; draw(); },v=>v.toFixed(2)).el,range("ρ₂ (sideways)",-1.5,1.5,0.01,r2,v=>{ r2=v; draw(); },v=>v.toFixed(2)).el,range("θ (turn)",-180,180,1,th,v=>{ th=v; draw(); },v=>v+"°").el),
    h("div",{class:"ctl"},sb),svg,legend([["var(--muted)","identity (start)"],["var(--c5)","Exp(sξ), ending at Exp(ξ)"],["var(--c2)","centre of the arc"],["var(--c3)","Exp(ρ only) Exp(θ only)"]]),mat,st);
  draw();
};

// Day 3: Gauss-Newton on SE(2) for a planar PnP problem. Unknown: T (world → camera). A 1-D camera sees known landmarks:
// p = T ℓ̃, u = p_y / p_x. Each step solves the normal equations for ξ and retracts with T ← T Exp(ξ).
VIZ.w04_pnp=el=>{
  const LM=[[6,3],[7.5,1.2],[6.5,-1],[8,-2.6],[5,1.6]], CAM=[1,-0.5,15*DEG], SIG=0.005, W=600, H=425, X=x=>96+48*x, Y=y=>165-48*y, U=u=>300+260*Math.max(-1.1,Math.min(1.1,u));
  const cam=(T,l)=>{ const c=Math.cos(T[2]), s=Math.sin(T[2]); return [Math.max(0.05,c*l[0]-s*l[1]+T[0]),s*l[0]+c*l[1]+T[1]]; };
  const nz=rng(11), Z=LM.map(l=>{ const p=cam(se2Inv(CAM),l); return p[1]/p[0]+SIG*nz.n(); });
  const lin=T=>{ const c=Math.cos(T[2]), s=Math.sin(T[2]), A=M.zeros(3,3), b=[0,0,0]; let cost=0;
    LM.forEach((l,i)=>{ const p=cam(T,l), e=Z[i]-p[1]/p[0], g=[-p[1]/(p[0]*p[0]),1/p[0]], q=[-l[1],l[0]];      // g = ∂h/∂p, q = Sℓ
      const Hr=[g[0]*c+g[1]*s,-g[0]*s+g[1]*c,g[0]*(c*q[0]-s*q[1])+g[1]*(s*q[0]+c*q[1])];                     // row of H = g R⁰ [I  Sℓ]
      for(let a=0;a<3;a++){ b[a]+=Hr[a]*e/(SIG*SIG); for(let k=0;k<3;k++) A[a][k]+=Hr[a]*Hr[k]/(SIG*SIG); } cost+=e*e/(SIG*SIG); });
    return {A,b,cost}; };
  let dth=25, dy=1, T, hist, last;
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Top: map with five known landmarks, the true camera pose, and the current estimate with its sight lines. Bottom: the one-dimensional image, with measured and predicted landmark positions."});
  const st=h("div",{class:"stats"}), out=h("div",{class:"out"});
  const draw=()=>{ const G=se2Inv(T), gx=X(G[0]), gy=Y(G[1]); svg.innerHTML="";
    LM.forEach(l=>svg.append(sv("line",{x1:gx,y1:gy,x2:X(l[0]),y2:Y(l[1]),stroke:"var(--c3)","stroke-width":1,opacity:0.45})));
    LM.forEach(l=>svg.append(sv("circle",{cx:X(l[0]),cy:Y(l[1]),r:6,fill:"var(--c5)"})));
    svg.append(robot(X(CAM[0]),Y(CAM[1]),CAM[2],"none",{stroke:"var(--ink)","stroke-dasharray":"3 3"}),robot(gx,gy,G[2],"var(--c3)"),
      sv("line",{x1:U(-1.1),y1:385,x2:U(1.1),y2:385,stroke:"var(--line2)","stroke-width":1.5}),txt(8,363,"the camera's 1-D image",{class:"mut","font-size":12}));
    [-1,-0.5,0,0.5,1].forEach(u=>svg.append(txt(U(u),418,"u = "+u,{"text-anchor":"middle",class:"mut","font-size":11})));
    LM.forEach((l,i)=>{ const p=cam(T,l), u=p[1]/p[0];
      svg.append(sv("line",{x1:U(Z[i]),y1:385,x2:U(u),y2:385,stroke:"var(--c2)","stroke-width":5,opacity:0.6}),
        sv("line",{x1:U(Z[i]),y1:368,x2:U(Z[i]),y2:385,stroke:"var(--c5)","stroke-width":3}),sv("line",{x1:U(u),y1:385,x2:U(u),y2:402,stroke:"var(--c3)","stroke-width":3})); });
    const C=lin(T).cost; st.innerHTML="";
    st.append(stat("Steps taken",hist.length-1,last?"last ξ* = ("+last.map(v=>v.toFixed(3)).join(", ")+")":"press the button"),stat("Cost",C.toFixed(1),"sum of (residual / σ)², σ = 0.005"),
      stat("Position error",Math.hypot(G[0]-CAM[0],G[1]-CAM[1]).toFixed(3),"camera position, in map units"),stat("Heading error",(wrap(G[2]-CAM[2])/DEG).toFixed(2)+"°","estimate minus truth"));
    out.textContent="cost: "+hist.slice(-7).map(v=>v.toFixed(1)).join(" → "); };
  const reset=()=>{ T=se2Inv([CAM[0],CAM[1]+dy,CAM[2]+dth*DEG]); hist=[lin(T).cost]; last=null; draw(); };
  const step=()=>{ const {A,b,cost}=lin(T), xi=M.solve(A,b); if(!xi) return;
    for(let k=1;k>0.01;k/=2){ const xk=xi.map(v=>v*k), Tn=se2Mul(T,se2Exp(xk));                      // halve the step if it would raise the cost
      if(lin(Tn).cost<=cost){ T=Tn; last=xk; hist.push(lin(T).cost); break; } }
    draw(); };
  el.append(h("div",{class:"vt"},"Gauss-Newton on SE(2): planar PnP"),h("p",{class:"vd"},"Each press linearizes the five measurements about the current pose, solves the 3×3 normal equations for ξ, and updates T ← T Exp(ξ). A step is halved if it would raise the cost."),
    h("div",{class:"ctl"},range("Initial heading error",-30,30,1,dth,v=>{ dth=v; reset(); },v=>v+"°").el,range("Initial sideways error",-1.5,1.5,0.1,dy,v=>{ dy=v; reset(); },v=>v.toFixed(1)).el),
    h("div",{class:"ctl"},button("Gauss-Newton step",step,"primary"),button("Reset",reset)),svg,
    legend([["var(--c5)","landmarks and measured u"],["var(--c3)","estimate and predicted u"],["var(--c2)","residual"],["var(--ink)","true camera (dashed)"]]),out,st);
  reset();
};

// Day 4: the banana. N robots each take K unit steps forward; every step is Exp of (1, 0, 0) plus Gaussian noise in the
// tangent space. The cloud of end positions is compared with a Gaussian fitted in (x, y) and one fitted in the tangent space.
VIZ.w04_banana=el=>{
  let K=15, sth=6, sr=0.05, seed=1, sc=22; const N=500, W=600, H=440, X=x=>40+x*sc, Y=y=>220-y*sc;
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Scatter of the end positions of 500 simulated robots, the two-sigma ellipse of a Gaussian fitted to x and y, and the curved two-sigma contour of a Gaussian fitted in the tangent space"});
  const st=h("div",{class:"stats"});
  const draw=()=>{ const r=rng(seed), Tb=[K,0,0], Ti=se2Inv(Tb), P=[], Xi=[];
    for(let i=0;i<N;i++){ let T=[0,0,0]; for(let k=0;k<K;k++) T=se2Mul(T,se2Exp([1+sr*r.n(),sr*r.n(),sth*DEG*r.n()])); P.push(T); Xi.push(se2Log(se2Mul(Ti,T))); }
    const g2=meanCov(P.map(p=>[p[0],p[1]])), g3=meanCov(Xi), E=M.eigSym(g3.C), ax=j=>E.vec.map(row=>row[j]*Math.sqrt(Math.max(E.val[j],0))), va=ax(2), vb=ax(1), S=g2.C;
    sc=Math.min(520/(K+1.5),212/(2.3*Math.sqrt(S[1][1])+0.5));            // fit the path and the cloud to the canvas
    const tan=(a,b)=>se2Mul(Tb,se2Exp(g3.m.map((m,i)=>m+a*va[i]+b*vb[i]))), cont=[]; for(let i=0;i<=72;i++){ const p=tan(2*Math.cos(i*5*DEG),2*Math.sin(i*5*DEG)); cont.push([X(p[0]),Y(p[1])]); }
    const pm=tan(0,0), sdx=Math.sqrt(S[0][0]), skew=sdx>1e-9?P.reduce((s,p)=>s+Math.pow((p[0]-g2.m[0])/sdx,3),0)/N:0;
    svg.innerHTML=""; svg.append(sv("line",{x1:X(0),y1:Y(0),x2:X(K),y2:Y(0),stroke:"var(--line2)","stroke-width":2,"stroke-dasharray":"6 5"}));
    P.forEach(p=>svg.append(sv("circle",{cx:X(p[0]).toFixed(1),cy:Y(p[1]).toFixed(1),r:2,fill:"var(--c5)",opacity:0.55})));
    svg.append(covEllipse([[S[0][0],-S[0][1]],[-S[0][1],S[1][1]]],X(g2.m[0]),Y(g2.m[1]),2,sc,{stroke:"var(--c3)","stroke-width":2.5}),poly2(cont,{stroke:"var(--c1)"}),
      sv("circle",{cx:X(g2.m[0]),cy:Y(g2.m[1]),r:5,fill:"var(--c3)",stroke:"var(--surface)","stroke-width":1.5}),sv("circle",{cx:X(pm[0]),cy:Y(pm[1]),r:5,fill:"var(--c1)",stroke:"var(--surface)","stroke-width":1.5}),
      robot(X(0),Y(0),0,"var(--muted)"));
    st.innerHTML=""; st.append(stat("Heading spread at the end",(Math.sqrt(g3.C[2][2])/DEG).toFixed(1)+"°","standard deviation over "+N+" robots"),
      stat("Mean of (x, y)","("+g2.m[0].toFixed(2)+", "+g2.m[1].toFixed(2)+")","noise-free end is ("+K+", 0)"),
      stat("Tangent-space mean, as a position","("+pm[0].toFixed(2)+", "+pm[1].toFixed(2)+")","noise-free pose × Exp(mean of ξ)"),
      stat("Skewness of x",skew.toFixed(2),"a Gaussian has 0")); };
  el.append(h("div",{class:"vt"},"The banana distribution"),h("p",{class:"vd"},"500 robots each take K steps of one unit. Every step is Exp(u + δ) with u = (1, 0, 0) and Gaussian δ in the tangent space. Dots are where they end up. The view is scaled to fit."),
    h("div",{class:"ctl"},range("Steps K",1,24,1,K,v=>{ K=v; draw(); }).el,range("Heading noise per step",0,8,0.5,sth,v=>{ sth=v; draw(); },v=>v+"°").el,range("Translation noise per step",0,0.2,0.01,sr,v=>{ sr=v; draw(); },v=>v.toFixed(2)).el),
    h("div",{class:"ctl"},button("New samples",()=>{ seed++; draw(); })),svg,
    legend([["var(--c5)","end positions"],["var(--c3)","Gaussian in (x, y): mean and 2σ ellipse"],["var(--c1)","Gaussian in the tangent space: mean and 2σ contour"],["var(--line2)","noise-free path"]]),st);
  draw();
};

// Day 5: how good are θ₁ + θ₂ and the Jacobian formula (2.34) as approximations of Log(Exp(θ₁) Exp(θ₂)) on SO(3)?
VIZ.w04_bch=el=>{
  let ex=-1, a2=60, al=90; const box=h("div"), st=h("div",{class:"stats"}), sup=n=>"10"+String(n).replace("-","⁻").replace(/\d/g,d=>"⁰¹²³⁴⁵⁶⁷⁸⁹"[d]);
  const errs=(s,t2)=>{ const t1=[s*Math.sin(al*DEG),0,s*Math.cos(al*DEG)], exact=so3Log(M.mul(so3Exp(t1),so3Exp(t2))), jac=M.mv(M.inv(so3J(t2))||M.eye(3),t1).map((v,i)=>v+t2[i]);
    return {exact,e1:nrm(exact.map((v,i)=>v-t1[i]-t2[i])),e2:nrm(exact.map((v,i)=>v-jac[i]))}; };
  const draw=()=>{ const t2=[0,0,a2*DEG], lg=v=>Math.log10(Math.max(v,1e-8)), s1=[], s2=[];
    for(let e=-3;e<=0.001;e+=0.25){ const q=errs(Math.pow(10,e),t2), x=Math.pow(10,e), fmt=v=>Math.pow(10,v).toExponential(1)+" rad"; s1.push({x,y:lg(q.e1),fmt}); s2.push({x,y:lg(q.e2),fmt}); }
    const q=errs(Math.pow(10,ex),t2);
    box.innerHTML=""; box.append(lineChart({h:290,aria:"Error of the two approximations against the size of the first rotation, both on logarithmic axes",
      x:{min:0.001,max:1,log:true,ticks:[[0.001,"0.001"],[0.01,"0.01"],[0.1,"0.1"],[1,"1 rad"]],label:"size of θ₁ (rad)"},y:{min:-8,max:0,ticks:[-6,-4,-2,0].map(v=>[v,sup(v)]),label:"error (rad)"},
      series:[{name:"θ₁ + θ₂",color:"var(--c3)",points:s1},{name:"θ₂ + J(θ₂)⁻¹θ₁, equation (2.34)",color:"var(--c1)",points:s2}],marker:{x:Math.pow(10,ex),y:lg(q.e2)}}));
    st.innerHTML=""; st.append(stat("Exact combined rotation vector","("+q.exact.map(v=>v.toFixed(3)).join(", ")+")","Log(Exp(θ₁) Exp(θ₂)), in rad"),stat("Plain sum is off by",q.e1.toExponential(1)+" rad",(q.e1/DEG).toFixed(2)+"°"),
      stat("Equation (2.34) is off by",q.e2.toExponential(1)+" rad",(q.e2/DEG).toFixed(3)+"°")); };
  el.append(h("div",{class:"vt"},"Combining two rotations in the tangent space"),h("p",{class:"vd"},"θ₂ is a rotation about the z-axis. θ₁ is a smaller rotation about an axis tilted away from z. The chart compares two approximations of Log(Exp(θ₁) Exp(θ₂)) with the exact value."),
    h("div",{class:"ctl"},range("Size of θ₁",-3,0,0.1,ex,v=>{ ex=v; draw(); },v=>Math.pow(10,v).toFixed(3)+" rad").el,range("Size of θ₂",0,120,1,a2,v=>{ a2=v; draw(); },v=>v+"°").el,range("Angle between the two axes",0,180,5,al,v=>{ al=v; draw(); },v=>v+"°").el),
    box,st);
  draw();
};
