// Week 1 widgets for Days 2–5 (Day 1's widget is in d1.js).
const w1legend=items=>h("div",{class:"legend"},...items.map(([c,t])=>h("span",{},h("i",{style:"background:"+c}),t)));
const w1clamp=(v,lo,hi)=>Math.max(lo,Math.min(hi,v));

// Day 2: Bayes' rule for one scalar. Gaussian prior times Gaussian likelihood gives a Gaussian posterior.
VIZ.w01_bayes1d=el=>{
  const W=600, H=300, L=20, R=580, B=246, T=16, x0=-2, x1=8;
  let m0=2, s0=1, z=4, sz=0.5;
  const pdf=(x,m,s)=>Math.exp(-0.5*((x-m)/s)**2)/(s*Math.sqrt(2*Math.PI));
  const X=x=>L+(x-x0)/(x1-x0)*(R-L);
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Three bell curves over the robot's position: the prior, the likelihood of the measurement, and the narrower posterior that lies between them"});
  const st=h("div",{class:"stats"});
  const draw=()=>{
    const lam=1/(s0*s0)+1/(sz*sz), s=1/Math.sqrt(lam), m=(m0/(s0*s0)+z/(sz*sz))/lam;       // precisions add; the mean is precision-weighted
    const top=1.1*pdf(m,m,s), Y=p=>B-p/top*(B-T);
    const d=(mu,sg)=>{ let q=""; for(let i=0;i<=240;i++){ const x=x0+(x1-x0)*i/240; q+=(i?"L":"M")+X(x).toFixed(1)+" "+Y(pdf(x,mu,sg)).toFixed(1); } return q; };
    const curve=(mu,sg,c)=>sv("path",{d:d(mu,sg),fill:"none",stroke:c,"stroke-width":2.5});
    svg.innerHTML="";
    svg.append(sv("path",{d:d(m,s)+" L"+R+" "+B+" L"+L+" "+B+" Z",fill:"var(--c1-soft)",stroke:"none"}));
    for(let v=x0;v<=x1;v++){ svg.append(sv("line",{x1:X(v),x2:X(v),y1:B,y2:B+5,stroke:"var(--line2)"})); if(v%2===0) svg.append(txt(X(v),B+18,v+" m",{"text-anchor":"middle",class:"mut","font-size":11})); }
    svg.append(sv("line",{x1:L,x2:R,y1:B,y2:B,stroke:"var(--line2)"}),
      sv("line",{x1:X(z),x2:X(z),y1:B,y2:Y(pdf(z,z,sz)),stroke:"var(--c2)","stroke-width":1.5,"stroke-dasharray":"4 4"}),
      sv("line",{x1:X(m),x2:X(m),y1:B,y2:Y(pdf(m,m,s)),stroke:"var(--c1)","stroke-width":1.5,"stroke-dasharray":"4 4"}),
      curve(m0,s0,"var(--c5)"),curve(z,sz,"var(--c2)"),curve(m,s,"var(--c1)"),
      txt(w1clamp(X(m),L+40,R-40),B+34,"MAP "+m.toFixed(2),{"text-anchor":"middle",fill:"var(--c1)","font-weight":700}),
      txt(w1clamp(X(z),L+40,R-40),B+50,"MLE "+z.toFixed(2),{"text-anchor":"middle",fill:"var(--c2)","font-weight":700}));
    st.innerHTML=""; st.append(stat("Maximum likelihood (MLE)",z.toFixed(2)+" m","the measurement alone"),
      stat("Maximum a posteriori (MAP)",m.toFixed(2)+" m","prior and measurement"),
      stat("Posterior standard deviation",s.toFixed(3)+" m","prior "+s0.toFixed(1)+", measurement "+sz.toFixed(1)),
      stat("Weight on the measurement",(s0*s0/(s0*s0+sz*sz)).toFixed(2),"MAP = prior mean + weight × (z − prior mean)")); };
  el.append(h("div",{class:"vt"},"Prior × likelihood = posterior"),h("p",{class:"vd"},"A robot's position x along a corridor. The prior comes from odometry, the measurement z from a range sensor. The posterior is computed with Bayes' rule."),
    h("div",{class:"ctl"},range("Prior mean",0,6,0.1,m0,v=>{ m0=v; draw(); },v=>v.toFixed(1)+" m").el,range("Prior σ",0.2,3,0.1,s0,v=>{ s0=v; draw(); },v=>v.toFixed(1)+" m").el),
    h("div",{class:"ctl"},range("Measurement z",0,6,0.1,z,v=>{ z=v; draw(); },v=>v.toFixed(1)+" m").el,range("Measurement σ",0.2,3,0.1,sz,v=>{ sz=v; draw(); },v=>v.toFixed(1)+" m").el),
    svg,w1legend([["var(--c5)","prior p(x)"],["var(--c2)","likelihood p(z | x)"],["var(--c1)","posterior p(x | z)"]]),st);
  draw();
};

// Day 3: fuse two 2-D Gaussian estimates of the same point by adding information matrices and information vectors.
VIZ.w01_fuse2d=el=>{
  const W=600, H=360, sc=56, cx=300, cy=180, SL=1.5, SS=0.4, A0=[-1.6,-0.2], B0=[0.6,-1.2];
  let A=[...A0], B=[...B0], angA=20, angB=110;
  const cov=deg=>{ const c=Math.cos(deg*Math.PI/180), s=Math.sin(deg*Math.PI/180), a=SL*SL, b=SS*SS; return [[a*c*c+b*s*s,(a-b)*c*s],[(a-b)*c*s,a*s*s+b*c*c]]; };   // R diag(a,b) Rᵀ
  const flip=S=>[[S[0][0],-S[0][1]],[-S[1][0],S[1][1]]];                 // the maths has y up, the screen has y down
  const X=p=>cx+p[0]*sc, Y=p=>cy-p[1]*sc, mat=S=>"["+S[0][0].toFixed(2)+"  "+S[0][1].toFixed(2)+" ;  "+S[1][0].toFixed(2)+"  "+S[1][1].toFixed(2)+"]";
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Two elongated one-sigma uncertainty ellipses, A and B, with draggable centres, and the smaller ellipse of the fused estimate"});
  const g=sv("g"), out=h("div",{class:"panel"}), st=h("div",{class:"stats"});
  const handle=(name,col)=>sv("g",{},sv("circle",{r:12,fill:"var(--surface)",stroke:col,"stroke-width":3}),txt(0,4.5,name,{"text-anchor":"middle","font-weight":700,fill:"var(--ink)","pointer-events":"none"}));
  const hA=handle("A","var(--c5)"), hB=handle("B","var(--c2)");
  const draw=()=>{
    const SA=cov(angA), SB=cov(angB), LA=M.inv(SA), LB=M.inv(SB), Lm=M.add(LA,LB);     // information matrices add
    const ea=M.mv(LA,A), eb=M.mv(LB,B), eta=[ea[0]+eb[0],ea[1]+eb[1]];                  // information vectors add
    const mu=M.solve(Lm,eta), S=M.inv(Lm), ev=M.eigSym(S).val;
    g.innerHTML="";
    g.append(sv("line",{x1:0,x2:W,y1:cy,y2:cy,stroke:"var(--line)"}),sv("line",{x1:cx,x2:cx,y1:0,y2:H,stroke:"var(--line)"}),
      covEllipse(flip(S),X(mu),Y(mu),1,sc,{stroke:"var(--c1)",fill:"var(--c1-soft)","stroke-width":2.5}),
      covEllipse(flip(SA),X(A),Y(A),1,sc,{stroke:"var(--c5)"}),covEllipse(flip(SB),X(B),Y(B),1,sc,{stroke:"var(--c2)"}),
      sv("circle",{cx:X(mu),cy:Y(mu),r:4.5,fill:"var(--c1)"}));
    [[hA,A],[hB,B]].forEach(([e,p])=>{ const c=e.firstChild, t=e.lastChild; c.setAttribute("cx",X(p).toFixed(1)); c.setAttribute("cy",Y(p).toFixed(1)); t.setAttribute("x",X(p).toFixed(1)); t.setAttribute("y",(Y(p)+4.5).toFixed(1)); });
    out.innerHTML=""; out.append(h("div",{class:"out"},"Λ(A) = "+mat(LA)),h("div",{class:"out"},"Λ(B) = "+mat(LB)),h("div",{class:"out"},"Λ(A) + Λ(B) = "+mat(Lm)+"   ← information of the fused estimate"));
    st.innerHTML=""; st.append(stat("Fused mean","("+mu[0].toFixed(2)+", "+mu[1].toFixed(2)+")","A ("+A[0].toFixed(2)+", "+A[1].toFixed(2)+") · B ("+B[0].toFixed(2)+", "+B[1].toFixed(2)+")"),
      stat("Fused ellipse half-axes",Math.sqrt(ev[1]).toFixed(2)+" × "+Math.sqrt(ev[0]).toFixed(2),"1-sigma. A and B: "+SL.toFixed(2)+" × "+SS.toFixed(2)+" each"),
      stat("Fused ellipse area",Math.round(100*Math.sqrt(ev[0]*ev[1])/(SL*SS))+"% of A's","smaller means more certain")); };
  const mover=set=>(x,y)=>{ set([w1clamp((x-cx)/sc,-4.8,4.8),w1clamp((cy-y)/sc,-2.8,2.8)]); draw(); };
  svg.append(g,hA,hB); drag(svg,hA,mover(p=>A=p)); drag(svg,hB,mover(p=>B=p));
  el.append(h("div",{class:"vt"},"Fusing two estimates of one point"),h("p",{class:"vd"},"A and B are two independent Gaussian estimates of the same landmark position. Each is sure across its ellipse and unsure along it. Drag A or B; turn the ellipses with the sliders."),
    h("div",{class:"ctl"},range("Direction of A's long axis",0,180,5,angA,v=>{ angA=v; draw(); },v=>v+"°").el,range("Direction of B's long axis",0,180,5,angB,v=>{ angB=v; draw(); },v=>v+"°").el),
    h("div",{class:"ctl"},button("Reset positions",()=>{ A=[...A0]; B=[...B0]; draw(); })),
    svg,w1legend([["var(--c5)","estimate A"],["var(--c2)","estimate B"],["var(--c1)","fused estimate"]]),out,st);
  draw();
};

// Day 4: weighted least-squares line fit y = x1 + x2 t through draggable points, solved with the weighted normal equations.
VIZ.w01_linefit=el=>{
  const W=600, H=330, K=5, X=t=>40+t*54, Y=y=>300-y*40, P0=[[1,1.4],[2.5,2.4],[4,2.8],[5.5,3.9],[7,4.4],[9,2]];   // point K is the amber one
  let P=P0.map(p=>[...p]), sg=1;
  const fit=w=>{ let a=0,b=0,c=0,d=0,e=0; P.forEach(([t,y],i)=>{ a+=w[i]; b+=w[i]*t; c+=w[i]*t*t; d+=w[i]*y; e+=w[i]*t*y; });
    return M.solve([[a,b],[b,c]],[d,e]); };                               // (AᵀWA) x = AᵀWb with rows [1, t_i] and W = diag(1/σ_i²)
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Six draggable data points with two fitted lines: the weighted least-squares line and the equal-weight line, and the vertical residual of each point"});
  const g=sv("g"), st=h("div",{class:"stats"});
  const dots=P.map((_,i)=>sv("circle",{r:i===K?11:9,fill:i===K?"var(--c2)":"var(--ink)",stroke:"var(--surface)","stroke-width":2}));
  const eq=x=>x?x[0].toFixed(2)+(x[1]<0?" − ":" + ")+Math.abs(x[1]).toFixed(2)+" t":"no unique line";
  const draw=()=>{
    const w=P.map((_,i)=>i===K?1/(sg*sg):1), x=fit(w), xe=fit(P.map(()=>1)), line=(q,a)=>sv("line",Object.assign({x1:X(0),y1:Y(q[0]),x2:X(10),y2:Y(q[0]+10*q[1])},a));
    g.innerHTML="";
    g.append(sv("line",{x1:X(0),x2:X(10),y1:Y(0),y2:Y(0),stroke:"var(--line2)"}),sv("line",{x1:X(0),x2:X(0),y1:Y(0),y2:Y(7),stroke:"var(--line2)"}));
    for(let v=2;v<=10;v+=2) g.append(txt(X(v),Y(0)+17,"t = "+v,{"text-anchor":"middle",class:"mut","font-size":11}));
    for(let v=2;v<=6;v+=2) g.append(txt(X(0)-6,Y(v)+4,v,{"text-anchor":"end",class:"mut","font-size":11}));
    let cost=0, rk=0;
    if(xe) g.append(line(xe,{stroke:"var(--muted)","stroke-width":1.5,"stroke-dasharray":"6 5"}));
    if(x){ P.forEach(([t,y],i)=>{ const r=y-x[0]-x[1]*t; cost+=w[i]*r*r; if(i===K) rk=r; g.append(sv("line",{x1:X(t),x2:X(t),y1:Y(y),y2:Y(y-r),stroke:i===K?"var(--c2)":"var(--c3)","stroke-width":2})); });
      g.append(line(x,{stroke:"var(--c1)","stroke-width":2.5})); }
    dots.forEach((c,i)=>{ c.setAttribute("cx",X(P[i][0]).toFixed(1)); c.setAttribute("cy",Y(P[i][1]).toFixed(1)); });
    st.innerHTML=""; st.append(stat("Weighted fit",eq(x),"amber point has σ = "+sg.toFixed(1)+", the others σ = 1"),stat("Equal-weight fit",eq(xe),"every σ = 1"),
      stat("Weighted cost",x?cost.toFixed(2):"—","sum of (residual ÷ σ)²"),stat("Amber point's residual",x?rk.toFixed(2):"—",x?"= "+(rk/sg).toFixed(2)+" of its own σ":"")); };
  svg.append(g,...dots);
  dots.forEach((c,i)=>drag(svg,c,(x,y)=>{ P[i]=[w1clamp((x-40)/54,0,10),w1clamp((300-y)/40,0,7)]; draw(); }));
  el.append(h("div",{class:"vt"},"Weighted line fit"),h("p",{class:"vd"},"Fits y = x₁ + x₂·t to six measurements by solving the weighted normal equations. Drag any point. The slider sets how much noise you believe the amber measurement has."),
    h("div",{class:"ctl"},range("σ of the amber point",0.2,5,0.1,sg,v=>{ sg=v; draw(); },v=>v.toFixed(1)).el,button("Reset points",()=>{ P=P0.map(p=>[...p]); draw(); })),
    svg,w1legend([["var(--c1)","weighted least squares"],["var(--muted)","equal weights"],["var(--c3)","residuals"],["var(--c2)","amber point and its residual"]]),st);
  draw();
};

// Day 5: the same small least-squares problem solved in single precision by the normal equations (Cholesky) and by QR.
// Unknowns: two poses x1, x2. Rows after whitening and scaling: odometry [-1, 1 | 1], GPS on x1 [e, 0 | 0.4e], GPS on x2 [0, e | 1.1e],
// with e = (odometry σ) / (GPS σ). As e shrinks, A becomes ill-conditioned: κ(A) = sqrt(2 + e²) / e.
VIZ.w01_cond=el=>{
  const f=Math.fround, id=v=>v;
  const dot=(u,v,q)=>{ let s=0; for(let i=0;i<u.length;i++) s=q(s+q(u[i]*v[i])); return s; };          // q rounds every operation: id = double, f = single
  const back=(r11,r12,r22,y1,y2,q)=>{ const x2=q(y2/r22); return [q(q(y1-q(r12*x2))/r11),x2]; };       // back-substitution R x = y
  const viaNE=(a1,a2,b,q)=>{ const g11=dot(a1,a1,q), g12=dot(a1,a2,q), g22=dot(a2,a2,q), r11=q(Math.sqrt(g11)), r12=q(g12/r11), d=q(g22-q(r12*r12));
    if(!(d>0)) return null;                                                                             // AᵀA is not positive definite in this arithmetic
    const r22=q(Math.sqrt(d)), y1=q(dot(a1,b,q)/r11), y2=q(q(dot(a2,b,q)-q(r12*y1))/r22); return back(r11,r12,r22,y1,y2,q); };
  const viaQR=(a1,a2,b,q)=>{ const r11=q(Math.sqrt(dot(a1,a1,q))), q1=a1.map(v=>q(v/r11)), r12=dot(q1,a2,q), v=a2.map((a,i)=>q(a-q(r12*q1[i]))), r22=q(Math.sqrt(dot(v,v,q)));
    if(!(r22>0)) return null;                                                                           // Gram–Schmidt on the columns of [A b]
    const q2=v.map(a=>q(a/r22)), y1=dot(q1,b,q), y2=dot(q2,b.map((bi,i)=>q(bi-q(y1*q1[i]))),q); return back(r11,r12,r22,y1,y2,q); };
  const run=ratio=>{ const e=1/ratio, a1=[-1,e,0].map(f), a2=[1,0,e].map(f), b=[1,0.4*e,1.1*e].map(f);
    const ref=viaQR(a1,a2,b,id), ne=viaNE(a1,a2,b,f), qr=viaQR(a1,a2,b,f), err=x=>x?Math.hypot(x[0]-ref[0],x[1]-ref[1])/Math.hypot(ref[0],ref[1]):1;
    return {ratio,e,k:Math.sqrt(2+e*e)/e,ne,qr,ene:err(ne),eqr:err(qr),one:f(1+f(f(e)*f(e)))}; };
  const lg=v=>Math.max(-9,Math.min(0,Math.log10(v))), efmt=v=>v>=1?"100% (failed)":v.toExponential(1), xfmt=x=>x?"("+x[0].toFixed(6)+", "+x[1].toFixed(6)+")":"failed: AᵀA is not positive definite";
  const sweep=[]; for(let s=0;s<=5.001;s+=0.25) sweep.push(run(Math.pow(10,s)));
  let s=2;
  const holder=h("div"), out=h("div",{class:"panel"}), st=h("div",{class:"stats"});
  const draw=()=>{ const o=run(Math.pow(10,s)), pf=v=>"10^"+v.toFixed(1);
    holder.innerHTML=""; holder.append(lineChart({h:290,aria:"Relative error of the solution against the condition number of A, for the normal equations and for QR, in single precision",
      x:{min:1,max:1e6,log:true,ticks:[[1,"1"],[10,"10"],[100,"100"],[1e3,"1e3"],[1e4,"1e4"],[1e5,"1e5"],[1e6,"1e6"]],label:"condition number κ(A)"},
      y:{min:-9,max:0.5,ticks:[[-8,"1e-8"],[-6,"1e-6"],[-4,"1e-4"],[-2,"0.01"],[0,"1"]],label:"relative error"},
      series:[{name:"normal equations + Cholesky",color:"var(--c3)",points:sweep.map(p=>({x:p.k,y:lg(p.ene),fmt:pf}))},{name:"QR",color:"var(--c1)",points:sweep.map(p=>({x:p.k,y:lg(p.eqr),fmt:pf}))}],
      marker:{x:o.k,y:lg(o.ene)}}));
    out.innerHTML=""; out.append(h("div",{class:"out"},"normal equations: x = "+xfmt(o.ne)),h("div",{class:"out"},"QR:               x = "+xfmt(o.qr)),
      h("div",{class:"out"},"1 + ε² in single precision = "+o.one.toPrecision(9)+(o.one===1?"   ← the ε² is gone":"")));
    st.innerHTML=""; st.append(stat("Condition number of A",o.k.toPrecision(3),"κ(A), with ε = "+o.e.toExponential(1)),stat("Condition number of AᵀA",(o.k*o.k).toExponential(1),"= κ(A)², what the normal equations see"),
      stat("Error, normal equations",efmt(o.ene),"relative to the exact answer"),stat("Error, QR",efmt(o.eqr),"relative to the exact answer")); };
  el.append(h("div",{class:"vt"},"Squaring the condition number"),h("p",{class:"vd"},"Two poses, one odometry measurement x₂ − x₁ = 1, and a GPS fix on each pose (0.4 and 1.1). The same least-squares problem is solved in single precision (about 7 digits) two ways and compared with the double-precision answer."),
    h("div",{class:"ctl"},range("GPS σ ÷ odometry σ  (= 1/ε)",0,5,0.25,s,v=>{ s=v; draw(); },v=>{ const r=Math.pow(10,v); return r<100?r.toPrecision(2):Math.round(r).toLocaleString("en-US"); }).el),
    holder,out,st);
  draw();
};
