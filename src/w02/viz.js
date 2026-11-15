// Week 2 widgets: factor graphs, MAP as least squares, the normal equations, and SD / GN / LM.
// Poses are plain vectors [x, y, θ], which is how Chapter 1 treats them.
const wrap=a=>Math.atan2(Math.sin(a),Math.cos(a));
const sq=v=>v.reduce((s,x)=>s+x*x,0), I3=[[1,0,0],[0,1,0],[0,0,1]], SUB="₁₂₃₄₅₆₇₈₉";
// relative pose of q seen from p (the odometry and loop-closure measurement model) and its two Jacobian blocks
const rel=(p,q)=>{ const c=Math.cos(p[2]), s=Math.sin(p[2]), dx=q[0]-p[0], dy=q[1]-p[1];
  return {h:[c*dx+s*dy,-s*dx+c*dy,wrap(q[2]-p[2])], Hi:[[-c,-s,-s*dx+c*dy],[s,-c,-c*dx-s*dy],[0,0,-1]], Hj:[[c,s,0],[-s,c,0],[0,0,1]]}; };
// whitened system of (1.23): a factor is {at:[first column of each block], H:[Jacobian blocks], r: z - h, s:[sigma of each row]}
const stack=(n,fs)=>{ const A=[], b=[]; fs.forEach(f=>f.r.forEach((ri,k)=>{ const row=new Array(n).fill(0); f.at.forEach((o,q)=>f.H[q][k].forEach((v,c)=>{ row[o+c]=v/f.s[k]; })); A.push(row); b.push(ri/f.s[k]); })); return {A,b}; };
const normal=(A,b)=>{ const At=M.T(A); return {L:M.mul(At,A), g:M.mv(At,b)}; };      // Λ = AᵀA and Aᵀb of (1.25)
const legend=items=>h("div",{class:"legend"},...items.map(([c,t])=>h("span",{},h("i",{style:"background:"+c}),t)));
const fx=(v,d=2)=>(Math.abs(v)<0.5*10**-d?0:v).toFixed(d);                           // never prints "-0.00"

// Day 1: build a small SLAM problem, draw its factor graph, and count what the factors can and cannot pin down.
VIZ.w02_fg=el=>{
  const LMK=[[1.5,1.2],[3.2,1.3],[4.6,-1.2],[6.2,1.1],[7.6,-1.1]], W=600, H=300;
  let N=3, L=2, R=1.5; const on={prior:true,odo:true,lpri:false,range:false};
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Factor graph of the chosen SLAM problem. Large circles are pose and landmark variables, small dark dots are factors, and each factor is joined to the variables it depends on."});
  const st=h("div",{class:"stats"});
  const draw=()=>{ const P=Array.from({length:N},(_,i)=>[i+1,0,0]), obs=[], col={}; let n=3*N;
    for(let j=0;j<L;j++) for(let i=0;i<N;i++) if(Math.hypot(LMK[j][0]-P[i][0],LMK[j][1])<=R){ if(!(j in col)){ col[j]=n; n+=2; } obs.push([i,j]); }
    const seen=Object.keys(col).map(Number), fs=[], cnt=[];
    if(on.prior){ fs.push({at:[0],H:[I3],r:[0,0,0],s:[1,1,1]}); cnt.push("1 prior"); }
    if(on.odo){ for(let i=0;i+1<N;i++){ const e=rel(P[i],P[i+1]); fs.push({at:[3*i,3*i+3],H:[e.Hi,e.Hj],r:[0,0,0],s:[1,1,1]}); } cnt.push((N-1)+" odometry"); }
    obs.forEach(([i,j])=>{ const dx=LMK[j][0]-P[i][0], dy=LMK[j][1], q=dx*dx+dy*dy, d=Math.sqrt(q), Hp=[[dy/q,-dx/q,-1]], Hl=[[-dy/q,dx/q]];   // bearing = atan2(dy,dx) - θ
      if(on.range){ Hp.push([-dx/d,-dy/d,0]); Hl.push([dx/d,dy/d]); }
      fs.push({at:[3*i,col[j]],H:[Hp,Hl],r:Hp.map(()=>0),s:Hp.map(()=>1)}); });
    cnt.push(obs.length+(on.range?" bearing-range":" bearing"));
    if(on.lpri){ seen.forEach(j=>fs.push({at:[col[j]],H:[[[1,0],[0,1]]],r:[0,0],s:[1,1]})); cnt.push(seen.length+" landmark priors"); }
    const A=stack(n,fs).A, m=A.length; let rank=0;
    if(m){ const ev=M.eigSym(M.mul(M.T(A),A)).val; rank=ev.filter(v=>v>1e-9*Math.max(1,ev[ev.length-1])).length; }
    const xmax=Math.max(N,...seen.map(j=>LMK[j][0]))+0.5, sc=Math.min(92,570/(xmax-0.35)), x0=(W-(xmax-0.35)*sc)/2, X=x=>x0+(x-0.35)*sc, Y=y=>150-y*sc, dots=[];
    const ln=(a,b)=>{ svg.append(sv("line",{x1:X(a[0]),y1:Y(a[1]),x2:X(b[0]),y2:Y(b[1]),stroke:"var(--muted)","stroke-width":1.5})); return [(a[0]+b[0])/2,(a[1]+b[1])/2]; };
    svg.innerHTML=""; svg.setAttribute("viewBox","0 0 "+W+" "+([2,4].some(j=>j<L&&LMK[j][0]<=xmax-0.4)?H:190));   // no landmark below the path: crop the empty half
    if(on.prior){ ln([0.45,0],P[0]); dots.push([0.45,0]); }
    if(on.odo) for(let i=0;i+1<N;i++) dots.push(ln(P[i],P[i+1]));
    obs.forEach(([i,j])=>dots.push(ln(P[i],LMK[j])));
    if(on.lpri) seen.forEach(j=>{ const q=[LMK[j][0]+0.45,LMK[j][1]]; ln(LMK[j],q); dots.push(q); });
    dots.forEach(d=>svg.append(sv("circle",{cx:X(d[0]),cy:Y(d[1]),r:5,fill:"var(--ink)"})));
    P.forEach((p,i)=>svg.append(sv("circle",{cx:X(p[0]),cy:Y(0),r:16,fill:"var(--c5-soft)",stroke:"var(--c5)","stroke-width":1.5}),txt(X(p[0]),Y(0)+5,"p"+SUB[i],{"text-anchor":"middle"})));
    for(let j=0;j<L;j++){ if(LMK[j][0]>xmax-0.4) continue; const ok=j in col;
      svg.append(sv("circle",Object.assign({cx:X(LMK[j][0]),cy:Y(LMK[j][1]),r:16,fill:ok?"var(--c2-soft)":"var(--surface)",stroke:ok?"var(--c2)":"var(--line2)","stroke-width":1.5},ok?{}:{"stroke-dasharray":"4 3"})),
        txt(X(LMK[j][0]),Y(LMK[j][1])+5,"ℓ"+SUB[j],Object.assign({"text-anchor":"middle"},ok?{}:{class:"mut"}))); }
    st.innerHTML=""; st.append(stat("Variables",String(N+seen.length),N+" poses, "+seen.length+" landmarks"),stat("Factors",String(fs.length),cnt.join(", ")),
      stat("Unknown numbers",String(n),"3 per pose, 2 per landmark"),stat("Measured numbers",String(m),"one equation each"),
      stat("Undetermined directions",String(n-rank),n-rank?"the factors cannot fix these":"every unknown is pinned down")); };
  const tg=(label,k)=>{ const b=button(label,()=>{ on[k]=!on[k]; b.classList.toggle("on",on[k]); draw(); }); b.classList.toggle("on",on[k]); return b; };
  el.append(h("div",{class:"vt"},"Factor graph builder"),h("p",{class:"vd"},"A robot drives along a line and sees every landmark within its sensing range. Bearings are measured relative to the robot's heading. A dashed landmark is seen by no pose and is not part of the problem."),
    h("div",{class:"ctl"},range("Poses",2,8,1,N,v=>{ N=v; draw(); }).el,range("Landmarks",0,5,1,L,v=>{ L=v; draw(); }).el,range("Sensing range",1,3,0.1,R,v=>{ R=v; draw(); },v=>v.toFixed(1)).el),
    h("div",{class:"ctl"},tg("Prior on p₁","prior"),tg("Odometry","odo"),tg("Landmark priors","lpri"),tg("Measure range too","range")),svg,
    legend([["var(--c5)","pose variable"],["var(--c2)","landmark variable"],["var(--ink)","factor (dot)"]]),st);
  draw();
};

// Day 2: bearing factors on one unknown landmark. Each cell shows exp(-J/2), the product of the factors that are switched on.
VIZ.w02_rays=el=>{
  const W=600, H=360, C=6, sc=100, ox=100, oy=300, P=[[0,0],[2.5,0]], T=[2,1.6], MU=[2.6,1.1], S0=0.7;
  const Z=P.map((p,k)=>Math.atan2(T[1]-p[1],T[0]-p[0])+[0.03,-0.04][k]);            // the two measured bearings, each a little off
  let sig=5; const on=[true,false,false];
  const cv=h("canvas",{width:W,height:H,role:"img","aria-label":"Top view. Two robot poses each measure the bearing to one landmark. The shading shows how likely each landmark position is under the factors that are switched on."}), ctx=cv.getContext("2d");
  const st=h("div",{class:"stats"});
  const J=(x,y)=>{ let s=0; P.forEach((p,k)=>{ if(on[k]){ const e=wrap(Z[k]-Math.atan2(y-p[1],x-p[0]))/(sig*Math.PI/180); s+=e*e; } });
    if(on[2]) s+=((x-MU[0])**2+(y-MU[1])**2)/(S0*S0); return s; };
  const draw=()=>{ const any=on.some(v=>v), uniq=on[2]||(on[0]&&on[1]), ink=cssv("--ink")||"#444", X=x=>ox+x*sc, Y=y=>oy-y*sc; let best=[0,0,Infinity];
    ctx.globalAlpha=1; ctx.fillStyle=cssv("--surface")||"#fff"; ctx.fillRect(0,0,W,H); ctx.fillStyle=cssv("--c5")||"#36c";
    if(any) for(let i=0;i<W/C;i++) for(let j=0;j<H/C;j++){ const x=(i*C+C/2-ox)/sc, y=(oy-j*C-C/2)/sc, v=J(x,y); if(v<best[2]) best=[x,y,v]; ctx.globalAlpha=Math.exp(-v/2); ctx.fillRect(i*C,j*C,C,C); }
    ctx.globalAlpha=1; ctx.strokeStyle=ink; ctx.fillStyle=ink; ctx.lineWidth=1.5; ctx.font="13px sans-serif"; ctx.textAlign="center";
    P.forEach((p,k)=>{ if(on[k]){ ctx.setLineDash([6,5]); ctx.beginPath(); ctx.moveTo(X(p[0]),Y(p[1])); ctx.lineTo(X(p[0]+7*Math.cos(Z[k])),Y(p[1]+7*Math.sin(Z[k]))); ctx.stroke(); ctx.setLineDash([]); }
      ctx.beginPath(); ctx.arc(X(p[0]),Y(p[1]),7,0,7); ctx.fill(); ctx.fillText("p"+SUB[k],X(p[0]),Y(p[1])+26); });
    ctx.beginPath(); ctx.moveTo(X(T[0])-6,Y(T[1])-6); ctx.lineTo(X(T[0])+6,Y(T[1])+6); ctx.moveTo(X(T[0])-6,Y(T[1])+6); ctx.lineTo(X(T[0])+6,Y(T[1])-6); ctx.stroke(); ctx.fillText("true ℓ",X(T[0]),Y(T[1])+30);
    if(on[2]){ ctx.beginPath(); ctx.arc(X(MU[0]),Y(MU[1]),4,0,7); ctx.stroke(); ctx.fillText("prior mean",X(MU[0])+48,Y(MU[1])+5); }
    if(uniq){ ctx.strokeStyle=cssv("--c3")||"#c36"; ctx.lineWidth=2.5; ctx.beginPath(); ctx.arc(X(best[0]),Y(best[1]),9,0,7); ctx.stroke(); }
    st.innerHTML=""; st.append(stat("Factors multiplied",String(on.filter(v=>v).length),"switch them on and off"),
      stat("MAP estimate of ℓ",uniq?"("+best[0].toFixed(2)+", "+best[1].toFixed(2)+")":any?"not unique":"—",uniq?"pink ring; cost J = "+best[2].toFixed(2):any?"a whole ray fits equally well":"no information"),
      stat("Distance to the true ℓ",uniq?Math.hypot(best[0]-T[0],best[1]-T[1]).toFixed(2):"—","true ℓ = (2.00, 1.60)")); };
  const tg=(label,k)=>{ const b=button(label,()=>{ on[k]=!on[k]; b.classList.toggle("on",on[k]); draw(); }); b.classList.toggle("on",on[k]); return b; };
  el.append(h("div",{class:"vt"},"Multiplying factors on one landmark"),h("p",{class:"vd"},"Two poses are known. Each measured the bearing to the same landmark. Darker means a larger product of the factors that are on, evaluated on a grid of landmark positions."),
    h("div",{class:"ctl"},range("Bearing noise σ",1,15,1,sig,v=>{ sig=v; draw(); },v=>v+"°").el),
    h("div",{class:"ctl"},tg("Bearing from p₁",0),tg("Bearing from p₂",1),tg("Gaussian prior on ℓ",2)),cv,st);
  document.addEventListener("themechange",draw); draw();
};

// Day 3: a three-pose problem on a line, solved through the normal equations with Cholesky and again with Householder QR.
VIZ.w02_lls=el=>{
  const SP=0.1, W=600, H=150, X=x=>60+x*180; let so=0.2, sc=0.1, z=2.3;
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Number line with the three robot positions: from odometry alone, and from the least-squares solution that also uses the measurement of x3 minus x1."});
  const box=h("div",{class:"panel"}), st=h("div",{class:"stats"});
  const arr=(A,sep)=>"\\left[\\begin{array}{"+(sep?"rrr|r":"r".repeat(A[0].length))+"}"+A.map(r=>r.map(v=>fx(v)).join(" & ")).join(" \\\\ ")+"\\end{array}\\right]", colv=v=>arr(v.map(x=>[x]));
  const draw=()=>{ const A=[[1/SP,0,0],[-1/so,1/so,0],[0,-1/so,1/so],[-1/sc,0,1/sc]], b=[0,1/so,1/so,z/sc], {L:Lam,g}=normal(A,b), Lc=M.chol(Lam); if(!Lc) return;
    const R=M.T(Lc), y=[], d=[0,0,0];
    for(let i=0;i<3;i++){ let s=g[i]; for(let k=0;k<i;k++) s-=Lc[i][k]*y[k]; y[i]=s/Lc[i][i]; }                 // forward substitution: Rᵀ y = Aᵀ b
    for(let i=2;i>=0;i--){ let s=y[i]; for(let k=i+1;k<3;k++) s-=R[i][k]*d[k]; d[i]=s/R[i][i]; }                 // back substitution: R δ = y
    const Q=A.map((r,i)=>[...r,b[i]]);                                                                           // Householder QR of [A | b]
    for(let j=0;j<3;j++){ const v=Q.map((r,i)=>i<j?0:r[j]); v[j]+=(v[j]<0?-1:1)*Math.sqrt(sq(v)); const vv=sq(v); if(vv<1e-18) continue;
      for(let c=j;c<4;c++){ const t=2*Q.reduce((s,r,i)=>s+v[i]*r[c],0)/vv; Q.forEach((r,i)=>{ r[c]-=t*v[i]; }); } }
    svg.innerHTML=""; svg.append(sv("line",{class:"axis",x1:24,x2:576,y1:80,y2:80}));
    for(let t=0;t<=5;t++) svg.append(sv("line",{class:"axis",x1:X(t/2),x2:X(t/2),y1:80,y2:86}),txt(X(t/2),100,(t/2).toFixed(1),{"text-anchor":"middle",class:"mut","font-size":11}));
    [0,1,2].forEach(k=>svg.append(sv("circle",{cx:X(k),cy:80,r:9,fill:"none",stroke:"var(--muted)","stroke-width":1.5})));
    svg.append(sv("path",{d:"M"+X(z)+" 106 l-7 12 h14 z",fill:"var(--c2)"}),txt(Math.min(X(z),500),136,"measured x₃ − x₁ = "+z.toFixed(2),{"text-anchor":"middle","font-size":12}));
    d.forEach((v,k)=>svg.append(sv("circle",{cx:X(v),cy:80,r:5,fill:"var(--c1)"}),txt(X(v),60,"x"+SUB[k],{"text-anchor":"middle"})));
    box.innerHTML=""; [["[\\,\\mathbf{A}\\mid\\mathbf{b}\\,]="+arr(A.map((r,i)=>[...r,b[i]]),1),"whitened Jacobian and right-hand side, one row per factor"],
      ["\\boldsymbol{\\Lambda}=\\mathbf{A}\\T\\mathbf{A}="+arr(Lam)+",\\quad \\mathbf{A}\\T\\mathbf{b}="+colv(g),"normal equations (1.25)"],
      ["\\mathbf{R}="+arr(R)+",\\quad \\mathbf{y}="+colv(y)+",\\quad \\boldsymbol{\\delta}^*="+colv(d),"Cholesky factor (1.26), forward substitution (1.27), back substitution (1.28)"],
      ["\\mathbf{Q}\\T[\\,\\mathbf{A}\\mid\\mathbf{b}\\,]=\\begin{bmatrix}\\mathbf{R} & \\mathbf{d} \\\\ \\mathbf{0} & \\mathbf{e}\\end{bmatrix}="+arr(Q,1),"Householder QR (1.29): the same R up to row signs, no AᵀA formed"]]
      .forEach(([s,c])=>box.append(h("div",{style:"overflow-x:auto;padding:4px 0"},tex(s,true)),h("div",{class:"small",style:"color:var(--muted)"},c)));
    st.innerHTML=""; st.append(stat("Least-squares positions",d.map(v=>fx(v,3)).join(", "),"x₁, x₂, x₃"),stat("Odometry alone","0, 1, 2","hollow circles"),
      stat("Leftover cost",fx(sq(M.mv(A,d).map((v,i)=>v-b[i])),3),"‖Aδ* − b‖² = e² = "+fx(Q[3][3]*Q[3][3],3))); };
  el.append(h("div",{class:"vt"},"Four factors, three unknowns, one linear solve"),h("p",{class:"vd"},"A robot on a line: a prior x₁ = 0 (σ = 0.1), two odometry steps of 1.0, and one measurement of x₃ − x₁. Everything is linear, so with the linearization point at zero the update δ* is the answer itself."),
    h("div",{class:"ctl"},range("Odometry σ",0.05,0.5,0.05,so,v=>{ so=v; draw(); },v=>v.toFixed(2)).el,range("σ of the x₃ − x₁ measurement",0.05,0.5,0.05,sc,v=>{ sc=v; draw(); },v=>v.toFixed(2)).el,range("Measured x₃ − x₁",1.5,2.5,0.05,z,v=>{ z=v; draw(); },v=>v.toFixed(2)).el),
    svg,legend([["var(--muted)","odometry alone"],["var(--c1)","least-squares solution"],["var(--c2)","the x₃ − x₁ measurement"]]),box,st);
  draw();
};

// Days 4 and 5: one unknown. A landmark sits on the wall x = 1 at unknown height y; the robot at the origin measured bearing 0.
// J(y) = ((0 - atan y)/σ)². Gauss-Newton jumps to the minimum of the local parabola; Levenberg-Marquardt damps the jump.
const nls1=(el,lm)=>{
  const S=0.1, W=600, H=300, P={l:46,r:14,t:14,b:38}, YM=6, JM=250, Jf=y=>(Math.atan(y)/S)**2;
  let y0=2, k0=-1, y, lam, it, msg, path;
  const X=v=>P.l+(v+YM)/(2*YM)*(W-P.l-P.r), Y=v=>H-P.b-v/JM*(H-P.t-P.b);
  const curve=(f,a)=>{ let d="", pen=false; for(let i=0;i<=240;i++){ const v=-YM+i*YM/120, q=f(v); if(q<=JM){ d+=(pen?"L":"M")+X(v).toFixed(1)+" "+Y(q).toFixed(1)+" "; pen=true; } else pen=false; } return sv("path",Object.assign({d:d||"M0 0",fill:"none","stroke-width":2},a)); };
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"The cost J as a function of the landmark height y, the current estimate, and the local quadratic model that the next step minimizes."});
  const out=h("div",{class:"out"}), st=h("div",{class:"stats"});
  const lin=()=>({A:1/((1+y*y)*S), b:-Math.atan(y)/S});
  const draw=()=>{ svg.innerHTML="";
    for(let t=0;t<=5;t++) svg.append(sv("line",{class:"grid",x1:P.l,x2:W-P.r,y1:Y(t*50),y2:Y(t*50)}),txt(P.l-6,Y(t*50)+4,String(t*50),{"text-anchor":"end",class:"mut","font-size":11}));
    for(let t=-6;t<=6;t+=2) svg.append(txt(X(t),H-P.b+16,String(t),{"text-anchor":"middle",class:"mut","font-size":11}));
    svg.append(sv("line",{class:"axis",x1:P.l,x2:W-P.r,y1:Y(0),y2:Y(0)}),txt((P.l+W-P.r)/2,H-5,"landmark height y",{"text-anchor":"middle",class:"mut","font-size":11}),curve(Jf,{stroke:"var(--ink)"}));
    const {A,b}=lin(), dgn=b/A, d=lm?A*b/(A*A+lam):dgn;
    if(Math.abs(y)<=YM){ svg.append(curve(v=>(A*(v-y)-b)**2,{stroke:"var(--c2)","stroke-dasharray":lm?"5 4":"none"}));
      if(lm) svg.append(curve(v=>(A*(v-y)-b)**2+lam*(v-y)**2,{stroke:"var(--c4)"}));
      if(Math.abs(y+d)<=YM) svg.append(sv("line",{x1:X(y+d),x2:X(y+d),y1:Y(0),y2:Y(Math.min(JM,Jf(y+d))),stroke:"var(--c3)","stroke-width":1.5,"stroke-dasharray":"3 4"}),sv("circle",{cx:X(y+d),cy:Y(Math.min(JM,Jf(y+d))),r:5,fill:"var(--surface)",stroke:"var(--c3)","stroke-width":2}));
      svg.append(sv("circle",{cx:X(y),cy:Y(Jf(y)),r:6,fill:"var(--c3)",stroke:"var(--surface)","stroke-width":2})); }
    else svg.append(txt(W/2,40,"The estimate has left the chart: y = "+y.toPrecision(3),{"text-anchor":"middle",fill:"var(--c3)"}));
    out.textContent="y: "+path.slice(-7).map(v=>Math.abs(v)<1e4?fx(v,2):v.toExponential(1)).join(" → ");
    st.innerHTML=""; st.append(stat("Iteration",String(it),msg),stat("Estimate y",Math.abs(y)<1e4?fx(y,3):y.toExponential(1),"minimum is at y = 0"),stat("Cost J(y)",fx(Jf(y),2),"whitened with σ = 0.1 rad"),
      lm?stat("Damping λ",String(+lam.toPrecision(2)),"next step "+fx(d,2)+" (GN alone: "+fx(dgn,2)+")"):stat("Next GN step",Math.abs(dgn)<1e4?fx(dgn,2):dgn.toExponential(1),"δ = b / A")); };
  const reset=()=>{ y=y0; lam=10**k0; it=0; msg="start"; path=[y]; draw(); };
  const step=()=>{ if(!(Math.abs(y)<1e6)){ msg="diverged"; draw(); return; } const {A,b}=lin(), d=lm?A*b/(A*A+lam):b/A; it++;
    if(Math.abs(d)<1e-9) msg="converged";
    else if(!lm){ y+=d; path.push(y); msg="step taken"; }
    else if(Jf(y+d)<Jf(y)){ y+=d; path.push(y); lam=Math.max(lam/10,1e-9); msg="accepted, λ ÷ 10"; }
    else { lam=Math.min(lam*10,1e9); msg="rejected, λ × 10"; }
    draw(); };
  el.append(h("div",{class:"vt"},lm?"Levenberg-Marquardt on one unknown":"Gauss-Newton on one unknown"),
    h("p",{class:"vd"},"A landmark is on the wall x = 1 at unknown height y. The robot at the origin measured bearing 0, so J(y) = (atan y / 0.1)². The filled dot is the estimate, the hollow dot is where the next step lands."),
    h("div",{class:"ctl"},range("Starting estimate",-4,4,0.1,y0,v=>{ y0=v; reset(); },v=>v.toFixed(1)).el,lm?range("Starting λ",-3,3,1,k0,v=>{ k0=v; reset(); },v=>String(10**v)).el:null),
    h("div",{class:"ctl"},button("Step",step,"primary"),button("Reset",reset)),svg,
    legend(lm?[["var(--ink)","true cost J(y)"],["var(--c2)","Gauss-Newton model"],["var(--c4)","damped model, + λδ²"]]:[["var(--ink)","true cost J(y)"],["var(--c2)","local quadratic model"]]),out,st);
  reset();
};
VIZ.w02_gn1d=el=>nls1(el,false);
VIZ.w02_lm1d=el=>nls1(el,true);

// Days 4 and 5 (MAIN): a 12-step loop of poses with noisy odometry and one loop closure, optimized by the chosen methods side by side.
const pgo=(el,ms,init)=>{
  const N=13, SXY=0.1, SP=0.05, W=600, H=320, sc=52, ox=274, oy=272;
  const NAME={sd:"Steepest descent",gn:"Gauss-Newton",lm:"Levenberg-Marquardt"}, CL={sd:"var(--c2)",gn:"var(--c5)",lm:"var(--c1)"};
  let sdeg=5, cf=0.5, seed=1, show=ms[ms.length-1], truth, edges, sth, lmax, run, it;
  const comp=(p,d)=>[p[0]+Math.cos(p[2])*d[0]-Math.sin(p[2])*d[1],p[1]+Math.sin(p[2])*d[0]+Math.cos(p[2])*d[1],p[2]+d[2]];
  const lin=x=>{ const p=i=>x.slice(3*i,3*i+3), fs=[{at:[0],H:[I3],r:[-x[0],-x[1],wrap(-x[2])],s:[SP,SP,SP]}];        // prior: p1 = (0, 0, 0)
    edges.forEach(e=>{ const q=rel(p(e.i),p(e.j)); fs.push({at:[3*e.i,3*e.j],H:[q.Hi,q.Hj],r:[e.z[0]-q.h[0],e.z[1]-q.h[1],wrap(e.z[2]-q.h[2])],s:[SXY,SXY,sth]}); });
    const s=stack(3*N,fs); s.J=sq(s.b); return s; };
  const setup=()=>{ const r=rng(seed); sth=sdeg*Math.PI/180; truth=[[0,0,0]];
    for(let k=1;k<N;k++) truth.push(comp(truth[k-1],[1,0,Math.PI/6]));
    const noisy=(i,j)=>{ const t=rel(truth[i],truth[j]).h; return {i,j,z:[t[0]+r.n()*SXY,t[1]+r.n()*SXY,t[2]+r.n()*sth]}; };
    edges=[]; for(let k=0;k+1<N;k++) edges.push(noisy(k,k+1)); edges.push(noisy(N-1,0));                              // 12 odometry steps, then the loop closure
    let x0=[[0,0,0]]; for(let k=0;k+1<N;k++) x0.push(init==="odo"?comp(x0[k],edges[k].z):init==="zero"?[0,0,0]:[r.n()*1.5,r.n()*1.5+1.8,r.n()*2]);
    x0=x0.flat(); const l0=lin(x0), ev=M.eigSym(normal(l0.A,l0.b).L).val; lmax=ev[ev.length-1];
    run={}; it=0; ms.forEach(m=>{ run[m]={x:x0.slice(),J:[l0.J],lam:1e-3,note:"start"}; }); draw(); };
  const adv=m=>{ const s=run[m], Jc=s.J[s.J.length-1]; if(!(Jc<1e12)){ s.J.push(Jc); s.note="diverged"; return; }
    const {A,b}=lin(s.x), {L,g}=normal(A,b);
    const d=m==="sd"?g.map(v=>2*cf/lmax*v):M.solve(m==="gn"?L:L.map((r,i)=>r.map((v,j)=>i===j?v*(1+s.lam):v)),g);       // (1.35), (1.36), (1.38)
    if(!d){ s.J.push(Jc); s.note="singular system"; return; }
    const xn=s.x.map((v,i)=>v+d[i]), Jn=lin(xn).J;
    if(m!=="lm"){ s.x=xn; s.J.push(isFinite(Jn)?Jn:Infinity); s.note=Jn>Jc?"cost went up":"cost went down"; }
    else if(Jn<Jc){ s.x=xn; s.J.push(Jn); s.lam=Math.max(s.lam/10,1e-9); s.note="accepted, λ ÷ 10"; }
    else { s.J.push(Jc); if(Math.max(...d.map(Math.abs))<1e-7) s.note="converged"; else { s.lam=Math.min(s.lam*10,1e9); s.note="rejected, λ × 10"; } } };
  const go=k=>{ for(let i=0;i<k&&it<300;i++){ ms.forEach(adv); it++; } draw(); };
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Top view of the pose graph: the true loop, the current estimate of the selected method with a heading tick at each pose, and the loop-closure constraint between the last pose and the first."});
  const chart=h("div"), st=h("div",{class:"stats"}), lg=h("div");
  const cl=v=>isFinite(v)?Math.max(-9999,Math.min(9999,v)):0, X=p=>cl(ox+p[0]*sc).toFixed(1), Y=p=>cl(oy-p[1]*sc).toFixed(1);
  const poly=(Q,a)=>sv("path",Object.assign({d:Q.map((p,i)=>(i?"L":"M")+X(p)+" "+Y(p)).join(" "),fill:"none","stroke-width":2,"stroke-linejoin":"round"},a));
  const fj=J=>J<1e6?(J<100?J.toFixed(2):J.toFixed(0)):J.toExponential(1), lgj=J=>Math.log10(Math.min(Math.max(J,1e-3),1e6));
  const draw=()=>{ const x=run[show].x, Q=Array.from({length:N},(_,i)=>x.slice(3*i,3*i+3)), c=CL[show];
    svg.innerHTML=""; svg.append(poly(truth,{stroke:"var(--line2)","stroke-dasharray":"6 5"}),poly(Q,{stroke:c}),
      sv("line",{x1:X(Q[N-1]),y1:Y(Q[N-1]),x2:X(Q[0]),y2:Y(Q[0]),stroke:"var(--c3)","stroke-width":2,"stroke-dasharray":"3 4"}));
    Q.forEach(p=>svg.append(sv("line",{x1:X(p),y1:Y(p),x2:X([p[0]+0.22*Math.cos(p[2]),p[1]+0.22*Math.sin(p[2])]),y2:Y([p[0]+0.22*Math.cos(p[2]),p[1]+0.22*Math.sin(p[2])]),stroke:"var(--ink)","stroke-width":1.5}),sv("circle",{cx:X(p),cy:Y(p),r:3.5,fill:c})));
    svg.append(sv("circle",{cx:X(Q[0]),cy:Y(Q[0]),r:6,fill:"var(--ink)"}),txt(+X(Q[0]),+Y(Q[0])+22,"start",{"text-anchor":"middle",class:"mut"}));
    const all=ms.flatMap(m=>run[m].J.map(lgj)), lo=Math.floor(Math.min(...all)), hi=Math.max(lo+1,Math.ceil(Math.max(...all))), xm=Math.max(10,it), ts=Math.ceil(xm/40)*5, yt=[], xt=[];
    for(let p=lo;p<=hi;p++) yt.push([p,p<4?String(10**p):"10"+"⁴⁵⁶"[p-4]]);
    for(let t=0;t<=xm;t+=ts) xt.push([t,String(t)]);
    chart.innerHTML=""; chart.append(lineChart({w:W,h:230,aria:"Cost J after each iteration, on a logarithmic scale, one curve per method",x:{min:0,max:xm,ticks:xt,label:"iteration"},y:{min:lo,max:hi,ticks:yt,label:"cost J (log scale)"},
      series:ms.map(m=>({name:NAME[m],color:CL[m],dots:it<=40,points:run[m].J.map((J,i)=>({x:i,y:lgj(J),label:NAME[m]+", iteration "+i,fmt:()=>fj(J)}))}))}));
    lg.innerHTML=""; lg.append(legend([["var(--line2)","true loop"],[c,"estimate: "+NAME[show]],["var(--c3)","loop-closure constraint"]]));
    st.innerHTML=""; st.append(stat("Iterations",String(it),"start cost "+fj(run[ms[0]].J[0])));
    ms.forEach(m=>{ const s=run[m]; st.append(stat(NAME[m],"J = "+fj(s.J[s.J.length-1]),m==="sd"?"α = "+(cf/lmax).toExponential(1)+", "+s.note:m==="lm"?"λ = "+(+s.lam.toPrecision(1))+", "+s.note:s.note)); }); };
  const sel=select("Show the estimate of",ms.map(m=>[m,NAME[m]]),v=>{ show=v; draw(); }); sel.input.value=show;
  const ini=select("Initial estimate",[["odo","Chained odometry"],["rand","Scrambled poses"],["zero","All poses at the start"]],v=>{ init=v; setup(); }); ini.input.value=init;
  el.append(h("div",{class:"vt"},"Pose-graph optimizer"),h("p",{class:"vd"},"Thirteen poses, 39 unknowns. Factors: a prior on the first pose, 12 odometry steps (1 unit forward, 30° left, position σ = 0.1), and one loop closure saying the last pose is back at the first. All methods start from the same estimate."),
    h("div",{class:"ctl"},range("Heading noise σ per step",1,10,0.5,sdeg,v=>{ sdeg=v; setup(); },v=>v.toFixed(1)+"°").el,ms.includes("sd")?range("SD step size α × λmax",0.1,1.5,0.05,cf,v=>{ cf=v; setup(); },v=>v.toFixed(2)).el:null),
    h("div",{class:"ctl"},ini.el,sel.el),
    h("div",{class:"ctl"},button("Step",()=>go(1),"primary"),button("Run 20",()=>go(20)),button("Reset",()=>setup()),button("New noise",()=>{ seed++; setup(); })),svg,lg,chart,st);
  setup();
};
VIZ.w02_pgo2=el=>pgo(el,["sd","gn"],"odo");
VIZ.w02_pgo=el=>pgo(el,["sd","gn","lm"],"rand");
