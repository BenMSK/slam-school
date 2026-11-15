// Week 12 widgets. Days 1, 3 and 4 share one toy, a "flatland" camera. The world is the plane (x, z). A camera is
// [cx, cz, th]: th = 0 looks along +z and a positive th turns towards +x. A point is [px, pz]. The image is a line:
// a point lands on it at u = F·xc/zc pixels, where (xc, zc) is the point expressed in the camera frame.
const F=300, FOV=50*Math.PI/180, AW=1e8;                       // focal length (px), half field of view, weight of an anchor prior
const camPt=(c,p)=>{ const dx=p[0]-c[0], dz=p[1]-c[1], s=Math.sin(c[2]), co=Math.cos(c[2]); return [co*dx-s*dz, s*dx+co*dz]; };
// projection u and its derivatives: jc = du/d[cx, cz, th], jp = du/d[px, pz]
const projJ=(c,p)=>{ const [x,z]=camPt(c,p), s=Math.sin(c[2]), co=Math.cos(c[2]), a=F/z, b=-F*x/(z*z), jp=[a*co+b*s, -a*s+b*co];
  return {u:F*x/z, z, jc:[-jp[0], -jp[1], -F*(1+x*x/(z*z))], jp}; };
// the fixed bundle-adjustment scene: 6 cameras in a row, 14 points in front of them, a camera sees what is inside its field of view
const BA=(()=>{ const cams=[], pts=[], vis=[];
  for(let i=0;i<6;i++) cams.push([i-2.5, 0.2*Math.sin(1.7*i), 0.15*Math.cos(2.3*i)]);
  for(let j=0;j<14;j++) pts.push([-2.8+5.6*j/13, 1.3+0.7*((j*5)%14)/13]);
  cams.forEach((c,i)=>pts.forEach((p,j)=>{ const [x,z]=camPt(c,p); if(z>0.3&&Math.abs(Math.atan2(x,z))<FOV) vis.push([i,j]); }));
  return {cams, pts, vis}; })();
const baObs=(sig,r)=>BA.vis.map(([i,j])=>({i, j, z:projJ(BA.cams[i],BA.pts[j]).u+(sig?sig*r.n():0)}));
const baRms=(cams,pts,obs)=>Math.sqrt(obs.reduce((s,o)=>s+(o.z-projJ(cams[o.i],pts[o.j]).u)**2,0)/obs.length);
// Normal equations H d = b of the BA problem at (cams, pts). Unknown order: 3 numbers per camera, then 2 per point.
// anc lists camera-vector entries held near their true values by a strong prior (this removes the gauge freedom).
const baSystem=(cams,pts,obs,sig,anc)=>{ const nc=3*cams.length, n=nc+2*pts.length, H=M.zeros(n,n), b=new Array(n).fill(0); let cost=0;
  for(const o of obs){ const q=projJ(cams[o.i],pts[o.j]), r=(o.z-q.u)/sig, ix=[3*o.i,3*o.i+1,3*o.i+2,nc+2*o.j,nc+2*o.j+1], J=[...q.jc,...q.jp].map(v=>v/sig);
    cost+=r*r/2; for(let a=0;a<5;a++){ b[ix[a]]+=J[a]*r; for(let c=0;c<5;c++) H[ix[a]][ix[c]]+=J[a]*J[c]; } }
  for(const k of anc){ const r=BA.cams[k/3|0][k%3]-cams[k/3|0][k%3]; H[k][k]+=AW; b[k]+=AW*r; cost+=AW*r*r/2; }
  return {H, b, cost, nc}; };
const sub=(A,r0,r1,c0,c1)=>A.slice(r0,r1).map(r=>r.slice(c0,c1));
// Schur complement trick, equation (7.7): eliminate the points (H_pp is inverted one 2×2 block at a time), solve the
// reduced camera system, back-substitute the points. lam is Levenberg-Marquardt damping on the diagonal. null if singular.
const schur=(S,lam)=>{ const nc=S.nc, n=S.b.length, H=S.H.map((r,i)=>r.map((v,j)=>i===j?v*(1+lam):v)), b=S.b;
  const Hcc=sub(H,0,nc,0,nc), Hcp=sub(H,0,nc,nc,n), Di=M.zeros(n-nc,n-nc);
  for(let k=nc;k<n;k+=2){ const a=H[k][k], o=H[k][k+1], d=H[k+1][k+1], det=a*d-o*o, q=k-nc; if(!(Math.abs(det)>1e-9)) return null;
    Di[q][q]=d/det; Di[q+1][q+1]=a/det; Di[q][q+1]=Di[q+1][q]=-o/det; }
  const HD=M.mul(Hcp,Di), Hred=M.sub(Hcc,M.mul(HD,M.T(Hcp))), t=M.mv(HD,b.slice(nc)), dc=M.solve(Hred,b.slice(0,nc).map((v,i)=>v-t[i])); if(!dc) return null;
  const u=M.mv(M.T(Hcp),dc), dp=M.mv(Di,b.slice(nc).map((v,i)=>v-u[i])); return {H, Hcc, Hcp, Di, Hred, dc, dp}; };
const moved=(V,d,w)=>V.map((v,i)=>v.map((x,k)=>x+d[w*i+k]));
// drawing helpers for the flatland scene (world z up on screen)
const scene=(sc,ox,oy)=>{ const X=p=>(ox+sc*p[0]).toFixed(1), Y=p=>(oy-sc*p[1]).toFixed(1), tip=(c,k,L)=>[c[0]+L*Math.sin(c[2]+k*0.45), c[1]+L*Math.cos(c[2]+k*0.45)];
  return {X, Y,
    cam:(c,a,L=0.32)=>sv("path",Object.assign({d:"M"+X(c)+" "+Y(c)+" L"+X(tip(c,-1,L))+" "+Y(tip(c,-1,L))+" L"+X(tip(c,1,L))+" "+Y(tip(c,1,L))+" Z","stroke-width":2,"stroke-linejoin":"round"},a)),
    ray:(c,p,a)=>sv("line",Object.assign({x1:X(c),y1:Y(c),x2:X(p),y2:Y(p),"stroke-width":1},a)),
    dot:(p,r,a)=>sv("circle",Object.assign({cx:X(p),cy:Y(p),r},a)) }; };
const legend=items=>h("div",{class:"legend"},...items.map(([c,t])=>h("span",{},h("i",{style:"background:"+c}),t)));
const toggle=(label,on)=>{ const b=button(label+": off",()=>{ const v=!b.classList.contains("on"); b.classList.toggle("on",v); b.textContent=label+(v?": on":": off"); on(v); }); return b; };

// Day 1: pose-only bundle adjustment (tracking). The map is known; Gauss-Newton moves only the camera, 3 unknowns.
VIZ.w12_track=el=>{
  const PTS=[[-2.4,4.6],[-1.8,3.2],[-1.3,5.4],[-0.8,2.8],[-0.3,4.2],[0.2,5.8],[0.6,3.4],[1.1,4.9],[1.6,2.9],[2.1,5.3],[2.6,3.9],[3.1,5.0]], TRUE=[0.5,0.3,0.12], BAD=[2,6];
  let seed=1, sig=1, out=false, hub=false, c, it, z, w;
  const G=scene(50,300,318), svg=sv("svg",{viewBox:"0 0 600 412",role:"img","aria-label":"Top view of a flatland camera and twelve known map points, with the one-dimensional image below showing observed and predicted keypoint positions"});
  const st=h("div",{class:"stats"});
  const mk=()=>{ const r=rng(seed); z=PTS.map(p=>projJ(TRUE,p).u+sig*r.n()); if(out){ z[BAD[0]]+=60; z[BAD[1]]-=45; } c=[0,0,0]; it=0; };
  const weights=()=>{ const del=1.345*Math.max(sig,0.5); w=PTS.map((p,j)=>{ const r=Math.abs(z[j]-projJ(c,p).u); return hub&&r>del?del/r:1; }); };
  const step=()=>{ weights(); const H=M.zeros(3,3), b=[0,0,0];
    PTS.forEach((p,j)=>{ const q=projJ(c,p); if(q.z<0.2) return; const r=z[j]-q.u; for(let a=0;a<3;a++){ b[a]+=w[j]*q.jc[a]*r; for(let k=0;k<3;k++) H[a][k]+=w[j]*q.jc[a]*q.jc[k]; } });
    const d=M.solve(H,b); if(d&&d.every(isFinite)){ c=c.map((v,k)=>v+d[k]); it++; } };
  const draw=()=>{ weights(); svg.innerHTML=""; const U=u=>(300+u*0.8).toFixed(1), pred=PTS.map(p=>projJ(c,p).u);
    PTS.forEach((p,j)=>svg.append(G.ray(c,p,{stroke:out&&BAD.includes(j)?"var(--c3)":"var(--line2)",opacity:(0.25+0.75*w[j]).toFixed(2)})));
    svg.append(G.cam(TRUE,{fill:"none",stroke:"var(--muted)","stroke-dasharray":"4 3"},0.5),G.cam(c,{fill:"var(--c5-soft)",stroke:"var(--c5)"},0.5));
    PTS.forEach(p=>svg.append(G.dot(p,5,{fill:"var(--c2)",stroke:"var(--surface)","stroke-width":1.5})));
    svg.append(txt(12,18,"top view",{class:"mut","font-size":12}),txt(12,352,"the 1-D image (pixels)",{class:"mut","font-size":12}),sv("line",{x1:30,x2:570,y1:380,y2:380,stroke:"var(--line2)","stroke-width":1.5}));
    [-300,-150,0,150,300].forEach(u=>svg.append(sv("line",{x1:U(u),x2:U(u),y1:377,y2:383,stroke:"var(--muted)"}),txt(U(u),406,String(u),{"text-anchor":"middle",class:"mut","font-size":11})));
    PTS.forEach((p,j)=>{ const bad=out&&BAD.includes(j), pu=Math.max(-340,Math.min(340,pred[j]));
      svg.append(sv("line",{x1:U(z[j]),x2:U(pu),y1:370,y2:390,stroke:bad?"var(--c3)":"var(--muted)","stroke-width":1,opacity:(0.25+0.75*w[j]).toFixed(2)}),
        sv("line",{x1:U(z[j]),x2:U(z[j]),y1:358,y2:372,stroke:bad?"var(--c3)":"var(--ink)","stroke-width":3}),sv("line",{x1:U(pu),x2:U(pu),y1:388,y2:394,stroke:"var(--c5)","stroke-width":3})); });
    const rms=Math.sqrt(PTS.reduce((s,p,j)=>s+(z[j]-pred[j])**2,0)/PTS.length);
    st.innerHTML=""; st.append(stat("Iterations",String(it),"start = previous pose"),stat("RMS reprojection error",rms.toFixed(1)+" px","all 12 matches"),
      stat("Position error",(100*Math.hypot(c[0]-TRUE[0],c[1]-TRUE[1])).toFixed(1)+" cm"),stat("Heading error",(Math.abs(c[2]-TRUE[2])*180/Math.PI).toFixed(2)+"°")); };
  el.append(h("div",{class:"vt"},"Tracking one frame: pose-only BA"),h("p",{class:"vd"},"Twelve map points are known. Gauss-Newton moves only the camera (x, z, heading) until the predicted keypoints meet the observed ones."),
    h("div",{class:"ctl"},range("Pixel noise σ",0,3,0.5,sig,v=>{ sig=v; mk(); draw(); },v=>v+" px").el),
    h("div",{class:"ctl"},button("Gauss-Newton step",()=>{ step(); draw(); },"primary"),button("Reset pose",()=>{ c=[0,0,0]; it=0; draw(); }),button("New noise",()=>{ seed++; mk(); draw(); }),
      toggle("2 wrong matches",v=>{ out=v; mk(); draw(); }),toggle("Huber kernel",v=>{ hub=v; draw(); })),
    svg,legend([["var(--c5)","estimated camera, predicted keypoints"],["var(--muted)","true camera (unknown to the tracker)"],["var(--ink)","observed keypoints"],["var(--c2)","map points"]]),st);
  mk(); draw();
};

// Day 2: Lucas-Kanade alignment of a shifted 1-D intensity profile, with an image pyramid.
const lkSamp=(I,x)=>{ const m=I.length, f=Math.floor(x), t=x-f; return I[((f%m)+m)%m]*(1-t)+I[(((f+1)%m)+m)%m]*t; };      // linear interpolation, periodic
const lkDown=I=>{ const m=I.length, o=[]; for(let k=0;k<m;k+=2){ let v=0; [1,4,6,4,1].forEach((g,q)=>{ v+=g*I[(k+q-2+m)%m]; }); o.push(v/16); } return o; };   // blur, then keep every second sample
// texture: fine random stripes (2–6 px) plus coarse ones (24–60 px), lightly blurred; the second image is the first moved by d pixels
const lkBuild=(seed,d,N,LV)=>{ const r=rng(seed), seg=(lo,hi)=>{ const t=[]; while(t.length<N){ const len=lo+Math.floor(r.u()*(hi-lo+1)), v=r.u(); for(let q=0;q<len&&t.length<N;q++) t.push(v); } return t; };
  const fine=seg(2,6), coarse=seg(24,60), I0=fine.map((v,k)=>(v+coarse[k])/2), I=I0.map((_,k)=>(I0[(k+N-1)%N]+2*I0[k]+I0[(k+1)%N])/4);
  const P1=[I], P2=[I.map((_,k)=>lkSamp(I,k-d))]; for(let l=1;l<=LV;l++){ P1.push(lkDown(P1[l-1])); P2.push(lkDown(P2[l-1])); } return {P1, P2}; };
// one Gauss-Newton step for the shift s (in pixels of this level): residual r = I2(x+s) − I1(x), Jacobian = dI2/dx at x+s
const lkStep=(A,B,s)=>{ let g=0, q=0; for(let k=0;k<A.length;k++){ const r=lkSamp(B,k+s)-A[k], J=(lkSamp(B,k+s+1)-lkSamp(B,k+s-1))/2; g+=J*r; q+=J*J; } return q>1e-12?-g/q:0; };
const lkCost=(A,B,s)=>{ let e=0; for(let k=0;k<A.length;k++) e+=(lkSamp(B,k+s)-A[k])**2; return e/A.length; };
const lkSolve=(P,s0,top)=>{ let x=s0; for(let l=top;l>=0;l--){ let s=x/2**l; for(let i=0;i<30;i++){ const ds=lkStep(P.P1[l],P.P2[l],s); s+=ds; if(Math.abs(ds)<1e-3) break; } x=s*2**l; } return x; };
VIZ.w12_lk=el=>{
  const N=256, LV=3, R=64; let seed=1, d=20, pyr=false, P, s, lev, n, trail, done, basin;
  const svg=sv("svg",{viewBox:"0 0 600 350",role:"img","aria-label":"Top: the reference profile, the shifted profile and the shifted profile warped back by the current estimate. Bottom: the alignment cost as a function of the shift, with the current estimate and a strip showing which starting values converge to the true shift"});
  const st=h("div",{class:"stats"});
  const reset=()=>{ s=0; lev=pyr?LV:0; n=0; trail=[0]; done=false; };
  const mk=()=>{ P=lkBuild(seed,d,N,LV); basin=[]; for(let s0=-R;s0<=R;s0+=2) basin.push(Math.abs(lkSolve(P,s0,pyr?LV:0)-d)<0.5); reset(); };
  const step=()=>{ if(done) return; const sl=s/2**lev, ds=lkStep(P.P1[lev],P.P2[lev],sl); s=Math.max(-2*R,Math.min(2*R,(sl+ds)*2**lev)); n++; trail.push(s); if(Math.abs(ds)<0.02){ if(lev>0) lev--; else done=true; } };
  const draw=()=>{ svg.innerHTML=""; const A=P.P1[lev], B=P.P2[lev], m=A.length, sc=2**lev, PX=k=>(20+k*sc*560/N).toFixed(1), PY=v=>(128-v*104).toFixed(1);
    const line=(f,a)=>sv("path",Object.assign({d:A.map((_,k)=>(k?"L":"M")+PX(k)+" "+PY(f(k))).join(" "),fill:"none","stroke-width":2,"stroke-linejoin":"round"},a));
    svg.append(txt(20,14,"intensity along one image row, "+m+" pixels wide"+(lev?" (pyramid level "+lev+")":""),{class:"mut","font-size":12}),
      line(k=>B[k],{stroke:"var(--c3)","stroke-width":1.2,opacity:0.6}),line(k=>A[k],{stroke:"var(--c5)"}),line(k=>lkSamp(B,k+s/sc),{stroke:"var(--c1)","stroke-dasharray":"5 3"}));
    const SX=v=>(300+v*4.2).toFixed(1), cs=[]; for(let v=-R;v<=R;v++) cs.push(lkCost(A,B,v/sc)); const mx=Math.max(...cs,1e-9), SY=e=>(292-e/mx*100).toFixed(1);
    svg.append(txt(20,176,"cost E(s) at this level",{class:"mut","font-size":12}),sv("line",{x1:SX(-R),x2:SX(R),y1:292,y2:292,stroke:"var(--line2)"}),
      sv("line",{x1:SX(d),x2:SX(d),y1:184,y2:292,stroke:"var(--muted)","stroke-dasharray":"4 3"}),txt(+SX(d)+5,194,"true shift",{class:"mut","font-size":11}),
      sv("path",{d:cs.map((e,i)=>(i?"L":"M")+SX(i-R)+" "+SY(e)).join(" "),fill:"none",stroke:"var(--c4)","stroke-width":2}));
    basin.forEach((ok,i)=>svg.append(sv("rect",{x:(+SX(-R+2*i)-4.2).toFixed(1),y:298,width:8.4,height:9,fill:ok?"var(--c1)":"var(--c3-soft)"})));
    [-60,-30,0,30,60].forEach(v=>svg.append(txt(SX(v),322,String(v),{"text-anchor":"middle",class:"mut","font-size":11})));
    svg.append(txt(300,342,"shift s (pixels); the strip marks starting values that end at the true shift",{"text-anchor":"middle",class:"mut","font-size":11}));
    const cl=v=>Math.max(-R,Math.min(R,v));
    trail.forEach((v,i)=>svg.append(sv("circle",{cx:SX(cl(v)),cy:SY(lkCost(A,B,cl(v)/sc)),r:i===trail.length-1?6:3,fill:i===trail.length-1?"var(--acc)":"var(--muted)",stroke:"var(--surface)","stroke-width":1.5})));
    let lo=basin.length, hi=-1; const i0=Math.round((d+R)/2); if(basin[i0]){ lo=hi=i0; while(basin[lo-1]) lo--; while(basin[hi+1]) hi++; }
    st.innerHTML=""; st.append(stat("True shift",d+" px"),stat("Estimate",s.toFixed(2)+" px",n+" step"+(n===1?"":"s")+(done?", converged":"")),
      stat("Working at",lev?"level "+lev:"full resolution",m+" pixels"),stat("Basin of convergence",hi<0?"none":"about "+(2*(hi-lo)+2)+" px wide",pyr?"with the pyramid":"full resolution only")); };
  el.append(h("div",{class:"vt"},"Direct alignment in 1-D (Lucas-Kanade)"),h("p",{class:"vd"},"The pink profile is the blue one moved by the true shift. Gauss-Newton looks for the shift s that makes the warped (green, dashed) profile fall on the blue one. It starts at s = 0."),
    h("div",{class:"ctl"},range("True shift",-40,40,1,d,v=>{ d=v; mk(); draw(); },v=>v+" px").el),
    h("div",{class:"ctl"},button("Gauss-Newton step",()=>{ step(); draw(); },"primary"),button("Run to convergence",()=>{ for(let i=0;i<80&&!done;i++) step(); draw(); }),
      button("Reset",()=>{ reset(); draw(); }),toggle("Pyramid",v=>{ pyr=v; mk(); draw(); }),button("New texture",()=>{ seed++; mk(); draw(); })),
    svg,legend([["var(--c5)","reference I₁"],["var(--c3)","shifted I₂"],["var(--c1)","I₂ warped by the estimate"],["var(--c4)","cost"]]),st);
  mk(); draw();
};

// Day 3: bundle adjustment with the Schur complement, one step at a time. Cameras 1 and 2 are anchored.
VIZ.w12_ba=el=>{
  const ANC=[0,1,2,3,4,5]; let seed=1, sig=0.5, init=1, cams, pts, obs, S, R, phase, it, lam, msg, diff;
  const G=scene(86,300,205), svg=sv("svg",{viewBox:"0 0 600 248",role:"img","aria-label":"Top view of six flatland cameras and fourteen points: true positions, current estimates, and the observation rays"});
  const mat=sv("svg",{viewBox:"0 0 600 320",role:"img","aria-label":"Left: sparsity pattern of the full Hessian with camera block, point block and coupling blocks. Right: sparsity pattern of the reduced camera system after eliminating the points"});
  const st=h("div",{class:"stats"}), out=h("div",{class:"out"});
  const lin=()=>{ S=baSystem(cams,pts,obs,Math.max(sig,0.05),ANC); R=null; phase=0; };
  const reset=()=>{ const r=rng(seed*31+7); obs=baObs(sig,r);
    cams=BA.cams.map((c,i)=>i<2?[...c]:[c[0]+0.1*init*r.n(),c[1]+0.1*init*r.n(),c[2]+0.05*init*r.n()]); pts=BA.pts.map(p=>[p[0]+0.15*init*r.n(),p[1]+0.15*init*r.n()]);
    it=0; lam=1e-3; msg="Linearized at the initial guess."; lin(); };
  // one press: 0 → eliminate points, 1 → move cameras, 2 → move points and accept or reject the step
  const next=()=>{
    if(phase===0){ R=schur(S,lam); if(!R){ lam=Math.min(lam*10,1e6); msg="Singular system; damping raised."; return; }
      const dd=M.solve(R.H,S.b); diff=dd?Math.max(...dd.map((v,i)=>Math.abs(v-(i<S.nc?R.dc[i]:R.dp[i-S.nc])))):NaN; phase=1; msg="Points eliminated: the 46 × 46 system became 18 × 18."; }
    else if(phase===1){ R.old=[cams,pts]; cams=moved(cams,R.dc,3); phase=2; msg="Reduced camera system solved; cameras moved."; }
    else { const np=moved(pts,R.dp,2), c1=baSystem(cams,np,obs,Math.max(sig,0.05),ANC).cost, c0=S.cost;
      if(c1<c0*(1-1e-9)){ it++; pts=np; lam=Math.max(lam/5,1e-7); msg="Points back-substituted. Iteration "+it+" accepted: cost "+c0.toFixed(1)+" → "+c1.toFixed(1)+"."; }
      else if(Math.abs(c1-c0)<=1e-6*(1+c0)){ [cams,pts]=R.old; msg="Converged: another step no longer lowers the cost ("+c0.toFixed(1)+")."; }
      else { [cams,pts]=R.old; lam=Math.min(lam*10,1e6); msg="Step rejected (cost would rise from "+c0.toFixed(1)+" to "+(isFinite(c1)?c1.toExponential(1):"∞")+"); damping raised."; }
      lin(); } };
  const pattern=(A,x0,y0,cell,col)=>{ const g=sv("g"); A.forEach((r,i)=>r.forEach((v,j)=>{ if(Math.abs(v)>1e-9) g.append(sv("rect",{x:(x0+j*cell).toFixed(1),y:(y0+i*cell).toFixed(1),width:(cell-0.8).toFixed(1),height:(cell-0.8).toFixed(1),fill:col(i,j)})); })); return g; };
  const draw=()=>{ svg.innerHTML="";
    obs.forEach(o=>svg.append(G.ray(cams[o.i],pts[o.j],{stroke:"var(--line2)"})));
    BA.pts.forEach(p=>svg.append(G.dot(p,5,{fill:"none",stroke:"var(--muted)","stroke-width":1.5})));
    BA.cams.forEach(c=>svg.append(G.cam(c,{fill:"none",stroke:"var(--muted)","stroke-dasharray":"4 3"})));
    pts.forEach(p=>svg.append(G.dot(p,4,{fill:"var(--c2)"})));
    cams.forEach((c,i)=>svg.append(G.cam(c,{fill:i<2?"var(--c1-soft)":"var(--c5-soft)",stroke:i<2?"var(--c1)":"var(--c5)"}),txt(G.X(c),+G.Y(c)+17,String(i+1),{"text-anchor":"middle","font-size":12,class:"mut"})));
    mat.innerHTML=""; const n=S.b.length, nc=S.nc, c=6, x0=18, y0=28, e=x0+nc*c, f=y0+nc*c;
    mat.append(txt(x0,16,"H: "+n+" × "+n,{"font-size":12,class:"mut"}),pattern(S.H,x0,y0,c,(i,j)=>i<nc&&j<nc?"var(--c5)":i>=nc&&j>=nc?"var(--c2)":"var(--c4)"),
      sv("line",{x1:e-0.4,x2:e-0.4,y1:y0,y2:y0+n*c,stroke:"var(--muted)","stroke-dasharray":"3 3"}),sv("line",{x1:x0,x2:x0+n*c,y1:f-0.4,y2:f-0.4,stroke:"var(--muted)","stroke-dasharray":"3 3"}),
      sv("rect",{x:x0-1,y:y0-1,width:n*c+1.2,height:n*c+1.2,fill:"none",stroke:"var(--line2)"}));
    const rx=338, ry=64, rc=12;
    if(phase>0){ mat.append(txt(rx,16,"Reduced camera system: "+nc+" × "+nc,{"font-size":12,class:"mut"}),pattern(R.Hred,rx,ry,rc,()=>"var(--c5)"),sv("rect",{x:rx-1,y:ry-1,width:nc*rc+1.2,height:nc*rc+1.2,fill:"none",stroke:"var(--line2)"}));
      for(let i=0;i<nc/3;i++) mat.append(txt(rx+(3*i+1.5)*rc,ry-8,String(i+1),{"text-anchor":"middle","font-size":12,class:"mut"}),txt(rx-10,ry+(3*i+1.5)*rc+4,String(i+1),{"text-anchor":"middle","font-size":12,class:"mut"}));
      mat.append(txt(rx,ry-28,"camera",{"font-size":11,class:"mut"})); }
    else mat.append(txt(rx,120,"Press Next step to",{"font-size":13,class:"mut"}),txt(rx,138,"eliminate the points.",{"font-size":13,class:"mut"}));
    out.textContent="Step "+(phase+1)+" of 3 · "+msg;
    st.innerHTML=""; st.append(stat("Iterations",String(it),"damping λ = "+lam.toExponential(0)),stat("RMS reprojection error",baRms(cams,pts,obs).toFixed(2)+" px",obs.length+" observations"),
      stat("Worst point error",(100*Math.max(...pts.map((p,j)=>Math.hypot(p[0]-BA.pts[j][0],p[1]-BA.pts[j][1])))).toFixed(1)+" cm","against the true map"),
      stat("Schur vs. direct solve",phase>0&&isFinite(diff)?diff.toExponential(0):"—","largest difference in d")); };
  el.append(h("div",{class:"vt"},"Bundle adjustment, one Schur step at a time"),h("p",{class:"vd"},"6 cameras (3 unknowns each) observe 14 points (2 unknowns each) through 1-D images. Cameras 1 and 2 are anchored. Each Gauss-Newton iteration is solved with the Schur complement, equation (7.7)."),
    h("div",{class:"ctl"},range("Pixel noise σ",0,2,0.1,sig,v=>{ sig=v; reset(); draw(); },v=>v.toFixed(1)+" px").el,range("Initial error",0,1.5,0.1,init,v=>{ init=v; reset(); draw(); },v=>"× "+v.toFixed(1)).el),
    h("div",{class:"ctl"},button("Next step",()=>{ next(); draw(); },"primary"),button("Run 10 iterations",()=>{ for(let k=0;k<10;k++){ do next(); while(phase!==0); } draw(); }),
      button("Reset",()=>{ reset(); draw(); }),button("New noise",()=>{ seed++; reset(); draw(); })),
    svg,legend([["var(--c1)","anchored cameras"],["var(--c5)","estimated cameras"],["var(--c2)","estimated points"],["var(--muted)","truth (dashed, hollow)"]]),out,mat,
    legend([["var(--c5)","camera–camera H_cc"],["var(--c4)","camera–point H_cp"],["var(--c2)","point–point H_pp"]]),st);
  reset(); draw();
};

// Day 4: gauge freedom. Move the whole solution by a similarity transform and watch the error and the eigenvalues of H.
VIZ.w12_gauge=el=>{
  const ANCS={none:[],cam:[0,1,2],min:[0,1,2,3]}, obs=baObs(0), O=[0,1]; let mode="tx", t=0.6, anc="none";
  const G=scene(60,300,225), svg=sv("svg",{viewBox:"0 0 600 275",role:"img","aria-label":"Top view of the bundle adjustment scene before and after the chosen transformation"});
  const st=h("div",{class:"stats"}), out=h("div",{class:"out"});
  const xf=(v,i,isCam)=>{ const a=mode==="rot"?0.5*t:0, k=mode==="sc"?1+0.4*t:1, dx=v[0]-O[0], dz=v[1]-O[1];
    const p=[O[0]+k*(Math.cos(a)*dx-Math.sin(a)*dz)+(mode==="tx"||(mode==="one"&&isCam&&i===3)?t:0), O[1]+k*(Math.sin(a)*dx+Math.cos(a)*dz)+(mode==="tz"?0.5*t:0)];
    return isCam?[p[0],p[1],v[2]-a]:p; };
  const draw=()=>{ const cams=BA.cams.map((c,i)=>xf(c,i,true)), pts=BA.pts.map((p,j)=>xf(p,j,false)); svg.innerHTML="";
    BA.pts.forEach(p=>svg.append(G.dot(p,4,{fill:"none",stroke:"var(--line2)","stroke-width":1.5}))); BA.cams.forEach(c=>svg.append(G.cam(c,{fill:"none",stroke:"var(--line2)"})));
    obs.forEach(o=>svg.append(G.ray(cams[o.i],pts[o.j],{stroke:"var(--line2)"})));
    pts.forEach(p=>svg.append(G.dot(p,4,{fill:"var(--c2)"}))); cams.forEach((c,i)=>svg.append(G.cam(c,{fill:"var(--c5-soft)",stroke:"var(--c5)"}),txt(G.X(c),+G.Y(c)+17,String(i+1),{"text-anchor":"middle","font-size":12,class:"mut"})));
    const ev=M.eigSym(baSystem(cams,pts,obs,1,ANCS[anc]).H).val, top=ev[ev.length-1], zero=ev.filter(v=>Math.abs(v)<1e-9*top).length;
    out.textContent="Six smallest eigenvalues of H: "+ev.slice(0,6).map(v=>Math.abs(v)<1e-9*top?"0":v.toPrecision(2)).join(",  ");
    st.innerHTML=""; st.append(stat("RMS reprojection error",baRms(cams,pts,obs).toFixed(3)+" px","after the move"),stat("Zero eigenvalues of H",String(zero),"directions the images cannot see"),stat("Unknowns",String(ev.length),"18 camera + 28 point")); };
  el.append(h("div",{class:"vt"},"Gauge freedom: moves the images cannot see"),h("p",{class:"vd"},"The scene of Day 3 with perfect measurements. Transform the whole solution and read off the reprojection error, then count the zero eigenvalues of the Hessian."),
    h("div",{class:"ctl"},select("Move",[["tx","Shift everything along x"],["tz","Shift everything along z"],["rot","Rotate everything"],["sc","Scale everything"],["one","Shift only camera 4"]],v=>{ mode=v; draw(); }).el,
      range("Amount",-1,1,0.05,t,v=>{ t=v; draw(); },v=>v.toFixed(2)).el),
    h("div",{class:"ctl"},select("Anchor priors",[["none","None"],["cam","Camera 1 pose (3 numbers)"],["min","Camera 1 pose + x of camera 2 (4 numbers)"]],v=>{ anc=v; draw(); }).el),
    svg,legend([["var(--line2)","original solution"],["var(--c5)","moved cameras"],["var(--c2)","moved points"]]),out,st);
  draw();
};

// Day 4: the power series (7.9) for the inverse of the reduced camera system, on the Day 3 problem.
VIZ.w12_power=el=>{
  const S=baSystem(BA.cams,BA.pts,baObs(0),1,[0,1,2,3,4,5]), fro=A=>Math.sqrt(A.reduce((s,r)=>s+r.reduce((q,v)=>q+v*v,0),0)); let lg=-1;
  const host=h("div"), st=h("div",{class:"stats"});
  const draw=()=>{ const R=schur(S,10**lg), G=M.inv(R.Hcc), E=M.sub(R.Hcc,R.Hred), P=M.mul(G,E), exact=M.inv(R.Hred), ne=fro(exact), pts=[]; let T=G, sum=G;
    for(let m=0;m<=40;m++){ if(m){ T=M.mul(P,T); sum=M.add(sum,T); } pts.push({x:m,y:Math.max(-12,Math.log10(fro(M.sub(sum,exact))/ne+1e-300)),label:"m = "+m,fmt:v=>"error 10^"+v.toFixed(1)}); }
    const Li=M.inv(M.chol(R.Hcc)), rho=M.eigSym(M.mul(M.mul(Li,E),M.T(Li))).val.pop(), need=e=>{ const i=pts.findIndex(p=>p.y<=e); return i<0?"more than 40":String(i); };
    host.innerHTML=""; host.append(lineChart({w:600,h:280,aria:"Relative error of the truncated power series against the number of terms, on a logarithmic vertical axis",
      x:{min:0,max:40,ticks:[0,10,20,30,40].map(v=>[v,String(v)]),label:"cut-off m (number of extra terms)"},y:{min:-12,max:0,ticks:[0,-3,-6,-9,-12].map(v=>[v,v?"1e"+v:"1"]),label:"relative error"},
      series:[{name:"error of the m-term series",color:"var(--c5)",points:pts}]}));
    st.innerHTML=""; st.append(stat("Spectral radius ρ",rho.toFixed(4),"of H_cc⁻¹ H_cp H_pp⁻¹ H_cpᵀ"),stat("Terms for 1% error",need(-2)),stat("Terms for 10⁻⁶ error",need(-6))); };
  el.append(h("div",{class:"vt"},"Power series for the inverse Schur complement"),h("p",{class:"vd"},"Equation (7.9) on the 18 × 18 reduced camera system of Day 3: how close is the sum of the first m + 1 terms to the true inverse?"),
    h("div",{class:"ctl"},range("Damping λ",-3,1,0.5,lg,v=>{ lg=v; draw(); },v=>(10**v).toExponential(0)).el),host,st);
  draw();
};

// Day 5: fusing depth images into a truncated signed distance function, equation (7.12), along one camera ray.
VIZ.w12_tsdf=el=>{
  const S0=2, X0=1, X1=3, NV=101; let K=5, sg=0.05, tau=0.3, seed=1;
  const svg=sv("svg",{viewBox:"0 0 600 260",role:"img","aria-label":"Signed distance along one viewing ray: each depth reading gives a truncated ramp, their average is the fused distance function, and its zero crossing is the fused surface"});
  const st=h("div",{class:"stats"});
  const draw=()=>{ const r=rng(seed), zs=Array.from({length:K},()=>S0+sg*r.n()), xs=Array.from({length:NV},(_,k)=>X0+(X1-X0)*k/(NV-1));
    const di=(z,x)=>Math.max(-tau,Math.min(tau,z-x)), D=xs.map(x=>{ let a=0, w=0; zs.forEach(z=>{ if(z-x>-tau){ a+=di(z,x); w++; } }); return w?a/w:null; });
    let cross=null; for(let k=0;k<NV-1&&cross==null;k++) if(D[k]!=null&&D[k+1]!=null&&D[k]>0&&D[k+1]<=0) cross=xs[k]+(xs[k+1]-xs[k])*D[k]/(D[k]-D[k+1]);
    const X=x=>(40+(x-X0)/(X1-X0)*540).toFixed(1), Y=v=>(130-v*170).toFixed(1), mean=zs.reduce((a,b)=>a+b,0)/K; svg.innerHTML="";
    svg.append(sv("line",{x1:40,x2:580,y1:130,y2:130,stroke:"var(--line2)"}),sv("line",{x1:X(S0),x2:X(S0),y1:14,y2:236,stroke:"var(--muted)","stroke-dasharray":"4 3"}),txt(+X(S0)+6,24,"true surface",{class:"mut","font-size":11}),
      txt(40,24,"in front of the surface: D > 0",{class:"mut","font-size":11}),txt(580,232,"behind it: D < 0",{class:"mut","font-size":11,"text-anchor":"end"}));
    zs.forEach(z=>svg.append(sv("path",{d:xs.map((x,k)=>(k?"L":"M")+X(x)+" "+Y(di(z,x))).join(" "),fill:"none",stroke:"var(--c5)","stroke-width":1,opacity:0.45})));
    let dd=""; D.forEach((v,k)=>{ if(v!=null) dd+=(dd?"L":"M")+X(xs[k])+" "+Y(v)+" "; }); svg.append(sv("path",{d:dd,fill:"none",stroke:"var(--c1)","stroke-width":3}));
    if(cross!=null) svg.append(sv("circle",{cx:X(cross),cy:130,r:6,fill:"var(--acc)",stroke:"var(--surface)","stroke-width":2}));
    [1,1.5,2,2.5,3].forEach(x=>svg.append(txt(X(x),254,x.toFixed(1)+" m",{"text-anchor":"middle",class:"mut","font-size":11})));
    st.innerHTML=""; st.append(stat("Fused surface",cross==null?"—":cross.toFixed(3)+" m","zero crossing of D"),stat("Mean of the readings",mean.toFixed(3)+" m",K+" depth image"+(K>1?"s":"")),
      stat("Error of the fused surface",cross==null?"—":(1000*Math.abs(cross-S0)).toFixed(0)+" mm","one reading: about "+(1000*sg).toFixed(0)+" mm")); };
  el.append(h("div",{class:"vt"},"Fusing depth images along one ray"),h("p",{class:"vd"},"Every depth image gives each voxel on the ray a truncated signed distance to the surface it measured. Equation (7.12) averages them; the surface is where the average crosses zero."),
    h("div",{class:"ctl"},range("Depth images",1,30,1,K,v=>{ K=v; draw(); }).el,range("Depth noise σ",0,0.2,0.01,sg,v=>{ sg=v; draw(); },v=>(100*v).toFixed(0)+" cm").el,range("Truncation τ",0.05,0.6,0.05,tau,v=>{ tau=v; draw(); },v=>(100*v).toFixed(0)+" cm").el),
    h("div",{class:"ctl"},button("New noise",()=>{ seed++; draw(); })),svg,legend([["var(--c5)","one depth image"],["var(--c1)","fused D(x)"],["var(--acc)","fused surface"]]),st);
  draw();
};
