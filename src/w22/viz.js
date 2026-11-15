// Week 22 · Metric-semantic SLAM (Chapter 16). Five widgets:
//   w22_shape    deformable keypoint shape model (16.2) fitted to detected semantic keypoints (16.3)
//   w22_quadric  ellipsoid -> dual quadric -> dual conic -> bounding box (16.5)-(16.11)
//   w22_assoc    nearest-neighbour vs max-mixture vs EM data association among look-alike objects (§16.2.2)
//   w22_fusion   Bayesian multi-class log-odds fusion for one voxel (16.24)-(16.26)
//   w22_scene    queries on a small 3D scene graph, and the memory counts (16.34)-(16.35)
const w22set=(sl,v,fmt)=>{ sl.input.value=v; sl.el.querySelector("output").textContent=fmt(v); };   // move a slider without firing its handler
const w22leg=(lg,items)=>{ lg.innerHTML=""; items.forEach(([c,t])=>lg.append(h("span",{},h("i",{style:"background:"+c}),t))); };

// ---- Day 1: a car seen from above. Shape = mean + c1*(longer front) + c2*(wider); placed with scale λ and yaw (position known).
VIZ.w22_shape=el=>{
  const B0=[[2.3,0],[1.8,0.7],[-1.8,0.7],[-2.3,0],[-1.8,-0.7],[1.8,-0.7]],     // mean shape b_{j,0}: front, 4 wheels, rear (m, object frame)
        B1=[[0.9,0],[0.6,0],[0,0],[0,0],[0,0],[0.6,0]],                        // mode 1 b_{j,1}: longer front
        B2=[[0,0],[0,0.3],[0,0.3],[0,0],[0,-0.3],[0,-0.3]];                    // mode 2 b_{j,2}: wider
  const TRUE=[0.8,0.5,1.3,25*Math.PI/180], LO=[0,0,0.6,-40*Math.PI/180], HI=[1.2,1.2,1.6,40*Math.PI/180];
  const W=600, H=330, sc=40, X=p=>300+p[0]*sc, Y=p=>165-p[1]*sc;
  const pred=t=>{ const c=Math.cos(t[3]), s=Math.sin(t[3]); return B0.map((b,j)=>{ const x=b[0]+t[0]*B1[j][0]+t[1]*B2[j][0], y=b[1]+t[0]*B1[j][1]+t[1]*B2[j][1]; return [t[2]*(c*x-s*y), t[2]*(s*x+c*y)]; }); };
  const rn=rng(5), Z=pred(TRUE).map(p=>[p[0]+rn.n()*0.05, p[1]+rn.n()*0.05]);    // detected keypoints, 5 cm noise
  const res=t=>{ const P=pred(t), r=[]; P.forEach((p,j)=>r.push(Z[j][0]-p[0], Z[j][1]-p[1])); return r; };
  const cost=t=>res(t).reduce((s,v)=>s+v*v,0);
  let th=[0,0,1,0];
  const fm=[v=>(+v).toFixed(2), v=>(+v).toFixed(2), v=>(+v).toFixed(2), v=>(+v).toFixed(0)+"°"];
  const sl=[range("Shape c₁ (longer front)",0,1.2,0.01,0,v=>{ th[0]=v; draw(); },fm[0]), range("Shape c₂ (wider)",0,1.2,0.01,0,v=>{ th[1]=v; draw(); },fm[1]),
            range("Scale λ",0.6,1.6,0.01,1,v=>{ th[2]=v; draw(); },fm[2]), range("Yaw of pose q",-40,40,1,0,v=>{ th[3]=v*Math.PI/180; draw(); },fm[3])];
  const sync=()=>{ th=th.map((v,i)=>Math.max(LO[i],Math.min(HI[i],v))); sl.forEach((s,i)=>w22set(s,i===3?th[3]*180/Math.PI:th[i],fm[i])); draw(); };
  const fitShape=()=>{ // linear least squares in (c1, c2) with λ and yaw held: z_j - λR b_j0 = λR (c1 b_j1 + c2 b_j2)
    const r0=res([0,0,th[2],th[3]]), a1=pred([1,0,th[2],th[3]]), a2=pred([0,1,th[2],th[3]]), a0=pred([0,0,th[2],th[3]]);
    const A=[]; a0.forEach((p,j)=>{ A.push([a1[j][0]-p[0], a2[j][0]-p[0]], [a1[j][1]-p[1], a2[j][1]-p[1]]); });
    const At=M.T(A), c=M.solve(M.mul(At,A), M.mv(At,r0)); if(c){ th[0]=c[0]; th[1]=c[1]; } sync(); };
  const gnStep=()=>{ // one Gauss-Newton step on all four unknowns, numerical Jacobian of the 12 residuals
    const r=res(th), J=r.map(()=>[0,0,0,0]);
    for(let k=0;k<4;k++){ const a=th.slice(), b=th.slice(); a[k]+=1e-5; b[k]-=1e-5; const ra=res(a), rb=res(b); r.forEach((_,i)=>{ J[i][k]=(rb[i]-ra[i])/2e-5; }); }   // J = d(pred)/dθ = -d(res)/dθ
    const Jt=M.T(J), Hm=M.mul(Jt,J); for(let k=0;k<4;k++) Hm[k][k]+=1e-9;
    const d=M.solve(Hm, M.mv(Jt,r)); if(d) th=th.map((v,k)=>v+d[k]); sync(); };
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Top view of a car outline built from six semantic landmarks, compared with six detected keypoints"});
  const st=h("div",{class:"stats"}), lg=h("div",{class:"legend"});
  w22leg(lg,[["var(--c5)","predicted semantic landmarks λ·R·sⱼ"],["var(--c3)","detected keypoints zⱼ"],["var(--c2)","residuals"]]);
  function draw(){ const P=pred(th), c=cost(th); svg.innerHTML="";
    svg.append(sv("line",{x1:20,y1:165,x2:580,y2:165,stroke:"var(--line)"}), sv("line",{x1:300,y1:15,x2:300,y2:315,stroke:"var(--line)"}));
    svg.append(sv("path",{d:P.map((p,i)=>(i?"L":"M")+X(p).toFixed(1)+" "+Y(p).toFixed(1)).join(" ")+" Z",fill:"var(--c5-soft)",stroke:"var(--c5)","stroke-width":2,"stroke-linejoin":"round"}));
    P.forEach((p,j)=>svg.append(sv("line",{x1:X(p),y1:Y(p),x2:X(Z[j]),y2:Y(Z[j]),stroke:"var(--c2)","stroke-width":2.5})));
    P.forEach(p=>svg.append(sv("circle",{cx:X(p),cy:Y(p),r:5,fill:"var(--c5)"})));
    Z.forEach(z=>svg.append(sv("circle",{cx:X(z),cy:Y(z),r:6,fill:"none",stroke:"var(--c3)","stroke-width":2.5})));
    svg.append(txt(24,30,"world axes; the object sits at the origin",{class:"mut","font-size":11}));
    st.innerHTML=""; st.append(stat("Cost Σ‖zⱼ − prediction‖²",c.toFixed(4)+" m²","6 keypoints, 12 residuals"), stat("RMS keypoint error",(Math.sqrt(c/6)*100).toFixed(1)+" cm","noise level is about 7 cm"),
      stat("Estimate (c₁, c₂, λ, yaw)",th[0].toFixed(2)+", "+th[1].toFixed(2)+", "+th[2].toFixed(2)+", "+(th[3]*180/Math.PI).toFixed(1)+"°","4 unknowns instead of 12 free coordinates")); }
  el.append(h("div",{class:"vt"},"A deformable shape model fitted to keypoints"),h("p",{class:"vd"},"Shape sⱼ = bⱼ,₀ + c₁bⱼ,₁ + c₂bⱼ,₂, then scaled by λ and rotated by the object's yaw. A bird's-eye camera is assumed, so the projection only drops the height."),
    h("div",{class:"ctl"},...sl.map(s=>s.el)),
    h("div",{class:"ctl"},button("Fit shape only (c₁, c₂)",fitShape),button("Gauss-Newton step (all four)",gnStep,"primary"),button("Reset",()=>{ th=[0,0,1,0]; sync(); },"ghost")),svg,lg,st);
  draw();
};

// ---- Day 2: project an ellipsoid with the dual quadric. Camera at the world origin looking along +z (T = identity), x right, y down.
VIZ.w22_quadric=el=>{
  const S=[2.2,0.75,0.9], DIR=[0.25,0,1];        // semi-axes s (m); the object's centre is d·DIR, at camera height
  let lam=1, d=6, yaw=30;
  const conic=(lam,d,yaw)=>{ const a=yaw*Math.PI/180, c=Math.cos(a), s=Math.sin(a), R=[[c,0,s],[0,1,0],[-s,0,c]];
    const m=M.mul(M.mul(R,[[lam*lam*S[0]*S[0],0,0],[0,lam*lam*S[1]*S[1],0],[0,0,lam*lam*S[2]*S[2]]]),M.T(R)), t=DIR.map(v=>v*d);
    const C=m.map((r,i)=>r.map((v,j)=>v-t[i]*t[j]));              // P Q*_ℓ Pᵀ = R Λ Rᵀ − t tᵀ, the upper-left block of (16.7)
    const n=-C[2][2]; if(n<=1e-6) return null;                     // camera inside the ellipsoid's depth range: no outline
    const Ch=C.map(r=>r.map(v=>v/n)), cu=-Ch[0][2], cv=-Ch[1][2];  // normalised so the corner is −1 (the role of β)
    const S2=[[Ch[0][0]+cu*cu, Ch[0][1]+cu*cv],[Ch[0][1]+cu*cv, Ch[1][1]+cv*cv]];
    if(S2[0][0]<=0||S2[1][1]<=0) return null;
    return {Ch,cu,cv,S2,m,t,hw:Math.sqrt(S2[0][0]),hh:Math.sqrt(S2[1][1])}; };
  const vech=C=>[C[0][0],C[1][0],C[2][0],C[1][1],C[2][1],C[2][2]];
  const T0=conic(1,6,30), BOX=[T0.cu-T0.hw, T0.cv-T0.hh, 2*T0.hw, 2*T0.hh];      // the detection z = (u, v, w, h)
  const cu0=BOX[0]+BOX[2]/2, cv0=BOX[1]+BOX[3]/2, a2=BOX[2]*BOX[2]/4, b2=BOX[3]*BOX[3]/4;
  const CZ=[[a2-cu0*cu0,-cu0*cv0,-cu0],[-cu0*cv0,b2-cv0*cv0,-cv0],[-cu0,-cv0,-1]];   // (16.9) multiplied out
  const W=600, H=300, U=u=>10+(u+0.45)*200, V=v=>150+v*200, TX=x=>420+x*18, TZ=z=>272-z*18;
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Left: the image plane with the projected ellipse of an ellipsoid, its bounding box, and the detected bounding box. Right: top view of the camera, the ellipsoid, and the two tangent rays"});
  const st=h("div",{class:"stats"}), lg=h("div",{class:"legend"}), warn=h("div",{class:"warn"});
  w22leg(lg,[["var(--c5)","projected conic and its box"],["var(--c3)","detected bounding box z"],["var(--muted)","tangent rays (top view)"]]);
  const draw=()=>{ const q=conic(lam,d,yaw); svg.innerHTML="";
    svg.append(sv("defs",{},sv("clipPath",{id:"w22clipA"},sv("rect",{x:10,y:24,width:280,height:266})),sv("clipPath",{id:"w22clipB"},sv("rect",{x:310,y:24,width:280,height:266}))));
    svg.append(sv("rect",{x:10,y:24,width:280,height:266,rx:8,fill:"var(--soft)",stroke:"var(--line2)"}),sv("rect",{x:310,y:24,width:280,height:266,rx:8,fill:"var(--soft)",stroke:"var(--line2)"}),
      txt(14,16,"image plane (normalized coordinates)",{class:"mut","font-size":12}),txt(314,16,"top view (x right, depth z up)",{class:"mut","font-size":12}));
    const gA=sv("g",{"clip-path":"url(#w22clipA)"}), gB=sv("g",{"clip-path":"url(#w22clipB)"}); svg.append(gA,gB);
    gA.append(sv("rect",{x:U(BOX[0]),y:V(BOX[1]),width:BOX[2]*200,height:BOX[3]*200,fill:"none",stroke:"var(--c3)","stroke-width":2.5}));
    gB.append(sv("path",{d:"M"+TX(0)+" "+TZ(0)+" l-9 12 h18 z",fill:"var(--ink)"}));
    if(q){ gA.append(covEllipse(q.S2,U(q.cu),V(q.cv),1,200,{stroke:"var(--c5)",fill:"var(--c5-soft)","fill-opacity":0.7}),
        sv("rect",{x:U(q.cu-q.hw),y:V(q.cv-q.hh),width:2*q.hw*200,height:2*q.hh*200,fill:"none",stroke:"var(--c5)","stroke-width":1.5,"stroke-dasharray":"5 4"}));
      [q.cu-q.hw,q.cu+q.hw].forEach(u=>gB.append(sv("line",{x1:TX(0),y1:TZ(0),x2:TX(u*16),y2:TZ(16),stroke:"var(--muted)","stroke-width":1.2,"stroke-dasharray":"4 4"})));
      gB.append(covEllipse([[q.m[0][0],-q.m[0][2]],[-q.m[0][2],q.m[2][2]]],TX(q.t[0]),TZ(q.t[2]),1,18,{stroke:"var(--c5)",fill:"var(--c5-soft)"})); }
    const r=q?Math.sqrt(vech(q.Ch).reduce((s,v,i)=>s+(v-vech(CZ)[i])**2,0)):NaN;
    warn.textContent=q?"":"The camera is inside the ellipsoid's depth range, so there is no outline. Increase the distance.";
    st.innerHTML=""; st.append(stat("Detected box (w × h)",BOX[2].toFixed(3)+" × "+BOX[3].toFixed(3),"made with λ = 1, d = 6 m, yaw 30°"),
      stat("Projected box (w × h)",q?(2*q.hw).toFixed(3)+" × "+(2*q.hh).toFixed(3):"none","from the dual conic C*ₓ"),
      stat("Residual ‖b − Av/β‖",q?r.toFixed(4):"—","equation (16.11), Σ = I")); };
  el.append(h("div",{class:"vt"},"From an ellipsoid to a bounding box"),h("p",{class:"vd"},"A car-sized ellipsoid (semi-axes 2.2, 0.75, 0.9 m, times λ) is projected with C* = P T⁻¹ Q* T⁻ᵀ Pᵀ. The widget reads the box off the conic and compares it with one detection. The object sits at camera height, so its outline is an axis-aligned ellipse like the one inside a box; in general the outline is tilted and (16.10) cannot reach exactly zero."),
    h("div",{class:"ctl"},range("Scale λ",0.5,2,0.1,lam,v=>{ lam=v; draw(); },v=>(+v).toFixed(1)).el,range("Distance d",3,12,0.5,d,v=>{ d=v; draw(); },v=>(+v).toFixed(1)+" m").el,range("Yaw",0,180,5,yaw,v=>{ yaw=v; draw(); },v=>v+"°").el),svg,warn,lg,st);
  draw();
};

// ---- Day 3: the robot returns to six mapped objects with a drifted position estimate. Which detection is which object?
VIZ.w22_assoc=el=>{
  const L=[[0.6,4,0],[1.8,4,0],[3,4,1],[4.2,4,0],[5.4,4,0],[-1.4,2.2,1]];     // map: x, y, class (0 chair, 1 table)
  const PT=[3,0], SP=1.5, ACC=0.9, NZ=0.1, CC=["var(--c5)","var(--c2)"], CN=["chair","table"];
  let seed=4, drift=0.8, sig=0.4, method="mm", sem=true, Z, p0, p, Wt, it;
  const make=()=>{ const r=rng(seed); Z=L.map(l=>({z:[l[0]-PT[0]+r.n()*NZ, l[1]-PT[1]+r.n()*NZ], y:r.u()<ACC?l[2]:1-l[2]})); };   // z = ℓ − p + noise, label right 90% of the time
  const reset=()=>{ p0=[PT[0]+drift,PT[1]]; p=p0.slice(); Wt=null; it=0; draw(); };
  const lik=(k,j,q)=>{ const dx=Z[k].z[0]+q[0]-L[j][0], dy=Z[k].z[1]+q[1]-L[j][1]; return Math.exp(-(dx*dx+dy*dy)/(2*sig*sig))*(sem?(Z[k].y===L[j][2]?ACC:1-ACC):1); };
  const step=()=>{
    if(method==="nn"){ if(!Wt) Wt=Z.map(m=>{ let b=0, bd=1e9; L.forEach((l,j)=>{ const dd=Math.hypot(m.z[0]+p0[0]-l[0], m.z[1]+p0[1]-l[1]); if(dd<bd){ bd=dd; b=j; } }); return L.map((_,j)=>+(j===b)); }); }   // decided once, geometry only
    else Wt=Z.map((m,k)=>{ const v=L.map((_,j)=>lik(k,j,p)); if(method==="mm"){ const b=v.indexOf(Math.max(...v)); return v.map((_,j)=>+(j===b)); }   // max-mixture: keep the best component
      const s=v.reduce((a,b)=>a+b,0)||1e-300; return v.map(x=>x/s); });                                                                                    // EM: E-step weights
    let nx=p0[0]/(SP*SP), ny=p0[1]/(SP*SP), den=1/(SP*SP);                      // weighted least squares for p (the M-step); the odometry prior has σ = 1.5 m
    Z.forEach((m,k)=>L.forEach((l,j)=>{ const w=Wt[k][j]/(sig*sig); nx+=w*(l[0]-m.z[0]); ny+=w*(l[1]-m.z[1]); den+=w; }));
    p=[nx/den,ny/den]; it++; };
  const W=600, H=330, X=x=>20+(x+2)*62, Y=y=>292-y*62;
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Top view: six mapped objects, the robot's true and estimated position, the six detections placed with the current estimate, and lines showing which object each detection is associated with"});
  const st=h("div",{class:"stats"}), lg=h("div",{class:"legend"}), tb=h("div",{class:"panel"});
  w22leg(lg,[[CC[0],"chair"],[CC[1],"table"],["var(--c1)","weight on the right object"],["var(--c3)","weight on a wrong object"]]);
  function draw(){ svg.innerHTML="";
    if(Wt) Z.forEach((m,k)=>L.forEach((l,j)=>{ const w=Wt[k][j]; if(w>0.02) svg.append(sv("line",{x1:X(m.z[0]+p[0]),y1:Y(m.z[1]+p[1]),x2:X(l[0]),y2:Y(l[1]),stroke:j===k?"var(--c1)":"var(--c3)","stroke-width":(1+3*w).toFixed(1),"stroke-opacity":(0.25+0.75*w).toFixed(2)})); }));
    L.forEach((l,j)=>svg.append(sv("rect",{x:X(l[0])-9,y:Y(l[1])-9,width:18,height:18,rx:3,fill:CC[l[2]]}),txt(X(l[0]),Y(l[1])-15,"ℓ"+(j+1),{"text-anchor":"middle","font-size":12,"data-top":1,"paint-order":"stroke",stroke:"var(--surface)","stroke-width":4})));
    Z.forEach((m,k)=>svg.append(sv("circle",{cx:X(m.z[0]+p[0]),cy:Y(m.z[1]+p[1]),r:6,fill:CC[m.y],stroke:"var(--ink)","stroke-width":1.5}),txt(X(m.z[0]+p[0]),Y(m.z[1]+p[1])+21,"z"+(k+1),{"text-anchor":"middle","font-size":11,class:"mut","data-top":1})));
    svg.append(sv("circle",{cx:X(PT[0]),cy:Y(PT[1]),r:9,fill:"none",stroke:"var(--ink)","stroke-width":2,"stroke-dasharray":"3 3"}),sv("circle",{cx:X(p[0]),cy:Y(p[1]),r:7,fill:"var(--ink)"}),
      txt(X(PT[0])-14,Y(PT[1])+24,"true position",{"text-anchor":"end","font-size":11,class:"mut"}),txt(X(p[0])+12,Y(p[1])+24,"estimate",{"font-size":11,class:"mut","data-top":1}));
    const wrong=Wt?Wt.filter((w,k)=>w.indexOf(Math.max(...w))!==k).length:null;
    st.innerHTML=""; st.append(stat("Position error",Math.hypot(p[0]-PT[0],p[1]-PT[1]).toFixed(2)+" m",it?"after "+it+" step"+(it>1?"s":""):"odometry guess"),
      stat("Wrong associations",wrong==null?"—":wrong+" of 6","largest weight is not the true object"),stat("Detected labels",Z.map(m=>CN[m.y][0].toUpperCase()).join(" "),"truth: C C T C C T"));
    tb.innerHTML=""; const t=h("table",{},h("tr",{},h("th",{},"w"),...L.map((l,j)=>h("th",{},"ℓ"+(j+1)+" "+CN[l[2]][0].toUpperCase()))));
    Z.forEach((m,k)=>t.append(h("tr",{},h("td",{},"z"+(k+1)+" "+CN[m.y][0].toUpperCase()),...L.map((_,j)=>{ const w=Wt?Wt[k][j]:0; return h("td",{style:w>0.02?"background:var(--"+(j===k?"c1":"c3")+"-soft);font-weight:600":""},Wt?(w<0.005?"·":w.toFixed(2)):"·"); }))));
    tb.append(h("p",{},it?"Association weights used in step "+it+" (rows: detections with their reported label; columns: mapped objects).":"No associations yet. Press Step."),t); }
  const lb=button("Class labels: used",()=>{ sem=!sem; lb.textContent="Class labels: "+(sem?"used":"ignored"); lb.classList.toggle("on",sem); reset(); }); lb.classList.add("on");
  const ms=select("Method ",[["mm","Max-mixture (hard, re-chosen every step)"],["nn","Nearest neighbour (hard, chosen once)"],["em","EM (soft weights)"]],v=>{ method=v; reset(); });
  el.append(h("div",{class:"vt"},"Which chair is which? Three ways to associate"),h("p",{class:"vd"},"The map (six objects) is known. The robot's position is the only continuous unknown; it starts at the drifted odometry guess. Each step re-estimates the associations, then the position."),
    h("div",{class:"ctl"},ms.el,lb),
    h("div",{class:"ctl"},range("Odometry drift",0,1.4,0.1,drift,v=>{ drift=v; reset(); },v=>(+v).toFixed(1)+" m").el,range("Association σ",0.15,0.8,0.05,sig,v=>{ sig=v; reset(); },v=>(+v).toFixed(2)+" m").el),
    h("div",{class:"ctl"},button("Step",()=>{ step(); draw(); },"primary"),button("Run 10 steps",()=>{ for(let i=0;i<10;i++) step(); draw(); }),button("Reset",reset,"ghost"),button("New noise",()=>{ seed++; make(); reset(); },"ghost")),svg,lg,st,tb);
  make(); reset();
};

// ---- Day 4: one voxel, classes {free, chair, table}. Log-odds h relative to "free" accumulate the observation log-odds l(z).
VIZ.w22_fusion=el=>{
  const NAME=["free","chair","table"], PASS=[0.9,0.2,0.2], FH=0.05;         // p(ray passes | class); p(hit with either label | free)
  let acc=0.75, truth=1, hv, hist, r, last;
  const pz=(z,k)=>z===0?PASS[k]:(k===0?FH:(1-PASS[k])*(z===k?acc:1-acc));     // z: 0 = ray passes through, 1 = hit labelled chair, 2 = hit labelled table
  const lz=z=>[0,1,2].map(k=>Math.log(pz(z,k)/pz(z,0)));                      // (16.25): log p(z|m=k) / p(z|m=0)
  const soft=v=>{ const mx=Math.max(...v), e=v.map(x=>Math.exp(x-mx)), s=e[0]+e[1]+e[2]; return e.map(x=>x/s); };
  const obs=z=>{ const l=lz(z); hv=hv.map((v,k)=>v+l[k]); hist.push(soft(hv)); last=[z,l]; };       // (16.24)
  const sample=()=>{ const u=r.u(); let s=0; for(let z=0;z<3;z++){ s+=pz(z,truth); if(u<s) return z; } return 2; };
  const reset=()=>{ hv=[0,0,0]; hist=[soft(hv)]; r=rng(11); last=null; draw(); };
  const chart=h("div"), st=h("div",{class:"stats"}), out=h("div",{class:"out"});
  const ZN=["ray passes through","hit labelled chair","hit labelled table"];
  function draw(){ const n=hist.length-1, pr=hist[n], xm=Math.max(10,n);
    chart.innerHTML=""; chart.append(lineChart({w:600,h:250,aria:"Probability of each class for the voxel after each observation",x:{min:0,max:xm,ticks:[0,0.25,0.5,0.75,1].map(f=>[Math.round(f*xm),""+Math.round(f*xm)]),label:"observations fused"},
      y:{min:0,max:1,ticks:[[0,"0"],[0.5,"0.5"],[1,"1"]],label:"probability"},series:[0,1,2].map(k=>({name:NAME[k],color:["var(--muted)","var(--c5)","var(--c2)"][k],dots:n<=24,points:hist.map((q,i)=>({x:i,y:q[k],label:NAME[k]+" after "+i,fmt:v=>v.toFixed(3)}))}))}));
    st.innerHTML=""; [0,1,2].forEach(k=>st.append(stat("p("+NAME[k]+")",(pr[k]*100).toFixed(pr[k]<0.01||pr[k]>0.99?2:1)+"%","h = "+hv[k].toFixed(2))));
    out.textContent=last?"Observation "+n+": "+ZN[last[0]]+"   l(z) = ["+last[1].map(v=>v.toFixed(2)).join(", ")+"]":"No observations yet: h = [0, 0, 0], a uniform prior over the three classes."; }
  const ts=select("True class of the voxel ",[["1","chair"],["2","table"],["0","free"]],v=>{ truth=+v; reset(); });
  el.append(h("div",{class:"vt"},"Bayesian label fusion in one voxel"),h("p",{class:"vd"},"Each observation adds its log-odds vector l(z) to the voxel's h. The softmax of h gives the class probabilities. Sampled observations come from the sensor model with the true class chosen below."),
    h("div",{class:"ctl"},ts.el,range("Label accuracy of the segmentation",0.5,0.95,0.05,acc,v=>{ acc=v; reset(); },v=>Math.round(v*100)+"%").el),
    h("div",{class:"ctl"},button("Sample 1",()=>{ obs(sample()); draw(); },"primary"),button("Sample 10",()=>{ for(let i=0;i<10;i++) obs(sample()); draw(); }),button("Reset",reset,"ghost")),
    h("div",{class:"ctl"},button("Add: hit chair",()=>{ obs(1); draw(); }),button("Add: hit table",()=>{ obs(2); draw(); }),button("Add: pass through",()=>{ obs(0); draw(); })),chart,out,st);
  reset();
};

// ---- Day 5: a three-layer scene graph (building → rooms → objects) and what a query has to touch.
VIZ.w22_scene=el=>{
  const ROOMS=[["kitchen",[1.5,4.5],["fridge","stove","mug","chair"]],["living room",[6,4.5],["sofa","table","chair","lamp","mug"]],["bedroom",[2,1.5],["bed","lamp","chair"]],["bathroom",[7.5,1.5],["toilet","sink"]]];
  const root={name:"flat",layer:3,pos:[5,3],kids:[]}, all=[root]; let ox=0;
  ROOMS.forEach(([name,pos,objs],ri)=>{ const rm={name,layer:2,pos,kids:[],par:root,sx:0}; root.kids.push(rm); all.push(rm);
    objs.forEach((c,i)=>{ const o={name:c,layer:1,par:rm,kids:[],pos:[+(pos[0]-0.9+0.6*i).toFixed(1),+(pos[1]+(i%2?0.5:-0.5)).toFixed(1)],sx:30+ox*41.5,row:ox%2}; ox++; rm.kids.push(o); all.push(o); });
    rm.sx=(rm.kids[0].sx+rm.kids[rm.kids.length-1].sx)/2; });
  root.sx=300; const NOBJ=all.filter(n=>n.layer===1).length;
  const QS=[{t:"Bring me the mug in the kitchen",room:"kitchen",cls:"mug"},{t:"Go to the living room",room:"living room"},{t:"Which room is the bed in?",cls:"bed",up:true},{t:"Count the chairs in the flat",cls:"chair"}];
  let qi=0, dl=0.1;
  const run=q=>{ const seen=[root], hit=[];               // top-down: open a room only if the query does not rule it out
    root.kids.forEach(rm=>{ seen.push(rm); if(q.room&&rm.name!==q.room) return; if(!q.cls){ hit.push(rm); return; }
      rm.kids.forEach(o=>{ seen.push(o); if(o.name===q.cls) hit.push(q.up?o.par:o); }); });
    return {seen,hit}; };
  const W=600, H=270, LY={3:34,2:112,1:196};
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"A scene graph drawn as a tree: one building node, four room nodes, fourteen object nodes. Nodes examined by the query and the answer nodes are highlighted"});
  const st=h("div",{class:"stats"}), out=h("div",{class:"out"}), lg=h("div",{class:"legend"});
  w22leg(lg,[["var(--c2)","examined by the query"],["var(--c1)","answer"],["var(--line2)","not touched"]]);
  const draw=()=>{ const q=QS[qi], {seen,hit}=run(q); svg.innerHTML="";
    all.forEach(n=>{ if(n.par) svg.append(sv("line",{x1:n.sx,y1:LY[n.layer],x2:n.par.sx,y2:LY[n.par.layer],stroke:seen.includes(n)?"var(--c2)":"var(--line2)","stroke-width":seen.includes(n)?2:1.2})); });
    all.forEach(n=>{ const a=hit.includes(n), s=seen.includes(n), y=LY[n.layer], k=a?"var(--c1)":s?"var(--c2)":"var(--line2)";
      if(n.layer===1) svg.append(sv("circle",{cx:n.sx,cy:y,r:8,fill:a?"var(--c1)":s?"var(--c2-soft)":"var(--surface)",stroke:k,"stroke-width":2}),
        txt(n.sx,y+24+(n.row?14:0),n.name,{"text-anchor":"middle","font-size":11,"data-top":1,"paint-order":"stroke",stroke:"var(--surface)","stroke-width":4}));
      else{ const w=n.name.length*7.4+18;                          // rooms and the building are pills that hide the edges behind them
        svg.append(sv("rect",{x:n.sx-w/2,y:y-12,width:w,height:24,rx:12,fill:a?"var(--c1-soft)":s?"var(--c2-soft)":"var(--surface)",stroke:k,"stroke-width":2}),txt(n.sx,y+4.5,n.name,{"text-anchor":"middle","font-size":13,"data-top":1})); } });
    ["objects","rooms","building"].forEach((s,i)=>svg.append(txt(6,[H-8,LY[2]-28,14][i],"layer "+(i+1)+": "+s,{class:"mut","font-size":11})));
    const vox=Math.round(10*6*2.5/(dl*dl*dl)), Lsym=new Set(all.map(n=>n.name)).size;
    out.textContent="Answer: "+(hit.length?(q.cls==="chair"?hit.length+" chairs, at ":"")+hit.map(n=>n.name+" ("+n.pos[0]+", "+n.pos[1]+") m").join("; "):"nothing found");
    st.innerHTML=""; st.append(stat("Nodes examined, scene graph",seen.length+" of "+all.length,"1 building + rooms + opened rooms' objects"),stat("Entries examined, flat object list",""+NOBJ,"every object, each tagged with its room"),
      stat("Memory, labels in every voxel",(Lsym*vox).toLocaleString("en-US"),"L·V/δ³ with L = "+Lsym+" symbols, V = 150 m³"),stat("Memory, hierarchy",(vox+all.length).toLocaleString("en-US"),"V/δ³ + 14 objects + 4 rooms + 1 building")); };
  el.append(h("div",{class:"vt"},"Asking a 3D scene graph"),h("p",{class:"vd"},"A 10 m × 6 m flat stored as building → rooms → objects. Every node carries a position. The query walks down from the top and skips rooms it can rule out."),
    h("div",{class:"ctl"},select("Query ",QS.map((q,i)=>[i,q.t]),v=>{ qi=+v; draw(); }).el,range("Voxel size δ",0.05,0.5,0.05,dl,v=>{ dl=v; draw(); },v=>(+v).toFixed(2)+" m").el),svg,lg,out,st);
  draw();
};
