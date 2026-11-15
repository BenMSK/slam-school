// Week 17 · Observability and visual-inertial odometry.
// Every widget builds a real observability / information matrix for a small model and reads the unobservable
// directions off its zero eigenvalues (M.eigSym).

const R2=(t,v)=>{ const c=Math.cos(t), s=Math.sin(t); return [c*v[0]-s*v[1], s*v[0]+c*v[1]]; };
const wrap=a=>Math.atan2(Math.sin(a),Math.cos(a));
const gram=A=>M.mul(M.T(A),A);                                   // AᵀA
const ZERO=1e-12;                                                // an eigenvalue below ZERO × (largest one) counts as zero
const nZero=val=>val.filter(v=>v<ZERO*val[val.length-1]).length;
const sig=v=>Math.abs(v)>=100||Math.abs(v)<0.01?v.toExponential(1):v.toPrecision(3);

// log-scale bar chart of eigenvalues (ascending); the bars under the dashed line are the zero eigenvalues
const spectrum=(svg,val)=>{ const n=val.length, mx=Math.max(val[n-1],1e-300), x0=62, bw=(590-x0)/n, Y=q=>20-q*6;
  svg.innerHTML="";
  [[0,"λmax"],[-4,"÷ 10⁴"],[-8,"÷ 10⁸"],[-12,"÷ 10¹²"],[-16,"÷ 10¹⁶"]].forEach(([q,l])=>svg.append(
    sv("line",q===-12?{x1:x0,x2:590,y1:Y(q),y2:Y(q),stroke:"var(--c3)","stroke-dasharray":"5 4"}:{class:"grid",x1:x0,x2:590,y1:Y(q),y2:Y(q)}),
    txt(x0-6,Y(q)+4,l,{"text-anchor":"end",class:"mut","font-size":11})));
  val.forEach((v,i)=>{ const z=v<ZERO*mx, q=Math.max(-16,Math.log10(Math.max(v,1e-300)/mx)), y=z?Y(-16)-4:Math.min(Y(q),Y(-16)-4);
    svg.append(sv("rect",{x:(x0+i*bw+3).toFixed(1),y:y.toFixed(1),width:(bw-6).toFixed(1),height:(Y(-16)-y).toFixed(1),rx:2,fill:z?"var(--c3)":"var(--c5)"}));
    if(z) svg.append(txt((x0+(i+.5)*bw).toFixed(1),Y(-16)+15,"0",{"text-anchor":"middle","font-size":12,fill:"var(--c3)"})); });
  svg.append(txt(326,146,"eigenvalues of the information matrix, smallest on the left",{"text-anchor":"middle",class:"mut","font-size":11})); };
const specSvg=()=>sv("svg",{viewBox:"0 0 600 150",role:"img","aria-label":"Bar chart, on a logarithmic scale, of the eigenvalues of the information matrix. Bars below the dashed line are zero: each one is an unobservable direction."});

// ---------- Day 1: the rank test on a 1-D cart ----------
VIZ.w17_rank=el=>{ let mode="pos", K=3;
  const tb=h("div",{class:"tbl"}), out=h("div",{class:"out"}), st=h("div",{class:"stats"});
  const draw=()=>{ const names=mode==="lm"?["p","v","b","ℓ"]:["p","v","b"], H=mode==="pos"?[1,0,0,0]:mode==="vel"?[0,1,0,0]:[-1,0,0,1], n=names.length, rows=[];
    for(let k=0;k<K;k++){ const P=[[1,k,-k*k/2,0],[0,1,-k,0],[0,0,1,0],[0,0,0,1]];             // Φ(k+1,1) = Φᵏ for Δt = 1 s
      rows.push(names.map((_,j)=>H.reduce((s,hv,i)=>s+hv*P[i][j],0))); }
    const e=M.eigSym(gram(rows)), nz=nZero(e.val);
    tb.innerHTML=""; tb.append(h("table",{},h("tr",{},h("th",{},"Row of M"),...names.map(q=>h("th",{},"column "+q))),
      ...rows.map((r,k)=>h("tr",{},h("td",{},"H Φ"+"⁰¹²³⁴⁵"[k]+"  (reading "+(k+1)+")"),...r.map(v=>h("td",{},String(v)))))));
    const basis=[]; for(let i=0;i<nz;i++){ const u=e.vec.map(r=>r[i]), big=u.reduce((a,b)=>Math.abs(b)>Math.abs(a)?b:a,0);
      basis.push("("+u.map((v,j)=>names[j]+": "+(Math.abs(v/big)<5e-7?"0":(v/big).toFixed(2))).join(",  ")+")"); }
    out.textContent=nz?"M u = 0 for u = "+basis.join("  and  "):"Only u = 0 solves M u = 0: every state can be recovered.";
    st.innerHTML=""; st.append(stat("States",String(n),names.join(", ")),stat("Rank of M",String(n-nz),K+" reading"+(K>1?"s":"")),stat("Unobservable directions",String(nz),nz?"rank is short by "+nz:"fully observable")); };
  el.append(h("div",{class:"vt"},"Rank test on a cart"),h("p",{class:"vd"},"A cart on a rail with position p, velocity v and accelerometer bias b; one time step is 1 s. Each reading adds the row H Φᵏ to the observability matrix M. The landmark option adds its unknown position ℓ to the state."),
    h("div",{class:"ctl"},select("What the extra sensor measures",[["pos","position p"],["vel","velocity v"],["lm","landmark distance ℓ − p"]],v=>{ mode=v; draw(); }).el,
      range("Number of readings",1,6,1,K,v=>{ K=v; draw(); }).el),tb,out,st);
  draw();
};

// ---------- Day 2: what one range / bearing measurement says about a point ----------
VIZ.w17_point=el=>{ const ox=70, oy=150, sc=60, SR=0.15, SB=3*Math.PI/180; let p=[4.5,1.2], lr=true, lb=true;
  const svg=sv("svg",{viewBox:"0 0 600 300",role:"img","aria-label":"A sensor at the left looks along the x axis at a point feature that can be dragged. Coloured curves show the region where the point can be, given a range measurement, a bearing measurement, or both."});
  const g=sv("g"), dot=sv("circle",{r:9,fill:"var(--acc)",stroke:"var(--surface)","stroke-width":2}), st=h("div",{class:"stats"}), out=h("div",{class:"out"});
  const draw=()=>{ const r=Math.hypot(p[0],p[1]), be=Math.atan2(p[1],p[0]), cx=ox+sc*p[0], cy=oy-sc*p[1];
    const Hr=[p[0]/r,p[1]/r], Hb=[-p[1]/(r*r),p[0]/(r*r)], wr=lr?1/(SR*SR):0, wb=lb?1/(SB*SB):0;
    const I=[[wr*Hr[0]*Hr[0]+wb*Hb[0]*Hb[0],wr*Hr[0]*Hr[1]+wb*Hb[0]*Hb[1]],[0,wr*Hr[1]*Hr[1]+wb*Hb[1]*Hb[1]]]; I[1][0]=I[0][1];
    g.innerHTML=""; g.append(sv("line",{x1:ox,y1:oy,x2:590,y2:oy,stroke:"var(--line2)","stroke-dasharray":"4 4"}),sv("line",{x1:ox,y1:oy,x2:cx,y2:cy,stroke:"var(--muted)","stroke-width":1.5}),
      sv("path",{d:"M"+(ox-14)+" "+(oy-10)+" L"+(ox+6)+" "+oy+" L"+(ox-14)+" "+(oy+10)+" z",fill:"var(--ink)"}),txt(ox-18,oy-18,"sensor",{"text-anchor":"end",class:"mut","font-size":12}));
    if(lr) [-3,3].forEach(k=>g.append(sv("circle",{cx:ox,cy:oy,r:Math.max(1,sc*(r+k*SR)),fill:"none",stroke:"var(--c2)","stroke-width":1.5})));
    if(lb) [-3,3].forEach(k=>{ const e=R2(be+k*SB,[9,0]); g.append(sv("line",{x1:ox,y1:oy,x2:ox+sc*e[0],y2:oy-sc*e[1],stroke:"var(--c5)","stroke-width":1.5})); });
    if(lr&&lb){ const S=M.inv(I); g.append(covEllipse([[S[0][0],-S[0][1]],[-S[0][1],S[1][1]]],cx,cy,3,sc,{stroke:"var(--c3)",fill:"var(--c3-soft)","fill-opacity":.6})); }
    dot.setAttribute("cx",cx); dot.setAttribute("cy",cy);
    const f=v=>(Math.abs(v)<5e-4?0:v).toFixed(3);
    out.textContent="H_r = [ "+f(Hr[0])+"  "+f(Hr[1])+" ]    H_b = [ "+f(Hb[0])+"  "+f(Hb[1])+" ] rad/m    H_b · p = "+f(Hb[0]*p[0]+Hb[1]*p[1]);
    st.innerHTML=""; st.append(stat("Range, bearing",r.toFixed(2)+" m, "+(be*180/Math.PI).toFixed(1)+"°","of the point in the sensor frame"),
      stat("Std. dev. along the ray",lr?SR.toFixed(2)+" m":"∞",lr?"set by the range noise":"no range: depth is free"),
      stat("Std. dev. across the ray",lb?(r*SB).toFixed(2)+" m":"∞",lb?"range × bearing noise":"no bearing: direction is free")); };
  drag(svg,dot,(x,y)=>{ p=[Math.min(8.4,Math.max(0.8,(x-ox)/sc)),Math.min(2.3,Math.max(-2.3,(oy-y)/sc))]; draw(); });
  const br=button("Range measured",()=>{ lr=!lr; br.classList.toggle("on",lr); draw(); }), bb=button("Bearing measured",()=>{ lb=!lb; bb.classList.toggle("on",lb); draw(); });
  br.classList.add("on"); bb.classList.add("on"); svg.append(g,dot);
  el.append(h("div",{class:"vt"},"One point measurement"),h("p",{class:"vd"},"Drag the point. Range noise is 0.15 m and bearing noise 3°; the curves are drawn at 3σ. The two buttons are the entries λr and λb of the selection matrix Λ."),
    h("div",{class:"ctl"},br,bb),svg,h("div",{class:"legend"},...[["var(--c2)","range ± 3σ"],["var(--c5)","bearing ± 3σ"],["var(--c3)","both: 3σ ellipse from (HᵀR⁻¹H)⁻¹"]].map(([c,t])=>h("span",{},h("i",{style:"background:"+c}),t))),out,st);
  draw();
};

// ---------- Days 3, 4, 6: a 2-D visual-inertial system ----------
// State, in the order of the book's (11.37): x = [θ, b_g, v(2), b_a(2), p(2), f_1(2), f_2(2), f_3(2)]  (14 numbers).
// IMU: one gyro and a 2-axis accelerometer at 20 Hz for 4 s. Camera: bearing (and optionally range) to 3 landmarks every 0.5 s.
const DT=0.05, NS=80, EVERY=10, LM=[[1.6,2.5],[3.9,-1.3],[-0.6,-2.6]], POS=[6,8,10,12];
function labModel(o){
  const g=o.view==="side"?[0,-9.81]:[0,0], still=o.motion==="pr", th0=0.3, bg=0.02, ba=[0.05,-0.03], p0=[-2.7,-0.5], v0=still?[0,0]:[1.1,0.3];
  const ab=t=>o.motion==="ca"?[0.25,0.35]:o.motion==="va"?[0.6*Math.sin(1.7*t),0.5*Math.cos(1.1*t)-0.1]:[0,0];   // kinematic acceleration, body frame
  const om=t=>(o.spin||still)?0.5+0.4*Math.sin(1.3*t):0;
  // IMU readings from the true motion. The truth is produced by the same discrete integration (11.12) that run() uses, so they fit exactly.
  const imu=[]; let th=th0;
  for(let i=0;i<NS;i++){ const a=ab(i*DT), w=om(i*DT), gb=R2(-th,g); imu.push([w+bg,a[0]-gb[0]+ba[0],a[1]-gb[1]+ba[1]]); th+=w*DT; }
  const x0=[th0,bg,...v0,...ba,...p0,...LM.flat()], rb=o.cam==="rb";
  const run=x=>{ let t=x[0], v=[x[2],x[3]], p=[x[6],x[7]]; const z=[], path=[], fr=[];        // integrate the IMU from initial state x, predict the camera readings
    for(let i=0;;i++){ path.push(p);
      if(i%EVERY===0){ fr.push([p[0],p[1],t]); for(let j=0;j<LM.length;j++){ const d=R2(-t,[x[8+2*j]-p[0],x[9+2*j]-p[1]]); z.push(Math.atan2(d[1],d[0])); if(rb) z.push(Math.hypot(d[0],d[1])); } }
      if(i===NS) break;
      const m=imu[i], a=R2(t,[m[1]-x[4],m[2]-x[5]]); a[0]+=g[0]; a[1]+=g[1];
      p=[p[0]+v[0]*DT+a[0]*DT*DT/2,p[1]+v[1]*DT+a[1]*DT*DT/2]; v=[v[0]+a[0]*DT,v[1]+a[1]*DT]; t+=(m[0]-x[1])*DT; }
    return {z,path,fr}; };
  const isB=i=>!rb||i%2===0, diff=(a,b)=>a.map((q,i)=>isB(i)?wrap(q-b[i]):q-b[i]);
  // observability matrix M = ∂(all camera readings)/∂(initial state), by central differences: the book's stack of H_k Φ(k,1)
  const n=x0.length, eps=1e-5, cols=[];
  for(let k=0;k<n;k++){ const a=[...x0], b=[...x0]; a[k]+=eps; b[k]-=eps; cols.push(diff(run(a).z,run(b).z).map(q=>q/(2*eps))); }
  const Mx=M.T(cols), eig=M.eigSym(gram(Mx)), lmax=eig.val[n-1];
  // the four named directions as tangent vectors u, and the information |M u|² / |u|² along each
  const J=v=>[-v[1],v[0]], put=(u,k,v)=>{ u[k]=v[0]; u[k+1]=v[1]; }, U={tx:new Array(n).fill(0),ty:new Array(n).fill(0),rot:new Array(n).fill(0),sc:new Array(n).fill(0)};
  POS.forEach(k=>{ U.tx[k]=1; U.ty[k+1]=1; put(U.rot,k,J([x0[k],x0[k+1]])); put(U.sc,k,[x0[k],x0[k+1]]); });
  U.rot[0]=1; put(U.rot,2,J(v0)); put(U.rot,4,R2(-th0,J(g)).map(q=>-q)); put(U.sc,2,v0); put(U.sc,4,ab(0).map(q=>-q));
  const info={}; for(const k in U){ const y=M.mv(Mx,U[k]); info[k]=y.reduce((s,q)=>s+q*q,0)/U[k].reduce((s,q)=>s+q*q,0); }
  // the same four changes at finite size (amt in [-1, 1]), applied to the initial state; the accelerometer bias absorbs what it can
  const label=(kind,amt)=>kind==="rot"?(0.6*amt*180/Math.PI).toFixed(0)+"°":kind==="sc"?"× "+(1+0.5*amt).toFixed(2):(1.2*amt).toFixed(2)+" m";
  const xform=(kind,amt)=>{ const x=[...x0];
    if(kind==="tx"||kind==="ty") POS.forEach(k=>{ x[k+(kind==="ty"?1:0)]+=1.2*amt; });
    else if(kind==="rot"){ const a=0.6*amt; x[0]+=a; [2,...POS].forEach(k=>put(x,k,R2(a,[x[k],x[k+1]]))); const d=R2(-th0,M.sub([R2(-a,g)],[g])[0]); x[4]+=d[0]; x[5]+=d[1]; }
    else { const s=1+0.5*amt, a=ab(0); [2,...POS].forEach(k=>{ x[k]*=s; x[k+1]*=s; }); x[4]+=(1-s)*a[0]; x[5]+=(1-s)*a[1]; }
    return x; };
  const change=x=>{ const d=diff(run(x).z,run(x0).z), rms=f=>{ const q=d.filter((_,i)=>f(i)); return q.length?Math.sqrt(q.reduce((s,v)=>s+v*v,0)/q.length):0; };
    return {b:rms(isB)*180/Math.PI,r:rms(i=>!isB(i))}; };
  return {x0,run,eig,lmax,nz:nZero(eig.val),info,xform,change,label,rb};
}
function lab(el,o){
  const st=Object.assign({view:"top",motion:"va",spin:true,cam:"b",kind:"rot",amt:0.5},o); let m;
  const scene=sv("svg",{viewBox:"0 0 600 300",role:"img","aria-label":"A 2-D scene with three landmarks and the path of a robot carrying an IMU and a camera. A second, coloured copy shows the whole solution after the chosen shift, rotation or scaling."});
  const spec=specSvg(), tb=h("div",{class:"tbl"}), stats=h("div",{class:"stats"}), note=h("p",{class:"warn"});
  const X=p=>(290+34*p[0]).toFixed(1), Y=p=>(150-34*p[1]).toFixed(1);
  const sol=(r,x,col,rays)=>{ const k=[];
    if(rays) [r.fr[0],r.fr[r.fr.length-1]].forEach(f=>LM.forEach((_,j)=>k.push(sv("line",{x1:X(f),y1:Y(f),x2:X([x[8+2*j],x[9+2*j]]),y2:Y([x[8+2*j],x[9+2*j]]),stroke:"var(--line2)","stroke-width":1}))));
    k.push(sv("path",{d:r.path.map((p,i)=>(i?"L":"M")+X(p)+" "+Y(p)).join(" "),fill:"none",stroke:col,"stroke-width":2.5,"stroke-linejoin":"round"}));
    r.fr.forEach(f=>{ const e=R2(f[2],[0.4,0]); k.push(sv("line",{x1:X(f),y1:Y(f),x2:X([f[0]+e[0],f[1]+e[1]]),y2:Y([f[0]+e[0],f[1]+e[1]]),stroke:col,"stroke-width":1.5}),sv("circle",{cx:X(f),cy:Y(f),r:3,fill:col})); });
    LM.forEach((_,j)=>k.push(sv("rect",{x:X([x[8+2*j],0])-5,y:Y([0,x[9+2*j]])-5,width:10,height:10,fill:col})));
    return k; };
  const draw=()=>{ const xt=m.xform(st.kind,st.amt), c=m.change(xt), thr=ZERO*m.lmax, side=st.view==="side";
    scene.innerHTML=""; scene.append(...sol(m.run(xt),xt,"var(--c3)",false),...sol(m.run(m.x0),m.x0,"var(--ink)",true));
    spectrum(spec,m.eig.val);
    const rows=[["tx","Shift everything along x"],["ty","Shift everything along "+(side?"z (up)":"y")],["rot",side?"Rotate everything in the plane (tilt against gravity)":"Rotate everything about gravity (yaw)"],["sc","Scale everything: path, speed, landmarks"]];
    tb.innerHTML=""; tb.append(h("table",{},h("tr",{},h("th",{},"Change to the whole solution"),h("th",{},"Information about it, |M u|² / |u|²"),h("th",{},"Verdict")),
      ...rows.map(([k,t])=>h("tr",{},h("td",{},t),h("td",{},m.info[k]<thr?"0":sig(m.info[k])),h("td",{},m.info[k]<thr?"unobservable":"observable")))));
    const extra=m.nz-rows.filter(([k])=>m.info[k]<thr).length;
    note.textContent=extra>0?extra+" more zero eigenvalue"+(extra>1?"s are":" is")+" not in the table: with no translation there is no parallax, so the depth of each landmark is unobservable on its own (feature scale).":"";
    const none=c.b<1e-6&&c.r<1e-6, f=v=>v<1e-6?"0":sig(v);
    stats.innerHTML=""; stats.append(stat("Zero eigenvalues",m.nz+" of "+m.x0.length,"unobservable directions"),
      stat("Applied change",rows.find(r=>r[0]===st.kind)[1].split(" ")[0]+" "+m.label(st.kind,st.amt),"pink copy; accelerometer bias re-fitted"),
      stat("Camera readings change by",f(c.b)+"° bearing"+(m.rb?", "+f(c.r)+" m range":""),none?"invisible to the sensors":"the data can tell the two apart")); };
  const build=()=>{ m=labModel(st); sp.disabled=st.motion==="pr"; draw(); };
  const sp=button("Body rotates",()=>{ st.spin=!st.spin; sp.classList.toggle("on",st.spin); build(); }); sp.classList.toggle("on",st.spin);
  const pick=(lab_,opts,key,after)=>{ const s=select(lab_,opts,v=>{ st[key]=v; after(); }); s.input.value=st[key]; return s.el; };
  const amt=range("Size of the change",-1,1,0.05,st.amt,v=>{ st.amt=v; draw(); },v=>v.toFixed(2));
  el.append(h("div",{class:"vt"},"Observability lab"),h("p",{class:"vd"},"A robot with a gyro, a 2-axis accelerometer and a camera that sees three landmarks. M is the derivative of all camera readings with respect to the 14 unknowns (initial heading, velocity, position, both biases, landmarks), with the IMU readings held fixed. Accelerations are set in the body frame."),
    h("div",{class:"ctl"},pick("View",[["top","top (gravity ⊥ plane)"],["side","side (gravity in plane)"]],"view",build),
      pick("Motion",[["cv","constant velocity"],["ca","constant acceleration"],["va","varying acceleration"],["pr","pure rotation"]],"motion",build),
      pick("Camera",[["b","bearing only (monocular)"],["rb","range and bearing"]],"cam",build),sp),
    h("div",{class:"ctl"},pick("Try this change",[["tx","shift along x"],["ty","shift along y / z"],["rot","rotate"],["sc","scale"]],"kind",draw),amt.el),
    scene,h("div",{class:"legend"},...[["var(--ink)","true solution, with rays at the first and last frame"],["var(--c3)","changed solution, driven by the same IMU readings"]].map(([c,t])=>h("span",{},h("i",{style:"background:"+c}),t))),
    spec,tb,note,stats);
  build();
}
VIZ.w17_obslab=el=>lab(el,{});
VIZ.w17_degenerate=el=>lab(el,{motion:"ca",kind:"sc"});

// ---------- Day 5: marginalization, two linearization points, and first-estimate Jacobians ----------
// 2-D SLAM with poses x0, x1, x2 (x, y, φ) and landmarks l1, l2. Old factors (odometry 0→1, landmarks seen from 0 and 1) are
// linearized at estimate A and x0 is marginalized with a Schur complement. New factors (odometry 1→2, landmarks seen from 2)
// are linearized later, at estimate B, unless first-estimate Jacobians keep A for the variables the prior touches.
const FA={x:[[0,0,0],[2,0.3,0.25],[3.8,1.2,0.6]],l:[[1.5,2.4],[4.3,-1.3]]};
function fejInfo(del,fej,seed){ const IX=[0,3,6], IL=[9,11], r=rng(seed);
  const B={x:FA.x.map((x,i)=>i?x.map(v=>v+del*r.n()):x),l:FA.l.map(l=>l.map(v=>v+del*r.n()))}, C=fej?{x:[FA.x[0],FA.x[1],B.x[2]],l:FA.l}:B;
  // rows of a factor z = Rᵢᵀ(q − pᵢ), where q is a landmark (k = its column) or the next position; odometry adds the row φⱼ − φᵢ
  const rel=(L,P,i,q,k)=>{ const x=P.x[i], d=[q[0]-x[0],q[1]-x[1]], c=Math.cos(x[2]), s=Math.sin(x[2]), R=[[c,s],[-s,c]], e=R2(-x[2],[-d[1],d[0]]);
    for(let a=0;a<2;a++){ const row=new Array(13).fill(0); row[IX[i]]=-R[a][0]; row[IX[i]+1]=-R[a][1]; row[IX[i]+2]=-e[a]; row[k]=R[a][0]; row[k+1]=R[a][1]; L.push(row); } };
  const odo=(L,P,i,j)=>{ rel(L,P,i,P.x[j],IX[j]); const row=new Array(13).fill(0); row[IX[i]+2]=-1; row[IX[j]+2]=1; L.push(row); };
  const Jo=[], Jn=[];
  odo(Jo,FA,0,1); [0,1].forEach(i=>[0,1].forEach(j=>rel(Jo,FA,i,FA.l[j],IL[j])));          // old factors, linearized at A
  odo(Jn,C,1,2); [0,1].forEach(j=>rel(Jn,C,2,C.l[j],IL[j]));                               // new factors, linearized at B (or at A where the prior reaches)
  const Lo=gram(Jo), Ln=gram(Jn), keep=[3,4,5,6,7,8,9,10,11,12], m=[0,1,2], Lmm=M.inv(m.map(a=>m.map(b=>Lo[a][b]))), Lrm=keep.map(a=>m.map(b=>Lo[a][b]));
  const S=M.sub(keep.map(a=>keep.map(b=>Lo[a][b])),M.mul(M.mul(Lrm,Lmm),M.T(Lrm)));       // Schur complement: the prior that x0 leaves on x1, l1, l2
  const Lam=S.map((row,a)=>row.map((v,b)=>v+Ln[keep[a]][keep[b]]));
  return {B,C,val:M.eigSym(Lam.map((row,a)=>row.map((v,b)=>(v+Lam[b][a])/2))).val}; }
VIZ.w17_fej=el=>{ let del=0.15, fej=false, seed=1;
  const scene=sv("svg",{viewBox:"0 0 600 250",role:"img","aria-label":"Three robot poses and two landmarks. Dashed grey lines are the old factors folded into the prior when the first pose was marginalized; blue lines are the new factors. Amber marks show where the estimate has moved to since the old factors were linearized."});
  const spec=specSvg(), st=h("div",{class:"stats"});
  const X=p=>(190+50*p[0]).toFixed(1), Y=p=>(140-50*p[1]).toFixed(1);
  const draw=()=>{ const {B,C,val}=fejInfo(del,fej,seed), nz=nZero(val);
    spectrum(spec,val); scene.innerHTML="";
    const ln=(a,b,col,dash)=>scene.append(sv("line",Object.assign({x1:X(a),y1:Y(a),x2:X(b),y2:Y(b),stroke:col,"stroke-width":1.5},dash?{"stroke-dasharray":"4 4"}:{})));
    ln(FA.x[0],FA.x[1],"var(--muted)",1); [0,1].forEach(i=>FA.l.forEach(l=>ln(FA.x[i],l,"var(--line2)",1)));
    ln(C.x[1],C.x[2],"var(--c5)"); C.l.forEach(l=>ln(C.x[2],l,"var(--c5)"));
    const pose=(x,col,hollow)=>{ const t=R2(x[2],[0.32,0]); scene.append(sv("line",{x1:X(x),y1:Y(x),x2:X([x[0]+t[0],x[1]+t[1]]),y2:Y([x[0]+t[0],x[1]+t[1]]),stroke:col,"stroke-width":2}),sv("circle",{cx:X(x),cy:Y(x),r:7,fill:hollow?"var(--surface)":col,stroke:col,"stroke-width":2})); };
    const lm=(l,col)=>scene.append(sv("rect",{x:X(l)-6,y:Y(l)-6,width:12,height:12,fill:col}));
    FA.l.forEach(l=>lm(l,"var(--ink)")); B.l.forEach(l=>lm(l,"var(--c2)")); pose(FA.x[0],"var(--muted)",true); pose(FA.x[1],"var(--ink)"); pose(B.x[1],"var(--c2)"); pose(B.x[2],"var(--c2)");
    scene.append(txt(+X(FA.x[0])-12,+Y(FA.x[0])+26,"x0, marginalized",{"text-anchor":"end",class:"mut","font-size":12}));
    st.innerHTML=""; st.append(stat("Zero eigenvalues",String(nz),nz===3?"x, y and yaw stay unobservable":"yaw has become “observable”"),
      stat("Third-smallest eigenvalue",nz>=3?"0":sig(val[2]),nz>=3?"no information about global yaw":"information that no measurement supplied"),
      stat("New factors linearize x1, l1, l2",fej?"at estimate A":"at estimate B",fej?"the same point as the prior":"the prior used estimate A")); };
  const fb=button("First-estimate Jacobians",()=>{ fej=!fej; fb.classList.toggle("on",fej); draw(); });
  el.append(h("div",{class:"vt"},"Two linearization points for one variable"),h("p",{class:"vd"},"A sliding window after x0 has been marginalized. The information matrix of the window is the prior (old factors, linearized at estimate A) plus the new factors, with unit weights."),
    h("div",{class:"ctl"},range("How far the estimate has moved since then (A → B)",0,0.5,0.01,del,v=>{ del=v; draw(); },v=>v.toFixed(2)+" m, rad").el),h("div",{class:"ctl"},fb,button("New perturbation",()=>{ seed++; draw(); })),
    scene,h("div",{class:"legend"},...[["var(--ink)","estimate A (when the prior was built)"],["var(--c2)","estimate B (now)"],["var(--line2)","old factors, inside the prior"],["var(--c5)","new factors"]].map(([c,t])=>h("span",{},h("i",{style:"background:"+c}),t))),spec,st);
  draw();
};
