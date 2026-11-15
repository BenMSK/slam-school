// Week 18: leg odometry. One planar two-link leg is shared by every widget.
// Base frame: x forward, z up, hip at the origin. q1 = hip angle from straight down (positive swings the leg forward),
// q2 = knee angle relative to the thigh (negative bends the knee forward, like a human knee).
const L1=0.4, L2=0.4, D2R=Math.PI/180;
const fkLeg=(q1,q2)=>[L1*Math.sin(q1)+L2*Math.sin(q1+q2), -L1*Math.cos(q1)-L2*Math.cos(q1+q2)];                 // f_p(q): foot position in the base frame
const jacLeg=(q1,q2)=>[[L1*Math.cos(q1)+L2*Math.cos(q1+q2), L2*Math.cos(q1+q2)],[L1*Math.sin(q1)+L2*Math.sin(q1+q2), L2*Math.sin(q1+q2)]];   // J_v(q) = d f_p / d q
const ikLeg=(x,z)=>{ const c=Math.max(-1,Math.min(1,(x*x+z*z-L1*L1-L2*L2)/(2*L1*L2))), q2=-Math.acos(c); return [Math.atan2(x,-z)-Math.atan2(L2*Math.sin(q2),L1+L2*Math.cos(q2)), q2]; };
const f1=v=>v.toFixed(1), f2=v=>v.toFixed(2), f3=v=>v.toFixed(3), sgn=v=>(v<0?"−":"")+Math.abs(v).toFixed(3);
const arrow=(x1,y1,x2,y2,col,w=2.5)=>{ const g=sv("g",{}), a=Math.atan2(y2-y1,x2-x1); if(Math.hypot(x2-x1,y2-y1)<3) return g;
  const hx=x2-9*Math.cos(a), hy=y2-9*Math.sin(a), px=4.5*Math.sin(a), py=4.5*Math.cos(a);
  g.append(sv("line",{x1:f1(x1),y1:f1(y1),x2:f1(hx),y2:f1(hy),stroke:col,"stroke-width":w}),
    sv("path",{d:"M"+f1(x2)+" "+f1(y2)+" L"+f1(hx-px)+" "+f1(hy+py)+" L"+f1(hx+px)+" "+f1(hy-py)+" z",fill:col})); return g; };
// draw one leg with its hip at screen (hx,hy), sc pixels per meter; returns the screen position of the foot
const legG=(svg,hx,hy,sc,q,col,w)=>{ const kx=hx+sc*L1*Math.sin(q[0]), ky=hy+sc*L1*Math.cos(q[0]), f=fkLeg(q[0],q[1]), fx=hx+sc*f[0], fy=hy-sc*f[1];
  svg.append(sv("path",{d:"M"+f1(hx)+" "+f1(hy)+" L"+f1(kx)+" "+f1(ky)+" L"+f1(fx)+" "+f1(fy),fill:"none",stroke:col,"stroke-width":w,"stroke-linecap":"round","stroke-linejoin":"round"}),
    sv("circle",{cx:f1(kx),cy:f1(ky),r:w*0.75,fill:"var(--surface)",stroke:col,"stroke-width":2})); return [fx,fy]; };
const legend=items=>h("div",{class:"legend"},...items.map(([c,t])=>h("span",{},h("i",{style:"background:"+c}),t)));

// Day 2: forward kinematics and the Jacobian of the leg
VIZ.w18_leg=el=>{
  let q1=30, q2=-60, d1=-1, d2=0; const W=600, H=310, hx=300, hy=62, sc=250, vs=60;
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"A two-link leg hanging from the robot base, with arrows for the foot velocity each joint produces, the total foot velocity, and the body velocity that results when the foot is in stance"});
  const st=h("div",{class:"stats"}), jm=h("div",{class:"panel"});
  const draw=()=>{ const a=q1*D2R, b=q2*D2R, f=fkLeg(a,b), J=jacLeg(a,b), v=[J[0][0]*d1+J[0][1]*d2,J[1][0]*d1+J[1][1]*d2], det=J[0][0]*J[1][1]-J[0][1]*J[1][0];
    svg.innerHTML="";
    svg.append(sv("rect",{x:hx-80,y:hy-44,width:160,height:44,rx:10,fill:"var(--soft)",stroke:"var(--line2)"}),txt(hx-70,hy-26,"base",{class:"mut","font-size":12}),
      arrow(hx,hy,hx+124,hy,"var(--muted)",1.5),txt(hx+130,hy+4,"x",{class:"mut","font-size":12}),arrow(hx,hy,hx,hy-58,"var(--muted)",1.5),txt(hx+7,hy-50,"z",{class:"mut","font-size":12}));
    const p=legG(svg,hx,hy,sc,[a,b],"var(--ink)",7);
    svg.append(sv("circle",{cx:hx,cy:hy,r:6,fill:"var(--ink)"}),
      arrow(p[0],p[1],p[0]+vs*J[0][0],p[1]-vs*J[1][0],"var(--c5)",2),arrow(p[0],p[1],p[0]+vs*J[0][1],p[1]-vs*J[1][1],"var(--c4)",2),
      arrow(p[0],p[1],p[0]+vs*v[0],p[1]-vs*v[1],"var(--c3)",3),arrow(hx,hy,hx-vs*v[0],hy+vs*v[1],"var(--c1)",3),
      sv("circle",{cx:f1(p[0]),cy:f1(p[1]),r:5,fill:"var(--c3)",stroke:"var(--surface)","stroke-width":2}));
    jm.innerHTML=""; jm.append(tex("\\mathbf{J}_v(\\mathbf{q})=\\begin{bmatrix}"+f3(J[0][0])+" & "+f3(J[0][1])+"\\\\ "+f3(J[1][0])+" & "+f3(J[1][1])+"\\end{bmatrix}\\ \\text{m/rad},\\qquad \\det\\mathbf{J}_v=l_1l_2\\sin q_2="+f3(det),true));
    st.innerHTML=""; st.append(stat("Foot position in the base frame","("+sgn(f[0])+", "+sgn(f[1])+")","fₚ(q) as (x, z), in m"),
      stat("Foot velocity seen from the base","("+sgn(v[0])+", "+sgn(v[1])+")","Jᵥ times the joint rates, in m/s"),
      stat("Body velocity if this foot is in stance","("+sgn(-v[0])+", "+sgn(-v[1])+")","minus the foot velocity, in m/s; body not rotating"),
      stat("Singular?",Math.abs(det)<0.015?"yes":"no",Math.abs(det)<0.015?"straight leg: the two columns are parallel":"|det| = "+f3(Math.abs(det)))); };
  el.append(h("div",{class:"vt"},"A two-link leg: forward kinematics and Jacobian"),h("p",{class:"vd"},"Thigh and shank are 0.4 m each. The widget evaluates fₚ(q) and Jᵥ(q) for the angles you set, then multiplies by the joint rates."),
    h("div",{class:"ctl"},range("Hip angle q₁",-60,60,1,q1,v=>{ q1=v; draw(); },v=>v+"°").el,range("Knee angle q₂",-120,0,1,q2,v=>{ q2=v; draw(); },v=>v+"°").el),
    h("div",{class:"ctl"},range("Hip rate",-2,2,0.1,d1,v=>{ d1=v; draw(); },v=>f1(v)+" rad/s").el,range("Knee rate",-2,2,0.1,d2,v=>{ d2=v; draw(); },v=>f1(v)+" rad/s").el),
    svg,legend([["var(--c5)","column 1 of Jᵥ: hip alone at 1 rad/s"],["var(--c4)","column 2: knee alone at 1 rad/s"],["var(--c3)","foot velocity"],["var(--c1)","body velocity in stance"]]),jm,st);
  draw();
};

// Day 3: a planar biped walks; the body position is dead-reckoned from the stance-foot constraint
const NS=12, SPS=20, LS=0.3, HH=0.65, LIFT=0.06;                    // footsteps, samples per step, step length, hip height, swing-foot lift (m)
const simWalk=(sigDeg,slip,seed)=>{ const r=rng(seed), sig=sigDeg*D2R, out=[]; let cT=LS/2, cE=null;      // cT: where the stance foot truly touched down; cE: where the estimator believes it is
  for(let i=0;i<=NS*SPS;i++){ const n=i?Math.floor((i-1)/SPS):0, u=(i-n*SPS)/SPS, s=n%2, q=[null,null];
    q[s]=ikLeg(LS/2-LS*u,-HH); q[1-s]=ikLeg(-LS/2+LS*u,-HH+LIFT*Math.sin(Math.PI*u));                 // the gait: stance foot sweeps back, swing foot arcs forward
    const qm=q.map(a=>[a[0]+sig*r.n(),a[1]+sig*r.n()]);                                                 // encoder readings (12.26)
    const foot=cT-slip*LS*u, tb=[foot-(LS/2-LS*u),HH], fS=fkLeg(qm[s][0],qm[s][1]);                     // truth: the stance foot slides back by slip*LS over the step
    if(!cE) cE=[tb[0]+fS[0],tb[1]+fS[1]];                                                               // the start pose is known
    const eb=[cE[0]-fS[0],cE[1]-fS[1]];                                                                 // (12.17) with R = I: t = c - f_p(q)
    out.push({tb,eb,q,s,foot,ce:cE[0]});
    if(u===1&&n<NS-1){ const fN=fkLeg(qm[1-s][0],qm[1-s][1]); cE=[eb[0]+fN[0],eb[1]+fN[1]]; cT=tb[0]+LS/2; } }   // touchdown: the other foot becomes the fixed point
  return out; };
VIZ.w18_walk=el=>{
  let sig=0.3, slip=0, seed=4, frame=NS*SPS, raf=0, D=simWalk(sig,slip,seed);
  const W=600, H=212, gy=178, X=x=>36+x*134, hy=gy-HH*134;
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Side view of a two-legged robot walking to the right, with its true body position and the body position computed by leg odometry"});
  const st=h("div",{class:"stats"}), ch=h("div",{});
  const draw=()=>{ const d=D[frame], ex=D.map(o=>100*(o.eb[0]-o.tb[0])), ec=D.map(o=>100*(o.ce-o.foot)), bx=X(d.tb[0]);
    svg.innerHTML=""; svg.append(sv("line",{x1:0,y1:gy,x2:W,y2:gy,stroke:"var(--line2)","stroke-width":2}));
    for(let m=0;m<=4;m++) svg.append(sv("line",{x1:X(m),y1:gy,x2:X(m),y2:gy+7,stroke:"var(--line2)","stroke-width":1.5}),txt(X(m),gy+22,m+" m",{"text-anchor":"middle",class:"mut","font-size":11}));
    svg.append(sv("line",{x1:X(0),y1:hy,x2:f1(bx),y2:hy,stroke:"var(--line2)","stroke-width":2,"stroke-dasharray":"5 4"}),
      sv("path",{d:D.slice(0,frame+1).map((o,i)=>(i?"L":"M")+f1(X(o.eb[0]))+" "+f1(gy-o.eb[1]*134)).join(" "),fill:"none",stroke:"var(--c3)","stroke-width":2}),
      sv("rect",{x:f1(bx-13),y:f1(hy-58),width:26,height:64,rx:9,fill:"var(--soft)",stroke:"var(--line2)"}));
    legG(svg,bx,hy,134,d.q[1-d.s],"var(--muted)",5); legG(svg,bx,hy,134,d.q[d.s],"var(--c5)",5);
    svg.append(sv("circle",{cx:f1(bx),cy:hy,r:4,fill:"var(--ink)"}),
      sv("circle",{cx:f1(X(d.eb[0])),cy:f1(gy-d.eb[1]*134),r:7,fill:"none",stroke:"var(--c3)","stroke-width":2.5}),
      sv("path",{d:"M"+f1(X(d.ce))+" "+(gy+2)+" l-6 10 l12 0 z",fill:"var(--c3)"}));
    const m=[1,2,5,10,20,50,100,200].find(v=>v>=1.15*Math.max(0.5,...ex.map(Math.abs),...ec.map(Math.abs)))||500, e=ex[frame], dist=d.tb[0];
    ch.innerHTML=""; ch.append(lineChart({w:600,h:230,aria:"Error of the leg-odometry body position and of the assumed contact point, in centimeters, against the number of footsteps",
      x:{min:0,max:NS,ticks:[0,2,4,6,8,10,12].map(v=>[v,""+v]),label:"footsteps taken"},y:{min:-m,max:m,ticks:[-m,-m/2,0,m/2,m].map(v=>[v,""+v]),label:"error in x (cm)"},
      series:[{name:"assumed contact point minus true stance foot",color:"var(--c5)",dots:false,points:ec.map((y,i)=>({x:i/SPS,y}))},{name:"estimated minus true body position",color:"var(--c3)",dots:false,points:ex.map((y,i)=>({x:i/SPS,y}))}],marker:{x:frame/SPS,y:e}}));
    st.innerHTML=""; st.append(stat("True distance walked",f2(dist)+" m",slip?"each step is "+f1(100*LS*(1-slip))+" cm":"each step is 30 cm"),stat("Leg odometry says",f2(d.eb[0])+" m","from the joint angles alone"),
      stat("Error in x",f1(e)+" cm",dist>0.05?f1(100*Math.abs(e)/(100*dist))+"% of the distance":"—"),stat("Error in z",f1(100*(d.eb[1]-d.tb[1]))+" cm","body height")); };
  const tm=range("Time",0,NS*SPS,1,frame,v=>{ frame=v; draw(); },v=>"footstep "+f1(v/SPS));
  const setFrame=v=>{ tm.input.value=v; tm.input.dispatchEvent(new Event("input")); };
  const stop=()=>{ cancelAnimationFrame(raf); raf=0; pb.textContent="Play"; };
  const tick=()=>{ if(frame>=NS*SPS){ stop(); return; } setFrame(Math.min(NS*SPS,frame+2)); raf=requestAnimationFrame(tick); };
  const pb=button("Play",()=>{ if(raf){ stop(); return; } if(frame>=NS*SPS) setFrame(0); pb.textContent="Pause"; raf=requestAnimationFrame(tick); },"primary");
  const redo=()=>{ D=simWalk(sig,slip,seed); draw(); };
  el.append(h("div",{class:"vt"},"Walking on leg odometry"),h("p",{class:"vd"},"A planar biped takes 12 steps of 30 cm with its body upright. The estimator sees only the four encoder readings and assumes the stance foot does not move."),
    h("div",{class:"ctl"},range("Encoder noise (1σ per reading)",0,2,0.1,sig,v=>{ sig=v; redo(); },v=>f1(v)+"°").el,range("Slip: stance foot slides back",0,30,1,0,v=>{ slip=v/100; redo(); },v=>v+"% of a step").el),
    h("div",{class:"ctl"},tm.el,pb,button("New noise",()=>{ seed++; redo(); })),svg,
    legend([["var(--c5)","stance leg"],["var(--muted)","swing leg"],["var(--line2)","true body path"],["var(--c3)","leg-odometry body (ring) and assumed contact point (triangle)"]]),ch,st);
  draw();
};

// Day 4: contact detection by thresholding the normal force, checked against the friction cone (12.22)
VIZ.w18_contact=el=>{
  let thr=100, mu=0.6, sig=10, seed=5; const N=300, F=200, A=60, W=600, H=310, x0=46, x1=586, yT=16, yB=216, YM=260;
  const X=i=>x0+i/(N-1)*(x1-x0), Y=v=>yB-Math.max(0,Math.min(YM,v))/YM*(yB-yT);
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Measured normal force of one foot over three strides, the normal force needed to prevent slipping, the detection threshold, and strips showing when the foot is truly not slipping and when the detector reports contact"});
  const st=h("div",{class:"stats"});
  const strip=(flags,y,col)=>{ for(let i=0;i<N;i++) if(flags[i]){ let j=i; while(j+1<N&&flags[j+1]) j++; svg.append(sv("rect",{x:f1(X(i-0.5)),y,width:f1(X(j+0.5)-X(i-0.5)),height:11,fill:col})); i=j; } };
  const draw=()=>{ const r=rng(seed), meas=[], need=[], valid=[], det=[];
    for(let i=0;i<N;i++){ const k=i%100, on=k<55, u=k/54, z=on?F*Math.min(1,u/0.2,(1-u)/0.2):0, fx=on?A*Math.abs(Math.cos(Math.PI*u)):0;   // normal force, and the tangential force the gait needs
      meas.push(z+sig*r.n()); need.push(on?fx/mu:null); valid.push(z>0&&fx<=mu*z); det.push(meas[i]>thr); }
    svg.innerHTML=""; [0,100,200].forEach(v=>svg.append(sv("line",{x1:x0,y1:Y(v),x2:x1,y2:Y(v),stroke:"var(--line)","stroke-width":1}),txt(x0-6,Y(v)+4,v+(v?"":" N"),{"text-anchor":"end",class:"mut","font-size":11})));
    let dn=""; need.forEach((v,i)=>{ if(v!=null) dn+=(need[i-1]==null?"M":"L")+f1(X(i))+" "+f1(Y(v))+" "; });
    svg.append(sv("path",{d:dn,fill:"none",stroke:"var(--c2)","stroke-width":2,"stroke-dasharray":"5 3"}),
      sv("path",{d:meas.map((v,i)=>(i?"L":"M")+f1(X(i))+" "+f1(Y(v))).join(" "),fill:"none",stroke:"var(--c5)","stroke-width":2,"stroke-linejoin":"round"}),
      sv("line",{x1:x0,y1:Y(thr),x2:x1,y2:Y(thr),stroke:"var(--c3)","stroke-width":2}));
    strip(valid,234,"var(--c1)"); strip(det,252,"var(--c5)"); strip(det.map((v,i)=>v!==valid[i]),270,"var(--c3)");
    svg.append(txt(x0,H-8,"time: three strides (stance, then swing)",{class:"mut","font-size":11}));
    const nd=det.filter(Boolean).length, nv=valid.filter(Boolean).length, bad=det.filter((v,i)=>v&&!valid[i]).length, lost=valid.filter((v,i)=>v&&!det[i]).length, flips=det.filter((v,i)=>i&&v!==det[i-1]).length;
    st.innerHTML=""; st.append(stat("Contact claimed while slipping or in the air",bad+" of "+nd+" samples",nd?Math.round(100*bad/nd)+"% of what leg odometry would use":"nothing detected"),
      stat("Good stance thrown away",lost+" of "+nv+" samples",nv?Math.round(100*lost/nv)+"% of the usable stance":"the foot always slips"),
      stat("Contact switches",""+flips,"6 would be one touchdown and one lift-off per stride")); };
  el.append(h("div",{class:"vt"},"Contact detection by thresholding the normal force"),h("p",{class:"vd"},"One foot of a trotting quadruped: the normal force ramps up to 200 N in stance and is zero in swing. The foot holds only while the friction cone |fₓ| ≤ μ f_z is satisfied."),
    h("div",{class:"ctl"},range("Threshold on f_z",0,220,5,thr,v=>{ thr=v; draw(); },v=>v+" N").el,range("Friction coefficient μ",0.2,1,0.05,mu,v=>{ mu=v; draw(); },v=>f2(v)).el,range("Force sensor noise (1σ)",0,40,1,sig,v=>{ sig=v; draw(); },v=>v+" N").el),
    h("div",{class:"ctl"},button("New noise",()=>{ seed++; draw(); })),svg,
    legend([["var(--c5)","measured f_z (line) and detected contact (strip)"],["var(--c2)","f_z needed to hold, |fₓ|/μ"],["var(--c3)","threshold (line) and wrong decisions (strip)"],["var(--c1)","foot truly not slipping"]]),st);
  draw();
};

// Day 5: encoder noise pushed through the forward kinematics, first order (12.27) against sampling
VIZ.w18_cov=el=>{
  let q1=30, q2=-60, sig=0.5, seed=2; const NSMP=300, W=600, H=320, cx=440, cy=160, hx=130, hy=40, sc=250;
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Left: the two-link leg. Right: a magnified cloud of foot positions computed from noisy joint angles, with the two-sigma ellipse predicted by the Jacobian"});
  const st=h("div",{class:"stats"});
  const draw=()=>{ const zs=sig<=0.7?5000:2500, a=q1*D2R, b=q2*D2R, s=sig*D2R, f=fkLeg(a,b), J=jacLeg(a,b), S=M.scale(M.mul(J,M.T(J)),s*s), r=rng(seed), ev=M.eigSym(S).val;
    svg.innerHTML=""; svg.append(sv("rect",{x:hx-60,y:hy-30,width:120,height:30,rx:8,fill:"var(--soft)",stroke:"var(--line2)"}),txt(hx-50,hy-10,"base",{class:"mut","font-size":12}));
    const p=legG(svg,hx,hy,sc,[a,b],"var(--ink)",6);
    svg.append(sv("circle",{cx:f1(p[0]),cy:f1(p[1]),r:5,fill:"var(--c3)"}),sv("rect",{x:cx-150,y:cy-150,width:300,height:300,rx:10,fill:"var(--surface)",stroke:"var(--line2)"}),
      sv("line",{x1:cx-150,y1:cy,x2:cx+150,y2:cy,stroke:"var(--line)"}),sv("line",{x1:cx,y1:cy-120,x2:cx,y2:cy+150,stroke:"var(--line)"}));
    let sx=0, sz=0, mx=0, mz=0; const P=[]; for(let n=0;n<NSMP;n++){ const g=fkLeg(a+s*r.n(),b+s*r.n()); P.push([g[0]-f[0],g[1]-f[1]]); mx+=P[n][0]/NSMP; mz+=P[n][1]/NSMP; }
    P.forEach(d=>{ sx+=(d[0]-mx)**2/(NSMP-1); sz+=(d[1]-mz)**2/(NSMP-1); const px=cx+zs*d[0], py=cy-zs*d[1]; if(Math.abs(px-cx)<146&&Math.abs(py-cy)<146) svg.append(sv("circle",{cx:f1(px),cy:f1(py),r:2,fill:"var(--c5)",opacity:0.55})); });
    svg.append(covEllipse([[S[0][0],-S[0][1]],[-S[0][1],S[1][1]]],cx,cy,2,zs,{stroke:"var(--c3)","stroke-width":2.5}),
      sv("line",{x1:cx-140,y1:cy+136,x2:cx-90,y2:cy+136,stroke:"var(--ink)","stroke-width":2}),txt(cx-84,cy+140,zs>3000?"1 cm":"2 cm",{"font-size":11}),txt(cx-140,cy-134,"foot position error, magnified",{class:"mut","font-size":11}));
    st.innerHTML=""; st.append(stat("Predicted 1σ","x "+f1(1000*Math.sqrt(S[0][0]))+" mm, z "+f1(1000*Math.sqrt(S[1][1]))+" mm","from Jᵥ Σq Jᵥᵀ, first order (12.27)"),
      stat("Measured 1σ, "+NSMP+" samples","x "+f1(1000*Math.sqrt(sx))+" mm, z "+f1(1000*Math.sqrt(sz))+" mm","through the exact fₚ(q)"),
      stat("Ellipse axes (1σ)",f1(1000*Math.sqrt(Math.max(0,ev[1])))+" and "+f1(1000*Math.sqrt(Math.max(0,ev[0])))+" mm","long and short")); };
  el.append(h("div",{class:"vt"},"Encoder noise at the foot"),h("p",{class:"vd"},"Each joint reading gets independent Gaussian noise. Dots: the exact forward kinematics of 300 noisy readings. Ellipse: the 2σ contour of the first-order covariance."),
    h("div",{class:"ctl"},range("Hip angle q₁",-60,60,1,q1,v=>{ q1=v; draw(); },v=>v+"°").el,range("Knee angle q₂",-120,0,1,q2,v=>{ q2=v; draw(); },v=>v+"°").el,range("Encoder noise (1σ)",0.1,1.5,0.1,sig,v=>{ sig=v; draw(); },v=>f1(v)+"°").el),
    h("div",{class:"ctl"},button("New samples",()=>{ seed++; draw(); })),svg,legend([["var(--c5)","sampled foot positions"],["var(--c3)","2σ ellipse from the Jacobian"]]),st);
  draw();
};
