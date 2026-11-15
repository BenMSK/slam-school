// Week 16: a planar IMU. World: x to the right, z up, gravity (0, -9.81). Pitch θ is counter-clockwise and R(θ) maps
// body to world. One gyro (pitch rate) and a two-axis accelerometer (body x, body z). The measurement model is
// equations (11.1)-(11.2), integration is (11.12), the increments are (11.14), all restricted to the plane.
const G16=9.81, RAD16=Math.PI/180;
const rot16=(t,v)=>{ const c=Math.cos(t), s=Math.sin(t); return [c*v[0]-s*v[1], s*v[0]+c*v[1]]; };      // R(t) v
const arrow16=(x1,y1,x2,y2,col,wd)=>{ const a=Math.atan2(y2-y1,x2-x1), L=Math.hypot(x2-x1,y2-y1), s=Math.min(10,L*0.6), g=sv("g",{});
  const P=d=>(x2-s*Math.cos(a+d)).toFixed(1)+" "+(y2-s*Math.sin(a+d)).toFixed(1);
  g.append(sv("line",{x1,y1,x2,y2,stroke:col,"stroke-width":wd||2.5,"stroke-linecap":"round"}));
  if(L>3) g.append(sv("path",{d:"M"+x2.toFixed(1)+" "+y2.toFixed(1)+" L"+P(0.4)+" L"+P(-0.4)+" z",fill:col}));
  return g; };
const leg16=items=>h("div",{class:"legend"},...items.map(([c,t])=>h("span",{},h("i",{style:"background:"+c}),t)));
const set16=(s,v)=>{ s.input.value=v; s.input.dispatchEvent(new Event("input")); };
const ticks16=(lo,hi)=>{ const raw=Math.max(hi-lo,1e-9)/4, p=10**Math.floor(Math.log10(raw)), st=(raw/p>=5?5:raw/p>=2?2:1)*p, out=[];
  for(let v=Math.ceil(lo/st-1e-9)*st; v<=hi+1e-9; v+=st) out.push([v,String(+v.toFixed(6))]); return out; };
const pair16=(v,n)=>"("+v[0].toFixed(n)+", "+v[1].toFixed(n)+")";

// Days 1 and 2: what a two-axis accelerometer reads on a tilted, accelerating body.
VIZ.w16_tilt=el=>{
  let th=20, ax=0, az=0; const cx=170, cy=185, bx=460, sc=9;
  const svg=sv("svg",{viewBox:"0 0 600 300",role:"img","aria-label":"Left, world frame: a tilted body with three arrows from its centre, the acceleration, minus gravity pointing up, and their sum, the specific force. Right, body frame: the same specific force drawn against the body axes, which is the accelerometer reading."});
  const st=h("div",{class:"stats"});
  const draw=()=>{ const t=th*RAD16, f=[ax,az+G16], m=rot16(-t,f), n=Math.hypot(m[0],m[1]), xb=rot16(t,[1,0]), zb=rot16(t,[0,1]);
    svg.innerHTML="";
    svg.append(txt(cx,20,"World frame",{"text-anchor":"middle",class:"mut"}),txt(bx,20,"Body frame: the reading",{"text-anchor":"middle",class:"mut"}),
      sv("line",{x1:330,y1:10,x2:330,y2:290,stroke:"var(--line)"}),
      sv("rect",{x:cx-60,y:cy-16,width:120,height:32,rx:6,fill:"var(--soft)",stroke:"var(--line2)",transform:"rotate("+(-th)+" "+cx+" "+cy+")"}),
      arrow16(cx,cy,cx+78*xb[0],cy-78*xb[1],"var(--muted)",1.5),arrow16(cx,cy,cx+78*zb[0],cy-78*zb[1],"var(--muted)",1.5),
      txt(cx+100*xb[0],cy-100*xb[1]+4,"x body",{"text-anchor":"middle",class:"mut","font-size":11}),txt(cx+100*zb[0],cy-100*zb[1]+4,"z body",{"text-anchor":"middle",class:"mut","font-size":11}),
      arrow16(cx,cy,cx+f[0]*sc,cy-f[1]*sc,"var(--c3)",6),arrow16(cx,cy,cx,cy-G16*sc,"var(--c2)",2),arrow16(cx,cy,cx+ax*sc,cy-az*sc,"var(--c5)",2),
      arrow16(bx,cy,bx+90,cy,"var(--muted)",1.5),arrow16(bx,cy,bx,cy-150,"var(--muted)",1.5),
      txt(bx+94,cy+4,"x body",{class:"mut","font-size":11}),txt(bx-9,cy-138,"z body",{"text-anchor":"end",class:"mut","font-size":11}),
      arrow16(bx,cy,bx+m[0]*sc,cy-m[1]*sc,"var(--c3)",4),sv("circle",{cx,cy,r:4,fill:"var(--ink)"}),sv("circle",{cx:bx,cy,r:4,fill:"var(--ink)"}));
    st.innerHTML=""; st.append(stat("Accelerometer reads (x, z)",pair16(m,2)+" m/s²","Rᵀ(a − g), equation (11.1) without bias and noise"),
      stat("Length of the reading",n.toFixed(2)+" m/s²","9.81 only when the acceleration is zero"),
      stat("Pitch you would infer assuming rest",n<0.05?"undefined":(Math.atan2(m[0],m[1])/RAD16).toFixed(1)+"°","true pitch "+th+"°")); };
  const sx=range("Horizontal acceleration",-5,5,0.5,ax,v=>{ ax=v; draw(); },v=>v.toFixed(1)+" m/s²"), sz=range("Vertical acceleration",-9.81,5,0.01,az,v=>{ az=v; draw(); },v=>v.toFixed(2)+" m/s²");
  el.append(h("div",{class:"vt"},"What the accelerometer reads"),h("p",{class:"vd"},"A body in the vertical plane with a two-axis accelerometer. The reading is the specific force a − g, expressed in the body's own axes."),
    h("div",{class:"ctl"},range("Pitch",-45,45,1,th,v=>{ th=v; draw(); },v=>v+"°").el,sx.el,sz.el),
    h("div",{class:"ctl"},button("At rest",()=>{ set16(sx,0); set16(sz,0); }),button("Free fall",()=>{ set16(sx,0); set16(sz,-9.81); })),
    svg,leg16([["var(--c5)","acceleration a"],["var(--c2)","−g (up, 9.81)"],["var(--c3)","specific force a − g"]]),st);
  draw();
};

// Day 2: Allan deviation of a simulated gyro at rest: white noise of density N plus a bias random walk of strength K.
VIZ.w16_allan=el=>{
  let N=0.02, K=0.01, seed=14; const dt=0.01, n=60000, box=h("div"), st=h("div",{class:"stats"});
  const draw=()=>{ const r=rng(seed), y=new Float64Array(n), meas=[], model=[]; let b=0, s1=0;
    for(let k=0;k<n;k++){ y[k]=b+N/Math.sqrt(dt)*r.n(); b+=K*Math.sqrt(dt)*r.n(); }     // discrete noise: Cov/Δt per sample; bias step: Δt·Cov
    [1,2,5,10,20,50,100,200,500,1000,2000,5000].forEach(m=>{ const c=Math.floor(n/m); let prev=0, s=0;
      for(let j=0;j<c;j++){ let a=0; for(let k=j*m;k<(j+1)*m;k++) a+=y[k]; a/=m; if(j) s+=(a-prev)*(a-prev); prev=a; }
      const tau=m*dt, ad=Math.sqrt(s/(2*(c-1))); if(m===100) s1=ad;
      meas.push({x:tau,y:Math.max(-3.5,Math.min(0,Math.log10(ad))),label:"τ = "+tau+" s",fmt:v=>(10**v).toPrecision(2)+" °/s"});
      model.push({x:tau,y:Math.max(-3.5,Math.min(0,0.5*Math.log10(N*N/tau+K*K*tau/3)))}); });
    box.innerHTML=""; box.append(lineChart({h:290,aria:"Log-log plot of Allan deviation against averaging time. It falls with slope minus one half while white noise dominates and rises with slope plus one half once the bias random walk dominates.",
      x:{min:0.01,max:60,log:true,ticks:[[0.01,"0.01"],[0.1,"0.1"],[1,"1"],[10,"10"]],label:"averaging time τ (s), log scale"},
      y:{min:-3.5,max:0,ticks:[[0,"1"],[-1,"0.1"],[-2,"0.01"],[-3,"0.001"]],label:"Allan deviation (°/s)"},
      series:[{name:"measured from the simulated record",color:"var(--c3)",points:meas},{name:"model √(N²/τ + K²τ/3)",color:"var(--c5)",dots:false,points:model}]}));
    st.innerHTML=""; st.append(stat("Allan deviation at τ = 1 s",s1.toFixed(4)+" °/s","the white-noise line predicts N = "+N.toFixed(3)),
      stat("Model curve bottoms out at",K>0?(Math.sqrt(3)*N/K).toFixed(1)+" s":"never","τ = √3·N/K"),
      stat("Record","10 min at 100 Hz","only 12 averages fit at τ = 50 s")); };
  el.append(h("div",{class:"vt"},"Allan deviation of a gyro at rest"),h("p",{class:"vd"},"Simulates 60,000 samples of white noise plus a random-walking bias, averages them in blocks of length τ, and plots how much neighbouring block averages differ."),
    h("div",{class:"ctl"},range("White noise density N",0.005,0.05,0.005,N,v=>{ N=v; draw(); },v=>v.toFixed(3)+" °/s/√Hz").el,range("Bias random walk K",0,0.02,0.002,K,v=>{ K=v; draw(); },v=>v.toFixed(3)+" °/s/√s").el),
    h("div",{class:"ctl"},button("New noise",()=>{ seed++; draw(); })),box,st);
  draw();
};

// Day 3: dead reckoning with equation (11.12) along a known hilly path, with biases the integrator does not know about.
VIZ.w16_deadreck=el=>{
  const T=20, dt=0.01, n=2000, V=2, A=1.5, Om=Math.PI/5; let bg=0.1, ba=0, sg=0.01, sa=0.005, seed=1, avg=true;
  const svg=sv("svg",{viewBox:"0 0 600 225",role:"img","aria-label":"Side view of a vehicle driving over two hills: the true path and the path obtained by integrating the simulated gyroscope and accelerometer"});
  const box=h("div"), st=h("div",{class:"stats"});
  const truth=t=>{ const u=A*Om*Math.sin(Om*t)/V, az=A*Om*Om*Math.cos(Om*t); return {p:[V*t,A*(1-Math.cos(Om*t))], a:[0,az], th:Math.atan(u), w:az/V/(1+u*u)}; };
  const draw=()=>{ const r=rng(seed), sgd=sg*RAD16/Math.sqrt(dt), sad=sa/Math.sqrt(dt), b=bg*RAD16;
    let th=0, v=[V,0], p=[0,0]; const P=[[0,0]], Q=[[0,0]], E=[{x:0,y:0}];
    let q=truth(0);
    for(let k=0;k<n;k++){ const q1=truth((k+1)*dt), f0=rot16(-q.th,[q.a[0],q.a[1]+G16]);
      const w=(avg?(q1.th-q.th)/dt:q.w)+b+sgd*r.n(), f=[f0[0]+ba+sad*r.n(), f0[1]+sad*r.n()], Rf=rot16(th,f);          // measurements (11.1)-(11.2), then one step of (11.12)
      p=[p[0]+v[0]*dt+Rf[0]*dt*dt/2, p[1]+v[1]*dt+(Rf[1]-G16)*dt*dt/2]; v=[v[0]+Rf[0]*dt, v[1]+(Rf[1]-G16)*dt]; th+=w*dt;
      q=q1; if((k+1)%20===0){ P.push(q.p); Q.push(p); E.push({x:(k+1)*dt,y:Math.hypot(p[0]-q.p[0],p[1]-q.p[1])}); } }
    const all=P.concat(Q), x0=Math.min(...all.map(q=>q[0])), x1=Math.max(...all.map(q=>q[0])), z0=Math.min(...all.map(q=>q[1])), z1=Math.max(...all.map(q=>q[1]));
    const s=Math.min(560/(x1-x0),170/(z1-z0)), X=q=>(20+(q[0]-x0)*s).toFixed(1), Y=q=>(105+((z0+z1)/2-q[1])*s).toFixed(1);
    const path=(L,a)=>sv("path",Object.assign({d:L.map((q,i)=>(i?"L":"M")+X(q)+" "+Y(q)).join(" "),fill:"none","stroke-width":2.5,"stroke-linejoin":"round"},a));
    svg.innerHTML=""; svg.append(path(P,{stroke:"var(--line2)","stroke-dasharray":"6 5"}),path(Q,{stroke:"var(--c3)"}),
      sv("circle",{cx:X(P[0]),cy:Y(P[0]),r:5,fill:"var(--ink)"}),sv("circle",{cx:X(P[100]),cy:Y(P[100]),r:5,fill:"var(--line2)"}),sv("circle",{cx:X(Q[100]),cy:Y(Q[100]),r:5,fill:"var(--c3)"}),
      txt(X(P[100]),+Y(P[100])-10,"true end",{"text-anchor":"end",class:"mut","font-size":11}),txt(X(Q[100]),+Y(Q[100])+20,"IMU end",{"text-anchor":"middle",class:"mut","font-size":11}),
      txt(20,220,"scale: "+(100/s).toFixed(1)+" m per 100 px",{class:"mut","font-size":11}));
    const mg=t=>G16*Math.abs(b)*t*t*t/6, ma=t=>Math.abs(ba)*t*t/2, ts=E.map(e=>e.x), ym=Math.max(0.1,E[100].y,mg(T),ma(T))*1.08, cl=y=>Math.min(y,ym);
    box.innerHTML=""; box.append(lineChart({h:250,aria:"Position error of dead reckoning against time, with the cubic curve predicted for a gyroscope bias and the quadratic curve predicted for an accelerometer bias",
      x:{min:0,max:T,ticks:[[0,"0"],[5,"5"],[10,"10"],[15,"15"],[20,"20"]],label:"time (s)"},y:{min:0,max:ym,ticks:ticks16(0,ym),label:"position error (m)"},
      series:[{name:"simulated error",color:"var(--c3)",dots:false,points:E.map(e=>({x:e.x,y:cl(e.y)}))},{name:"gyro-bias model g·b·t³/6",color:"var(--c5)",dots:false,points:ts.map(t=>({x:t,y:cl(mg(t))}))},{name:"accel-bias model ½·b·t²",color:"var(--c2)",dots:false,points:ts.map(t=>({x:t,y:cl(ma(t))}))}]}));
    const tilt=th-truth(T).th;
    st.innerHTML=""; st.append(stat("Position error after 20 s",E[100].y.toFixed(2)+" m","integrated path vs true path"),
      stat("Tilt error after 20 s",(tilt/RAD16).toFixed(2)+"°","leaks g·sin(tilt) = "+(G16*Math.abs(Math.sin(tilt))).toFixed(2)+" m/s² sideways"),
      stat("Gyro bias alone predicts",mg(T).toFixed(2)+" m","g·b·t³/6"),stat("Accel bias alone predicts",ma(T).toFixed(2)+" m","½·b·t²")); };
  el.append(h("div",{class:"vt"},"Dead reckoning with a planar IMU"),h("p",{class:"vd"},"A vehicle drives 40 m over two hills in 20 s. A 100 Hz gyroscope and accelerometer are simulated and integrated with equation (11.12), starting from the exact initial state."),
    h("div",{class:"ctl"},range("Gyro bias",-0.5,0.5,0.05,bg,v=>{ bg=v; draw(); },v=>v.toFixed(2)+" °/s").el,range("Accel bias (body x)",-0.2,0.2,0.01,ba,v=>{ ba=v; draw(); },v=>v.toFixed(2)+" m/s²").el),
    h("div",{class:"ctl"},range("Gyro noise",0,0.1,0.005,sg,v=>{ sg=v; draw(); },v=>v.toFixed(3)+" °/s/√Hz").el,range("Accel noise",0,0.05,0.005,sa,v=>{ sa=v; draw(); },v=>v.toFixed(3)+" m/s²/√Hz").el),
    h("div",{class:"ctl"},select("Gyro output ",[["avg","average rate over each 10 ms"],["inst","instantaneous sample every 10 ms"]],v=>{ avg=v==="avg"; draw(); }).el,button("New noise",()=>{ seed++; draw(); })),svg,leg16([["var(--line2)","true path"],["var(--c3)","integrated IMU"]]),box,st);
  draw();
};

// Days 4 and 5 share one second of simulated IMU data between keyframes i and j: 100 samples at 100 Hz.
// True motion: starts level, pitch rate 0.35 rad/s, world acceleration (2, 1) m/s². Noise densities 0.002 rad/s/√Hz and 0.02 m/s²/√Hz.
const SGD16=0.002/Math.sqrt(0.01), SAD16=0.02/Math.sqrt(0.01);
const imu16=()=>{ const r=rng(5), N=100, dt=0.01, om=[], ac=[];
  for(let k=0;k<N;k++){ const f=rot16(-0.35*k*dt,[2,1+G16]); om.push(0.35+SGD16*r.n()); ac.push([f[0]+SAD16*r.n(), f[1]+SAD16*r.n()]); }
  return {om,ac,N,dt,T:N*dt}; };
// The increments of (11.14) with biases (bg, ba) removed, their bias Jacobians as used in (11.30), and the covariance of
// [δθ, δv, δp] from (11.28)-(11.29) in iterative form. In the plane the wedge term −f^ δφ becomes J f δθ, J = 90° rotation.
const preint16=(d,bg,ba)=>{ const dt=d.dt, Q=[[SGD16*SGD16,0,0],[0,SAD16*SAD16,0],[0,0,SAD16*SAD16]];
  let dth=0, dv=[0,0], dp=[0,0], S=M.zeros(5,5); const Jvg=[0,0], Jpg=[0,0], Jva=M.zeros(2,2), Jpa=M.zeros(2,2);
  for(let k=0;k<d.N;k++){ const c=Math.cos(dth), s=Math.sin(dth), R=[[c,-s],[s,c]], Rf=rot16(dth,[d.ac[k][0]-ba[0],d.ac[k][1]-ba[1]]), JRf=[-Rf[1],Rf[0]], tk=k*dt;
    const A=M.eye(5), B=M.zeros(5,3); A[3][1]=dt; A[4][2]=dt; B[0][0]=dt;
    for(let a=0;a<2;a++){ A[1+a][0]=JRf[a]*dt; A[3+a][0]=JRf[a]*dt*dt/2; for(let q=0;q<2;q++){ B[1+a][1+q]=R[a][q]*dt; B[3+a][1+q]=R[a][q]*dt*dt/2; } }
    S=M.add(M.mul(M.mul(A,S),M.T(A)),M.mul(M.mul(B,Q),M.T(B)));
    for(let a=0;a<2;a++){ Jpg[a]+=Jvg[a]*dt-JRf[a]*tk*dt*dt/2; for(let q=0;q<2;q++) Jpa[a][q]+=Jva[a][q]*dt-R[a][q]*dt*dt/2; }
    for(let a=0;a<2;a++){ Jvg[a]-=JRf[a]*tk*dt; for(let q=0;q<2;q++) Jva[a][q]-=R[a][q]*dt; }
    dp=[dp[0]+dv[0]*dt+Rf[0]*dt*dt/2, dp[1]+dv[1]*dt+Rf[1]*dt*dt/2]; dv=[dv[0]+Rf[0]*dt, dv[1]+Rf[1]*dt]; dth+=(d.om[k]-bg)*dt; }
  return {dth,dv,dp,Jvg,Jpg,Jva,Jpa,S}; };

// Day 4: the stored increments predict keyframe j for any state at keyframe i; naive integration has to start over.
VIZ.w16_preint=el=>{
  const d=imu16(), pre=preint16(d,0,[0,0]), T=d.T, sc=34, ox=200, oy=85; let thi=0, vx=2, vz=0, moves=0;
  const svg=sv("svg",{viewBox:"0 0 600 340",role:"img","aria-label":"The predicted position of keyframe j drawn two ways: as the curved path from naive integration, and as a chain of three arrows, initial velocity times time, the gravity drop, and the rotated preintegrated position increment. Both end at the same point."});
  const st=h("div",{class:"stats"}), X=q=>ox+q[0]*sc, Y=q=>oy-q[1]*sc, sd=i=>Math.sqrt(pre.S[i][i]);
  const naive=(t,v0)=>{ let th=t, p=[0,0], v=v0.slice(); const P=[p], dt=d.dt;
    for(let k=0;k<d.N;k++){ const Rf=rot16(th,d.ac[k]); p=[p[0]+v[0]*dt+Rf[0]*dt*dt/2, p[1]+v[1]*dt+(Rf[1]-G16)*dt*dt/2]; v=[v[0]+Rf[0]*dt, v[1]+(Rf[1]-G16)*dt]; th+=d.om[k]*dt; P.push(p); }
    return {P,p}; };
  const draw=()=>{ const t=thi*RAD16, nv=naive(t,[vx,vz]), a=[vx*T,vz*T], b=[a[0],a[1]-G16*T*T/2], Rp=rot16(t,pre.dp), pj=[b[0]+Rp[0],b[1]+Rp[1]];
    svg.innerHTML=""; svg.append(sv("path",{d:nv.P.map((q,i)=>(i?"L":"M")+X(q).toFixed(1)+" "+Y(q).toFixed(1)).join(" "),fill:"none",stroke:"var(--c3)","stroke-width":5,"stroke-linecap":"round",opacity:0.75}),
      arrow16(ox,oy,X(a),Y(a),"var(--c5)"),arrow16(X(a),Y(a),X(b),Y(b),"var(--c2)"),arrow16(X(b),Y(b),X(pj),Y(pj),"var(--c1)"),
      sv("circle",{cx:ox,cy:oy,r:5,fill:"var(--ink)"}),sv("circle",{cx:X(pj),cy:Y(pj),r:7,fill:"none",stroke:"var(--ink)","stroke-width":2}),
      txt(ox-12,oy+4,"keyframe i",{"text-anchor":"end",class:"mut","font-size":12}),txt(X(pj)+13,Y(pj)+4,"keyframe j",{class:"mut","font-size":12}));
    st.innerHTML=""; st.append(stat("Position of j, naive integration",pair16(nv.p,3)+" m","100 samples integrated again"),
      stat("Position of j, from the increments",pair16(pj,3)+" m","one line: pᵢ + vᵢΔt + ½gΔt² + RᵢΔp"),
      stat("Difference",Math.hypot(pj[0]-nv.p[0],pj[1]-nv.p[1]).toExponential(1)+" m","floating-point round-off"),
      stat("IMU samples read so far",100*(moves+1)+" vs 100","naive vs preintegrated")); };
  const mv=f=>v=>{ f(v); moves++; draw(); };
  el.append(h("div",{class:"vt"},"Preintegration: integrate once, reuse for any starting state"),h("p",{class:"vd"},"One second of IMU data (100 samples) between keyframes i and j. The sliders play the optimizer changing its estimate of the state at i."),
    h("div",{class:"ctl"},range("Orientation at i",-30,30,1,thi,mv(v=>thi=v),v=>v+"°").el,range("Velocity at i, x",0,4,0.1,vx,mv(v=>vx=v),v=>v.toFixed(1)+" m/s").el,range("Velocity at i, z",-2,2,0.1,vz,mv(v=>vz=v),v=>v.toFixed(1)+" m/s").el),
    h("div",{class:"out"},"Stored increments: Δθ = "+(pre.dth/RAD16).toFixed(2)+"°,  Δv = "+pair16(pre.dv,3)+" m/s,  Δp = "+pair16(pre.dp,3)+" m"),
    h("div",{class:"out"},"Their 1σ noise: δθ "+(sd(0)/RAD16).toFixed(3)+"°,  δv "+pair16([sd(1),sd(2)],3)+" m/s,  δp "+pair16([sd(3)*1000,sd(4)*1000],1)+" mm"),
    svg,leg16([["var(--c3)","naive integration (11.13)"],["var(--c5)","vᵢ Δt"],["var(--c2)","½ g Δt²"],["var(--c1)","Rᵢ Δp (stored increment)"]]),st);
  draw();
};

// Day 5: a bias update handled three ways: ignore it, first-order correction (11.30), or integrate again.
VIZ.w16_bias=el=>{
  const d=imu16(), base=preint16(d,0,[0,0]); let dg=1, da=0; const box=h("div"), st=h("div",{class:"stats"});
  const cmp=(g,a)=>{ const ex=preint16(d,g*RAD16,[a,0]).dp, fo=[0,1].map(i=>base.dp[i]+base.Jpg[i]*g*RAD16+base.Jpa[i][0]*a);
    return {e1:1000*Math.hypot(fo[0]-ex[0],fo[1]-ex[1]), e0:1000*Math.hypot(base.dp[0]-ex[0],base.dp[1]-ex[1])}; };
  const draw=()=>{ const pts=[]; for(let g=-5;g<=5.001;g+=0.25) pts.push({x:g,y:cmp(g,da).e1}); const c=cmp(dg,da), ym=Math.max(...pts.map(q=>q.y),0.01)*1.1;
    box.innerHTML=""; box.append(lineChart({h:240,aria:"Error of the first-order corrected position increment against the size of the gyroscope bias update. It grows like the square of the update.",
      x:{min:-5,max:5,ticks:[[-5,"−5"],[-2.5,"−2.5"],[0,"0"],[2.5,"2.5"],[5,"5"]],label:"gyro bias update δb (°/s)"},y:{min:0,max:ym,ticks:ticks16(0,ym),label:"error of corrected Δp (mm)"},
      series:[{name:"first-order correction",color:"var(--c1)",dots:false,points:pts}],marker:{x:dg,y:c.e1}}));
    st.innerHTML=""; st.append(stat("Δp error if the increments are left alone",c.e0.toFixed(1)+" mm","no samples re-read"),
      stat("Δp error after the first-order update",c.e1.toFixed(2)+" mm","no samples re-read, 2 small matrix products"),
      stat("Δp error after integrating again","0 mm","100 samples re-read")); };
  el.append(h("div",{class:"vt"},"A bias update without re-integration"),h("p",{class:"vd"},"The one-second increments were computed with bias estimate zero. The optimizer now changes the bias estimate. Compare the corrected Δp with a full re-integration."),
    h("div",{class:"ctl"},range("Gyro bias update δb",-5,5,0.1,dg,v=>{ dg=v; draw(); },v=>v.toFixed(1)+" °/s").el,range("Accel bias update δb (body x)",-0.5,0.5,0.05,da,v=>{ da=v; draw(); },v=>v.toFixed(2)+" m/s²").el),box,st);
  draw();
};

// Day 5: rectangle rule (Euler, as in (11.12)) against a piecewise-linear acceleration (trapezoid) on a known signal.
VIZ.w16_integ=el=>{
  const w1=2*Math.PI*0.9, w2=2*Math.PI*2.3, T=2; let f=10; const box=h("div"), st=h("div",{class:"stats"});
  const acc=t=>0.2*Math.cos(w1*t)+0.08*Math.sin(w2*t), vel=t=>0.2/w1*Math.sin(w1*t)-0.08/w2*Math.cos(w2*t), pos=t=>0.2/(w1*w1)*(1-Math.cos(w1*t))-0.08/(w2*w2)*Math.sin(w2*t);
  const draw=()=>{ const dt=1/f, n=Math.round(T*f), s=[0,0,0,0], E=[{x:0,y:0}], Tr=[{x:0,y:0}], Tru=[]; let ve=vel(0), pe=0, vt=vel(0), pt=0;
    for(let k=0;k<n;k++){ const a0=acc(k*dt), a1=acc((k+1)*dt), t=(k+1)*dt;
      pe+=ve*dt+a0*dt*dt/2; ve+=a0*dt;                              // acceleration held constant over the step
      pt+=vt*dt+(2*a0+a1)*dt*dt/6; vt+=(a0+a1)*dt/2;                // acceleration linear over the step, integrated exactly
      s[0]+=(ve-vel(t))**2; s[1]+=(pe-pos(t))**2; s[2]+=(vt-vel(t))**2; s[3]+=(pt-pos(t))**2; E.push({x:t,y:pe*1000}); Tr.push({x:t,y:pt*1000}); }
    for(let i=0;i<=200;i++) Tru.push({x:i*T/200,y:pos(i*T/200)*1000});
    const ys=E.concat(Tr,Tru).map(q=>q.y), lo=Math.min(...ys), hi=Math.max(...ys), pad=(hi-lo)*0.06, rm=s.map(q=>Math.sqrt(q/n)), mm=v=>(v*1000).toFixed(v<1e-4?3:2);
    box.innerHTML=""; box.append(lineChart({h:270,aria:"Position against time over two seconds: the exact curve, the rectangle-rule double integration, and the trapezoid-rule double integration",
      x:{min:0,max:T,ticks:[[0,"0"],[0.5,"0.5"],[1,"1"],[1.5,"1.5"],[2,"2"]],label:"time (s)"},y:{min:lo-pad,max:hi+pad,ticks:ticks16(lo,hi),label:"position (mm)"},
      series:[{name:"exact",color:"var(--line2)",dots:false,points:Tru},{name:"rectangle rule (Euler)",color:"var(--c3)",dots:f<=40,points:E},{name:"piecewise-linear (trapezoid)",color:"var(--c1)",dots:f<=40,points:Tr}]}));
    st.innerHTML=""; st.append(stat("Euler RMSE, velocity",mm(rm[0])+" mm/s"),stat("Euler RMSE, position",mm(rm[1])+" mm"),stat("Trapezoid RMSE, velocity",mm(rm[2])+" mm/s"),stat("Trapezoid RMSE, position",mm(rm[3])+" mm")); };
  el.append(h("div",{class:"vt"},"Integration accuracy against sampling rate"),h("p",{class:"vd"},"A known smooth acceleration is sampled, then integrated twice with known initial conditions, once with the rectangle rule and once assuming the acceleration is linear between samples."),
    h("div",{class:"ctl"},range("Sampling rate",5,100,5,f,v=>{ f=v; draw(); },v=>v+" Hz").el),box,st);
  draw();
};
