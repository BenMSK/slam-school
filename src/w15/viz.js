// Week 15 · Event-based SLAM. One ideal 1-D event camera (equation (10.1) at every pixel) shared by three widgets,
// plus an ATE/RPE calculator for Day 5.
const EDGE=3, DARK=0.1, ON="var(--c3)", OFF="var(--c5)";
const ss=u=>{ u=Math.max(0,Math.min(1,u)); return u*u*(3-2*u); };                    // smoothstep: an edge EDGE pixels wide
// linear intensity of bright bars [left, width, peak] on a dark background, as a function of the scene coordinate u
const scene=bars=>u=>bars.reduce((s,[a,w,pk])=>s+(pk-DARK)*ss((u-a)/EDGE)*ss((a+w-u)/EDGE),DARK);
// Events of pixels 0..N-1 watching I(x - v t) for t in [0,T] ms with contrast threshold C. Each pixel keeps the log
// intensity at its last event (ref) and fires when the current value is C away from it. Crossing times are linearly
// interpolated inside the 0.05 ms time step. The 1e-9 slack makes "exactly back at the old level" count as a crossing.
const simulate=(I,v,C,N,T)=>{ const ev=[], dt=0.05, n=Math.round(T/dt);
  for(let k=0;k<N;k++){ let ref=Math.log(I(k)), Lp=ref;
    for(let i=1;i<=n;i++){ const L=Math.log(I(k-v*i*dt));
      for(;;){ const p=L-ref>=C-1e-9?1:ref-L>=C-1e-9?-1:0; if(!p) break; ref+=p*C;
        ev.push({x:k,t:(i-1+(L===Lp?1:Math.min(1,Math.max(0,(ref-Lp)/(L-Lp)))))*dt,p}); }
      Lp=L; } }
  return ev; };
// all events of one polarity as a single path of round dots
const dots=(ev,p,X,Y)=>sv("path",{d:ev.filter(e=>e.p===p).map(e=>"M"+X(e).toFixed(1)+" "+Y(e).toFixed(1)+"h.01").join("")||"M0 0",fill:"none",stroke:p>0?ON:OFF,"stroke-width":3,"stroke-linecap":"round"});
const leg=items=>h("div",{class:"legend"},...items.map(([c,t])=>h("span",{},h("i",{style:"background:"+c}),t)));
const BAR=[[-14,12,1]];                                                              // one bright bar, starts left of the sensor

// Day 1: event camera next to a 30 Hz frame camera, both watching the same moving bar.
VIZ.w15_events=el=>{
  const N=60, T=100, FPS=30, W=600, H=396, x0=62, x1=560, y0=24, y1=250, f0=298, f1=372, cw=(x1-x0)/N;
  let v=0.5, C=0.3, E=15, g=1;
  const X=k=>x0+(k+0.5)*cw, Y=t=>y0+t*(y1-y0)/T;
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Top: events of a 60-pixel event camera plotted as pixel against time while a bright bar moves across. Bottom: the pixel values of three frames taken by a 30 Hz frame camera of the same scene."});
  const st=h("div",{class:"stats"});
  const draw=()=>{ const I0=scene(BAR), I=u=>g*I0(u), ev=simulate(I,v,C,N,T), nOn=ev.filter(e=>e.p>0).length;
    svg.innerHTML="";
    svg.append(txt(x0,14,"Event camera: every event at its pixel (→) and time (↓)",{class:"mut","font-size":11}),
      txt(x0,276,"Frame camera: 8-bit pixel values of its three frames",{class:"mut","font-size":11}),
      sv("rect",{x:x0,y:y0,width:x1-x0,height:y1-y0,fill:"var(--soft)",stroke:"var(--line)"}),
      sv("rect",{x:x0,y:f0,width:x1-x0,height:f1-f0,fill:"var(--soft)",stroke:"var(--line)"}));
    [0,50,100].forEach(t=>svg.append(txt(x1+5,Y(t)+4,t+" ms",{class:"mut","font-size":11})));
    [0,20,40,59].forEach(k=>svg.append(txt(X(k),f1+15,(k?"":"pixel ")+k,{class:"mut","font-size":11,"text-anchor":"middle"})));
    svg.append(txt(x0-6,f0+8,"255",{class:"mut","font-size":11,"text-anchor":"end"}),txt(x0-6,f1,"0",{class:"mut","font-size":11,"text-anchor":"end"}));
    for(let j=0;j<3;j++){ const tf=j*1000/FPS, te=Math.min(T,tf+E), S=24, c=-8+v*(tf+E/2);     // bar centre at mid-exposure
      svg.append(sv("rect",{x:x0,y:Y(tf),width:x1-x0,height:Y(te)-Y(tf),fill:"var(--c2)","fill-opacity":0.22}),
        txt(x0-6,Y(tf)+11,"frame "+(j+1),{class:"mut","font-size":11,"text-anchor":"end"}));
      // pixel value: light collected over the exposure, clipped at full well, quantized to 8 bits
      const px=k=>{ let s=0; for(let q=0;q<S;q++) s+=I(k-v*(tf+E*(q+0.5)/S)); return Math.round(255*Math.min(1,s/S)); };
      svg.append(sv("path",{d:Array.from({length:N},(_,k)=>(k?"L":"M")+X(k).toFixed(1)+" "+(f1-px(k)/255*(f1-f0)).toFixed(1)).join(" "),fill:"none",stroke:"var(--c2)","stroke-width":2,"stroke-linejoin":"round"}));
      if(c>=2&&c<=N-3) svg.append(txt(X(c),f0-5,"frame "+(j+1),{class:"mut","font-size":11,"text-anchor":"middle"})); }
    svg.append(dots(ev,1,e=>X(e.x),e=>Y(e.t)),dots(ev,-1,e=>X(e.x),e=>Y(e.t)));
    const lv=x=>Math.round(255*Math.min(1,g*x));
    st.innerHTML=""; st.append(stat("Events in 100 ms",String(ev.length),nOn+" ON, "+(ev.length-nOn)+" OFF"),
      stat("Events per edge, per pixel",String(Math.floor(Math.log(10)/C)),"⌊ln 10 / C⌋ = ⌊"+(Math.log(10)/C).toFixed(2)+"⌋"),
      stat("Blur in one frame",(v*E).toFixed(1)+" px","speed × exposure"),
      stat("Jump between frames",(v*1000/FPS).toFixed(1)+" px","speed ÷ 30 Hz"),
      stat("Bar minus background",(lv(1)-lv(DARK))+" of 255","grey levels in a sharp frame")); };
  el.append(h("div",{class:"vt"},"Event camera and frame camera, same scene"),
    h("p",{class:"vd"},"A bright bar (10 times brighter than the background, edges 3 pixels wide) moves to the right across 60 pixels for 100 ms. Each pixel applies equation (10.1)."),
    h("div",{class:"ctl"},range("Speed",0.1,2,0.1,v,x=>{ v=x; draw(); },x=>x.toFixed(1)+" px/ms").el,range("Contrast threshold C",0.1,1,0.05,C,x=>{ C=x; draw(); },x=>x.toFixed(2)).el),
    h("div",{class:"ctl"},range("Exposure time",1,33,1,E,x=>{ E=x; draw(); },x=>x+" ms").el,select("Scene light",[["1","normal"],["0.01","100 times dimmer"],["100","100 times brighter"]],x=>{ g=+x; draw(); }).el),
    svg,leg([[ON,"ON event (brighter)"],[OFF,"OFF event (darker)"],["var(--c2)","frame: exposure band (top), pixel values (bottom)"]]),st);
  draw();
};

// Day 3: three representations of the same packet of events.
VIZ.w15_repr=el=>{
  const N=60, T=100, B=4, W=600, H=306, x0=30, x1=570, cw=(x1-x0)/N, y0=22, y1=132;
  const ev=simulate(scene(BAR),0.5,0.3,N,T);
  let now=60, D=20, tau=10;
  const X=k=>x0+(k+0.5)*cw, Y=t=>y0+t*(y1-y0)/T;
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"The events of a moving bar with a time window marked, and below it the same window as an event frame, a time surface, and a four-bin voxel grid, each drawn as a row of 60 pixels."});
  const st=h("div",{class:"stats"});
  const row=(y,hh,vals,m,col)=>{ svg.append(sv("rect",{x:x0,y,width:x1-x0,height:hh,fill:"var(--soft)",stroke:"var(--line)"}));
    vals.forEach((a,k)=>{ if(Math.abs(a)>1e-3) svg.append(sv("rect",{x:(x0+k*cw).toFixed(1),y,width:cw.toFixed(1),height:hh,fill:col||(a>0?ON:OFF),"fill-opacity":Math.min(1,Math.abs(a)/m).toFixed(2)})); }); };
  const draw=()=>{ const t0=now-D, win=ev.filter(e=>e.t>t0&&e.t<=now);
    const fr=new Array(N).fill(0), on=new Array(N).fill(0), off=new Array(N).fill(0), ts=new Array(N).fill(0), vox=Array.from({length:B},()=>new Array(N).fill(0));
    win.forEach(e=>{ fr[e.x]+=e.p; (e.p>0?on:off)[e.x]++; const s=(B-1)*(e.t-t0)/D; for(let b=0;b<B;b++) vox[b][e.x]+=e.p*Math.max(0,1-Math.abs(b-s)); });
    const last=new Array(N).fill(-Infinity); ev.forEach(e=>{ if(e.t<=now&&e.t>last[e.x]) last[e.x]=e.t; });
    last.forEach((t,k)=>{ ts[k]=Math.exp(-(now-t)/tau); });
    const mf=Math.max(1,...fr.map(Math.abs)), mv=Math.max(1,...vox.flat().map(Math.abs));
    svg.innerHTML="";
    svg.append(txt(x0,14,"All events (pixel →, time ↓). The shaded band is the window that gets converted.",{class:"mut","font-size":11}),
      sv("rect",{x:x0,y:y0,width:x1-x0,height:y1-y0,fill:"var(--soft)",stroke:"var(--line)"}),
      sv("rect",{x:x0,y:Y(Math.max(0,t0)),width:x1-x0,height:Y(now)-Y(Math.max(0,t0)),fill:"var(--c2)","fill-opacity":0.25}),
      dots(ev,1,e=>X(e.x),e=>Y(e.t)),dots(ev,-1,e=>X(e.x),e=>Y(e.t)),
      sv("line",{x1:x0,x2:x1,y1:Y(now),y2:Y(now),stroke:"var(--ink)","stroke-width":1.5}),
      txt(x0,152,"Event frame: sum of polarities per pixel over the window",{class:"mut","font-size":11}),
      txt(x0,194,"Time surface: exp(−(now − time of the pixel's last event) / τ)",{class:"mut","font-size":11}),
      txt(x0,236,"Voxel grid: "+B+" time bins across the window, oldest on top",{class:"mut","font-size":11}));
    row(158,16,fr,mf); row(200,16,ts,1,"var(--c4)"); vox.forEach((r,b)=>row(242+b*14,14,r,mv));
    st.innerHTML=""; st.append(stat("Events in the window",String(win.length),"from "+Math.max(0,t0)+" to "+now+" ms"),
      stat("Pixels with ON and OFF",String(on.filter((a,k)=>a&&off[k]).length),"their polarities cancel in the event frame"),
      stat("Largest |sum| in the event frame",String(mf),"per-edge count is 7"),
      stat("Timing kept",(D/(B-1)).toFixed(1)+" ms","voxel bin spacing; the event frame keeps none")); };
  el.append(h("div",{class:"vt"},"Three ways to turn events into an array"),
    h("p",{class:"vd"},"The bar of Day 1 at 0.5 px/ms with C = 0.30. Pink is positive, blue is negative, violet is the time surface; stronger colour means larger value."),
    h("div",{class:"ctl"},range("Now",10,100,1,now,x=>{ now=x; draw(); },x=>x+" ms").el,range("Window length",5,50,1,D,x=>{ D=x; draw(); },x=>x+" ms").el,range("Decay τ",2,40,1,tau,x=>{ tau=x; draw(); },x=>x+" ms").el),
    svg,st);
  draw();
};

// Day 4: contrast maximization for one unknown, the image speed.
VIZ.w15_cmax=el=>{
  const N=60, T=60, VT=0.6, TR=30, W=600, H=300, x0=30, x1=570, cw=(x1-x0)/N, y0=22, y1=142, b0=178, b1=278;
  const sig=simulate(scene([[2,6,1],[12,9,0.45]]),VT,0.3,N,T);
  let th=0, noise=30, ev=[], curve=[], best=0, msg="";
  const warp=(e,t)=>e.x-t*(e.t-TR);
  const iwe=t=>{ const Hh=new Array(N).fill(0); for(const e of ev){ const x=warp(e,t), i=Math.floor(x), f=x-i; if(i>=0&&i<N) Hh[i]+=1-f; if(i+1>=0&&i+1<N) Hh[i+1]+=f; } return Hh; };
  const vari=Hh=>{ const m=Hh.reduce((a,b)=>a+b,0)/N; return Hh.reduce((a,b)=>a+(b-m)*(b-m),0)/N; };
  const f=t=>vari(iwe(t));
  const remake=()=>{ const r=rng(7); ev=sig.concat(Array.from({length:Math.round(sig.length*noise/100)},()=>({x:Math.floor(r.u()*N),t:r.u()*T,p:r.u()<0.5?1:-1})));
    curve=[]; for(let i=0;i<=120;i++) curve.push({x:i/100,y:f(i/100)}); best=curve.reduce((a,b)=>b.y>a.y?b:a); };
  const X=x=>x0+(x+0.5)*cw, Y=t=>y0+t*(y1-y0)/T;
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Top: events drawn at their warped pixel position against time. Bottom: the image of warped events, the number of events that land on each pixel."});
  const chart=h("div"), st=h("div",{class:"stats"}), out=h("div",{class:"out"});
  const draw=()=>{ const Hh=iwe(th), m=Math.max(1,...Hh), inside=ev.filter(e=>{ const x=warp(e,th); return x>=-0.5&&x<=N-0.5; });
    svg.innerHTML="";
    svg.append(txt(x0,14,"Events after the warp x′ = x − θ (t − 30 ms). Warped pixel →, time ↓",{class:"mut","font-size":11}),
      txt(x0,170,"Image of warped events: how many events land on each pixel",{class:"mut","font-size":11}),
      sv("rect",{x:x0,y:y0,width:x1-x0,height:y1-y0,fill:"var(--soft)",stroke:"var(--line)"}),
      sv("rect",{x:x0,y:b0,width:x1-x0,height:b1-b0,fill:"var(--soft)",stroke:"var(--line)"}),
      sv("line",{x1:x0,x2:x1,y1:Y(TR),y2:Y(TR),stroke:"var(--line2)","stroke-dasharray":"4 4"}),
      dots(inside,1,e=>X(warp(e,th)),e=>Y(e.t)),dots(inside,-1,e=>X(warp(e,th)),e=>Y(e.t)));
    Hh.forEach((a,k)=>{ if(a>0) svg.append(sv("rect",{x:(x0+k*cw+1).toFixed(1),y:(b1-a/m*(b1-b0-4)).toFixed(1),width:(cw-2).toFixed(1),height:(a/m*(b1-b0-4)).toFixed(1),fill:"var(--c1)"})); });
    svg.append(txt(x1,b1+15,"tallest column: "+m.toFixed(1)+" events",{class:"mut","font-size":11,"text-anchor":"end"}),txt(x0,b1+15,"pixel 0",{class:"mut","font-size":11}));
    const ym=best.y*1.1; chart.innerHTML="";
    chart.append(lineChart({h:220,aria:"Variance of the image of warped events as a function of the candidate speed. It has one sharp peak near 0.6 pixels per millisecond.",
      x:{min:0,max:1.2,ticks:[0,0.2,0.4,0.6,0.8,1,1.2].map(a=>[a,a.toFixed(1)]),label:"candidate speed θ (px/ms)"},y:{min:0,max:ym,ticks:[0.5,1].map(a=>[a*best.y,(a*best.y).toFixed(0)]),label:"variance"},
      series:[{name:"variance of the IWE",color:"var(--c1)",dots:false,points:curve}],marker:{x:th,y:f(th)}}));
    st.innerHTML=""; st.append(stat("Candidate speed θ",th.toFixed(2)+" px/ms"),stat("Variance of the IWE",f(th).toFixed(1),"the objective"),
      stat("Best θ on the curve",best.x.toFixed(2)+" px/ms","variance "+best.y.toFixed(1)),stat("True speed",VT.toFixed(2)+" px/ms",ev.length+" events, "+(ev.length-sig.length)+" of them noise"));
    out.textContent=msg||"Drag θ, or press Climb to let a local search do it."; };
  const sl=range("Candidate speed θ",0,1.2,0.01,th,x=>{ th=x; draw(); },x=>x.toFixed(2)+" px/ms");
  const set=x=>{ sl.input.value=x; sl.input.dispatchEvent(new Event("input")); };
  // derivative-free local search: try one step left and right, move if better, otherwise halve the step
  const climb=()=>{ let s=0.16, t=th, ft=f(t), n=1; const from=th;
    while(s>0.004){ const c=[t-s,t+s].filter(a=>a>=0&&a<=1.2).map(a=>[f(a),a]).sort((a,b)=>b[0]-a[0])[0]; n+=2; if(c&&c[0]>ft){ ft=c[0]; t=c[1]; } else s/=2; }
    msg="Climb from θ = "+from.toFixed(2)+": stopped at θ = "+t.toFixed(2)+" after "+n+" evaluations of the variance."; set(Math.round(t*100)/100); };
  el.append(h("div",{class:"vt"},"Contrast maximization in one dimension"),
    h("p",{class:"vd"},"Two bars of different brightness move at an unknown constant speed for 60 ms. Every event is slid along a line of slope θ to where it would have been at 30 ms, and the slid events are counted per pixel."),
    h("div",{class:"ctl"},sl.el,range("Noise events",0,100,10,noise,x=>{ noise=x; msg=""; remake(); draw(); },x=>x+"% extra").el),
    h("div",{class:"ctl"},button("Climb",climb,"primary"),button("Reset θ to 0",()=>{ msg=""; set(0); })),
    svg,leg([[ON,"ON event"],[OFF,"OFF event"],["var(--c1)","image of warped events (IWE)"]]),chart,st,out);
  remake(); draw();
};

// Day 5: absolute trajectory error against relative pose error for one yaw glitch.
VIZ.w15_ate=el=>{
  const n=40, W=600, H=300, R=Math.PI/180;
  let g=10, k=8;
  const walk=glitch=>{ const P=[[0,0,0]]; let x=0, y=0, th=0;
    for(let i=0;i<n;i++){ th+=(i>=15&&i<30?6*R:0)+(glitch&&i===k?g*R:0); x+=Math.cos(th); y+=Math.sin(th); P.push([x,y,th]); } return P; };
  const rel=(P,i)=>{ const c=Math.cos(P[i][2]), s=Math.sin(P[i][2]), dx=P[i+1][0]-P[i][0], dy=P[i+1][1]-P[i][1]; return [c*dx+s*dy,-s*dx+c*dy]; };   // step i seen from pose i
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Top view of a ground-truth path and an estimated path that contains one wrong turn, after the estimate has been rigidly aligned to the ground truth. Short lines join matching poses."});
  const st=h("div",{class:"stats"});
  const draw=()=>{ const Q=walk(false), P=walk(true), mean=(A,j)=>A.reduce((s,p)=>s+p[j],0)/A.length;
    const qx=mean(Q,0), qy=mean(Q,1), px=mean(P,0), py=mean(P,1);
    // least-squares rigid alignment in 2-D: rotate by a about the centroids
    let sn=0, cs=0; P.forEach((p,i)=>{ const ax=p[0]-px, ay=p[1]-py, bx=Q[i][0]-qx, by=Q[i][1]-qy; sn+=ax*by-ay*bx; cs+=ax*bx+ay*by; });
    const a=Math.atan2(sn,cs), A=P.map(p=>[qx+Math.cos(a)*(p[0]-px)-Math.sin(a)*(p[1]-py),qy+Math.sin(a)*(p[0]-px)+Math.cos(a)*(p[1]-py)]);
    const ate=Math.sqrt(A.reduce((s,p,i)=>s+(p[0]-Q[i][0])**2+(p[1]-Q[i][1])**2,0)/A.length);
    const e=Array.from({length:n},(_,i)=>{ const u=rel(P,i), w=rel(Q,i); return Math.hypot(u[0]-w[0],u[1]-w[1]); }), rpe=Math.sqrt(e.reduce((s,x)=>s+x*x,0)/n);
    const all=Q.concat(A), mnx=Math.min(...all.map(p=>p[0])), mxx=Math.max(...all.map(p=>p[0])), mny=Math.min(...all.map(p=>p[1])), mxy=Math.max(...all.map(p=>p[1]));
    const sc=Math.min((W-60)/(mxx-mnx),(H-50)/(mxy-mny)), X=p=>(W/2+(p[0]-(mnx+mxx)/2)*sc).toFixed(1), Y=p=>(H/2-(p[1]-(mny+mxy)/2)*sc).toFixed(1);
    const line=(B,at)=>sv("path",Object.assign({d:B.map((p,i)=>(i?"L":"M")+X(p)+" "+Y(p)).join(" "),fill:"none","stroke-width":2.5,"stroke-linejoin":"round"},at));
    svg.innerHTML=""; Q.forEach((q,i)=>svg.append(sv("line",{x1:X(q),y1:Y(q),x2:X(A[i]),y2:Y(A[i]),stroke:"var(--c2)","stroke-width":1.5})));
    svg.append(line(Q,{stroke:"var(--line2)","stroke-dasharray":"6 5"}),line(A,{stroke:"var(--c3)"}),
      sv("circle",{cx:X(A[k]),cy:Y(A[k]),r:6,fill:"var(--c3)",stroke:"var(--surface)","stroke-width":2}),sv("circle",{cx:X(Q[0]),cy:Y(Q[0]),r:5,fill:"var(--ink)"}));
    st.innerHTML=""; st.append(stat("ATE (RMSE)",ate.toFixed(2)+" steps","after rigid alignment"),stat("RPE translation (RMSE)",rpe.toFixed(3)+" steps","over 40 consecutive pairs"),
      stat("Largest single RPE",Math.max(...e).toFixed(3)+" steps","the glitch step; all others are 0"),stat("End-point error",Math.hypot(P[n][0]-Q[n][0],P[n][1]-Q[n][1]).toFixed(2)+" steps","start poses matched, no alignment")); };
  el.append(h("div",{class:"vt"},"One wrong turn: ATE against RPE"),
    h("p",{class:"vd"},"A 40-step path (unit steps, one 90° bend). The estimate copies the true motion except for one extra turn at one step."),
    h("div",{class:"ctl"},range("Size of the wrong turn",0,30,1,g,x=>{ g=x; draw(); },x=>x+"°").el,range("It happens at step",1,39,1,k,x=>{ k=x; draw(); }).el),
    svg,leg([["var(--line2)","ground truth"],["var(--c3)","estimate, aligned (dot: the glitch)"],["var(--c2)","error at each pose"]]),st);
  draw();
};
