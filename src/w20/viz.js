// Week 20 · NeRF and 3D Gaussian splatting as maps.
// w20_ray: volume rendering along one camera ray (Day 2, review).  w20_field: one 1-D field stored three ways (Day 3).
// w20_splat: 2-D Gaussians splatted onto a 1-D image (Day 4).      w20_track: camera tracking against a splat map (Day 5).
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
const legend=items=>h("div",{class:"legend"},...items.map(([c,t])=>h("span",{},h("i",{style:"background:"+c}),t)));
// front-to-back "over" compositing done by the browser: layers [[colour, alpha], …] nearest first, on the page surface.
// color-mix(A a%, rest) = a·A + (1−a)·rest, which is exactly one step of equation (14.6).
const over=layers=>layers.reduceRight((s,[c,a])=>"color-mix(in srgb, "+c+" "+(clamp(a,0,1)*100).toFixed(1)+"%, "+s+")","var(--surface)");
const swatch=layers=>h("span",{style:"display:inline-block;width:64px;height:20px;border-radius:6px;border:1px solid var(--line2);vertical-align:middle;background:"+over(layers)});
// one gradient-descent step on f at p: central-difference gradient, then halve the step until f goes down
const gdStep=(f,p,lr)=>{ const f0=f(p), g=p.map((_,i)=>{ const a=p.slice(), b=p.slice(); a[i]+=1e-4; b[i]-=1e-4; return (f(a)-f(b))/2e-4; });
  for(let s=lr,t=0;t<24;t++,s/=2){ const q=p.map((v,i)=>v-s*g[i]); if(f(q)<f0) return q; } return p; };

// ---------- one ray: a wide thin "fog" bump and a narrow dense "wall" bump of density ----------
const TF=6, FOG_SD=0.5, WALL_SD=0.12;
const dens=(p,t)=>[p.sf*Math.exp(-((t-p.a)**2)/(2*FOG_SD**2)), p.sw*Math.exp(-((t-p.b)**2)/(2*WALL_SD**2))];
// the quadrature of equation (14.2) at sample positions ts: delta_i = t_{i+1} − t_i (the last one runs to the far bound)
const rayRender=(p,ts)=>{ let T=1, wf=0, ww=0, depth=0;
  const S=ts.map((t,i)=>{ const [f,w]=dens(p,t), s=f+w, d=(i+1<ts.length?ts[i+1]:TF)-t, al=1-Math.exp(-s*d), o={t,s,al,T,w:T*al,ff:s>0?f/s:0};
    wf+=o.w*o.ff; ww+=o.w*(1-o.ff); depth+=o.w*t; T*=1-al; return o; });
  return {S,wf,ww,bg:T,depth}; };
// N evenly spaced bins on [0, TF]; one sample per bin, at the middle or (jit) uniformly at random inside it
const raySamples=(N,jit,seed)=>{ const r=rng(seed); return Array.from({length:N},(_,i)=>(i+(jit?r.u():0.5))*TF/N); };

VIZ.w20_ray=el=>{
  const p={sf:0.4,sw:12,a:2,b:4.5}; let N=16, jit=false, seed=1;
  const X=t=>50+t/TF*530, base=[104,214,324], PH=80, ys=s=>PH*Math.sqrt(Math.min(s,20)/20);
  const svg=sv("svg",{viewBox:"0 0 600 350",role:"img","aria-label":"Three stacked plots along one camera ray: the density profile with its samples, the transmittance, and the weight of each sample"});
  const g=sv("g"), hf=sv("circle",{r:22,cy:base[0],fill:"transparent"}), hw=sv("circle",{r:22,cy:base[0],fill:"transparent"}), st=h("div",{class:"stats"});
  svg.append(g,hf,hw);
  const mix=R=>[["var(--c3)",R.wf],["var(--c5)",R.ww/((1-R.wf)||1)]], parts=R=>"fog "+R.wf.toFixed(2)+" · wall "+R.ww.toFixed(2)+" · background "+R.bg.toFixed(2);
  const draw=()=>{ const R=rayRender(p,raySamples(N,jit,seed)), E=rayRender(p,raySamples(1200,false,0)), fine=E.S.filter((_,i)=>i%4===0), bw=Math.max(2,530/N*0.7);
    g.innerHTML="";
    ["density σ(t), square-root scale. Drag the two round handles.","transmittance T(t): line = exact, dots = T at the samples","weight of each sample, w = T·α"].forEach((s,i)=>g.append(
      sv("line",{x1:X(0),x2:X(TF),y1:base[i],y2:base[i],stroke:"var(--line2)"}),txt(X(0),base[i]-PH-8,s,{class:"mut","font-size":11})));
    [0,1].forEach(c=>g.append(sv("path",{d:"M"+X(0)+" "+base[0]+fine.map(s=>"L"+X(s.t).toFixed(1)+" "+(base[0]-ys(s.s*(c?1-s.ff:s.ff))).toFixed(1)).join("")+"L"+X(TF)+" "+base[0]+"Z",
      fill:c?"var(--c5-soft)":"var(--c3-soft)",stroke:c?"var(--c5)":"var(--c3)","stroke-width":1.5})));
    g.append(sv("path",{d:fine.map((s,i)=>(i?"L":"M")+X(s.t).toFixed(1)+" "+(base[1]-PH*s.T).toFixed(1)).join(""),fill:"none",stroke:"var(--muted)","stroke-width":2}));
    R.S.forEach(s=>{ const x=X(s.t), hb=PH*s.w*(1-s.ff), hp=PH*s.w*s.ff;
      g.append(sv("line",{x1:x,x2:x,y1:base[0],y2:base[0]-ys(s.s),stroke:"var(--ink)","stroke-width":1}),sv("circle",{cx:x,cy:base[0]-ys(s.s),r:2.5,fill:"var(--ink)"}),
        sv("circle",{cx:x,cy:base[1]-PH*s.T,r:3,fill:"var(--acc)"}),
        sv("rect",{x:x-bw/2,y:base[2]-hp,width:bw,height:hp,fill:"var(--c3)"}),sv("rect",{x:x-bw/2,y:base[2]-hp-hb,width:bw,height:hb,fill:"var(--c5)"})); });
    g.append(sv("line",{x1:X(R.depth),x2:X(R.depth),y1:base[2],y2:base[2]+9,stroke:"var(--acc)","stroke-width":3}),
      txt(clamp(X(R.depth),110,490),base[2]+22,"expected depth "+R.depth.toFixed(2),{"text-anchor":"middle","font-size":11}),
      sv("circle",{cx:X(p.a),cy:base[0],r:8,fill:"var(--c3)",stroke:"var(--surface)","stroke-width":2}),sv("circle",{cx:X(p.b),cy:base[0],r:8,fill:"var(--c5)",stroke:"var(--surface)","stroke-width":2}));
    hf.setAttribute("cx",X(p.a)); hw.setAttribute("cx",X(p.b));
    st.innerHTML=""; st.append(stat("Pixel from "+N+" samples",swatch(mix(R)),parts(R)),stat("Exact pixel",swatch(mix(E)),parts(E)),
      stat("Expected depth",R.depth.toFixed(2),"exact "+E.depth.toFixed(2)),stat("Weight error",(Math.abs(R.wf-E.wf)+Math.abs(R.ww-E.ww)).toFixed(3),"fog and wall, against exact")); };
  drag(svg,hf,x=>{ p.a=clamp((x-50)/530*TF,0.5,5.5); draw(); }); drag(svg,hw,x=>{ p.b=clamp((x-50)/530*TF,0.5,5.5); draw(); });
  const jb=button("Random position in each bin",()=>{ jit=!jit; jb.classList.toggle("on",jit); draw(); });
  el.append(h("div",{class:"vt"},"One camera ray through fog and a wall"),h("p",{class:"vd"},"The camera is at the left (t = 0) and looks right, to the far bound t = 6. Pink density is fog, blue density is a wall. Equation (14.2) turns the samples into one pixel."),
    h("div",{class:"ctl"},range("Fog density",0,3,0.05,p.sf,v=>{ p.sf=v; draw(); },v=>v.toFixed(2)).el,range("Wall density",0,20,0.5,p.sw,v=>{ p.sw=v; draw(); },v=>v.toFixed(1)).el,range("Samples N",2,64,1,N,v=>{ N=v; draw(); }).el),
    h("div",{class:"ctl"},jb,button("New random draw",()=>{ seed++; draw(); })),svg,
    legend([["var(--c3)","fog"],["var(--c5)","wall"],["var(--ink)","samples"],["var(--acc)","T at each sample"]]),st);
  draw();
};

// ---------- one 1-D field stored three ways ----------
const fTrue=x=>0.5+0.22*Math.sin(2.6*Math.PI*x)+0.08*Math.sin(8.6*Math.PI*x+1)+(x>0.8?0.25:0);
const FX=Array.from({length:101},(_,i)=>i/100), HOLE=[0.24,0.36], SPLIT=0.6;
// features of position x. "global": 1 and R/2 sine-cosine pairs, each non-zero everywhere. "grid": R cells, linear interpolation between the two nearest nodes.
const feat=(kind,R,x)=>{ if(kind==="global") return [1].concat(...Array.from({length:R>>1},(_,k)=>[Math.sin((k+1)*Math.PI*x),Math.cos((k+1)*Math.PI*x)]));
  const r=new Array(R+1).fill(0), s=clamp(x*R,0,R), i=Math.min(R-1,Math.floor(s)); r[i]=1-(s-i); r[i+1]=s-i; return r; };
const ridge=(F,y,lam)=>{ const Ft=M.T(F), A=M.mul(Ft,F); A.forEach((r,i)=>r[i]+=lam); return M.solve(A,M.mv(Ft,y)); };       // least squares with a small pull toward zero
const minNorm=(F,r,eps)=>{ const A=M.mul(F,M.T(F)); A.forEach((q,i)=>q[i]+=eps); return M.mv(M.T(F),M.solve(A,r)); };        // smallest parameter change d with F d = r
// → predictors before and after exploring, and how many numbers are stored at each stage
const fieldModel=(kind,R,replay)=>{ const obs0=FX.filter(x=>x<=SPLIT&&(x<HOLE[0]||x>HOLE[1])), obs1=FX.filter(x=>x>SPLIT), all=obs0.concat(obs1);
  if(kind==="points"){ const mk=P=>x=>{ let b=P[0]; for(const q of P) if(Math.abs(q-x)<Math.abs(b-x)) b=q; return Math.abs(b-x)<=0.015?fTrue(b):NaN; }; return {f0:mk(obs0),f1:mk(all),n:[obs0.length,all.length],obs0,obs1}; }
  const F=xs=>xs.map(x=>feat(kind,R,x)), th0=ridge(F(obs0),obs0.map(fTrue),1e-3), pr=th=>x=>feat(kind,R,x).reduce((s,v,i)=>s+v*th[i],0), f0=pr(th0);
  const d=minNorm(F(obs1),obs1.map(x=>fTrue(x)-f0(x)),1e-2), th1=replay?ridge(F(all),all.map(fTrue),1e-3):th0.map((v,i)=>v+d[i]);
  return {f0,f1:pr(th1),n:[th0.length,th0.length],obs0,obs1}; };
const rms=(f,xs)=>Math.sqrt(xs.reduce((s,x)=>s+(f(x)-fTrue(x))**2,0)/xs.length);

VIZ.w20_field=el=>{
  let kind="global", R=24, stage=0, replay=false;
  const X=x=>40+x*540, Y=y=>250-clamp(y,-0.1,1.25)*175, gap=FX.filter(x=>x>HOLE[0]&&x<HOLE[1]);
  const svg=sv("svg",{viewBox:"0 0 600 280",role:"img","aria-label":"A 1-D field over position: the true field, the stored model, a gap that is never observed, and a region that is observed only after exploring"}), st=h("div",{class:"stats"});
  const curve=(f,a)=>{ let d="", pen=false; for(let i=0;i<=300;i++){ const x=i/300, y=f(x); if(isNaN(y)){ pen=false; continue; } d+=(pen?"L":"M")+X(x).toFixed(1)+" "+Y(y).toFixed(1); pen=true; }
    return sv("path",Object.assign({d,fill:"none","stroke-width":2.5,"stroke-linejoin":"round"},a)); };
  const fmt=v=>isNaN(v)?"empty":v.toFixed(3);
  const draw=()=>{ const m=fieldModel(kind,R,replay), f=stage?m.f1:m.f0; svg.innerHTML="";
    svg.append(sv("rect",{x:X(HOLE[0]),y:20,width:X(HOLE[1])-X(HOLE[0]),height:238,fill:"var(--c2-soft)"}),sv("rect",{x:X(SPLIT),y:20,width:X(1)-X(SPLIT),height:238,fill:stage?"var(--c1-soft)":"var(--soft)"}),
      sv("line",{x1:X(0),x2:X(1),y1:Y(0),y2:Y(0),stroke:"var(--line2)"}),
      txt(X(0.3),14,"gap, never seen",{"text-anchor":"middle","font-size":11,class:"mut"}),txt(X(0.8),14,stage?"new region, now seen":"new region, not seen yet",{"text-anchor":"middle","font-size":11,class:"mut"}),
      txt(X(0.1),14,"old region",{"text-anchor":"middle","font-size":11,class:"mut"}),txt(X(0),272,"position x = 0",{"font-size":11,class:"mut"}),txt(X(1),272,"x = 1",{"text-anchor":"end","font-size":11,class:"mut"}),
      curve(fTrue,{stroke:"var(--line2)","stroke-dasharray":"6 5"}));
    if(stage) svg.append(curve(m.f0,{stroke:"var(--c3)",opacity:0.45}));
    svg.append(curve(f,{stroke:"var(--acc)"}));
    st.innerHTML=""; st.append(stat("Numbers stored",String(m.n[stage]),kind==="points"?"one value per point, resolution not used":"parameters"),
      stat("Error, old region",fmt(rms(f,m.obs0)),stage?"was "+fmt(rms(m.f0,m.obs0))+" before exploring":"before exploring"),
      stat("Error in the gap",fmt(rms(f,gap)),"never observed"),stat("Error, new region",stage?fmt(rms(f,m.obs1)):"—",stage?(replay&&kind!=="points"?"trained on old and new":"trained on new data only"):"press Explore")); };
  const eb=button("Explore the new region",()=>{ stage=1; draw(); },"primary"), rb=button("Replay old keyframes",()=>{ replay=!replay; rb.classList.toggle("on",replay); draw(); });
  el.append(h("div",{class:"vt"},"One field, three ways to store it"),h("p",{class:"vd"},"A 1-D stand-in for a scene: the true field (dashed) is observed in the old region except for a gap. Explore then fits the new region. Without replay it uses the new samples only and changes the stored numbers as little as it can."),
    h("div",{class:"ctl"},select("Representation",[["global","One global function (stands in for an MLP)"],["grid","Grid of cells, linear interpolation"],["points","Points"]],v=>{ kind=v; draw(); }).el,
      range("Resolution",4,48,2,R,v=>{ R=v; draw(); }).el),
    h("div",{class:"ctl"},eb,rb,button("Reset",()=>{ stage=0; draw(); })),svg,
    legend([["var(--line2)","true field"],["var(--acc)","stored model"],["var(--c3)","model before exploring"]]),st);
  draw();
};

// ---------- 2-D Gaussian splatting onto a 1-D image ----------
// World: x sideways, z depth. A Gaussian has mean (x, z), standard deviations s1, s2 along axes rotated by th, and opacity o.
const UMAX=0.9, NPX=90, UPIX=Array.from({length:NPX},(_,i)=>-UMAX+(i+0.5)*2*UMAX/NPX), SC=64, sx=x=>300+x*SC, sy=z=>340-z*SC;
const GCOL=["var(--c3)","var(--c5)","var(--c1)"], GNAME=["pink","blue","green"];
const MAP=[{x:-0.8,z:2,s1:0.55,s2:0.16,th:0.5,o:0.7},{x:0.6,z:3,s1:0.8,s2:0.22,th:-0.35,o:0.85},{x:0.1,z:4.2,s1:1.7,s2:0.28,th:0,o:0.95}];
const gCov=q=>{ const c=Math.cos(q.th), s=Math.sin(q.th), a=q.s1*q.s1, b=q.s2*q.s2; return [[c*c*a+s*s*b,c*s*(a-b)],[c*s*(a-b),s*s*a+c*c*b]]; };   // Σ = R S Sᵀ Rᵀ, equation (14.4)
// camera at (cx, 0) looking along +z, image line z = 1: centre u = x/z and variance J Σ Jᵀ with J = [1/z, −x/z²] (equation (14.5); W is the identity here)
const gProj=(q,cx)=>{ const x=q.x-cx, z=q.z, S=gCov(q), a=1/z, b=-x/(z*z); return {u:x/z,v:a*a*S[0][0]+2*a*b*S[0][1]+b*b*S[1][1]}; };
// equations (14.6) and (14.7) at every pixel u: sort by depth, alpha = o·G(u), weight = alpha · product of (1 − alpha) in front
const splat=(Gs,cx,U)=>{ const P=Gs.map(q=>gProj(q,cx)), ord=Gs.map((_,k)=>k).sort((a,b)=>Gs[a].z-Gs[b].z), al=Gs.map(()=>[]), w=Gs.map(()=>[]);
  U.forEach((u,i)=>{ let T=1; for(const k of ord){ const a=Gs[k].o*Math.exp(-((u-P[k].u)**2)/(2*P[k].v)); al[k][i]=a; w[k][i]=T*a; T*=1-a; } });
  return {P,ord,al,w}; };
const imgLoss=(A,B)=>{ let s=0, n=0; A.forEach((r,k)=>r.forEach((v,i)=>{ s+=(v-B[k][i])**2; n++; })); return s/n; };   // each Gaussian's colour is one image channel
// top view: Gaussians as 1- and 2-sigma ellipses; each camera [x, colour, filled] with its field of view and image line
const drawScene=(g,Gs,cams)=>{ cams.forEach(([cx,col,solid])=>g.append(
    sv("path",{d:"M"+sx(cx-UMAX*5)+" "+sy(5)+"L"+sx(cx)+" "+sy(0)+"L"+sx(cx+UMAX*5)+" "+sy(5),fill:"none",stroke:col,"stroke-dasharray":"4 4",opacity:0.7}),
    sv("line",{x1:sx(cx-UMAX),x2:sx(cx+UMAX),y1:sy(1),y2:sy(1),stroke:col,"stroke-width":2}),
    sv("path",{d:"M"+sx(cx)+" "+(sy(0)-12)+"l-9 16h18z",fill:solid?col:"var(--surface)",stroke:col,"stroke-width":2})));
  Gs.forEach((q,k)=>{ const S=gCov(q), Ss=[[S[0][0],-S[0][1]],[-S[0][1],S[1][1]]];
    g.append(covEllipse(Ss,sx(q.x),sy(q.z),2,SC,{stroke:GCOL[k],"stroke-width":1,fill:GCOL[k],"fill-opacity":(0.25*q.o).toFixed(2)}),
      covEllipse(Ss,sx(q.x),sy(q.z),1,SC,{stroke:"none",fill:GCOL[k],"fill-opacity":(0.6*q.o).toFixed(2)})); }); };
// a 1-D image as a strip of pixels, from the per-Gaussian weights w[k][i] in depth order ord
const strip=(g,ord,w,y,hh)=>UPIX.forEach((_,i)=>{ let rem=1; const L=ord.map(k=>{ const a=rem>1e-9?w[k][i]/rem:0; rem-=w[k][i]; return [GCOL[k],a]; });
  g.append(sv("rect",{x:(30+i*540/NPX).toFixed(1),y,width:(540/NPX+0.5).toFixed(1),height:hh,style:"fill:"+over(L)})); });

VIZ.w20_splat=el=>{
  const start=()=>[{...MAP[0],x:-0.3,z:2.4,th:0.1,o:0.5},{...MAP[1],x:1,z:2.7,th:0,o:0.6},{...MAP[2],x:-0.4,z:4.4,th:0.15,o:0.8}];
  let Gs=start(), sel=0, up=0.1, steps=0;
  const tgt=splat(MAP,0,UPIX), deg=r=>Math.round(r*180/Math.PI);
  const svg=sv("svg",{viewBox:"0 0 600 478",role:"img","aria-label":"Top view of three coloured Gaussians in front of a camera, the weight of each Gaussian at every pixel, the rendered 1-D image, and the target 1-D image"});
  const g=sv("g"), hs=Gs.map(()=>sv("circle",{r:20,fill:"transparent"})), tb=h("div",{style:"overflow-x:auto"}), st=h("div",{class:"stats"});
  svg.append(g,...hs);
  const pack=()=>Gs.flatMap(q=>[q.x,q.z,q.th,q.o]), unpack=p=>Gs.map((q,k)=>({...q,x:clamp(p[4*k],-4,4),z:clamp(p[4*k+1],0.6,4.8),th:p[4*k+2],o:clamp(p[4*k+3],0.02,0.98)}));
  const loss=p=>imgLoss(splat(unpack(p),0,UPIX).w,tgt.w);
  const ro=range("Opacity o",0.05,0.95,0.05,Gs[0].o,v=>{ Gs[sel].o=v; draw(); },v=>v.toFixed(2)), rr=range("Rotation",-90,90,5,deg(Gs[0].th),v=>{ Gs[sel].th=v*Math.PI/180; draw(); },v=>v+"°");
  const se=select("Edit Gaussian",GNAME.map((n,k)=>[k,n]),v=>{ sel=+v; sync(); draw(); });
  const sync=()=>{ se.input.value=sel; [[ro,Gs[sel].o.toFixed(2)],[rr,deg(Gs[sel].th)]].forEach(([r,v])=>{ r.input.value=v; r.el.querySelector("output").textContent=v+(r===rr?"°":""); }); };
  const draw=()=>{ const R=splat(Gs,0,UPIX), one=splat(Gs,0,[up]); g.innerHTML=""; drawScene(g,Gs,[[0,"var(--muted)",true]]);
    g.append(sv("line",{x1:sx(0),y1:sy(0),x2:sx(up*5),y2:sy(5),stroke:"var(--ink)","stroke-width":1.5}),txt(sx(0)+14,sy(0)+4,"camera",{"font-size":11,class:"mut"}),txt(sx(UMAX)+6,sy(1)+4,"image line z = 1",{"font-size":11,class:"mut"}));
    R.ord.forEach((k,j)=>{ const hw=2*Math.sqrt(R.P[k].v), a=clamp(R.P[k].u-hw,-UMAX,UMAX), b=clamp(R.P[k].u+hw,-UMAX,UMAX);
      g.append(sv("line",{x1:sx(a),x2:sx(b),y1:sy(1)-5-5*j,y2:sy(1)-5-5*j,stroke:GCOL[k],"stroke-width":4})); });
    Gs.forEach((q,k)=>{ g.append(sv("circle",{cx:sx(q.x),cy:sy(q.z),r:k===sel?6:4,fill:GCOL[k],stroke:"var(--ink)","stroke-width":k===sel?2:1})); hs[k].setAttribute("cx",sx(q.x)); hs[k].setAttribute("cy",sy(q.z)); });
    g.append(txt(30,358,"weight of each Gaussian at every pixel",{"font-size":11,class:"mut"}),sv("line",{x1:30,x2:570,y1:422,y2:422,stroke:"var(--line2)"}));
    Gs.forEach((_,k)=>g.append(sv("path",{d:R.w[k].map((v,i)=>(i?"L":"M")+(30+(i+0.5)*540/NPX).toFixed(1)+" "+(422-56*v).toFixed(1)).join(""),fill:"none",stroke:GCOL[k],"stroke-width":2})));
    strip(g,R.ord,R.w,428,18); strip(g,tgt.ord,tgt.w,454,18);
    const px=30+(up+UMAX)/(2*UMAX)*540; g.append(sv("line",{x1:px,x2:px,y1:364,y2:474,stroke:"var(--ink)","stroke-width":1.5}),
      txt(26,441,"now",{"text-anchor":"end","font-size":11,class:"mut"}),txt(26,467,"goal",{"text-anchor":"end","font-size":11,class:"mut"}));
    let T=1; const rows=one.ord.map(k=>{ const a=one.al[k][0], r=h("tr",{},h("td",{},GNAME[k]),h("td",{},Gs[k].z.toFixed(2)),h("td",{},one.P[k].u.toFixed(2)),h("td",{},Math.sqrt(one.P[k].v).toFixed(3)),h("td",{},a.toFixed(3)),h("td",{},T.toFixed(3)),h("td",{},(T*a).toFixed(3))); T*=1-a; return r; });
    tb.innerHTML=""; tb.append(h("table",{},h("tr",{},...["Front to back","depth z","centre u′","width σ′","α at pixel","T in front","weight"].map(s=>h("th",{},s))),...rows,
      h("tr",{},h("td",{},"background"),h("td",{},""),h("td",{},""),h("td",{},""),h("td",{},""),h("td",{},T.toFixed(3)),h("td",{},T.toFixed(3)))));
    st.innerHTML=""; st.append(stat("Pixel u = "+up.toFixed(2),swatch(one.ord.map(k=>[GCOL[k],one.al[k][0]])),"blend of the rows below"),stat("Loss against the goal image",imgLoss(R.w,tgt.w).toFixed(4),steps+" gradient steps so far")); };
  hs.forEach((c,k)=>drag(svg,c,(x,y)=>{ Gs[k].x=clamp((x-300)/SC,-4,4); Gs[k].z=clamp((340-y)/SC,0.6,4.8); sel=k; sync(); draw(); }));
  el.append(h("div",{class:"vt"},"Splatting three Gaussians onto a 1-D image"),h("p",{class:"vd"},"Top view. Each Gaussian is projected onto the image line with equation (14.5), then the pixels are blended front to back with equations (14.6) and (14.7). Drag a Gaussian by its centre dot."),
    h("div",{class:"ctl"},se.el,ro.el,rr.el,range("Pixel u",-0.88,0.88,0.02,up,v=>{ up=v; draw(); },v=>v.toFixed(2)).el),
    h("div",{class:"ctl"},button("10 gradient steps toward the goal",()=>{ let p=pack(); for(let i=0;i<10;i++) p=gdStep(loss,p,40); Gs=unpack(p); steps+=10; sync(); draw(); },"primary"),button("Reset",()=>{ Gs=start(); steps=0; sync(); draw(); })),
    svg,tb,st);
  draw();
};

// ---------- tracking: the map is fixed, the camera's sideways position is the unknown ----------
VIZ.w20_track=el=>{
  let x0=0.8, cx=x0, noise=0, steps=0, raf=0, tgt, curve;
  const loss=c=>imgLoss(splat(MAP,c,UPIX).w,tgt.w);
  const setup=()=>{ const r=rng(7), T=splat(MAP,0,UPIX); tgt={ord:T.ord,w:T.w.map(row=>row.map(v=>clamp(v+noise*r.n(),0,1)))};
    curve=Array.from({length:121},(_,i)=>({x:-3+i*0.05,y:0})); curve.forEach(q=>q.y=loss(q.x)); };
  const svg=sv("svg",{viewBox:"0 0 600 430",role:"img","aria-label":"Top view of a fixed map of three Gaussians with the true camera and the estimated camera, the image the true camera recorded, and the image rendered from the estimate"});
  const ch=h("div"), st=h("div",{class:"stats"});
  const draw=()=>{ const R=splat(MAP,cx,UPIX), top=Math.max(...curve.map(q=>q.y))*1.1||1; svg.innerHTML="";
    drawScene(svg,MAP,[[0,"var(--muted)",false],[cx,"var(--acc)",true]]);
    strip(svg,tgt.ord,tgt.w,364,18); strip(svg,R.ord,R.w,406,18);
    svg.append(txt(30,359,"image the camera recorded (true position x = 0)",{"font-size":11,class:"mut"}),txt(30,401,"image rendered from the estimated position",{"font-size":11,class:"mut"}));
    ch.innerHTML=""; ch.append(lineChart({w:600,h:210,aria:"Photometric loss as a function of the assumed camera position, with the current estimate marked",x:{min:-3,max:3,ticks:[-3,-2,-1,0,1,2,3].map(v=>[v,String(v)]),label:"assumed camera position x"},
      y:{min:0,max:top,ticks:[0,top/2,top].map(v=>[v,v.toFixed(3)]),label:"loss"},series:[{name:"photometric loss",color:"var(--c5)",dots:false,points:curve}],marker:{x:clamp(cx,-3,3),y:Math.min(loss(cx),top)}}));
    st.innerHTML=""; st.append(stat("Estimated camera x",cx.toFixed(3),"true value 0"),stat("Photometric loss",loss(cx).toFixed(4),"mean squared image difference"),stat("Gradient steps",String(steps))); };
  const step=()=>{ const c=gdStep(p=>loss(p[0]),[cx],3)[0], moved=Math.abs(c-cx)>1e-5; cx=c; steps++; draw(); return moved; };
  const stop=()=>{ cancelAnimationFrame(raf); raf=0; pb.textContent="Run"; };
  const pb=button("Run",()=>{ if(raf) return stop(); pb.textContent="Pause"; let n=0, f=0;
    const tick=()=>{ if(f++%5===0&&!(step()&&++n<60)) return stop(); raf=requestAnimationFrame(tick); }; raf=requestAnimationFrame(tick); },"primary");   // one step every 5th frame, so the motion is visible
  const reset=()=>{ stop(); cx=x0; steps=0; draw(); };
  el.append(h("div",{class:"vt"},"Tracking a camera against a splat map"),h("p",{class:"vd"},"The three Gaussians are a finished map. The camera took one image at x = 0. Tracking renders the map from a guessed position and slides the guess downhill on the image difference."),
    h("div",{class:"ctl"},range("Starting guess x",-2.5,2.5,0.1,x0,v=>{ x0=v; reset(); },v=>v.toFixed(1)).el,range("Image noise",0,0.3,0.02,noise,v=>{ noise=v; setup(); reset(); },v=>v.toFixed(2)).el),
    h("div",{class:"ctl"},button("Gradient step",()=>{ stop(); step(); }),pb,button("Reset",reset)),svg,
    legend([["var(--muted)","true camera"],["var(--acc)","estimated camera"]]),ch,st);
  setup(); draw();
};
