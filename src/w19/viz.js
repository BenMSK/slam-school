// Week 19: boosting SLAM with deep learning. The w19* functions hold the arithmetic; the VIZ.* functions only draw it.
const w19sum=a=>a.reduce((s,v)=>s+v,0);
const w19softmax=(s,beta)=>{ const m=Math.max(...s), e=s.map(v=>Math.exp(beta*(v-m))), Z=w19sum(e); return e.map(v=>v/Z); };
const w19leg=items=>h("div",{class:"legend"},...items.map(([c,t])=>h("span",{},h("i",{style:"background:"+c}),t)));

// ---------- Day 1: attention = softmax of query-key scores, then a weighted average of the values ----------
VIZ.w19_attn=el=>{
  const K=[15,70,135,200,255,320], cx=140, cy=140, R=95, bx=312, bw=34, gap=12, by=226, bh=170; let qa=85, beta=4;
  const svg=sv("svg",{viewBox:"0 0 600 292",role:"img","aria-label":"Left: a query descriptor and six key descriptors drawn as unit vectors on a circle. Right: one bar per key showing its attention weight, with the score q·k written below"});
  const st=h("div",{class:"stats"});
  const pt=(deg,r)=>[cx+r*Math.cos(deg*Math.PI/180),cy-r*Math.sin(deg*Math.PI/180)];
  const draw=()=>{ const s=K.map(k=>Math.cos((qa-k)*Math.PI/180)), a=w19softmax(s,beta), best=a.indexOf(Math.max(...a));
    svg.innerHTML=""; svg.append(sv("circle",{cx,cy,r:R,fill:"none",stroke:"var(--line2)"}));
    K.forEach((k,j)=>{ const [x,y]=pt(k,R), [lx,ly]=pt(k,R+17);
      svg.append(sv("line",{x1:cx,y1:cy,x2:x,y2:y,stroke:"var(--c5)","stroke-width":(1.5+5*a[j]).toFixed(2)}),sv("circle",{cx:x,cy:y,r:4,fill:"var(--c5)"}),txt(lx,ly+4,"k"+(j+1),{"text-anchor":"middle","font-size":12})); });
    const [qx,qy]=pt(qa,R-22); svg.append(sv("line",{x1:cx,y1:cy,x2:qx,y2:qy,stroke:"var(--c3)","stroke-width":3}),sv("circle",{cx:qx,cy:qy,r:6,fill:"var(--c3)"}),
      txt(cx,284,"query q (pink) and keys k1…k6 (blue)",{"text-anchor":"middle",class:"mut","font-size":11}),
      sv("line",{x1:bx-8,y1:by,x2:bx+6*(bw+gap)-gap+8,y2:by,stroke:"var(--line2)"}));
    a.forEach((v,j)=>{ const x=bx+j*(bw+gap), hh=bh*v;
      svg.append(sv("rect",{x,y:by-hh,width:bw,height:Math.max(hh,0.5),rx:3,fill:j===best?"var(--c3)":"var(--c5)"}),txt(x+bw/2,by-hh-6,v.toFixed(2),{"text-anchor":"middle","font-size":11}),
        txt(x+bw/2,by+16,"k"+(j+1),{"text-anchor":"middle","font-size":12,class:"mut"}),txt(x+bw/2,by+32,s[j].toFixed(2),{"text-anchor":"middle","font-size":11,class:"mut"})); });
    svg.append(txt(bx+3*(bw+gap)-gap/2,284,"attention weight (bar), score q·k (number below)",{"text-anchor":"middle",class:"mut","font-size":11}));
    st.innerHTML=""; st.append(stat("Largest weight",a[best].toFixed(2),"on key k"+(best+1)),stat("Output Σ aⱼ vⱼ",w19sum(a.map((v,j)=>v*(j+1))).toFixed(2),"with values vⱼ = j"),stat("Weights sum to",w19sum(a).toFixed(2),"always 1")); };
  el.append(h("div",{class:"vt"},"One attention step"),h("p",{class:"vd"},"Scores are dot products of unit vectors, weights are softmax(β · score), and the output is the weighted average of the values the keys carry."),
    h("div",{class:"ctl"},range("Query direction",0,355,5,qa,v=>{ qa=v; draw(); },v=>v+"°").el,range("Sharpness β",0,20,0.5,beta,v=>{ beta=v; draw(); }).el),svg,st);
  draw();
};

// ---------- Day 2: view-synthesis (photometric) loss for a wall seen by a rectified stereo pair ----------
const w19T=u=>0.5+0.2*Math.sin(u/41*6.2832+1)+0.17*Math.sin(u/23*6.2832+4)+0.1*Math.sin(u/15*6.2832+2.2);      // brightness of the wall texture
const w19photo=(dTrue,d)=>{ const N=64, FB=24, I1=[], I2=[], Wp=[]; let L=0, g=0, H=0, c=0;                  // disparity = FB / depth, in pixels
  for(let x=0;x<N;x++){ I1.push(w19T(x)); I2.push(w19T(x+FB/dTrue)); }                                        // a point at x in image 1 sits at x − FB/dTrue in image 2
  for(let x=0;x<N;x++){ const p=x-FB/d, a=Math.floor(p), t=p-a; if(a<0||a+1>=N){ Wp.push(null); continue; }
    const v=(1-t)*I2[a]+t*I2[a+1], e=v-I1[x], J=-FB*(I2[a+1]-I2[a]);                                          // linear interpolation, and its derivative with respect to 1/d
    Wp.push(v); L+=e*e; g+=e*J; H+=J*J; c++; }
  return {I1,I2,Wp,L:c?L/c:0,g,H}; };
VIZ.w19_photo=el=>{
  let dT=4, d=2.5; const x0=96, cw=7.5, rowY=[18,52,86,120], rh=26;
  const svg=sv("svg",{viewBox:"0 0 600 156",role:"img","aria-label":"Four rows of 64 pixels: image 1, image 2, image 2 warped with the guessed depth, and the absolute difference between image 1 and the warped image"});
  const chart=h("div"), st=h("div",{class:"stats"});
  const draw=()=>{ const r=w19photo(dT,d); svg.innerHTML="";
    [["image 1",r.I1],["image 2",r.I2],["warped 2",r.Wp],["difference",r.Wp.map((v,x)=>v==null?null:Math.abs(v-r.I1[x]))]].forEach(([name,row],k)=>{
      svg.append(txt(x0-8,rowY[k]+rh/2+4,name,{"text-anchor":"end","font-size":12}),sv("rect",{x:x0,y:rowY[k],width:cw*64,height:rh,fill:"none",stroke:"var(--line2)"}));
      row.forEach((v,x)=>{ if(v!=null) svg.append(sv("rect",{x:(x0+x*cw).toFixed(1),y:rowY[k],width:cw+0.3,height:rh,fill:k===3?"var(--c3)":"var(--ink)","fill-opacity":(k===3?Math.min(1,4*v):Math.max(0,Math.min(1,v))).toFixed(3)})); }); });
    const pts=[]; let ymax=0.001; for(let q=1.5;q<=10.001;q+=0.05){ const y=w19photo(dT,q).L; ymax=Math.max(ymax,y); pts.push({x:q,y}); }
    chart.innerHTML=""; chart.append(lineChart({h:230,aria:"View-synthesis loss as a function of the guessed depth, with a dot at the current guess",x:{min:1.5,max:10,ticks:[2,4,6,8,10].map(v=>[v,v+" m"]),label:"guessed depth d"},
      y:{min:0,max:ymax*1.1,ticks:[[0,"0"],[ymax,ymax.toFixed(3)]]},series:[{name:"loss",dots:false,points:pts}],marker:{x:d,y:r.L}}));
    st.innerHTML=""; st.append(stat("Shift used for the warp",(24/d).toFixed(2)+" px","f·b/d with f·b = 24 px·m"),stat("True shift",(24/dT).toFixed(2)+" px","depth "+dT+" m"),stat("Loss",r.L.toFixed(4),"mean squared difference")); };
  const gs=range("Guessed depth d",1.5,10,0.05,d,v=>{ d=v; draw(); },v=>v.toFixed(2)+" m");
  const show=()=>{ gs.input.value=d; gs.el.querySelector("output").textContent=d.toFixed(2)+" m"; draw(); };
  const stepDown=()=>{ const r=w19photo(dT,d); if(r.H<1e-12) return; const rho=Math.min(1/1.5,Math.max(0.1,1/d-r.g/r.H)); d=1/rho; show(); };
  el.append(h("div",{class:"vt"},"View-synthesis loss in one dimension"),h("p",{class:"vd"},"A textured wall at one depth, seen by two cameras side by side. Image 2 is pulled back to image 1 with the guessed depth and compared pixel by pixel."),
    h("div",{class:"ctl"},gs.el,range("True depth",2,8,0.5,dT,v=>{ dT=v; draw(); },v=>v+" m").el),
    h("div",{class:"ctl"},button("Step downhill",stepDown,"primary"),button("Reset guess",()=>{ d=2.5; show(); })),svg,chart,st);
  draw();
};

// ---------- Day 3: RAFT in one dimension: all-pairs correlation, pyramid, lookup, iterative update ----------
const w19raft=(seed,shift,n)=>{ const CH=16, MA=3, r=rng(seed), L=n+12, F=[];                                 // F[j] = feature vector of scene point j
  for(let j=0;j<L;j++) F.push([]);
  for(let m=0;m<CH;m++){ const w=[]; for(let j=0;j<L+MA;j++) w.push(r.n()); for(let j=0;j<L;j++){ let s=0; for(let o=0;o<MA;o++) s+=w[j+o]; F[j].push(s); } }
  const unit=v=>{ const nn=Math.hypot(...v)||1; return v.map(x=>x/nn); }, g1=[], g2=[];
  for(let i=0;i<n;i++){ g1.push(unit(F[i+8])); g2.push(unit(F[i+8-shift].map(x=>x+0.5*r.n()))); }               // pixel i of image 1 matches pixel i+shift of image 2
  const C=[g1.map(a=>g2.map(b=>w19sum(a.map((x,m)=>x*b[m]))))];                                               // level 0: C[i][k] = ⟨g1[i], g2[k]⟩, equation (13.4) in 1-D
  for(let l=1;l<3;l++) C.push(C[l-1].map(row=>{ const o=[]; for(let k=0;k+1<row.length;k+=2) o.push((row[k]+row[k+1])/2); return o; }));   // average-pool the image-2 axis
  return C; };
const w19look=(row,c)=>{ const a=Math.floor(c), t=c-a, v=k=>(k>=0&&k<row.length)?row[k]:0; return (1-t)*v(a)+t*v(a+1); };
// One iteration. The lookup is RAFT's; the update is a fixed hand-written rule standing in for the learned Conv-GRU:
// soft-argmax of the looked-up correlations (β = 20), then a 3-tap smoothing across neighbouring pixels.
const w19step=(C,f,r,levels)=>{ const n=f.length, df=f.map((fi,i)=>{ const p=i+fi, v=[], dsp=[];
    for(let l=0;l<levels;l++){ const sc=1<<l, c=(p-(sc-1)/2)/sc; for(let d=-r;d<=r;d++){ v.push(w19look(C[l][i],c+d)); dsp.push(d*sc); } }
    return w19sum(w19softmax(v,20).map((a,q)=>a*dsp[q])); });
  return f.map((fi,i)=>fi+(df[Math.max(0,i-1)]+2*df[i]+df[Math.min(n-1,i+1)])/4); };
const w19epe=(f,shift)=>{ let s=0, c=0; f.forEach((fi,i)=>{ if(i+shift<f.length){ s+=Math.abs(fi-shift); c++; } }); return c?s/c:0; };
VIZ.w19_raft=el=>{
  const n=16, cs=18, x0=64, y0=40, fx=392, fs=24; let shift=5, r=2, lv=3, C, f, it, playing=false, last=0;
  const svg=sv("svg",{viewBox:"0 0 600 350",role:"img","aria-label":"Left: the 16 by 16 correlation volume as a heat map, with the true match, the current estimate and the lookup window marked in every row. Right: the current flow estimate of every pixel as a bar"});
  const st=h("div",{class:"stats"});
  const draw=()=>{ svg.innerHTML=""; const X=k=>x0+(k+0.5)*cs, Y=i=>y0+(i+0.5)*cs;
    svg.append(txt(x0+n*cs/2,16,"pixel k in image 2 →",{"text-anchor":"middle",class:"mut","font-size":11}),txt(x0-34,y0+n*cs/2,"pixel i in image 1",{"text-anchor":"middle",class:"mut","font-size":11,transform:"rotate(-90 "+(x0-34)+" "+(y0+n*cs/2)+")"}),
      txt(fx+84,16,"flow estimate f(i), pixels",{"text-anchor":"middle",class:"mut","font-size":11}));
    [0,5,10,15].forEach(q=>svg.append(txt(X(q),y0-6,q,{"text-anchor":"middle",class:"mut","font-size":11}),txt(x0-8,Y(q)+4,q,{"text-anchor":"end",class:"mut","font-size":11})));
    for(let i=0;i<n;i++) for(let k=0;k<n;k++) svg.append(sv("rect",{x:x0+k*cs,y:y0+i*cs,width:cs-1,height:cs-1,fill:"var(--c5)","fill-opacity":Math.max(0.03,C[0][i][k]).toFixed(2)}));
    const reach=r*(1<<(lv-1)), cl=v=>Math.max(x0,Math.min(x0+n*cs,v));
    for(let i=0;i<n;i++){ const p=i+f[i]; if(i+shift<n) svg.append(sv("rect",{x:x0+(i+shift)*cs,y:y0+i*cs,width:cs-1,height:cs-1,fill:"none",stroke:"var(--c1)","stroke-width":2}));
      if(lv>1) svg.append(sv("line",{x1:cl(X(p-reach)),x2:cl(X(p+reach)),y1:Y(i),y2:Y(i),stroke:"var(--c2)","stroke-width":1.5,"stroke-dasharray":"2 3"}));
      svg.append(sv("line",{x1:cl(X(p-r)),x2:cl(X(p+r)),y1:Y(i),y2:Y(i),stroke:"var(--c3)","stroke-width":2.5}));
      if(p>-0.5&&p<n-0.5) svg.append(sv("circle",{cx:X(p),cy:Y(i),r:3.5,fill:"var(--c3)",stroke:"var(--surface)","stroke-width":1}));
      svg.append(sv("rect",{x:fx,y:y0+i*cs+3,width:Math.max(0.5,Math.min(7.5,Math.max(0,f[i]))*fs),height:cs-7,rx:2,fill:"var(--c3)"})); }
    svg.append(sv("line",{x1:fx,x2:fx,y1:y0-2,y2:y0+n*cs+2,stroke:"var(--line2)"}),sv("line",{x1:fx+shift*fs,x2:fx+shift*fs,y1:y0-2,y2:y0+n*cs+2,stroke:"var(--c1)","stroke-width":2,"stroke-dasharray":"5 4"}),
      txt(fx,y0+n*cs+18,"0",{"text-anchor":"middle",class:"mut","font-size":11}),txt(fx+shift*fs,y0+n*cs+18,shift?"true "+shift:"",{"text-anchor":"middle","font-size":11}));
    st.innerHTML=""; st.append(stat("Iteration",it+"",it?"lookup → update, "+it+" time"+(it>1?"s":""):"flow starts at zero"),stat("Mean flow error",w19epe(f,shift).toFixed(2)+" px","pixels whose match is inside image 2"),
      stat("Values read per pixel",lv*(2*r+1)+"","per iteration, out of "+(lv>2?28:lv>1?24:16)+" stored")); };
  const reset=()=>{ C=w19raft(3,shift,n); f=new Array(n).fill(0); it=0; playing=false; pb.textContent="Play"; draw(); };
  const step=()=>{ if(it>=40) return; f=w19step(C,f,r,lv); it++; draw(); };
  const tick=t=>{ if(!playing) return; if(t-last>420){ last=t; step(); } if(it>=12){ playing=false; pb.textContent="Play"; return; } requestAnimationFrame(tick); };
  const pb=button("Play",()=>{ if(playing){ playing=false; pb.textContent="Play"; return; } if(it>=12){ f=new Array(n).fill(0); it=0; draw(); } playing=true; pb.textContent="Pause"; last=0; requestAnimationFrame(tick); },"primary");
  el.append(h("div",{class:"vt"},"RAFT in one dimension"),h("p",{class:"vd"},"Two rows of 16 pixels, each pixel with a 16-channel feature vector. Image 2 is image 1 moved to the right by the true shift, plus noise."),
    h("div",{class:"ctl"},range("True shift",0,6,1,shift,v=>{ shift=v; reset(); },v=>v+" px").el,range("Lookup radius r",1,3,1,r,v=>{ r=v; reset(); }).el,range("Pyramid levels",1,3,1,lv,v=>{ lv=v; reset(); }).el),
    h("div",{class:"ctl"},button("Step",()=>{ playing=false; pb.textContent="Play"; step(); }),pb,button("Reset",reset,"ghost")),svg,
    w19leg([["var(--c5)","correlation (darker = larger)"],["var(--c1)","true match"],["var(--c3)","estimate and finest lookup window"],["var(--c2)","reach of the coarsest level"]]),st);
  reset();
};

// ---------- Day 4: confidence-weighted Gauss-Newton for a 2-D rigid motion ----------
const w19scene=()=>{ const r=rng(11), th=12*Math.PI/180, t=[0.5,0.25], c=Math.cos(th), s=Math.sin(th), P=[], Q=[], bad=[];
  for(let k=0;k<12;k++){ const b=k>=9, p=b?[1.25+0.3*(k-10),-0.75+0.25*r.n()]:[-2+3.4*r.u(),-1.05+2.1*r.u()];
    P.push(p); bad.push(b); Q.push([c*p[0]-s*p[1]+t[0]+0.02*r.n()+(b?-0.75:0),s*p[0]+c*p[1]+t[1]+0.02*r.n()+(b?0.45:0)]); }   // the last three sit on an object that moves by itself
  return {P,Q,bad,th,t}; };
// x = (t_x, t_y, θ). Each step solves (JᵀWJ) Δ = JᵀW r for the residuals r_k = q_k − (R(θ) p_k + t), then adds Δ.
const w19gn=(P,Q,w,iters)=>{ let x=[0,0,0];
  for(let n=0;n<iters;n++){ const c=Math.cos(x[2]), s=Math.sin(x[2]), A=M.zeros(3,3), b=[0,0,0];
    P.forEach((p,k)=>{ [[[1,0,-s*p[0]-c*p[1]],Q[k][0]-(c*p[0]-s*p[1]+x[0])],[[0,1,c*p[0]-s*p[1]],Q[k][1]-(s*p[0]+c*p[1]+x[1])]].forEach(([J,res])=>{
      for(let a=0;a<3;a++){ b[a]+=w[k]*J[a]*res; for(let d=0;d<3;d++) A[a][d]+=w[k]*J[a]*J[d]; } }); });
    const dx=M.solve(A,b); if(!dx) break; x=x.map((v,a)=>v+dx[a]); }
  return x; };
VIZ.w19_wgn=el=>{
  const S=w19scene(), sc=92, X=v=>300+v*sc, Y=v=>165-v*sc; let wb=1, iters=6;
  const svg=sv("svg",{viewBox:"0 0 600 330",role:"img","aria-label":"Twelve image points with arrows to where they were matched in the next frame, and circles where the current motion estimate predicts them. Three arrows belong to an object that moves by itself"});
  svg.append(sv("defs",{},sv("marker",{id:"w19arA",viewBox:"0 0 10 10",refX:9,refY:5,markerWidth:6,markerHeight:6,orient:"auto"},sv("path",{d:"M0 0 L10 5 L0 10 z",fill:"var(--c5)"})),
    sv("marker",{id:"w19arB",viewBox:"0 0 10 10",refX:9,refY:5,markerWidth:6,markerHeight:6,orient:"auto"},sv("path",{d:"M0 0 L10 5 L0 10 z",fill:"var(--c3)"}))));
  const g=sv("g"), st=h("div",{class:"stats"}), out=h("div",{class:"out"}); svg.append(g);
  const draw=()=>{ const w=S.bad.map(b=>b?wb:1), x=w19gn(S.P,S.Q,w,iters), c=Math.cos(x[2]), s=Math.sin(x[2]); let cost=0; g.innerHTML="";
    S.P.forEach((p,k)=>{ const q=S.Q[k], pr=[c*p[0]-s*p[1]+x[0],s*p[0]+c*p[1]+x[1]], col=S.bad[k]?"var(--c3)":"var(--c5)"; cost+=w[k]*((q[0]-pr[0])**2+(q[1]-pr[1])**2);
      g.append(sv("line",{x1:X(p[0]),y1:Y(p[1]),x2:X(q[0]),y2:Y(q[1]),stroke:col,"stroke-width":S.bad[k]?0.8+1.4*wb:2,"marker-end":"url(#w19ar"+(S.bad[k]?"B":"A")+")"}),sv("circle",{cx:X(p[0]),cy:Y(p[1]),r:3,fill:col}),
        sv("line",{x1:X(pr[0]),y1:Y(pr[1]),x2:X(q[0]),y2:Y(q[1]),stroke:"var(--c2)","stroke-width":2}),sv("circle",{cx:X(pr[0]),cy:Y(pr[1]),r:5,fill:"var(--surface)",stroke:"var(--ink)","stroke-width":1.5})); });
    const dth=(x[2]-S.th)*180/Math.PI, dt=Math.hypot(x[0]-S.t[0],x[1]-S.t[1]);
    st.innerHTML=""; st.append(stat("Rotation θ",(x[2]*180/Math.PI).toFixed(2)+"°","true 12°, error "+dth.toFixed(2)+"°"),stat("Translation t","("+x[0].toFixed(3)+", "+x[1].toFixed(3)+")","true (0.5, 0.25), error "+dt.toFixed(3)),stat("Weighted cost",cost.toFixed(4),"Σ wₖ‖rₖ‖²"));
    out.textContent="weights: 9 static matches w = 1, 3 moving-object matches w = "+wb.toFixed(2)+"   ·   Gauss-Newton steps from θ = 0, t = 0: "+iters; };
  el.append(h("div",{class:"vt"},"Confidence weights change the Gauss-Newton answer"),h("p",{class:"vd"},"Estimate one rigid motion (rotation θ, translation t) from 12 matches by weighted Gauss-Newton. Nine matches are on the static scene. Three are on an object that moved by itself."),
    h("div",{class:"ctl"},range("Weight on the moving-object matches",0,1,0.05,wb,v=>{ wb=v; draw(); },v=>v.toFixed(2)).el,range("Gauss-Newton steps",0,6,1,iters,v=>{ iters=v; draw(); }).el),svg,
    w19leg([["var(--c5)","match on the static scene"],["var(--c3)","match on the moving object"],["var(--c2)","residual: predicted point (circle) to matched point"]]),out,st);
  draw();
};

// ---------- Day 5: DUSt3R's scale-normalized regression loss (13.15)-(13.16) and confidence-aware loss (13.17) ----------
const w19pts=()=>{ const r=rng(5), G=[], E=[], hard=[]; for(let k=0;k<10;k++){ G.push([-3+6*r.u(),1.6+3.2*r.u()]); const a=6.2832*r.u(); E.push([Math.cos(a),Math.sin(a)]); hard.push(k>=7); } return {G,E,hard}; };
const w19conf=(S,scale,sh,alpha)=>{ const X=S.G.map((p,k)=>{ const e=S.hard[k]?sh:0.04; return [scale*(p[0]+e*S.E[k][0]),scale*(p[1]+e*S.E[k][1])]; });   // prediction = scale × (truth + error)
  const nrm=A=>w19sum(A.map(p=>Math.hypot(p[0],p[1])))/A.length, z=nrm(X), zb=nrm(S.G);                         // (13.16): mean distance to the origin
  const Xn=X.map(p=>[p[0]/z,p[1]/z]), Gn=S.G.map(p=>[p[0]/zb,p[1]/zb]), l=Xn.map((p,k)=>Math.hypot(p[0]-Gn[k][0],p[1]-Gn[k][1]));   // (13.15)
  const Cc=l.map(v=>Math.max(1,alpha/Math.max(v,1e-9)));                                                        // the C ≥ 1 that minimises C·ℓ − α log C
  return {Xn,Gn,l,C:Cc,raw:w19sum(X.map((p,k)=>Math.hypot(p[0]-S.G[k][0],p[1]-S.G[k][1])))/X.length,mean:w19sum(l)/l.length,Lc:w19sum(l.map((v,k)=>Cc[k]*v-alpha*Math.log(Cc[k])))}; };
VIZ.w19_conf=el=>{
  const S=w19pts(), sc=150, X=v=>300+v*sc, Y=v=>300-v*sc; let scale=2, sh=0.5, alpha=0.2;
  const svg=sv("svg",{viewBox:"0 0 600 320",role:"img","aria-label":"Top view. The camera is at the bottom centre. Ten ground-truth points and ten predicted points are drawn after both clouds were normalized to mean distance one. The size of each predicted point shows the confidence the loss prefers for it"});
  const st=h("div",{class:"stats"});
  const draw=()=>{ const r=w19conf(S,scale,sh,alpha); svg.innerHTML="";
    svg.append(sv("path",{d:"M300 300 L286 280 L314 280 z",fill:"var(--ink)"}),txt(322,298,"camera 1 (origin)",{class:"mut","font-size":11}),
      sv("path",{d:"M"+X(-1)+" "+Y(0)+" A "+sc+" "+sc+" 0 0 1 "+X(1)+" "+Y(0),fill:"none",stroke:"var(--line2)","stroke-dasharray":"4 5"}),txt(X(-1)+6,Y(0)-8,"distance 1",{class:"mut","font-size":11}));
    r.Gn.forEach((p,k)=>{ const q=r.Xn[k], col=S.hard[k]?"var(--c3)":"var(--c5)";
      svg.append(sv("line",{x1:X(p[0]),y1:Y(p[1]),x2:X(q[0]),y2:Y(q[1]),stroke:"var(--c2)","stroke-width":2}),sv("circle",{cx:X(q[0]),cy:Y(q[1]),r:(3+2.6*Math.log(r.C[k])).toFixed(1),fill:col,"fill-opacity":0.55,stroke:col}),
        sv("rect",{x:X(p[0])-3.5,y:Y(p[1])-3.5,width:7,height:7,fill:"var(--surface)",stroke:"var(--ink)","stroke-width":1.5})); });
    const ce=r.C.filter((v,k)=>!S.hard[k]), ch=r.C.filter((v,k)=>S.hard[k]);
    st.innerHTML=""; st.append(stat("Raw error, no normalization",r.raw.toFixed(2)+" m","mean ‖X − X̂‖"),stat("Mean ℓ after normalization",r.mean.toFixed(3),"equation (13.15)"),
      stat("Preferred confidence C",(w19sum(ce)/ce.length).toFixed(1)+" / "+(w19sum(ch)/ch.length).toFixed(1),"mean over easy / hard points"),stat("Loss (13.17) at those C",r.Lc.toFixed(2),"with all C = 1 it is "+w19sum(r.l).toFixed(2))); };
  el.append(h("div",{class:"vt"},"Scale-normalized regression and confidence"),h("p",{class:"vd"},"Ten points seen from above. The prediction is the truth plus an error, times a global scale. Both clouds are divided by their mean distance to the camera before they are compared."),
    h("div",{class:"ctl"},range("Scale of the prediction",0.5,3,0.1,scale,v=>{ scale=v; draw(); },v=>"× "+v.toFixed(1)).el,range("Error on the 3 hard points",0,0.8,0.05,sh,v=>{ sh=v; draw(); },v=>v.toFixed(2)+" m").el,range("α",0.02,0.6,0.02,alpha,v=>{ alpha=v; draw(); },v=>v.toFixed(2)).el),svg,
    w19leg([["var(--ink)","ground truth (squares)"],["var(--c5)","prediction, easy point"],["var(--c3)","prediction, hard point"],["var(--c2)","error ℓ"]]),st);
  draw();
};
