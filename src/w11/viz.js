// Week 11 widgets: trajectory metrics (ATE/RPE), depth from parallax, lens distortion, Harris corners, two-view geometry.
const w11leg=(lg,items)=>{ lg.innerHTML=""; items.forEach(([c,t])=>lg.append(h("span",{},h("i",{style:"background:"+c}),t))); };
const w11clamp=(v,a,b)=>Math.max(a,Math.min(b,v));

// Day 1: absolute trajectory error (after rigid alignment) against relative pose error, on a planar path.
VIZ.w11_traj=el=>{
  const N=60, W=600, H=340, sc=11, ox=215, oy=318, D2R=Math.PI/180;
  let sth=0.3, kink=10, dl=1, seed=5;
  const turn=k=>(k>=15&&k<20)||(k>=35&&k<40)?Math.PI/10:0;                 // two 90° left turns, 5 steps each
  const walk=noisy=>{ const r=rng(seed), P=[[0,0,0]]; let x=0, y=0, th=0;
    for(let k=0;k<N;k++){ const nt=r.n(), nd=r.n(); th+=turn(k)+(noisy?nt*sth*D2R+(k===10?kink*D2R:0):0); const d=1+(noisy?nd*0.01:0);
      x+=d*Math.cos(th); y+=d*Math.sin(th); P.push([x,y,th]); } return P; };
  // closed-form 2-D rigid alignment (rotation + translation) of the estimate onto the ground truth
  const align=(P,Q)=>{ const n=P.length, mp=[0,0], mq=[0,0]; P.forEach((p,i)=>{ mp[0]+=p[0]/n; mp[1]+=p[1]/n; mq[0]+=Q[i][0]/n; mq[1]+=Q[i][1]/n; });
    let s=0, c=0; P.forEach((p,i)=>{ const a=[p[0]-mp[0],p[1]-mp[1]], b=[Q[i][0]-mq[0],Q[i][1]-mq[1]]; c+=a[0]*b[0]+a[1]*b[1]; s+=a[0]*b[1]-a[1]*b[0]; });
    const f=Math.atan2(s,c), cf=Math.cos(f), sf=Math.sin(f);
    return P.map(p=>[cf*(p[0]-mp[0])-sf*(p[1]-mp[1])+mq[0], sf*(p[0]-mp[0])+cf*(p[1]-mp[1])+mq[1]]); };
  const rel=(P,i,j)=>{ const c=Math.cos(P[i][2]), s=Math.sin(P[i][2]), dx=P[j][0]-P[i][0], dy=P[j][1]-P[i][1]; return [c*dx+s*dy,-s*dx+c*dy,P[j][2]-P[i][2]]; };
  const X=p=>(ox+p[0]*sc).toFixed(1), Y=p=>(oy-p[1]*sc).toFixed(1);
  const path=(P,a)=>sv("path",Object.assign({d:P.map((p,i)=>(i?"L":"M")+X(p)+" "+Y(p)).join(" "),fill:"none","stroke-width":2.5,"stroke-linejoin":"round"},a));
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Top view of a ground-truth path with two left turns, the estimated path as integrated, and the estimated path after rigid alignment to the ground truth, with error segments between matching poses"});
  const st=h("div",{class:"stats"}), lg=h("div",{class:"legend"});
  const draw=()=>{ const Q=walk(false), P=walk(true), A=align(P,Q);
    const ate=Math.sqrt(A.reduce((s,a,i)=>s+(a[0]-Q[i][0])**2+(a[1]-Q[i][1])**2,0)/A.length);
    let et=0, er=0, m=0; for(let i=0;i+dl<=N;i++){ const a=rel(P,i,i+dl), b=rel(Q,i,i+dl); et+=(a[0]-b[0])**2+(a[1]-b[1])**2; er+=(a[2]-b[2])**2; m++; }
    svg.innerHTML=""; svg.append(path(Q,{stroke:"var(--line2)","stroke-dasharray":"6 5"}),path(P,{stroke:"var(--c3)",opacity:0.35}));
    for(let i=4;i<=N;i+=4) svg.append(sv("line",{x1:X(A[i]),y1:Y(A[i]),x2:X(Q[i]),y2:Y(Q[i]),stroke:"var(--c2)","stroke-width":1.5}));
    svg.append(path(A,{stroke:"var(--c5)"}),sv("circle",{cx:X(Q[0]),cy:Y(Q[0]),r:5,fill:"var(--ink)"}),txt(+X(Q[0])-8,+Y(Q[0])+4,"start",{"text-anchor":"end",class:"mut"}));
    st.innerHTML=""; st.append(stat("ATE (RMSE)",ate.toFixed(2)+" m","after rigid alignment"),
      stat("RPE translation (RMSE)",Math.sqrt(et/m).toFixed(3)+" m","per "+dl+" m travelled"),
      stat("RPE rotation (RMSE)",(Math.sqrt(er/m)/D2R).toFixed(2)+"°","per "+dl+" m travelled"));
    w11leg(lg,[["var(--line2)","ground truth"],["var(--c3)","estimate, as integrated"],["var(--c5)","estimate, aligned"],["var(--c2)","position error (every 4th pose)"]]); };
  el.append(h("div",{class:"vt"},"Two ways to score a trajectory"),h("p",{class:"vd"},"60 odometry steps of 1 m. The estimate integrates noisy steps (distance noise fixed at 1%), plus one sudden heading error at step 10."),
    h("div",{class:"ctl"},range("Heading noise per step",0,2,0.1,sth,v=>{ sth=v; draw(); },v=>v.toFixed(1)+"°").el,range("One-off heading error at step 10",0,20,1,kink,v=>{ kink=v; draw(); },v=>v+"°").el,
      range("RPE interval Δ",1,30,1,dl,v=>{ dl=v; draw(); },v=>v+" steps").el),
    h("div",{class:"ctl"},button("New noise",()=>{ seed++; draw(); })),svg,lg,st);
  draw();
};

// Day 2: depth of a point triangulated from two noisy pixel measurements, as a function of baseline and depth.
VIZ.w11_parallax=el=>{
  const F=500, W=600, H=370, sc=9, sx=60, ox=300, oy=350, NS=300;      // sideways axis stretched (sx) so the baseline is visible
  let b=0.5, Z=20, sp=1;
  const X=x=>ox+sx*x, Y=z=>oy-sc*z;
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Top view of two cameras side by side observing one point. A cloud of triangulated positions from noisy pixel measurements is stretched along the depth direction."});
  const st=h("div",{class:"stats"}), lg=h("div",{class:"legend"});
  const cam=x=>sv("circle",{cx:X(x),cy:Y(0),r:5,fill:"var(--c4)"});
  const draw=()=>{ const r=rng(7), zs=[]; svg.innerHTML="";
    for(let z=0;z<=35;z+=10) svg.append(sv("line",{x1:40,y1:Y(z),x2:W-10,y2:Y(z),class:"grid"}),txt(8,Y(z)+4,z+" m",{class:"mut","font-size":11}));
    svg.append(sv("line",{x1:X(-b/2),y1:Y(0),x2:X(0),y2:Y(Z),stroke:"var(--c4)","stroke-width":1.2}),sv("line",{x1:X(b/2),y1:Y(0),x2:X(0),y2:Y(Z),stroke:"var(--c4)","stroke-width":1.2}));
    for(let i=0;i<NS;i++){ const u1=F*(b/2)/Z+r.n()*sp, u2=-F*(b/2)/Z+r.n()*sp, d=u1-u2; if(d<=0.02) continue;
      const z=F*b/d, x=z*(u1+u2)/(2*F); zs.push(z); if(z<37) svg.append(sv("circle",{cx:X(x).toFixed(1),cy:Y(z).toFixed(1),r:1.8,fill:"var(--c5)",opacity:0.55})); }
    const sz=Z*Z*Math.SQRT2*sp/(F*b); zs.sort((p,q)=>p-q);
    const q=f=>zs.length?zs[Math.min(zs.length-1,Math.floor(f*zs.length))]:NaN, spread=(q(0.84)-q(0.16))/2;
    svg.append(sv("line",{x1:X(0)+40,y1:Y(Math.min(37,Z+sz)),x2:X(0)+40,y2:Y(Math.max(0,Z-sz)),stroke:"var(--c2)","stroke-width":4,"stroke-linecap":"round"}),
      txt(X(0)+50,Y(Z)+4,"±1σ predicted",{class:"mut","font-size":11}),sv("circle",{cx:X(0),cy:Y(Z),r:4,fill:"var(--c3)"}),cam(-b/2),cam(b/2));
    st.innerHTML=""; st.append(stat("Disparity d = f·b/z",(F*b/Z).toFixed(1)+" px","f = 500 px"),stat("Predicted depth σ",sz.toFixed(2)+" m","z²·σ_d / (f·b)"),
      stat("Measured depth spread",isFinite(spread)?spread.toFixed(2)+" m":"—","middle 68% of "+zs.length+" valid samples"),stat("Parallax angle",(2*Math.atan(b/(2*Z))*180/Math.PI).toFixed(2)+"°","between the two rays"));
    w11leg(lg,[["var(--c4)","cameras and viewing rays"],["var(--c3)","true point"],["var(--c5)","300 triangulations with pixel noise"],["var(--c2)","predicted ±1σ in depth"]]); };
  el.append(h("div",{class:"vt"},"Depth from two views"),h("p",{class:"vd"},"Each camera measures the point's pixel column with Gaussian noise. Every pair of noisy measurements is triangulated; the dots are the results. The sideways axis is stretched about 7 times so that the baseline is visible."),
    h("div",{class:"ctl"},range("Baseline b",0.1,4,0.1,b,v=>{ b=v; draw(); },v=>v.toFixed(1)+" m").el,range("True depth z",5,30,1,Z,v=>{ Z=v; draw(); },v=>v+" m").el,
      range("Pixel noise σ",0.2,2,0.1,sp,v=>{ sp=v; draw(); },v=>v.toFixed(1)+" px").el),svg,lg,st);
  draw();
};

// Day 3: pinhole projection with radial distortion (k1, k2), and undistortion by fixed-point iteration.
VIZ.w11_lens=el=>{
  const IW=640, IH=480, U0=320, V0=240, s=0.62, W=600, H=Math.round(IH*s)+20;
  let f=400, k1=-0.2, k2=0.02;
  const dist=(x,y)=>{ const r2=x*x+y*y, g=1+k1*r2+k2*r2*r2; return [x*g,y*g]; };
  const pix=(x,y)=>[(f*x+U0)*s,(f*y+V0)*s];
  const undist=(xd,yd)=>{ let x=xd, y=yd; for(let n=1;n<=100;n++){ const r2=x*x+y*y, g=1+k1*r2+k2*r2*r2; if(Math.abs(g)<1e-9) return null;
      const nx=xd/g, ny=yd/g, e=Math.hypot(nx-x,ny-y); x=nx; y=ny; if(e<1e-6) return n; } return null; };
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"A 640 by 480 pixel image of a square grid on a wall. The dashed grid is the ideal pinhole image; the solid grid is the image after radial lens distortion."});
  const st=h("div",{class:"stats"}), lg=h("div",{class:"legend"});
  const draw=()=>{ const im=sv("svg",{x:(W-IW*s)/2,y:10,width:IW*s,height:IH*s,viewBox:"0 0 "+IW*s+" "+IH*s}); svg.innerHTML="";
    im.append(sv("rect",{x:0,y:0,width:IW*s,height:IH*s,fill:"var(--soft)",stroke:"var(--line2)"}));
    const line=(g,a)=>{ const pts=[]; for(let t=0;t<=24;t++) pts.push(g(t/24)); im.append(sv("path",Object.assign({d:pts.map((p,i)=>(i?"L":"M")+p[0].toFixed(1)+" "+p[1].toFixed(1)).join(" "),fill:"none"},a))); };
    for(let pass=0;pass<2;pass++){ const a=pass?{stroke:"var(--c5)","stroke-width":1.8}:{stroke:"var(--line2)","stroke-width":1,"stroke-dasharray":"4 4"}, map=pass?dist:(x,y)=>[x,y];
      for(let i=-4;i<=4;i++) line(t=>pix(...map(i*0.2,-0.6+1.2*t)),a);
      for(let j=-3;j<=3;j++) line(t=>pix(...map(-0.8+1.6*t,j*0.2)),a); }
    const c=dist(0.8,0.6), a0=pix(0.8,0.6), a1=pix(...c);
    im.append(sv("circle",{cx:U0*s,cy:V0*s,r:3.5,fill:"var(--c3)"}),sv("circle",{cx:a1[0].toFixed(1),cy:a1[1].toFixed(1),r:4,fill:"var(--c2)"})); svg.append(im);
    const shift=Math.hypot(a1[0]-a0[0],a1[1]-a0[1])/s, inward=Math.hypot(c[0],c[1])<1, it=undist(c[0],c[1]);
    st.innerHTML=""; st.append(stat("Horizontal field of view",(2*Math.atan(U0/f)*180/Math.PI).toFixed(1)+"°","2·atan(320 / f), pinhole"),
      stat("Shift of grid corner",shift.toFixed(1)+" px",shift<0.05?"no distortion":inward?"inward: barrel":"outward: pincushion"),
      stat("Undistortion iterations",it==null?"no convergence":String(it),"for that corner, to 10⁻⁶"));
    w11leg(lg,[["var(--line2)","ideal pinhole image of the grid"],["var(--c5)","with radial distortion"],["var(--c3)","principal point (u₀, v₀)"],["var(--c2)","grid corner x′ = 0.8, y′ = 0.6"]]); };
  el.append(h("div",{class:"vt"},"Pinhole plus radial distortion"),h("p",{class:"vd"},"A grid with lines every 0.2 in normalized coordinates x′ = x/z, y′ = y/z, projected into a 640 × 480 image with principal point (320, 240)."),
    h("div",{class:"ctl"},range("Focal length f",300,600,10,f,v=>{ f=v; draw(); },v=>v+" px").el,range("k₁",-0.25,0.25,0.01,k1,v=>{ k1=v; draw(); },v=>v.toFixed(2)).el,
      range("k₂",-0.03,0.1,0.01,k2,v=>{ k2=v; draw(); },v=>v.toFixed(2)).el),svg,lg,st);
  draw();
};

// Day 4: second-moment matrix of a 5×5 patch, its eigenvalues, and the Shi-Tomasi / Harris corner tests.
VIZ.w11_harris=el=>{
  const NX=32, NY=24, C=9, GAP=24, W=600, H=NY*C+26, TAU=0.05, KH=0.04, LOC={flat:[0,0],edge:[0,-5],corner:[8,5]};
  let ang=20, con=0.7, px=0, py=0, which="corner", I=null;
  const image=()=>{ const a=ang*Math.PI/180, ca=Math.cos(a), sa=Math.sin(a), A=M.zeros(NY,NX);
    for(let y=0;y<NY;y++) for(let x=0;x<NX;x++){ const u=x+0.5-16, v=y+0.5-12, sd=Math.max(Math.abs(ca*u+sa*v)-8,Math.abs(-sa*u+ca*v)-5);   // distance outside a 16 × 10 rectangle
      A[y][x]=0.15+con/(1+Math.exp(sd/0.6)); } return A; };                                                                              // soft edge, about 2 px wide
  const tensor=(cx,cy)=>{ let a=0, b=0, d=0; for(let dy=-2;dy<=2;dy++) for(let dx=-2;dx<=2;dx++){ const x=cx+dx, y=cy+dy, gx=(I[y][x+1]-I[y][x-1])/2, gy=(I[y+1][x]-I[y-1][x])/2; a+=gx*gx; b+=gx*gy; d+=gy*gy; }
    return [[a,b],[b,d]]; };
  const eig=T=>{ const e=M.eigSym(T).val; return [Math.max(0,e[1]),Math.max(0,e[0])]; };       // [larger, smaller]
  const place=()=>{ if(!which) return; const a=ang*Math.PI/180, l=LOC[which]; px=Math.floor(Math.cos(a)*l[0]-Math.sin(a)*l[1]+16); py=Math.floor(Math.sin(a)*l[0]+Math.cos(a)*l[1]+12); };
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Left: a small grey-level image of a rotated rectangle with a movable 5 by 5 probe window. Right: the smaller eigenvalue of the second-moment matrix at every pixel, which is large only at the four corners."});
  const g=sv("g"), probe=sv("rect",{width:5*C,height:5*C,fill:"var(--c2)","fill-opacity":0.18,stroke:"var(--c2)","stroke-width":2.5});
  const st=h("div",{class:"stats"}), out=h("div",{class:"out"}), lg=h("div",{class:"legend"});
  const info=()=>{ px=w11clamp(px,3,NX-4); py=w11clamp(py,3,NY-4); probe.setAttribute("x",(px-2)*C); probe.setAttribute("y",(py-2)*C+20);
    const T=tensor(px,py), [l1,l2]=eig(T), R=l1*l2-KH*(l1+l2)**2;
    out.textContent="M = [ "+T[0][0].toFixed(3)+"  "+T[0][1].toFixed(3)+" ;  "+T[1][0].toFixed(3)+"  "+T[1][1].toFixed(3)+" ]   at pixel ("+px+", "+py+")";
    st.innerHTML=""; st.append(stat("Larger eigenvalue λ₁",l1.toFixed(3)),stat("Smaller eigenvalue λ₂",l2.toFixed(3),"Shi-Tomasi score"),stat("Harris R",R.toFixed(4),"λ₁λ₂ − 0.04 (λ₁+λ₂)²"),
      stat("Verdict",l2>TAU?"corner":l1>TAU?"edge":"flat","threshold τ = "+TAU)); };
  const draw=()=>{ I=image(); place(); g.innerHTML=""; const L=M.zeros(NY,NX); let mx=1e-9;
    for(let y=3;y<NY-3;y++) for(let x=3;x<NX-3;x++){ L[y][x]=eig(tensor(x,y))[1]; mx=Math.max(mx,L[y][x]); }
    for(let y=0;y<NY;y++) for(let x=0;x<NX;x++){ g.append(sv("rect",{x:x*C,y:y*C+20,width:C,height:C,fill:"var(--ink)","fill-opacity":I[y][x].toFixed(3)}));
      if(L[y][x]>mx*0.02) g.append(sv("rect",{x:NX*C+GAP+x*C,y:y*C+20,width:C,height:C,fill:"var(--c1)","fill-opacity":(L[y][x]/mx).toFixed(3)})); }
    for(let y=3;y<NY-3;y++) for(let x=3;x<NX-3;x++){ if(L[y][x]<=TAU) continue; let top=true;
      for(let dy=-2;dy<=2&&top;dy++) for(let dx=-2;dx<=2;dx++){ const v=(L[y+dy]||[])[x+dx]; if(v>L[y][x]||(v===L[y][x]&&(dy<0||(dy===0&&dx<0)))){ top=false; break; } }
      if(top) g.append(sv("circle",{cx:(x+0.5)*C,cy:(y+0.5)*C+20,r:7,fill:"none",stroke:"var(--c3)","stroke-width":2.5})); }
    g.append(sv("rect",{x:0.5,y:20,width:NX*C-1,height:NY*C,fill:"none",stroke:"var(--line2)"}),sv("rect",{x:NX*C+GAP,y:20,width:NX*C-1,height:NY*C,fill:"none",stroke:"var(--line2)"}),
      txt(0,13,"image I(u, v)",{class:"mut","font-size":12}),txt(NX*C+GAP,13,"smaller eigenvalue λ₂ at each pixel",{class:"mut","font-size":12}));
    info(); };
  const jump=w=>button(w[0].toUpperCase()+w.slice(1),()=>{ which=w; place(); info(); });
  svg.append(g,probe); drag(svg,probe,(x,y)=>{ which=null; px=Math.floor(x/C); py=Math.floor((y-20)/C); info(); });
  el.append(h("div",{class:"vt"},"Flat, edge, or corner?"),h("p",{class:"vd"},"The amber window is a 5 × 5 patch. Its second-moment matrix M sums the products of image gradients over the patch. Drag the window, or jump to a preset."),
    h("div",{class:"ctl"},range("Rotate the shape",0,90,5,ang,v=>{ ang=v; draw(); },v=>v+"°").el,range("Contrast",0.1,0.8,0.05,con,v=>{ con=v; draw(); },v=>v.toFixed(2)).el),
    h("div",{class:"ctl"},jump("flat"),jump("edge"),jump("corner")),svg,out,st,lg);
  w11leg(lg,[["var(--c2)","5 × 5 probe window (drag it)"],["var(--c3)","detected: local maximum of λ₂ above τ"],["var(--c1)","size of λ₂"]]);
  draw();
};

// Day 5: two pinhole cameras. Epipolar line l2 = F p1 of a point picked in image 1, and reprojection residuals in image 2.
VIZ.w11_twoview=el=>{
  const F=300, IW=280, IH=200, U0=140, V0=100, sc=18, ox=300, oy=188, W=600, H=440, D2R=Math.PI/180;
  const P=[[-2,-0.8,6],[-0.9,0.7,5],[0.3,-0.3,7.5],[1.4,0.6,5.5],[2.2,-0.9,8],[-1.6,0.2,8.5],[0.9,-1,6.2],[-0.2,0.9,9]];   // points in camera-1 (= world) coordinates: x right, y down, z forward
  let c2=[2.5,0,0], yaw=-12, dz=0, eyaw=0, sel=2;
  const Rot=a=>{ const c=Math.cos(a), s=Math.sin(a); return [[c,0,-s],[0,1,0],[s,0,c]]; };      // world → camera 2 for a camera yawed by a about the y axis
  const toC2=(x,a)=>M.mv(Rot(a),[x[0]-c2[0],x[1]-c2[1],x[2]-c2[2]]);
  const proj=x=>x[2]>0.2?[F*x[0]/x[2]+U0,F*x[1]/x[2]+V0]:null;
  const X=x=>ox+sc*x, Z=z=>oy-sc*z;
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Top: top view of two cameras and eight 3-D points, with the viewing ray of the selected point. Bottom: the two images. The epipolar line of the selected point is drawn in image 2, together with observed and predicted pixel positions."});
  const g=sv("g"), handle=sv("circle",{r:13,fill:"var(--c2)","fill-opacity":0.25,stroke:"var(--c2)","stroke-width":2});
  const st=h("div",{class:"stats"}), lg=h("div",{class:"legend"});
  const frustum=(c,a,col)=>{ const d=[-1,1].map(s=>[c[0]+1.3*Math.sin(a+s*25*D2R),c[2]+1.3*Math.cos(a+s*25*D2R)]);
    return sv("path",{d:"M"+X(c[0])+" "+Z(c[2])+" L"+X(d[0][0])+" "+Z(d[0][1])+" L"+X(d[1][0])+" "+Z(d[1][1])+" z",fill:col,"fill-opacity":0.35,stroke:col,"stroke-width":1.5}); };
  const panel=(x,label)=>{ const im=sv("svg",{x,y:232,width:IW,height:IH,viewBox:"0 0 "+IW+" "+IH}); im.append(sv("rect",{x:0,y:0,width:IW,height:IH,fill:"var(--soft)",stroke:"var(--line2)"}));
    g.append(txt(x,226,label,{class:"mut","font-size":12}),im); return im; };
  const draw=()=>{ g.innerHTML=""; const a=yaw*D2R, R=Rot(a), t=M.mv(R,c2).map(v=>-v);                 // x² = R x¹ + t
    const E=M.mul([[0,-t[2],t[1]],[t[2],0,-t[0]],[-t[1],t[0],0]],R), ps=P[sel], xe=ps.map(v=>v*(ps[2]+dz)/ps[2]);
    // top view
    g.append(txt(8,16,"top view: x to the right, z forward",{class:"mut","font-size":12}),sv("line",{x1:X(0),y1:Z(0),x2:X(c2[0]),y2:Z(c2[2]),stroke:"var(--line2)","stroke-width":1.5,"stroke-dasharray":"4 4"}),
      sv("line",{x1:X(0),y1:Z(0),x2:X(ps[0]*10/ps[2]),y2:Z(10),stroke:"var(--c3)","stroke-width":1.5}),sv("line",{x1:X(c2[0]),y1:Z(c2[2]),x2:X(ps[0]),y2:Z(ps[2]),stroke:"var(--c3)","stroke-width":1.2,"stroke-dasharray":"3 3"}),
      frustum([0,0,0],0,"var(--c4)"),frustum(c2,a,"var(--c2)"),txt(X(0)-12,Z(0)-2,"camera 1",{"text-anchor":"end","font-size":12}),txt(X(c2[0])+18,Z(c2[2])+4,"camera 2 (drag)",{"font-size":12}));
    P.forEach((p,j)=>g.append(sv("circle",{cx:X(p[0]),cy:Z(p[2]),r:j===sel?5.5:4,fill:j===sel?"var(--c3)":"var(--c5)"})));
    if(dz) g.append(sv("circle",{cx:X(xe[0]),cy:Z(xe[2]),r:5.5,fill:"none",stroke:"var(--c2)","stroke-width":2}));
    // image 1
    const i1=panel(10,"image 1"), i2=panel(310,"image 2");
    P.forEach((p,j)=>{ const z=proj(p); i1.append(sv("circle",{cx:z[0].toFixed(1),cy:z[1].toFixed(1),r:j===sel?7:5,fill:j===sel?"var(--c3)":"var(--c5)"}));
      const hit=sv("circle",{cx:z[0].toFixed(1),cy:z[1].toFixed(1),r:17,fill:"var(--ink)","fill-opacity":0}); hit.style.cursor="pointer"; hit.addEventListener("click",()=>{ sel=j; draw(); }); i1.append(hit); });
    // epipolar line in image 2: l = K⁻ᵀ E K⁻¹ p1
    const z1=proj(ps), e=M.mv(E,[(z1[0]-U0)/F,(z1[1]-V0)/F,1]), l=[e[0]/F,e[1]/F,e[2]-(U0*e[0]+V0*e[1])/F], ln=Math.hypot(l[0],l[1]), ok=ln>1e-9;
    if(ok){ const q=Math.abs(l[1])>Math.abs(l[0])?[[-20,-(l[2]-20*l[0])/l[1]],[IW+20,-(l[2]+(IW+20)*l[0])/l[1]]]:[[-(l[2]-20*l[1])/l[0],-20],[-(l[2]+(IH+20)*l[1])/l[0],IH+20]];
      i2.append(sv("line",{x1:q[0][0].toFixed(1),y1:q[0][1].toFixed(1),x2:q[1][0].toFixed(1),y2:q[1][1].toFixed(1),stroke:"var(--c3)","stroke-width":1.5})); }
    const dist=z=>ok?Math.abs(l[0]*z[0]+l[1]*z[1]+l[2])/ln:NaN;
    let cost=0, n=0, own=NaN, eown=NaN, lo=Infinity, hi=-Infinity;
    P.forEach((p,j)=>{ const zo=proj(toC2(p,a)), zp=proj(toC2(j===sel?xe:p,a+eyaw*D2R)); if(!zo) return;
      if(zp){ const r2=(zo[0]-zp[0])**2+(zo[1]-zp[1])**2; cost+=r2/2; n++; if(j===sel) eown=Math.sqrt(r2);
        if(r2>0.01) i2.append(sv("line",{x1:zo[0].toFixed(1),y1:zo[1].toFixed(1),x2:zp[0].toFixed(1),y2:zp[1].toFixed(1),stroke:"var(--c2)","stroke-width":2}),sv("circle",{cx:zp[0].toFixed(1),cy:zp[1].toFixed(1),r:5,fill:"var(--surface)",stroke:"var(--c2)","stroke-width":2})); }
      i2.append(sv("circle",{cx:zo[0].toFixed(1),cy:zo[1].toFixed(1),r:j===sel?7:5,fill:j===sel?"var(--c3)":"var(--c5)"}));
      const d=dist(zo); if(j===sel) own=d; else { lo=Math.min(lo,d); hi=Math.max(hi,d); } });
    const ep=proj(t); if(ep&&ok) i2.append(sv("path",{d:"M"+(ep[0]-5).toFixed(1)+" "+ep[1].toFixed(1)+" h10 M"+ep[0].toFixed(1)+" "+(ep[1]-5).toFixed(1)+" v10",stroke:"var(--ink)","stroke-width":2}));
    handle.setAttribute("cx",X(c2[0])); handle.setAttribute("cy",Z(c2[2]));
    const px=v=>isFinite(v)?v.toFixed(1)+" px":"—";
    st.innerHTML=""; st.append(stat("Selected point to its line",ok?px(own):"no baseline","|p₂ᵀ F p₁| as a distance"),stat("Other points to that line",isFinite(lo)&&ok?lo.toFixed(0)+" to "+hi.toFixed(0)+" px":"—","nearest to farthest"),
      stat("Residual of selected point",px(eown),"‖z − π(·)‖ in image 2"),stat("E_reproj, image 2",cost.toFixed(1),"½ Σ ‖e‖², σ = 1 px, "+n+" points"));
    w11leg(lg,[["var(--c3)","selected point, its ray and epipolar line"],["var(--c5)","observed pixels"],["var(--c2)","predicted pixels and residuals"],["var(--ink)","+ epipole (camera 1 seen from camera 2)"]]); };
  const reset=()=>{ c2=[2.5,0,0]; yaw=-12; dz=0; eyaw=0; sel=2; s1.input.value=yaw; s2.input.value=dz; s3.input.value=eyaw; [s1,s2,s3].forEach(s=>s.input.dispatchEvent(new Event("input"))); };
  const s1=range("Camera 2 yaw",-40,40,1,yaw,v=>{ yaw=v; draw(); },v=>v+"°"), s2=range("Depth error of selected point",-3,3,0.1,dz,v=>{ dz=v; draw(); },v=>(v>0?"+":"")+v.toFixed(1)+" m"),
    s3=range("Yaw error in camera 2's estimated pose",-5,5,0.1,eyaw,v=>{ eyaw=v; draw(); },v=>(v>0?"+":"")+v.toFixed(1)+"°");
  svg.append(g,handle); drag(svg,handle,(x,y)=>{ c2=[w11clamp((x-ox)/sc,-7,7),0,w11clamp((oy-y)/sc,-1.5,4)]; draw(); });
  el.append(h("div",{class:"vt"},"Two views of the same points"),h("p",{class:"vd"},"Both cameras have f = 300 px. Tap a point in image 1 (or press Next point) to select it. Image 2 shows where each point is observed and where the current estimate predicts it."),
    h("div",{class:"ctl"},s1.el,s2.el,s3.el),h("div",{class:"ctl"},button("Next point",()=>{ sel=(sel+1)%P.length; draw(); }),button("Reset",reset,"ghost")),svg,lg,st);
  draw();
};
