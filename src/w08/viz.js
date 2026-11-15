// Week 8 · dense maps. One 2-D scene, given as an exact signed distance function, feeds every widget of the week.
const WX=12, WY=6, SC=50;                                   // world size (m) and pixels per metre; y points down, like the screen
const sdBox=(x,y,cx,cy,hx,hy)=>{ const qx=Math.abs(x-cx)-hx, qy=Math.abs(y-cy)-hy; return Math.hypot(Math.max(qx,0),Math.max(qy,0))+Math.min(Math.max(qx,qy),0); };
const ROOM=[6,3,5.375,2.375], BOX=[3.5,2.125,0.875,0.5], THIN=[6.125,3.375,0.05,1], DISK=[8.9,3.6,0.8];   // surfaces sit mid-cell on the 25 cm grid
// room walls (negative inside the wall), a box, a 10 cm thin wall, a round pillar. The shapes do not touch, so the min is the exact distance.
const scene=(x,y)=>Math.min(-sdBox(x,y,...ROOM),sdBox(x,y,...BOX),sdBox(x,y,...THIN),Math.hypot(x-DISK[0],y-DISK[1])-DISK[2]);
// range along a ray by sphere tracing: stepping by the distance to the nearest surface can never jump through one
const cast=(ox,oy,a)=>{ const c=Math.cos(a), s=Math.sin(a); let t=0; for(let i=0;i<200;i++){ const d=scene(ox+t*c,oy+t*s); if(d<1e-4) break; t+=d; } return t; };
const DASH={fill:"none",stroke:"var(--c3)","stroke-width":1.5,"stroke-dasharray":"5 4"};
const truth=()=>{ const R=b=>sv("rect",Object.assign({x:(b[0]-b[2])*SC,y:(b[1]-b[3])*SC,width:2*b[2]*SC,height:2*b[3]*SC},DASH));
  return [R(ROOM),R(BOX),R(THIN),sv("circle",Object.assign({cx:DISK[0]*SC,cy:DISK[1]*SC,r:DISK[2]*SC},DASH))]; };
const legend=items=>h("div",{class:"legend"},...items.map(([c,t])=>h("span",{},h("i",{style:"background:"+c}),t)));
const px=v=>(v*SC).toFixed(1);
const segPath=S=>S.map(([p,q])=>"M"+px(p[0])+" "+px(p[1])+"L"+px(q[0])+" "+px(q[1])).join("");
// Marching squares over samples f(i,j), i=0..nx, j=0..ny (NaN = no data), negative = inside. Sample (i,j) sits at (o+i*s, o+j*s).
// Returns line segments [[x,y],[x,y]] in world units. An edge is crossed when its two ends differ in sign; the crossing is
// placed by linear interpolation (lerp) or at the midpoint.
const msq=(nx,ny,f,lerp,s,o)=>{ const S=[];
  for(let j=0;j<ny;j++) for(let i=0;i<nx;i++){ const a=f(i,j), b=f(i+1,j), c=f(i+1,j+1), d=f(i,j+1); if(isNaN(a+b+c+d)) continue;
    const X=(p,q,x0,y0,x1,y1)=>{ if((p<0)===(q<0)) return null; const t=lerp?p/(p-q):.5; return [o+s*(x0+t*(x1-x0)),o+s*(y0+t*(y1-y0))]; };
    const top=X(a,b,i,j,i+1,j), right=X(b,c,i+1,j,i+1,j+1), bot=X(d,c,i,j+1,i+1,j+1), left=X(a,d,i,j,i,j+1), P=[top,right,bot,left].filter(Boolean);
    if(P.length===2) S.push(P);
    else if(P.length===4){ if(((a+b+c+d)<0)===(a<0)) S.push([top,right],[bot,left]); else S.push([left,top],[right,bot]); } }   // saddle: the cell's mean value decides
  return S; };

// Days 1 and 2: a robot with a 360° range scanner builds a log-odds occupancy grid (tsdf=false) or a TSDF (tsdf=true).
const mapper=(el,tsdf)=>{
  const D=tsdf?.125:.25, NX=WX/D, NY=WY/D, N=NX*NY, noise=rng(8);
  const A=new Float64Array(N), Wt=new Float64Array(N);       // A: log-odds l, or the TSDF value; Wt: TSDF weight
  let ro=[4.6,4.2], last=ro, pr=tsdf?[28,22]:[14,10], nb=tsdf?240:120, sig=tsdf?.05:.03, ph=.7, pm=.4, tau=.5, scans=0, z=[], play=false, raf=0, ti=0, fr=0;
  const scan=()=>{ z=[]; for(let j=0;j<nb;j++) z.push(Math.max(.05,cast(ro[0],ro[1],2*Math.PI*j/nb)+sig*noise.n()));
    if(tsdf){   // projection-based integrator: visit every cell, look up the beam that points at it, average the truncated projective distance
      for(let cy=0;cy<NY;cy++) for(let cx=0;cx<NX;cx++){ const dx=(cx+.5)*D-ro[0], dy=(cy+.5)*D-ro[1], j=Math.round(Math.atan2(dy,dx)/(2*Math.PI)*nb+nb)%nb, d=z[j]-Math.hypot(dx,dy);
        if(d<-tau) continue; const k=cy*NX+cx; A[k]=(Wt[k]*A[k]+Math.min(d,tau))/(Wt[k]+1); Wt[k]++; } }
    else {      // ray-tracing integrator: every cell gets at most one update per scan, "occupied" wins over "free"
      const m=new Int8Array(N), lo=Math.log(ph/(1-ph)), lf=Math.log(pm/(1-pm));
      z.forEach((zj,j)=>{ const a=2*Math.PI*j/nb, c=Math.cos(a), s=Math.sin(a), cell=t=>{ const cx=Math.floor((ro[0]+t*c)/D), cy=Math.floor((ro[1]+t*s)/D); return cx>=0&&cy>=0&&cx<NX&&cy<NY?cy*NX+cx:-1; };
        // ponytail: samples the ray every D/4 instead of an exact grid traversal (Day 4), so it can skip a cell the ray only clips at a corner
        for(let t=0;t<zj;t+=D/4){ const k=cell(t); if(k>=0&&!m[k]) m[k]=1; } const k=cell(zj); if(k>=0) m[k]=2; });
      for(let k=0;k<N;k++) if(m[k]) A[k]=Math.max(-4,Math.min(4,A[k]+(m[k]===2?lo:lf))); }
    scans++; last=ro; };
  const svg=sv("svg",{viewBox:"0 0 "+WX*SC+" "+WY*SC,role:"img","aria-label":tsdf?"Top view of a room. Each grid cell is coloured by its fused truncated signed distance, and a line marks the zero crossing":"Top view of a room. Each grid cell is shaded by its occupancy probability: solid occupied, clear free, half tone unknown"});
  const gc=sv("g",{"shape-rendering":"crispEdges"}), cells=[];
  for(let k=0;k<N;k++){ const c=sv("rect",{x:(k%NX)*D*SC,y:Math.floor(k/NX)*D*SC,width:D*SC,height:D*SC,fill:"var(--ink)"}); cells.push(c); gc.append(c); }
  const rays=sv("path",{fill:"none",stroke:"var(--acc)","stroke-width":.6,opacity:.3}), ends=sv("path",{fill:"none",stroke:"var(--acc)","stroke-width":3,"stroke-linecap":"round"}),
    zc=sv("path",{fill:"none",stroke:"var(--ink)","stroke-width":2.2,"stroke-linecap":"round"}),
    probe=sv("g",{},sv("circle",{r:30,fill:"none","pointer-events":"all"}),sv("rect",{x:-8,y:-8,width:16,height:16,fill:"none",stroke:"var(--c1)","stroke-width":3})),
    robot=sv("g",{},sv("circle",{r:30,fill:"none","pointer-events":"all"}),sv("circle",{r:8,fill:"var(--acc)",stroke:"var(--surface)","stroke-width":2}));
  svg.append(gc,rays,...truth(),...(tsdf?[zc]:[]),ends,probe,robot);
  const st=h("div",{class:"stats"});
  const draw=()=>{
    for(let k=0;k<N;k++){ const c=cells[k];
      if(!tsdf) c.setAttribute("fill-opacity",(.9/(1+Math.exp(-2*A[k]))).toFixed(2));   // shade = probability with the log-odds doubled, for contrast
      else if(!Wt[k]){ c.setAttribute("fill","var(--muted)"); c.setAttribute("fill-opacity",.25); }
      else { c.setAttribute("fill",A[k]<0?"var(--c5)":"var(--c2)"); c.setAttribute("fill-opacity",Math.min(.9,.1+.8*Math.abs(A[k])/tau).toFixed(2)); } }
    const [ox,oy]=last, E=z.map((r,j)=>[ox+r*Math.cos(2*Math.PI*j/z.length),oy+r*Math.sin(2*Math.PI*j/z.length)]);
    rays.setAttribute("d",E.map(e=>"M"+px(ox)+" "+px(oy)+"L"+px(e[0])+" "+px(e[1])).join(""));
    ends.setAttribute("d",E.map(e=>"M"+px(e[0])+" "+px(e[1])+"h.01").join(""));
    robot.setAttribute("transform","translate("+px(ro[0])+" "+px(ro[1])+")"); probe.setAttribute("transform","translate("+px((pr[0]+.5)*D)+" "+px((pr[1]+.5)*D)+")");
    const k=pr[1]*NX+pr[0], med=a=>a.sort((x,y)=>x-y)[a.length>>1]*100;
    st.innerHTML="";
    if(tsdf){ const S=msq(NX-1,NY-1,(i,j)=>Wt[j*NX+i]?A[j*NX+i]:NaN,true,D,D/2); zc.setAttribute("d",segPath(S));
      const e=[]; S.forEach(([p,q])=>e.push(Math.abs(scene(p[0],p[1])),Math.abs(scene(q[0],q[1]))));
      st.append(stat("Scans fused",scans),stat("Probe cell (green square)",Wt[k]?A[k].toFixed(2)+" m":"no data","weight "+Wt[k]+" · true distance "+scene((pr[0]+.5)*D,(pr[1]+.5)*D).toFixed(2)+" m"),
        stat("Zero crossing: typical error",e.length?med(e).toFixed(1)+" cm":"—","median distance to the true surface"),stat("Raw end points: typical error",med(E.map(p=>Math.abs(scene(p[0],p[1])))).toFixed(1)+" cm","median, last scan alone")); }
    else { let o=0, f=0; for(let i=0;i<N;i++){ if(A[i]>0) o++; else if(A[i]<0) f++; }
      st.append(stat("Scans fused",scans),stat("Probe cell (green square)","p = "+(1/(1+Math.exp(-A[k]))).toFixed(2),"log-odds l = "+A[k].toFixed(2)),stat("Occupied / free / unknown",o+" / "+f+" / "+(N-o-f),"of "+N+" cells")); } };
  const reset=()=>{ A.fill(0); Wt.fill(0); scans=0; scan(); draw(); };
  drag(svg,robot,(x,y)=>{ x=Math.max(.7,Math.min(WX-.7,x/SC)); y=Math.max(.7,Math.min(WY-.7,y/SC)); if(scene(x,y)<.15) return;
    ro=[x,y]; if(Math.hypot(x-last[0],y-last[1])>=.3) scan(); draw(); });
  drag(svg,probe,(x,y)=>{ pr=[Math.max(0,Math.min(NX-1,Math.floor(x/SC/D))),Math.max(0,Math.min(NY-1,Math.floor(y/SC/D)))]; draw(); });
  // the tour: a fixed polyline through free space, one scan every 0.5 m
  const stops=[]; [[1.5,4.7],[1.5,1],[5.2,1],[5.2,5],[7.2,5],[7.2,1.2],[10.6,1.2],[10.6,4.8]].forEach((p,i,T)=>{ if(!i) return stops.push(p);
    const q=T[i-1], n=Math.round(Math.hypot(p[0]-q[0],p[1]-q[1])/.5); for(let s=1;s<=n;s++) stops.push([q[0]+(p[0]-q[0])*s/n,q[1]+(p[1]-q[1])*s/n]); });
  const stop=()=>{ play=false; cancelAnimationFrame(raf); pb.textContent="Auto tour"; pb.classList.remove("on"); };
  const tick=()=>{ if(!play) return; if(fr++%4===0){ ro=stops[ti++]; scan(); draw(); if(ti>=stops.length){ ti=0; return stop(); } } raf=requestAnimationFrame(tick); };
  const pb=button("Auto tour",()=>{ if(play) return stop(); play=true; fr=0; pb.textContent="Pause"; pb.classList.add("on"); raf=requestAnimationFrame(tick); });
  const pct=v=>v.toFixed(2);
  el.append(h("div",{class:"vt"},tsdf?"TSDF fusion in a small room":"Occupancy grid in a small room"),
    h("p",{class:"vd"},(tsdf?"Each scan adds one truncated projective distance to every cell it sees, by a running average. ":"Each scan adds log-odds to the cells its beams cross (free) and end in (occupied). ")+"Drag the robot (dot) to scan as it moves; drag the green square to inspect a cell. Cells are "+D*100+" cm."),
    h("div",{class:"ctl"},range("Beams per scan",30,360,30,nb,v=>{ nb=v; }).el,range("Range noise σ",0,10,1,sig*100,v=>{ sig=v/100; },v=>v+" cm").el,
      ...(tsdf?[range("Truncation τ",.1,1,.05,tau,v=>{ tau=v; reset(); },v=>v.toFixed(2)+" m").el]
        :[range("p(occupied | beam ends here)",.55,.95,.05,ph,v=>{ ph=v; },pct).el,range("p(occupied | beam passes)",.05,.45,.05,pm,v=>{ pm=v; },pct).el])),
    h("div",{class:"ctl"},button("Scan here",()=>{ scan(); draw(); },"primary"),pb,button("Reset map",()=>{ stop(); ti=0; reset(); })),svg,
    legend(tsdf?[["var(--c2)","positive: in front of a surface"],["var(--c5)","negative: behind it"],["var(--muted)","never updated"],["var(--ink)","zero crossing"],["var(--c3)","true surfaces (dashed)"]]
      :[["var(--ink)","solid: occupied · clear: free · half tone: unknown"],["var(--acc)","beams of the last scan"],["var(--c3)","true surfaces (dashed)"]]),st);
  reset();
};
VIZ.w08_occ=el=>mapper(el,false);
VIZ.w08_tsdf=el=>mapper(el,true);

// Day 3: marching squares on a sampled signed distance function.
VIZ.w08_march=el=>{
  const SP=[2,1,.5,.25,.125]; let si=1, lerp=true, shape="disk", R=2.1;
  const f=(x,y)=>shape==="disk"?Math.hypot(x-6,y-3)-R:scene(x,y);
  const svg=sv("svg",{viewBox:"0 0 "+WX*SC+" "+WY*SC,role:"img","aria-label":"A grid of signed distance samples, the outline that marching squares extracts from them, and the true outline dashed"}), st=h("div",{class:"stats"});
  svg.style.overflow="visible";
  const draw=()=>{ const s=SP[si], nx=WX/s, ny=WY/s, F=[]; for(let j=0;j<=ny;j++) for(let i=0;i<=nx;i++) F.push(f(i*s,j*s));
    const v=(i,j)=>F[j*(nx+1)+i], S=msq(nx,ny,v,lerp,s,0); svg.innerHTML="";
    if(s>=.5){ for(let i=0;i<=nx;i++) svg.append(sv("line",{class:"grid",x1:px(i*s),x2:px(i*s),y1:0,y2:WY*SC})); for(let j=0;j<=ny;j++) svg.append(sv("line",{class:"grid",y1:px(j*s),y2:px(j*s),x1:0,x2:WX*SC})); }
    svg.append(...(shape==="disk"?[sv("circle",Object.assign({cx:6*SC,cy:3*SC,r:R*SC},DASH))]:truth()),sv("path",{d:segPath(S),fill:"none",stroke:"var(--ink)","stroke-width":2.5,"stroke-linecap":"round"}));
    if(s>=.5) F.forEach((val,k)=>svg.append(sv("circle",{cx:px(k%(nx+1)*s),cy:px(Math.floor(k/(nx+1))*s),r:s>=1?4.5:3,fill:val<0?"var(--c5)":"var(--surface)",stroke:val<0?"var(--c5)":"var(--c2)","stroke-width":2})));
    if(s>=1) for(let j=0;j<ny;j++) for(let i=0;i<nx;i++){ const cs=msq(1,1,(a,b)=>v(i+a,j+b),lerp,s,0); if(!cs.length) continue;
      const code=(v(i,j)<0)+2*(v(i+1,j)<0)+4*(v(i+1,j+1)<0)+8*(v(i,j+1)<0), mx=(cs[0][0][0]+cs[0][1][0]-s)/2, my=(cs[0][0][1]+cs[0][1][1]-s)/2, n=Math.hypot(mx,my)||1;   // label on the side away from the line
      svg.append(txt(px((i+.5)*s-.27*s*mx/n),+px((j+.5)*s-.27*s*my/n)+4,String(code),{"text-anchor":"middle","font-size":12,"font-weight":700,fill:"var(--c4)",stroke:"var(--surface)","stroke-width":3,"paint-order":"stroke","data-top":1})); }
    let e=0, len=0; S.forEach(([p,q])=>{ e+=Math.abs(f(p[0],p[1]))+Math.abs(f(q[0],q[1])); len+=Math.hypot(p[0]-q[0],p[1]-q[1]); });
    st.innerHTML=""; st.append(stat("Samples",(nx+1)*(ny+1),(nx+1)+" × "+(ny+1)+", spacing "+s+" m"),stat("Line segments",S.length),
      stat("Mean vertex error",S.length?(e/2/S.length*100).toFixed(1)+" cm":"—","distance of the vertices to the true outline"),
      stat("Outline length",len.toFixed(2)+" m",shape==="disk"?"true circle: "+(2*Math.PI*R).toFixed(2)+" m":"true outlines: "+(4*(ROOM[2]+ROOM[3]+BOX[2]+BOX[3]+THIN[2]+THIN[3])+2*Math.PI*DISK[2]).toFixed(2)+" m")); };
  const lb=button("Interpolation: on",()=>{ lerp=!lerp; lb.classList.toggle("on",lerp); lb.textContent="Interpolation: "+(lerp?"on":"off"); draw(); }); lb.classList.add("on");
  el.append(h("div",{class:"vt"},"Marching squares"),h("p",{class:"vd"},"A signed distance function is sampled at the grid corners (filled dot: negative, inside). Each cell with mixed signs gets a line piece; the number is the cell's case, 1 + 2 + 4 + 8 for the inside corners taken clockwise from the top left."),
    h("div",{class:"ctl"},select("Shape",[["disk","Disk"],["room","Room of Days 1–2"]],v=>{ shape=v; draw(); }).el,range("Grid spacing",0,4,1,si,v=>{ si=v; draw(); },v=>SP[v]+" m").el,range("Disk radius",.3,2.8,.1,R,v=>{ R=v; draw(); },v=>v.toFixed(1)+" m").el),
    h("div",{class:"ctl"},lb),svg,legend([["var(--ink)","extracted outline"],["var(--c3)","true outline (dashed)"],["var(--c5)","sample inside (negative)"]]),st);
  draw();
};

// Day 4: a quadtree over the same room. A node is split only if a surface can lie inside it.
VIZ.w08_quad=el=>{ let depth=6;
  // |distance at the centre| > half the diagonal proves the whole square is on one side (the distance changes by at most 1 m per metre).
  // The test is conservative: it may split a square that the surface just misses, never the other way round.
  const build=dmax=>{ const L=[]; const rec=(x,y,e,d)=>{ const v=scene(x+e/2,y+e/2), clear=Math.abs(v)>e*Math.SQRT1_2;
      if(clear||d===dmax) return L.push([x,y,e,clear?(v>0?0:1):2]); const g=e/2; rec(x,y,g,d+1); rec(x+g,y,g,d+1); rec(x,y+g,g,d+1); rec(x+g,y+g,g,d+1); };
    rec(0,0,6,0); rec(6,0,6,0); return L; };
  const svg=sv("svg",{viewBox:"0 0 "+WX*SC+" "+WY*SC,role:"img","aria-label":"The room divided by a quadtree: large squares in open space and inside walls, small squares only along the surfaces"}), st=h("div",{class:"stats"});
  const FILL=["none","var(--muted)","var(--c2)"];
  const draw=()=>{ const L=build(depth), prev=build(depth-1).length, dense=2*4**depth; svg.innerHTML="";
    L.forEach(([x,y,e,t])=>svg.append(sv("rect",{x:px(x),y:px(y),width:px(e),height:px(e),fill:FILL[t],"fill-opacity":t===1?.45:.8,stroke:"var(--line2)","stroke-width":.6})));
    svg.append(...truth());
    st.innerHTML=""; st.append(stat("Finest cell",(600/2**depth).toFixed(1)+" cm","depth "+depth+" below two 6 m roots"),stat("Quadtree leaves",L.length,"× "+(L.length/prev).toFixed(2)+" from depth "+(depth-1)+" · plus "+(L.length-2)/3+" inner nodes"),
      stat("Dense grid cells",dense,"× 4.00 from depth "+(depth-1)),stat("Leaves ÷ dense cells",(100*L.length/dense).toFixed(1)+" %")); };
  el.append(h("div",{class:"vt"},"Quadtree refinement"),h("p",{class:"vd"},"The 2-D version of an octree. A square is split into four only while a surface can pass through it, down to the chosen depth."),
    h("div",{class:"ctl"},range("Maximum depth",1,7,1,depth,v=>{ depth=v; draw(); }).el),svg,
    legend([["var(--c2)","finest cells that may contain a surface"],["var(--muted)","solid"],["var(--line2)","free (no fill)"],["var(--c3)","true surfaces (dashed)"]]),st);
  draw();
};

// Day 5: Gaussian process regression of a 1-D profile, equations (5.10) and (5.11) of the book.
VIZ.w08_gp=el=>{ let ell=1, sn=.1, J=12, seed=1; const SF=.7, W=600, H=300, X=x=>40+x*54, Y=y=>150-y*65;
  const f=x=>.6*Math.sin(.9*x)+.3*Math.sin(2.3*x+1), k=(a,b)=>SF*SF*Math.exp(-(a-b)*(a-b)/(2*ell*ell));
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"A curve with noisy samples, the Gaussian process mean through them, and a band of two standard deviations that widens where there are no samples"}), st=h("div",{class:"stats"});
  const draw=()=>{ const r=rng(seed), xs=[], ys=[]; for(let j=0;j<J;j++){ const u=r.u()*7.6, x=u<4.2?u:u+2.4; xs.push(x); ys.push(f(x)+sn*r.n()); }   // nothing is sampled between 4.2 m and 6.6 m
    const Ki=M.inv(xs.map((a,i)=>xs.map((b,j)=>k(a,b)+(i===j?sn*sn:0)))); svg.innerHTML=""; st.innerHTML="";
    if(!Ki) return st.append(h("p",{class:"warn"},"The kernel matrix is numerically singular for these settings. Raise the noise."));
    const al=M.mv(Ki,ys), T=[]; let se=0, smax=0, xmax=0;
    for(let i=0;i<=100;i++){ const x=i/10, ks=xs.map(a=>k(x,a)), m=ks.reduce((s,v,j)=>s+v*al[j],0), q=M.mv(Ki,ks), sd=Math.sqrt(Math.max(0,SF*SF-ks.reduce((s,v,j)=>s+v*q[j],0)));
      T.push([x,m,sd]); se+=(m-f(x))**2; if(sd>smax){ smax=sd; xmax=x; } }
    const P=(g,rev)=>(rev?[...T].reverse():T).map(t=>X(t[0]).toFixed(1)+" "+Y(g(t)).toFixed(1)).join(" L");
    svg.append(sv("rect",{x:X(4.2),y:20,width:X(6.6)-X(4.2),height:260,fill:"var(--soft)"}),txt(X(5.4),36,"no samples here",{"text-anchor":"middle",class:"mut","font-size":11}),
      sv("line",{class:"axis",x1:X(0),x2:X(10),y1:Y(0),y2:Y(0)}),sv("path",{d:"M"+P(t=>t[1]+2*t[2])+" L"+P(t=>t[1]-2*t[2],true)+"Z",fill:"var(--c5-soft)"}),
      sv("path",{d:"M"+T.map(t=>X(t[0]).toFixed(1)+" "+Y(f(t[0])).toFixed(1)).join(" L"),fill:"none",stroke:"var(--c3)","stroke-width":1.5,"stroke-dasharray":"5 4"}),
      sv("path",{d:"M"+P(t=>t[1]),fill:"none",stroke:"var(--c5)","stroke-width":2.5}),...xs.map((x,j)=>sv("circle",{cx:X(x),cy:Y(ys[j]),r:4,fill:"var(--ink)",stroke:"var(--surface)","stroke-width":1.5})));
    [0,2,4,6,8,10].forEach(v=>svg.append(txt(X(v),H-6,v+" m",{"text-anchor":"middle",class:"mut","font-size":11})));
    st.append(stat("Training points J",J,"one "+J+" × "+J+" matrix to invert"),stat("RMS error of the mean",(100*Math.sqrt(se/101)).toFixed(1)+" cm","against the true curve"),
      stat("Largest predicted std. dev.",(100*smax).toFixed(1)+" cm","at x = "+xmax.toFixed(1)+" m · prior σ is "+(100*SF).toFixed(0)+" cm")); };
  el.append(h("div",{class:"vt"},"Gaussian process regression of a wall profile"),h("p",{class:"vd"},"Noisy samples of an unknown curve, a squared-exponential kernel, and the predictive mean and ±2σ band from equations (5.10) and (5.11)."),
    h("div",{class:"ctl"},range("Kernel length-scale ℓ",.2,3,.1,ell,v=>{ ell=v; draw(); },v=>v.toFixed(1)+" m").el,range("Measurement noise σ",.01,.5,.01,sn,v=>{ sn=v; draw(); },v=>(100*v).toFixed(0)+" cm").el,range("Training points J",3,30,1,J,v=>{ J=v; draw(); }).el),
    h("div",{class:"ctl"},button("New samples",()=>{ seed++; draw(); })),svg,legend([["var(--c5)","GP mean, band = ±2σ"],["var(--c3)","true curve (dashed)"],["var(--ink)","samples"]]),st);
  draw();
};
