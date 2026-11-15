// Week 1 · Day 1: odometry drift around a square loop, and what one loop closure does to it.
VIZ.w01_drift=el=>{
  const N=40, SIDE=10, W=600, H=350, sc=21, ox=195, oy=285;          // world: x right, y up, one unit per step
  let sth=1, sd=3, seed=3, closed=false;
  const X=p=>(ox+p[0]*sc).toFixed(1), Y=p=>(oy-p[1]*sc).toFixed(1);
  const path=(P,a)=>sv("path",Object.assign({d:P.map((p,i)=>(i?"L":"M")+X(p)+" "+Y(p)).join(" "),fill:"none","stroke-width":2.5,"stroke-linejoin":"round"},a));
  const walk=noisy=>{ const r=rng(seed), P=[[0,0]]; let th=0, x=0, y=0;
    for(let k=0;k<N;k++){ th+=(k>0&&k%SIDE===0?Math.PI/2:0)+(noisy?r.n()*sth*Math.PI/180:0); const d=1+(noisy?r.n()*sd/100:0); x+=d*Math.cos(th); y+=d*Math.sin(th); P.push([x,y]); }
    return P; };
  const meanErr=(P,Q)=>P.reduce((s,p,i)=>s+Math.hypot(p[0]-Q[i][0],p[1]-Q[i][1]),0)/P.length;
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Top view of a robot driving a square loop: the true path, the path from odometry alone, and the path after a loop closure"});
  const st=h("div",{class:"stats"}), lg=h("div",{class:"legend"});
  const draw=()=>{ const T=walk(false), O=walk(true), e=[O[N][0]-O[0][0],O[N][1]-O[0][1]];
    // position-only back-end: equally weighted odometry steps plus the hard constraint p_N = p_0 has the closed-form
    // least-squares solution "subtract k/N of the end gap from pose k"
    const C=O.map((p,k)=>[p[0]-k/N*e[0],p[1]-k/N*e[1]]);
    svg.innerHTML=""; svg.append(path(T,{stroke:"var(--line2)","stroke-dasharray":"6 5"}),path(O,{stroke:"var(--c3)",opacity:closed?0.35:1}));
    if(closed) svg.append(path(C,{stroke:"var(--c1)"}));
    else svg.append(sv("line",{x1:X(O[N]),y1:Y(O[N]),x2:X(O[0]),y2:Y(O[0]),stroke:"var(--c2)","stroke-width":2,"stroke-dasharray":"3 4"}));
    svg.append(sv("circle",{cx:X(O[0]),cy:Y(O[0]),r:6,fill:"var(--ink)"}),txt(+X(O[0])-10,+Y(O[0])+22,"start",{"text-anchor":"end",class:"mut"}),
      sv("circle",{cx:X((closed?C:O)[N]),cy:Y((closed?C:O)[N]),r:5,fill:closed?"var(--c1)":"var(--c3)",stroke:"var(--surface)","stroke-width":2}));
    st.innerHTML=""; st.append(stat("Gap between start and end",Math.hypot(e[0],e[1]).toFixed(2)+" steps","odometry only"),
      stat("Mean position error",meanErr(O,T).toFixed(2)+" steps","odometry only"),
      stat("Mean position error",closed?meanErr(C,T).toFixed(2)+" steps":"—",closed?"after the loop closure":"press Close the loop"));
    lg.innerHTML=""; [["var(--line2)","true path"],["var(--c3)","odometry only"],closed?["var(--c1)","after loop closure"]:["var(--c2)","gap a loop closure would measure"]].forEach(([c,t])=>lg.append(h("span",{},h("i",{style:"background:"+c}),t))); };
  const cb=button("Close the loop",()=>{ closed=!closed; cb.classList.toggle("on",closed); cb.textContent=closed?"Loop closed ✓":"Close the loop"; draw(); });
  el.append(h("div",{class:"vt"},"Drift around a square"),h("p",{class:"vd"},"40 odometry steps of one unit each, with a 90° turn at each corner. The same noise is used for every curve until you press New noise."),
    h("div",{class:"ctl"},range("Heading noise per step",0,3,0.1,sth,v=>{ sth=v; draw(); },v=>v.toFixed(1)+"°").el,range("Distance noise per step",0,10,0.5,sd,v=>{ sd=v; draw(); },v=>v+"%").el),
    h("div",{class:"ctl"},cb,button("New noise",()=>{ seed++; draw(); })),svg,lg,st);
  draw();
};
