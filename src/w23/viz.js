// Week 23 · open-world spatial AI.
// Every "embedding" in this file is hand-built on eight readable attribute axes. None of them comes from CLIP or any trained model.
const AT=["seat","soft","screen","cold","drink","green","flat top","floor"];
const unit=v=>{ const n=Math.hypot(...v)||1; return v.map(x=>x/n); };
const dot=(a,b)=>a.reduce((s,x,i)=>s+x*b[i],0);
const vec=o=>unit(AT.map(a=>o[a]||0));
const OBJ={sofa:vec({seat:1,soft:1}),armchair:vec({seat:1,soft:.5}),chair:vec({seat:1,"flat top":.25}),tv:vec({screen:1}),table:vec({"flat top":1}),
  plant:vec({green:1}),fridge:vec({cold:1,drink:.4}),can:vec({drink:1,cold:.4}),floor:vec({floor:1})};
const show=v=>v.map((x,i)=>x>.005?AT[i]+" "+x.toFixed(2):"").filter(Boolean).join(" · ");

// Day 1: symmetric InfoNCE on four image-caption pairs whose embeddings live on the unit circle; plain gradient descent on the angles.
VIZ.w23_contrast=el=>{
  const N=4, CX=140, CY=150, R=100, MX=330, MY=60, MS=50;
  let tau=0.2, seed=18, th, ph, steps, playing=false, left=0;
  const init=()=>{ const r=rng(seed); th=[]; ph=[]; for(let i=0;i<N;i++){ th.push(r.u()*2*Math.PI); ph.push(r.u()*2*Math.PI); } steps=0; };
  const calc=()=>{ const S=th.map(a=>ph.map(b=>Math.cos(a-b))), E=S.map(r=>r.map(s=>Math.exp(s/tau)));
    const rs=E.map(r=>r.reduce((a,b)=>a+b,0)), cs=ph.map((_,j)=>E.reduce((a,r)=>a+r[j],0));
    const P=E.map((r,i)=>r.map(e=>e/rs[i])), Q=E.map(r=>r.map((e,j)=>e/cs[j]));       // softmax along each row, and along each column
    let loss=0; for(let i=0;i<N;i++) loss-=(Math.log(P[i][i])+Math.log(Q[i][i]))/(2*N);
    return {S,P,Q,loss}; };
  const step=()=>{ const {P,Q}=calc(), dth=Array(N).fill(0), dph=Array(N).fill(0);
    for(let i=0;i<N;i++) for(let j=0;j<N;j++){ const g=(P[i][j]+Q[i][j]-(i===j?2:0))/(2*N*tau), s=Math.sin(th[i]-ph[j]);   // g = dL/dS_ij; dS_ij/dth_i = -sin, dS_ij/dph_j = +sin
      dth[i]-=g*s; dph[j]+=g*s; }
    const lr=1.2*tau;                                                                    // step scaled by tau so the motion per step is similar at every temperature
    for(let i=0;i<N;i++){ th[i]-=lr*dth[i]; ph[i]-=lr*dph[i]; } steps++; };
  const svg=sv("svg",{viewBox:"0 0 600 300",role:"img","aria-label":"Left: four image embeddings and four caption embeddings on a unit circle, coloured by pair. Right: the four by four matrix of cosine similarities between every image and every caption."});
  const st=h("div",{class:"stats"});
  const draw=()=>{ const {S,loss}=calc(); svg.innerHTML="";
    svg.append(sv("circle",{cx:CX,cy:CY,r:R,fill:"none",stroke:"var(--line2)","stroke-width":1.5}),txt(CX,292,"embedding space (unit circle)",{"text-anchor":"middle",class:"mut","font-size":11}));
    for(let i=0;i<N;i++){ const ax=CX+R*Math.cos(th[i]), ay=CY-R*Math.sin(th[i]), bx=CX+R*Math.cos(ph[i]), by=CY-R*Math.sin(ph[i]);
      svg.append(sv("line",{x1:CX,y1:CY,x2:ax,y2:ay,stroke:COL[i],"stroke-width":1,opacity:.5}),sv("line",{x1:CX,y1:CY,x2:bx,y2:by,stroke:COL[i],"stroke-width":1,opacity:.5,"stroke-dasharray":"3 3"}),
        sv("circle",{cx:ax,cy:ay,r:8,fill:COL[i]}),sv("rect",{x:bx-7,y:by-7,width:14,height:14,fill:"var(--surface)",stroke:COL[i],"stroke-width":3})); }
    for(let i=0;i<N;i++){ svg.append(txt(MX-10,MY+i*MS+MS/2+4,"I"+(i+1),{"text-anchor":"end",fill:COL[i],"font-weight":700}),txt(MX+i*MS+MS/2,MY-8,"T"+(i+1),{"text-anchor":"middle",fill:COL[i],"font-weight":700}));
      for(let j=0;j<N;j++){ svg.append(sv("rect",{x:MX+j*MS,y:MY+i*MS,width:MS-2,height:MS-2,rx:5,fill:"var(--acc)","fill-opacity":(0.85*(S[i][j]+1)/2).toFixed(2),stroke:i===j?"var(--ink)":"var(--line)","stroke-width":i===j?2:1}),
        txt(MX+j*MS+MS/2-1,MY+i*MS+MS/2+4,S[i][j].toFixed(2),{"text-anchor":"middle","font-size":12,fill:"var(--ink)"})); } }
    svg.append(txt(MX+2*MS,MY+4*MS+18,"cosine similarity, image i with caption j",{"text-anchor":"middle",class:"mut","font-size":11}));
    let dg=0, off=0; for(let i=0;i<N;i++) for(let j=0;j<N;j++){ if(i===j) dg+=S[i][j]/N; else off+=S[i][j]/(N*N-N); }
    st.innerHTML=""; st.append(stat("Gradient steps",String(steps)),stat("Loss",loss.toFixed(3),"all-equal similarities give ln 4 = 1.386"),stat("Mean similarity, true pairs",dg.toFixed(2),"the diagonal"),stat("Mean similarity, wrong pairs",off.toFixed(2),"off the diagonal")); };
  const pb=button("Play",()=>{ playing=!playing; pb.textContent=playing?"Pause":"Play"; if(playing){ left=120; requestAnimationFrame(tick); } },"primary");
  const tick=()=>{ if(!playing) return; step(); draw(); if(--left>0) requestAnimationFrame(tick); else { playing=false; pb.textContent="Play"; } };
  const stop=()=>{ playing=false; pb.textContent="Play"; };
  el.append(h("div",{class:"vt"},"Contrastive learning on four image-caption pairs"),h("p",{class:"vd"},"Filled dots are image embeddings, hollow squares are caption embeddings, one colour per pair. Each step is one gradient-descent step on the symmetric InfoNCE loss."),
    h("div",{class:"ctl"},range("Temperature τ",0.05,1,0.05,tau,v=>{ tau=v; draw(); },v=>v.toFixed(2)).el),
    h("div",{class:"ctl"},pb,button("Step",()=>{ stop(); step(); draw(); }),button("Reset",()=>{ stop(); init(); draw(); }),button("New start",()=>{ stop(); seed++; init(); draw(); })),
    svg,h("div",{class:"legend"},h("span",{},"● image embedding"),h("span",{},"□ caption embedding")),st);
  init(); draw();
};

// Day 2: the zero-shot classifier of equation (17.1) with a class set the reader edits.
VIZ.w23_zeroshot=el=>{
  const IMG=[["armchair","an armchair"],["can","a soda can"],["plant","a potted plant"],["table","a table"],["fridge","a refrigerator"]];
  const CLS=[["sofa",OBJ.sofa,true],["television",OBJ.tv,true],["plant",OBJ.plant,true],["chair",vec({seat:1,soft:.3}),false],["drink",vec({drink:1,cold:.3}),false],["table",OBJ.table,false]];
  let img="armchair"; const on=CLS.map(c=>c[2]);
  const bars=h("div",{class:"bars"}), out=h("div",{class:"out"});
  const draw=()=>{ const f=OBJ[img], S=CLS.map((c,i)=>on[i]?dot(f,c[1]):null); let best=-1; S.forEach((s,i)=>{ if(s!=null&&(best<0||s>S[best])) best=i; });
    bars.innerHTML=""; S.forEach((s,i)=>{ if(s!=null) bars.append(h("div",{class:"bar"+(i===best?" hot":"")},h("span",{},"a photo of a "+CLS[i][0]),h("div",{class:"bb"},h("i",{style:"width:"+(Math.max(0,s)*100).toFixed(0)+"%"})),h("span",{class:"bv"},s.toFixed(2)))); });
    out.textContent=best<0?"The class set is empty, so there is nothing to compare the image with.":"image feature: "+show(f)+"  →  arg max: “a photo of a "+CLS[best][0]+"”"+(S[best]<.3?"  (score "+S[best].toFixed(2)+": no class in the set fits, but arg max still returns one)":""); };
  const tg=CLS.map((c,i)=>{ const b=button(c[0],()=>{ on[i]=!on[i]; b.classList.toggle("on",on[i]); draw(); }); b.classList.toggle("on",on[i]); return b; });
  el.append(h("div",{class:"vt"},"A zero-shot classifier you can re-define"),h("p",{class:"vd"},"Toy 8-dimensional unit features, hand-built (not CLIP). Each bar is the cosine similarity between the image feature and one class prompt; the highlighted bar is the arg max of equation (17.1)."),
    h("div",{class:"ctl"},select("Image",IMG,v=>{ img=v; draw(); }).el),h("p",{class:"vd"},"Class set C (press to add or remove a prompt):"),h("div",{class:"ctl"},...tg),bars,out);
  draw();
};

// Day 3 (MAIN): open-vocabulary query on a 16 x 10 grid map. Views are fused per cell with equations (17.2)-(17.3).
VIZ.w23_query=el=>{
  const GW=16, GH=10, CS=36, OX=12, OY=8, NV=20, SIG=6, D=AT.length;
  const grid=Array.from({length:GH},()=>Array(GW).fill("floor"));
  const BLK=[], put=(name,c0,r0,c1,r1)=>{ BLK.push([c0,r0,c1,r1]); for(let r=r0;r<=r1;r++) for(let c=c0;c<=c1;c++) grid[r][c]=name; };
  put("sofa",1,1,5,2); put("armchair",1,5,2,6); put("tv",2,9,4,9); put("plant",7,0,8,1); put("table",10,3,13,5); put("can",11,4,11,4);
  put("chair",9,4,9,4); put("chair",14,4,14,4); put("chair",11,2,12,2); put("chair",11,6,12,6); put("fridge",14,8,15,9);
  const LAB=[["sofa",3.5,2],["armchair",2,6],["tv",3.5,9.5],["plant",8,1],["table",12.5,5.5],["can",11.5,4.5],["chair",9.5,4.5],["chair",14.5,4.5],["chairs",12,2.5],["chairs",12,6.5],["fridge",15,9]];
  const QS=[["a comfy place to sit",vec({seat:.8,soft:1})],["sofa",OBJ.sofa],["something to drink",vec({drink:1,cold:.3})],["television",OBJ.tv],["something green",OBJ.plant],["a place to put my cup",OBJ.table]];
  let qi=0, nv=1, s=0.6, thr=0.7, seed=11, VC, Z;
  const noise=()=>{ const r=rng(seed); VC=[]; Z=[]; for(let v=0;v<NV;v++){ VC.push([r.u()*GW,r.u()*GH]); const zv=[]; for(let k=0;k<GW*GH*D;k++) zv.push(r.n()); Z.push(zv); } };
  const fuse=(r,c)=>{ const f0=OBJ[grid[r][c]]; let f=Array(D).fill(0), ck=0;                // equations (17.2) and (17.3), starting from zero confidence
    for(let v=0;v<nv;v++){ const o=(r*GW+c)*D, obs=unit(f0.map((x,i)=>x+s*Z[v][o+i])), g=Math.hypot(c+.5-VC[v][0],r+.5-VC[v][1]), a=Math.exp(-g*g/(2*SIG*SIG));
      f=f.map((x,i)=>(ck*x+a*obs[i])/(ck+a)); ck+=a; }
    return unit(f); };
  const svg=sv("svg",{viewBox:"0 0 600 376",role:"img","aria-label":"Top view of a room as a 16 by 10 grid. Each cell is shaded by the cosine similarity between its fused feature and the query; cells above the threshold are outlined."});
  const st=h("div",{class:"stats"}), out=h("div",{class:"out"});
  const draw=()=>{ const q=QS[qi][1]; let k=0, tp=0, fn=0, mc=0; const hit=[]; svg.innerHTML="";
    for(let r=0;r<GH;r++) for(let c=0;c<GW;c++){ const f=fuse(r,c), f0=OBJ[grid[r][c]], sim=dot(f,q), want=dot(f0,q)>=thr, got=sim>=thr; mc+=dot(f,f0)/(GW*GH);
      if(got){ k++; hit.push([c,r]); if(want) tp++; } else if(want) fn++;
      svg.append(sv("rect",{x:OX+c*CS,y:OY+r*CS,width:CS,height:CS,fill:"var(--acc)","fill-opacity":(0.9*Math.max(0,sim)).toFixed(2),stroke:"var(--line)","stroke-width":.5})); }
    BLK.forEach(([c0,r0,c1,r1])=>svg.append(sv("rect",{x:OX+c0*CS,y:OY+r0*CS,width:(c1-c0+1)*CS,height:(r1-r0+1)*CS,fill:"none",stroke:"var(--muted)","stroke-width":1.3})));
    hit.forEach(([c,r])=>svg.append(sv("rect",{x:OX+c*CS+2,y:OY+r*CS+2,width:CS-4,height:CS-4,fill:"none",stroke:"var(--c2)","stroke-width":2.5,rx:3})));
    LAB.forEach(([t,x,y])=>svg.append(txt(OX+x*CS,OY+y*CS+4,t,{"text-anchor":"middle","font-size":11,fill:"var(--ink)",stroke:"var(--surface)","stroke-width":3,"paint-order":"stroke","font-weight":600})));
    out.textContent="query feature: "+show(q);
    st.innerHTML=""; st.append(stat("Cells retrieved",String(k),"similarity ≥ "+thr.toFixed(2)),stat("Correct",tp+" of "+k,"also pass with noise-free features"),stat("Missed",String(fn),"should pass, did not"),stat("Mean cosine to the clean feature",mc.toFixed(2),"over all 160 cells")); };
  el.append(h("div",{class:"vt"},"Query a feature map with words"),h("p",{class:"vd"},"Each cell stores one fused 8-dimensional feature (hand-built toy features, not CLIP). Every view adds a noisy observation of every cell, weighted by its confidence α as in equations (17.2)–(17.3). Darker = more similar to the query."),
    h("div",{class:"ctl"},select("Query",QS.map((x,i)=>[i,x[0]]),v=>{ qi=+v; draw(); }).el,range("Views fused",1,NV,1,nv,v=>{ nv=v; draw(); }).el),
    h("div",{class:"ctl"},range("Feature noise per view",0,1.2,0.1,s,v=>{ s=v; draw(); },v=>v.toFixed(1)).el,range("Similarity threshold",0.1,0.95,0.05,thr,v=>{ thr=v; draw(); },v=>v.toFixed(2)).el,button("New noise",()=>{ seed++; noise(); draw(); })),
    out,svg,h("div",{class:"legend"},h("span",{},h("i",{style:"background:var(--acc)"}),"similarity to the query"),h("span",{},h("i",{style:"background:var(--c2)"}),"retrieved (above threshold)"),h("span",{},h("i",{style:"background:var(--muted)"}),"true object outline")),st);
  noise(); draw();
};

// Day 4: agglomerative information bottleneck on a chain of 12 primitives, edge weights from equation (17.9).
VIZ.w23_aib=el=>{
  // [name, relevance to "clean backpacks", relevance to "get condiment packets"]; hand-set stand-ins for the cosine scores
  const PR=[["wall",.02,.02],["backpack strap",.80,.03],["backpack body",.85,.02],["backpack pocket",.78,.04],["chair leg",.03,.02],["chair seat",.04,.03],
    ["table top",.03,.10],["ketchup packet",.02,.85],["mustard packet",.03,.82],["napkin",.03,.20],["wall",.02,.02],["poster",.03,.02]];
  const TS=[["both","clean backpacks + get condiment packets"],["bp","clean backpacks only"],["cp","get condiment packets only"]];
  const n=PR.length; let mode="both", cl, I0, hist, last;
  const rowp=p=>mode==="both"?[p[1],p[2],1-p[1]-p[2]]:mode==="bp"?[p[1],1-p[1]]:[p[2],1-p[2]];
  const kl=(a,b)=>a.reduce((t,x,i)=>t+(x>0?x*Math.log(x/b[i]):0),0);
  const mix=(a,b)=>{ const p=a.p+b.p, wa=a.p/p, wb=b.p/p; return {m:a.m.concat(b.m),p,py:a.py.map((x,i)=>wa*x+wb*b.py[i])}; };
  const cost=(a,b)=>{ const c=mix(a,b); return a.p*kl(a.py,c.py)+b.p*kl(b.py,c.py); };          // = (p_a + p_b) * D_JS, equation (17.9)
  const info=()=>{ const py=cl[0].py.map((_,i)=>cl.reduce((t,c)=>t+c.p*c.py[i],0)); return cl.reduce((t,c)=>t+c.p*kl(c.py,py),0); };
  const reset=()=>{ cl=PR.map((p,i)=>({m:[i],p:1/n,py:rowp(p)})); I0=info(); hist=[{x:n,y:100}]; last=null; };
  const best=()=>{ let b=-1, bd=Infinity; for(let i=0;i+1<cl.length;i++){ const d=cost(cl[i],cl[i+1]); if(d<bd){ bd=d; b=i; } } return [b,bd]; };
  const merge=()=>{ if(cl.length<2) return; const [b,d]=best(); cl.splice(b,2,mix(cl[b],cl[b+1])); last=d; hist.push({x:cl.length,y:100*info()/I0}); };
  const svg=sv("svg",{viewBox:"0 0 600 330",role:"img","aria-label":"Twelve scene primitives in a column. A bar next to each shows its relevance to the tasks. Brackets on the left group the primitives into the current clusters, and the number between two clusters is the cost of merging them."});
  const st=h("div",{class:"stats"}), ch=h("div");
  const draw=()=>{ svg.innerHTML=""; const RH=25, Y0=22, [b]=cl.length>1?best():[-1], nt=mode==="both"?2:1, tc=mode==="cp"?[COL[1]]:[COL[0],COL[1]];
    svg.append(txt(34,13,"primitive",{class:"mut","font-size":11}),txt(170,13,"p(y | x): relevance to the task list",{class:"mut","font-size":11}),txt(588,13,"merge cost d (nats)",{class:"mut","font-size":11,"text-anchor":"end"}));
    cl.forEach((c,ci)=>{ const r0=c.m[0], r1=c.m[c.m.length-1], top=Math.max(...c.py.slice(0,nt)), ti=c.py.indexOf(top), col=top>0.4?tc[ti]:"var(--line2)";
      svg.append(sv("rect",{x:10,y:Y0+r0*RH+2,width:14,height:(r1-r0+1)*RH-4,rx:4,fill:col}));
      if(ci+1<cl.length){ const d=cost(c,cl[ci+1]), y=Y0+(r1+1)*RH; svg.append(sv("line",{x1:10,x2:590,y1:y,y2:y,stroke:ci===b?"var(--c3)":"var(--line)","stroke-width":ci===b?2:1,"stroke-dasharray":ci===b?"":"3 4"}),
        txt(588,y-6,d.toFixed(4)+(ci===b?"  ← cheapest":""),{"text-anchor":"end","font-size":11,fill:ci===b?"var(--c3)":"var(--muted)","font-weight":ci===b?700:400})); } });
    PR.forEach((p,i)=>{ const y=Y0+i*RH, py=rowp(p); let x=170; svg.append(txt(34,y+17,p[0],{"font-size":12,fill:"var(--ink)"}));
      py.forEach((v,j)=>{ const w=v*230; svg.append(sv("rect",{x,y:y+6,width:Math.max(0,w),height:13,fill:j<nt?tc[j]:"var(--line2)"})); x+=w; }); });
    st.innerHTML=""; st.append(stat("Clusters",String(cl.length),"started from "+n+" primitives"),stat("Task information kept",(100*info()/I0).toFixed(1)+"%","I(X̃;Y) ÷ I(X;Y)"),stat("Cost of the last merge",last==null?"—":last.toFixed(4)+" nats","information about Y lost"));
    ch.innerHTML=""; ch.append(lineChart({h:210,aria:"Task information kept, in percent, against the number of clusters left",x:{min:1,max:n,ticks:[[1,"1"],[3,"3"],[5,"5"],[7,"7"],[9,"9"],[12,"12"]],label:"clusters left (merging goes right to left)"},
      y:{min:0,max:100,ticks:[[0,"0"],[50,"50"],[100,"100"]],label:"% kept"},series:[{name:"task information kept",color:"var(--c5)",points:hist.map(p=>({x:p.x,y:p.y,label:p.x+" clusters",fmt:v=>v.toFixed(1)+"%"}))}]})); };
  el.append(h("div",{class:"vt"},"Task-driven clustering, one merge at a time"),h("p",{class:"vd"},"Twelve primitives along a wall, neighbours connected. Each press merges the pair of neighbouring clusters with the smallest weight d from equation (17.9). The relevance numbers are hand-set, not computed by CLIP."),
    h("div",{class:"ctl"},select("Task list",TS,v=>{ mode=v; reset(); draw(); }).el),
    h("div",{class:"ctl"},button("Merge cheapest pair",()=>{ merge(); draw(); },"primary"),button("Merge 5 times",()=>{ for(let i=0;i<5;i++) merge(); draw(); }),button("Reset",()=>{ reset(); draw(); })),
    svg,h("div",{class:"legend"},h("span",{},h("i",{style:"background:var(--c1)"}),"clean backpacks"),h("span",{},h("i",{style:"background:var(--c2)"}),"get condiment packets"),h("span",{},h("i",{style:"background:var(--line2)"}),"neither task")),st,ch);
  reset(); draw();
};
