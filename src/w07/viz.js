// Week 7: differentiable optimization. One toy bilevel problem is used all week:
//   lower level (back-end)   L(x,y) = ½(x²−y)² + ½(x−1)²     x: state, y: measurement produced by the front-end
//   upper level (training)   U(x*)  = ½(x*−XGT)²             XGT: ground truth
const XGT=1.5, X0=1;
const gradL=(x,y)=>2*x*(x*x-y)+x-1;                  // ∂L/∂x
const Hxx=(x,y)=>6*x*x-2*y+1;                         // ∂²L/∂x²   (∂²L/∂x∂y = −2x)
// one inner step Φ(x,y), returned with its partial derivatives: [Φ, ∂Φ/∂x, ∂Φ/∂y]
const stepGD=eta=>(x,y)=>[x-eta*gradL(x,y), 1-eta*Hxx(x,y), 2*eta*x];
const stepGN=(x,y)=>{ const a=4*x*x+1, g=gradL(x,y); return [x-g/a, 1-(Hxx(x,y)*a-g*8*x)/(a*a), 2*x/a]; };   // JᵀJ = 4x²+1
const solveLL=y=>{ let x=X0; for(let i=0;i<100;i++) x=stepGN(x,y)[0]; return x; };           // converged x*(y)
// K inner steps, then reverse-mode through the last M of them: grad = Σ_t λ_t·∂Φ_t/∂y with λ_{t−1} = λ_t·∂Φ_t/∂x
const unroll=(step,y,K,M)=>{ const xs=[X0], A=[0], B=[0];
  for(let t=1;t<=K;t++){ const [x,a,b]=step(xs[t-1],y); xs.push(x); A.push(a); B.push(b); }
  let lam=xs[K]-XGT, grad=0; for(let t=K;t>=1&&t>K-M;t--){ grad+=lam*B[t]; lam*=A[t]; }
  return {xs,grad}; };
const impGrad=(x,y)=>(x-XGT)*2*x/Hxx(x,y);            // (4.13) evaluated at x: −v·H⁻¹·∂²L/∂x∂y
const fdGrad=y=>{ const U=yy=>0.5*(solveLL(yy)-XGT)**2; return (U(y+1e-4)-U(y-1e-4))/2e-4; };
const f4=v=>Number.isFinite(v)?v.toFixed(4).replace("-","−"):"not finite";
const f3=v=>Number.isFinite(v)?v.toFixed(3).replace("-","−"):"NaN";
// round an axis range outward to "nice" tick values: → {lo, hi, ticks:[[v,label]]}
const niceAxis=(lo,hi)=>{ const raw=(hi-lo||1e-3)/4, pw=Math.pow(10,Math.floor(Math.log10(raw))), m=raw/pw, st=(m<1.5?1:m<3?2:m<7?5:10)*pw, d=Math.max(0,-Math.floor(Math.log10(st)));
  const a=Math.floor(lo/st+1e-9), b=Math.ceil(hi/st-1e-9), t=[]; for(let i=a;i<=b;i++) t.push([i*st,(i*st).toFixed(d).replace("-","−")]);
  return {lo:a*st,hi:b*st,ticks:t}; };
const toggle=(btns,i)=>btns.forEach((b,j)=>b.classList.toggle("on",i===j));

// Day 1: forward and reverse mode on the computation graph of L(x,y)
VIZ.w07_graph=el=>{
  let x=1.5, y=2, mode="fx", k=8;
  const W=600, H=250, BW=108, BH=56;
  const N={x:{cx:52,cy:160,w:90,f:"input x"}, y:{cx:52,cy:50,w:90,f:"input y"}, v1:{cx:180,cy:110,f:"v1 = x²"}, v2:{cx:312,cy:50,f:"v2 = v1 − y"},
    v3:{cx:250,cy:205,f:"v3 = x − 1"}, v4:{cx:442,cy:50,f:"v4 = ½·v2²"}, v5:{cx:442,cy:205,f:"v5 = ½·v3²"}, L:{cx:540,cy:128,f:"L = v4 + v5"}};
  const E=[["x","v1"],["x","v3"],["v1","v2"],["y","v2"],["v2","v4"],["v3","v5"],["v4","L"],["v5","L"]];
  const FW=["x","y","v1","v2","v3","v4","v5","L"], RV=["L","v5","v4","v3","v2","v1","y","x"];
  const RULE={f:{x:"seed", y:"seed", v1:"dv1 = 2x · dx", v2:"dv2 = dv1 − dy", v3:"dv3 = dx", v4:"dv4 = v2 · dv2", v5:"dv5 = v3 · dv3", L:"dL = dv4 + dv5"},
    r:{L:"seed: dL/dL = 1", v5:"dL/dv5 = dL/dL · 1", v4:"dL/dv4 = dL/dL · 1", v3:"dL/dv3 = dL/dv5 · v3", v2:"dL/dv2 = dL/dv4 · v2", v1:"dL/dv1 = dL/dv2 · 1", y:"dL/dy = dL/dv2 · (−1)", x:"dL/dx = dL/dv1 · 2x + dL/dv3 · 1"}};
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Computation graph of the lower-level cost L with the value of every node and the derivative carried by the chosen autodiff mode"});
  const out=h("div",{class:"out"}), st=h("div",{class:"stats"});
  const draw=()=>{ const v={x,y}; v.v1=x*x; v.v2=v.v1-y; v.v3=x-1; v.v4=0.5*v.v2*v.v2; v.v5=0.5*v.v3*v.v3; v.L=v.v4+v.v5;
    const rev=mode==="r", s=mode==="fy"?"y":"x", d={};
    if(rev){ d.L=1; d.v5=1; d.v4=1; d.v3=d.v5*v.v3; d.v2=d.v4*v.v2; d.v1=d.v2; d.y=-d.v2; d.x=d.v1*2*x+d.v3; }
    else { d.x=s==="x"?1:0; d.y=s==="y"?1:0; d.v1=2*x*d.x; d.v2=d.v1-d.y; d.v3=d.x; d.v4=v.v2*d.v2; d.v5=v.v3*d.v3; d.L=d.v4+d.v5; }
    const ord=rev?RV:FW, done=ord.slice(0,k), cur=k?ord[k-1]:null;
    svg.innerHTML=""; svg.append(sv("defs",{},sv("marker",{id:"w07arG",viewBox:"0 0 10 10",refX:9,refY:5,markerWidth:7,markerHeight:7,orient:"auto"},sv("path",{d:"M0 0 L10 5 L0 10 z",fill:"var(--muted)"}))));
    E.forEach(([a,b])=>{ const p=N[rev?b:a], q=N[rev?a:b], dx=q.cx-p.cx, dy=q.cy-p.cy;
      // clip the segment to the two boxes
      const clip=(n,ux,uy)=>{ const hw=(n.w||BW)/2+3, hh=BH/2+3; return Math.min(Math.abs(ux)>1e-9?hw/Math.abs(ux):1e9, Math.abs(uy)>1e-9?hh/Math.abs(uy):1e9); };
      const len=Math.hypot(dx,dy), ux=dx/len, uy=dy/len, t0=clip(p,ux,uy), t1=clip(q,ux,uy);
      svg.append(sv("line",{x1:p.cx+ux*t0,y1:p.cy+uy*t0,x2:q.cx-ux*t1,y2:q.cy-uy*t1,stroke:"var(--muted)","stroke-width":1.5,"marker-end":"url(#w07arG)"})); });
    Object.entries(N).forEach(([id,n])=>{ const w=n.w||BW, on=done.includes(id);
      svg.append(sv("rect",{x:n.cx-w/2,y:n.cy-BH/2,width:w,height:BH,rx:9,fill:id===cur?"var(--acc-soft)":on?"var(--c5-soft)":"var(--surface)",stroke:id===cur?"var(--acc)":on?"var(--c5)":"var(--line2)","stroke-width":id===cur?2.5:1.2}),
        txt(n.cx,n.cy-12,n.f,{"text-anchor":"middle","font-size":12,"font-weight":700,fill:"var(--ink)"}),
        txt(n.cx,n.cy+4,"= "+f3(v[id]),{"text-anchor":"middle","font-size":11,class:"mut"}));
      if(on) svg.append(txt(n.cx,n.cy+20,(rev?"dL/d"+id:"d"+id+"/d"+s)+": "+f3(d[id]),{"text-anchor":"middle","font-size":11,fill:"var(--ink)"})); });
    out.textContent=k===0?"Step 0: only the values are known (the ordinary evaluation of L). Move the Step slider.":
      "Step "+k+" of 8, node "+cur+":  "+(rev?RULE.r[cur]:(RULE.f[cur]==="seed"?"seed: d"+cur+"/d"+s:RULE.f[cur]))+"  =  "+f3(d[cur]);
    const full=k===8;
    st.innerHTML=""; st.append(stat("Slope in x from this pass",full&&(rev||s==="x")?f3(rev?d.x:d.L):"—","∂L/∂x, exact: "+f3(gradL(x,y))),
      stat("Slope in y from this pass",full&&(rev||s==="y")?f3(rev?d.y:d.L):"—","∂L/∂y, exact: "+f3(y-x*x)),
      stat("Passes for both partials",rev?"1":"2",rev?"reverse: one per output":"forward: one per input")); };
  const bs=[["fx","Forward, seed x"],["fy","Forward, seed y"],["r","Reverse"]].map(([m,t],i)=>button(t,()=>{ mode=m; toggle(bs,i); draw(); }));
  bs[0].classList.add("on");
  el.append(h("div",{class:"vt"},"Forward and reverse mode on one graph"),h("p",{class:"vd"},"The lower-level cost L(x, y) = ½(x² − y)² + ½(x − 1)² split into elementary operations. Each node shows its value and, once the pass reaches it, the derivative that mode carries."),
    h("div",{class:"ctl"},range("x",0.5,2,0.05,x,v=>{ x=v; draw(); },v=>v.toFixed(2)).el,range("y",0.5,3,0.05,y,v=>{ y=v; draw(); },v=>v.toFixed(2)).el,range("Step",0,8,1,k,v=>{ k=v; draw(); }).el),
    h("div",{class:"ctl"},...bs),svg,out,st);
  draw();
};

// Days 2, 3, 6: gradient of the upper-level cost by unrolling, truncated unrolling, implicit differentiation, finite differences
VIZ.w07_bilevel=el=>{
  let y=2, K=2, Mt=30, eta=0.1, solver="gd";
  const KM=30, box=h("div"), st=h("div",{class:"stats"});
  const draw=()=>{ const step=solver==="gn"?stepGN:stepGD(eta), ref=fdGrad(y), full=[], tr=[], im=[]; let cur;
    for(let k=0;k<=KM;k++){ const u=unroll(step,y,k,k), t=unroll(step,y,k,Mt), g={u:u.grad,t:t.grad,i:impGrad(u.xs[k],y),x:u.xs[k]};
      full.push({x:k,y:g.u,label:"unrolled, K = "+k,fmt:f4}); tr.push({x:k,y:g.t,label:"truncated, K = "+k,fmt:f4}); im.push({x:k,y:g.i,label:"implicit at x_K, K = "+k,fmt:f4}); if(k===K) cur=g; }
    const vals=full.concat(Mt<KM?tr:[]).map(p=>p.y).concat(ref,0).filter(Number.isFinite), v0=Math.min(...vals), v1=Math.max(...vals), pad=(v1-v0||0.01)*0.05;
    const ax=niceAxis(v0-pad,v1+pad), lo=ax.lo, hi=ax.hi, ok=p=>Number.isFinite(p.y)&&p.y>=lo&&p.y<=hi;
    const series=[{name:"unrolled through all K steps",color:"var(--c5)",points:full.filter(ok)}];
    if(Mt<KM) series.push({name:"truncated: last "+Mt+" step"+(Mt>1?"s":"")+" only",color:"var(--c2)",points:tr.filter(ok)});
    series.push({name:"implicit formula evaluated at x_K",color:"var(--c4)",points:im.filter(ok)},{name:"finite-difference reference",color:"var(--muted)",dots:false,points:[{x:0,y:ref},{x:KM,y:ref}]});
    box.innerHTML=""; box.append(lineChart({h:290,aria:"Estimates of the upper-level gradient dU/dy against the number of inner steps K",x:{min:0,max:KM,ticks:[0,5,10,15,20,25,30].map(v=>[v,""+v]),label:"inner steps K"},
      y:{min:lo,max:hi,ticks:ax.ticks},series,marker:ok({y:cur.u})?{x:K,y:cur.u}:null}));
    st.innerHTML=""; st.append(stat("Inner iterate",f4(cur.x),"x_K; converged x* = "+f4(solveLL(y))),
      stat("Unrolled gradient",f4(cur.u),"stores "+K+" iterate"+(K===1?"":"s")),
      stat("Truncated gradient",f4(cur.t),"stores "+Math.min(Mt,K)+" iterate"+(Math.min(Mt,K)===1?"":"s")),
      stat("Implicit formula",f4(cur.i),"evaluated at x_K; stores 1 iterate"),
      stat("Finite difference",f4(ref),"reference, at x*")); };
  const es=range("GD step size η",0.02,0.15,0.01,eta,v=>{ eta=v; draw(); },v=>v.toFixed(2));
  el.append(h("div",{class:"vt"},"Four ways to get dU/dy"),h("p",{class:"vd"},"Back-end: x*(y) = argmin ½(x² − y)² + ½(x − 1)², started at x₀ = 1. Training loss: U = ½(x* − 1.5)². Vertical axis: each estimate of dU/dy; horizontal axis: the number of inner steps K. Estimates far off the scale are not drawn; the tiles below always show them."),
    h("div",{class:"ctl"},range("Front-end output y",0.5,3,0.05,y,v=>{ y=v; draw(); },v=>v.toFixed(2)).el,range("Inner steps K",0,KM,1,K,v=>{ K=v; draw(); }).el,range("Backprop through last M",1,KM,1,Mt,v=>{ Mt=v; draw(); },v=>v>=KM?"all":v).el),
    h("div",{class:"ctl"},select("Inner solver",[["gd","Gradient descent"],["gn","Gauss-Newton"]],v=>{ solver=v; draw(); }).el,es.el),box,st);
  draw();
};

// Day 3: the solution curve x*(y), its slope from the implicit function theorem, Algorithm 2, and the outer loop
VIZ.w07_implicit=el=>{
  let y=2, alpha=4, etaq=0.1, kq=10, taken=0;
  const W=600, H=290, P={l:48,r:16,t:14,b:40}, Y0=0.5, Y1=3, XA=0.7, XB=1.8;
  const px=v=>P.l+(v-Y0)/(Y1-Y0)*(W-P.l-P.r), py=v=>H-P.b-(v-XA)/(XB-XA)*(H-P.t-P.b);
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"The back-end solution x-star as a function of the front-end output y, with the tangent line whose slope is given by implicit differentiation"});
  const out=h("div",{class:"out"}), st=h("div",{class:"stats"});
  const ys=range("Front-end output y",Y0,Y1,0.01,y,v=>{ y=v; taken=0; draw(); },v=>v.toFixed(3));
  const draw=()=>{ const x=solveLL(y), Hh=Hxx(x,y), s=2*x/Hh, v=x-XGT;
    svg.innerHTML="";
    [0.5,1,1.5,2,2.5,3].forEach(t=>svg.append(sv("line",{class:"grid",x1:px(t),x2:px(t),y1:P.t,y2:H-P.b}),txt(px(t),H-P.b+16,t.toFixed(1),{"text-anchor":"middle",class:"mut","font-size":11})));
    [0.8,1,1.2,1.4,1.6,1.8].forEach(t=>svg.append(sv("line",{class:"grid",x1:P.l,x2:W-P.r,y1:py(t),y2:py(t)}),txt(P.l-6,py(t)+4,t.toFixed(1),{"text-anchor":"end",class:"mut","font-size":11})));
    svg.append(txt((P.l+W-P.r)/2,H-5,"front-end output y",{"text-anchor":"middle",class:"mut","font-size":11}),txt(P.l+6,P.t+10,"back-end solution x*(y)",{class:"mut","font-size":11}));
    svg.append(sv("line",{x1:P.l,x2:W-P.r,y1:py(XGT),y2:py(XGT),stroke:"var(--c1)","stroke-width":1.5,"stroke-dasharray":"6 5"}),txt(P.l+70,py(XGT)-6,"ground truth 1.5",{"font-size":11,fill:"var(--c1)"}));
    let d=""; for(let i=0;i<=60;i++){ const yy=Y0+(Y1-Y0)*i/60; d+=(i?"L":"M")+px(yy).toFixed(1)+" "+py(solveLL(yy)).toFixed(1)+" "; }
    svg.append(sv("path",{d,fill:"none",stroke:"var(--c5)","stroke-width":2.5}));
    const a=Math.max(Y0,y-0.45), b=Math.min(Y1,y+0.45);
    svg.append(sv("line",{x1:px(a),y1:py(x+s*(a-y)),x2:px(b),y2:py(x+s*(b-y)),stroke:"var(--c3)","stroke-width":2}),
      sv("line",{x1:px(y),y1:py(x),x2:px(y),y2:py(XGT),stroke:"var(--c2)","stroke-width":2,"stroke-dasharray":"3 4"}),
      sv("circle",{cx:px(y),cy:py(x),r:6,fill:"var(--acc)",stroke:"var(--surface)","stroke-width":2}));
    // Algorithm 2: gradient descent on Q(q) = ½qHq − qv; Hq is a finite-difference Hessian-vector product, H is never formed
    const hvp=q=>(gradL(x+1e-4*q,y)-gradL(x-1e-4*q,y))/2e-4;
    const qs=[0]; for(let i=0;i<kq;i++) qs.push(qs[i]-etaq*(hvp(qs[i])-v));
    const q=qs[kq], gA=2*x*q, gE=impGrad(x,y);
    out.textContent="Algorithm 2:  q = "+qs.slice(0,5).map(f4).join(" → ")+(kq>4?" → … → "+f4(q):"")+"    (exact v/H = "+f4(v/Hh)+")";
    st.innerHTML=""; st.append(stat("Back-end solution",f4(x),"x*(y) at y = "+y.toFixed(3)),stat("Slope of the curve",f4(s),"dx*/dy = 2x*/H by (4.12), H = "+Hh.toFixed(3)),
      stat("Loss slope",f4(v),"v = ∂U/∂x* = x* − 1.5"),stat("Gradient via Algorithm 2",f4(gA),"dU/dy = 2x*·q; exact: "+f4(gE)),
      stat("Upper-level cost U",(0.5*v*v).toFixed(5),taken+" upper-level step"+(taken===1?"":"s")+" taken")); };
  const setY=v=>{ y=Math.min(Y1,Math.max(Y0,v)); ys.input.value=y; ys.el.querySelector("output").textContent=y.toFixed(3); draw(); };
  el.append(h("div",{class:"vt"},"Implicit differentiation on the solution curve"),h("p",{class:"vd"},"Blue: where the back-end ends up for each front-end output y. Pink: the tangent whose slope comes from (4.12), using only derivatives of L at the solution. Amber: the error the training loss U penalizes."),
    h("div",{class:"ctl"},ys.el,range("Upper-level step size α",1,12,0.5,alpha,v=>{ alpha=v; }).el),
    h("div",{class:"ctl"},range("Algorithm 2 step size η",0.02,0.3,0.01,etaq,v=>{ etaq=v; draw(); },v=>v.toFixed(2)).el,range("Algorithm 2 iterations k",0,20,1,kq,v=>{ kq=v; draw(); }).el),
    h("div",{class:"ctl"},button("Upper-level step: y ← y − α·dU/dy",()=>{ taken++; setY(y-alpha*impGrad(solveLL(y),y)); },"primary"),button("Reset",()=>{ taken=0; setY(2); })),svg,
    h("div",{class:"legend"},...[["var(--c5)","solution curve x*(y)"],["var(--c3)","tangent from implicit differentiation"],["var(--c2)","error x* − 1.5"]].map(([c,t])=>h("span",{},h("i",{style:"background:"+c}),t))),out,st);
  draw();
};

// Day 4: right Jacobians on SE(2), checked numerically. Elements are [x, y, θ]; tangent vectors are [ρ1, ρ2, θ] (translation first)
const se2={
  mul:(a,b)=>{ const c=Math.cos(a[2]), s=Math.sin(a[2]); return [a[0]+c*b[0]-s*b[1], a[1]+s*b[0]+c*b[1], a[2]+b[2]]; },
  inv:a=>{ const c=Math.cos(a[2]), s=Math.sin(a[2]); return [-(c*a[0]+s*a[1]), -(-s*a[0]+c*a[1]), -a[2]]; },
  ab:t=>Math.abs(t)<1e-4?[1-t*t/6, t/2-t*t*t/24]:[Math.sin(t)/t,(1-Math.cos(t))/t],          // sinθ/θ and (1−cosθ)/θ, by series near 0
  Exp:t=>{ const [A,B]=se2.ab(t[2]); return [A*t[0]-B*t[1], B*t[0]+A*t[1], t[2]]; },
  Log:a=>{ const th=Math.atan2(Math.sin(a[2]),Math.cos(a[2])), [A,B]=se2.ab(th), n=A*A+B*B; return [(A*a[0]+B*a[1])/n, (-B*a[0]+A*a[1])/n, th]; },
  Ad:a=>{ const c=Math.cos(a[2]), s=Math.sin(a[2]); return [[c,-s,a[1]],[s,c,-a[0]],[0,0,1]]; } };
VIZ.w07_lie=el=>{
  let op="c1", arm=1.5, th1=40, dth=20, dro=0;
  const chi=[1,0.6,20*Math.PI/180], W=600, H=320, S=70, OX=190, OY=250;
  const X=p=>OX+S*p[0], Y=p=>OY-S*p[1];
  const svg=sv("svg",{viewBox:"0 0 "+W+" "+H,role:"img","aria-label":"Planar frames before and after a small right perturbation, showing how the perturbation of the input moves the output of the chosen operation"});
  const panel=h("div",{class:"panel"}), st=h("div",{class:"stats"}), lg=h("div",{class:"legend"});
  const frame=(p,col,label,dy)=>{ const c=Math.cos(p[2]), s=Math.sin(p[2]), x=X(p), y=Y(p);
    svg.append(sv("line",{x1:x,y1:y,x2:x+30*c,y2:y-30*s,stroke:col,"stroke-width":3}),sv("line",{x1:x,y1:y,x2:x-18*s,y2:y-18*c,stroke:col,"stroke-width":1.5}),sv("circle",{cx:x,cy:y,r:4,fill:col}));
    if(label) svg.append(txt(x+8,y+(dy||18),label,{"font-size":12,fill:col})); };
  const link=(a,b,col)=>svg.append(sv("line",{x1:X(a),y1:Y(a),x2:X(b),y2:Y(b),stroke:col,"stroke-width":1.5,"stroke-dasharray":"4 4"}));
  const mat=(A,title)=>h("div",{},h("div",{style:"font-size:.8rem;color:var(--muted)"},title),h("pre",{style:"margin:2px 0 8px;font-size:.82rem;line-height:1.5"},A.map(r=>r.map(v=>(Math.abs(v)<5e-7?0:v).toFixed(3).replace("-","−").padStart(7)).join(" ")).join("\n")));
  const vec=v=>"["+v.map(f3).join(", ")+"]";
  const draw=()=>{ const chi1=[arm,0,th1*Math.PI/180], tau=[dro,0,dth*Math.PI/180], pt=op==="act", m=pt?2:3;
    const tip=c=>se2.mul(c,[arm,0,0]).slice(0,2);                                        // group action χ·p on the point p = (link length, 0)
    const f=op==="c1"?c=>se2.mul(c,chi1):op==="c2"?c=>se2.mul(chi,c):op==="inv"?c=>se2.inv(c):tip, at=op==="c2"?chi1:chi;
    // f(χ ⊕ τ) ⊖ f(χ): Log(f(χ)⁻¹ ∘ f(χ ∘ Exp τ)) for a group-valued f, plain subtraction for a point-valued f
    const diff=t=>{ const a=f(at), b=f(se2.mul(at,se2.Exp(t))); return pt?[b[0]-a[0],b[1]-a[1]]:se2.Log(se2.mul(se2.inv(a),b)); };
    const J=M.zeros(m,3); for(let j=0;j<3;j++){ const e=[0,0,0]; e[j]=1e-5; const p=diff(e); e[j]=-1e-5; const q=diff(e); for(let i=0;i<m;i++) J[i][j]=(p[i]-q[i])/2e-5; }
    const c=Math.cos(chi[2]), s=Math.sin(chi[2]);
    const C=op==="c1"?se2.Ad(se2.inv(chi1)):op==="c2"?M.eye(3):op==="inv"?M.scale(se2.Ad(chi),-1):[[c,-s,-s*arm],[s,c,c*arm]], lin=M.mv(C,tau), act=diff(tau);
    svg.innerHTML=""; svg.append(sv("line",{class:"grid",x1:20,x2:W-20,y1:OY,y2:OY}),sv("line",{class:"grid",x1:OX,x2:OX,y1:10,y2:H-10}),txt(OX-6,OY+16,"world origin",{"text-anchor":"end",class:"mut","font-size":11}));
    const A="var(--muted)", Bc="var(--c3)", cp=se2.mul(chi,se2.Exp(tau));
    if(op==="inv"){ frame(chi,A,"χ"); frame(cp,Bc,"χ ⊕ τ",-10); frame(se2.inv(chi),A,"χ⁻¹"); frame(se2.inv(cp),Bc,"(χ ⊕ τ)⁻¹",-10); }
    else if(pt){ const p0=tip(chi), p1=tip(cp), pl=[p0[0]+lin[0],p0[1]+lin[1]];
      link([0,0,0],chi,A); link(chi,p0,A); link(cp,p1,Bc); frame(chi,A,"χ"); frame(cp,Bc,null);
      svg.append(sv("line",{x1:X(p0),y1:Y(p0),x2:X(pl),y2:Y(pl),stroke:"var(--c2)","stroke-width":2.5}),sv("circle",{cx:X(pl),cy:Y(pl),r:5,fill:"var(--c2)"}),
        sv("circle",{cx:X(p0),cy:Y(p0),r:5,fill:A}),sv("circle",{cx:X(p1),cy:Y(p1),r:5,fill:Bc}),txt(X(p0)+9,Y(p0)+18,"χ · p",{"font-size":12,fill:A}),txt(X(p1)-9,Y(p1)-9,"actual",{"font-size":12,fill:Bc,"text-anchor":"end"})); }
    else { const e0=se2.mul(chi,chi1), c1=op==="c1"?cp:chi, e1=op==="c1"?se2.mul(cp,chi1):se2.mul(e0,se2.Exp(tau));
      link([0,0,0],chi,A); link(chi,e0,A); if(op==="c1") link(c1,e1,Bc);
      frame(chi,A,"χ"); if(op==="c1") frame(c1,Bc,null); frame(e0,A,"χ ∘ χ₁"); frame(e1,Bc,"perturbed",-10); }
    lg.innerHTML=""; (pt?[[A,"before"],[Bc,"after perturbing χ by τ"],["var(--c2)","linear prediction J·τ"]]:[[A,"before"],[Bc,"after perturbing by τ"]]).forEach(([k,t])=>lg.append(h("span",{},h("i",{style:"background:"+k}),t)));
    panel.innerHTML=""; panel.append(h("div",{style:"display:flex;gap:18px;flex-wrap:wrap"},mat(J,"Jacobian by finite differences of (4.23)"),mat(C,"closed form: "+(op==="c1"?"Ad of χ₁⁻¹":op==="c2"?"identity":op==="inv"?"−Ad of χ":"R·[ I | p⊥ ], p⊥ = (−p₂, p₁)"))));
    let err=0; for(let i=0;i<m;i++) for(let j=0;j<3;j++) err=Math.max(err,Math.abs(J[i][j]-C[i][j]));
    st.innerHTML=""; st.append(stat("Largest entry difference",err.toExponential(1),"numeric vs closed form"),stat("Linear prediction",vec(lin),pt?"J·τ, displacement of the point":"J·τ as [ρ₁, ρ₂, θ] in the output frame"),
      stat("Actual change",vec(act),"f(χ ⊕ τ) ⊖ f(χ) for the τ you chose"),stat("Linearization error",Math.hypot(...lin.map((v,i)=>v-act[i])).toExponential(1),pt?"grows like |τ|²":"rounding only: this map is exactly linear in τ")); };
  el.append(h("div",{class:"vt"},"Jacobians of composition, inversion, and action on SE(2)"),h("p",{class:"vd"},"A planar arm: χ is the pose of the first joint, χ₁ the transform along the link, χ ∘ χ₁ the end-effector, and χ · p the position of the link tip p. The chosen input is perturbed on the right by τ = [ρ₁, 0, θ]."),
    h("div",{class:"ctl"},select("Differentiate",[["c1","χ ∘ χ₁ with respect to χ"],["c2","χ ∘ χ₁ with respect to χ₁"],["inv","χ⁻¹ with respect to χ"],["act","χ · p with respect to χ (group action)"]],v=>{ op=v; draw(); }).el),
    h("div",{class:"ctl"},range("Link length (χ₁ translation)",0.5,2.5,0.1,arm,v=>{ arm=v; draw(); },v=>v.toFixed(1)).el,range("Link bend (χ₁ rotation)",-90,90,5,th1,v=>{ th1=v; draw(); },v=>v+"°").el),
    h("div",{class:"ctl"},range("τ: rotation θ",-40,40,1,dth,v=>{ dth=v; draw(); },v=>v+"°").el,range("τ: forward ρ₁",-0.5,0.5,0.05,dro,v=>{ dro=v; draw(); },v=>v.toFixed(2)).el),svg,lg,panel,st);
  draw();
};

// Day 5: the coefficient γ(θ) = sin(θ/2)/θ of (4.40)–(4.41) and its derivative, in emulated 32-bit floats
VIZ.w07_small=el=>{
  let p=-3, zero=false;
  const f=Math.fround;
  const exact=t=>{ let v=0, d=0; for(let k=0,fac=1;k<=10;k++){ if(k) fac*=2*k*(2*k+1); const c=(k%2?-1:1)/(Math.pow(2,2*k+1)*fac); v+=c*Math.pow(t,2*k); if(k) d+=c*2*k*Math.pow(t,2*k-1); } return [v,d]; };   // float64 series, θ ≤ 1
  const calc=t=>{ const t32=f(t), hh=f(t32/2), s=f(Math.sin(hh)), c=f(Math.cos(hh)), t2=f(t32*t32);
    return {nv:f(s/t32), nd:f(f(f(c*0.5)/t32)-f(s/t2)),                               // quotient rule, as autodiff would apply it to sin(θ/2)/θ
      tv:f(0.5-f(t2/48)+f(f(t2*t2)/3840)), td:f(f(-t32/24)+f(f(t2*t32)/960)), ex:exact(t)}; };
  const rel=(a,b)=>Number.isFinite(a)?Math.min(2,Math.max(-9,Math.log10(Math.abs(a-b)/Math.abs(b)+1e-30))):2;
  const box=h("div"), st=h("div",{class:"stats"}), note=h("p",{class:"warn"});
  const e3=v=>Number.isFinite(v)?v.toExponential(3).replace("-","−").replace("e−","e-"):"NaN";
  const draw=()=>{ const A=[], B=[]; for(let q=-6;q<=0.001;q+=0.25){ const t=Math.pow(10,q), r=calc(t); A.push({x:t,y:rel(r.nd,r.ex[1]),label:"naive, θ = 1e"+q,fmt:v=>"relative error 1e"+v.toFixed(1)}); B.push({x:t,y:rel(r.td,r.ex[1]),label:"series, θ = 1e"+q,fmt:v=>"relative error 1e"+v.toFixed(1)}); }
    const t=zero?0:Math.pow(10,p), r=calc(t), ex=zero?[0.5,0]:r.ex;
    box.innerHTML=""; box.append(lineChart({h:280,aria:"Relative error of the derivative of gamma in 32-bit floats against the angle, for the naive formula and for the two-term series",
      x:{min:1e-6,max:1,log:true,ticks:[-6,-5,-4,-3,-2,-1,0].map(q=>[Math.pow(10,q),"1e"+q]),label:"rotation angle θ (rad)"},
      y:{min:-9,max:2,ticks:[-8,-6,-4,-2,0,2].map(q=>[q,"1e"+q])},
      series:[{name:"naive formula (quotient rule), float32",color:"var(--c3)",points:A},{name:"series −θ/24 + θ³/960, float32",color:"var(--c1)",points:B}],marker:zero?null:{x:t,y:rel(r.nd,ex[1])}}));
    st.innerHTML=""; st.append(stat("Gamma, naive formula",e3(r.nv),"sin(θ/2)/θ in float32"),stat("Gamma, series",e3(r.tv),"γₒ of (4.41) in float32"),stat("Gamma, exact",e3(ex[0]),"float64 reference"),
      stat("Derivative, naive",e3(r.nd),"dγ/dθ by the quotient rule, float32"),stat("Derivative, series",e3(r.td),"−θ/24 + θ³/960, float32"),stat("Derivative, exact",e3(ex[1]),"float64 reference"));
    note.textContent=zero?"At θ = 0 the naive formula computes 0/0. The value and the gradient are NaN, and one NaN poisons every parameter the gradient reaches.":""; };
  const zb=button("Set θ = 0 exactly",()=>{ zero=!zero; zb.classList.toggle("on",zero); draw(); });
  el.append(h("div",{class:"vt"},"Why the exponential map switches to a series near zero"),h("p",{class:"vd"},"γ(θ) = sin(θ/2)/θ is the factor that multiplies ν in the quaternion Exp. Every operation is rounded to 32-bit floats, as on a GPU. Vertical axis: the relative error of the derivative dγ/dθ (1e0 means 100% wrong; errors above 1e2 are clipped)."),
    h("div",{class:"ctl"},range("Angle θ",-6,0,0.25,p,v=>{ p=v; draw(); },v=>"1e"+v+" rad").el,zb),box,note,st);
  draw();
};
