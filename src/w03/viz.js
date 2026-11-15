// Week 3 widgets: sparsity patterns, variable elimination, Schur complement, Bayes tree updates, Givens updates.

// ---------- shared helpers ----------
const fmt=v=>Math.abs(v)<5e-10?"":v.toFixed(2).replace("-","−");
const rhs=v=>Math.abs(v)<5e-10?"0.00":v;      // a right-hand side is shown even when it is zero
const CELL="padding:2px 9px;border:0;text-align:right;font-family:var(--f-mono);white-space:nowrap;";
// a small matrix as an HTML table. head: column names; rows: [{l: label, c: [numbers or strings], bg: one colour or one per cell}]
const mtab=(head,rows)=>h("table",{style:"width:auto;margin:6px 0"},
  h("tr",{},h("th",{style:CELL+"background:none"}),...head.map(t=>h("th",{style:CELL+"background:none"},t))),
  ...rows.map(r=>h("tr",{},h("td",{style:CELL+"text-align:left;color:var(--muted);font-family:inherit"},r.l||""),
    ...r.c.map((v,j)=>h("td",{style:CELL+"background:"+((Array.isArray(r.bg)?r.bg[j]:r.bg)||"none")},typeof v==="number"?fmt(v):v)))));
const legend=items=>h("div",{class:"legend"},...items.map(([c,t])=>h("span",{},h("i",{style:"background:"+c+";height:10px"}),t)));
const arrow=(x1,y1,x2,y2,col)=>{ const a=Math.atan2(y2-y1,x2-x1), p=(d,w)=>(x2-d*Math.cos(a)+w*Math.sin(a)).toFixed(1)+","+(y2-d*Math.sin(a)-w*Math.cos(a)).toFixed(1);
  return sv("g",{},sv("line",{x1,y1,x2:x2-6*Math.cos(a),y2:y2-6*Math.sin(a),stroke:col,"stroke-width":2}),sv("polygon",{points:p(0,0)+" "+p(10,4.5)+" "+p(10,-4.5),fill:col})); };

// graph of a set of factors (each factor = array of variable ids): adj[v] = set of variables that share a factor with v
const adjOf=(n,facs)=>{ const adj=Array.from({length:n},()=>new Set()); facs.forEach(f=>f.forEach(a=>f.forEach(b=>{ if(a!==b) adj[a].add(b); }))); return adj; };
// eliminate v from the graph: its neighbours (the separator) become a clique. fill = number of edges that had to be added
const elimNode=(adj,v)=>{ const s=[...adj[v]]; let fill=0; s.forEach(a=>{ adj[a].delete(v); s.forEach(b=>{ if(a<b&&!adj[a].has(b)){ adj[a].add(b); adj[b].add(a); fill++; } }); }); adj[v].clear(); return {s,fill}; };
// symbolic elimination in a given order: sep[v] = separator of v (sorted by elimination position), fills[v] = fill-in made when v went
const elimSym=(n,facs,order)=>{ const adj=adjOf(n,facs), pos=[], sep=[], fills=[]; let fill=0; order.forEach((v,i)=>pos[v]=i);
  order.forEach(v=>{ const e=elimNode(adj,v); sep[v]=e.s.sort((a,b)=>pos[a]-pos[b]); fills[v]=e.fill; fill+=e.fill; }); return {sep,fill,fills,pos}; };
// greedy minimum degree: repeatedly eliminate the variable in `free` that has the fewest neighbours left
const minDeg=(adj,free)=>{ const left=new Set(free), out=[]; while(left.size){ let best=-1; left.forEach(v=>{ if(best<0||adj[v].size<adj[best].size) best=v; }); out.push(best); left.delete(best); elimNode(adj,best); } return out; };
// Bayes tree from the conditionals p(v | sep[v]), visited in reverse elimination order (Kaess et al. 2012).
// C[i] = {f: frontal variables, s: separator, par: parent index or -1}; of[v] = index of the clique where v is frontal
const bayesTree=(order,sep)=>{ const C=[], of=[];
  for(let i=order.length-1;i>=0;i--){ const v=order[i], s=sep[v];
    if(!s.length){ of[v]=C.push({f:[v],s:[],par:-1})-1; continue; }
    const p=of[s[0]], c=C[p];                                             // s ⊆ c.f ∪ c.s always, so equal sizes means equal sets
    if(c.f.length+c.s.length===s.length){ c.f.unshift(v); of[v]=p; } else of[v]=C.push({f:[v],s,par:p})-1; }
  return {C,of}; };
// Householder QR of [A | b] (last column is the right-hand side), in place: upper-trapezoidal with a positive diagonal
const qr=B=>{ const m=B.length, k=B[0].length-1;
  for(let c=0;c<Math.min(m,k);c++){ let nrm=0; for(let r=c;r<m;r++) nrm+=B[r][c]*B[r][c]; nrm=Math.sqrt(nrm); if(nrm<1e-12) continue;
    const v=B.map((row,r)=>r<c?0:row[c]); v[c]+=(v[c]<0?-1:1)*nrm; let vv=0; v.forEach(x=>vv+=x*x);
    for(let j=c;j<=k;j++){ let s=0; for(let r=c;r<m;r++) s+=v[r]*B[r][j]; s*=2/vv; for(let r=c;r<m;r++) B[r][j]-=s*v[r]; }
    for(let r=c+1;r<m;r++) B[r][c]=0;
    if(B[c][c]<0) for(let j=c;j<=k;j++) B[c][j]=-B[c][j]; }
  return B; };

// ---------- Day 1: factor graph ↔ Jacobian pattern ↔ information matrix pattern ↔ R pattern ----------
VIZ.w03_pattern=el=>{
  let N=12, rad=70, loop=false, ord="pl";
  const LM=[...Array(7)].map((_,i)=>{ const a=(i+.5)/7*2*Math.PI, r=i%2?36:112; return [145+r*Math.sin(a),126-r*Math.cos(a)*.9]; });
  const svg=sv("svg",{viewBox:"0 0 600 480",role:"img","aria-label":"Top left: a robot path with landmarks and the factors between them. Top right: the nonzero pattern of the Jacobian A. Bottom: the patterns of the information matrix and of its triangular factor R, with fill-in marked."});
  const st=h("div",{class:"stats"});
  const draw=()=>{
    const P=[...Array(N)].map((_,i)=>{ const a=i/N*5.6; return [145+75*Math.sin(a),126-75*Math.cos(a)]; });
    const seen=LM.map(l=>P.map((p,i)=>Math.hypot(p[0]-l[0],p[1]-l[1])<rad?i:-1).filter(i=>i>=0)), id=[]; let n=N; seen.forEach((s,j)=>{ id[j]=s.length?n++:-1; });
    const facs=[[0]]; for(let i=1;i<N;i++) facs.push([i-1,i]); if(loop) facs.push([0,N-1]); seen.forEach((s,j)=>s.forEach(i=>facs.push([i,id[j]])));
    const all=[...Array(n).keys()], poses=all.slice(0,N), lms=all.slice(N);
    let order=ord==="pl"?all:ord==="lp"?[...lms,...poses]:minDeg(adjOf(n,facs),all);
    if(ord==="rnd"){ const r=rng(4); order=[...all]; for(let i=n-1;i>0;i--){ const j=Math.floor(r.u()*(i+1)); [order[i],order[j]]=[order[j],order[i]]; } }
    const {sep,fill,pos}=elimSym(n,facs,order), adj=adjOf(n,facs), col=v=>v<N?"var(--c5)":"var(--c2)";
    svg.innerHTML="";
    // the factor graph, drawn as a map
    for(let i=1;i<N;i++) svg.append(sv("line",{x1:P[i-1][0],y1:P[i-1][1],x2:P[i][0],y2:P[i][1],stroke:"var(--c5)","stroke-width":2}));
    seen.forEach((s,j)=>s.forEach(i=>svg.append(sv("line",{x1:P[i][0],y1:P[i][1],x2:LM[j][0],y2:LM[j][1],stroke:"var(--c2)","stroke-width":1}))));
    if(loop) svg.append(sv("line",{x1:P[0][0],y1:P[0][1],x2:P[N-1][0],y2:P[N-1][1],stroke:"var(--c3)","stroke-width":2.5,"stroke-dasharray":"5 4"}));
    P.forEach(p=>svg.append(sv("circle",{cx:p[0],cy:p[1],r:5,fill:"var(--c5)"})));
    LM.forEach((l,j)=>svg.append(sv("rect",{x:l[0]-5,y:l[1]-5,width:10,height:10,fill:id[j]<0?"var(--line2)":"var(--c2)"})));
    svg.append(txt(P[0][0],P[0][1]-10,"x1",{"text-anchor":"middle","font-size":12}),txt(P[N-1][0]-10,P[N-1][1]-2,"x"+N,{"text-anchor":"end","font-size":12}));
    // a pattern plot: one small rectangle per nonzero, plus a strip on top that says which columns are poses and which landmarks
    const grid=(x0,y0,w,hmax,nr,cells,title)=>{ const cw=w/n, ch=Math.min(hmax/nr,cw);
      svg.append(txt(x0,y0-12,title,{"font-size":12}),sv("rect",{x:x0,y:y0,width:w,height:ch*nr,fill:"var(--soft)"}));
      order.forEach((v,c)=>svg.append(sv("rect",{x:(x0+c*cw).toFixed(1),y:y0-6,width:cw.toFixed(1),height:3,fill:col(v)})));
      cells.forEach(([r,c,f])=>svg.append(sv("rect",{x:(x0+c*cw+.4).toFixed(1),y:(y0+r*ch+.4).toFixed(1),width:Math.max(cw-.8,1).toFixed(1),height:Math.max(ch-.8,1).toFixed(1),fill:f}))); };
    grid(330,34,250,195,facs.length,facs.flatMap((f,r)=>f.map(v=>[r,pos[v],col(v)])),"A: "+facs.length+" rows (factors) × "+n+" columns");
    const lam=[], R=[]; let nl=n, nr=n;
    all.forEach(v=>{ lam.push([pos[v],pos[v],"var(--ink)"]); R.push([pos[v],pos[v],"var(--ink)"]);
      adj[v].forEach(u=>{ lam.push([pos[v],pos[u],"var(--ink)"]); nl++; });
      sep[v].forEach(u=>{ R.push([pos[v],pos[u],adj[v].has(u)?"var(--ink)":"var(--c3)"]); nr++; }); });
    grid(40,268,200,200,n,lam,"Λ = AᵀA: "+n+" × "+n);
    grid(350,268,200,200,n,R,"R, where RᵀR = Λ");
    st.innerHTML=""; st.append(stat("Nonzeros in A",facs.reduce((s,f)=>s+f.length,0)),stat("Nonzeros in Λ",nl,"of "+n*n),stat("Nonzeros in R",nr,"upper triangle holds "+n*(n+1)/2),stat("Fill-in",fill,"entries of R that are zero in Λ")); };
  const lb=button("Add a loop closure",()=>{ loop=!loop; lb.classList.toggle("on",loop); draw(); });
  el.append(h("div",{class:"vt"},"Sparsity patterns of a small SLAM problem"),h("p",{class:"vd"},"A robot drives most of a circle and sees landmarks within its sensor range. Every variable is one number here, so each small square is one matrix entry. Columns are drawn in elimination order."),
    h("div",{class:"ctl"},range("Poses",4,20,1,N,v=>{ N=v; draw(); }).el,range("Sensor range",30,110,2,rad,v=>{ rad=v; draw(); }).el),
    h("div",{class:"ctl"},select("Variable ordering",[["pl","poses, then landmarks"],["lp","landmarks, then poses"],["md","minimum degree (greedy)"],["rnd","random"]],v=>{ ord=v; draw(); }).el,lb),
    svg,legend([["var(--c5)","pose column"],["var(--c2)","landmark column"],["var(--ink)","nonzero"],["var(--c3)","fill-in"]]),st);
  draw();
};

// ---------- Days 2 and 3: variable elimination on the toy problem, numerically, by partial QR ----------
const TN=["ℓ₁","ℓ₂","p₁","p₂","p₃"];
// the book's toy factor graph with every variable a single number. Columns ℓ1 ℓ2 p1 p2 p3 | b; row i is factor ϕ(i+1)
const TOY=[[0,0,1,0,0,0],[0,0,-1,1,0,1],[0,0,0,-1,1,1],[1,0,0,0,0,.6],[0,1,0,0,0,2.4],[0,0,1,0,0,.2],[1,0,-1,0,0,.5],[1,0,0,-1,0,-.3],[0,1,0,0,-1,.5]];
const TOYF=TOY.map(r=>[0,1,2,3,4].filter(c=>r[c]));
// eliminate the first k variables of `order` (book eq. 1.45–1.49). R: conditionals so far; F: factors that remain; x: MAP estimate once k = 5
const toyElim=(order,k)=>{ const pos=[]; order.forEach((v,i)=>pos[v]=i);
  const nm=vs=>vs.map(v=>TN[v]).join(", ");
  let F=TOY.map((r,i)=>({vars:TOYF[i],rows:[r],name:"ϕ"+"₁₂₃₄₅₆₇₈₉"[i]})), last=null; const R=[];
  for(let t=0;t<k;t++){ const v=order[t], used=F.filter(f=>f.vars.includes(v)); F=F.filter(f=>!f.vars.includes(v));
    const sep=[...new Set(used.flatMap(f=>f.vars))].filter(x=>x!==v).sort((a,b)=>pos[a]-pos[b]), cols=[v,...sep];
    const B=qr(used.flatMap(f=>f.rows).map(r=>[...cols.map(c=>r[c]),r[5]]));
    const full=b=>{ const r=[0,0,0,0,0,b[cols.length]]; cols.forEach((c,i)=>r[c]=b[i]); return r; };
    R.push({v,sep,row:full(B[0]),name:"p("+TN[v]+(sep.length?" | "+nm(sep):"")+")"});
    const rest=B.slice(1,1+sep.length).filter(b=>b.slice(0,-1).some(x=>Math.abs(x)>1e-9)).map(full);
    if(rest.length) F.push({vars:sep,rows:rest,name:"τ("+nm(sep)+")",tau:true});
    last={v,sep,used:used.map(f=>f.name).join(", "),cond:R[t].name,made:rest.length?"τ("+nm(sep)+")":""}; }
  let x=null; if(k===5){ x=[]; for(let t=4;t>=0;t--){ const {v,sep,row}=R[t]; x[v]=(row[5]-sep.reduce((s,c)=>s+row[c]*x[c],0))/row[v]; } }
  return {R,F,last,x}; };
// self-check, once at load: elimination in a fill-producing order must reproduce the dense solution of the normal equations
{ const A=TOY.map(r=>r.slice(0,5)), At=M.T(A), x=M.solve(M.mul(At,A),M.mv(At,TOY.map(r=>r[5]))), y=toyElim([3,0,1,2,4],5).x;
  console.assert(x.every((v,i)=>Math.abs(v-y[i])<1e-9),"w03: elimination disagrees with the normal equations"); }
VIZ.w03_elim=el=>{
  const XY=[[205,66],[545,74],[100,190],[300,190],[490,190]], ORD={book:[0,1,2,3,4],poses:[2,3,4,0,1],hub:[3,0,1,2,4],rev:[4,3,2,1,0]};
  let ord="book", k=0;
  const svg=sv("svg",{viewBox:"0 0 600 285",role:"img","aria-label":"The toy SLAM graph with three poses and two landmarks. Variables that have been eliminated are shaded and receive arrows from the variables they are conditioned on. The factors that remain are small squares."});
  const info=h("div",{class:"out",style:"font-family:inherit;line-height:1.5"}), tab=h("div",{style:"overflow-x:auto"}), st=h("div",{class:"stats"}), res=h("div",{class:"out"});
  const draw=()=>{ const order=ORD[ord], S=toyElim(order,k), sym=elimSym(5,TOYF,order), gone=order.slice(0,k);
    svg.innerHTML="";
    S.R.forEach(({v,sep})=>sep.forEach(s=>{ const a=XY[s], b=XY[v], d=Math.hypot(b[0]-a[0],b[1]-a[1]), ux=(b[0]-a[0])/d, uy=(b[1]-a[1])/d; svg.append(arrow(a[0]+20*ux,a[1]+20*uy,b[0]-20*ux,b[1]-20*uy,"var(--c5)")); }));
    const cnt={}, un=[0,0,0,0,0];
    S.F.forEach(f=>{ let x, y; const m=f.vars.length;
      if(m===1){ const v=f.vars[0], d=[[-.7,.7],[.7,.7],[0,1]][un[v]++%3]; x=XY[v][0]+38*d[0]; y=XY[v][1]+38*d[1]*(v<2?-1:1); }
      else{ x=f.vars.reduce((s,v)=>s+XY[v][0],0)/m; y=f.vars.reduce((s,v)=>s+XY[v][1],0)/m; const key=[...f.vars].sort().join(); cnt[key]=(cnt[key]||0)+1;
        if(m>2) x+=40; if(cnt[key]>1) y+=30; if(XY.some(p=>Math.hypot(p[0]-x,p[1]-y)<34)) y+=52; }
      const c=f.tau?"var(--c3)":"var(--muted)";
      f.vars.forEach(v=>svg.append(sv("line",{x1:XY[v][0],y1:XY[v][1],x2:x,y2:y,stroke:c,"stroke-width":f.tau?2:1.5})));
      svg.append(sv("rect",{x:x-6,y:y-6,width:12,height:12,fill:f.tau?"var(--c3)":"var(--ink)"})); });
    XY.forEach((p,v)=>{ const g=gone.includes(v); svg.append(sv("circle",{cx:p[0],cy:p[1],r:20,fill:g?"var(--c5-soft)":"var(--surface)",stroke:g?"var(--c5)":"var(--ink)","stroke-width":k&&order[k-1]===v?4:2}),txt(p[0],p[1]+5,TN[v],{"text-anchor":"middle","font-size":14})); });
    const L=S.last;
    info.textContent=!k?"Nine factors on five variables. Nothing has been eliminated yet: the table below is [A | b] with its columns in the chosen order."
      :"Step "+k+": eliminate "+TN[L.v]+". Its factors "+L.used+" are multiplied into one product factor, which QR splits into the conditional "+L.cond+(L.made?" and a new factor "+L.made+" on the separator.":L.sep.length?". Nothing is left over for the separator.":". The separator is empty, so this is a prior and the elimination is finished.");
    tab.innerHTML=""; tab.append(mtab([...order.map(v=>TN[v]),"d or b"],[
      ...S.R.map(r=>({l:r.name,c:[...order.map(c=>r.row[c]),rhs(r.row[5])],bg:"var(--c5-soft)"})),
      ...S.F.flatMap(f=>f.rows.map(r=>({l:f.name,c:[...order.map(c=>r[c]),rhs(r[5])],bg:f.tau?"var(--c3-soft)":""})))]));
    st.innerHTML=""; st.append(stat("Eliminated",k+" of 5"),stat("Nonzeros in R so far",gone.reduce((s,v)=>s+1+sym.sep[v].length,0),"blue rows of the table"),stat("Fill-in so far",gone.reduce((s,v)=>s+sym.fills[v],0),"graph edges that did not exist before"));
    res.textContent=S.x?"Back-substitution, last row first: "+[2,3,4,0,1].map(v=>TN[v]+" = "+S.x[v].toFixed(3).replace("-","−")).join(",  "):"Eliminate all five variables to read off the estimate by back-substitution."; };
  const go=d=>{ k=Math.max(0,Math.min(5,d)); draw(); };
  el.append(h("div",{class:"vt"},"Variable elimination, one variable at a time"),h("p",{class:"vd"},"The book's toy problem with every variable a single number. Each step runs a real partial QR factorization on the factors that touch the variable."),
    h("div",{class:"ctl"},select("Elimination order",[["book","ℓ₁ ℓ₂ p₁ p₂ p₃ (the book's)"],["poses","p₁ p₂ p₃ ℓ₁ ℓ₂ (poses first)"],["hub","p₂ ℓ₁ ℓ₂ p₁ p₃ (p₂ first)"],["rev","p₃ p₂ p₁ ℓ₂ ℓ₁ (reversed)"]],v=>{ ord=v; go(0); }).el,
      button("Eliminate next",()=>go(k+1),"primary"),button("Back",()=>go(k-1),"ghost"),button("Reset",()=>go(0),"ghost")),
    svg,legend([["var(--ink)","original factor"],["var(--c3)","new factor τ made by elimination"],["var(--c5)","eliminated variable and its conditional (a row of R)"]]),info,tab,st,res);
  draw();
};

// ---------- Day 3: marginalizing and conditioning a 3-variable Gaussian chain ----------
VIZ.w03_schur=el=>{
  let a=1, b=1, e=1; const NM=["x₁","x₂","x₃"];
  const box=h("div",{style:"display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:4px 18px"}), st=h("div",{class:"stats"});
  const svg=sv("svg",{viewBox:"0 0 600 220",role:"img","aria-label":"One-sigma ellipses of the two variables that remain: the marginal distribution and the distribution conditioned on the removed variable."});
  const draw=()=>{ const L=[[1+a,-a,0],[-a,a+b,-b],[0,-b,b]], S=M.inv(L), r=[0,1,2].filter(i=>i!==e), nm=r.map(i=>NM[i]);
    const sub=A=>r.map(i=>r.map(j=>A[i][j])), schur=A=>r.map(i=>r.map(j=>A[i][j]-A[i][e]*A[e][j]/A[e][e]));
    const Lm=schur(L), Sm=sub(S), Lc=sub(L), Sc=schur(S), isFill=Math.abs(Lc[0][1])<1e-9&&Math.abs(Lm[0][1])>1e-9;
    const z=A=>A.map(row=>row.map(v=>Math.abs(v)<5e-10?"0":v));
    const blk=(title,A,names,bg)=>h("div",{},h("div",{style:"font-size:.8rem;color:var(--muted);margin-top:6px"},title),mtab(names,z(A).map((row,i)=>({l:names[i],c:row,bg:bg?bg(i):""}))));
    const hot=i=>[0,1,2].map(j=>i===e||j===e?"var(--soft)":"");
    box.innerHTML=""; box.append(blk("Information matrix Λ",L,NM,hot),
      blk("Marginalize "+NM[e]+", information: Schur complement",Lm,nm,i=>[0,1].map(j=>i!==j&&isFill?"var(--c3-soft)":"var(--c5-soft)")),
      blk("Condition on "+NM[e]+", information: delete its row and column",Lc,nm,()=>"var(--c2-soft)"),
      blk("Covariance Σ = Λ⁻¹",S,NM,hot),
      blk("Marginalize "+NM[e]+", covariance: delete its row and column",Sm,nm,()=>"var(--c5-soft)"),
      blk("Condition on "+NM[e]+", covariance: Schur complement",Sc,nm,()=>"var(--c2-soft)"));
    const sc=75/Math.sqrt(Math.max(Sm[0][0],Sm[1][1])), fl=A=>[[A[0][0],-A[0][1]],[-A[0][1],A[1][1]]];      // screen y points down
    svg.innerHTML=""; svg.append(sv("line",{class:"axis",x1:150,y1:110,x2:450,y2:110}),sv("line",{class:"axis",x1:300,y1:12,x2:300,y2:208}),
      txt(456,114,nm[0],{"font-size":13}),txt(308,22,nm[1],{"font-size":13}),
      covEllipse(fl(Sm),300,110,1,sc,{stroke:"var(--c5)","stroke-width":2.5}),covEllipse(fl(Sc),300,110,1,sc,{stroke:"var(--c2)","stroke-width":2.5}));
    st.innerHTML=""; st.append(stat("Λ entry between "+nm.join(" and "),(fmt(Lc[0][1])||"0")+" → "+(fmt(Lm[0][1])||"0"),"before → after marginalizing "+NM[e]),
      stat("Std. dev. of "+nm[1],Math.sqrt(Sm[1][1]).toFixed(2)+" vs "+Math.sqrt(Sc[1][1]).toFixed(2),"marginal vs conditional")); };
  el.append(h("div",{class:"vt"},"Marginalize or condition: one variable of a chain"),h("p",{class:"vd"},"Three numbers in a chain: a prior on x₁ (precision 1), a link x₁–x₂ and a link x₂–x₃. Remove one variable in each of the two ways and compare all four results."),
    h("div",{class:"ctl"},range("Precision of link x₁–x₂",.2,4,.1,a,v=>{ a=v; draw(); },v=>v.toFixed(1)).el,range("Precision of link x₂–x₃",.2,4,.1,b,v=>{ b=v; draw(); },v=>v.toFixed(1)).el,
      select("Variable to remove",[[1,"x₂ (the middle)"],[0,"x₁ (an end)"],[2,"x₃ (an end)"]],v=>{ e=+v; draw(); }).el),
    box,svg,legend([["var(--c5)","marginal of the two that remain (1σ)"],["var(--c2)","conditional, given the removed variable (1σ)"],["var(--c3)","fill-in: a new nonzero"]]),st);
  draw();
};

// ---------- Days 4 and 5: the Bayes tree of a small SLAM run, updated as factors arrive ----------
VIZ.w03_isam=el=>{
  // one entry per time step: [pose position, landmark seen (or -1), loop closure to pose index (or -1)]
  const RUN=[[0,0,-1,-1],[1,0,0,-1],[2,0,0,-1],[3,0,-1,-1],[3,1,1,-1],[3,2,1,-1],[2,2,-1,-1],[1,2,2,-1],[0,2,2,-1],[0,1,-1,-1],[.25,.4,-1,0],[1.1,.4,0,-1],[2.1,.4,-1,-1],[2.45,.95,-1,-1]], LXY=[[1.5,-.75,0,22],[3.75,1.5,21,4],[.5,2.75,0,-13]];      // landmark x, y and where its label goes
  const px=p=>[28+p[0]*46,196-p[1]*52];
  let t=0, V=[], F=[], order=[], tree={C:[],of:[]}, hot=new Set(), newF=[], work=[], lmId=[], prev=-1;
  const reset=()=>{ t=0; V=[]; F=[]; order=[]; tree={C:[],of:[]}; hot=new Set(); newF=[]; work=[]; lmId=[]; prev=-1; };
  const step=()=>{ if(t>=RUN.length) return; const [gx,gy,lm,lc]=RUN[t], x=V.length, fresh=[x], add=[];
    V.push({n:"x"+(t+1),xy:px([gx,gy]),pose:true,low:!gy});
    add.push(t?[prev,x]:[x]); prev=x; if(lc>=0) add.push([lc,x]);
    if(lm>=0){ if(lmId[lm]==null){ lmId[lm]=V.length; fresh.push(V.length); V.push({n:"l"+(lm+1),xy:px(LXY[lm]),lab:LXY[lm].slice(2)}); } add.push([x,lmId[lm]]); }
    // the cliques on the paths from the touched variables to the root are taken apart; their frontal variables are re-eliminated
    const touched=[...new Set(add.flat())].filter(v=>!fresh.includes(v)), aff=new Set();
    touched.forEach(v=>{ for(let c=tree.of[v];c>=0&&!aff.has(c);c=tree.C[c].par) aff.add(c); });
    const redo=new Set([...aff].flatMap(c=>tree.C[c].f));
    // new order: untouched variables keep theirs; then the re-eliminated ones by greedy minimum degree, with the variables of the new factors
    // forced last so they end up in the root (the idea of constrained COLAMD).
    // ponytail: the whole tree is rebuilt with this order instead of being edited in place. It gives the same tree, because the
    // conditionals of the untouched variables do not change; fine for 17 variables, edit in place if the run ever grows large.
    F.push(...add); const keep=order.filter(v=>!redo.has(v)), adj=adjOf(V.length,F); keep.forEach(v=>elimNode(adj,v));
    order=[...keep,...minDeg(adj,order.filter(v=>redo.has(v)&&!touched.includes(v))),...touched,...fresh];
    newF=add; fresh.forEach(v=>redo.add(v)); hot=redo;
    tree=bayesTree(order,elimSym(V.length,F,order).sep); work.push(redo.size); t++; };
  const svg=sv("svg",{viewBox:"0 0 600 400",role:"img","aria-label":"Left: the robot's path, landmarks and factors so far. Right: the Bayes tree, root at the top, with the cliques that were recomputed in this step highlighted."});
  const info=h("div",{class:"out",style:"font-family:inherit;line-height:1.5"}), st=h("div",{class:"stats"}), bars=h("div",{class:"bars"});
  const draw=()=>{ svg.innerHTML="";
    svg.append(txt(8,18,"Factor graph",{"font-size":12,class:"mut"}),txt(250,18,"Bayes tree (root at the top)",{"font-size":12,class:"mut"}));
    F.forEach(f=>{ if(f.length<2) return; const a=V[f[0]].xy, b=V[f[1]].xy, nw=newF.includes(f); svg.append(sv("line",{x1:a[0],y1:a[1],x2:b[0],y2:b[1],stroke:nw?"var(--c3)":V[f[1]].pose?"var(--c5)":"var(--c2)","stroke-width":nw?3:1.5})); });
    V.forEach((v,i)=>{ const [x,y]=v.xy, hi=hot.has(i);
      svg.append(v.pose?sv("circle",{cx:x,cy:y,r:9,fill:hi?"var(--c3-soft)":"var(--surface)",stroke:hi?"var(--c3)":"var(--c5)","stroke-width":2}):sv("rect",{x:x-8,y:y-8,width:16,height:16,fill:hi?"var(--c3-soft)":"var(--surface)",stroke:hi?"var(--c3)":"var(--c2)","stroke-width":2}),
        txt(x+(v.pose?0:v.lab[0]),y+(v.pose?(v.low?22:-13):v.lab[1]),v.n,{"text-anchor":"middle","font-size":11})); });
    // tree layout: leaves take consecutive columns, a parent sits above the middle of its children
    const C=tree.C, kids=C.map(()=>[]), X=[], D=[]; let leaves=0; C.forEach((c,i)=>{ if(c.par>=0) kids[c.par].push(i); });
    const place=(i,d)=>{ D[i]=d; if(!kids[i].length){ X[i]=leaves++; return; } kids[i].forEach(j=>place(j,d+1)); X[i]=kids[i].reduce((s,j)=>s+X[j],0)/kids[i].length; };
    C.forEach((c,i)=>{ if(c.par<0) place(i,0); });
    const depth=Math.max(0,...D)+1, dy=Math.min(44,350/depth), cx=i=>245+(X[i]+.5)*350/Math.max(leaves,1), cy=i=>44+D[i]*dy, nm=vs=>vs.map(v=>V[v].n).join(",");
    C.forEach((c,i)=>{ if(c.par>=0) svg.append(sv("line",{x1:cx(c.par),y1:cy(c.par),x2:cx(i),y2:cy(i),stroke:"var(--line2)","stroke-width":1.5})); });
    C.forEach((c,i)=>{ const s=nm(c.f)+(c.s.length?" : "+nm(c.s):""), w=s.length*6.1+12, hi=c.f.some(v=>hot.has(v));
      svg.append(sv("rect",{x:cx(i)-w/2,y:cy(i)-10,width:w,height:20,rx:10,fill:hi?"var(--c3-soft)":"var(--surface)",stroke:hi?"var(--c3)":"var(--line2)","stroke-width":1.5}),txt(cx(i),cy(i)+4,s,{"text-anchor":"middle","font-size":11,"data-top":1})); });
    const w=work[t-1]||0, r=RUN[t-1];
    info.textContent=!t?"Empty. Press Add next pose to start the run."
      :"Step "+t+": added x"+t+(t>1?" with an odometry factor":" with a prior")+(r[2]>=0?", an observation of l"+(r[2]+1):"")+(r[3]>=0?", and a loop closure to x"+(r[3]+1):"")+". Re-eliminated "+w+" of "+V.length+" variables."+(t===RUN.length?" End of the run.":"");
    const tot=work.reduce((s,x)=>s+x,0); let batch=0, n=0; RUN.slice(0,t).forEach((r,i)=>{ n+=1+(r[2]>=0&&!RUN.slice(0,i).some(q=>q[2]===r[2])); batch+=n; });
    st.innerHTML=""; st.append(stat("Cliques in the tree",C.length),stat("Re-eliminated this step",t?w+" of "+V.length:"—","variables"),stat("Total so far",t?tot+" vs "+batch:"—","incremental vs refactoring everything each step"));
    bars.innerHTML=""; work.forEach((x,i)=>bars.append(h("div",{class:"bar"+(i===t-1?" hot":"")},h("span",{},"step "+(i+1)),h("div",{class:"bb"},h("i",{style:"width:"+(100*x/17).toFixed(0)+"%"})),h("span",{class:"bv"},x+" var."))));
    nb.disabled=t>=RUN.length; };
  const nb=button("Add next pose",()=>{ step(); draw(); },"primary");
  el.append(h("div",{class:"vt"},"A Bayes tree that grows with the robot"),h("p",{class:"vd"},"Fourteen poses around a block and three landmarks. Each step adds a pose and its factors, finds the cliques between the touched variables and the root, and re-eliminates only their variables."),
    h("div",{class:"ctl"},nb,button("Run to the end",()=>{ while(t<RUN.length) step(); draw(); }),button("Reset",()=>{ reset(); draw(); },"ghost")),
    svg,legend([["var(--c3)","new factor, and what was recomputed in this step"],["var(--c5)","pose and odometry"],["var(--c2)","landmark and observation"]]),info,st,
    h("div",{style:"font-size:.8rem;color:var(--muted);margin-top:8px"},"Variables re-eliminated at each step"),bars);
  for(let i=0;i<6;i++) step();
  draw();
};

// ---------- Day 5: adding one measurement row to R with Givens rotations (the linear-algebra view of an incremental update) ----------
VIZ.w03_givens=el=>{
  const n=5, A0=[[1,0,0,0,0,0],[-1,1,0,0,0,1],[0,-1,1,0,0,1.1],[0,0,-1,1,0,.9],[0,0,0,-1,1,1]];
  const NEW={loop:[-1,0,0,0,1,4.6],mid:[0,0,-1,0,1,2.3],gps:[0,0,0,0,1,4.4]}; let kind="loop", k=0;
  const solve=R=>{ const x=[]; for(let i=n-1;i>=0;i--){ let s=R[i][n]; for(let j=i+1;j<n;j++) s-=R[i][j]*x[j]; x[i]=s/R[i][i]; } return x; };
  const tab=h("div",{style:"overflow-x:auto"}), info=h("div",{class:"out",style:"font-family:inherit;line-height:1.5"}), st=h("div",{class:"stats"});
  const draw=()=>{ const R=qr(A0.map(r=>[...r])), a=[...NEW[kind]], x0=solve(R), first=a.findIndex(v=>v), tot=n-first, hit=[]; k=Math.max(0,Math.min(tot,k));
    for(let c=first;c<first+k;c++){ const r=Math.hypot(R[c][c],a[c]), cs=R[c][c]/r, sn=a[c]/r; for(let j=c;j<=n;j++){ const u=R[c][j], w=a[j]; R[c][j]=cs*u+sn*w; a[j]=-sn*u+cs*w; } a[c]=0; hit.push(c); }
    tab.innerHTML=""; tab.append(mtab(["x₁","x₂","x₃","x₄","x₅","d"],[...R.map((r,i)=>({l:"row "+(i+1),c:r.map((v,j)=>j===n?rhs(v):v),bg:hit.includes(i)?(i===first+k-1?"var(--c2-soft)":"var(--c5-soft)"):""})),{l:"new row",c:a.map((v,j)=>j===n?rhs(v):v),bg:"var(--c3-soft)"}]));
    info.textContent=!k?"R is the factor of five poses in a chain (a prior and four odometry steps). The new measurement is one extra row under it. "+tot+" rotation"+(tot>1?"s are":" is")+" needed."
      :k<tot?"Rotation "+k+" mixed row "+(first+k)+" of R with the new row and zeroed the new row's entry in column x"+(first+k)+"."
      :"Done. The new row is zero except for its last number, which is a leftover residual. R is upper-triangular again and holds all six measurements.";
    const x1=solve(R), f=x=>x.map(v=>v.toFixed(2)).join(", ");
    st.innerHTML=""; st.append(stat("Rotations",k+" of "+tot),stat("Rows of R changed",hit.length+" of "+n,"rows above the first nonzero are never touched"),stat("Estimate before",f(x0)),stat("Estimate after",k===tot?f(x1):"—",k===tot?"by back-substitution":"finish the rotations")); };
  el.append(h("div",{class:"vt"},"Update R with one new measurement"),h("p",{class:"vd"},"Each Givens rotation mixes one row of R with the new row so that one entry of the new row becomes zero. Nothing is refactored from scratch."),
    h("div",{class:"ctl"},select("New measurement",[["loop","x₅ − x₁ = 4.6 (loop closure to the start)"],["mid","x₅ − x₃ = 2.3"],["gps","x₅ = 4.4 (touches only the newest pose)"]],v=>{ kind=v; k=0; draw(); }).el,
      button("Apply next rotation",()=>{ k++; draw(); },"primary"),button("Reset",()=>{ k=0; draw(); },"ghost")),
    tab,legend([["var(--c3-soft)","the new measurement row"],["var(--c2-soft)","row of R changed by the last rotation"],["var(--c5-soft)","rows changed earlier"]]),info,st);
  draw();
};
