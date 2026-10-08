"""外部CDN・ROS・Webサーバ不要の高さ候補説明デモをHTMLへ書き出す。

表示用snapshotだけを受け取り、認識アルゴリズムは実行しない。箱は観測外包で、
体積の占有を意味しない。ブラウザのCanvasで軽量な直交投影を行う。
"""
import json
from pathlib import Path


def write_demo_html(path, document):
    """JSONの非有限値を拒否し、script終端をescapeして単一HTMLへ埋め込む。"""
    data = json.dumps(document, ensure_ascii=False, allow_nan=False).replace('<', '\\u003c')
    Path(path).write_text(_HTML.replace('__SCENE_JSON__', data), encoding='utf-8')


_HTML = r'''<!doctype html>
<html lang="ja"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>高さ候補・確認・解除：説明用デモ</title>
<style>
body{font:15px system-ui,sans-serif;margin:20px;background:#f2f4f7;color:#182335}h1{font-size:23px}
.warn{background:#fff1cc;padding:12px;border-left:5px solid #c88400}.controls{padding:12px;background:white;margin:10px 0}
label{display:inline-block;margin:5px 12px 5px 0}input[type=range]{width:260px}.views{display:flex;gap:12px;flex-wrap:wrap}
canvas{background:#fff;border:1px solid #b9c3d1;max-width:100%;touch-action:none}#scene{width:850px;height:550px}
#top{width:450px;height:450px}pre{white-space:pre-wrap;background:white;padding:12px;font-size:13px}.legend{line-height:1.9}
.green{color:#118349}.orange{color:#b86400}.cyan{color:#007c9b}.purple{color:#8054b4}.red{color:#d21f37}
button,select{padding:5px 9px}summary{cursor:pointer}
</style>
<h1>独立frameの高さ候補と、短期観測による確認・黒解除</h1>
<p class="warn">診断・説明用。箱は観測点の外包で、内部全部がoccupiedとは限りません。
確認済み ≠ 実物の確定。黒解除／白 ≠ 安全の証明。高い枝・天井の通過可否は未判定です。</p>
<div class="controls">
<button id="play">再生</button><button id="reset">視点リセット</button>
<label>snapshot <input id="frame" type="range" min="0" value="0"><span id="time"></span></label>
<label>N <select id="window"><option>1</option><option selected>3</option><option>5</option></select></label><br>
<label><input id="points" type="checkbox" checked>今回の投影点</label>
<label><input id="boxes" type="checkbox" checked>今回の候補外包</label>
<label><input id="means" type="checkbox" checked>対応候補の平均z</label>
<label><input id="hazard" type="checkbox" checked>保持hazard</label>
<label><input id="robot" type="checkbox" checked>車体サイズ目安</label>
</div>
<p>左：ドラッグで回転、ホイールで拡大。右：上面図。右のセルをクリックすると保持値を表示。
N変更は同じ入力から並行計算した状態を切り替えます。</p>
<div class="views"><canvas id="scene"></canvas><canvas id="top"></canvas></div>
<div class="legend"><span class="green">緑：ground仮説</span> ／ <span class="orange">橙：thin</span> ／
<span class="cyan">青緑：broad（z方向に広い）</span> ／ <span class="purple">紫：sparse（1点）</span><br>
候補外包の色は幾何分類でありhazard色ではありません。青い線は対応候補の短期平均z。
hazardは白0→黒100、紫は全cue unknown。上面図の薄灰背景は保持データなし（安全とは扱わない）。
橙の枠は確認待ちのobstacle、赤の枠は確認済み。
obstacle未確認でもterrain要因で黒になることがあります。</div>
<pre id="info"></pre><pre id="pick">上面図のセルをクリックしてください。</pre>
<details><summary>入力・表示制限</summary><pre id="meta"></pre></details>
<script>
'use strict';
const doc=__SCENE_JSON__, frames=doc.frames, $=id=>document.getElementById(id);
let yaw=-0.75,pitch=0.65,zoom=75,drag=null,timer=null,selected=null;
const colors={ground_provisional:'#118349',thin:'#b86400',broad:'#007c9b',sparse:'#8054b4'};
const frame=()=>frames[Number($('frame').value)], N=()=>$('window').value;
$('frame').max=frames.length-1;$('meta').textContent=JSON.stringify(doc.metadata,null,2);
function resize(){for(const id of ['scene','top']){const c=$(id);c.width=c.clientWidth*devicePixelRatio;c.height=c.clientHeight*devicePixelRatio;}draw();}
function project(p){const f=frame(),x=p[0]-f.pose[0],y=p[1]-f.pose[1],z=p[2]-f.pose[2];
 const X=Math.cos(yaw)*x-Math.sin(yaw)*y,Y=Math.sin(yaw)*x+Math.cos(yaw)*y;
 return [$( 'scene').clientWidth/2+zoom*X,$('scene').clientHeight*.64-zoom*(z*Math.cos(pitch)-Y*Math.sin(pitch)),Y*Math.cos(pitch)+z*Math.sin(pitch)];}
function line(ctx,a,b,color,width=1){const p=project(a),q=project(b);ctx.strokeStyle=color;ctx.lineWidth=width;ctx.beginPath();ctx.moveTo(p[0],p[1]);ctx.lineTo(q[0],q[1]);ctx.stroke();}
function bounds(ctx,b,color,width=1){const vs=[];for(let z=0;z<2;z++)for(let y=0;y<2;y++)for(let x=0;x<2;x++)vs.push([b[x],b[2+y],b[4+z]]);
 for(let i=0;i<8;i++)for(const bit of [1,2,4])if(!(i&bit))line(ctx,vs[i],vs[i|bit],color,width);}
function costColor(v){if(v===null)return '#b588c7';const a=Math.round(255*(1-v/100));return `rgb(${a},${a},${a})`;}
function draw(){const f=frame(),n=N(),c=$('scene'),ctx=c.getContext('2d');ctx.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0);ctx.clearRect(0,0,c.clientWidth,c.clientHeight);
 const r=doc.metadata.view_radius_m;
 // 1m格子は尺度の参照であり、地面推定のmeshではない。
 for(let a=-Math.ceil(r);a<=Math.ceil(r);a++){
 line(ctx,[f.pose[0]+a,f.pose[1]-r,f.pose[2]-.12],[f.pose[0]+a,f.pose[1]+r,f.pose[2]-.12],'#e0e6ec');
 line(ctx,[f.pose[0]-r,f.pose[1]+a,f.pose[2]-.12],[f.pose[0]+r,f.pose[1]+a,f.pose[2]-.12],'#e0e6ec');}
 if($('hazard').checked){const tiles=[...f.tiles[n]].sort((a,b)=>project(a.slice(0,3))[2]-project(b.slice(0,3))[2]);
 for(const t of tiles){const d=doc.metadata.resolution/2,vs=[[t[0]-d,t[1]-d,t[2]],[t[0]+d,t[1]-d,t[2]],[t[0]+d,t[1]+d,t[2]],[t[0]-d,t[1]+d,t[2]]].map(project);
 ctx.fillStyle=costColor(t[3]);ctx.globalAlpha=.65;ctx.beginPath();vs.forEach((p,i)=>i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]));ctx.closePath();ctx.fill();ctx.globalAlpha=1;}}
 if($('points').checked){ctx.fillStyle='#4c5968';for(const p of f.points){const q=project(p);ctx.fillRect(q[0]-1,q[1]-1,2,2);}}
 if($('boxes').checked)for(const o of f.objects){const b=o.bounds;
 bounds(ctx,b,colors[o.kind]||'#666',o.kind==='broad'?2:1);
 if(o.kind==='sparse'){const q=project([(b[0]+b[1])/2,(b[2]+b[3])/2,b[4]]);ctx.fillStyle=colors.sparse;ctx.beginPath();ctx.arc(q[0],q[1],4,0,2*Math.PI);ctx.fill();}}
 if($('means').checked)for(const o of f.objects)line(ctx,[o.bounds[0],(o.bounds[2]+o.bounds[3])/2,o.mean_z[n]],[o.bounds[1],(o.bounds[2]+o.bounds[3])/2,o.mean_z[n]],'#255ac9',2);
 if($('robot').checked){const base=f.pose;
 const transform=(x,y,z)=>[base[0]+Math.cos(base[3])*x-Math.sin(base[3])*y,base[1]+Math.sin(base[3])*x+Math.cos(base[3])*y,base[2]+z];
 const vs=[];for(let z of [0,.22])for(let y of [-.225,.225])for(let x of [-.275,.275])vs.push(transform(x,y,z));
 for(let i=0;i<8;i++)for(const bit of [1,2,4])if(!(i&bit))line(ctx,vs[i],vs[i|bit],'#146fb4',2);
 line(ctx,transform(0,0,.1),transform(.65,0,.1),'#146fb4',3);}
 const axes=[[1,0,0,'#c83737','x'],[0,1,0,'#118349','y'],[0,0,1,'#255ac9','z']];
 for(const [x,y,z,color,label] of axes){const p=[f.pose[0]+x,f.pose[1]+y,f.pose[2]+z];line(ctx,f.pose,p,color,2);const q=project(p);ctx.fillStyle=color;ctx.fillText(label+' 1m',q[0]+3,q[1]);}
 drawTop();$('time').textContent=` ${Number($('frame').value)+1}/${frames.length}  ${f.seconds.toFixed(2)}s`;
 $('info').textContent=`${doc.metadata.source}\n採用frame #${f.frame_index}、候補 ${f.objects.length}、表示点 ${f.points.length}/${f.full_point_count}\n`+
 `今回平面validセル ${f.valid_cells}、保持obstacle 確認待ち ${f.states[n].pending}／確認済み ${f.states[n].confirmed}\n`+
 `今回解除 ${f.clears[n]}（これは直前の採用frame1枚のイベント数。表示snapshot間の全件数ではありません）\n`+
 `N=${n}、保持hazardは全cueの最大値。物体枠は今回frameだけで、見えなくなった箱を実在物として残しません。`;
 if(selected)showPick();}
function drawTop(){const c=$('top'),ctx=c.getContext('2d'),f=frame(),r=doc.metadata.view_radius_m;
 ctx.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0);ctx.clearRect(0,0,c.clientWidth,c.clientHeight);
 ctx.fillStyle='#e5eaf0';ctx.fillRect(0,0,c.clientWidth,c.clientHeight);
 const s=Math.min(c.clientWidth,c.clientHeight)/(2*r),xy=(x,y)=>[c.clientWidth/2+(x-f.pose[0])*s,c.clientHeight/2-(y-f.pose[1])*s];
 if($('hazard').checked)for(const t of f.tiles[N()]){const p=xy(t[0],t[1]),d=doc.metadata.resolution*s;
 ctx.fillStyle=costColor(t[3]);ctx.fillRect(p[0]-d/2,p[1]-d/2,d,d);
 if(t[5]!=='inactive'){ctx.strokeStyle=t[5]==='confirmed'?'#d21f37':'#ee9d00';ctx.lineWidth=1;ctx.strokeRect(p[0]-d/2,p[1]-d/2,d,d);}}
 if($('boxes').checked)for(const o of f.objects){const p=xy(o.bounds[0],o.bounds[3]),w=(o.bounds[1]-o.bounds[0])*s,h=(o.bounds[3]-o.bounds[2])*s;
 ctx.strokeStyle=colors[o.kind];ctx.strokeRect(p[0],p[1],Math.max(2,w),Math.max(2,h));}
 if($('robot').checked){ctx.save();ctx.translate(c.clientWidth/2,c.clientHeight/2);ctx.rotate(-f.pose[3]);ctx.strokeStyle='#146fb4';ctx.lineWidth=2;ctx.strokeRect(-.275*s,-.225*s,.55*s,.45*s);ctx.beginPath();ctx.moveTo(0,0);ctx.lineTo(.65*s,0);ctx.stroke();ctx.restore();}
 ctx.fillStyle='#182335';ctx.fillText('odom上面図：上が +y、右が +x',10,20);
 ctx.strokeStyle='#182335';ctx.beginPath();ctx.moveTo(15,c.clientHeight-20);ctx.lineTo(15+s,c.clientHeight-20);ctx.stroke();ctx.fillText('1m',15,c.clientHeight-25);}
function showPick(){const f=frame(),tiles=f.tiles[N()],t=tiles.find(v=>Math.floor(v[0]/doc.metadata.resolution)===selected[0]&&Math.floor(v[1]/doc.metadata.resolution)===selected[1]);
 const objects=f.objects.filter(o=>o.cell[0]===selected[0]&&o.cell[1]===selected[1]);
 $('pick').textContent=JSON.stringify({cell:selected,held_tile:t?{xyz:t.slice(0,3),hazard:t[3],missing_cues:t[4],obstacle_state:t[5],black_causes:t[6],obstacle_output_m:t[7]}:'保持データなし（未観測または表示半径外）',current_candidates:objects},null,2);}
$('top').addEventListener('click',e=>{const c=$('top'),rect=c.getBoundingClientRect(),s=Math.min(c.clientWidth,c.clientHeight)/(2*doc.metadata.view_radius_m),f=frame();
 selected=[Math.floor((f.pose[0]+(e.clientX-rect.left-c.clientWidth/2)/s)/doc.metadata.resolution),Math.floor((f.pose[1]-(e.clientY-rect.top-c.clientHeight/2)/s)/doc.metadata.resolution)];showPick();});
$('scene').addEventListener('pointerdown',e=>{drag=[e.clientX,e.clientY];$('scene').setPointerCapture(e.pointerId);});
$('scene').addEventListener('pointermove',e=>{if(!drag)return;yaw+=(e.clientX-drag[0])*.008;pitch=Math.max(.08,Math.min(1.5,pitch+(e.clientY-drag[1])*.006));drag=[e.clientX,e.clientY];draw();});
$('scene').addEventListener('pointerup',()=>drag=null);$('scene').addEventListener('pointercancel',()=>drag=null);
$('scene').addEventListener('wheel',e=>{e.preventDefault();zoom=Math.max(15,Math.min(400,zoom*Math.exp(-e.deltaY*.001)));draw();},{passive:false});
$('reset').onclick=()=>{yaw=-.75;pitch=.65;zoom=75;draw();};
$('play').onclick=()=>{if(timer){clearInterval(timer);timer=null;$('play').textContent='再生';return;}
 $('play').textContent='停止';timer=setInterval(()=>{$('frame').value=(Number($('frame').value)+1)%frames.length;draw();},500);};
for(const id of ['frame','window','points','boxes','means','hazard','robot'])$(id).addEventListener('input',draw);
// 配布資料から特定のsnapshot/Nへ直接リンクできる。入力範囲外は既定へ戻す。
const initial=new URLSearchParams(location.hash.slice(1));
if(initial.has('frame'))$('frame').value=Math.max(0,Math.min(frames.length-1,Number(initial.get('frame'))||0));
if(['1','3','5'].includes(initial.get('n')))$('window').value=initial.get('n');
window.addEventListener('resize',resize);resize();
</script></html>'''
