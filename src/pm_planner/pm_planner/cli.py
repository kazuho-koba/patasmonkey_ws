"""ROS graphを作らずmissionを投影・補間し、JSON／SVGへ保存する。"""
import argparse
from dataclasses import asdict,replace
import hashlib
import json
from pathlib import Path
from xml.sax.saxutils import escape
from .io import read_mission,read_anchor,read_limits
from .reference import build_reference


def write_svg(path,points,waypoints,reference):
    """metric XYの形状・元点・円弧を表示する。地図tileや走行可能領域ではない。"""
    geometry=[(p[0],p[1]) for p in reference.samples] if reference else points
    xs=[p[0] for p in points+geometry];ys=[p[1] for p in points+geometry]
    xmin,xmax,ymin,ymax=min(xs),max(xs),min(ys),max(ys)
    scale=min(800/max(xmax-xmin,1.),600/max(ymax-ymin,1.))
    def xy(p):return (60+(p[0]-xmin)*scale,680-(p[1]-ymin)*scale)
    def polyline(ps,color):
        coords=' '.join('%.2f,%.2f'%xy(p) for p in ps)
        return '<polyline points="%s" fill="none" stroke="%s" stroke-width="2"/>'%(coords,color)
    lines=['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="760" viewBox="0 0 1000 760">',
           '<rect width="1000" height="760" fill="white"/>',
           '<text x="20" y="30" font-family="sans-serif" font-size="20">Stage 1: preview only / no motor output</text>',
           '<text x="20" y="55" font-family="sans-serif">Gray: mission polyline / Blue: geometry-checked reference</text>',
           polyline(points,'#888')]
    if reference:lines.append(polyline(geometry,'#1876cc'))
    for i,(point,waypoint) in enumerate(zip(points,waypoints)):
        x,y=xy(point)
        lines.append('<circle cx="%.2f" cy="%.2f" r="4" fill="#b32653"/>'%(x,y))
        lines.append('<text x="%.2f" y="%.2f" font-family="sans-serif" font-size="12">%s</text>'%(
            x+6,y-6,escape('%d %s'%(i+1,waypoint['attribute']))))
    lines.append('</svg>');path.write_text('\n'.join(lines)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mission');parser.add_argument('--anchor',required=True)
    parser.add_argument('--vehicle-config',required=True);parser.add_argument('--output',required=True)
    parser.add_argument('--spacing',type=float,default=.1)
    parser.add_argument('--reference-turn-margin',type=float,default=.1,help='車両最小半径に加える参照Path半径の余裕[m]')
    parser.add_argument('--corner-allowance',type=float,default=.3)
    parser.add_argument('--pass-radius',type=float,default=.5)
    parser.add_argument('--sharp-turn-strategy',choices=('symmetric_bulb','reject'),default='symmetric_bulb')
    parser.add_argument('--uturn-direction',choices=('counterclockwise','clockwise'),default='counterclockwise')
    args=parser.parse_args()
    mission=read_mission(args.mission);anchor=read_anchor(args.anchor);limits=read_limits(args.vehicle_config)
    points=[anchor.project(w['latitude'],w['longitude']) for w in mission['waypoints']]
    reference=None;error=None
    try:reference=build_reference(points,mission['waypoints'],limits.with_turn_margin(args.reference_turn_margin),args.spacing,args.corner_allowance,args.pass_radius,
        sharp_turn_strategy=args.sharp_turn_strategy,uturn_direction=args.uturn_direction)
    except ValueError as exc:error=str(exc)
    output=Path(args.output);output.mkdir(parents=True,exist_ok=False)
    result=dict(stage=1,dry_run=True,drive_ready=False,geometry_valid=reference is not None,
                geometry_error=error,frame_id=anchor.frame_id,anchor=asdict(anchor),vehicle_limits=asdict(limits),
                reference_turn_margin_m=args.reference_turn_margin,
                # node側と同じ正規化snapshotをhash化し、YAMLの空白差で別missionにしない。
                mission=mission,mission_sha256=hashlib.sha256(json.dumps(mission,sort_keys=True).encode()).hexdigest(),
                projected_waypoints=points,reference_samples=reference.samples if reference else [],
                events=reference.events if reference else [],length_m=reference.length if reference else None,
                turns=reference.turns if reference else [],sharp_turn_strategy=args.sharp_turn_strategy,
                uturn_direction=args.uturn_direction,environment_checked=False,
                limitations=['幾何previewのみ。衝突・回廊・速度時間割は未検証','manual anchorはlocalizationと自動整合しない'])
    (output/'summary.json').write_text(json.dumps(result,ensure_ascii=False,allow_nan=False,indent=2)+'\n')
    write_svg(output/'reference.svg',points,mission['waypoints'],reference)
    if reference:
        # 全経路の縮尺では小さな折返しが見えないので、鋭角部だけ拡大SVGも作る。
        # 計算経路を変えず、進入・元点・離脱の3ラベルで走行順序を示す。
        for turn in reference.turns:
            if turn['mode']!='symmetric_bulb':continue
            i=turn['waypoint_index']
            local=[p for p in reference.samples if turn['start_s']-1e-8<=p[3]<=turn['end_s']+1e-8]
            labels=[dict(attribute='entry'),dict(attribute='waypoint %d (%s)'%(i+1,turn['direction'])),dict(attribute='exit')]
            write_svg(output/('turn_%03d.svg'%(i+1)),[turn['entry'],points[i],turn['exit']],
                      labels,replace(reference,samples=local))
    print(json.dumps({k:result[k] for k in ('dry_run','drive_ready','geometry_valid','geometry_error','length_m')},ensure_ascii=False))
    if error:raise SystemExit(2)
