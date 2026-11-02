"""座標規約、円弧の幾何制約、順序付き通過・停止属性の非駆動テスト。"""
import math
from pathlib import Path
import pytest

from pm_planner.geodesy import Anchor,ecef
from pm_planner.reference import Limits,build_reference,project_local
from pm_planner.progress import Progress
from pm_planner.io import read_mission,read_limits


def test_reference_radius_margin_uses_vehicle_limits():
    """参照経路は車体の最小半径に余裕を加え、車体制約は変更しない。"""
    base=Limits(.3,1.,1.,1.)
    planned=base.with_turn_margin(.1)
    assert planned.radius == pytest.approx(.4)
    assert base.radius == pytest.approx(.3)
    with pytest.raises(ValueError):base.with_turn_margin(-.01)


def test_vehicle_limits_follow_source_symlink(tmp_path):
    """install側リンクを維持したままsrc側のYAMLを更新すると再読込に反映される。"""
    source=tmp_path/'source.yaml'
    link=tmp_path/'installed.yaml'
    link.symlink_to(source)
    def write(radius):
        source.write_text('vehicle_interface_node:\n  ros__parameters:\n'
                          '    min_turn_radius: {}\n    autonomous_max_linear_speed: 1.0\n'
                          '    max_yaw_rate: 1.0\n    max_yaw_accel: 1.0\n'.format(radius))
    write(.3)
    assert read_limits(link).radius==pytest.approx(.3)
    write(.4)
    assert read_limits(link).radius==pytest.approx(.4)


@pytest.mark.parametrize('efficiency',[.25,.5,1.])
def test_skid_efficiency_does_not_modify_body_limits(tmp_path,efficiency):
    """スキッド補正を二重に掛けず、車体上限と半径を設定どおり取得する。"""
    import yaml
    source=tmp_path/'vehicle_control.yaml'
    source.write_text(yaml.safe_dump({'vehicle_interface_node':{'ros__parameters':{
        'min_turn_radius':.3,'autonomous_max_linear_speed':2.,'max_linear_speed':1.,
        'max_yaw_rate':1.,'max_yaw_accel':2.,'skid_steer_yaw_efficiency':efficiency}}}))
    limits=read_limits(source)
    assert limits.radius==pytest.approx(.3)
    assert limits.max_speed==pytest.approx(1.)
    assert limits.max_yaw_rate==pytest.approx(1.)
    assert limits.max_yaw_accel==pytest.approx(2.)


LIMITS=Limits(.4,1.,1.,1.)


def waypoints(kinds):
    return [dict(waypoint_id='point%d'%i,attribute=kind,pause_sec=1.,note='') for i,kind in enumerate(kinds)]


def line(kinds=('start','pass','goal')):
    points=[(float(i),0.) for i in range(len(kinds))]
    return build_reference(points,waypoints(kinds),LIMITS)


def test_geographic_axes_and_anchor():
    anchor=Anchor(35.,139.,10.,'test',x=3.,y=4.)
    assert anchor.project(35.,139.)==pytest.approx((3.,4.),abs=1e-8)
    east=anchor.project(35.,139.00001);north=anchor.project(35.00001,139.)
    assert east[0]>3.8 and abs(east[1]-4)<.001
    assert north[1]>5. and abs(north[0]-3)<.001
    rotated=Anchor(35.,139.,10.,'test',east_yaw_rad=math.pi/2)
    x,y=rotated.project(35.,139.00001)
    assert abs(x)<.001 and y>.8
    with pytest.raises(ValueError):Anchor(91.,139.,0.,'test')
    with pytest.raises(ValueError):Anchor(35.,139.,0.,'')


def test_ecef_against_pyproj():
    pyproj=pytest.importorskip('pyproj')
    transformer=pyproj.Transformer.from_crs(4979,4978,always_xy=True)
    assert ecef(35.,139.,10.)==pytest.approx(transformer.transform(139.,35.,10.),abs=1e-6)


def test_fillet_geometry_and_waypoint_station():
    ref=build_reference([(0.,0.),(2.,0.),(2.,2.)],waypoints(('start','pass','goal')),LIMITS)
    assert ref.length==pytest.approx(4.-.8+math.pi*.4/2)
    assert ref.events[1]['station']==pytest.approx(1.6+math.pi*.4/4)
    for a,b in zip(ref.samples,ref.samples[1:]):
        assert 0<b[3]-a[3]<=.10000001
        assert math.hypot(b[0]-a[0],b[1]-a[1])<=.10000001
        yaw_difference=math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]))
        assert abs(yaw_difference)/(b[3]-a[3])<=1/.4+1e-6
    assert ref.samples[0][:2]==(0.,0.) and ref.samples[-1][:2]==pytest.approx((2.,2.))


@pytest.mark.parametrize('points,kinds',[
    ([(0.,0.),(0.,0.),(1.,0.)],('start','pass','goal')),
    ([(0.,0.),(1.,0.),(1.,1.)],('start','pause','goal')),
    ([(0.,0.),(.5,0.),(.5,.5),(1.,.5)],('start','pass','pass','goal'))])
def test_invalid_paths_are_rejected(points,kinds):
    with pytest.raises(ValueError):build_reference(points,waypoints(kinds),LIMITS)


def test_home_can_round_turn_and_legacy_rejection_is_selectable():
    mission=read_mission(Path(__file__).resolve().parents[2]/'pm_mission/missions/home.yaml')
    first=mission['waypoints'][0];anchor=Anchor(first['latitude'],first['longitude'],0.,'preview')
    points=[anchor.project(w['latitude'],w['longitude']) for w in mission['waypoints']]
    ref=build_reference(points,mission['waypoints'],Limits(.3,1.,1.,1.))
    turn=next(t for t in ref.turns if t['waypoint_index']==4)
    assert turn['mode']=='symmetric_bulb' and turn['radius_m']==.3
    assert turn['trim_m']<.83
    progress=Progress(ref);first=ref.samples[0]
    progress.start((0.,first[0],first[1],first[2],0.))
    for x,y,yaw,s in ref.samples[1:]:
        progress.update((s/.25,x,y,yaw,.25))
        assert progress.state!='FAULT',progress.reason
    last=ref.samples[-1]
    progress.update((last[3]/.25+.1,last[0],last[1],last[2],0.))
    assert progress.state=='COMPLETE'
    assert progress.completed==[w['waypoint_id'] for w in mission['waypoints']]
    with pytest.raises(ValueError,match='waypoint 5'):
        build_reference(points,mission['waypoints'],LIMITS,sharp_turn_strategy='reject')


def test_start_requires_position_heading_and_stop():
    p=Progress(line())
    for pose in ((0.,2.,0.,0.,0.),(0.,0.,0.,math.pi,0.),(0.,0.,0.,0.,.5)):
        with pytest.raises(ValueError):p.start(pose)
    assert p.state=='READY'


def test_progress_and_stationary_goal():
    p=Progress(line());p.start((0.,0.,0.,0.,0.))
    for i in range(1,21):p.update((i*.2,i*.1,0.,0.,.5))
    assert p.state=='STOP_REQUIRED'
    p.update((4.2,2.,0.,0.,0.));assert p.state=='COMPLETE'
    assert p.completed==['point0','point1','point2']


def test_pause_and_confirm_do_not_skip_stop():
    p=Progress(line(('start','pause','confirm','goal')));p.start((0.,0.,0.,0.,0.))
    stamp=0.
    for i in range(1,11):stamp+=.2;p.update((stamp,i*.1,0.,0.,.5))
    assert p.state=='STOP_REQUIRED'
    stamp+=.2;p.update((stamp,1.,0.,0.,0.));assert p.state=='WAITING_PAUSE'
    for _ in range(6):stamp+=.2;p.update((stamp,1.,0.,0.,0.))
    assert p.state=='RUNNING' and p.index==2
    for i in range(11,21):stamp+=.2;p.update((stamp,i*.1,0.,0.,.5))
    stamp+=.2;p.update((stamp,2.,0.,0.,0.));assert p.state=='WAITING_CONFIRM'
    for _ in range(10):stamp+=.2;p.update((stamp,2.,0.,0.,0.))
    assert p.state=='WAITING_CONFIRM'
    p.confirm();assert p.index==3 and p.state=='RUNNING'


def test_leaving_confirm_point_requires_rearrival():
    p=Progress(line(('start','confirm','goal')));p.start((0.,0.,0.,0.,0.))
    for i in range(1,11):p.update((i*.2,i*.1,0.,0.,0.))
    assert p.state=='WAITING_CONFIRM'
    p.update((2.2,1.,.3,0.,0.))
    assert p.state!='WAITING_CONFIRM'
    with pytest.raises(ValueError):p.confirm()


def test_pose_jump_and_clock_rewind_fault():
    p=Progress(line());p.start((0.,0.,0.,0.,0.));p.update((.1,1.5,0.,0.,0.))
    assert p.state=='FAULT'
    p=Progress(line());p.start((1.,0.,0.,0.,0.));p.update((.9,0.,0.,0.,0.))
    assert p.state=='FAULT'


def test_local_projection_does_not_jump_to_return_leg():
    points=[(0.,0.),(5.,0.),(5.,5.),(0.,5.),(0.,.3)]
    ref=build_reference(points,waypoints(('start','pass','pass','pass','goal')),LIMITS)
    distance,station=project_local(ref,0.,.3,0.,2.)
    assert distance==pytest.approx(.3) and station<1.


def test_pause_is_not_auto_resumed_by_new_pose():
    p=Progress(line());p.start((0.,0.,0.,0.,0.));p.pause('pose stale')
    p.update((.2,.1,0.,0.,0.));assert p.state=='MANUAL_PAUSED'
    p.resume((.3,.1,0.,0.,0.));assert p.state=='RUNNING'


def test_cancel_cannot_be_resumed():
    p=Progress(line());p.start((0.,0.,0.,0.,0.));p.cancel()
    p.update((.2,.1,0.,0.,0.));assert p.state=='CANCELED'
    with pytest.raises(ValueError):p.resume((.3,.1,0.,0.,0.))
    with pytest.raises(ValueError):p.start((.3,.1,0.,0.,0.))


@pytest.mark.parametrize('degrees',[130.,173.46,179.999,180.,-130.,-173.46,-179.999,-180.])
def test_bulb_symmetry_waypoint_curvature_and_progress(degrees):
    """左右・180度近傍を含め、対称性と元点通過、同じ経路での進捗を検証する。"""
    theta=math.radians(degrees);u=(1.,0.);v=(math.cos(theta),math.sin(theta))
    points=[(-3.,0.),(0.,0.),(3*v[0],3*v[1])]
    ref=build_reference(points,waypoints(('start','pass','goal')),Limits(.3,1.,1.,1.),spacing=.025,
                        uturn_direction='clockwise' if degrees<0 else 'counterclockwise')
    turn=ref.turns[0];assert turn['mode']=='symmetric_bulb'
    assert turn['direction']==('clockwise' if degrees<0 else 'counterclockwise')
    assert math.hypot(*turn['entry'])==pytest.approx(math.hypot(*turn['exit']))
    assert turn['trim_m']<=.3*(1+math.sqrt(3))+1e-9
    middle=min(ref.samples,key=lambda p:abs(p[3]-turn['event_s']))
    assert middle[:2]==pytest.approx((0.,0.),abs=1e-9)
    # 角二等分線に垂直な軸に対する鏡映が、弧長中点の前後で一致する。
    nx,ny=-math.sin(theta/2),math.cos(theta/2)
    corner=[p for p in ref.samples if turn['start_s']-1e-9<=p[3]<=turn['end_s']+1e-9]
    for a,b in zip(corner,reversed(corner)):
        dot=a[0]*nx+a[1]*ny
        assert b[:2]==pytest.approx((2*dot*nx-a[0],2*dot*ny-a[1]),abs=1e-8)
    for a,b in zip(ref.samples,ref.samples[1:]):
        ds=b[3]-a[3];assert 0<ds<=.02500001
        dyaw=math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]))
        assert abs(dyaw)<=ds/.3+1e-7
        # 接線方向へ進んでおり、後退・位置の不連続を混入させていない。
        mean_yaw=a[2]+dyaw/2
        assert (b[0]-a[0])*math.cos(mean_yaw)+(b[1]-a[1])*math.sin(mean_yaw)>0
    progress=Progress(ref);first=ref.samples[0]
    progress.start((0.,first[0],first[1],first[2],0.))
    for x,y,yaw,s in ref.samples[1:]:
        progress.update((s/.25,x,y,yaw,.25))
        assert progress.state!='FAULT',progress.reason
    last=ref.samples[-1]
    progress.update((last[3]/.25+.1,last[0],last[1],last[2],0.))
    assert progress.state=='COMPLETE' and progress.completed==['point0','point1','point2']


def test_short_segments_and_invalid_turn_policy_are_rejected():
    wp=waypoints(('start','pass','goal'))
    with pytest.raises(ValueError,match='重な'):
        build_reference([(-.4,0.),(0.,0.),(-.4,.01)],wp,Limits(.3,1.,1.,1.))
    with pytest.raises(ValueError,match='sharp_turn_strategy'):
        build_reference([(-3.,0.),(0.,0.),(-3.,.1)],wp,LIMITS,sharp_turn_strategy='invalid')


def test_rejoin_advances_multiple_pass_events_without_consuming_goal():
    """手動で先行した距離に上限を課さず、PASSだけを推定通過にする。"""
    ref=build_reference([(0.,0.),(3.,0.),(6.,0.),(10.,0.)],waypoints(('start','pass','pass','goal')),LIMITS)
    p=Progress(ref);p.start((0.,0.,0.,0.,0.));p.pause()
    p.rejoin((10.,7.,0.,0.,0.))
    assert p.station==pytest.approx(7.) and p.index==3
    assert p.completed==['point0','point1','point2']
    assert p.status()['inferred_completed_waypoint_ids']==['point1','point2']
    p.pause();p.rejoin((11.,10.,0.,0.,0.))
    assert p.state=='RUNNING' and 'point3' not in p.completed
    p.update((11.1,10.,0.,0.,0.));assert p.state=='COMPLETE'


@pytest.mark.parametrize('pose,message',[
    ((10.,1.,2.,0.,0.),'離れ'),((10.,1.,0.,math.pi,0.),'heading'),
    ((10.,1.,0.,0.,.1),'停止')])
def test_rejoin_refusal_preserves_progress(pose,message):
    p=Progress(line());p.start((0.,0.,0.,0.,0.));p.pause();before=p.status()
    with pytest.raises(ValueError,match=message):p.rejoin(pose)
    assert p.status()==before


def test_rejoin_ambiguous_same_heading_segments_are_rejected():
    """交差や近接した同方向の別周回を位置だけで選ばない。"""
    from pm_planner.reference import Reference
    samples=[(0.,0.,0.,0.),(5.,0.,0.,5.),(5.,5.,0.,10.),(0.,5.,0.,15.),(0.,.1,0.,19.9),(5.,.1,0.,24.9)]
    ref=Reference(samples,[dict(waypoint_id='s',attribute='start',station=0.,x=0.,y=0.),
                           dict(waypoint_id='g',attribute='goal',station=24.9,x=5.,y=.1)],LIMITS,.1,[])
    p=Progress(ref);p.start((0.,0.,0.,0.,0.));p.pause()
    with pytest.raises(ValueError,match='曖昧'):p.rejoin((10.,2.,.05,0.,0.))
    assert p.station==0. and p.state=='MANUAL_PAUSED'



def test_rejoin_does_not_skip_required_stop_or_turn_back_after_goal():
    p=Progress(line(('start','pause','goal')));p.start((0.,0.,0.,0.,0.));p.pause()
    with pytest.raises(ValueError,match='停止属性'):p.rejoin((5.,1.5,0.,0.,0.))
    assert p.station==0. and p.index==1
    p=Progress(line());p.start((0.,0.,0.,0.,0.));p.pause()
    with pytest.raises(ValueError,match='行き過ぎ'):p.rejoin((5.,2.6,0.,0.,0.))
    assert p.station==0.


def test_pass_crosses_route_station_at_one_metre_offset_but_visit_does_not():
    """同じ横ずれでも大まかなPASSは通過、必須VISITは未到達で停止する。"""
    result=[]
    for kind in ('pass','visit'):
        ref=build_reference([(0.,0.),(3.,0.),(7.,0.)],waypoints(('start',kind,'goal')),LIMITS)
        p=Progress(ref,max_deviation=2.);p.start((0.,0.,0.,0.,0.))
        for i in range(1,46):
            p.update((i*.2,i*.1,min(1.,i*.05),0.,.5))
            if p.state=='FAULT':break
        result.append(p)
    assert result[0].state=='RUNNING' and result[0].index==2
    assert result[1].state=='FAULT' and result[1].index==1


def test_visit_path_keeps_corner_point_and_cannot_be_skipped_on_rejoin():
    ref=build_reference([(0.,0.),(3.,0.),(3.,3.)],waypoints(('start','visit','goal')),LIMITS)
    assert min(math.hypot(x-3.,y) for x,y,_,_ in ref.samples)<1e-8
    p=Progress(ref,max_deviation=2.);p.start((0.,0.,0.,0.,0.));p.pause()
    with pytest.raises(ValueError,match='停止属性'):p.rejoin((20.,3.,2.,math.pi/2,0.))
