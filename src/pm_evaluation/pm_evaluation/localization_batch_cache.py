"""内容を変えない低帯域抽出と、VO/EKF/描画それぞれの依存関係によるcache。

入力のCDR・型・記録ns・順序を長さ付きSHA256へ含める。抽出の前後が一致しない
場合は再生入力として使わない。推定のrate・duration・初期値・実装はcache keyへ含め、
表示や選択した他方式の一覧は数値計算のkeyへ入れない。
"""
import hashlib
import json
import os
from pathlib import Path
import shutil
import sqlite3
import struct
import uuid
from functools import lru_cache

import yaml

from pm_evaluation.localization_batch_io import metadata,records,TOPICS


def digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),ensure_ascii=False).encode()).hexdigest()


def file_hash(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


@lru_cache(maxsize=2048)
def _implementation_hash(path,size,mtime):
    return file_hash(path)


def implementation(files):
    """実装内容で識別し、ホスト/コンテナ上の絶対パスの違いは計算条件にしない。"""
    result=[]
    for p in set(map(Path,files)):
        p=p.resolve();label=str(p)
        for marker in ('/src/','/lib/','/share/'):
            if marker in label:
                label=marker.strip('/')+'/'+label.split(marker,1)[1];break
        st=p.stat();result.append((label,_implementation_hash(str(p),st.st_size,st.st_mtime_ns)))
    return sorted(result)


def source_identity(bag):
    info,paths=metadata(bag)
    return digest(dict(metadata=info,parts=[(p.name,p.stat().st_size,p.stat().st_mtime_ns) for p in paths]))


def _add(h,row):
    ns,topic,name,raw=row
    h.update(struct.pack('<q',int(ns)))
    for field in (topic.encode(),name.encode(),bytes(raw)):
        h.update(struct.pack('<Q',len(field)));h.update(field)


def record_digest(bag,topics=TOPICS):
    """記録順の完全一致を検査する。headerもCDR内にそのまま含まれる。"""
    h=hashlib.sha256();count=0
    for row in records(bag,topics):_add(h,row);count+=1
    return dict(sha256=h.hexdigest(),count=count)


def analysis_timestamp():
    """開発コンテナがUTCでも、閲覧用directoryは利用者のJSTに揃える。"""
    from datetime import datetime,timezone,timedelta
    return datetime.now(timezone(timedelta(hours=9)))


def analysis_directory(root, key, stamp=None, reuse=True, ready=None):
    """人が読む日時名と内部のcache keyを分離する。

    同じkeyの既存日時directoryはready判定も通過した場合のみ再利用する。
    失敗・未完了を再試行するときは別directoryへ保存する。新規計算はJSTの分までを
    名前にし、同分の別条件や強制再実行は_02以降で区別する。mkdirの排他性で
    同時起動時の衝突も避ける。既存ハッシュdirectoryは移動せず残す。
    """
    from datetime import datetime
    root=Path(root);root.mkdir(parents=True,exist_ok=True)
    if reuse:
        for folder in sorted(root.iterdir(),reverse=True):
            marker=folder/'analysis_identity.json'
            try:
                if (folder.is_dir() and json.loads(marker.read_text()).get('key')==key
                        and (ready is None or ready(folder))):
                    return folder
            except (OSError,ValueError):continue
    stamp=stamp or analysis_timestamp().strftime('%Y%m%d_%H%M')
    number=1
    while True:
        folder=root/(stamp if number==1 else stamp+'_%02d'%number)
        try:folder.mkdir()
        except FileExistsError:number+=1;continue
        (folder/'analysis_identity.json').write_text(json.dumps(dict(key=key,created_at=analysis_timestamp().isoformat()),indent=2))
        return folder


def extract_lowband(bag,root):
    """一度だけSQLiteへ抽出し、書込直後の全recordを再読して一致を確認する。

    大きい元bagの内容を毎回hashし直す代わりに、元metadata・各partのサイズ/mtimeで
    source変更を検知する。cache自体はSHA256も検査し、書換えを見逃さない。
    """
    identity=source_identity(bag)
    key=digest(dict(version=1,source=identity,topics=TOPICS))
    destination=analysis_directory(root,key)
    marker=destination/'verified.json'
    if marker.exists():
        saved=json.loads(marker.read_text())
        if (saved.get('source')==identity and (destination/'data.db3').is_file()
                and file_hash(destination/'data.db3')==saved.get('database_sha256')
                and file_hash(destination/'metadata.yaml')==saved.get('metadata_sha256')):
            return destination,dict(key=key,status='cached',verification=saved)
    if marker.exists() or (destination/'data.db3').exists():
        # 不完全/変更済みのcacheを別条件の成功結果として再利用しない。
        saved_identity=json.loads((destination/'analysis_identity.json').read_text())
        saved_identity['key']='invalid:'+key
        (destination/'analysis_identity.json').write_text(json.dumps(saved_identity,indent=2))
        destination=analysis_directory(root,key,reuse=False)
        marker=destination/'verified.json'
    temporary=Path(root)/('.building_'+uuid.uuid4().hex);temporary.mkdir(parents=True)
    connection=sqlite3.connect(str(temporary/'data.db3'))
    connection.executescript('''CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT NOT NULL,type TEXT NOT NULL,
serialization_format TEXT NOT NULL,offered_qos_profiles TEXT NOT NULL);
CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER NOT NULL,timestamp INTEGER NOT NULL,data BLOB NOT NULL);
CREATE INDEX timestamp_idx ON messages(timestamp ASC);''')
    topics={};counts={};h=hashlib.sha256();first=last=None;count=0
    try:
        for row in records(bag,TOPICS):
            ns,topic,name,raw=row
            if topic not in topics:
                topics[topic]=(len(topics)+1,name)
                connection.execute('INSERT INTO topics VALUES(?,?,?,?,?)',(topics[topic][0],topic,name,'cdr',''))
            elif topics[topic][1]!=name:raise ValueError('同topic内で型が変更された')
            connection.execute('INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)',(topics[topic][0],ns,raw))
            counts[topic]=counts.get(topic,0)+1;count+=1;_add(h,row)
            first=ns if first is None else min(first,ns);last=ns if last is None else max(last,ns)
        connection.commit()
    finally:connection.close()
    info=dict(version=4,storage_identifier='sqlite3',relative_file_paths=['data.db3'],
              duration={'nanoseconds':0 if first is None else last-first},
              starting_time={'nanoseconds_since_epoch':first or 0},message_count=count,
              compression_format='',compression_mode='',topics_with_message_count=[
                  dict(message_count=counts[t],topic_metadata=dict(name=t,type=value[1],serialization_format='cdr',offered_qos_profiles=''))
                  for t,value in topics.items()])
    (temporary/'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information':info},sort_keys=False))
    if source_identity(bag)!=identity:raise ValueError('抽出中に元bagが変更されたため中止')
    after=record_digest(temporary,TOPICS)
    if after!={'sha256':h.hexdigest(),'count':count}:
        raise ValueError('低帯域抽出で内容/時刻/順序が変化したため使用しない: '+str(temporary))
    saved=dict(source=identity,records=after,database_sha256=file_hash(temporary/'data.db3'),
               metadata_sha256=file_hash(temporary/'metadata.yaml'))
    (temporary/'verified.json').write_text(json.dumps(saved,indent=2))
    # 完全照合後に公開し、verified.jsonを最後に置いて完了を示す。
    for name in ('data.db3','metadata.yaml'):
        shutil.move(str(temporary/name),str(destination/name))
    shutil.move(str(temporary/'verified.json'),str(destination/'verified.json'))
    temporary.rmdir()
    return destination,dict(key=key,status='extracted_verified',verification=saved)


def vio_key(source,variant,rate,duration,settings_files,code_files):
    return digest(dict(stage='vio_v1',source=source,variant=variant,rate=rate,duration=duration,
                       settings=implementation(settings_files),code=implementation(code_files)))


def ekf_key(job,folder,input_digest,code_files,workers):
    """有効YAML・配線・初期stateで判定。VOを使わないjobへVO条件を混ぜない。"""
    specs=[]
    for node in job['nodes']:
        specs.append(dict(package=node['package'],executable=node['executable'],name=node['name'],
            remaps=node['remaps'],parameters=yaml.safe_load((Path(folder)/node['parameters']).read_text())))
    return digest(dict(stage='ekf_v1',input=input_digest,nodes=specs,rate=job['rate'],duration=job['duration'],
                       workers=workers,outputs=job['outputs'],code=implementation(code_files)))


def render_key(input_key,map_file,code_files):
    return digest(dict(stage='render_v1',input=input_key,
                       basemap=file_hash(map_file) if map_file and Path(map_file).is_file() else None,
                       code=implementation(code_files)))


def render_needed(folder,key,required,resume=True):
    marker=Path(folder)/'render_complete.json'
    return not (resume and marker.exists() and json.loads(marker.read_text()).get('key')==key
                and all((Path(folder)/name).is_file() for name in required))


def mark_render(folder,key):
    (Path(folder)/'render_complete.json').write_text(json.dumps({'key':key},indent=2))


def stage_code_files():
    """描画の編集でVO/EKF cacheを無効にしないよう依存ファイルを分離する。"""
    from ament_index_python.packages import get_package_prefix
    import pm_localization
    package=Path(__file__).parent
    io=package/'localization_batch_io.py'
    transport=[io,package/'localization_vio_replay.py']
    ekf=[package/'localization_batch_plan.py',package/'localization_batch_parallel.py',
         package/'cli/batch_localization_worker.py',package/'cli/batch_localization_group_worker.py',*transport]
    ekf+=list(Path(pm_localization.__file__).parent.glob('*.py'))
    native=Path(get_package_prefix('robot_localization'))/'lib'
    ekf+=[native/'robot_localization/ekf_node',native/'robot_localization/navsat_transform_node']
    ekf+=list(native.glob('librl_lib.so*'))
    render=[package/'localization_batch_plot.py',package/'cli/analyze_localization_batch.py',
            package/'cli/plot_bag_trajectories.py',*transport]
    return dict(ekf=ekf,render=render,vo=[package/'cli/batch_vio_worker.py',*transport])


def result_ready(folder):
    """計算完了印に対応するbagが変更/欠落していたら再計算する。"""
    folder=Path(folder)
    try:
        saved=json.loads((folder/'complete.json').read_text())
        return saved.get('status')=='complete' and saved.get('storage')==source_identity(folder/'output_bag')
    except (OSError,ValueError,KeyError):return False


def vo_ready(folder):
    folder=Path(folder)
    try:
        saved=json.loads((folder/'complete.json').read_text())
        return saved.get('status')=='complete' and saved.get('storage')==source_identity(folder/'merged_input')
    except (OSError,ValueError,KeyError):return False
