"""記録時の実効parameter・Git・設定を保存する読取り専用の再現情報収集。"""
import datetime
import hashlib
import json
import os
import re
import shutil
import subprocess
from pathlib import Path


def file_sha256(path):
    """大きな実行バイナリを一括メモリコピーせず、固定1 MiB bufferでhash化する。"""
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def redact(value):
    """認証情報をmetadataへ持ち出さない。名前で判別できない秘密は対象外なので注意。"""
    if isinstance(value, dict):
        return {key: ('<redacted>' if re.search(r'password|passwd|secret|token|credential|username', key, re.I)
                      else redact(item)) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [redact(item) for item in value]
    return value


def atomic_json(path, value):
    """書込み途中のJSONを残さず、最新snapshotを置換する。"""
    path = Path(path)
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(redact(value), ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    temporary.replace(path)


def git_info(root, destination):
    """HEAD・履歴・tracked差分・未追跡ソースを保存し、Gitなしも明示する。

    source treeのrevisionであり、実際のinstallと一致する保証はしない。
    private/認証ファイルはコピーしない。重いデータやbagはGit snapshot対象外。
    """
    root, destination = Path(root), Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    def git(*args):
        return subprocess.check_output(['git', '-C', str(root), *args], text=True,
                                       stderr=subprocess.DEVNULL, timeout=10)
    try:
        info = {'path':str(root), 'head':git('rev-parse','HEAD').strip(),
                'branch':git('branch','--show-current').strip(),
                'status':git('status','--porcelain'),
                'recent_commits':git('log','-10','--format=%H %cI %s').splitlines()}
        # ソース差分だけ保存。HEADとの差にはstaged/unstaged双方を含む。
        changed = git('diff','HEAD','--name-only').splitlines()
        untracked = git('ls-files','--others','--exclude-standard').splitlines()
        # 外部repoのinclude/CMake等も含めるが、bag/build/local notesはコピーしない。
        def eligible(relative):
            path = Path(relative)
            if path.parts[0] in ('bags','build','install','log','notes','.ssh','.aws','.codex','.agents'):
                return False
            if re.search(r'private|password|credential|secret',relative,re.I):
                return False
            if path.suffix not in ('.py','.cpp','.cc','.c','.h','.hpp','.yaml','.yml',
                                   '.xml','.urdf','.sh','.cfg','.json','.cmake','.txt','.md','.toml'):
                return False
            return (root/path).is_file() and (root/path).stat().st_size <= 1024*1024
        allowed = [p for p in changed+untracked if eligible(p)]
        # 削除済みtrackedファイルも復元に必要な差分として保存する。
        allowed += [p for p in changed if not (root/p).exists()
                    and not re.search(r'private|password|credential|secret',p,re.I)]
        if changed:
            selected = [p for p in changed if p in allowed]
            patch = git('diff','HEAD','--', *selected) if selected else ''
            (destination/'tracked_changes.patch').write_text(patch,encoding='utf-8')
        info['excluded_paths'] = sorted(set(changed+untracked)-set(allowed))
        info['untracked_copied'] = []
        for relative in untracked:
            source = root/relative
            if relative in allowed and source.is_file() and source.stat().st_size <= 1024*1024:
                target = destination/'untracked'/relative
                target.parent.mkdir(parents=True,exist_ok=True)
                shutil.copy2(source,target)
                info['untracked_copied'].append(relative)
        info['untracked_not_copied'] = sorted(set(untracked)-set(info['untracked_copied']))
        return info
    except (OSError, subprocess.SubprocessError) as error:
        return {'path':str(root),'error':type(error).__name__,'available':False}


def snapshot_versions(destination, workspace, external_workspace):
    """主workspaceと外部workspace配下のGit repoを保存する。ネットワークは使わない。"""
    destination = Path(destination)
    roots = [Path(workspace), Path(external_workspace)]
    source = Path(external_workspace)/'src'
    if source.is_dir():
        roots += [p for p in sorted(source.iterdir()) if (p/'.git').exists()]
    return [git_info(root,destination/('repo_'+str(index))) for index,root in enumerate(roots)]


def copy_configuration(paths, destination):
    """使用したinstall側設定をhash付きで保存する。OpenVINS相対校正も追跡する。"""
    destination = Path(destination)
    destination.mkdir(parents=True,exist_ok=True)
    sources = [Path(os.path.abspath(str(p))) for p in paths]
    entries = []
    for index, source in enumerate(sources):
        if not source.is_file() or re.search(r'private|credential|secret',str(source),re.I):
            entries.append({'source':str(source),'copied':False})
            continue
        text = source.read_text(encoding='utf-8')
        # 秘密keyを含む設定は原文を保存しない。通常の校正/URDFはbyte一致で保存。
        safe = re.sub(r'(?im)^(\s*(?:password|passwd|token|secret|username)\s*[:=]).*$',
                      r'\1 "<redacted>"', text)
        # logicalな絶対pathの階層を保存し、相対calibration参照がそのまま解決できるようにする。
        # resolve()はinstall symlinkを辿って親を変えるため使わない。..だけを正規化する。
        name = str(Path('files')/str(source).lstrip('/'))
        target = destination/name
        target.parent.mkdir(parents=True,exist_ok=True)
        target.write_text(safe,encoding='utf-8')
        entries.append({'source':str(source),'copy':name,'redacted':safe!=text,
                        'sha256':file_sha256(source)})
        for relative in re.findall(r'^\s*relative_config_(?:imu|imucam):\s*["\']?([^"\'#\s]+)',text,re.M):
            child = Path(os.path.abspath(str(source.parent/relative)))
            if child not in sources:
                sources.append(child)
    return entries


def utc_now():
    """同梱情報の取得時刻をUTCで保存する（bag名のローカル時刻とは別）。"""
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def installed_inventory(destination):
    """source commitとは別に、実際のoverlay package prefixと実行ファイルhashを残す。"""
    from ament_index_python.packages import get_package_prefix, PackageNotFoundError
    result = {}
    destination = Path(destination)
    for name in ('pm_bringup','pm_config','pm_description','pm_perception','pm_localization',
                 'pm_vehicle_interface','depthai_driver','ov_msckf','robot_localization',
                 'hwt905_rs485_driver','ublox_gps','ntrip_client'):
        try:
            prefix = Path(get_package_prefix(name))
            files = []
            executable_dir = prefix/'lib'/name
            if executable_dir.is_dir():
                files += [p for p in executable_dir.iterdir() if p.is_file()]
            files += list(prefix.glob('lib/python*/site-packages/'+name+'/**/*.py'))
            hashes = []
            for file in files:
                # バイナリは再コピーせずhashだけ。Python実装は実行時内容をコピーする。
                digest = file_sha256(file)
                hashes.append({'path':str(file),'sha256':digest})
                if file.suffix == '.py':
                    relative = file.relative_to(prefix)
                    target = destination/name/relative
                    target.parent.mkdir(parents=True,exist_ok=True)
                    shutil.copy2(file,target)
            package_xml = prefix/'share'/name/'package.xml'
            result[name] = {'prefix':str(prefix),'files':hashes,
                            'package_xml':package_xml.read_text() if package_xml.is_file() else None}
        except (OSError, PackageNotFoundError) as error:
            result[name] = {'error':type(error).__name__}
    return result
