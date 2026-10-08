#!/usr/bin/env python3
"""前回結果の完全再現を検査し、通過footprintの黒原因・出典時刻を集計する。"""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path

import numpy as np


def distribution(values):
    """非有限値を統計に混ぜず、少数標本も件数とともに報告する。"""
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return dict(count=len(values), min=float(values.min()), median=float(np.median(values)),
                p95=float(np.percentile(values, 95)), max=float(values.max())) if len(values) else dict(count=0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory'); parser.add_argument('--reference', required=True)
    args = parser.parse_args(); folder = Path(args.directory); reference = Path(args.reference)
    summary = json.loads((folder/'summary.json').read_text())
    original = json.loads((reference/'summary.json').read_text())
    # N変更時はraw幅とN1等の共通対照を照合。異なるNの確認済み出力が違うのは意図通り。
    common_modes = set(original['results']) & set(summary['results'])
    for mode in common_modes:
        result = original['results'][mode]
        if any(summary['results'][mode][k] != v for k, v in result.items()):
            raise ValueError('前回summaryの数値と不一致: '+mode)
    old_rows = list(csv.DictReader((reference/'footprints.csv').open()))
    new_rows = list(csv.DictReader((folder/'footprints.csv').open()))
    old_common = [r for r in old_rows if r['mode'] in common_modes]
    new_common = [r for r in new_rows if r['mode'] in common_modes]
    if len(old_common) != len(new_common) or any(any(new[k] != v for k,v in old.items())
                                            for old,new in zip(old_common,new_common)):
        raise ValueError('前回footprintのstamp／既存値と不一致')
    cells = list(csv.DictReader((folder/'black_cells.csv').open()))
    confirmed_mode = next(mode for mode in summary['results'] if mode.endswith('_confirmed'))
    conservative_mode = next(mode for mode in summary['results'] if mode.endswith('_conservative'))
    modes = ('N1', confirmed_mode, conservative_mode)
    output = dict(common_mode_summary_identical=True, common_mode_footprint_rows_identical=True,
                  compared_reference_modes=sorted(common_modes), modes={})
    units = {'slope':'slope_deg', 'roughness':'roughness_m', 'step':'step_m', 'obstacle':'obstacle_m'}
    for mode in modes:
        selected = [r for r in cells if r['mode'] == mode]
        count = summary['results'][mode]['black_cells']
        if len(selected) != count:
            raise ValueError('黒セルCSVとfootprint集計の件数が不一致')
        result = dict(summary['results'][mode]['cause_analysis'])
        result['exclusive_cell_combinations'] = dict(Counter(r['causes'] for r in selected))
        result['black_value_and_source_age'] = {}
        for cue, field in units.items():
            rows = [r for r in selected if cue in r['causes'].split('+')]
            stamp_field = 'obstacle_first_positive_stamp_ns' if cue == 'obstacle' else 'terrain_stamp_ns'
            ages = [(int(r['passage_stamp_ns'])-int(r[stamp_field]))/1e9 for r in rows if r[stamp_field]]
            if any(age <= 0 for age in ages):
                raise ValueError('通過前でない観測が原因に混入しています')
            result['black_value_and_source_age'][cue] = dict(
                values=distribution([float(r[field]) for r in rows]),
                source_age_seconds=distribution(ages))
        # 判定要因（地形cue）とは別に、その黒がどの観測状態で残っていたかを集計。
        # 未観測はFOV外・欠測・sampling等を区別できないので一括とする。
        states = Counter(); state_footprints = {}; latest_invalid_bits = Counter()
        for r in selected:
            if r['last_cell_observation_stamp_ns'] != r['latest_depth_stamp_ns']:
                state = 'unobserved_in_latest_frame'
            elif int(r['plane_reason_bits'] or 0) != 0:
                state = 'observed_but_plane_invalid'
                bits = int(r['plane_reason_bits'])
                for bit, name in [(1,'self_unknown'),(2,'support_low'),(4,'degenerate'),(8,'slope_gate'),(16,'roughness_gate')]:
                    latest_invalid_bits[name] += bool(bits & bit)
            elif r['above_plane_hit'] == '0':
                state = 'observed_plane_valid_without_high_hit'
            else:
                state = 'observed_plane_valid_with_high_hit'
            states[state] += 1
            state_footprints.setdefault(state,set()).add(r['passage_stamp_ns'])
        result['latest_frame_state_all_black_cells'] = dict(states)
        result['latest_frame_state_overlapping_footprints'] = {k:len(v) for k,v in state_footprints.items()}
        result['latest_observed_invalid_plane_reasons_overlapping'] = dict(latest_invalid_bits)
        # obstacle黒だけの観測状態も独立集計し、terrainの黒と混同しない。
        obstacle_states = Counter()
        for r in selected:
            if 'obstacle' not in r['causes'].split('+'):
                continue
            if r['last_cell_observation_stamp_ns'] != r['latest_depth_stamp_ns']:
                state = 'unobserved_in_latest_frame'
            elif int(r['plane_reason_bits'] or 0):
                state = 'observed_but_plane_invalid'
            elif r['above_plane_hit'] == '0':
                state = 'observed_plane_valid_without_high_hit'
            else:
                state = 'observed_plane_valid_with_high_hit'
            obstacle_states[state] += 1
        result['latest_frame_state_obstacle_black_cells'] = dict(obstacle_states)
        last_states = Counter(); last_obstacle_states = Counter(); last_invalid_bits = Counter()
        held_terrain = 0; last_ages = []
        for r in selected:
            bits = int(r['plane_reason_bits'] or 0)
            state = ('plane_invalid' if bits else 'plane_valid_without_high_hit'
                     if r['above_plane_hit'] == '0' else 'plane_valid_with_high_hit')
            last_states[state] += 1
            if 'obstacle' in r['causes'].split('+'):
                last_obstacle_states[state] += 1
            for bit, name in [(1,'self_unknown'),(2,'support_low'),(4,'degenerate'),(8,'slope_gate'),(16,'roughness_gate')]:
                last_invalid_bits[name] += bool(bits & bit)
            if any(cue in r['causes'].split('+') for cue in ('slope','roughness','step')):
                held_terrain += int(r['terrain_stamp_ns']) < int(r['last_cell_observation_stamp_ns'])
            last_ages.append((int(r['passage_stamp_ns'])-int(r['last_cell_observation_stamp_ns']))/1e9)
        result['last_observation_state_all_black_cells'] = dict(last_states)
        result['last_observation_state_obstacle_black_cells'] = dict(last_obstacle_states)
        result['last_observation_invalid_plane_reasons_overlapping'] = dict(last_invalid_bits)
        result['terrain_black_cells_retained_after_newer_cell_observation'] = held_terrain
        result['last_cell_observation_age_seconds'] = distribution(last_ages)
        output['modes'][mode] = result
    black_sets = {mode: {r['stamp_ns'] for r in new_rows if r['mode']==mode and int(r['black_cells']) > 0}
                  for mode in modes}
    if not black_sets[confirmed_mode] <= black_sets['N1']:
        raise ValueError('確認済みの黒がN1の部分集合ではありません')
    removed = black_sets['N1']-black_sets[confirmed_mode]
    output['n1_only_footprints'] = len(removed)
    output['n1_only_cause_combinations'] = dict(Counter('+'.join(cue for cue in units
        if int(r[cue+'_black_cells']) > 0) for r in new_rows if r['mode']=='N1' and r['stamp_ns'] in removed))
    pending_path = folder/'pending_obstacle_cells.csv'
    if pending_path.exists():
        pending = list(csv.DictReader(pending_path.open()))
        conservative_obstacles = {(r['passage_stamp_ns'],r['cell_x'],r['cell_y']) for r in cells
            if r['mode']==conservative_mode and 'obstacle' in r['causes'].split('+')}
        confirmed_obstacles = {(r['passage_stamp_ns'],r['cell_x'],r['cell_y']) for r in cells
            if r['mode']==confirmed_mode and 'obstacle' in r['causes'].split('+')}
        if {(r['passage_stamp_ns'],r['cell_x'],r['cell_y']) for r in pending} != conservative_obstacles-confirmed_obstacles:
            raise ValueError('確認待ちと原因CSVの位置／通過stampが不一致です')
        for r in pending:
            if int(r['matched_window_observations']) != sum(int(r[k]) for k in
                    ('matched_window_positive','matched_window_valid_no_hit','matched_window_plane_unknown')):
                raise ValueError('候補窓の観測数内訳が一致しません')
        output['pending_obstacle_analysis'] = dict(
            cell_passage_instances=len(pending), footprints=len({r['passage_stamp_ns'] for r in pending}),
            unique_cells=len({(r['cell_x'],r['cell_y']) for r in pending}),
            histograms={name: dict(sorted(Counter(int(r[name]) for r in pending).items())) for name in
                ('matched_window_observations','matched_window_positive','matched_window_valid_no_hit',
                 'matched_window_plane_unknown','recent_cell_observed','recent_plane_valid','recent_cell_high_hit',
                 'positive_track_count')},
            joint_matched_window_counts=dict(Counter('%s obs / %s hit / %s no-hit / %s unknown' %
                (r['matched_window_observations'],r['matched_window_positive'],r['matched_window_valid_no_hit'],
                 r['matched_window_plane_unknown']) for r in pending)),
            recent_image_count_histogram=dict(Counter(r['recent_image_count'] for r in pending)),
            total_cell_observations=distribution([int(r['total_cell_observations']) for r in pending]),
            cell_observations_from_first_hit=distribution([int(r['cell_observations_from_first_hit']) for r in pending]),
            limitations=['Best-supported historical high track selected per pending cell, not proven physical identity',
                         'No positive track can mean overflow-only evidence; zero is not a free-space vote',
                         'Recent image window differs from last N matched candidate observations'])
    (folder/'cause_audit.json').write_text(json.dumps(output, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(output, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
