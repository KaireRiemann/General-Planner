#!/usr/bin/env python3
"""Offline analysis of a GP_RUN_DIR session. Python standard library only."""
import argparse
import collections
import bisect
import csv
import html
import hashlib
import datetime
import json
import math
from pathlib import Path

RETURN_NAMES = ['FAILED', 'NO_NEED', 'SUCCESS', 'FINISH', 'NEW_TRAJ', 'EMER',
                'OPT_FAILED', 'INIT_ERROR', 'REACH_HORIZON', 'REACH_GOAL',
                'NO_PATH', 'TIME_OUT', 'WAIT_FOR_TOPOLOGY']
STAGES = ['executor_wall_ms', 'exp_frontend_ms', 'exp_opt_ms', 'backup_frontend_ms',
          'backup_opt_ms', 'exp_generation_ms', 'backup_generation_ms', 'backend_total_ms']


def number(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (ValueError, TypeError):
        return None


def stats(values):
    values = sorted(v for v in values if v is not None)
    if not values:
        return {'n': 0, 'mean': None, 'p50': None, 'p95': None, 'p99': None, 'max': None}
    return dict(n=len(values), mean=sum(values)/len(values), max=values[-1],
                **{f'p{q}': values[max(0, math.ceil(len(values)*q/100)-1)] for q in (50, 95, 99)})


def read_rows(path, warnings):
    with path.open(newline='', errors='replace') as stream:
        reader = csv.DictReader(stream, skipinitialspace=True)
        for line, row in enumerate(reader, 2):
            if None in row or any(v is None for v in row.values()):
                warnings.append(f'{path.name}:{line}: incomplete/malformed row ignored')
                continue
            yield row


def analyze_bag(root, warnings):
    bag_path = root/'topics.bag'
    if not bag_path.exists():
        return {'available':False, 'reason':'no finalized optional ROS1 bag'}
    try:
        import rosbag
    except ImportError:
        warnings.append('ROS1 rosbag Python module unavailable; bus/odometry analysis skipped, bag retained')
        return {'available':False, 'reason':'rosbag module unavailable'}
    command, odometry = collections.defaultdict(list), collections.defaultdict(list)
    try:
        with rosbag.Bag(str(bag_path)) as bag:
            for topic,msg,record_time in bag.read_messages():
                kind=getattr(msg,'_type','')
                if kind not in ('quadrotor_msgs/PositionCommand','nav_msgs/Odometry'):continue
                stamp=msg.header.stamp.to_sec()
                p=msg.position if kind=='quadrotor_msgs/PositionCommand' else msg.pose.pose.position
                row={'stamp':stamp,'record_stamp':record_time.to_sec(),'frame_id':msg.header.frame_id,'trajectory_id':getattr(msg,'trajectory_id',''),'trajectory_flag':getattr(msg,'trajectory_flag',''),'x':p.x,'y':p.y,'z':p.z}
                (command if kind=='quadrotor_msgs/PositionCommand' else odometry)[topic].append(row)
    except (OSError, rosbag.ROSBagException) as error:
        warnings.append('Bag read failed: '+str(error));return {'available':False,'reason':str(error)}
    for name,streams in [('bus_commands',command),('bus_odometry',odometry)]:
        with (root/(name+'.csv')).open('w',newline='') as out:
            writer=csv.DictWriter(out,fieldnames=['topic','stamp','record_stamp','frame_id','trajectory_id','trajectory_flag','x','y','z']);writer.writeheader()
            for topic,rows in streams.items():
                for row in rows:writer.writerow(dict(topic=topic,**row))
    def describe(streams):
        return {topic:{'samples':len(rows),'positive_stamp_interval_ms':stats(
            (b['stamp']-a['stamp'])*1000 for a,b in zip(rows,rows[1:]) if b['stamp']>a['stamp'])}
            for topic,rows in streams.items()}
    # Only auto-match an unambiguous single command/odom pair. This compares
    # sampled references with odometry, not a controller latency estimate.
    matched=[]
    if len(command)==1 and len(odometry)==1:
        refs=sorted(next(iter(command.values())),key=lambda r:r['stamp']);stamps=[r['stamp'] for r in refs]
        for row in next(iter(odometry.values())):
            i=bisect.bisect_right(stamps,row['stamp'])-1
            if i>=0 and row['frame_id'] and row['frame_id']==refs[i]['frame_id'] and 0<=row['stamp']-stamps[i]<=.05:
                matched.append(math.sqrt(sum((row[k]-refs[i][k])**2 for k in ('x','y','z'))))
    return {'available':True,'commands':describe(command),'odometry':describe(odometry),
            'reference_to_odometry_distance_m':stats(matched),
            'matching':'one command and one odometry topic only; latest reference at/before odometry stamp, max age 50ms and identical nonempty frame_id; no interpolation or frame transform'}


def analyze(root):
    root = Path(root)
    warnings, cycles, events, commands, command_rows = [], [], [], [], []
    for process in sorted(root.glob('planner_*')):
        for file in process.glob('planning_cycles.csv'):
            for row in read_rows(file, warnings):
                if row.get('schema_version') != '1':
                    warnings.append(f'{file}: unsupported schema {row.get("schema_version")}')
                    continue
                if number(row.get('executor_wall_ms')) is None:
                    warnings.append(f'{file}: invalid executor duration')
                    continue
                row['process'] = process.name
                cycles.append(row)
        # Runtime event streams are authoritative; saved copies duplicate them.
        for file in process.glob('*diagnostic_events/*runtime.csv'):
            for row in read_rows(file, warnings):
                row['process'] = process.name
                events.append(row)
        for file in process.glob('*cmd_logs/*latest_log.csv'):
            stream_rows = list(read_rows(file, warnings))
            command_rows.extend(dict(r,process=process.name) for r in stream_rows)
            stamps = [number(r.get('time')) for r in stream_rows]
            gaps = [(b-a)*1000 for a,b in zip(stamps, stamps[1:]) if a is not None and b is not None and b>a]
            commands.append({'file': str(file.relative_to(root)), 'samples': len(stream_rows),
                             'positive_interval_ms': stats(gaps),
                             'non_increasing_stamps': sum(b<=a for a,b in zip(stamps,stamps[1:]) if a is not None and b is not None),
                             'max_command_speed_mps': max((math.sqrt(sum((number(r.get(k)) or 0)**2 for k in ['vel_x','vel_y','vel_z'])) for r in stream_rows), default=None)})
    keys = [(r['process'], r.get('replan_id')) for r in cycles]
    if len(set(keys)) != len(keys): warnings.append('Duplicate process/replan_id keys in planning_cycles')
    if not cycles: warnings.append('No completed executor calls found; do not interpret as a successful run')
    if not events: warnings.append('No diagnostic events; commit/mission interpretation unavailable')
    if not commands: warnings.append('No final command CSV; shutdown may have been incomplete or no commands published')
    bus = analyze_bag(root, warnings)
    manifest_path = root/'manifest.json'
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    if manifest.get('state') not in ('finished', 'interrupted', 'duration_limit', 'failed'):
        warnings.append('Session has no finalization metadata; live or incomplete run')
    if manifest.get('state')=='failed':warnings.append('Launched command failed; inspect console.log and manifest returncode')
    by_group = collections.defaultdict(list)
    for r in cycles: by_group[(r.get('task','unknown'), r.get('invocation','unknown'))].append(r)
    groups = []
    for (task, invocation), rows in sorted(by_group.items()):
        measured = [r for r in rows if number(r.get('period_budget_ms')) and number(r['period_budget_ms'])>0]
        reports = [r for r in rows if r.get('optimizer_report_valid')=='1']
        groups.append({'task':task, 'invocation':invocation, 'calls':len(rows),
          'return_codes': dict(collections.Counter(RETURN_NAMES[int(r['ret_code'])] if r.get('ret_code','').isdigit() and int(r['ret_code'])<len(RETURN_NAMES) else r.get('ret_code','unknown') for r in rows)),
          'over_period_calls':sum(float(r['executor_wall_ms'])>float(r['period_budget_ms']) for r in measured),
          'period_comparable_calls':len(measured),
          'executor_ms_by_return_code':{code:stats(number(r['executor_wall_ms']) for r in rows if r.get('ret_code')==code) for code in sorted(set(r.get('ret_code','unknown') for r in rows))},
          'stages_ms': {stage:stats(number(r.get(stage)) for r in rows) for stage in STAGES},
          'optimizer_reports':len(reports), 'fast_stops':sum(r.get('fast_stop')=='1' for r in reports),
          'fallbacks':sum(r.get('fallback_used')=='1' for r in reports),
          'iterations':stats(number(r.get('iterations')) for r in reports),
          'line_search_evaluations':stats(number(r.get('line_search_evaluations')) for r in reports),
          'continuous_feasible_reports':sum(r.get('continuous_feasible')=='1' for r in reports),
          'max_normalized_violation':stats(number(r.get('max_normalized_violation')) for r in reports)})
    publications={(e['process'],e.get('traj_seq')):e for e in events if e.get('event')=='trajectory_published'}
    with (root/'command_trace.csv').open('w',newline='') as stream:
        fields=['process','time','traj_seq','active_replan_id','publication_replan_id','publication_stamp']
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for r in command_rows:
            event=publications.get((r['process'],r.get('traj_seq')), {})
            writer.writerow(dict(process=r['process'],time=r.get('time',''),traj_seq=r.get('traj_seq',''),
                active_replan_id=r.get('replan_id',''),publication_replan_id=event.get('replan_id',''),publication_stamp=event.get('stamp','')))
    slowest=sorted(cycles,key=lambda r:float(r['executor_wall_ms']),reverse=True)[:10]
    event_counts = collections.Counter(e.get('event','unknown') for e in events)
    starts={(e['process'],e.get('replan_id')) for e in events if e.get('event') in ('plan_from_rest_start','replan_start')}
    unmatched=sorted(starts-set(keys))
    if unmatched: warnings.append(f'{len(unmatched)} start events without completed executor row (pre-executor exit, interruption or stalled call; inspect events)')
    summary={'schema_version':1, 'analysis_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
             'analysis_tool_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), 'manifest_state':manifest.get('state','unknown'), 'calls':len(cycles),
             'groups':groups,'slowest_cycles':slowest,'bus_observation':bus,'event_counts':dict(event_counts),'commands':commands,'publication_events':len(publications),'unmatched_start_keys':unmatched,
             'warnings':warnings,'limitations':[
              'Executor elapsed time is steady-clock wall time; simulator timestamps are for correlation only.',
              'Stage timings are nested: generation contains frontend/optimization; do not add nested columns.',
              'Period overruns measure executor calls only, not full callback scheduling or end-to-end latency.',
              'SUCCESS/NEW_TRAJ are planner return codes, not proof of commit, goal completion or collision freedom.',
              'Missing optimizer reports are unavailable, not zero violations. Continuous feasibility covers the existing solver oracle only.',
              'Internal command CSV covers normal sampling; publishWhilePlannerBusy bypasses it, so gaps are not complete publication intervals. Use the optional bus bag.',
              'Bag frame IDs must agree for reference/odometry comparison; this metric is not a time delay estimate.',
              'Command active_replan_id is the current planning call, not necessarily its producer; command_trace joins the trajectory publication via process/traj_seq.',
              'No complete deterministic replay guarantee: maps, all inputs and solver parameter gauges require additional capture.']}
    (root/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False))
    with (root/'cycles_joined.csv').open('w',newline='') as stream:
        if cycles:
            fields=list(cycles[0])+['related_events']
            writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
            joined=collections.defaultdict(list)
            for e in events: joined[(e['process'],e.get('replan_id'))].append(e.get('event',''))
            for r in cycles:writer.writerow(dict(r,related_events=';'.join(joined[(r['process'],r.get('replan_id'))])))
    def esc(value):return html.escape(str(value))
    body=['<!doctype html><html lang="zh"><meta charset="utf-8"><title>规划运行分析</title>',
      '<style>body{font:15px system-ui;max-width:1200px;margin:40px auto;padding:0 20px;color:#203040}table{border-collapse:collapse;width:100%;margin:18px 0}th,td{padding:9px;border-bottom:1px solid #ddd;text-align:left}pre{white-space:pre-wrap}.warn{color:#a34b00}</style>',
      '<h1>规划运行分析</h1>',f'<p>目录：{esc(root.name)} · 状态：{esc(summary["manifest_state"])} · 返回的规划调用：{len(cycles)}</p>',
      '<p>保留失败与跳过调用。规划返回成功不等于任务完成；本报告不推断碰撞安全。</p>']
    for warning in warnings:body.append(f'<p class="warn">{esc(warning)}</p>')
    for group in groups:
        body.append(f'<h2>{esc(group["task"])} / {esc(group["invocation"])}</h2><p>{group["calls"]} 次调用；超过配置重规划周期：{group["over_period_calls"]}/{group["period_comparable_calls"]}；返回码：{esc(group["return_codes"])}</p>')
        body.append('<table><tr><th>阶段 ms（包含嵌套）</th><th>均值</th><th>P50</th><th>P95</th><th>P99</th><th>最大值</th></tr>')
        for stage,vals in group['stages_ms'].items():body.append('<tr><td>'+esc(stage)+'</td>'+''.join('<td>'+('—' if vals[k] is None else f'{vals[k]:.3f}')+'</td>' for k in ['mean','p50','p95','p99','max'])+'</tr>')
        body.append('</table>'+f'<p>有效优化器报告 {group["optimizer_reports"]}；快停 {group["fast_stops"]}；回退 {group["fallbacks"]}；连续可行报告 {group["continuous_feasible_reports"]}。</p>')
    body.append('<h2>最慢调用</h2><pre>'+esc(json.dumps(slowest,ensure_ascii=False,indent=2))+'</pre>')
    body.append('<h2>事件与指令</h2><pre>'+esc(json.dumps({'events':dict(event_counts),'internal_command_samples':commands,'bus_observation':bus},ensure_ascii=False,indent=2))+'</pre>')
    body.append('<h2>解释边界</h2><ul>'+''.join('<li>'+esc(v)+'</li>' for v in summary['limitations'])+'</ul></html>')
    (root/'report.html').write_text('\n'.join(body))
    return summary

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run',type=Path)
    args=parser.parse_args();result=analyze(args.run)
    print(json.dumps({'calls':result['calls'],'warnings':result['warnings'],'report':str(args.run/'report.html')},ensure_ascii=False,indent=2))
