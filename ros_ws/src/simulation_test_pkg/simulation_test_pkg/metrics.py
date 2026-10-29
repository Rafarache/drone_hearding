"""Metrics derived exclusively from recorded, timestamped trajectories."""
import math
from pathlib import Path
from .storage import database


def entity_metrics(rows, goal, radius):
    if not rows:
        return None
    xy = xyz = valid_time = 0.0
    gaps = []
    for a, b in zip(rows, rows[1:]):
        dt = b['t'] - a['t']
        if dt <= 0:
            continue
        if dt > 1.000001:
            gaps.append([a['t'], b['t']])
            continue
        distance = math.hypot(b['x'] - a['x'], b['y'] - a['y'])
        xy += distance
        xyz += math.hypot(distance, b['z'] - a['z'])
        valid_time += dt
    duration = rows[-1]['t'] - rows[0]['t']
    result = dict(name=rows[0]['entity'], kind=rows[0]['kind'], samples=len(rows),
                  first_sample_s=rows[0]['t'], last_sample_s=rows[-1]['t'], observed_duration_s=duration,
                  valid_duration_s=valid_time, missing_intervals=gaps, incomplete=bool(gaps),
                  distance_xy_m=xy, distance_3d_m=xyz,
                  average_speed_xy_mps=xy / valid_time if valid_time else 0,
                  average_speed_3d_mps=xyz / valid_time if valid_time else 0,
                  peak_speed_xy_mps=max(math.hypot(r['vx'], r['vy']) for r in rows),
                  peak_speed_3d_mps=max(math.sqrt(r['vx']**2 + r['vy']**2 + r['vz']**2) for r in rows))
    if rows[0]['kind'] == 'cow':
        distance = lambda r: math.hypot(r['x'] - goal[0], r['y'] - goal[1])
        result.update(first_goal_arrival_s=next((r['t'] for r in rows if distance(r) <= radius), None),
                      final_goal_distance_m=distance(rows[-1]), final_inside_goal=distance(rows[-1]) <= radius)
    return result


def summarize(path, scenario, expected):
    if not Path(path).exists():
        return {'entities': [], 'missing_entities': expected, 'cows_at_goal': 0}
    with database(path) as db:
        entities = []
        for entity in expected:
            rows = [dict(r) for r in db.execute('SELECT * FROM positions WHERE entity=? ORDER BY t', (entity,))]
            result = entity_metrics(rows, (scenario['goal']['x'], scenario['goal']['y']), scenario['goal']['radius_m'])
            if result:
                entities.append(result)
    end = max((r['last_sample_s'] for r in entities), default=0)
    for result in entities:
        result['incomplete'] = result['incomplete'] or result['first_sample_s'] > .15 or end - result['last_sample_s'] > .15
    return {'entities': entities, 'missing_entities': [e for e in expected if e not in {r['name'] for r in entities}],
            'cows_at_goal': sum(bool(r.get('final_inside_goal')) for r in entities),
            'distance_cows_m': sum(r['distance_xy_m'] for r in entities if r['kind'] == 'cow'),
            'distance_drones_m': sum(r['distance_xy_m'] for r in entities if r['kind'] == 'drone')}
