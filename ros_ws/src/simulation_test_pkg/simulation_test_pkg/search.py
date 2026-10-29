"""Timestamped, authoritative per-drone search memory, stored as first-seen cells."""
import json
import math
from pathlib import Path
from .storage import database


def _point(value):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError('Expected a planar point')
    result = [float(v) for v in value]
    if not all(math.isfinite(v) for v in result):
        raise ValueError('Search coordinates must be finite')
    return result


def normalize_state(value):
    value = json.loads(value) if isinstance(value, str) else value
    if not isinstance(value, dict):
        raise ValueError('Expected a search-state object')
    if value.get('version') != 1 or value.get('frame_id') != 'map':
        raise ValueError('Unsupported search state version or frame')
    stamp = float(value['stamp_s'])
    grid = value['grid']
    width, height = grid['width'], grid['height']
    if type(width) is not int or type(height) is not int or not (0 < width * height <= 2000000) or min(width, height) < 1:
        raise ValueError('Invalid coverage dimensions')
    resolution = float(grid['resolution'])
    if not math.isfinite(stamp) or not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('Invalid search timestamp or resolution')
    cells = grid['seen_cells']
    if not isinstance(cells, list) or any(type(i) is not int or i < 0 or i >= width * height for i in cells):
        raise ValueError('Invalid searched-cell index')
    goal = value.get('goal')
    if goal is not None:
        if len(goal['patch']) != 4:
            raise ValueError('Expected four search patch corners')
        goal = {'viewpoint': _point(goal['viewpoint']), 'patch_center': _point(goal['patch_center']),
                'patch': [_point(p) for p in goal['patch']], 'strength': float(goal['strength'])}
        if not math.isfinite(goal['strength']) or goal['strength'] < 0:
            raise ValueError('Invalid search strength')
    return {'version': 1, 'frame_id': 'map', 'stamp_s': stamp,
            'grid': {'width': width, 'height': height, 'resolution': resolution,
                     'origin': _point(grid['origin']), 'seen_cells': sorted(set(cells))}, 'goal': goal}


class SearchHistory:
    def __init__(self, db):
        self.db = db
        db.executescript('''CREATE TABLE IF NOT EXISTS search_grids (
            drone TEXT PRIMARY KEY, first_t REAL, geometry TEXT);
            CREATE TABLE IF NOT EXISTS search_cells (
                drone TEXT, cell INTEGER, seen_at REAL, PRIMARY KEY(drone,cell));
            CREATE INDEX IF NOT EXISTS search_cells_time ON search_cells(drone,seen_at);
            CREATE TABLE IF NOT EXISTS search_goals (
                drone TEXT, t REAL, goal TEXT, PRIMARY KEY(drone,t));''')
        self.previous = {}

    def record(self, drone, t, state):
        geometry = {k: v for k, v in state['grid'].items() if k != 'seen_cells'}
        cells = set(state['grid']['seen_cells'])
        goal_json = json.dumps(state['goal'], separators=(',', ':'), allow_nan=False)
        previous = self.previous.get(drone)
        if previous:
            old_t, old_geometry, old_cells, old_goal = previous
            if t < old_t:
                return
            if geometry != old_geometry or not old_cells <= cells:
                raise ValueError('Coverage geometry changed or permanent search memory shrank')
        else:
            old_cells, old_goal = set(), ''
            self.db.execute('INSERT INTO search_grids VALUES (?,?,?)', (drone, t, json.dumps(geometry)))
        self.db.executemany('INSERT OR IGNORE INTO search_cells VALUES (?,?,?)',
                            [(drone, cell, t) for cell in sorted(cells - old_cells)])
        if goal_json != old_goal:
            self.db.execute('INSERT OR REPLACE INTO search_goals VALUES (?,?,?)', (drone, t, goal_json))
        self.previous[drone] = (t, geometry, cells, goal_json)


def history(path, end=1200):
    if not Path(path).exists():
        return {'available': False, 'drones': []}
    with database(path) as db:
        tables = {r['name'] for r in db.execute("SELECT name FROM sqlite_master WHERE type='table'")}
        if 'search_grids' not in tables:
            return {'available': False, 'drones': []}
        drones = []
        for row in db.execute('SELECT * FROM search_grids WHERE first_t<=? ORDER BY drone', (end,)):
            drone = dict(drone=row['drone'], first_t=row['first_t'], grid=json.loads(row['geometry']))
            drone['cells'] = [[r['cell'], r['seen_at']] for r in db.execute(
                'SELECT cell,seen_at FROM search_cells WHERE drone=? AND seen_at<=? ORDER BY seen_at,cell',
                (row['drone'], end))]
            drone['goals'] = [{'t': r['t'], 'goal': json.loads(r['goal'])} for r in db.execute(
                'SELECT t,goal FROM search_goals WHERE drone=? AND t<=? ORDER BY t', (row['drone'], end))]
            drones.append(drone)
    return {'available': True, 'drones': drones}
