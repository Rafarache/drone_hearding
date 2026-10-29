"""Local catalog and append-only, simulation-time trajectory storage."""
from contextlib import contextmanager
from datetime import datetime, timezone
import json
from pathlib import Path
import sqlite3
import shutil
import uuid

TERMINAL = {'success', 'timed_out', 'failed', 'cancelled', 'interrupted'}


def utcnow():
    return datetime.now(timezone.utc).isoformat()


def identifier():
    return uuid.uuid4().hex


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False))
    temporary.replace(path)


@contextmanager
def database(path):
    connection = sqlite3.connect(str(path), timeout=15)
    connection.row_factory = sqlite3.Row
    connection.execute('PRAGMA busy_timeout=15000')
    try:
        yield connection
        connection.commit()
    except BaseException:
        connection.rollback()
        raise
    finally:
        connection.close()


class Store:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.path = self.root / 'catalog.sqlite3'
        with database(self.path) as db:
            db.executescript('''PRAGMA journal_mode=WAL;
                CREATE TABLE IF NOT EXISTS scenarios (
                    id TEXT PRIMARY KEY, body TEXT NOT NULL, created TEXT, updated TEXT);
                CREATE TABLE IF NOT EXISTS batches (
                    id TEXT PRIMARY KEY, name TEXT, created TEXT);
                CREATE TABLE IF NOT EXISTS runs (
                    id TEXT PRIMARY KEY, batch_id TEXT, scenario_id TEXT,
                    body TEXT NOT NULL, status TEXT NOT NULL, created TEXT,
                    sequence INTEGER, cancel_requested INTEGER DEFAULT 0);
            ''')
        self._recover_deletions()

    def _recover_deletions(self):
        trash = self.root / '.deleting'
        if not trash.exists():
            return
        with database(self.path) as db:
            db.execute('BEGIN IMMEDIATE')
            for directory in trash.iterdir():
                run_id = directory.name.split('-', 1)[0]
                if len(run_id) != 32 or not directory.is_dir():
                    continue
                exists = db.execute('SELECT 1 FROM runs WHERE id=?', (run_id,)).fetchone()
                original = self.run_dir(run_id)
                if exists and not original.exists():
                    directory.replace(original)
                else:
                    shutil.rmtree(directory)

    def delete_runs(self, run_ids):
        run_ids = list(dict.fromkeys(run_ids))
        moved = []
        trash = self.root / '.deleting'
        trash.mkdir(exist_ok=True)
        try:
            with database(self.path) as db:
                db.execute('BEGIN IMMEDIATE')
                rows = []
                for run_id in run_ids:
                    self.run_dir(run_id)
                    row = db.execute('SELECT id,status,batch_id FROM runs WHERE id=?', (run_id,)).fetchone()
                    if row is None:
                        raise KeyError(run_id)
                    if row['status'] not in TERMINAL | {'queued'}:
                        raise ValueError('Cancel or finish running experiments before deleting them')
                    rows.append(row)
                for row in rows:
                    original = self.run_dir(row['id'])
                    if original.exists():
                        destination = trash / (row['id'] + '-' + identifier())
                        original.replace(destination)
                        moved.append((original, destination))
                    db.execute('DELETE FROM runs WHERE id=?', (row['id'],))
                db.execute('DELETE FROM batches WHERE NOT EXISTS (SELECT 1 FROM runs WHERE runs.batch_id=batches.id)')
        except BaseException:
            for original, destination in reversed(moved):
                if destination.exists():
                    destination.replace(original)
            raise
        for _, destination in moved:
            shutil.rmtree(destination)
        return run_ids

    def run_dir(self, run_id):
        if len(run_id) != 32 or any(c not in '0123456789abcdef' for c in run_id):
            raise KeyError(run_id)
        return self.root / 'runs' / run_id

    def scenarios(self):
        with database(self.path) as db:
            return [dict(json.loads(r['body']), id=r['id'], created_at=r['created'], updated_at=r['updated'])
                    for r in db.execute('SELECT * FROM scenarios ORDER BY updated DESC')]

    def scenario(self, scenario_id):
        return next((s for s in self.scenarios() if s['id'] == scenario_id), None)

    def save_scenario(self, body, scenario_id=None):
        now = utcnow()
        scenario_id = scenario_id or identifier()
        with database(self.path) as db:
            db.execute('''INSERT INTO scenarios VALUES (?, ?, ?, ?)
                          ON CONFLICT(id) DO UPDATE SET body=excluded.body, updated=excluded.updated''',
                       (scenario_id, json.dumps(body), now, now))
        return self.scenario(scenario_id)

    def delete_scenario(self, scenario_id):
        with database(self.path) as db:
            db.execute('DELETE FROM scenarios WHERE id=?', (scenario_id,))

    def enqueue(self, name, prepared):
        batch_id, now = identifier(), utcnow()
        with database(self.path) as db:
            db.execute('INSERT INTO batches VALUES (?, ?, ?)', (batch_id, name, now))
            for sequence, (run_id, scenario_id, body) in enumerate(prepared):
                body.update(id=run_id, batch_id=batch_id, batch_name=name, created_at=now,
                            status='queued', sequence=sequence)
                db.execute('INSERT INTO runs (id,batch_id,scenario_id,body,status,created,sequence) VALUES (?,?,?,?,?,?,?)',
                           (run_id, batch_id, scenario_id, json.dumps(body), 'queued', now, sequence))
        return batch_id

    def claim(self):
        with database(self.path) as db:
            db.execute('BEGIN IMMEDIATE')
            row = db.execute("SELECT * FROM runs WHERE status='queued' ORDER BY created,sequence LIMIT 1").fetchone()
            if row is None:
                return None
            db.execute("UPDATE runs SET status='starting' WHERE id=?", (row['id'],))
            result = json.loads(row['body'])
            result['status'] = 'starting'
            return result

    def runs(self):
        with database(self.path) as db:
            rows = list(db.execute('SELECT * FROM runs ORDER BY created DESC,sequence'))
        return [self._run(row) for row in rows]

    def run(self, run_id):
        with database(self.path) as db:
            row = db.execute('SELECT * FROM runs WHERE id=?', (run_id,)).fetchone()
        return self._run(row) if row else None

    def _run(self, row):
        result = json.loads(row['body'])
        result.update(status=row['status'], cancel_requested=bool(row['cancel_requested']))
        if row['status'] in {'starting', 'running'}:
            try:
                result.update(json.loads((self.run_dir(row['id']) / 'state.json').read_text()))
            except (OSError, ValueError):
                pass
        return result

    def update(self, run_id, changes):
        with database(self.path) as db:
            row = db.execute('SELECT body,status FROM runs WHERE id=?', (run_id,)).fetchone()
            if row is None:
                return
            body = json.loads(row['body'])
            body.update(changes)
            db.execute('UPDATE runs SET body=?,status=? WHERE id=?',
                       (json.dumps(body), changes.get('status', row['status']), run_id))

    def cancel(self, run_id=None, batch_id=None):
        with database(self.path) as db:
            clause, value = ('id=?', run_id) if run_id else ('batch_id=?', batch_id)
            db.execute(f"UPDATE runs SET cancel_requested=1 WHERE {clause} AND status NOT IN ('success','timed_out','failed','cancelled','interrupted')", (value,))
            db.execute(f"UPDATE runs SET status='cancelled' WHERE {clause} AND status='queued'", (value,))

    def batches(self):
        with database(self.path) as db:
            batches = [dict(r) for r in db.execute('SELECT * FROM batches ORDER BY created DESC')]
        runs = self.runs()
        for batch in batches:
            batch['runs'] = [r for r in runs if r['batch_id'] == batch['id']]
            states = [r['status'] for r in batch['runs']]
            batch['status'] = ('running' if any(s in {'starting', 'running'} for s in states)
                               else 'queued' if 'queued' in states else 'finished')
        return batches


class TrajectoryWriter:
    def __init__(self, path):
        self.db = sqlite3.connect(str(path))
        self.db.executescript('''PRAGMA journal_mode=WAL;
            CREATE TABLE IF NOT EXISTS positions (
                t REAL, entity TEXT, kind TEXT, x REAL,y REAL,z REAL,
                qx REAL,qy REAL,qz REAL,qw REAL,vx REAL,vy REAL,vz REAL,
                PRIMARY KEY(entity,t));
            CREATE INDEX IF NOT EXISTS positions_time ON positions(t);
            CREATE TABLE IF NOT EXISTS detections (t REAL PRIMARY KEY, points TEXT);
            CREATE TABLE IF NOT EXISTS events (t REAL, type TEXT, detail TEXT);
        ''')
        from .search import SearchHistory
        self.search = SearchHistory(self.db)
        self.db.commit()

    def frame(self, t, entities):
        self.db.executemany('INSERT OR REPLACE INTO positions VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)',
                            [(t, e['name'], e['kind'], *e['pose'], *e['velocity']) for e in entities])

    def detections(self, t, points):
        self.db.execute('INSERT OR REPLACE INTO detections VALUES (?,?)', (t, json.dumps(points)))

    def event(self, t, kind, detail):
        self.db.execute('INSERT INTO events VALUES (?,?,?)', (t, kind, detail))

    def flush(self):
        self.db.commit()

    def close(self):
        self.db.commit()
        self.db.execute('PRAGMA wal_checkpoint(TRUNCATE)')
        self.db.close()


def trajectory(path, start=0, end=1200, stride=1):
    if not Path(path).exists():
        return {'frames': [], 'detections': []}
    with database(path) as db:
        rows = db.execute('SELECT * FROM positions WHERE t>=? AND t<=? ORDER BY t,entity', (start, end))
        frames = {}
        for r in rows:
            if stride > 1 and round(r['t'] * 10) % stride:
                continue
            frames.setdefault(r['t'], []).append(dict(r))
        previous = db.execute('SELECT * FROM detections WHERE t<? ORDER BY t DESC LIMIT 1', (start,)).fetchall()
        detections = [dict(r) for r in previous]
        detections += [dict(r) for r in db.execute('SELECT * FROM detections WHERE t>=? AND t<=? ORDER BY t', (start, end))]
        events = [dict(r) for r in db.execute('SELECT * FROM events WHERE t>=? AND t<=? ORDER BY t', (start, end))]
    return {'frames': [{'t': t, 'entities': e} for t, e in frames.items()],
            'detections': [{'t': d['t'], 'points': json.loads(d['points'])} for d in detections], 'events': events}
