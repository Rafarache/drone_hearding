"""Loopback-only experiment API and built React application."""
import argparse
from contextlib import asynccontextmanager
import csv
import io
import json
import copy
import xml.etree.ElementTree as ET
from pathlib import Path
from urllib.parse import urlparse

from fastapi import FastAPI, HTTPException, Query, Request
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

from .configuration import defaults
from .metrics import summarize
from .models import BatchSpec, Scenario, Visualizations, DeleteRuns
from .storage import Store, database, trajectory, atomic_json, identifier, utcnow
from .supervisor import Supervisor, submit_batch


def csv_response(rows, filename):
    stream = io.StringIO()
    if rows:
        writer = csv.DictWriter(stream, fieldnames=list(dict.fromkeys(key for row in rows for key in row)))
        writer.writeheader()
        for row in rows:
            writer.writerow({k: json.dumps(v) if isinstance(v, (dict, list)) else v for k, v in row.items()})
    return Response(stream.getvalue(), media_type='text/csv',
                    headers={'Content-Disposition': f'attachment; filename="{filename}"'})


def create_app(data_dir='/ros2_ws/test_results', web_dir=None, manage_worker=True, default_factory=defaults):
    store = Store(data_dir)
    supervisor = Supervisor(store)

    @asynccontextmanager
    async def lifespan(app):
        if manage_worker:
            if not store.scenarios():
                store.save_scenario(default_factory().model_dump())
            supervisor.start()
        try:
            yield
        finally:
            if manage_worker:
                supervisor.stop()

    app = FastAPI(title='Herding Test Lab', lifespan=lifespan)
    app.state.store = store
    from starlette.middleware.trustedhost import TrustedHostMiddleware
    app.add_middleware(TrustedHostMiddleware, allowed_hosts=['127.0.0.1', 'localhost'] +
                       ([] if manage_worker else ['testserver']))

    @app.middleware('http')
    async def same_origin(request: Request, call_next):
        if request.method not in {'GET', 'HEAD', 'OPTIONS'}:
            origin = request.headers.get('origin')
            if origin and urlparse(origin).netloc != request.headers.get('host'):
                return JSONResponse({'detail': 'Cross-origin control requests are not allowed'}, status_code=403)
            if request.headers.get('sec-fetch-site') == 'cross-site':
                return JSONResponse({'detail': 'Cross-site control requests are not allowed'}, status_code=403)
        return await call_next(request)

    def get_run(run_id):
        run = store.run(run_id)
        if run is None:
            raise HTTPException(404, 'Run not found')
        return run

    @app.get('/api/health')
    def health():
        return {'ok': True, 'data_dir': str(store.root), 'active_run': supervisor.active,
                'max_duration_s': 1200, 'recording_hz': 10}

    @app.get('/api/defaults')
    def get_defaults():
        return default_factory().model_dump()

    @app.get('/api/scenarios')
    def scenarios():
        return store.scenarios()

    @app.post('/api/scenarios', status_code=201)
    def create_scenario(scenario: Scenario):
        return store.save_scenario(scenario.model_dump())

    @app.put('/api/scenarios/{scenario_id}')
    def edit_scenario(scenario_id: str, scenario: Scenario):
        if store.scenario(scenario_id) is None:
            raise HTTPException(404, 'Scenario not found')
        return store.save_scenario(scenario.model_dump(), scenario_id)

    @app.delete('/api/scenarios/{scenario_id}', status_code=204)
    def delete_scenario(scenario_id: str):
        store.delete_scenario(scenario_id)
        return Response(status_code=204)

    @app.get('/api/batches')
    def batches():
        return store.batches()

    @app.post('/api/batches', status_code=202)
    def enqueue(spec: BatchSpec):
        try:
            batch_id = submit_batch(store, spec)
        except (ValueError, OSError) as error:
            raise HTTPException(422, str(error)) from error
        return next(b for b in store.batches() if b['id'] == batch_id)

    @app.post('/api/batches/{batch_id}/cancel')
    def cancel_batch(batch_id: str):
        store.cancel(batch_id=batch_id)
        return {'status': 'cancellation_requested'}

    @app.get('/api/runs')
    def runs():
        return store.runs()

    @app.get('/api/runs/{run_id}')
    def run(run_id: str):
        return get_run(run_id)

    @app.post('/api/runs/{run_id}/cancel')
    def cancel(run_id: str):
        get_run(run_id)
        store.cancel(run_id=run_id)
        return {'status': 'cancellation_requested'}

    def delete_experiments(run_ids):
        try:
            return {'deleted': store.delete_runs(run_ids)}
        except KeyError as error:
            raise HTTPException(404, 'One or more experiments were not found') from error
        except ValueError as error:
            raise HTTPException(409, str(error)) from error

    @app.delete('/api/runs/{run_id}')
    def delete_experiment(run_id: str):
        get_run(run_id)
        return delete_experiments([run_id])

    @app.post('/api/runs/delete')
    def delete_selected(request: DeleteRuns):
        return delete_experiments(request.run_ids)

    @app.get('/api/runs/{run_id}/search')
    def search_history(run_id: str, end: float = Query(1200, ge=0, le=1200)):
        from .search import history
        get_run(run_id)
        return history(store.run_dir(run_id) / 'trajectory.sqlite3', end)

    @app.get('/api/runs/{run_id}/visualizations')
    def live_settings(run_id: str):
        run = get_run(run_id)
        directory = store.run_dir(run_id)
        manifest = json.loads((directory / 'manifest.json').read_text())
        result = {'available': manifest.get('live_visualization_controls', False),
                  'settings': Visualizations.model_validate(run['scenario'].get('visualization', {})).model_dump(),
                  'applied': False, 'errors': []}
        if (directory / 'live_state.json').exists():
            result.update(json.loads((directory / 'live_state.json').read_text()))
        if (directory / 'live_request.json').exists():
            request = json.loads((directory / 'live_request.json').read_text())
            if result.get('revision') != request['revision']:
                result.update(settings=request['settings'], applied=False)
        return result

    @app.post('/api/runs/{run_id}/visualizations', status_code=202)
    def change_live_settings(run_id: str, settings: Visualizations):
        run = get_run(run_id)
        if run['status'] not in {'starting', 'running'}:
            raise HTTPException(409, 'Live views can only change while a test is running')
        if not live_settings(run_id)['available']:
            raise HTTPException(409, 'Live controls are available for experiments started with this version')
        atomic_json(store.run_dir(run_id) / 'live_request.json', {
            'revision': identifier(), 'settings': settings.model_dump(), 'requested_at': utcnow()})
        return live_settings(run_id)

    @app.post('/api/runs/{run_id}/rerun', status_code=202)
    def rerun(run_id: str):
        previous = get_run(run_id)
        # Reuse original frozen templates, not today's installed defaults.
        from .configuration import snapshot, FILES
        sources = {name: store.run_dir(run_id) / 'config' / name for name in FILES}
        original_world = store.run_dir(run_id) / 'config' / 'world.original.world'
        if not original_world.exists():
            original_world = store.run_dir(run_id) / 'config' / 'world.world'
        scenario = copy.deepcopy(previous['scenario'])
        # Older recordings had one shared radius; recover the frozen quiet
        # radius too, rather than replacing it with today's scenario default.
        herding = ET.parse(sources['simulation.xml']).getroot().find('herding')
        goal = scenario['goal']
        goal.setdefault('completion_radius_m', float(herding.get('final_goal_reached_radius_m')))
        goal.setdefault('extended_radius_m', float(ET.parse(sources['grid_config.xml']).getroot()
                                                   .find('goal_quiet_radius_m').text))
        batch_id = submit_batch(store, {'name': f"Rerun · {previous['scenario']['name']}",
                                        'items': [{'scenario': scenario}]},
                                snapshotter=lambda directory, scenario: snapshot(
                                    directory, scenario, sources, lambda *args: original_world))
        return {'batch_id': batch_id}

    @app.get('/api/runs/{run_id}/trajectory')
    def frames(run_id: str, start: float = Query(0, ge=0, le=1200),
               end: float = Query(1200, ge=0, le=1200), stride: int = Query(1, ge=1, le=100)):
        get_run(run_id)
        if end < start:
            raise HTTPException(422, 'end must be greater than or equal to start')
        return trajectory(store.run_dir(run_id) / 'trajectory.sqlite3', start, end, stride)

    @app.get('/api/runs/{run_id}/summary')
    def summary(run_id: str):
        run = get_run(run_id)
        return run.get('summary') or summarize(store.run_dir(run_id) / 'trajectory.sqlite3',
                                               run['scenario'], [e['name'] for e in run['entities']])

    @app.get('/api/runs/{run_id}/configuration')
    def configuration(run_id: str):
        get_run(run_id)
        directory = store.run_dir(run_id)
        manifest = json.loads((directory / 'manifest.json').read_text())
        if (directory / 'runtime.json').exists():
            manifest['runtime'] = json.loads((directory / 'runtime.json').read_text())
        return {'manifest': manifest,
                'files': {p.name: p.read_text(errors='replace') for p in (directory / 'config').iterdir()}}

    @app.get('/api/runs/{run_id}/logs')
    def logs(run_id: str):
        get_run(run_id)
        directory = store.run_dir(run_id)
        return {name: (directory / name).read_text(errors='replace')[-100000:]
                if (directory / name).exists() else '' for name in ('launch.log', 'worker.log', 'gazebo.log', 'rviz.log')}

    @app.get('/api/runs/{run_id}/export/{kind}')
    def export(run_id: str, kind: str):
        run = get_run(run_id)
        directory = store.run_dir(run_id)
        if kind == 'metrics':
            return csv_response(summary(run_id)['entities'], f'{run_id}-metrics.csv')
        if kind == 'trajectory':
            path = directory / 'trajectory.sqlite3'
            if not path.exists():
                return csv_response([], f'{run_id}-trajectory.csv')
            def rows():
                with database(path) as db:
                    cursor = db.execute('SELECT * FROM positions ORDER BY t,entity')
                    output = io.StringIO()
                    writer = csv.writer(output)
                    writer.writerow([column[0] for column in cursor.description])
                    yield output.getvalue()
                    for row in cursor:
                        output.seek(0)
                        output.truncate(0)
                        writer.writerow(tuple(row))
                        yield output.getvalue()
            return StreamingResponse(rows(), media_type='text/csv', headers={
                'Content-Disposition': f'attachment; filename="{run_id}-trajectory.csv"'})
        if kind == 'configuration':
            return JSONResponse(configuration(run_id), headers={'Content-Disposition': f'attachment; filename="{run_id}-configuration.json"'})
        if kind in {'launch.log', 'worker.log', 'gazebo.log', 'rviz.log'}:
            path = directory / kind
            if path.exists():
                return FileResponse(path, filename=f'{run_id}-{kind}')
        raise HTTPException(404, 'Export not found')

    if web_dir and Path(web_dir).is_dir():
        directory = Path(web_dir).resolve()
        if (directory / 'assets').is_dir():
            app.mount('/assets', StaticFiles(directory=directory / 'assets'), name='assets')

        @app.get('/{path:path}')
        def frontend(path: str):
            if path.startswith('api/'):
                raise HTTPException(404, 'API endpoint not found')
            return FileResponse(directory / 'index.html')
    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-dir', default='/ros2_ws/test_results')
    parser.add_argument('--web-dir', default='/ros2_ws/src/simulation_test_pkg/web/dist')
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()
    import uvicorn
    uvicorn.run(create_app(args.data_dir, args.web_dir), host='127.0.0.1', port=args.port, workers=1)


if __name__ == '__main__':
    main()
