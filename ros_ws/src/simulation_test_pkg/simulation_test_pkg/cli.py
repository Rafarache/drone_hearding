"""Run a JSON battery from the terminal using the dashboard's same queue."""
import argparse
import json
import time
from .models import BatchSpec
from .storage import Store, TERMINAL
from .supervisor import Supervisor, submit_batch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spec', required=True, help='Version 1 batch JSON file')
    parser.add_argument('--data-dir', default='/ros2_ws/test_results')
    arguments = parser.parse_args()
    store = Store(arguments.data_dir)
    supervisor = Supervisor(store)
    supervisor.start()
    exit_code = 0
    try:
        batch_id = submit_batch(store, BatchSpec.model_validate_json(open(arguments.spec).read()))
        print(f'Batch {batch_id} queued. Results: {store.root}', flush=True)
        previous = None
        while True:
            runs = [r for r in store.runs() if r['batch_id'] == batch_id]
            states = [(r['id'][:8], r['status'], round(r.get('simulation_duration_s', 0), 1)) for r in runs]
            if states != previous:
                print(json.dumps(states), flush=True)
                previous = states
            if all(r['status'] in TERMINAL for r in runs):
                exit_code = int(any(r['status'] in {'failed', 'interrupted'} for r in runs))
                break
            time.sleep(1)
    except KeyboardInterrupt:
        exit_code = 130
        print('Stopping the active run; queued runs are retained.', flush=True)
    finally:
        supervisor.stop()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
