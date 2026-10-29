import sys
from pathlib import Path
import pytest
sys.path.insert(0, str(Path(__file__).parents[1]))
from simulation_test_pkg.models import Scenario


@pytest.fixture
def scenario():
    return Scenario(name='Fixture', drones=[{'x': -5, 'y': 0}], cows=[{'x': 5, 'y': 0}], duration_s=1200)


@pytest.fixture
def manifest(scenario):
    return {'scenario': scenario.model_dump(), 'cow_positions_topic': '/fixture_cows',
            'entities': [{'name': '/fixture_drone0', 'kind': 'drone', 'index': 0},
                         {'name': 'cow0', 'kind': 'cow', 'index': 0}]}
