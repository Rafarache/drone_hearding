"""Shared website and CLI experiment contracts."""
from typing import Annotated
from pydantic import BaseModel, ConfigDict, Field, model_validator

Number = Annotated[float, Field(allow_inf_nan=False)]
Positive = Annotated[float, Field(gt=0, allow_inf_nan=False)]


class Model(BaseModel):
    model_config = ConfigDict(extra='forbid')


class Point(Model):
    x: Number
    y: Number


class CowStart(Point):
    z: Number = 0.01


class MapSettings(Model):
    center_x: Number = 0
    center_y: Number = 0
    width_m: Positive = 40
    height_m: Positive = 40


class Goal(Point):
    radius_m: Positive = 3
    completion_radius_m: Positive = 5
    extended_radius_m: Positive = 5

    @model_validator(mode='before')
    @classmethod
    def legacy_radii(cls, value):
        # Older scenarios used radius_m for both navigation and completion.
        if isinstance(value, dict) and 'radius_m' in value:
            value = dict(value)
            value.setdefault('completion_radius_m', value['radius_m'])
            if 'extended_radius_m' not in value:
                try:
                    value['extended_radius_m'] = max(5.0, float(value['radius_m']) + .5)
                except (TypeError, ValueError, OverflowError):
                    pass  # The normal numeric field validation reports the invalid radius.
        return value

    @model_validator(mode='after')
    def extended_outside_goal(self):
        if self.extended_radius_m <= self.radius_m:
            raise ValueError('Extended goal radius must be greater than the goal radius')
        return self


class Speeds(Model):
    cow_linear_mps: Positive = .5
    cow_angular_radps: Positive = 1
    drone_linear_mps: Positive = .4
    drone_angular_radps: Positive = 1


class Visualizations(Model):
    gazebo: bool = False
    rviz: bool = False
    occupancy: bool = False
    vectors: bool = False
    visited: bool = False
    camera: bool = False
    tracker: bool = False


class DeleteRuns(Model):
    run_ids: list[Annotated[str, Field(pattern=r'^[0-9a-f]{32}$')]] = Field(min_length=1, max_length=500)


class Scenario(Model):
    name: str = Field(min_length=1, max_length=100)
    description: str = Field(default='', max_length=2000)
    map: MapSettings = Field(default_factory=MapSettings)
    goal: Goal = Field(default_factory=lambda: Goal(x=-10, y=-10))
    drones: list[Point] = Field(min_length=1, max_length=16)
    cows: list[CowStart] = Field(min_length=1, max_length=100)
    speeds: Speeds = Field(default_factory=Speeds)
    visualization: Visualizations = Field(default_factory=Visualizations)
    duration_s: Annotated[float, Field(gt=0, le=1200, allow_inf_nan=False)] = 1200

    @model_validator(mode='after')
    def positions_inside_map(self):
        for point in [self.goal, *self.drones, *self.cows]:
            if (abs(point.x - self.map.center_x) > self.map.width_m / 2 or
                    abs(point.y - self.map.center_y) > self.map.height_m / 2):
                raise ValueError('All starting positions and the goal must be inside the map')
        if any(p.z < 0 for p in self.cows):
            raise ValueError('Cow starting height must be nonnegative')
        return self


class BatchItem(Model):
    scenario_id: str | None = None
    scenario: Scenario | None = None
    repeats: int = Field(default=1, ge=1, le=100)

    @model_validator(mode='after')
    def exactly_one_scenario(self):
        if (self.scenario is None) == (self.scenario_id is None):
            raise ValueError('Supply either scenario_id or scenario')
        return self


class BatchSpec(Model):
    version: int = Field(default=1, ge=1, le=1)
    name: str = Field(default='Experiment batch', min_length=1, max_length=100)
    items: list[BatchItem] = Field(min_length=1, max_length=100)

    @model_validator(mode='after')
    def bounded_queue(self):
        if sum(item.repeats for item in self.items) > 500:
            raise ValueError('A batch may contain at most 500 runs')
        return self
