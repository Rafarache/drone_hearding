export type Point = { x: number; y: number; z?: number };
export type Visualizations = {
  gazebo: boolean;
  rviz: boolean;
  occupancy: boolean;
  vectors: boolean;
  visited: boolean;
  camera: boolean;
  tracker: boolean;
};
export const emptyVisualizations: Visualizations = {
  gazebo: false,
  rviz: false,
  occupancy: false,
  vectors: false,
  visited: false,
  camera: false,
  tracker: false,
};
export const visualizationLabels: Record<keyof Visualizations, string> = {
  gazebo: "Gazebo simulation",
  rviz: "RViz",
  occupancy: "Occupancy grids",
  vectors: "Vector fields",
  visited: "Visited Areas maps",
  camera: "Annotated cameras",
  tracker: "Cow tracker plot",
};
export type SearchGoal = {
  viewpoint: [number, number];
  patch_center: [number, number];
  patch: [number, number][];
  strength: number;
};
export type DroneSearch = {
  drone: string;
  first_t: number;
  grid: {
    width: number;
    height: number;
    resolution: number;
    origin: [number, number];
  };
  cells: [number, number][];
  goals: { t: number; goal: SearchGoal | null }[];
};
export type SearchHistory = { available: boolean; drones: DroneSearch[] };
export type LiveState = {
  available: boolean;
  settings: Visualizations;
  applied: boolean;
  errors: string[];
  display?: string;
  virtual_display?: boolean;
};
export type Scenario = {
  name: string;
  description: string;
  map: {
    center_x: number;
    center_y: number;
    width_m: number;
    height_m: number;
  };
  goal: Point & { radius_m: number };
  drones: Point[];
  cows: Point[];
  speeds: {
    cow_linear_mps: number;
    cow_angular_radps: number;
    drone_linear_mps: number;
    drone_angular_radps: number;
  };
  duration_s: number;
  visualization?: Visualizations;
};
export type SavedScenario = Scenario & {
  id: string;
  created_at: string;
  updated_at: string;
};
export type Entity = {
  t: number;
  entity: string;
  kind: "cow" | "drone";
  x: number;
  y: number;
  z: number;
  qx: number;
  qy: number;
  qz: number;
  qw: number;
  vx: number;
  vy: number;
  vz: number;
};
export type Frame = { t: number; entities: Entity[] };
export type Detection = { t: number; points: [number, number][] };
export type Trajectory = { frames: Frame[]; detections: Detection[] };
export type EntityMetric = {
  name: string;
  kind: string;
  samples: number;
  distance_xy_m: number;
  distance_3d_m: number;
  average_speed_xy_mps: number;
  average_speed_3d_mps: number;
  peak_speed_xy_mps: number;
  peak_speed_3d_mps: number;
  observed_duration_s: number;
  valid_duration_s: number;
  incomplete: boolean;
  missing_intervals: number[][];
  first_goal_arrival_s?: number | null;
  final_goal_distance_m?: number;
  final_inside_goal?: boolean;
};
export type Summary = {
  entities: EntityMetric[];
  cows_at_goal: number;
  missing_entities: string[];
  distance_cows_m?: number;
  distance_drones_m?: number;
};
export type Run = {
  id: string;
  batch_id: string;
  batch_name: string;
  scenario: Scenario;
  entities: { name: string; kind: string; index: number }[];
  status: string;
  repetition: number;
  created_at: string;
  started_at?: string;
  finished_at?: string;
  simulation_duration_s?: number;
  real_duration_s?: number;
  startup_duration_s?: number;
  reason?: string;
  cancel_requested: boolean;
  summary?: Summary;
};
export type Batch = {
  id: string;
  name: string;
  created: string;
  status: string;
  runs: Run[];
};
export const terminal = (status: string) =>
  ["success", "timed_out", "failed", "cancelled", "interrupted"].includes(
    status,
  );
export const label = (name: string, kind: string) =>
  (kind === "drone" ? "D" : "C") + (name.match(/\d+$/)?.[0] || "0");
export const clock = (value = 0) =>
  `${Math.floor(value / 60)
    .toString()
    .padStart(2, "0")}:${Math.floor(value % 60)
    .toString()
    .padStart(2, "0")}`;
export const fixed = (value: number | undefined, digits = 2) =>
  value === undefined ? "—" : value.toFixed(digits);
export async function api<T>(path: string, options?: RequestInit): Promise<T> {
  const response = await fetch("/api" + path, {
    ...options,
    headers: { "Content-Type": "application/json", ...options?.headers },
  });
  if (!response.ok) {
    let detail;
    try {
      detail = (await response.json()).detail;
    } catch {
      detail = response.statusText;
    }
    throw new Error(
      typeof detail === "string" ? detail : JSON.stringify(detail),
    );
  }
  return response.status === 204 ? (undefined as T) : response.json();
}
export const scenarioOnly = (s: Scenario): Scenario => ({
  name: s.name,
  description: s.description,
  map: { ...s.map },
  goal: { ...s.goal },
  drones: s.drones.map((p) => ({ ...p })),
  cows: s.cows.map((p) => ({ ...p })),
  speeds: { ...s.speeds },
  duration_s: s.duration_s,
  visualization: { ...emptyVisualizations, ...s.visualization },
});
