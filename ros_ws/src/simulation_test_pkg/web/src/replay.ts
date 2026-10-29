import { Entity, Frame } from "./types";
export function lowerFrame(frames: Frame[], t: number) {
  let low = 0,
    high = frames.length;
  while (low < high) {
    const middle = (low + high) >>> 1;
    if (frames[middle].t <= t) low = middle + 1;
    else high = middle;
  }
  return low - 1;
}
export function interpolate(frames: Frame[], t: number): Entity[] {
  const index = lowerFrame(frames, t);
  if (index < 0) return [];
  const before = frames[index],
    after = frames[index + 1];
  if (t - before.t > 1) return [];
  if (!after || after.t - before.t > 1) return before.entities;
  const fraction = Math.max(
    0,
    Math.min(1, (t - before.t) / (after.t - before.t || 1)),
  );
  return before.entities.map((entity) => {
    const next = after.entities.find((e) => e.entity === entity.entity);
    if (!next) return entity;
    const yaw = (e: Entity) =>
      Math.atan2(
        2 * (e.qw * e.qz + e.qx * e.qy),
        1 - 2 * (e.qy * e.qy + e.qz * e.qz),
      );
    const a = yaw(entity),
      delta = Math.atan2(Math.sin(yaw(next) - a), Math.cos(yaw(next) - a)),
      angle = a + delta * fraction;
    return {
      ...entity,
      x: entity.x + (next.x - entity.x) * fraction,
      y: entity.y + (next.y - entity.y) * fraction,
      z: entity.z + (next.z - entity.z) * fraction,
      qx: 0,
      qy: 0,
      qz: Math.sin(angle / 2),
      qw: Math.cos(angle / 2),
    };
  });
}
export function trailOpacity(
  sampleTime: number,
  playhead: number,
  window: number,
) {
  return Math.max(0, Math.min(1, 1 - (playhead - sampleTime) / window));
}

// Coverage and selected goals are stepwise knowledge, never interpolated.
export function searchAtTime(
  history: import("./types").SearchHistory,
  time: number,
  selected: string[],
) {
  return history.drones
    .filter((drone) => selected.includes(drone.drone) && drone.first_t <= time)
    .map((drone) => {
      const cells = drone.cells
        .filter(([, seen]) => seen <= time)
        .map(([cell]) => cell);
      const goal =
        [...drone.goals].reverse().find((state) => state.t <= time)?.goal ??
        null;
      return {
        ...drone,
        visibleCells: cells,
        goal,
        percent: (cells.length / (drone.grid.width * drone.grid.height)) * 100,
      };
    });
}
