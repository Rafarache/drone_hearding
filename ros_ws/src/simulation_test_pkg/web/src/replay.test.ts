import { describe, it, expect } from "vitest";
import { interpolate, trailOpacity } from "./replay";
import { Entity } from "./types";
const entity = (x: number, yaw = 0): Entity => ({
  entity: "cow0",
  kind: "cow",
  t: 0,
  x,
  y: 0,
  z: 0,
  qx: 0,
  qy: 0,
  qz: Math.sin(yaw / 2),
  qw: Math.cos(yaw / 2),
  vx: 0,
  vy: 0,
  vz: 0,
});
describe("replay", () => {
  it("seeks forward and backward by timestamps", () => {
    const frames = [
      { t: 0, entities: [entity(0)] },
      { t: 1, entities: [entity(2)] },
    ];
    expect(interpolate(frames, 0.75)[0].x).toBe(1.5);
    expect(interpolate(frames, 0.25)[0].x).toBe(0.5);
  });
  it("does not interpolate across missing intervals or before recording", () => {
    const frames = [
      { t: 0, entities: [entity(0)] },
      { t: 5, entities: [entity(100)] },
    ];
    expect(interpolate(frames, 3)).toEqual([]);
    expect(interpolate(frames, -1)).toEqual([]);
  });
  it("takes the short heading path across the angle boundary", () => {
    const result = interpolate(
      [
        { t: 0, entities: [entity(0, 3)] },
        { t: 1, entities: [entity(0, -3)] },
      ],
      0.5,
    )[0];
    expect(Math.abs(result.qw)).toBeLessThan(0.001);
  });
  it("fades relative to the selected playhead, including backward seeks", () => {
    expect(trailOpacity(5, 20, 30)).toBe(0.5);
    expect(trailOpacity(5, 10, 30)).toBeCloseTo(5 / 6);
    expect(trailOpacity(0, 40, 30)).toBe(0);
  });
});

import { searchAtTime } from "./replay";
import { SearchHistory } from "./types";
describe("search replay", () => {
  const goal = {
    viewpoint: [1, 2] as [number, number],
    patch_center: [3, 2] as [number, number],
    patch: [
      [2, 1],
      [4, 1],
      [4, 3],
      [2, 3],
    ] as [number, number][],
    strength: 0.6,
  };
  const history: SearchHistory = {
    available: true,
    drones: [
      {
        drone: "drone0",
        first_t: 0,
        grid: { width: 2, height: 2, resolution: 1, origin: [0, 0] },
        cells: [
          [0, 1],
          [1, 5],
        ],
        goals: [
          { t: 0, goal: null },
          { t: 2, goal },
          { t: 6, goal: null },
        ],
      },
      {
        drone: "drone1",
        first_t: 3,
        grid: { width: 2, height: 2, resolution: 1, origin: [0, 0] },
        cells: [[3, 3]],
        goals: [{ t: 3, goal }],
      },
    ],
  };
  it("does not reveal future coverage or goals when seeking backwards", () => {
    expect(searchAtTime(history, 5, ["drone0"])[0].visibleCells).toEqual([
      0, 1,
    ]);
    expect(searchAtTime(history, 1, ["drone0"])[0].visibleCells).toEqual([0]);
    expect(searchAtTime(history, 1, ["drone0"])[0].goal).toBeNull();
    expect(searchAtTime(history, 3, ["drone0"])[0].goal).toEqual(goal);
    expect(searchAtTime(history, 6, ["drone0"])[0].goal).toBeNull();
  });
  it("filters drones and leaves unknown maps unknown", () => {
    expect(searchAtTime(history, 2, ["drone1"])).toEqual([]);
    expect(searchAtTime(history, 4, ["drone1"])[0].visibleCells).toEqual([3]);
    expect(searchAtTime(history, 4, [])).toEqual([]);
  });
});
