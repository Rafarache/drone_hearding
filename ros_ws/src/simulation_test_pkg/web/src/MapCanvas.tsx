import { useEffect, useMemo, useRef } from "react";
import {
  Scenario,
  Frame,
  Detection,
  Point,
  label,
  completionRadius,
  extendedRadius,
} from "./types";
import { interpolate, trailOpacity, searchAtTime } from "./replay";
const drones = ["#2684de", "#5469d4", "#198c93", "#865ad3"];
const cows = ["#d86f4d", "#b98b2e", "#ca565e", "#9b6b51"];
export function MapCanvas({
  scenario,
  frames = [],
  overview = [],
  time = 0,
  trail = 30,
  fullPath = false,
  detections = [],
  showDetections = false,
  searchStates = [],
  showSearched = false,
  showUnsearched = false,
  showSearchGoals = false,
  edit,
  onMove,
}: {
  scenario: Scenario;
  frames?: Frame[];
  overview?: Frame[];
  time?: number;
  trail?: number;
  fullPath?: boolean;
  detections?: Detection[];
  showDetections?: boolean;
  searchStates?: ReturnType<typeof searchAtTime>;
  showSearched?: boolean;
  showUnsearched?: boolean;
  showSearchGoals?: boolean;
  edit?: boolean;
  onMove?: (group: "drones" | "cows" | "goal", index: number, p: Point) => void;
}) {
  const canvas = useRef<HTMLCanvasElement>(null),
    drag = useRef<{ group: "drones" | "cows" | "goal"; index: number } | null>(
      null,
    );
  const transform = useRef({
    left: 0,
    top: 0,
    scale: 1,
    width: 1,
    height: 1,
    minX: 0,
    maxY: 0,
  });
  const trailGroups = useMemo(() => {
    const groups = new Map<
      string,
      { t: number; x: number; y: number; kind: string }[]
    >();
    for (const frame of fullPath ? overview : frames)
      for (const entity of frame.entities) {
        if (!groups.has(entity.entity)) groups.set(entity.entity, []);
        groups
          .get(entity.entity)!
          .push({ t: frame.t, x: entity.x, y: entity.y, kind: entity.kind });
      }
    return groups;
  }, [frames, overview, fullPath]);
  const drawRef = useRef<() => void>(() => {});
  drawRef.current = () => {
    const element = canvas.current;
    if (!element) return;
    const context = element.getContext("2d");
    if (!context) return;
    const bounds = element.getBoundingClientRect(),
      dpr = window.devicePixelRatio || 1;
    if (bounds.width < 80 || bounds.height < 80) return;
    element.width = Math.round(bounds.width * dpr);
    element.height = Math.round(bounds.height * dpr);
    context.scale(dpr, dpr);
    const { width, height } = bounds,
      padding = 36,
      scale = Math.min(
        (width - padding * 2) / scenario.map.width_m,
        (height - padding * 2) / scenario.map.height_m,
      );
    const plotW = scenario.map.width_m * scale,
      plotH = scenario.map.height_m * scale,
      left = (width - plotW) / 2,
      top = (height - plotH) / 2;
    const minX = scenario.map.center_x - scenario.map.width_m / 2,
      maxY = scenario.map.center_y + scenario.map.height_m / 2;
    transform.current = {
      left,
      top,
      scale,
      width: plotW,
      height: plotH,
      minX,
      maxY,
    };
    const pixel = (x: number, y: number) => [
      left + (x - minX) * scale,
      top + (maxY - y) * scale,
    ];
    context.fillStyle = "#f7fafb";
    context.fillRect(0, 0, width, height);
    context.fillStyle = "#fff";
    context.fillRect(left, top, plotW, plotH);
    context.save();
    context.beginPath();
    context.rect(left, top, plotW, plotH);
    context.clip();
    for (const state of searchStates) {
      const g = state.grid;
      if (showUnsearched) {
        const p = pixel(g.origin[0], g.origin[1] + g.height * g.resolution);
        context.fillStyle = "#f3eadc";
        context.fillRect(
          p[0],
          p[1],
          g.width * g.resolution * scale,
          g.height * g.resolution * scale,
        );
      }
    }
    if (showSearched) {
      context.fillStyle = "#d6eadb";
      for (const state of searchStates) {
        const g = state.grid;
        for (const cell of state.visibleCells) {
          const x = g.origin[0] + (cell % g.width) * g.resolution,
            y = g.origin[1] + Math.floor(cell / g.width) * g.resolution;
          const p = pixel(x, y + g.resolution);
          context.fillRect(
            p[0],
            p[1],
            g.resolution * scale + 0.2,
            g.resolution * scale + 0.2,
          );
        }
      }
    }
    context.restore();
    const step = Math.max(
      1,
      Math.ceil(Math.max(scenario.map.width_m, scenario.map.height_m) / 8 / 5) *
        5,
    );
    context.font = "10px system-ui";
    context.textAlign = "center";
    context.fillStyle = "#85939d";
    context.strokeStyle = "#e8eef1";
    context.lineWidth = 1;
    for (
      let x = Math.ceil(minX / step) * step;
      x <= minX + scenario.map.width_m;
      x += step
    ) {
      const px = pixel(x, 0)[0];
      context.beginPath();
      context.moveTo(px, top);
      context.lineTo(px, top + plotH);
      context.stroke();
      context.fillText(`${x}`, px, top + plotH + 19);
    }
    for (
      let y = Math.ceil((maxY - scenario.map.height_m) / step) * step;
      y <= maxY;
      y += step
    ) {
      const py = pixel(0, y)[1];
      context.beginPath();
      context.moveTo(left, py);
      context.lineTo(left + plotW, py);
      context.stroke();
      context.fillText(`${y}`, left - 19, py + 3);
    }
    context.strokeStyle = "#cfdbe1";
    context.strokeRect(left, top, plotW, plotH);
    context.save();
    context.beginPath();
    context.rect(left, top, plotW, plotH);
    context.clip();
    const goal = pixel(scenario.goal.x, scenario.goal.y);
    context.beginPath();
    context.arc(
      goal[0],
      goal[1],
      scenario.goal.radius_m * scale,
      0,
      Math.PI * 2,
    );
    context.fillStyle = "#2ca78315";
    context.fill();
    context.setLineDash([5, 4]);
    context.strokeStyle = "#2ca783";
    context.lineWidth = 1.5;
    context.stroke();
    context.setLineDash([]);
    const completion = completionRadius(scenario.goal);
    const extended = extendedRadius(scenario.goal);
    if (completion !== scenario.goal.radius_m) {
      context.beginPath();
      context.arc(goal[0], goal[1], completion * scale, 0, Math.PI * 2);
      context.setLineDash([8, 4]);
      context.strokeStyle = "#198c93";
      context.stroke();
    }
    if (
      scenario.goal.extended_radius_m !== undefined &&
      extended !== completion &&
      extended !== scenario.goal.radius_m
    ) {
      context.beginPath();
      context.arc(goal[0], goal[1], extended * scale, 0, Math.PI * 2);
      context.setLineDash([2, 4]);
      context.strokeStyle = "#7b8c99";
      context.stroke();
    }
    context.setLineDash([]);
    context.strokeStyle = "#2ca783";
    context.beginPath();
    context.moveTo(goal[0] - 6, goal[1]);
    context.lineTo(goal[0] + 6, goal[1]);
    context.moveTo(goal[0], goal[1] - 6);
    context.lineTo(goal[0], goal[1] + 6);
    context.stroke();
    for (const [name, points] of trailGroups) {
      let previous: { x: number; y: number; t: number } | null = null;
      const index = Number(name.match(/\d+$/)?.[0] || 0);
      for (const sample of points) {
        if (sample.t > time) break;
        if (!fullPath && sample.t < time - trail) {
          previous = null;
          continue;
        }
        const p = pixel(sample.x, sample.y);
        if (previous && sample.t - previous.t <= (fullPath ? 1.1 : 1.000001)) {
          context.beginPath();
          context.moveTo(previous.x, previous.y);
          context.lineTo(p[0], p[1]);
          context.strokeStyle = (sample.kind === "drone" ? drones : cows)[
            index % 4
          ];
          context.globalAlpha = fullPath
            ? 0.3
            : 0.08 + 0.75 * trailOpacity(sample.t, time, trail);
          context.lineWidth = 2;
          context.stroke();
        }
        previous = { x: p[0], y: p[1], t: sample.t };
      }
    }
    context.globalAlpha = 1;
    if (showSearchGoals) {
      for (const state of searchStates) {
        if (!state.goal) continue;
        const index = Number(state.drone.match(/\d+$/)?.[0] || 0),
          color = drones[index % 4],
          target = state.goal;
        context.strokeStyle = color;
        context.fillStyle = color + "18";
        context.lineWidth = 1.7;
        context.setLineDash([5, 3]);
        context.beginPath();
        target.patch.forEach((point, i) => {
          const p = pixel(...point);
          if (i === 0) context.moveTo(p[0], p[1]);
          else context.lineTo(p[0], p[1]);
        });
        context.closePath();
        context.fill();
        context.stroke();
        const p = pixel(...target.viewpoint),
          center = pixel(...target.patch_center);
        context.beginPath();
        context.moveTo(p[0], p[1]);
        context.lineTo(center[0], center[1]);
        context.stroke();
        context.setLineDash([]);
        context.beginPath();
        context.moveTo(p[0], p[1] - 7);
        context.lineTo(p[0] + 7, p[1]);
        context.lineTo(p[0], p[1] + 7);
        context.lineTo(p[0] - 7, p[1]);
        context.closePath();
        context.fillStyle = "#fff";
        context.fill();
        context.stroke();
        context.font = "600 10px system-ui";
        context.textAlign = "left";
        context.fillStyle = color;
        context.fillText(
          label(state.drone, "drone") + " SEARCH",
          Math.max(left + 3, Math.min(left + plotW - 65, p[0] + 10)),
          Math.max(top + 12, p[1] - 10),
        );
      }
    }
    if (showDetections) {
      const detection = [...detections].reverse().find((d) => d.t <= time);
      if (detection) {
        context.strokeStyle = "#9a61c8";
        context.lineWidth = 1.7;
        context.setLineDash([3, 3]);
        for (const p of detection.points) {
          const xy = pixel(...p);
          context.beginPath();
          context.arc(xy[0], xy[1], 8, 0, Math.PI * 2);
          context.stroke();
        }
        context.setLineDash([]);
      }
    }
    context.restore();
    const current = edit
      ? [
          ...scenario.drones.map((p, index) => ({
            ...p,
            z: 0,
            kind: "drone" as const,
            entity: `drone${index}`,
            qw: 1,
            qx: 0,
            qy: 0,
            qz: 0,
          })),
          ...scenario.cows.map((p, index) => ({
            ...p,
            z: p.z || 0,
            kind: "cow" as const,
            entity: `cow${index}`,
            qw: 1,
            qx: 0,
            qy: 0,
            qz: 0,
          })),
        ]
      : interpolate(frames, time);
    for (const entity of current) {
      const p = pixel(entity.x, entity.y),
        index = Number(entity.entity.match(/\d+$/)?.[0] || 0);
      context.fillStyle = (entity.kind === "drone" ? drones : cows)[index % 4];
      context.strokeStyle = "#fff";
      context.lineWidth = 2;
      if (entity.kind === "drone") {
        const angle = -Math.atan2(
          2 * (entity.qw * entity.qz + entity.qx * entity.qy),
          1 - 2 * (entity.qy * entity.qy + entity.qz * entity.qz),
        );
        context.save();
        context.translate(p[0], p[1]);
        context.rotate(angle);
        context.beginPath();
        context.moveTo(9, 0);
        context.lineTo(-6, -6);
        context.lineTo(-3, 0);
        context.lineTo(-6, 6);
        context.closePath();
        context.fill();
        context.stroke();
        context.restore();
      } else {
        context.beginPath();
        context.arc(p[0], p[1], 5, 0, Math.PI * 2);
        context.fill();
        context.stroke();
      }
      context.font = "600 11px system-ui";
      context.textAlign = "left";
      context.lineWidth = 3;
      context.strokeStyle = "#fff";
      const text = label(entity.entity, entity.kind),
        tx = Math.min(left + plotW - 22, Math.max(left + 3, p[0] + 10)),
        ty = Math.max(top + 12, p[1] - 7);
      context.strokeText(text, tx, ty);
      context.fillStyle = "#425561";
      context.fillText(text, tx, ty);
    }
    context.font = "600 10px system-ui";
    context.textAlign = "left";
    context.fillStyle = "#258b6d";
    context.fillText(
      "GOAL",
      Math.min(left + plotW - 34, Math.max(left + 3, goal[0] + 9)),
      Math.max(top + 12, goal[1] - 9),
    );
    context.font = "10px system-ui";
    context.fillStyle = "#85939d";
    context.fillText(
      "+Y ↑  /  +X →",
      left + plotW - 74,
      Math.max(12, top - 12),
    );
  };
  useEffect(() => {
    drawRef.current();
  }, [
    scenario,
    frames,
    overview,
    time,
    trail,
    fullPath,
    detections,
    showDetections,
    searchStates,
    showSearched,
    showUnsearched,
    showSearchGoals,
    edit,
  ]);
  useEffect(() => {
    const element = canvas.current;
    if (!element) return;
    const observer = new ResizeObserver(() => drawRef.current());
    observer.observe(element);
    return () => observer.disconnect();
  }, []);
  const coordinates = (event: React.PointerEvent) => {
    const rect = canvas.current!.getBoundingClientRect(),
      p = transform.current;
    return {
      x: Math.min(
        p.minX + scenario.map.width_m,
        Math.max(
          p.minX,
          (event.clientX - rect.left - p.left) / p.scale + p.minX,
        ),
      ),
      y: Math.min(
        p.maxY,
        Math.max(
          p.maxY - scenario.map.height_m,
          p.maxY - (event.clientY - rect.top - p.top) / p.scale,
        ),
      ),
    };
  };
  return (
    <canvas
      ref={canvas}
      className={"field-map " + (edit && onMove ? "editable" : "")}
      aria-label={
        edit
          ? onMove
            ? "Scenario map: drag drone, cow, or goal markers; coordinates can also be edited in the position tables"
            : "Scenario starting positions"
          : "Recorded top-down positions of drones, cows, and the goal"
      }
      onPointerDown={(event) => {
        if (!edit || !onMove) return;
        const p = coordinates(event),
          scale = transform.current.scale;
        const choices = [
          ...scenario.drones.map((v, index) => ({
            group: "drones" as const,
            index,
            p: v,
          })),
          ...scenario.cows.map((v, index) => ({
            group: "cows" as const,
            index,
            p: v,
          })),
          { group: "goal" as const, index: 0, p: scenario.goal },
        ];
        const hit = choices.find(
          (v) => Math.hypot(v.p.x - p.x, v.p.y - p.y) * scale < 16,
        );
        if (hit) {
          drag.current = hit;
          event.currentTarget.setPointerCapture(event.pointerId);
        }
      }}
      onPointerMove={(event) => {
        if (!drag.current) return;
        const p = coordinates(event);
        onMove?.(drag.current.group, drag.current.index, {
          x: Math.round(p.x * 10) / 10,
          y: Math.round(p.y * 10) / 10,
        });
      }}
      onPointerUp={() => {
        drag.current = null;
      }}
      onPointerCancel={() => {
        drag.current = null;
      }}
    />
  );
}
