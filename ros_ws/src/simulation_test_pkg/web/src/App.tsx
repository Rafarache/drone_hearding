import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import {
  Activity,
  ArrowLeft,
  ArrowRight,
  Check,
  ChevronRight,
  Clock3,
  Copy,
  Download,
  FileText,
  FlaskConical,
  Layers,
  LayoutDashboard,
  MapPin,
  Pause,
  Play,
  Plus,
  Radio,
  RefreshCw,
  Search,
  Settings2,
  Square,
  Trash2,
  TrendingUp,
  X,
} from "lucide-react";
import {
  api,
  Batch,
  clock,
  Detection,
  EntityMetric,
  fixed,
  Frame,
  label,
  Run,
  SavedScenario,
  Scenario,
  scenarioOnly,
  Summary,
  terminal,
  Trajectory,
  emptyVisualizations,
  completionRadius,
  extendedRadius,
} from "./types";
import { MapCanvas } from "./MapCanvas";
import { SearchControls, useSearchHistory } from "./SearchControls";
import { searchAtTime } from "./replay";
import { ViewSettings, LiveControls } from "./LiveControls";
const empty: Trajectory = { frames: [], detections: [] };
const statusText = (status: string) =>
  ({
    success: "Completed",
    timed_out: "Time limit",
    queued: "Queued",
    starting: "Starting",
    running: "Running",
    cancelled: "Cancelled",
    failed: "Failed",
    interrupted: "Interrupted",
  })[status] || status;
function Badge({ status }: { status: string }) {
  return (
    <span className={`badge ${status}`}>
      <span />
      {statusText(status)}
    </span>
  );
}
function Numeric({
  label: caption,
  value,
  onChange,
  min,
  step = "any",
}: {
  label: string;
  value: number;
  onChange: (v: number) => void;
  min?: number;
  step?: number | "any";
}) {
  return (
    <label className="input-label">
      {caption}
      <input
        type="number"
        value={value}
        min={min}
        step={step}
        required
        onChange={(event) => onChange(Number(event.target.value))}
      />
    </label>
  );
}
function Modal({
  title,
  subtitle,
  children,
  onClose,
  wide = false,
}: {
  title: string;
  subtitle?: string;
  children: React.ReactNode;
  onClose: () => void;
  wide?: boolean;
}) {
  const ref = useRef<HTMLDialogElement>(null);
  useEffect(() => {
    ref.current?.showModal();
    return () => ref.current?.close();
  }, []);
  return (
    <dialog
      ref={ref}
      aria-label={title}
      className={wide ? "modal wide" : "modal"}
      onCancel={onClose}
      onClick={(event) => {
        if (event.target === ref.current) onClose();
      }}
    >
      <div className="modal-heading">
        <div>
          <h2>{title}</h2>
          {subtitle && <p>{subtitle}</p>}
        </div>
        <button
          className="icon-button"
          aria-label="Close dialog"
          onClick={onClose}
        >
          <X size={20} />
        </button>
      </div>
      {children}
    </dialog>
  );
}
function MetricTable({ entities }: { entities: EntityMetric[] }) {
  return (
    <div className="table-scroll">
      <table>
        <thead>
          <tr>
            <th>Entity</th>
            <th>Distance · XY</th>
            <th>Avg. speed · XY</th>
            <th>Peak speed · XY</th>
            <th>Distance · 3D</th>
            <th>Avg. speed · 3D</th>
            <th>At goal</th>
          </tr>
        </thead>
        <tbody>
          {entities.map((entity) => (
            <tr key={entity.name}>
              <td>
                <span className={"entity-dot " + entity.kind} />
                <strong>{label(entity.name, entity.kind)}</strong>
                <small>{entity.name}</small>
                {entity.incomplete && (
                  <span className="warning-text">Recording gap</span>
                )}
              </td>
              <td>{fixed(entity.distance_xy_m)} m</td>
              <td>{fixed(entity.average_speed_xy_mps, 3)} m/s</td>
              <td>{fixed(entity.peak_speed_xy_mps, 3)} m/s</td>
              <td>{fixed(entity.distance_3d_m)} m</td>
              <td>{fixed(entity.average_speed_3d_mps, 3)} m/s</td>
              <td>
                {entity.kind === "cow" ? (
                  entity.final_inside_goal ? (
                    <span className="positive">
                      Yes · {clock(entity.first_goal_arrival_s ?? 0)}
                    </span>
                  ) : (
                    <span>{fixed(entity.final_goal_distance_m, 1)} m away</span>
                  )
                ) : (
                  "—"
                )}
              </td>
            </tr>
          ))}
        </tbody>
      </table>
      {!entities.length && (
        <div className="empty-inline">
          Metrics will appear when positions have been recorded.
        </div>
      )}
    </div>
  );
}
function useReplay(run: Run, time: number, trail: number) {
  const [data, setData] = useState<Trajectory>(empty),
    [overview, setOverview] = useState<Frame[]>([]),
    [loading, setLoading] = useState(false),
    [error, setError] = useState("");
  const cache = useRef(new Map<string, Trajectory>());
  const duration = run.simulation_duration_s || 0;
  const start = Math.max(0, Math.floor((time - trail) / 60) * 60),
    end = Math.min(duration, Math.floor(time / 60) * 60 + 61),
    live = !terminal(run.status);
  useEffect(() => {
    let active = true;
    const key = `${run.id}/${start}/${end}`;
    const saved = cache.current.get(key);
    if (saved) {
      setData(saved);
      return;
    }
    setLoading(true);
    setError("");
    api<Trajectory>(
      `/runs/${run.id}/trajectory?start=${start}&end=${Math.max(start, end)}`,
    )
      .then((result) => {
        if (!active) return;
        if (!live) {
          cache.current.set(key, result);
          if (cache.current.size > 30)
            cache.current.delete(cache.current.keys().next().value!);
        }
        setData(result);
      })
      .catch((reason) => {
        if (active) setError(reason.message);
      })
      .finally(() => {
        if (active) setLoading(false);
      });
    return () => {
      active = false;
    };
  }, [run.id, start, end, live, duration]);
  useEffect(() => {
    let active = true;
    api<Trajectory>(`/runs/${run.id}/trajectory?stride=10`)
      .then((result) => {
        if (active) setOverview(result.frames);
      })
      .catch(() => {});
    return () => {
      active = false;
    };
  }, [run.id, run.status, Math.floor(duration / 10)]);
  return { data, overview, loading, error };
}
function Replay({
  run,
  refresh,
  onClone,
  onDelete,
}: {
  run: Run;
  refresh: () => void;
  onClone: (scenario: Scenario) => void;
  onDelete: (run: Run) => void;
}) {
  const [time, setTime] = useState(0),
    [playing, setPlaying] = useState(false),
    [speed, setSpeed] = useState(1),
    [trail, setTrail] = useState(30),
    [fullPath, setFullPath] = useState(false),
    [showDetections, setShowDetections] = useState(true),
    [follow, setFollow] = useState(!terminal(run.status));
  const [tab, setTab] = useState("Overview"),
    [summary, setSummary] = useState<Summary | null>(run.summary || null),
    [config, setConfig] = useState<{
      manifest: unknown;
      files: Record<string, string>;
    } | null>(null),
    [logs, setLogs] = useState<Record<string, string> | null>(null),
    [error, setError] = useState("");
  const [selectedSearch, setSelectedSearch] = useState(
    run.entities.filter((e) => e.kind === "drone").map((e) => e.name),
  );
  const [showSearched, setShowSearched] = useState(true);
  const [showUnsearched, setShowUnsearched] = useState(true);
  const [showSearchGoals, setShowSearchGoals] = useState(true);
  const searchHistory = useSearchHistory(run);
  const searchStates = useMemo(
    () => searchAtTime(searchHistory.data, time, selectedSearch),
    [searchHistory.data, time, selectedSearch],
  );
  const duration = run.simulation_duration_s || 0;
  const {
    data,
    overview,
    loading,
    error: loadError,
  } = useReplay(run, time, trail);
  useEffect(() => {
    if (follow && !terminal(run.status)) setTime(duration);
  }, [duration, follow, run.status]);
  useEffect(() => {
    if (!playing) return;
    let frame: number,
      last = performance.now();
    const tick = (now: number) => {
      const dt = (now - last) / 1000;
      last = now;
      setTime((previous) => {
        const next = Math.min(duration, previous + dt * speed);
        if (next >= duration) setPlaying(false);
        return next;
      });
      frame = requestAnimationFrame(tick);
    };
    frame = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(frame);
  }, [playing, speed, duration]);
  useEffect(() => {
    let active = true;
    api<Summary>(`/runs/${run.id}/summary`)
      .then((result) => {
        if (active) setSummary(result);
      })
      .catch((reason) => {
        if (active) setError(reason.message);
      });
    return () => {
      active = false;
    };
  }, [run.id, run.status, Math.floor(duration / 5)]);
  useEffect(() => {
    if (tab === "Configuration")
      api<{ manifest: unknown; files: Record<string, string> }>(
        `/runs/${run.id}/configuration`,
      )
        .then(setConfig)
        .catch((reason) => setError(reason.message));
    if (tab === "Logs")
      api<Record<string, string>>(`/runs/${run.id}/logs`)
        .then(setLogs)
        .catch((reason) => setError(reason.message));
  }, [tab, run.id, run.status]);
  const action = async (kind: string) => {
    try {
      await api(`/runs/${run.id}/${kind}`, { method: "POST" });
      refresh();
    } catch (reason) {
      setError((reason as Error).message);
    }
  };
  const latestDetection = [...data.detections]
    .reverse()
    .find((d) => d.t <= time);
  return (
    <div className="run-detail">
      <div className="detail-heading">
        <div>
          <div className="eyebrow">
            EXPERIMENT {run.id.slice(0, 8).toUpperCase()} · REPEAT{" "}
            {run.repetition}
          </div>
          <h2>{run.scenario.name}</h2>
          <p>
            {run.batch_name} <span>·</span>{" "}
            {new Date(run.created_at).toLocaleString()}
          </p>
        </div>
        <div className="detail-actions">
          <Badge status={run.status} />
          <button
            className="icon-button danger"
            aria-label="Delete experiment"
            title="Delete experiment"
            disabled={["starting", "running"].includes(run.status)}
            onClick={() => onDelete(run)}
          >
            <Trash2 size={17} />
          </button>
          <button
            className="icon-button"
            aria-label="Clone run settings into a scenario"
            title="Clone settings"
            onClick={() =>
              onClone({
                ...scenarioOnly(run.scenario),
                name: run.scenario.name + " copy",
              })
            }
          >
            <Copy size={17} />
          </button>
          {terminal(run.status) ? (
            <button className="button small" onClick={() => action("rerun")}>
              <RefreshCw size={14} />
              Rerun
            </button>
          ) : (
            <button
              className="button small danger"
              onClick={() => action("cancel")}
              disabled={run.cancel_requested}
            >
              <Square size={12} />
              {run.cancel_requested ? "Stopping…" : "Cancel"}
            </button>
          )}
        </div>
      </div>
      {(error || loadError) && (
        <div className="error" role="alert">
          {error || loadError}
        </div>
      )}
      <div className="stats-grid">
        <div>
          <span>Simulation time</span>
          <strong>
            {clock(duration)}
            <small> / {clock(run.scenario.duration_s)}</small>
          </strong>
        </div>
        <div>
          <span>Real elapsed</span>
          <strong>{clock(run.real_duration_s)}</strong>
        </div>
        <div>
          <span>Cows at goal</span>
          <strong>
            {summary?.cows_at_goal ?? "—"}
            <small> / {run.scenario.cows.length}</small>
          </strong>
        </div>
        <div>
          <span>Total distance · XY</span>
          <strong>
            {fixed(
              (summary?.distance_cows_m || 0) +
                (summary?.distance_drones_m || 0),
              1,
            )}
            <small> m</small>
          </strong>
        </div>
      </div>
      <LiveControls run={run} />
      <section className="map-card">
        <div className="map-toolbar">
          <div className="map-title">
            <MapPin size={16} /> Trajectory replay{" "}
            {loading && <span className="subtle">Loading…</span>}
          </div>
          <div className="map-options">
            <label>
              <input
                type="checkbox"
                checked={showDetections}
                onChange={(event) => setShowDetections(event.target.checked)}
              />{" "}
              Detected cows
            </label>
            <select
              aria-label="Path trail duration"
              value={fullPath ? "full" : trail}
              onChange={(event) => {
                setFullPath(event.target.value === "full");
                if (event.target.value !== "full")
                  setTrail(Number(event.target.value));
              }}
            >
              <option value="15">15 s trail</option>
              <option value="30">30 s trail</option>
              <option value="60">60 s trail</option>
              <option value="120">120 s trail</option>
              <option value="full">Full path</option>
            </select>
          </div>
        </div>
        <SearchControls
          history={searchHistory.data}
          time={time}
          selected={selectedSearch}
          setSelected={setSelectedSearch}
          searched={showSearched}
          setSearched={setShowSearched}
          unsearched={showUnsearched}
          setUnsearched={setShowUnsearched}
          goals={showSearchGoals}
          setGoals={setShowSearchGoals}
          run={run}
        />
        {searchHistory.error && (
          <div className="error" role="alert">
            {searchHistory.error}
          </div>
        )}
        <div className="map-wrap">
          <MapCanvas
            scenario={run.scenario}
            frames={data.frames}
            overview={overview}
            time={time}
            trail={trail}
            fullPath={fullPath}
            detections={data.detections}
            showDetections={showDetections}
            searchStates={searchStates}
            showSearched={showSearched}
            showUnsearched={showUnsearched}
            showSearchGoals={showSearchGoals}
          />
          {!data.frames.length && (
            <div className="map-message">
              {run.status === "queued"
                ? "Waiting in the queue"
                : run.status === "starting"
                  ? "Waiting for models and simulation clock"
                  : loading
                    ? "Loading positions…"
                    : "No positions recorded at this time"}
            </div>
          )}
        </div>
        <div className="map-legend">
          {searchStates.length > 0 && showSearched && (
            <span>
              <i className="legend-searched" />
              Searched
            </span>
          )}
          {searchStates.length > 0 && showUnsearched && (
            <span>
              <i className="legend-unsearched" />
              Unsearched
            </span>
          )}
          <span>
            <i className="legend-drone" />
            Drones
          </span>
          <span>
            <i className="legend-cow" />
            Cows
          </span>
          <span>
            <i className="legend-goal" />
            Goal {run.scenario.goal.radius_m} m · Completion{" "}
            {completionRadius(run.scenario.goal)} m
            {run.scenario.goal.extended_radius_m !== undefined &&
              ` · Extended ${run.scenario.goal.extended_radius_m} m`}
          </span>
          <span>
            <i className="legend-detection" />
            Detected
            {latestDetection
              ? ` · ${(time - latestDetection.t).toFixed(1)} s old`
              : ""}
          </span>
          <span className="legend-note">Meters · simulation time</span>
        </div>
        <div className="player">
          <div className="player-controls">
            <button
              className="play-button"
              aria-label={playing ? "Pause replay" : "Play replay"}
              onClick={() => {
                setFollow(false);
                if (time >= duration) setTime(0);
                setPlaying(!playing);
              }}
              disabled={!duration}
            >
              {playing ? <Pause size={18} /> : <Play size={18} />}
            </button>
            <button
              className="icon-button"
              aria-label="Back five seconds"
              onClick={() => {
                setFollow(false);
                setTime(Math.max(0, time - 5));
              }}
            >
              <ArrowLeft size={15} />
            </button>
            <button
              className="icon-button"
              aria-label="Forward five seconds"
              onClick={() => {
                setFollow(false);
                setTime(Math.min(duration, time + 5));
              }}
            >
              <ArrowRight size={15} />
            </button>
            <span className="time-display">
              {clock(time)} <span>/ {clock(duration)}</span>
            </span>
            <div className="spacer" />
            {!terminal(run.status) && (
              <button
                className={"live-button " + (follow ? "selected" : "")}
                onClick={() => {
                  setFollow(true);
                  setPlaying(false);
                }}
              >
                <Radio size={13} />
                Live
              </button>
            )}
            <select
              aria-label="Playback speed"
              value={speed}
              onChange={(event) => setSpeed(Number(event.target.value))}
            >
              {[0.25, 0.5, 1, 2, 4, 8].map((value) => (
                <option key={value} value={value}>
                  {value}×
                </option>
              ))}
            </select>
          </div>
          <input
            className="timeline"
            type="range"
            aria-label="Replay time in simulation seconds"
            min="0"
            max={duration || 1}
            step="0.05"
            value={Math.min(time, duration)}
            onChange={(event) => {
              setFollow(false);
              setTime(Number(event.target.value));
            }}
          />
          <div className="timeline-labels">
            <span>00:00</span>
            <span>{clock(duration / 2)}</span>
            <span>{clock(duration)}</span>
          </div>
        </div>
      </section>
      <section className="bottom-card">
        <div className="tabs">
          {["Overview", "Configuration", "Logs"].map((name) => (
            <button
              key={name}
              className={tab === name ? "selected" : ""}
              onClick={() => setTab(name)}
            >
              {name}
            </button>
          ))}
          <div className="spacer" />
          <a
            className="text-button"
            href={`/api/runs/${run.id}/export/metrics`}
          >
            <Download size={14} />
            Metrics CSV
          </a>
          <a
            className="text-button"
            href={`/api/runs/${run.id}/export/trajectory`}
          >
            <Download size={14} />
            Positions CSV
          </a>
        </div>
        {tab === "Overview" && (
          <>
            <div className="section-caption">
              <div>
                <h3>Movement by entity</h3>
                <p>
                  Average speed includes stationary time. Gaps over one second
                  are excluded and flagged.
                </p>
              </div>
            </div>
            <MetricTable entities={summary?.entities || []} />
            <div className="run-notes">
              <span>
                <Clock3 size={14} />
                Startup: {fixed(run.startup_duration_s, 1)} real seconds
              </span>
              <span>
                <TrendingUp size={14} />
                Cow cap {run.scenario.speeds.cow_linear_mps} m/s · Drone cap{" "}
                {run.scenario.speeds.drone_linear_mps} m/s
              </span>
            </div>
            {run.reason && <div className="reason">{run.reason}</div>}
          </>
        )}
        {tab === "Configuration" && (
          <div className="file-list">
            <div className="section-caption">
              <p>Immutable inputs saved when this run was queued.</p>
              <a
                className="text-button"
                href={`/api/runs/${run.id}/export/configuration`}
              >
                <Download size={14} />
                Download snapshot
              </a>
            </div>
            {config ? (
              <>
                <details>
                  <summary>Provenance & checksums</summary>
                  <pre>{JSON.stringify(config.manifest, null, 2)}</pre>
                </details>
                {Object.entries(config.files).map(([name, text]) => (
                  <details key={name}>
                    <summary>
                      <FileText size={14} />
                      {name}
                    </summary>
                    <pre>{text}</pre>
                  </details>
                ))}
              </>
            ) : (
              <p>Loading configuration…</p>
            )}
          </div>
        )}
        {tab === "Logs" && (
          <div className="file-list">
            <div className="section-caption">
              <p>
                Latest 100 KB from each log. Downloads include the complete
                file.
              </p>
              <button
                className="text-button"
                onClick={() =>
                  api<Record<string, string>>(`/runs/${run.id}/logs`)
                    .then(setLogs)
                    .catch((reason) => setError(reason.message))
                }
              >
                <RefreshCw size={14} />
                Refresh
              </button>
            </div>
            {logs &&
              Object.entries(logs).map(([name, text]) => (
                <details key={name} open>
                  <summary>
                    {name}
                    <a
                      className="text-button"
                      href={`/api/runs/${run.id}/export/${name}`}
                    >
                      Download
                    </a>
                  </summary>
                  <pre>{text || "No log output yet."}</pre>
                </details>
              ))}
          </div>
        )}
      </section>
    </div>
  );
}
function ScenarioEditor({
  initial,
  scenarioId,
  onClose,
  onSave,
}: {
  initial: Scenario;
  scenarioId?: string;
  onClose: () => void;
  onSave: () => void;
}) {
  const [draft, setDraft] = useState<Scenario>(() => scenarioOnly(initial)),
    [error, setError] = useState(""),
    [busy, setBusy] = useState(false);
  const move = (
    group: "drones" | "cows" | "goal",
    index: number,
    point: { x: number; y: number; z?: number },
  ) =>
    setDraft((previous) =>
      group === "goal"
        ? { ...previous, goal: { ...previous.goal, ...point } }
        : {
            ...previous,
            [group]: previous[group].map((p, i) =>
              i === index ? { ...p, ...point } : p,
            ),
          },
    );
  const positions = (group: "drones" | "cows") => (
    <div className="position-card">
      <div>
        <h3>
          {group === "drones" ? "Drones" : "Cows"}{" "}
          <span>{draft[group].length}</span>
        </h3>
        <button
          type="button"
          className="text-button"
          onClick={() => {
            const count = draft[group].length;
            setDraft({
              ...draft,
              [group]: [
                ...draft[group],
                {
                  x: draft.map.center_x,
                  y: Math.min(
                    draft.map.center_y + draft.map.height_m / 2,
                    draft.map.center_y + count,
                  ),
                  ...(group === "cows" ? { z: 0.01 } : {}),
                },
              ],
            });
          }}
        >
          <Plus size={14} />
          Add
        </button>
      </div>
      <div className="position-head">
        <span>Entity</span>
        <span>X (m)</span>
        <span>Y (m)</span>
        {group === "cows" && <span>Z (m)</span>}
        <span />
      </div>
      {draft[group].map((p, index) => (
        <div
          className={"position-row " + (group === "cows" ? "has-z" : "")}
          key={index}
        >
          <strong className={group}>
            {group === "cows" ? "C" : "D"}
            {index}
          </strong>
          <input
            aria-label={`${group} ${index} x`}
            type="number"
            step="any"
            value={p.x}
            onChange={(event) =>
              move(group, index, { ...p, x: Number(event.target.value) })
            }
          />
          <input
            aria-label={`${group} ${index} y`}
            type="number"
            step="any"
            value={p.y}
            onChange={(event) =>
              move(group, index, { ...p, y: Number(event.target.value) })
            }
          />
          {group === "cows" && (
            <input
              aria-label={`${group} ${index} z`}
              type="number"
              step="any"
              min="0"
              value={p.z}
              onChange={(event) =>
                move(group, index, { ...p, z: Number(event.target.value) })
              }
            />
          )}
          <button
            type="button"
            className="icon-button"
            aria-label={`Remove ${group} ${index}`}
            disabled={draft[group].length === 1}
            onClick={() =>
              setDraft({
                ...draft,
                [group]: draft[group].filter((_, i) => i !== index),
              })
            }
          >
            <X size={14} />
          </button>
        </div>
      ))}
    </div>
  );
  return (
    <Modal
      wide
      title={scenarioId ? "Edit scenario" : "Create a scenario"}
      subtitle="Set the field, starting positions, and limits. Drag map markers or edit their coordinates."
      onClose={onClose}
    >
      <form
        onSubmit={async (event) => {
          event.preventDefault();
          setBusy(true);
          setError("");
          try {
            await api(`/scenarios${scenarioId ? "/" + scenarioId : ""}`, {
              method: scenarioId ? "PUT" : "POST",
              body: JSON.stringify(draft),
            });
            onSave();
          } catch (reason) {
            setError((reason as Error).message);
          } finally {
            setBusy(false);
          }
        }}
      >
        <div className="editor-grid">
          <div className="editor-fields">
            <label className="input-label">
              Scenario name
              <input
                required
                maxLength={100}
                value={draft.name}
                onChange={(event) =>
                  setDraft({ ...draft, name: event.target.value })
                }
              />
            </label>
            <label className="input-label">
              Notes
              <textarea
                rows={2}
                value={draft.description}
                maxLength={2000}
                onChange={(event) =>
                  setDraft({ ...draft, description: event.target.value })
                }
                placeholder="What would you like to compare?"
              />
            </label>
            <h3>Field & goal</h3>
            <div className="form-grid">
              {(["width_m", "height_m", "center_x", "center_y"] as const).map(
                (key) => (
                  <Numeric
                    key={key}
                    label={
                      {
                        width_m: "Width (m)",
                        height_m: "Height (m)",
                        center_x: "Center X (m)",
                        center_y: "Center Y (m)",
                      }[key]
                    }
                    value={draft.map[key]}
                    min={key.includes("_m") ? 0.1 : undefined}
                    onChange={(value) =>
                      setDraft({
                        ...draft,
                        map: { ...draft.map, [key]: value },
                      })
                    }
                  />
                ),
              )}
              <Numeric
                label="Goal X (m)"
                value={draft.goal.x}
                onChange={(x) =>
                  setDraft({ ...draft, goal: { ...draft.goal, x } })
                }
              />
              <Numeric
                label="Goal Y (m)"
                value={draft.goal.y}
                onChange={(y) =>
                  setDraft({ ...draft, goal: { ...draft.goal, y } })
                }
              />
              <Numeric
                label="Goal radius (m)"
                value={draft.goal.radius_m}
                min={0.1}
                onChange={(radius_m) =>
                  setDraft({ ...draft, goal: { ...draft.goal, radius_m } })
                }
              />
              <Numeric
                label="Completion radius (m)"
                value={completionRadius(draft.goal)}
                min={0.1}
                onChange={(completion_radius_m) =>
                  setDraft({
                    ...draft,
                    goal: { ...draft.goal, completion_radius_m },
                  })
                }
              />
              <Numeric
                label="Extended goal radius (m)"
                value={extendedRadius(draft.goal)}
                min={draft.goal.radius_m}
                onChange={(extended_radius_m) =>
                  setDraft({
                    ...draft,
                    goal: { ...draft.goal, extended_radius_m },
                  })
                }
              />
              <Numeric
                label="Limit (simulated minutes)"
                value={draft.duration_s / 60}
                min={0.01}
                step="any"
                onChange={(value) =>
                  setDraft({ ...draft, duration_s: Math.min(1200, value * 60) })
                }
              />
            </div>
            <p className="hint">
              Completion requires all cows inside the completion radius. Cows
              inside the goal radius stop generating pushing goals; a new
              arrival extends that area for 20 seconds. The extended radius must
              exceed the goal radius.
            </p>
            <h3>Speed limits</h3>
            <div className="form-grid">
              {(Object.keys(draft.speeds) as (keyof Scenario["speeds"])[]).map(
                (key) => (
                  <Numeric
                    key={key}
                    label={
                      {
                        cow_linear_mps: "Cows · linear (m/s)",
                        cow_angular_radps: "Cows · angular (rad/s)",
                        drone_linear_mps: "Drones · linear (m/s)",
                        drone_angular_radps: "Drones · angular (rad/s)",
                      }[key]
                    }
                    value={draft.speeds[key]}
                    min={0.01}
                    onChange={(value) =>
                      setDraft({
                        ...draft,
                        speeds: { ...draft.speeds, [key]: value },
                      })
                    }
                  />
                ),
              )}
            </div>
            <h3>Live simulation windows</h3>
            <ViewSettings
              settings={{ ...emptyVisualizations, ...draft.visualization }}
              onChange={(visualization) =>
                setDraft({ ...draft, visualization })
              }
            />
            <p className="hint">
              Choose windows to open on the simulator desktop. You can also
              change them while a test runs.
            </p>
            <p className="hint">
              Success ends a test early. The maximum is 20 simulated minutes.
            </p>
          </div>
          <div className="editor-map">
            <MapCanvas edit scenario={draft} onMove={move} />
            <div className="hint">
              <MapPin size={13} /> Drag D markers, C markers, or the goal to set
              positions.
            </div>
            <div className="positions-grid">
              {positions("drones")}
              {positions("cows")}
            </div>
          </div>
        </div>
        {error && (
          <div className="error" role="alert">
            {error}
          </div>
        )}
        <div className="modal-footer">
          <button type="button" className="button" onClick={onClose}>
            Cancel
          </button>
          <button className="button primary" disabled={busy}>
            <Check size={16} />
            {busy ? "Saving…" : "Save scenario"}
          </button>
        </div>
      </form>
    </Modal>
  );
}
function BatchEditor({
  scenarios,
  onClose,
  onSave,
}: {
  scenarios: SavedScenario[];
  onClose: () => void;
  onSave: () => void;
}) {
  const [name, setName] = useState("Experiment batch"),
    [items, setItems] = useState([
      { scenario_id: scenarios[0]?.id || "", repeats: 5 },
    ]),
    [error, setError] = useState(""),
    [busy, setBusy] = useState(false);
  const count = items.reduce((sum, item) => sum + item.repeats, 0),
    limit = items.reduce(
      (sum, item) =>
        sum +
        item.repeats *
          (scenarios.find((s) => s.id === item.scenario_id)?.duration_s || 0),
      0,
    );
  return (
    <Modal
      title="Queue a test battery"
      subtitle="Scenarios run sequentially in fresh, isolated simulations."
      onClose={onClose}
    >
      <form
        onSubmit={async (event) => {
          event.preventDefault();
          setBusy(true);
          setError("");
          try {
            await api("/batches", {
              method: "POST",
              body: JSON.stringify({ version: 1, name, items }),
            });
            onSave();
          } catch (reason) {
            setError((reason as Error).message);
          } finally {
            setBusy(false);
          }
        }}
      >
        <label className="input-label">
          Batch name
          <input
            value={name}
            required
            maxLength={100}
            onChange={(event) => setName(event.target.value)}
          />
        </label>
        <div className="batch-items">
          {items.map((item, index) => (
            <div className="batch-item" key={index}>
              <label className="input-label">
                Scenario
                <select
                  value={item.scenario_id}
                  required
                  onChange={(event) =>
                    setItems(
                      items.map((v, i) =>
                        i === index
                          ? { ...v, scenario_id: event.target.value }
                          : v,
                      ),
                    )
                  }
                >
                  {scenarios.map((scenario) => (
                    <option key={scenario.id} value={scenario.id}>
                      {scenario.name}
                    </option>
                  ))}
                </select>
              </label>
              <Numeric
                label="Repeats"
                value={item.repeats}
                min={1}
                step={1}
                onChange={(repeats) =>
                  setItems(
                    items.map((v, i) => (i === index ? { ...v, repeats } : v)),
                  )
                }
              />
              <button
                type="button"
                className="icon-button"
                aria-label={`Remove scenario ${index + 1} from batch`}
                disabled={items.length === 1}
                onClick={() => setItems(items.filter((_, i) => i !== index))}
              >
                <Trash2 size={16} />
              </button>
            </div>
          ))}
        </div>
        <button
          type="button"
          className="text-button"
          onClick={() =>
            setItems([
              ...items,
              { scenario_id: scenarios[0]?.id || "", repeats: 1 },
            ])
          }
        >
          <Plus size={15} />
          Add another scenario
        </button>
        <div className="batch-summary">
          <FlaskConical size={24} />
          <div>
            <strong>
              {count} test{count !== 1 ? "s" : ""} · up to{" "}
              {(limit / 60).toFixed(1)} simulated minutes
            </strong>
            <p>
              Each test ends at success or its time limit. Real duration depends
              on simulation speed.
            </p>
          </div>
        </div>
        {error && (
          <div className="error" role="alert">
            {error}
          </div>
        )}
        <div className="modal-footer">
          <button type="button" className="button" onClick={onClose}>
            Cancel
          </button>
          <button
            className="button primary"
            disabled={busy || !scenarios.length}
          >
            <Play size={15} />
            {busy ? "Preparing snapshots…" : "Start batch"}
          </button>
        </div>
      </form>
    </Modal>
  );
}
function Compare({ runs, onClose }: { runs: Run[]; onClose: () => void }) {
  const [summaries, setSummaries] = useState<Record<string, Summary>>({}),
    [error, setError] = useState("");
  useEffect(() => {
    Promise.all(
      runs.map(
        async (run) =>
          [run.id, await api<Summary>(`/runs/${run.id}/summary`)] as const,
      ),
    )
      .then((values) => setSummaries(Object.fromEntries(values)))
      .catch((reason) => setError(reason.message));
  }, [runs]);
  return (
    <Modal
      wide
      title="Compare experiments"
      subtitle="Configuration limits and measured results are saved independently for every run."
      onClose={onClose}
    >
      {error && <div className="error">{error}</div>}
      <div className="table-scroll">
        <table className="comparison">
          <thead>
            <tr>
              <th>Measurement</th>
              {runs.map((run) => (
                <th key={run.id}>
                  {run.scenario.name}
                  <small>
                    #{run.id.slice(0, 8)} · repeat {run.repetition}
                  </small>
                </th>
              ))}
            </tr>
          </thead>
          <tbody>
            {[
              ["Status", (run: Run) => statusText(run.status)],
              [
                "Simulation time",
                (run: Run) => clock(run.simulation_duration_s),
              ],
              ["Real elapsed", (run: Run) => clock(run.real_duration_s)],
              [
                "Cow limit",
                (run: Run) => `${run.scenario.speeds.cow_linear_mps} m/s`,
              ],
              [
                "Drone limit",
                (run: Run) => `${run.scenario.speeds.drone_linear_mps} m/s`,
              ],
              [
                "Cows at goal",
                (run: Run) =>
                  `${summaries[run.id]?.cows_at_goal ?? "—"} / ${run.scenario.cows.length}`,
              ],
              [
                "Cow distance · XY",
                (run: Run) => `${fixed(summaries[run.id]?.distance_cows_m)} m`,
              ],
              [
                "Drone distance · XY",
                (run: Run) =>
                  `${fixed(summaries[run.id]?.distance_drones_m)} m`,
              ],
            ].map(([name, value]) => (
              <tr key={name as string}>
                <td>{name as string}</td>
                {runs.map((run) => (
                  <td key={run.id}>{(value as (r: Run) => string)(run)}</td>
                ))}
              </tr>
            ))}
          </tbody>
        </table>
      </div>
      {runs.map((run) => (
        <section className="compare-entities" key={run.id}>
          <h3>
            {run.scenario.name} <small>#{run.id.slice(0, 8)}</small>
          </h3>
          <MetricTable entities={summaries[run.id]?.entities || []} />
        </section>
      ))}
    </Modal>
  );
}
export default function App() {
  const [page, setPage] = useState("runs"),
    [runs, setRuns] = useState<Run[]>([]),
    [scenarios, setScenarios] = useState<SavedScenario[]>([]),
    [batches, setBatches] = useState<Batch[]>([]),
    [defaults, setDefaults] = useState<Scenario | null>(null),
    [selected, setSelected] = useState<string | null>(null),
    [editor, setEditor] = useState<{ initial: Scenario; id?: string } | null>(
      null,
    ),
    [batchEditor, setBatchEditor] = useState(false),
    [compare, setCompare] = useState(false),
    [checked, setChecked] = useState<string[]>([]),
    [search, setSearch] = useState(""),
    [filter, setFilter] = useState("all"),
    [error, setError] = useState(""),
    [connected, setConnected] = useState(false);
  const refresh = useCallback(async () => {
    try {
      const [newRuns, newScenarios, newBatches] = await Promise.all([
        api<Run[]>("/runs"),
        api<SavedScenario[]>("/scenarios"),
        api<Batch[]>("/batches"),
      ]);
      setRuns(newRuns);
      setChecked((previous) =>
        previous.filter((id) => newRuns.some((run) => run.id === id)),
      );
      setScenarios(newScenarios);
      setBatches(newBatches);
      setConnected(true);
      setError("");
      setSelected((previous) =>
        previous && newRuns.some((r) => r.id === previous)
          ? previous
          : newRuns[0]?.id || null,
      );
    } catch (reason) {
      setError((reason as Error).message);
      setConnected(false);
    }
  }, []);
  useEffect(() => {
    refresh();
    api<Scenario>("/defaults")
      .then(setDefaults)
      .catch((reason) => setError(reason.message));
    const interval = setInterval(refresh, 1000);
    return () => clearInterval(interval);
  }, [refresh]);
  const selectedRun = runs.find((run) => run.id === selected),
    active = runs.find((run) => ["starting", "running"].includes(run.status)),
    completed = runs.filter((run) => run.status === "success").length;
  const newScenario = () => {
    if (defaults)
      setEditor({
        initial: {
          ...scenarioOnly(defaults),
          name: scenarios.length ? "New scenario" : defaults.name,
        },
      });
  };
  const visible = runs.filter(
    (run) =>
      (filter === "all" || run.status === filter) &&
      (run.scenario.name + " " + run.batch_name + " " + run.id)
        .toLowerCase()
        .includes(search.toLowerCase()),
  );
  const deleteExperiments = async (ids: string[]) => {
    const selectedRuns = runs.filter((run) => ids.includes(run.id));
    if (
      selectedRuns.some((run) => ["starting", "running"].includes(run.status))
    ) {
      setError("Cancel or finish running experiments before deleting them.");
      return;
    }
    if (
      !ids.length ||
      !window.confirm(
        `Permanently delete ${ids.length} experiment${ids.length === 1 ? "" : "s"} and all their recordings, configurations, metrics and logs? Saved scenarios will remain.`,
      )
    )
      return;
    try {
      await api("/runs/delete", {
        method: "POST",
        body: JSON.stringify({ run_ids: ids }),
      });
      setChecked((previous) => previous.filter((id) => !ids.includes(id)));
      if (selected && ids.includes(selected)) setSelected(null);
      await refresh();
    } catch (reason) {
      setError((reason as Error).message);
    }
  };
  return (
    <div className="app-shell">
      <aside className="sidebar">
        <a
          className="brand"
          href="#"
          onClick={(event) => {
            event.preventDefault();
            setPage("runs");
          }}
        >
          <div className="brand-icon">
            <FlaskConical size={23} />
          </div>
          <div>
            Herding Lab<span>SIMULATION WORKSPACE</span>
          </div>
        </a>
        <div className="workspace-label">EXPERIMENTS</div>
        <nav>
          <button
            className={page === "runs" ? "active" : ""}
            onClick={() => setPage("runs")}
          >
            <LayoutDashboard size={18} />
            Runs & replay<span>{runs.length}</span>
          </button>
          <button
            className={page === "scenarios" ? "active" : ""}
            onClick={() => setPage("scenarios")}
          >
            <MapPin size={18} />
            Scenarios<span>{scenarios.length}</span>
          </button>
          <button
            className={page === "queue" ? "active" : ""}
            onClick={() => setPage("queue")}
          >
            <Layers size={18} />
            Batch queue
            <span>{runs.filter((r) => r.status === "queued").length}</span>
          </button>
        </nav>
        <div className="sidebar-card">
          <Activity size={18} />
          <strong>
            {active ? "Experiment in progress" : "Ready to experiment"}
          </strong>
          <p>
            {active
              ? `${active.scenario.name} · ${clock(active.simulation_duration_s)} simulated`
              : "Design a scenario, queue a battery, and explore the results."}
          </p>
          {active && (
            <button
              onClick={() => {
                setPage("runs");
                setSelected(active.id);
              }}
            >
              View live run <ChevronRight size={14} />
            </button>
          )}
        </div>
        <div className="sidebar-footer">
          <span className={connected ? "connection online" : "connection"} />
          {connected ? "Local runner connected" : "Connecting to local runner"}
          <small>Data stays on this computer</small>
        </div>
      </aside>
      <main>
        <header className="topbar">
          <div>
            <div className="breadcrumb">
              Workspace <ChevronRight size={13} />{" "}
              {page === "runs"
                ? "Experiments"
                : page === "scenarios"
                  ? "Scenarios"
                  : "Batch queue"}
            </div>
            <h1>
              {page === "runs"
                ? "Runs & replay"
                : page === "scenarios"
                  ? "Scenario library"
                  : "Batch queue"}
            </h1>
            <p>
              {page === "runs"
                ? "Every experiment, from the first movement to the final goal."
                : page === "scenarios"
                  ? "Define repeatable starting layouts and compare speed settings."
                  : "Run your experiments sequentially. Each starts with a fresh world."}
            </p>
          </div>
          <div className="header-actions">
            <button
              className="button"
              onClick={newScenario}
              disabled={!defaults}
            >
              <Plus size={16} />
              New scenario
            </button>
            <button
              className="button primary"
              onClick={() => setBatchEditor(true)}
              disabled={!scenarios.length}
            >
              <Play size={15} />
              New batch
            </button>
          </div>
        </header>
        {error && (
          <div className="error page-error" role="alert">
            {error}
          </div>
        )}
        {page === "runs" && (
          <>
            <div className="workspace-stats">
              <div>
                <span className="stat-icon blue">
                  <FlaskConical size={19} />
                </span>
                <div>
                  <strong>{runs.length}</strong>
                  <span>Total runs</span>
                </div>
              </div>
              <div>
                <span className="stat-icon green">
                  <Check size={19} />
                </span>
                <div>
                  <strong>{completed}</strong>
                  <span>Completed successfully</span>
                </div>
              </div>
              <div>
                <span className="stat-icon amber">
                  <Clock3 size={19} />
                </span>
                <div>
                  <strong>
                    {runs.filter((run) => run.status === "queued").length}
                  </strong>
                  <span>In the queue</span>
                </div>
              </div>
              <div>
                <span className="stat-icon purple">
                  <TrendingUp size={19} />
                </span>
                <div>
                  <strong>
                    {runs.filter((run) => terminal(run.status)).length
                      ? Math.round(
                          (completed /
                            runs.filter((run) => terminal(run.status)).length) *
                            100,
                        )
                      : 0}
                    %
                  </strong>
                  <span>Success rate</span>
                </div>
              </div>
            </div>
            <div className="experiments-layout">
              <section className="run-list">
                <div className="list-heading">
                  <h3>
                    Experiments <span>{visible.length}</span>
                  </h3>
                  <button
                    className="icon-button"
                    aria-label="Refresh experiments"
                    onClick={refresh}
                  >
                    <RefreshCw size={15} />
                  </button>
                </div>
                <label className="search">
                  <Search size={15} />
                  <input
                    placeholder="Search runs…"
                    aria-label="Search runs"
                    value={search}
                    onChange={(event) => setSearch(event.target.value)}
                  />
                </label>
                <div className="run-list-tools">
                  <select
                    aria-label="Filter run status"
                    value={filter}
                    onChange={(event) => setFilter(event.target.value)}
                  >
                    <option value="all">All statuses</option>
                    {[
                      "success",
                      "running",
                      "queued",
                      "timed_out",
                      "failed",
                      "cancelled",
                      "interrupted",
                    ].map((s) => (
                      <option key={s} value={s}>
                        {statusText(s)}
                      </option>
                    ))}
                  </select>
                  <button
                    className="text-button"
                    disabled={checked.length < 2}
                    onClick={() => setCompare(true)}
                  >
                    Compare {checked.length ? `(${checked.length})` : ""}
                  </button>
                </div>
                <div className="run-selection-tools">
                  <label>
                    <input
                      type="checkbox"
                      aria-label="Select all filtered experiments"
                      disabled={!visible.length}
                      checked={
                        visible.length > 0 &&
                        visible.every((run) => checked.includes(run.id))
                      }
                      onChange={(event) =>
                        setChecked(
                          event.target.checked
                            ? [
                                ...new Set([
                                  ...checked,
                                  ...visible.map((r) => r.id),
                                ]),
                              ]
                            : checked.filter(
                                (id) => !visible.some((r) => r.id === id),
                              ),
                        )
                      }
                    />
                    Select all
                  </label>
                  <button
                    className="text-button danger"
                    disabled={
                      !checked.length ||
                      runs.some(
                        (r) =>
                          checked.includes(r.id) &&
                          ["starting", "running"].includes(r.status),
                      )
                    }
                    onClick={() => deleteExperiments(checked)}
                  >
                    <Trash2 size={13} />
                    Delete selected{" "}
                    {checked.length ? `(${checked.length})` : ""}
                  </button>
                </div>
                <div className="run-items">
                  {visible.map((run) => (
                    <div
                      key={run.id}
                      className={
                        "run-item " + (run.id === selected ? "selected" : "")
                      }
                    >
                      <input
                        type="checkbox"
                        aria-label={`Select ${run.scenario.name} ${run.id.slice(0, 8)} for comparison or deletion`}
                        checked={checked.includes(run.id)}
                        onChange={(event) =>
                          setChecked(
                            event.target.checked
                              ? [...checked, run.id]
                              : checked.filter((id) => id !== run.id),
                          )
                        }
                      />
                      <button onClick={() => setSelected(run.id)}>
                        <div>
                          <strong>{run.scenario.name}</strong>
                          <ChevronRight size={15} />
                        </div>
                        <p>
                          #{run.id.slice(0, 8)} · repeat {run.repetition}
                        </p>
                        <div className="run-item-bottom">
                          <Badge status={run.status} />
                          <span>{clock(run.simulation_duration_s)}</span>
                        </div>
                      </button>
                    </div>
                  ))}
                </div>
                {!runs.length && (
                  <div className="list-empty">
                    <FlaskConical size={28} />
                    <h3>Your first experiment starts here</h3>
                    <p>
                      Create a scenario, then queue a batch to start recording.
                    </p>
                    <button className="button" onClick={newScenario}>
                      Create scenario
                    </button>
                  </div>
                )}
              </section>
              {selectedRun ? (
                <Replay
                  key={selectedRun.id}
                  run={selectedRun}
                  refresh={refresh}
                  onDelete={(run) => deleteExperiments([run.id])}
                  onClone={(initial) => setEditor({ initial })}
                />
              ) : (
                <section className="welcome">
                  <div className="welcome-orbit">
                    <MapPin size={46} />
                  </div>
                  <div className="eyebrow">YOUR LOCAL EXPERIMENT ARCHIVE</div>
                  <h2>Test. Observe. Compare.</h2>
                  <p>
                    Record every cow and drone, replay any moment, and discover
                    what changes when you adjust your simulation.
                  </p>
                  <button
                    className="button primary"
                    onClick={
                      scenarios.length
                        ? () => setBatchEditor(true)
                        : newScenario
                    }
                  >
                    <Plus size={16} />
                    {scenarios.length
                      ? "Queue your first battery"
                      : "Create your first scenario"}
                  </button>
                  <div className="welcome-features">
                    <span>
                      <Clock3 size={16} />
                      20-minute simulation limit
                    </span>
                    <span>
                      <MapPin size={16} />
                      Interactive trajectory replay
                    </span>
                    <span>
                      <TrendingUp size={16} />
                      Per-entity movement metrics
                    </span>
                  </div>
                </section>
              )}
            </div>
          </>
        )}
        {page === "scenarios" && (
          <section className="scenario-library">
            {scenarios.map((scenario) => (
              <article className="scenario-card" key={scenario.id}>
                <div className="scenario-thumbnail">
                  <MapCanvas edit scenario={scenario} />
                </div>
                <div className="scenario-info">
                  <h3>{scenario.name}</h3>
                  <p>
                    {scenario.description ||
                      `${scenario.drones.length} drones · ${scenario.cows.length} cows · ${scenario.map.width_m} × ${scenario.map.height_m} m field`}
                  </p>
                  <div className="scenario-chips">
                    <span>Cows {scenario.speeds.cow_linear_mps} m/s</span>
                    <span>Drones {scenario.speeds.drone_linear_mps} m/s</span>
                    <span>{clock(scenario.duration_s)} limit</span>
                  </div>
                  <div className="scenario-actions">
                    <button
                      className="text-button"
                      onClick={() =>
                        setEditor({ initial: scenario, id: scenario.id })
                      }
                    >
                      <Settings2 size={15} />
                      Edit
                    </button>
                    <button
                      className="text-button"
                      onClick={() =>
                        setEditor({
                          initial: {
                            ...scenarioOnly(scenario),
                            name: scenario.name + " copy",
                          },
                        })
                      }
                    >
                      <Copy size={14} />
                      Clone
                    </button>
                    <div className="spacer" />
                    <button
                      className="icon-button danger"
                      aria-label={`Delete ${scenario.name}`}
                      onClick={async () => {
                        if (
                          !window.confirm(
                            `Delete scenario “${scenario.name}”? Recorded runs will remain.`,
                          )
                        )
                          return;
                        try {
                          await api(`/scenarios/${scenario.id}`, {
                            method: "DELETE",
                          });
                          refresh();
                        } catch (reason) {
                          setError((reason as Error).message);
                        }
                      }}
                    >
                      <Trash2 size={15} />
                    </button>
                  </div>
                </div>
              </article>
            ))}
            <button className="add-scenario" onClick={newScenario}>
              <Plus size={28} />
              <strong>Create a scenario</strong>
              <span>Define a layout worth testing</span>
            </button>
          </section>
        )}
        {page === "queue" && (
          <section className="batch-list">
            {batches.length ? (
              batches.map((batch) => (
                <article className="batch-card" key={batch.id}>
                  <div className="batch-card-heading">
                    <div>
                      <div className="eyebrow">
                        BATCH · {new Date(batch.created).toLocaleString()}
                      </div>
                      <h2>{batch.name}</h2>
                      <p>
                        {
                          batch.runs.filter((run) => terminal(run.status))
                            .length
                        }{" "}
                        of {batch.runs.length} finished
                      </p>
                    </div>
                    {batch.status !== "finished" && (
                      <button
                        className="button danger"
                        onClick={async () => {
                          try {
                            await api(`/batches/${batch.id}/cancel`, {
                              method: "POST",
                            });
                            refresh();
                          } catch (reason) {
                            setError((reason as Error).message);
                          }
                        }}
                      >
                        <Square size={13} />
                        Cancel batch
                      </button>
                    )}
                  </div>
                  <div className="batch-progress">
                    <div
                      style={{
                        width: `${(batch.runs.filter((run) => terminal(run.status)).length / batch.runs.length) * 100}%`,
                      }}
                    />
                  </div>
                  {batch.runs.map((run) => (
                    <button
                      className="queue-row"
                      key={run.id}
                      onClick={() => {
                        setSelected(run.id);
                        setPage("runs");
                      }}
                    >
                      <span className="queue-number">
                        {batch.runs.indexOf(run) + 1}
                      </span>
                      <div>
                        <strong>{run.scenario.name}</strong>
                        <small>
                          Repeat {run.repetition} · {run.scenario.cows.length}{" "}
                          cows · {run.scenario.drones.length} drones
                        </small>
                      </div>
                      <div className="spacer" />
                      <span>
                        {clock(run.simulation_duration_s)} /{" "}
                        {clock(run.scenario.duration_s)}
                      </span>
                      <Badge status={run.status} />
                      <ChevronRight size={16} />
                    </button>
                  ))}
                </article>
              ))
            ) : (
              <div className="welcome">
                <Layers size={36} />
                <h2>Your queue is clear</h2>
                <p>Choose a saved scenario and repeat it to compare results.</p>
                <button
                  className="button primary"
                  disabled={!scenarios.length}
                  onClick={() => setBatchEditor(true)}
                >
                  Create a batch
                </button>
              </div>
            )}
          </section>
        )}
      </main>
      {editor && (
        <ScenarioEditor
          initial={editor.initial}
          scenarioId={editor.id}
          onClose={() => setEditor(null)}
          onSave={() => {
            setEditor(null);
            refresh();
          }}
        />
      )}
      {batchEditor && (
        <BatchEditor
          scenarios={scenarios}
          onClose={() => setBatchEditor(false)}
          onSave={() => {
            setBatchEditor(false);
            setPage("queue");
            refresh();
          }}
        />
      )}
      {compare && (
        <Compare
          runs={runs.filter((run) => checked.includes(run.id))}
          onClose={() => setCompare(false)}
        />
      )}
    </div>
  );
}
