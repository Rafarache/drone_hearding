import { useEffect, useMemo, useState } from "react";
import { api, Run, SearchHistory, label, terminal } from "./types";
import { searchAtTime } from "./replay";
export const emptySearch: SearchHistory = { available: false, drones: [] };
export function useSearchHistory(run: Run) {
  const [data, setData] = useState<SearchHistory>(emptySearch);
  const [error, setError] = useState("");
  const duration = run.simulation_duration_s || 0;
  useEffect(() => {
    let active = true;
    api<SearchHistory>(`/runs/${run.id}/search?end=${duration}`)
      .then((value) => {
        if (active) {
          setData(value);
          setError("");
        }
      })
      .catch((reason) => {
        if (active) setError(reason.message);
      });
    return () => {
      active = false;
    };
  }, [run.id, run.status, duration]);
  return { data, error };
}
export function SearchControls({
  history,
  time,
  selected,
  setSelected,
  searched,
  setSearched,
  unsearched,
  setUnsearched,
  goals,
  setGoals,
  run,
}: {
  history: SearchHistory;
  time: number;
  selected: string[];
  setSelected: (v: string[]) => void;
  searched: boolean;
  setSearched: (v: boolean) => void;
  unsearched: boolean;
  setUnsearched: (v: boolean) => void;
  goals: boolean;
  setGoals: (v: boolean) => void;
  run: Run;
}) {
  const all = run.entities.filter((e) => e.kind === "drone");
  const current = useMemo(
    () =>
      searchAtTime(
        history,
        time,
        all.map((e) => e.name),
      ),
    [history, time, run.entities],
  );
  return (
    <details className="search-controls">
      <summary>
        Search map & goals{" "}
        <span>{history.available ? "Timeline layers" : "Not recorded"}</span>
      </summary>
      <div className="search-control-body">
        {history.available ? (
          <>
            <div className="layer-toggles">
              <label>
                <input
                  type="checkbox"
                  checked={searched}
                  onChange={(e) => setSearched(e.target.checked)}
                />
                Searched areas
              </label>
              <label>
                <input
                  type="checkbox"
                  checked={unsearched}
                  onChange={(e) => setUnsearched(e.target.checked)}
                />
                Unsearched areas
              </label>
              <label>
                <input
                  type="checkbox"
                  checked={goals}
                  onChange={(e) => setGoals(e.target.checked)}
                />
                Search goals
              </label>
            </div>
            <div className="drone-layer-list">
              {all.map((e) => {
                const state = current.find((d) => d.drone === e.name);
                return (
                  <label key={e.name}>
                    <input
                      type="checkbox"
                      checked={selected.includes(e.name)}
                      onChange={(event) =>
                        setSelected(
                          event.target.checked
                            ? [...selected, e.name]
                            : selected.filter((n) => n !== e.name),
                        )
                      }
                    />
                    <strong>{label(e.name, "drone")}</strong>
                    <span>
                      {state
                        ? `${state.percent.toFixed(1)}% searched`
                        : "No state at this time"}
                    </span>
                  </label>
                );
              })}
            </div>
            <p>
              Areas show the combined recorded knowledge of selected drones,
              including received peer observations. Goal diamonds mark viewing
              positions; dashed boxes mark search patches.
            </p>
          </>
        ) : (
          <p>
            {terminal(run.status)
              ? "This experiment was recorded before search-state recording was available. New tests record this layer."
              : "Waiting for search states from the drones."}
          </p>
        )}
      </div>
    </details>
  );
}
