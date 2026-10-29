import { useEffect, useState } from "react";
import {
  api,
  Run,
  Visualizations,
  LiveState,
  emptyVisualizations,
  visualizationLabels,
  terminal,
} from "./types";
export function ViewSettings({
  settings,
  onChange,
  disabled = false,
}: {
  settings: Visualizations;
  onChange: (v: Visualizations) => void;
  disabled?: boolean;
}) {
  return (
    <div className="view-settings">
      {(Object.keys(visualizationLabels) as (keyof Visualizations)[]).map(
        (key) => (
          <label key={key}>
            <input
              type="checkbox"
              disabled={disabled}
              checked={settings[key]}
              onChange={(event) =>
                onChange({ ...settings, [key]: event.target.checked })
              }
            />
            {visualizationLabels[key]}
          </label>
        ),
      )}
    </div>
  );
}
export function LiveControls({ run }: { run: Run }) {
  const [state, setState] = useState<LiveState | null>(null),
    [busy, setBusy] = useState(false),
    [error, setError] = useState("");
  useEffect(() => {
    let active = true;
    const load = () =>
      api<LiveState>(`/runs/${run.id}/visualizations`)
        .then((value) => {
          if (active) setState(value);
        })
        .catch((reason) => {
          if (active) setError(reason.message);
        });
    load();
    const timer = setInterval(load, 1000);
    return () => {
      active = false;
      clearInterval(timer);
    };
  }, [run.id]);
  if (terminal(run.status)) return null;
  return (
    <details className="live-controls">
      <summary>
        Live simulation windows{" "}
        <span>
          {state?.applied
            ? "Applied"
            : state?.available
              ? "Applying…"
              : "Waiting"}
        </span>
      </summary>
      <div className="live-control-body">
        {state?.available ? (
          <>
            <ViewSettings
              settings={state.settings}
              disabled={busy}
              onChange={async (settings) => {
                setBusy(true);
                setError("");
                setState({ ...state, settings, applied: false });
                try {
                  setState(
                    await api<LiveState>(`/runs/${run.id}/visualizations`, {
                      method: "POST",
                      body: JSON.stringify(settings),
                    }),
                  );
                } catch (reason) {
                  setError((reason as Error).message);
                } finally {
                  setBusy(false);
                }
              }}
            />
            <p>
              {state.virtual_display
                ? "This runner uses a virtual display. Windows are rendered there; the web replay stays available."
                : `Windows appear on the simulator desktop${state.display ? " (" + state.display + ")" : ""}. You can change these without stopping the test.`}
            </p>
            {state.errors.map((message, index) => (
              <div className="error" key={index}>
                {message}
              </div>
            ))}
          </>
        ) : (
          <p>
            Live window controls are available for tests started with the
            updated runner. Save defaults in the scenario editor.
          </p>
        )}
        {error && (
          <div className="error" role="alert">
            {error}
          </div>
        )}
      </div>
    </details>
  );
}
