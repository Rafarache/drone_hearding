import { useEffect, useRef, useState } from "react";
import { Hammer, X } from "lucide-react";
import { api } from "./types";

type BuildState = {
  available: boolean;
  status: string;
  id?: string;
  message?: string;
  started_at?: string;
  finished_at?: string;
  code?: { branch: string; revision: string; dirty: boolean };
  log?: string;
};

export function BuildControls({
  queued,
  onBlocked,
}: {
  queued: boolean;
  onBlocked: (blocked: boolean) => void;
}) {
  const [state, setState] = useState<BuildState | null>(null);
  const [open, setOpen] = useState(false);
  const [starting, setStarting] = useState(false);
  const [error, setError] = useState("");
  const [reconnecting, setReconnecting] = useState(false);
  const dialog = useRef<HTMLDialogElement>(null);
  const expected = useRef<string | null>(sessionStorage.getItem("lab-build"));
  const busy =
    starting || ["building", "restarting"].includes(state?.status || "");

  useEffect(() => {
    let active = true;
    const poll = async () => {
      try {
        const next = await api<BuildState>("/build");
        if (!active) return;
        setState(next);
        setReconnecting(false);
        onBlocked(["building", "restarting", "failed"].includes(next.status));
        if (next.id && ["building", "restarting"].includes(next.status)) {
          expected.current = next.id;
          sessionStorage.setItem("lab-build", next.id);
        }
        if (next.status === "succeeded" && next.id === expected.current) {
          sessionStorage.removeItem("lab-build");
          expected.current = null;
          // Fetch the newly built HTML, scripts, defaults and backend state.
          window.location.reload();
        }
        if (next.status === "failed") {
          sessionStorage.removeItem("lab-build");
          expected.current = null;
        }
      } catch {
        if (active) setReconnecting(true);
      }
    };
    void poll();
    const interval = setInterval(poll, 1000);
    return () => {
      active = false;
      clearInterval(interval);
    };
  }, [onBlocked]);

  useEffect(() => {
    if (open) dialog.current?.showModal();
    else dialog.current?.close();
  }, [open]);

  const start = async () => {
    setStarting(true);
    setError("");
    try {
      const next = await api<BuildState>("/build", { method: "POST" });
      setState(next);
      onBlocked(true);
      if (next.id) {
        expected.current = next.id;
        sessionStorage.setItem("lab-build", next.id);
      }
    } catch (reason) {
      setError((reason as Error).message);
    } finally {
      setStarting(false);
    }
  };

  return (
    <>
      <button className="button secondary" onClick={() => setOpen(true)}>
        <Hammer size={16} />
        {busy
          ? "Building…"
          : state?.status === "failed"
            ? "Build failed"
            : "Build latest code"}
      </button>
      <dialog
        ref={dialog}
        className="modal wide"
        aria-label="Build latest code"
        onCancel={() => setOpen(false)}
      >
        <div className="modal-heading">
          <div>
            <h2>Build latest code</h2>
            <p>Build the current local checkout and load it into the lab.</p>
          </div>
          <button
            className="icon-button"
            aria-label="Close build dialog"
            onClick={() => setOpen(false)}
          >
            <X size={20} />
          </button>
        </div>
        <p>
          Rebuilds all ROS packages and the website, then restarts the lab
          backend. The page reloads when the new build is ready. Local edits are
          included.
        </p>
        <p className="hint">
          This uses the branch checked out in your workspace. It does not
          download or switch branches.
        </p>
        {state?.code && (
          <p>
            Built checkout:{" "}
            <strong>{state.code.branch || "Detached HEAD"}</strong> ·{" "}
            {state.code.revision.slice(0, 8)}
            {state.code.dirty ? " · includes local edits" : ""}
          </p>
        )}
        <p role="status">
          {reconnecting
            ? "Reconnecting to the lab backend…"
            : state?.message || "Checking build controller…"}
        </p>
        {queued && (
          <div className="error">
            Finish or cancel running and queued experiments before building.
          </div>
        )}
        {state?.status === "failed" && (
          <div className="error">
            The build failed. Fix the error below and retry. New experiments are
            blocked until a build succeeds.
          </div>
        )}
        {error && (
          <div className="error" role="alert">
            {error}
          </div>
        )}
        {state?.log && (
          <pre className="build-log" aria-label="Build output">
            {state.log}
          </pre>
        )}
        <div className="modal-footer">
          <button className="button secondary" onClick={() => setOpen(false)}>
            Close
          </button>
          <button
            className="button primary"
            disabled={!state?.available || busy || queued || reconnecting}
            onClick={start}
          >
            <Hammer size={16} />
            {busy ? "Building…" : "Build and reload"}
          </button>
        </div>
      </dialog>
    </>
  );
}
