import { useEffect, useState, useCallback } from "react";
import { useParams, Link } from "react-router-dom";
import { SpeedChart, TrustChart, PositionChart } from "../components/Charts";
import { TripMap } from "../components/MapView";
import { MetricsBar } from "../components/MetricsBar";
import { Pedal } from "../components/Pedal";
import { runsApi, type Frame } from "../api/client";
import "./TripPage.css";

export default function TripPage() {
  const { id } = useParams<{ id: string }>();
  const [frames, setFrames] = useState<Frame[]>([]);
  const [currentIdx, setCurrentIdx] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);
  const [showGnss, setShowGnss] = useState(true);
  const [showModel, setShowModel] = useState(true);
  const [intervention, setIntervention] = useState(false);
  const [interventionActive, setInterventionActive] = useState(false);
  const [pedalValue, setPedalValue] = useState(0);
  const [loading, setLoading] = useState(true);
  const [err, setErr] = useState("");

  const load = useCallback(async () => {
    if (!id) return;
    setLoading(true);
    try {
      const telemetry = await runsApi.telemetry(id, 0, 1e9, 1);
      setFrames(telemetry);
      setLoading(false);
    } catch (e: any) {
      setErr(e.message);
      setLoading(false);
    }
  }, [id]);

  useEffect(() => { load(); }, [load]);

  useEffect(() => {
    if (!playing || currentIdx >= frames.length - 1) return;
    const dt = (frames[currentIdx + 1]?.t ?? 0.02) - (frames[currentIdx]?.t ?? 0);
    const timer = setTimeout(() => setCurrentIdx((i) => Math.min(i + 1, frames.length - 1)), dt * 1000 / speed);
    return () => clearTimeout(timer);
  }, [playing, currentIdx, frames, speed]);

  const handlePedal = (v: number) => {
    setPedalValue(v);
    if (intervention) {
      const f = frames[currentIdx];
      if (f) {
        const arr = [...frames];
        arr[currentIdx] = { ...f, u: v };
        setFrames(arr);
      }
    }
  };

  if (loading) return <div className="trip">Загрузка…</div>;
  if (err) return <div className="trip err">{err}</div>;

  return (
    <div className="trip">
      <header className="trip-header">
        <Link to="/runs" className="back">← К прогонам</Link>
        <h2>Прогон {id}</h2>
        <div className="controls">
          <button onClick={() => setPlaying(!playing)}>{playing ? "Пауза" : "Старт"}</button>
          <select value={speed} onChange={(e) => setSpeed(Number(e.target.value))}>
            <option value={0.5}>0.5×</option>
            <option value={1}>1×</option>
            <option value={2}>2×</option>
            <option value={4}>4×</option>
          </select>
          <label><input type="checkbox" checked={showGnss} onChange={(e) => setShowGnss(e.target.checked)} /> GNSS</label>
          <label><input type="checkbox" checked={showModel} onChange={(e) => setShowModel(e.target.checked)} /> Модель</label>
          <button onClick={() => setCurrentIdx(0)}>Сброс</button>
          <button className={intervention ? "active" : ""} onClick={() => setIntervention(!intervention)}>
            {intervention ? "Рулю" : "Смотрю"}
          </button>
          <Link to={`/runs/${id}/report`} style={{ marginLeft: 12, textDecoration: "none", color: "#0066cc" }}>Отчёт</Link>
        </div>
      </header>

      <main className="trip-main">
        <aside className="charts">
          <SpeedChart frames={frames} />
          <TrustChart frames={frames} />
          <PositionChart frames={frames} />
        </aside>
        <section className="map-area">
          <TripMap
            frames={frames}
            currentIdx={currentIdx}
            showGnss={showGnss}
            showModel={showModel}
          />
        </section>
      </main>

      <MetricsBar frames={frames} currentIdx={currentIdx} intervention={interventionActive} />

      {intervention && (
        <div className="pedal-overlay">
          <Pedal
            value={pedalValue}
            onChange={handlePedal}
            intervention={true}
            interventionActive={interventionActive}
          />
          <button
            className={interventionActive ? "active" : ""}
            onClick={() => setInterventionActive(!interventionActive)}
          >
            {interventionActive ? "Зафиксировать педаль" : "Активировать педаль"}
          </button>
        </div>
      )}
    </div>
  );
}