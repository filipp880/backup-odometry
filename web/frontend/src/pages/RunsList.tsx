import { useEffect, useState } from "react";
import { Link, useNavigate } from "react-router-dom";
import { ApiError, runsApi, type Run } from "../api/client";

export default function RunsList() {
  const [runs, setRuns] = useState<Run[]>([]);
  const [err, setErr] = useState("");
  const [loading, setLoading] = useState(true);
  const nav = useNavigate();

  const load = () =>
    runsApi.list().then(setRuns).catch((e: ApiError) => setErr(e.message)).finally(() => setLoading(false));

  useEffect(() => { load(); }, []);

  const onDelete = async (id: number) => {
    if (!window.confirm(`Удалить прогон ${id}?`)) return;
    try { await runsApi.remove(id); setRuns(runs.filter((r) => r.id !== id)); }
    catch (e) { setErr((e as ApiError).message); }
  };

  if (loading) return <p>Загрузка…</p>;
  return (
    <div>
      <h2>Прогоны</h2>
      {err && <p className="err">{err}</p>}
      <button onClick={() => nav("/runs/new")}>Добавить прогон</button>
      <table>
        <thead><tr><th>ID</th><th>Название</th><th>Маршрут</th><th>Статус</th><th></th></tr></thead>
        <tbody>
          {runs.map((r) => (
            <tr key={r.id}>
              <td>{r.id}</td>
              <td><Link to={`/runs/${r.id}`}>{r.title}</Link></td>
              <td>{r.routeId}</td>
              <td>{r.status}</td>
              <td>
                <Link to={`/runs/${r.id}/edit`}>Изменить</Link>{" "}
                <button onClick={() => onDelete(r.id)}>Удалить</button>
              </td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}
