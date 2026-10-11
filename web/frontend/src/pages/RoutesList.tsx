import { useEffect, useState } from "react";
import { Link, useNavigate } from "react-router-dom";
import { ApiError, routesApi, type Route } from "../api/client";

export default function RoutesList() {
  const [routes, setRoutes] = useState<Route[]>([]);
  const [err, setErr] = useState("");
  const [loading, setLoading] = useState(true);
  const nav = useNavigate();

  useEffect(() => {
    routesApi.list().then(setRoutes).catch((e: ApiError) => setErr(e.message)).finally(() => setLoading(false));
  }, []);

  const onDelete = async (id: number) => {
    if (!window.confirm(`Удалить маршрут ${id}?`)) return;
    try { await routesApi.remove(id); setRoutes(routes.filter((r) => r.id !== id)); }
    catch (e) { setErr((e as ApiError).message); }
  };

  if (loading) return <p>Загрузка…</p>;
  return (
    <div>
      <h2>Маршруты</h2>
      {err && <p className="err">{err}</p>}
      <button onClick={() => nav("/routes/new")}>Добавить маршрут</button>
      <table>
        <thead><tr><th>ID</th><th>Название</th><th>Откуда</th><th>Куда</th><th></th></tr></thead>
        <tbody>
          {routes.map((r) => (
            <tr key={r.id}>
              <td>{r.id}</td>
              <td><Link to={`/routes/${r.id}`}>{r.name}</Link></td>
              <td>{r.startPoint}</td>
              <td>{r.endPoint}</td>
              <td>
                <Link to={`/routes/${r.id}/edit`}>Изменить</Link>{" "}
                <button onClick={() => onDelete(r.id)}>Удалить</button>
              </td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}
