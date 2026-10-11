import { useEffect, useState } from "react";
import { Link, useNavigate, useParams } from "react-router-dom";
import { ApiError, routesApi, type Route } from "../api/client";

export default function RouteView() {
  const { id } = useParams();
  const nav = useNavigate();
  const [route, setRoute] = useState<Route | null>(null);
  const [err, setErr] = useState("");

  useEffect(() => {
    routesApi.get(Number(id)).then(setRoute).catch((e: ApiError) => setErr(e.message));
  }, [id]);

  const onDelete = async () => {
    if (!window.confirm(`Удалить маршрут ${id}?`)) return;
    try { await routesApi.remove(Number(id)); nav("/routes"); }
    catch (e) { setErr((e as ApiError).message); }
  };

  if (err) return <p className="err">{err}</p>;
  if (!route) return <p>Загрузка…</p>;
  return (
    <div>
      <h2>{route.name}</h2>
      <dl>
        <dt>ID</dt><dd>{route.id}</dd>
        <dt>Откуда</dt><dd>{route.startPoint}</dd>
        <dt>Куда</dt><dd>{route.endPoint}</dd>
        <dt>Остановки</dt><dd>{route.stops.join(", ")}</dd>
      </dl>
      <Link to={`/routes/${route.id}/edit`}>Изменить</Link>{" "}
      <button onClick={onDelete}>Удалить</button>{" "}
      <Link to="/routes">К списку</Link>
    </div>
  );
}
