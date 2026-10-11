import { useEffect, useState } from "react";
import { Link, useNavigate, useParams } from "react-router-dom";
import { ApiError, routesApi } from "../api/client";

export default function RouteForm({ mode }: { mode: "new" | "edit" }) {
  const { id } = useParams();
  const nav = useNavigate();
  const [name, setName] = useState("");
  const [startPoint, setStartPoint] = useState("");
  const [endPoint, setEndPoint] = useState("");
  const [err, setErr] = useState("");

  useEffect(() => {
    if (mode === "edit" && id)
      routesApi.get(Number(id)).then((r) => {
        setName(r.name); setStartPoint(r.startPoint); setEndPoint(r.endPoint);
      }).catch((e: ApiError) => setErr(e.message));
  }, [mode, id]);

  const submit = async (e: React.FormEvent) => {
    e.preventDefault();
    try {
      if (mode === "new") {
        const r = await routesApi.create({ name, startPoint, endPoint });
        nav(`/routes/${r.id}`);
      } else {
        await routesApi.update(Number(id), { name, startPoint, endPoint });
        nav(`/routes/${id}`);
      }
    } catch (e2) { setErr((e2 as ApiError).message); }
  };

  return (
    <div>
      <h2>{mode === "new" ? "Новый маршрут" : `Маршрут ${id}`}</h2>
      {err && <p className="err">{err}</p>}
      <form onSubmit={submit}>
        <label>Название<input value={name} onChange={(e) => setName(e.target.value)} required /></label>
        <label>Откуда<input value={startPoint} onChange={(e) => setStartPoint(e.target.value)} /></label>
        <label>Куда<input value={endPoint} onChange={(e) => setEndPoint(e.target.value)} /></label>
        <button type="submit">Сохранить</button>{" "}
        <Link to="/routes">Отмена</Link>
      </form>
    </div>
  );
}
