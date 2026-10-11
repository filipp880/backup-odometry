import { useEffect, useState } from "react";
import { Link, useNavigate, useParams } from "react-router-dom";
import { ApiError, runsApi } from "../api/client";

export default function RunForm({ mode }: { mode: "new" | "edit" }) {
  const { id } = useParams();
  const nav = useNavigate();
  const [title, setTitle] = useState("");
  const [routeId, setRouteId] = useState(1);
  const [description, setDescription] = useState("");
  const [err, setErr] = useState("");

  useEffect(() => {
    if (mode === "edit" && id)
      runsApi.get(Number(id)).then((r) => {
        setTitle(r.title); setRouteId(r.routeId); setDescription(r.description);
      }).catch((e: ApiError) => setErr(e.message));
  }, [mode, id]);

  const submit = async (e: React.FormEvent) => {
    e.preventDefault();
    try {
      if (mode === "new") {
        const r = await runsApi.create({ title, routeId, description });
        nav(`/runs/${r.id}`);
      } else {
        await runsApi.update(Number(id), { title, routeId, description });
        nav(`/runs/${id}`);
      }
    } catch (e2) { setErr((e2 as ApiError).message); }
  };

  return (
    <div>
      <h2>{mode === "new" ? "Новый прогон" : `Прогон ${id}`}</h2>
      {err && <p className="err">{err}</p>}
      <form onSubmit={submit}>
        <label>Название<input value={title} onChange={(e) => setTitle(e.target.value)} required /></label>
        <label>ID маршрута<input type="number" value={routeId} onChange={(e) => setRouteId(Number(e.target.value))} required /></label>
        <label>Описание<input value={description} onChange={(e) => setDescription(e.target.value)} /></label>
        <button type="submit">Сохранить</button>{" "}
        <Link to="/runs">Отмена</Link>
      </form>
    </div>
  );
}
