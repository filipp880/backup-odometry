import { useEffect, useState } from "react";
import { Link, useNavigate, useParams } from "react-router-dom";
import { ApiError, runsApi, type Run } from "../api/client";

export default function RunView() {
  const { id } = useParams();
  const nav = useNavigate();
  const [run, setRun] = useState<Run | null>(null);
  const [err, setErr] = useState("");

  useEffect(() => {
    runsApi.get(Number(id)).then(setRun).catch((e: ApiError) => setErr(e.message));
  }, [id]);

  const onDelete = async () => {
    if (!window.confirm(`Удалить прогон ${id}?`)) return;
    try { await runsApi.remove(Number(id)); nav("/runs"); }
    catch (e) { setErr((e as ApiError).message); }
  };

  if (err) return <p className="err">{err}</p>;
  if (!run) return <p>Загрузка…</p>;
  return (
    <div>
      <h2>{run.title}</h2>
      <dl>
        <dt>ID</dt><dd>{run.id}</dd>
        <dt>Маршрут</dt><dd>{run.routeId}</dd>
        <dt>Описание</dt><dd>{run.description}</dd>
        <dt>Дата</dt><dd>{run.date}</dd>
        <dt>Длительность, с</dt><dd>{run.duration}</dd>
        <dt>Длина, м</dt><dd>{run.length}</dd>
        <dt>Статус</dt><dd>{run.status}</dd>
        <dt>Файл</dt><dd>{run.sourceFile}</dd>
      </dl>
      <Link to={`/runs/${run.id}/edit`}>Изменить</Link>{" "}
      <Link to={`/runs/${run.id}/trip`}>Поездка</Link>{" "}
      <button onClick={onDelete}>Удалить</button>{" "}
      <Link to="/runs">К списку</Link>
    </div>
  );
}
