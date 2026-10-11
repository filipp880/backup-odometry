import { useEffect, useState } from "react";
import { useParams, Link } from "react-router-dom";
import { runsApi } from "../api/client";
import "./ReportPage.css";

export default function ReportPage() {
  const { id } = useParams<{ id: string }>();
  const [report, setReport] = useState<any>(null);
  const [loading, setLoading] = useState(true);
  const [err, setErr] = useState("");

  useEffect(() => {
    if (!id) return;
    runsApi.report(id).then(setReport).catch((e: any) => setErr(e.message)).finally(() => setLoading(false));
  }, [id]);

  if (loading) return <div className="report">Загрузка…</div>;
  if (err) return <div className="report err">{err}</div>;
  if (!report) return <div className="report">Нет отчёта</div>;

  return (
    <div className="report">
      <Link to="/runs" className="back">← К прогонам</Link>
      <h2>Отчёт по прогону {id}</h2>

      <section className="grid">
        <div className="card">
          <h3>Общие метрики</h3>
          <dl>
            <dt>Длительность</dt><dd>{report.duration?.toFixed(1)} с</dd>
            <dt>Кадров</dt><dd>{report.frames}</dd>
            <dt>Средняя скорость (модель)</dt><dd>{report.avg_speed_model?.toFixed(2)} м/с</dd>
            <dt>Средняя скорость (GNSS)</dt><dd>{report.avg_speed_gnss?.toFixed(2)} м/с</dd>
            <dt>Средняя скорость (колёса)</dt><dd>{report.avg_speed_wheels?.toFixed(2)} м/с</dd>
          </dl>
        </div>
        <div className="card">
          <h3>Точность</h3>
          <dl>
            <dt>RMSE модель</dt><dd>{report.rmse_model?.toFixed(2)} м/с</dd>
            <dt>RMSE колёса</dt><dd>{report.rmse_wheels?.toFixed(2)} м/с</dd>
            <dt>Выигрыш</dt>
              <dd style={{ color: (report.win ?? 0) >= 0 ? "#006600" : "#cc0000" }}>
                {(report.win ?? 0) >= 0 ? "+" : ""}{(report.win ?? 0).toFixed(2)} м/с
              </dd>
          </dl>
        </div>
      </section>

      <section className="limitations">
        <h3>⚠ Ограничения (обязательно к прочтению)</h3>
        <ul>
          <li>Симулятор не доказывает точность на реальности — метрики там измеряют согласие фильтра с самим собой.</li>
          <li>Вес пассажиров в модели — только «пустой/полный» (30/38 т), слайдер 0–200 не работает в реальном контуре.</li>
          <li>Уклон сгенерированного маршрута — не настоящая рельсовая геометрия.</li>
          <li>GNSS не участвует в основном контуре, только эталон и начальная выставка.</li>
          <li>Скрытие карты на экране не отключает карту в модели — для этого отдельный диагностический режим.</li>
          <li>Режим «Рулю» в реальном прогоне не создаёт новую истину — эталон остаётся записью.</li>
        </ul>
      </section>

      <Link to="/runs" className="back">← К прогонам</Link>
    </div>
  );
}