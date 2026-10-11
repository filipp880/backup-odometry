import { Link } from "react-router-dom";
import "./SimulatorPage.css";

export default function SimulatorPage() {
  return (
    <div className="simulator">
      <Link to="/runs" className="back">← К прогонам</Link>
      <h1>Симулятор (заглушка)</h1>
      <div className="card">
        <h2>⚠ Важно</h2>
        <p>Эта вкладка — <strong>отдельный режим</strong>. Она не смешивается с реальным прогоном в одном отчёте.</p>
        <p>В симуляторе:</p>
        <ul>
          <li>Педаль пользователя реально управляет миром.</li>
          <li>«Истина» создаётся <strong>тем же физическим движком</strong>, на котором стоит фильтр.</li>
          <li>Метрики показывают <strong>внутреннюю согласованность</strong> системы, а не точность на реальности.</li>
        </ul>
        <p>Настоящая проверка точности — только во вкладке «Поездка» (реальный прогон).</p>
      </div>

      <div className="card">
        <h2>Настройки симулятора (пока заглушка)</h2>
        <label>Длина маршрута (м)<input type="number" defaultValue={5000} /></label>
        <label>Профиль высот<select><option>плоский</option><option>подъём</option><option>волна</option></select></label>
        <label>Условия<select><option>сухо</option><option>мокро</option><option>песок</option><option>лёд</option></select></label>
        <label>Пассажиров<input type="range" min={0} max={200} defaultValue={100} /></label>
        <p style={{ color: "#666" }}>Реализация симулятора отложена. Это отдельная вкладка с собственным заголовком и подписью.</p>
      </div>

      <Link to="/runs" className="back">← К прогонам</Link>
    </div>
  );
}