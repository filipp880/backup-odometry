import { Link } from "react-router-dom";

export default function Nav() {
  return (
    <nav className="nav">
      <span className="brand">Трамвайный полигон</span>
      <Link to="/runs">Прогоны</Link>
      <Link to="/runs/new">+ Прогон</Link>
      <Link to="/routes">Маршруты</Link>
      <Link to="/routes/new">+ Маршрут</Link>
      <Link to="/simulator">Симулятор</Link>
      <a href="http://localhost:8000/docs" target="_blank" rel="noreferrer">Swagger</a>
      <Link to="/login" style={{ marginLeft: "auto" }}>Вход</Link>
    </nav>
  );
}
