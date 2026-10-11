import { BrowserRouter, Navigate, Route, Routes } from "react-router-dom";
import Nav from "./components/Nav";
import RouteForm from "./pages/RouteForm";
import RouteView from "./pages/RouteView";
import RoutesList from "./pages/RoutesList";
import RunForm from "./pages/RunForm";
import RunView from "./pages/RunView";
import RunsList from "./pages/RunsList";
import TripPage from "./pages/TripPage";
import ReportPage from "./pages/ReportPage";
import LoginPage from "./pages/LoginPage";
import SimulatorPage from "./pages/SimulatorPage";

export default function App() {
  return (
    <BrowserRouter>
      <Nav />
      <main>
        <Routes>
          <Route path="/" element={<Navigate to="/runs" />} />
          <Route path="/login" element={<LoginPage />} />
          <Route path="/runs" element={<RunsList />} />
          <Route path="/runs/new" element={<RunForm mode="new" />} />
          <Route path="/runs/:id" element={<RunView />} />
          <Route path="/runs/:id/trip" element={<TripPage />} />
          <Route path="/runs/:id/report" element={<ReportPage />} />
          <Route path="/runs/:id/edit" element={<RunForm mode="edit" />} />
          <Route path="/routes" element={<RoutesList />} />
          <Route path="/routes/new" element={<RouteForm mode="new" />} />
          <Route path="/routes/:id" element={<RouteView />} />
          <Route path="/routes/:id/edit" element={<RouteForm mode="edit" />} />
          <Route path="/simulator" element={<SimulatorPage />} />
        </Routes>
      </main>
    </BrowserRouter>
  );
}
