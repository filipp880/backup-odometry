import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  Legend,
  ResponsiveContainer,
} from "recharts";
import type { Frame } from "../api/client";

export function SpeedChart({ frames }: { frames: Frame[] }) {
  const data = frames.map((f) => ({
    t: f.t,
    "Модель": +f.v_model.toFixed(2),
    "GNSS": f.v_gnss ? +f.v_gnss.toFixed(2) : null,
    "Колёса": +f.v_wheels.toFixed(2),
    "Педаль": f.u,
  }));
  return (
    <ResponsiveContainer width="100%" height={200}>
      <LineChart data={data} margin={{ top: 5, right: 30, left: 20, bottom: 5 }}>
        <CartesianGrid strokeDasharray="3 3" />
        <XAxis dataKey="t" tickFormatter={(v) => `${v.toFixed(0)}с`} />
        <YAxis />
        <Tooltip />
        <Legend />
        <Line type="monotone" dataKey="Модель" stroke="#0066cc" strokeWidth={2} dot={false} />
        <Line type="monotone" dataKey="GNSS" stroke="#cc0000" strokeWidth={2} dot={false} />
        <Line type="monotone" dataKey="Колёса" stroke="#888" strokeWidth={1.5} strokeDasharray="4 4" dot={false} />
        <Line type="monotone" dataKey="Педаль" stroke="#009933" strokeWidth={1} dot={false} />
      </LineChart>
    </ResponsiveContainer>
  );
}

export function TrustChart({ frames }: { frames: Frame[] }) {
  const data = frames.map((f) => ({
    t: f.t,
    "Доверие": +f.trust.toFixed(2),
    "Скольжение": +f.slip.toFixed(2),
  }));
  return (
    <ResponsiveContainer width="100%" height={120}>
      <LineChart data={data} margin={{ top: 5, right: 30, left: 20, bottom: 5 }}>
        <CartesianGrid strokeDasharray="3 3" />
        <XAxis dataKey="t" tickFormatter={(v) => `${v.toFixed(0)}с`} />
        <YAxis domain={[0, 1]} />
        <Tooltip />
        <Legend />
        <Line type="monotone" dataKey="Доверие" stroke="#0066cc" strokeWidth={2} dot={false} />
        <Line type="monotone" dataKey="Скольжение" stroke="#cc0000" strokeWidth={2} dot={false} />
      </LineChart>
    </ResponsiveContainer>
  );
}

export function PositionChart({ frames }: { frames: Frame[] }) {
  const data = frames.map((f) => ({
    t: f.t,
    X: +f.x.toFixed(1),
    Y: +f.y.toFixed(1),
    S: +f.s.toFixed(1),
  }));
  return (
    <ResponsiveContainer width="100%" height={120}>
      <LineChart data={data} margin={{ top: 5, right: 30, left: 20, bottom: 5 }}>
        <CartesianGrid strokeDasharray="3 3" />
        <XAxis dataKey="t" tickFormatter={(v) => `${v.toFixed(0)}с`} />
        <YAxis />
        <Tooltip />
        <Legend />
        <Line type="monotone" dataKey="S" stroke="#0066cc" strokeWidth={2} dot={false} />
        <Line type="monotone" dataKey="X" stroke="#cc0000" strokeWidth={1.5} dot={false} />
        <Line type="monotone" dataKey="Y" stroke="#009933" strokeWidth={1.5} dot={false} />
      </LineChart>
    </ResponsiveContainer>
  );
}