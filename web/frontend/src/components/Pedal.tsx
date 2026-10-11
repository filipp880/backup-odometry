import { useRef, useEffect, useState } from "react";

export function Pedal({
  value,
  onChange,
  intervention,
  interventionActive,
}: {
  value: number;
  onChange: (v: number) => void;
  intervention: boolean;
  interventionActive: boolean;
}) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [dragging, setDragging] = useState(false);

  useEffect(() => {
    if (!canvasRef.current) return;
    const ctx = canvasRef.current.getContext("2d")!;
    const w = canvasRef.current.width;
    const h = canvasRef.current.height;
    ctx.clearRect(0, 0, w, h);
    // background
    ctx.fillStyle = "#eee";
    ctx.fillRect(0, 0, w, h);
    // zero line
    ctx.strokeStyle = "#999";
    ctx.beginPath();
    ctx.moveTo(w / 2, 0);
    ctx.lineTo(w / 2, h);
    ctx.stroke();
    // pedal bar
    const v = Math.max(-1, Math.min(1, value));
    const barH = (v * h) / 2;
    ctx.fillStyle = interventionActive ? "#ff9900" : v >= 0 ? "#009933" : "#cc0000";
    ctx.fillRect(w * 0.25, h / 2 - barH, w * 0.5, barH);
    // label
    ctx.fillStyle = "#333";
    ctx.font = "14px sans-serif";
    ctx.textAlign = "center";
    ctx.fillText(v >= 0 ? "ТЯГА" : "ТОРМОЗ", w / 2, v >= 0 ? 20 : h - 10);
  }, [value, interventionActive]);

  const handleMove = (e: React.MouseEvent | React.TouchEvent) => {
    if (!dragging || !canvasRef.current) return;
    const rect = canvasRef.current.getBoundingClientRect();
    const clientY = "touches" in e ? e.touches[0].clientY : e.clientY;
    const y = Math.max(0, Math.min(1, (rect.bottom - clientY) / rect.height));
    const v = (y - 0.5) * 2;
    onChange(v);
  };

  return (
    <div className="pedal-wrap">
      <canvas
        ref={canvasRef}
        width={120}
        height={300}
        onMouseDown={() => { if (intervention) setDragging(true); }}
        onMouseUp={() => setDragging(false)}
        onMouseLeave={() => setDragging(false)}
        onMouseMove={handleMove}
        onTouchStart={() => { if (intervention) setDragging(true); }}
        onTouchEnd={() => setDragging(false)}
        onTouchMove={handleMove}
        style={{ touchAction: "none", cursor: intervention ? "ns-resize" : "default" }}
      />
      <div className="pedal-hint">{intervention ? "Перетащите для управления" : "Режим: Смотрю"}</div>
    </div>
  );
}