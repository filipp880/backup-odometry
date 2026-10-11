export function MetricsBar({
  frames,
  currentIdx,
  intervention,
}: {
  frames: Frame[];
  currentIdx: number;
  intervention: boolean;
}) {
  if (!frames.length) return <div className="metrics">Нет данных</div>;
  const f = frames[currentIdx] ?? frames[frames.length - 1];
  const vErrM = frames.reduce((a, b) => a + Math.pow(b.v_model - (b.v_gnss ?? b.v_model), 2), 0);
  const vErrW = frames.reduce((a, b) => a + Math.pow(b.v_wheels - (b.v_gnss ?? b.v_wheels), 2), 0);
  const n = frames.length;
  const rmseM = Math.sqrt(vErrM / n);
  const rmseW = Math.sqrt(vErrW / n);
  const win = rmseW - rmseM;

  return (
    <div className="metrics">
      <div className="row">
        <span>Текущая скорость:</span>
        <strong>{f.v_model.toFixed(2)} м/с</strong>
      </div>
      <div className="row">
        <span>RMSE модель:</span>
        <strong>{rmseM.toFixed(2)} м/с</strong>
      </div>
      <div className="row">
        <span>RMSE колёса:</span>
        <strong>{rmseW.toFixed(2)} м/с</strong>
      </div>
      <div className="row">
        <span>Выигрыш:</span>
        <strong style={{ color: win >= 0 ? "#006600" : "#cc0000" }}>
          {win >= 0 ? "+" : ""}{win.toFixed(2)} м/с
        </strong>
      </div>
      <div className="row">
        <span>Доверие:</span>
        <strong>{(f.trust * 100).toFixed(0)}%</strong>
      </div>
      <div className="row">
        <span>Скольжение:</span>
        <strong>{(f.slip * 100).toFixed(0)}%</strong>
      </div>
      <div className="row">
        <span>Пройденный путь:</span>
        <strong>{f.s.toFixed(0)} м</strong>
      </div>
      {intervention && (
        <div className="intervention">
          ⚠ Режим «Рулю» — бенчмарк глючит (эталон — запись)
        </div>
      )}
    </div>
  );
}

import type { Frame } from "../api/client";