"""Model adapter protocol + mock implementation. Real impl can call Python subprocess."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, Iterator


@dataclass
class Frame:
    t: float
    u: float
    v_model: float
    v_gnss: float | None
    v_wheels: float
    s: float
    x: float
    y: float
    z: float
    trust: float
    slip: float
    heading: float | None = None


class ModelAdapter(Protocol):
    def replay(self, run_id: str) -> Iterator[Frame]: ...
    def available_runs(self) -> list[str]: ...


@dataclass
class MockAdapter:
    """Deterministic mock: GNSS truth + wheel noise + model that lags slightly."""
    _runs: dict[str, list[Frame]] = None

    DT = 0.02  # s, the frame period used below

    def __post_init__(self):
        import math
        self._runs = {}
        for run_id in ["30618_0e41eac3", "30618_27e994fc", "30618_1551d0a9"]:
            frames = []
            t = 0.0
            v = 0.0  # m/s body speed, integrated from the throttle
            s = 0.0  # distance travelled, m
            x = 0.0  # ENU-ish position along the route
            y = 0.0
            while t < 120.0:
                u = min(1.0, max(0.0, 0.5 * math.sin(t * 0.05) + 0.3))
                v_gnss = v
                # wheels: ~3% multiplicative noise; the model: ~0.5%, so the
                # report shows the model winning (rmse_model < rmse_wheels)
                v_wheels = v_gnss * (1.0 + 0.03 * math.sin(t * 0.7))
                v_model = v_gnss * 0.995 + 0.006 * math.sin(t)
                # acceleration from the throttle notch; the tram rolls on
                # momentum when the pedal is released
                v = max(0.0, v + (u - 0.3) * 0.02)
                s += v * self.DT
                x += v * self.DT
                frames.append(Frame(
                    t=t, u=u, v_model=v_model, v_gnss=v_gnss, v_wheels=v_wheels,
                    s=s, x=x, y=y, z=174.0, trust=0.9, slip=0.1, heading=0.0
                ))
                t += self.DT
            self._runs[run_id] = frames

    def replay(self, run_id: str) -> Iterator[Frame]:
        frames = self._runs.get(run_id, [])
        for f in frames:
            yield f

    def available_runs(self) -> list[str]:
        return list(self._runs.keys())


# Lazy singletons
_mock: MockAdapter | None = None


def get_adapter() -> ModelAdapter:
    global _mock
    if _mock is None:
        _mock = MockAdapter()
    return _mock