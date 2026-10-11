"""Domain entities. Pure data, no DB, no HTTP, no framework imports."""
from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


@dataclass
class Route:
    id: int = 0
    name: str = ""
    startPoint: str = ""
    endPoint: str = ""
    stops: list[str] = field(default_factory=list)
    polyline: list[list[float]] = field(default_factory=list)
    startHeight: float = 0.0
    endHeight: float = 0.0
    totalAscent: float = 0.0
    createdAt: str = field(default_factory=_now)
    updatedAt: str = field(default_factory=_now)


@dataclass
class Run:
    id: int = 0
    title: str = ""
    routeId: int = 0
    description: str = ""
    date: str = ""
    duration: float = 0.0
    length: float = 0.0
    status: str = "new"
    sourceFile: str = ""
    createdAt: str = field(default_factory=_now)
    updatedAt: str = field(default_factory=_now)
