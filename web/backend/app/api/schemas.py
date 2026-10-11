"""Pydantic schemas = API contract. Domain entities stay framework-free."""
from pydantic import BaseModel, Field


# --- runs ---

class RunCreate(BaseModel):
    title: str = Field(..., examples=["Утренний прогон"])
    routeId: int = Field(..., examples=[1])
    description: str = ""
    date: str = Field("", examples=["2026-09-01"])
    duration: float = 0.0
    length: float = 0.0
    status: str = "new"
    sourceFile: str = ""


class RunUpdate(BaseModel):
    title: str | None = None
    routeId: int | None = None
    description: str | None = None
    date: str | None = None
    duration: float | None = None
    length: float | None = None
    status: str | None = None
    sourceFile: str | None = None


class RunOut(RunCreate):
    id: int
    createdAt: str
    updatedAt: str


# --- routes ---

class RouteCreate(BaseModel):
    name: str = Field(..., examples=["Щукинская — Таллинская"])
    startPoint: str = ""
    endPoint: str = ""
    stops: list[str] = []
    polyline: list[list[float]] = []
    startHeight: float = 0.0
    endHeight: float = 0.0
    totalAscent: float = 0.0


class RouteUpdate(BaseModel):
    name: str | None = None
    startPoint: str | None = None
    endPoint: str | None = None
    stops: list[str] | None = None
    polyline: list[list[float]] | None = None
    startHeight: float | None = None
    endHeight: float | None = None
    totalAscent: float | None = None


class RouteOut(RouteCreate):
    id: int
    createdAt: str
    updatedAt: str
