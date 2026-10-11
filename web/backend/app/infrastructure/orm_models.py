"""ORM models, code-first. Column config lives here (like Fluent API)."""
from sqlalchemy import JSON, Index, String
from sqlalchemy.orm import DeclarativeBase, Mapped, mapped_column


class Base(DeclarativeBase):
    pass


class RouteRow(Base):
    __tablename__ = "routes"

    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)
    name: Mapped[str] = mapped_column(String(200), nullable=False)
    startPoint: Mapped[str] = mapped_column("start_point", String(200), default="")
    endPoint: Mapped[str] = mapped_column("end_point", String(200), default="")
    stops: Mapped[list] = mapped_column(JSON, default=list)
    polyline: Mapped[list] = mapped_column(JSON, default=list)
    startHeight: Mapped[float] = mapped_column("start_height", default=0.0)
    endHeight: Mapped[float] = mapped_column("end_height", default=0.0)
    totalAscent: Mapped[float] = mapped_column("total_ascent", default=0.0)
    createdAt: Mapped[str] = mapped_column("created_at", String(40), default="")
    updatedAt: Mapped[str] = mapped_column("updated_at", String(40), default="")


class RunRow(Base):
    __tablename__ = "runs"

    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)
    title: Mapped[str] = mapped_column(String(200), nullable=False)
    routeId: Mapped[int] = mapped_column("route_id", nullable=False, index=True)
    description: Mapped[str] = mapped_column(String(500), default="")
    date: Mapped[str] = mapped_column(String(20), default="")
    duration: Mapped[float] = mapped_column(default=0.0)
    length: Mapped[float] = mapped_column(default=0.0)
    status: Mapped[str] = mapped_column(String(20), default="new")
    sourceFile: Mapped[str] = mapped_column("source_file", String(200), default="")
    createdAt: Mapped[str] = mapped_column("created_at", String(40), default="")
    updatedAt: Mapped[str] = mapped_column("updated_at", String(40), default="")

    __table_args__ = (Index("ix_runs_route", "route_id"),)


class UserRow(Base):
    __tablename__ = "users"

    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)
    username: Mapped[str] = mapped_column(String(80), unique=True, nullable=False)
    passwordHash: Mapped[str] = mapped_column("password_hash", String(200), nullable=False)
    role: Mapped[str] = mapped_column(String(20), default="user")
