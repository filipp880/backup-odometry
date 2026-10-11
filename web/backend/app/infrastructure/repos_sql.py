"""SQL repositories. Same interfaces as memory ones — routers don't change."""
from sqlalchemy import func, select

from app.domain.entities import Route, Run
from app.domain.errors import StorageFull
from app.infrastructure.db import SessionLocal
from app.infrastructure.memory_repo import LIMIT
from app.infrastructure.orm_models import RouteRow, RunRow, UserRow


def _route_to_entity(r: RouteRow) -> Route:
    return Route(id=r.id, name=r.name, startPoint=r.startPoint, endPoint=r.endPoint,
                 stops=list(r.stops or []), polyline=list(r.polyline or []),
                 startHeight=r.startHeight, endHeight=r.endHeight,
                 totalAscent=r.totalAscent, createdAt=r.createdAt, updatedAt=r.updatedAt)


def _run_to_entity(r: RunRow) -> Run:
    return Run(id=r.id, title=r.title, routeId=r.routeId, description=r.description,
               date=r.date, duration=r.duration, length=r.length, status=r.status,
               sourceFile=r.sourceFile, createdAt=r.createdAt, updatedAt=r.updatedAt)


class RunSqlRepo:
    def list(self) -> list[Run]:
        with SessionLocal() as s:
            return [_run_to_entity(r) for r in s.scalars(select(RunRow).order_by(RunRow.id)).all()]

    def get(self, ident: int) -> Run | None:
        with SessionLocal() as s:
            row = s.get(RunRow, ident)
            return _run_to_entity(row) if row else None

    def add(self, run: Run) -> Run:
        with SessionLocal() as s:
            if s.scalar(select(func.count()).select_from(RunRow)) >= LIMIT:
                raise StorageFull(LIMIT)
            row = RunRow(title=run.title, routeId=run.routeId, description=run.description,
                         date=run.date, duration=run.duration, length=run.length,
                         status=run.status, sourceFile=run.sourceFile,
                         createdAt=run.createdAt, updatedAt=run.updatedAt)
            s.add(row)
            s.commit()
            s.refresh(row)
            return _run_to_entity(row)

    def update(self, run: Run) -> Run | None:
        with SessionLocal() as s:
            row = s.get(RunRow, run.id)
            if row is None:
                return None
            for f in ("title", "routeId", "description", "date", "duration",
                      "length", "status", "sourceFile", "updatedAt"):
                setattr(row, f, getattr(run, f))
            s.commit()
            return _run_to_entity(row)

    def delete(self, ident: int) -> bool:
        with SessionLocal() as s:
            row = s.get(RunRow, ident)
            if row is None:
                return False
            s.delete(row)
            s.commit()
            return True

    def count_by_route(self, route_id: int) -> int:
        with SessionLocal() as s:
            return s.scalar(select(func.count()).select_from(RunRow)
                            .where(RunRow.routeId == route_id)) or 0


class RouteSqlRepo:
    def list(self) -> list[Route]:
        with SessionLocal() as s:
            return [_route_to_entity(r) for r in s.scalars(select(RouteRow).order_by(RouteRow.id)).all()]

    def get(self, ident: int) -> Route | None:
        with SessionLocal() as s:
            row = s.get(RouteRow, ident)
            return _route_to_entity(row) if row else None

    def add(self, route: Route) -> Route:
        with SessionLocal() as s:
            if s.scalar(select(func.count()).select_from(RouteRow)) >= LIMIT:
                raise StorageFull(LIMIT)
            row = RouteRow(name=route.name, startPoint=route.startPoint, endPoint=route.endPoint,
                           stops=route.stops, polyline=route.polyline,
                           startHeight=route.startHeight, endHeight=route.endHeight,
                           totalAscent=route.totalAscent,
                           createdAt=route.createdAt, updatedAt=route.updatedAt)
            s.add(row)
            s.commit()
            s.refresh(row)
            return _route_to_entity(row)

    def update(self, route: Route) -> Route | None:
        with SessionLocal() as s:
            row = s.get(RouteRow, route.id)
            if row is None:
                return None
            for f in ("name", "startPoint", "endPoint", "stops", "polyline",
                      "startHeight", "endHeight", "totalAscent", "updatedAt"):
                setattr(row, f, getattr(route, f))
            s.commit()
            return _route_to_entity(row)

    def delete(self, ident: int) -> bool:
        with SessionLocal() as s:
            row = s.get(RouteRow, ident)
            if row is None:
                return False
            s.delete(row)
            s.commit()
            return True


class UserSqlRepo:
    def get_by_username(self, username: str) -> UserRow | None:
        with SessionLocal() as s:
            return s.scalars(select(UserRow).where(UserRow.username == username)).first()

    def get_by_id(self, ident: int) -> UserRow | None:
        with SessionLocal() as s:
            return s.get(UserRow, ident)

    def list(self) -> list[dict]:
        with SessionLocal() as s:
            rows = s.scalars(select(UserRow).order_by(UserRow.id)).all()
            return [{"id": r.id, "username": r.username, "role": r.role} for r in rows]

    def add(self, username: str, password_hash: str, role: str = "user") -> dict:
        with SessionLocal() as s:
            row = UserRow(username=username, passwordHash=password_hash, role=role)
            s.add(row)
            s.commit()
            s.refresh(row)
            return {"id": row.id, "username": row.username, "role": row.role}
