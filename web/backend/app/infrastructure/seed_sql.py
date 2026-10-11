"""DB seed: demo routes/runs + admin/user. Runs once when users table is empty."""
from sqlalchemy import select

from app.application.auth import hash_password
from app.infrastructure.db import SessionLocal
from app.infrastructure.memory_repo import RouteMemoryRepo, RunMemoryRepo
from app.infrastructure.orm_models import RouteRow, RunRow, UserRow
from app.infrastructure.seed import seed as memory_seed


def seed_sql() -> None:
    with SessionLocal() as s:
        if s.scalar(select(UserRow.id).limit(1)) is not None:
            return
        mem_routes, mem_runs = RouteMemoryRepo(), RunMemoryRepo()
        memory_seed(mem_routes, mem_runs)
        for r in mem_routes.list():
            s.add(RouteRow(name=r.name, startPoint=r.startPoint, endPoint=r.endPoint,
                           stops=r.stops, polyline=r.polyline, startHeight=r.startHeight,
                           endHeight=r.endHeight, totalAscent=r.totalAscent,
                           createdAt=r.createdAt, updatedAt=r.updatedAt))
        for r in mem_runs.list():
            s.add(RunRow(title=r.title, routeId=r.routeId, description=r.description,
                         date=r.date, duration=r.duration, length=r.length,
                         status=r.status, sourceFile=r.sourceFile,
                         createdAt=r.createdAt, updatedAt=r.updatedAt))
        s.add(UserRow(username="admin", passwordHash=hash_password("admin123"), role="admin"))
        s.add(UserRow(username="user", passwordHash=hash_password("user123"), role="user"))
        s.commit()
