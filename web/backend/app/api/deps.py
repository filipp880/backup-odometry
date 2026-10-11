"""DI wiring. STORAGE=memory (lab 2) or db (lab 3). Routers stay untouched."""
import os
from dataclasses import dataclass

from app.domain.interfaces import RouteRepo, RunRepo
from app.infrastructure.memory_repo import RouteMemoryRepo, RunMemoryRepo
from app.infrastructure.memory_users import UserMemoryRepo
from app.infrastructure.seed import seed

STORAGE = os.environ.get("STORAGE", "memory")


@dataclass
class Repos:
    runs: RunRepo
    routes: RouteRepo
    users: object


def _build() -> Repos:
    if STORAGE == "db":
        from app.infrastructure.repos_sql import RouteSqlRepo, RunSqlRepo, UserSqlRepo
        from app.infrastructure.seed_sql import seed_sql
        seed_sql()
        return Repos(runs=RunSqlRepo(), routes=RouteSqlRepo(), users=UserSqlRepo())
    runs, routes = RunMemoryRepo(), RouteMemoryRepo()
    seed(routes, runs)  # lab 2 seed
    return Repos(runs=runs, routes=routes, users=UserMemoryRepo())


_REPOS = _build()


def get_repos() -> Repos:
    return _REPOS
