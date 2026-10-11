"""Use cases. No HTTP, no DB — only entities + repo interfaces."""
from __future__ import annotations

from datetime import datetime, timezone

from app.domain.entities import Route, Run
from app.domain.errors import BadReference, Conflict, NotFound
from app.domain.interfaces import RouteRepo, RunRepo


def _touch(entity: Run | Route) -> None:
    entity.updatedAt = datetime.now(timezone.utc).isoformat()


# --- runs ---

def list_runs(repos: RunRepo) -> list[Run]:
    return repos.list()


def get_run(repos: RunRepo, ident: int) -> Run:
    run = repos.get(ident)
    if run is None:
        raise NotFound("run", ident)
    return run


def create_run(runs: RunRepo, routes: RouteRepo, run: Run) -> Run:
    if routes.get(run.routeId) is None:
        raise BadReference(f"route {run.routeId} does not exist")
    return runs.add(run)


def update_run(runs: RunRepo, routes: RouteRepo, ident: int, patch: dict) -> Run:
    run = get_run(runs, ident)
    if "routeId" in patch and routes.get(patch["routeId"]) is None:
        raise BadReference(f"route {patch['routeId']} does not exist")
    for key, value in patch.items():
        if key not in ("id", "createdAt", "updatedAt") and hasattr(run, key):
            setattr(run, key, value)
    _touch(run)
    assert runs.update(run) is not None
    return run


def delete_run(runs: RunRepo, ident: int) -> None:
    if not runs.delete(ident):
        raise NotFound("run", ident)


# --- routes ---

def list_routes(repos: RouteRepo) -> list[Route]:
    return repos.list()


def get_route(repos: RouteRepo, ident: int) -> Route:
    route = repos.get(ident)
    if route is None:
        raise NotFound("route", ident)
    return route


def create_route(repos: RouteRepo, route: Route) -> Route:
    return repos.add(route)


def update_route(repos: RouteRepo, ident: int, patch: dict) -> Route:
    route = get_route(repos, ident)
    for key, value in patch.items():
        if key not in ("id", "createdAt", "updatedAt") and hasattr(route, key):
            setattr(route, key, value)
    _touch(route)
    assert repos.update(route) is not None
    return route


def delete_route(routes: RouteRepo, runs: RunRepo, ident: int) -> None:
    get_route(routes, ident)
    if runs.count_by_route(ident):
        raise Conflict(f"route {ident} has runs, delete them first")
    assert routes.delete(ident)
