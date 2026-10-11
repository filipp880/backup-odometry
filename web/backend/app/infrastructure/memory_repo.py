"""In-memory repositories for lab 2. Limit: 50 records per entity."""
from __future__ import annotations

from app.domain.entities import Route, Run
from app.domain.errors import StorageFull

LIMIT = 50


class _MemoryStore:
    _kind = "record"

    def __init__(self) -> None:
        self._items: dict[int, Run | Route] = {}
        self._next_id = 1

    def list(self) -> list:
        return [self._items[k] for k in sorted(self._items)]

    def get(self, ident: int):
        return self._items.get(ident)

    def add(self, entity):
        if len(self._items) >= LIMIT:
            raise StorageFull(LIMIT)
        entity.id = self._next_id
        self._next_id += 1
        self._items[entity.id] = entity
        return entity

    def update(self, entity):
        if entity.id not in self._items:
            return None
        self._items[entity.id] = entity
        return entity

    def delete(self, ident: int) -> bool:
        return self._items.pop(ident, None) is not None


class RunMemoryRepo(_MemoryStore):
    _kind = "run"

    def count_by_route(self, route_id: int) -> int:
        return sum(1 for r in self._items.values() if r.routeId == route_id)


class RouteMemoryRepo(_MemoryStore):
    _kind = "route"
