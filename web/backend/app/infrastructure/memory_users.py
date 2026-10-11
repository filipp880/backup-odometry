"""In-memory users for lab 2 (no DB). Same duck-type as SQL rows."""
from types import SimpleNamespace

from app.application.auth import hash_password


class UserMemoryRepo:
    def __init__(self) -> None:
        self._items: dict[str, SimpleNamespace] = {}
        self._next_id = 1
        self.add("admin", hash_password("admin123"), "admin")
        self.add("user", hash_password("user123"), "user")

    def get_by_username(self, username: str):
        return self._items.get(username)

    def get_by_id(self, ident: int):
        return next((u for u in self._items.values() if u.id == ident), None)

    def list(self) -> list[dict]:
        return [{"id": u.id, "username": u.username, "role": u.role}
                for u in sorted(self._items.values(), key=lambda u: u.id)]

    def add(self, username: str, password_hash: str, role: str = "user") -> dict:
        user = SimpleNamespace(id=self._next_id, username=username,
                               passwordHash=password_hash, role=role)
        self._next_id += 1
        self._items[username] = user
        return {"id": user.id, "username": username, "role": role}
