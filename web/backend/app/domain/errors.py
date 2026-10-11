"""Domain errors. API layer maps them to HTTP codes."""


class DomainError(Exception):
    """Base for all domain errors."""


class NotFound(DomainError):
    def __init__(self, what: str, ident: int) -> None:
        super().__init__(f"{what} {ident} not found")
        self.what = what
        self.ident = ident


class StorageFull(DomainError):
    def __init__(self, limit: int) -> None:
        super().__init__(f"storage limit reached: {limit} records max")
        self.limit = limit


class Conflict(DomainError):
    pass


class BadReference(DomainError):
    pass
