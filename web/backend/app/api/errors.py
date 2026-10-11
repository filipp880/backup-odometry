"""Map domain errors to HTTP codes in one place."""
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse

from app.domain.errors import BadReference, Conflict, DomainError, NotFound, StorageFull


def register_handlers(app: FastAPI) -> None:
    @app.exception_handler(NotFound)
    async def _not_found(_: Request, exc: NotFound) -> JSONResponse:
        return JSONResponse(status_code=404, content={"detail": str(exc)})

    @app.exception_handler(StorageFull)
    async def _full(_: Request, exc: StorageFull) -> JSONResponse:
        return JSONResponse(status_code=409, content={"detail": str(exc)})

    @app.exception_handler(Conflict)
    async def _conflict(_: Request, exc: Conflict) -> JSONResponse:
        return JSONResponse(status_code=409, content={"detail": str(exc)})

    @app.exception_handler(BadReference)
    async def _bad_ref(_: Request, exc: BadReference) -> JSONResponse:
        return JSONResponse(status_code=400, content={"detail": str(exc)})

    @app.exception_handler(DomainError)
    async def _domain(_: Request, exc: DomainError) -> JSONResponse:
        return JSONResponse(status_code=500, content={"detail": str(exc)})
