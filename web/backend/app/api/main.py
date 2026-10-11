"""App entrypoint. Swagger UI at /docs."""
from fastapi import Depends, FastAPI

from app.api.deps import get_repos  # noqa: F401 (keeps wiring visible)
from app.api.errors import register_handlers
from app.api.guards import require_user
from app.api.routes_auth import router as auth_router
from app.api.routes_runs import router as runs_router
from app.api.routes_routes import router as routes_router
from app.api.routes_telemetry import router as telemetry_router


def create_app() -> FastAPI:
    app = FastAPI(title="Tram Polygon API",
                  description="CRUD for tram runs and routes. "
                              "STORAGE=memory: in-memory, 50 max (lab 2). "
                              "STORAGE=db: PostgreSQL (lab 3).",
                  version="0.3.0")
    register_handlers(app)
    app.include_router(runs_router, dependencies=[Depends(require_user)])
    app.include_router(routes_router)
    app.include_router(auth_router)
    app.include_router(telemetry_router, dependencies=[Depends(require_user)])

    @app.get("/", tags=["meta"], summary="Service info")
    def info():
        return {"service": "tram-polygon", "docs": "/docs"}

    return app


app = create_app()
