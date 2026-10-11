"""Route endpoints. Thin: parse input -> use case -> schema out."""
from fastapi import APIRouter, Depends

from app.api.deps import get_repos
from app.api.guards import require_user
from app.api.schemas import RouteCreate, RouteOut, RouteUpdate
from app.application import use_cases as uc
from app.domain.entities import Route

router = APIRouter(prefix="/api/routes", tags=["routes"])


@router.get("", response_model=list[RouteOut], summary="List routes")
def list_routes(deps=Depends(get_repos)):
    return [RouteOut(**vars(r)) for r in uc.list_routes(deps.routes)]


@router.get("/{ident}", response_model=RouteOut, summary="Get route by id",
            responses={404: {"description": "Route not found"}})
def get_route(ident: int, deps=Depends(get_repos)):
    return RouteOut(**vars(uc.get_route(deps.routes, ident)))


@router.post("", response_model=RouteOut, status_code=201, summary="Create route",
             dependencies=[Depends(require_user)],
             responses={401: {"description": "Login required"},
                        409: {"description": "Storage full (50 records)"}})
def create_route(body: RouteCreate, deps=Depends(get_repos)):
    route = uc.create_route(deps.routes, Route(**body.model_dump()))
    return RouteOut(**vars(route))


@router.put("/{ident}", response_model=RouteOut, summary="Update route",
            dependencies=[Depends(require_user)],
            responses={401: {"description": "Login required"},
                       404: {"description": "Route not found"}})
def update_route(ident: int, body: RouteUpdate, deps=Depends(get_repos)):
    patch = {k: v for k, v in body.model_dump().items() if v is not None}
    return RouteOut(**vars(uc.update_route(deps.routes, ident, patch)))


@router.delete("/{ident}", status_code=204, summary="Delete route",
               dependencies=[Depends(require_user)],
               responses={401: {"description": "Login required"},
                          404: {"description": "Route not found"},
                          409: {"description": "Route has runs"}})
def delete_route(ident: int, deps=Depends(get_repos)):
    uc.delete_route(deps.routes, deps.runs, ident)
