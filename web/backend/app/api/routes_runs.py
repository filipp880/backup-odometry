"""Run endpoints. Thin: parse input -> use case -> schema out."""
from fastapi import APIRouter, Depends

from app.api.deps import get_repos
from app.api.schemas import RunCreate, RunOut, RunUpdate
from app.application import use_cases as uc
from app.domain.entities import Run

router = APIRouter(prefix="/api/runs", tags=["runs"])


@router.get("", response_model=list[RunOut], summary="List runs")
def list_runs(deps=Depends(get_repos)):
    return [RunOut(**vars(r)) for r in uc.list_runs(deps.runs)]


@router.get("/{ident}", response_model=RunOut, summary="Get run by id",
            responses={404: {"description": "Run not found"}})
def get_run(ident: int, deps=Depends(get_repos)):
    return RunOut(**vars(uc.get_run(deps.runs, ident)))


@router.post("", response_model=RunOut, status_code=201, summary="Create run",
             responses={400: {"description": "Bad routeId"},
                        409: {"description": "Storage full (50 records)"}})
def create_run(body: RunCreate, deps=Depends(get_repos)):
    run = uc.create_run(deps.runs, deps.routes, Run(**body.model_dump()))
    return RunOut(**vars(run))


@router.put("/{ident}", response_model=RunOut, summary="Update run",
             responses={400: {"description": "Bad routeId"},
                        404: {"description": "Run not found"}})
def update_run(ident: int, body: RunUpdate, deps=Depends(get_repos)):
    patch = {k: v for k, v in body.model_dump().items() if v is not None}
    return RunOut(**vars(uc.update_run(deps.runs, deps.routes, ident, patch)))


@router.delete("/{ident}", status_code=204, summary="Delete run",
               responses={404: {"description": "Run not found"}})
def delete_run(ident: int, deps=Depends(get_repos)):
    uc.delete_run(deps.runs, ident)
