"""Auth endpoints: register / login / me / users (admin)."""
from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel, Field

from app.api.deps import get_repos
from app.api.guards import require_admin, require_user, user_public_row
from app.application.auth import check_password, hash_password, make_token

router = APIRouter(prefix="/api/auth", tags=["auth"])


class RegisterIn(BaseModel):
    username: str = Field(..., min_length=3, max_length=80)
    password: str = Field(..., min_length=4, max_length=120)


class LoginIn(BaseModel):
    username: str
    password: str


class TokenOut(BaseModel):
    token: str
    role: str


@router.post("/register", response_model=TokenOut, status_code=201, summary="Register user",
             responses={409: {"description": "Username taken"}})
def register(body: RegisterIn, deps=Depends(get_repos)):
    if deps.users.get_by_username(body.username) is not None:
        raise HTTPException(status_code=409, detail="username taken")
    row = deps.users.add(body.username, hash_password(body.password), "user")
    return TokenOut(token=make_token(row["id"], "user"), role="user")


@router.post("/login", response_model=TokenOut, summary="Login",
             responses={401: {"description": "Bad credentials"}})
def login(body: LoginIn, deps=Depends(get_repos)):
    row = deps.users.get_by_username(body.username)
    if row is None or not check_password(body.password, row.passwordHash):
        raise HTTPException(status_code=401, detail="bad credentials")
    return TokenOut(token=make_token(row.id, row.role), role=row.role)


@router.get("/me", summary="Current user", responses={401: {"description": "Not logged in"}})
def me(user: dict = Depends(require_user), deps=Depends(get_repos)):
    row = user_public_row(deps.users, int(user["sub"]))
    if row is None:
        raise HTTPException(status_code=401, detail="user gone")
    return row


@router.get("/users", summary="List users (admin only)",
            responses={401: {"description": "Not logged in"},
                       403: {"description": "Admin only"}})
def list_users(_: dict = Depends(require_admin), deps=Depends(get_repos)):
    return deps.users.list()
