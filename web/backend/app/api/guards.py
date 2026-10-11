"""Auth guards: anonymous / user / admin."""
from fastapi import Depends, HTTPException
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer

from app.application.auth import read_token

_bearer = HTTPBearer(auto_error=False)


def get_current_user(
    creds: HTTPAuthorizationCredentials | None = Depends(_bearer),
) -> dict | None:
    if creds is None:
        return None
    try:
        return read_token(creds.credentials)
    except Exception:
        raise HTTPException(status_code=401, detail="bad token")


def require_user(user: dict | None = Depends(get_current_user)) -> dict:
    if user is None:
        raise HTTPException(status_code=401, detail="login required")
    return user


def require_admin(user: dict | None = Depends(get_current_user)) -> dict:
    if user is None:
        raise HTTPException(status_code=401, detail="login required")
    if user.get("role") != "admin":
        raise HTTPException(status_code=403, detail="admin only")
    return user


def user_public_row(users: object, user_id: int) -> dict | None:
    row = users.get_by_id(user_id)  # type: ignore[attr-defined]
    if row is None:
        return None
    return {"id": row.id, "username": row.username, "role": row.role}
