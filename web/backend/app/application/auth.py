"""Passwords (stdlib pbkdf2) + JWT. No extra native deps."""
import hashlib
import hmac
import os
import secrets
import time

import jwt

SECRET = os.environ.get("JWT_SECRET", "dev-secret-change-me-32bytes-minimum-ok")
ALGO = "HS256"
TTL = 24 * 3600


def hash_password(password: str) -> str:
    salt = secrets.token_hex(16)
    dk = hashlib.pbkdf2_hmac("sha256", password.encode(), bytes.fromhex(salt), 200_000)
    return f"pbkdf2$200000${salt}${dk.hex()}"


def check_password(password: str, stored: str) -> bool:
    try:
        _, _, salt, hexhash = stored.split("$")
        dk = hashlib.pbkdf2_hmac("sha256", password.encode(), bytes.fromhex(salt), 200_000)
        return hmac.compare_digest(dk.hex(), hexhash)
    except (ValueError, TypeError):
        return False


def make_token(user_id: int, role: str) -> str:
    now = int(time.time())
    # RFC 7519: sub must be a string (pyjwt >= 2.10 enforces it on decode)
    return jwt.encode({"sub": str(user_id), "role": role, "iat": now, "exp": now + TTL},
                      SECRET, algorithm=ALGO)


def read_token(token: str) -> dict:
    return jwt.decode(token, SECRET, algorithms=[ALGO])
