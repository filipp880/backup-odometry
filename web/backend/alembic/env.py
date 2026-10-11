"""Alembic env. URL comes from DATABASE_URL, models from orm_models."""
import os

from alembic import context
from sqlalchemy import create_engine

from app.infrastructure.orm_models import Base

config = context.config
url = os.environ.get("DATABASE_URL") or config.get_main_option("sqlalchemy.url")
engine = create_engine(url)

with engine.connect() as connection:
    context.configure(connection=connection, target_metadata=Base.metadata)
    with context.begin_transaction():
        context.run_migrations()
