"""Initial schema: routes, runs, users."""
import sqlalchemy as sa
from alembic import op

revision = "0001_init"
down_revision = None
branch_labels = None
depends_on = None


def upgrade() -> None:
    op.create_table(
        "routes",
        sa.Column("id", sa.Integer(), autoincrement=True, nullable=False),
        sa.Column("name", sa.String(200), nullable=False),
        sa.Column("start_point", sa.String(200), nullable=True),
        sa.Column("end_point", sa.String(200), nullable=True),
        sa.Column("stops", sa.JSON(), nullable=True),
        sa.Column("polyline", sa.JSON(), nullable=True),
        sa.Column("start_height", sa.Float(), nullable=True),
        sa.Column("end_height", sa.Float(), nullable=True),
        sa.Column("total_ascent", sa.Float(), nullable=True),
        sa.Column("created_at", sa.String(40), nullable=True),
        sa.Column("updated_at", sa.String(40), nullable=True),
        sa.PrimaryKeyConstraint("id"),
    )
    op.create_table(
        "runs",
        sa.Column("id", sa.Integer(), autoincrement=True, nullable=False),
        sa.Column("title", sa.String(200), nullable=False),
        sa.Column("route_id", sa.Integer(), nullable=False),
        sa.Column("description", sa.String(500), nullable=True),
        sa.Column("date", sa.String(20), nullable=True),
        sa.Column("duration", sa.Float(), nullable=True),
        sa.Column("length", sa.Float(), nullable=True),
        sa.Column("status", sa.String(20), nullable=True),
        sa.Column("source_file", sa.String(200), nullable=True),
        sa.Column("created_at", sa.String(40), nullable=True),
        sa.Column("updated_at", sa.String(40), nullable=True),
        sa.PrimaryKeyConstraint("id"),
    )
    op.create_index("ix_runs_route", "runs", ["route_id"])
    op.create_table(
        "users",
        sa.Column("id", sa.Integer(), autoincrement=True, nullable=False),
        sa.Column("username", sa.String(80), nullable=False),
        sa.Column("password_hash", sa.String(200), nullable=False),
        sa.Column("role", sa.String(20), nullable=True),
        sa.PrimaryKeyConstraint("id"),
        sa.UniqueConstraint("username"),
    )


def downgrade() -> None:
    op.drop_table("users")
    op.drop_index("ix_runs_route", table_name="runs")
    op.drop_table("runs")
    op.drop_table("routes")
