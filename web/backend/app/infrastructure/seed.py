"""Demo seed data for lab 2 (in-memory). Replaced by DB seed in lab 3."""
from app.domain.entities import Route, Run
from app.infrastructure.memory_repo import RouteMemoryRepo, RunMemoryRepo


def seed(routes: RouteMemoryRepo, runs: RunMemoryRepo) -> None:
    demo_routes = [
        Route(name="Щукинская — Таллинская", startPoint="Щукинская", endPoint="Таллинская",
              stops=["Щукинская", "Строгино", "Таллинская"],
              startHeight=150.0, endHeight=174.0, totalAscent=24.0),
        Route(name="Таллинская — Щукинская", startPoint="Таллинская", endPoint="Щукинская",
              stops=["Таллинская", "Строгино", "Щукинская"],
              startHeight=174.0, endHeight=150.0, totalAscent=5.0),
        Route(name="Короткий круг", startPoint="Депо", endPoint="Депо",
              stops=["Депо", "Разворот"], startHeight=160.0, endHeight=160.0),
    ]
    for r in demo_routes:
        routes.add(r)
    demo_runs = [
        Run(title="Утренний прогон 1", routeId=1, description="сухо, без скольжений",
            date="2026-09-01", duration=1320.0, length=4700.0, status="done",
            sourceFile="30618_0e41eac3"),
        Run(title="Вечерний прогон", routeId=1, description="мокро, были пробуксовки",
            date="2026-09-02", duration=1410.0, length=4700.0, status="done",
            sourceFile="30618_27e994fc"),
        Run(title="Тест депо", routeId=3, description="короткий заезд",
            date="2026-09-03", duration=80.0, length=300.0, status="done",
            sourceFile="30618_1551d0a9"),
    ]
    for r in demo_runs:
        runs.add(r)
