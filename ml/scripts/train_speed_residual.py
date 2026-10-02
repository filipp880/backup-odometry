"""Train the residual model on the C++ feature dump and emit the runtime artefact.

Input is one or more ``features_dump.csv`` files produced by the C++ side: the 16
contract features in ``FEATURE_NAMES`` order, three targets, and a column that
identifies the run so the split can be grouped by bag.  Nothing here recomputes
features -- the dump is the contract, and deriving them a second time on the
Python side is exactly how the two implementations would drift apart.

Output is ``ml_model.yaml`` plus ``speed_residual.bin``: 51 little-endian float64
laid out as ``[W (3x16) row-major][b (3)]``.  The runtime contract is

    y = W @ ((x - mean) / std) + b

so the normalisation lives in the descriptor and must be applied by the caller
before the matrix product.

    python ml/scripts/train_speed_residual.py --dump path/to/features_dump*.csv
"""

from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

import numpy as np

# 0-based order, exactly as in the brief.  b_scale is index 7 and slip_index is
# index 8; "column 8" in prose is 1-based and does not mean slip_index here.
FEATURE_NAMES: tuple[str, ...] = (
    "u",              # 0  controller notional / 15, [-1, 1]
    "v",              # 1  filtered body speed, km/h
    "a_model",        # 2  physics model prediction, m/s^2
    "grade",          # 3  path grade, rad (0.0 in the dump: map disabled)
    "mu",             # 4  current adhesion estimate, [0.05, 0.90]
    "omega_front",    # 5  v_front / 0.30, rad/s
    "omega_rear",     # 6  v_rear / 0.30, rad/s
    "b_scale",        # 7  odometry scale, [0.80, 1.25]
    "slip_index",     # 8  slip detector output, [0, 1]
    "trust",          # 9  odometry trust, [0, 1]
    "cmd_rate",       # 10 d(u)/dt, 1/s
    "dt",             # 11 sample period, ~0.02
    "abs_u",          # 12 |u|
    "u_sq",           # 13 u^2
    "v_sq",           # 14 v^2
    "force_ratio",    # 15 |F_drive - F_brake| / adhesion limit, [0, 1]
)

#: column names in the C++ dump
TARGET_COLUMNS: tuple[str, ...] = ("target_a_residual", "target_log_scale", "target_mu")

#: names the runtime uses, from kOutputNames in ml_features.hpp.  The dump
#: prefixes them with target_; the contract does not, and validate_artifact.py
#: compares the descriptor against the header literally, so the two must not be
#: conflated.
TARGET_NAMES: tuple[str, ...] = ("a_residual", "log_scale", "mu")

N_FEATURES = len(FEATURE_NAMES)
N_OUTPUTS = len(TARGET_NAMES)
N_WEIGHTS = N_OUTPUTS * N_FEATURES + N_OUTPUTS  # 3*16 + 3 = 51

#: The C++ side clamps this; training is clipped to match so the model never sees
#: a target it will not be asked to predict.  Raised from 0.80 after measuring the
#: dump: on moving rows 61% of the residual exceeds 0.80, so 0.80 was cutting the
#: regime that decides the correction rather than bounding it.
A_RESIDUAL_LIMIT = 1.50
MU_MIN, MU_MAX = 0.05, 0.90
LOG_SCALE_LIMIT = 0.05

#: speed separating the two regimes, in the feature's own unit (km/h)
STOP_SPEED_KPH = 1.8
#: the standstill rows dominate the dump, so they are down-weighted rather than
#: dropped: the target there is a single constant, and letting it set the fit
#: produced a -0.355 m/s^2 bias on moving data
STANDSTILL_WEIGHT = 0.1
MOVING_WEIGHT = 1.0

GROUP_COLUMNS = ("bag_id", "bag", "run_id", "run", "source_bag", "file")

RIDGE_ALPHA = 1.0
SEED = 42


# --- loading ----------------------------------------------------------------


class DumpError(RuntimeError):
    pass


def resolve_dumps(patterns: list[str]) -> list[Path]:
    paths: list[Path] = []
    for pat in patterns:
        hits = sorted(glob.glob(pat)) or ([pat] if Path(pat).exists() else [])
        if not hits:
            raise DumpError(f"no file matches {pat!r}")
        paths.extend(Path(h) for h in hits)
    out, seen = [], set()
    for p in paths:
        r = p.resolve()
        if r not in seen:
            seen.add(r)
            out.append(p)
    if not out:
        raise DumpError("no dump files given")
    return out


def find_group_column(columns) -> str | None:
    for c in GROUP_COLUMNS:
        if c in columns:
            return c
    return None


HASH_KEYS = ("u", "a_model", "mu", "b_scale", "target_a_residual")


def content_group_id(df) -> str:
    """Group by content, not by file name.

    The vendored dataset contains 122 bag files but only ~72 distinct runs: 50
    files are byte-identical re-dumps under different names. Grouping by bag_id
    put one copy in train and its twin in the held-out fold, so the held-out
    rows were seen verbatim during training.
    """
    import hashlib

    keys = [k for k in HASH_KEYS if k in df.columns]
    blob = df[keys].to_csv(index=False).encode()
    return hashlib.sha256(blob).hexdigest()[:16]


def load_dumps(paths: list[Path], group_col: str | None, group_by_hash: bool = False) -> dict:
    """Read the dumps and decide which targets are actually trainable.

    Two realities of the current dump are handled explicitly rather than guessed at:
    there is no run-identifier column, so the split groups by source file (one file
    is one run), and some targets may be entirely NaN, which must not silently drop
    every row.
    """
    import pandas as pd

    frames = []
    required = (*FEATURE_NAMES, *TARGET_COLUMNS)
    for p in paths:
        df = pd.read_csv(p)
        missing = [c for c in required if c not in df.columns]
        if missing:
            raise DumpError(
                f"{p.name}: missing {len(missing)} required column(s): {missing[:6]}"
                f"{' ...' if len(missing) > 6 else ''}"
            )
        if group_by_hash:
            df["__group__"] = content_group_id(df)
            grouping = "content hash"
        else:
            col = group_col or find_group_column(df.columns)
            if col is not None:
                df["__group__"] = df[col].astype(str)
                grouping = f"column {col!r}"
            elif "t" in df.columns:
                # One dump file is NOT one run. The repaired dumps concatenate
                # several drives, visible as resets in t (measured: 6, 10 and 20
                # runs in the three files). Grouping by filename alone would put
                # neighbouring windows of the same drive on both sides of the
                # split, so the run index is recovered from the resets.
                t = df["t"].to_numpy(dtype=np.float64)
                run_idx = np.cumsum(np.concatenate(([0], np.diff(t) < 0)))
                df["__group__"] = [f"{p.stem}#{int(r)}" for r in run_idx]
                grouping = f"filename + {int(run_idx[-1]) + 1} run(s) split on t resets"
            else:
                df["__group__"] = p.stem
                grouping = "source file name (no t column to split runs on)"
        df["__source__"] = p.name
        extra = ["__group__", "__source__"]
        # Optional, requested by ML_CONTRACT.md: rows whose target hit the export
        # clip are an indicator of saturation, not a correction. Carried only if
        # the dump provides it, so older dumps still load unchanged.
        if "target_clipped" in df.columns:
            df["__clipped__"] = df["target_clipped"].astype(bool)
            extra.append("__clipped__")
            print(f"  {p.name}: has target_clipped flag")
        else:
            print(f"  {p.name}: NO target_clipped column")
        frames.append(df[list(required) + extra])
        print(f"  {p.name}: grouped by {grouping}")
    if not frames:
        raise DumpError("no rows loaded")

    data = pd.concat(frames, ignore_index=True)
    n_before = len(data)

    Y_raw = data[list(TARGET_COLUMNS)].to_numpy(dtype=np.float64)
    finite_per_target = np.isfinite(Y_raw).sum(axis=0)
    available = [j for j in range(N_OUTPUTS) if finite_per_target[j] > 0]
    missing_targets = [TARGET_COLUMNS[j] for j in range(N_OUTPUTS) if finite_per_target[j] == 0]
    if not available:
        raise DumpError(
            f"every target is NaN across all {n_before} rows; nothing can be trained"
        )
    if missing_targets:
        print(
            f"  targets with no data: {', '.join(missing_targets)}"
            f" -> their weights will be emitted as zeros"
        )

    X_all = data[list(FEATURE_NAMES)].to_numpy(dtype=np.float64)
    keep = np.isfinite(X_all).all(axis=1)
    for j in available:
        keep &= np.isfinite(Y_raw[:, j])
    if not keep.all():
        print(
            f"  dropped {n_before - int(keep.sum())} of {n_before} rows with non-finite "
            f"features or targets"
        )
    data = data.loc[keep]
    if not len(data):
        raise DumpError("every row was non-finite")

    return {
        "X": X_all[keep],
        "Y": Y_raw[keep],
        "group": data["__group__"].to_numpy(dtype=object),
        "source": data["__source__"].to_numpy(dtype=object),
    "clipped": (
        data["__clipped__"].to_numpy(dtype=bool)
        if "__clipped__" in data.columns
        else None
    ),
        "available": available,
        "missing_targets": missing_targets,
    }


# --- split ------------------------------------------------------------------


def regime_weights(v_kph: np.ndarray) -> np.ndarray:
    """Down-weight the standstill rows, which are mostly one constant target."""
    return np.where(
        np.asarray(v_kph, dtype=np.float64) <= STOP_SPEED_KPH, STANDSTILL_WEIGHT, MOVING_WEIGHT
    )


def group_folds(groups: np.ndarray, n_splits: int = 4, seed: int = SEED):
    """GroupKFold over runs, with the groups shuffled reproducibly.

    Grouping is mandatory rather than stylistic: the dump holds four runs of the
    same route, so a per-sample split would put neighbouring windows of one drive
    on both sides and the held-out numbers would be meaningless.
    """
    from sklearn.model_selection import GroupKFold

    uniq = sorted(set(groups.tolist()))
    n = len(uniq)
    if n < 2:
        # A single run cannot be split by group.  Fall back to contiguous
        # positional blocks, which for a time-ordered dump are time blocks, so
        # held-out rows still follow rows that were trained on.  Shuffling here
        # would leak: neighbouring samples overlap by construction.
        k = max(2, n_splits)
        bounds = np.linspace(0, groups.shape[0], k + 1).astype(int)
        blocks = [np.arange(bounds[i], bounds[i + 1]) for i in range(k)]
        blocks = [b for b in blocks if b.size]
        folds = []
        for i, te in enumerate(blocks):
            tr = np.concatenate([b for j, b in enumerate(blocks) if j != i])
            folds.append((tr, te))
        return folds, [[f"time-block {i}"] for i in range(len(folds))]
    k = max(2, min(n_splits, n))
    rng = np.random.default_rng(seed)
    order = rng.permutation(n)
    remap = {uniq[old]: uniq[new] for old, new in enumerate(order)}
    shuffled = np.array([remap[g] for g in groups], dtype=object)
    dummy = np.zeros((shuffled.shape[0], 1))
    gkf = GroupKFold(n_splits=k)
    folds = list(gkf.split(dummy, dummy[:, 0], groups=shuffled))
    fold_runs = [sorted({str(shuffled[i]) for i in te}) for _, te in folds]
    return folds, fold_runs


# --- target preparation -----------------------------------------------------


def prepare_targets(Y: np.ndarray, available: list[int]) -> tuple[np.ndarray, dict]:
    """Clip the trainable targets to the runtime limits, and report why.

    Clipping is applied only where the runtime enforces the same limit, and only
    to targets that actually have data.  The unclipped distribution is always
    reported, because that is the evidence for whether a limit is right.
    """
    Yc = np.zeros_like(Y)
    report: dict = {}
    for j in range(N_OUTPUTS):
        name = TARGET_NAMES[j]
        if j not in available:
            report[name] = {"trained": False}
            continue
        raw = Y[:, j]
        entry = {
            "trained": True,
            "p1": float(np.percentile(raw, 1)),
            "p50": float(np.percentile(raw, 50)),
            "p99": float(np.percentile(raw, 99)),
            "abs_max": float(np.abs(raw).max()),
        }
        if j == 0:
            entry["frac_over_limit"] = float((np.abs(raw) > A_RESIDUAL_LIMIT).mean())
            Yc[:, j] = np.clip(raw, -A_RESIDUAL_LIMIT, A_RESIDUAL_LIMIT)
        elif j == 1:
            entry["frac_over_limit"] = float((np.abs(raw) > LOG_SCALE_LIMIT).mean())
            Yc[:, j] = np.clip(raw, -LOG_SCALE_LIMIT, LOG_SCALE_LIMIT)
        else:
            entry["frac_out_of_range"] = float(((raw < MU_MIN) | (raw > MU_MAX)).mean())
            Yc[:, j] = np.clip(raw, MU_MIN, MU_MAX)
        report[name] = entry
    return Yc, report


def b_scale_leakage(X: np.ndarray, Y: np.ndarray) -> dict:
    """How much of log_scale is simply readable back out of b_scale.

    b_scale index 7 is derived from the same odometry the log_scale target
    corrects, so a model handed it can partly echo the answer instead of
    predicting it.  The number belongs in the write-up either way.
    """
    i = FEATURE_NAMES.index("b_scale")
    x, y = X[:, i], Y[:, 1]
    ok = np.isfinite(x) & np.isfinite(y)
    if ok.sum() < 10 or np.std(x[ok]) == 0 or np.std(y[ok]) == 0:
        return {"pearson_r": float("nan"), "n": int(ok.sum())}
    return {
        "pearson_r": float(np.corrcoef(x[ok], y[ok])[0, 1]),
        "n": int(ok.sum()),
    }


# --- model ------------------------------------------------------------------


def fit_linear(
    Xtr: np.ndarray,
    Ytr: np.ndarray,
    alpha: float = RIDGE_ALPHA,
    seed: int = SEED,
    sample_weight: np.ndarray | None = None,
):
    from sklearn.linear_model import Ridge

    model = Ridge(alpha=alpha, fit_intercept=True, random_state=seed)
    model.fit(Xtr, Ytr, sample_weight=sample_weight)
    return model


def evaluate(name: str, pred: np.ndarray, truth: np.ndarray, baseline: np.ndarray) -> dict:
    err = pred - truth
    berr = baseline - truth
    return {
        "name": name,
        "n": int(np.size(truth)),
        "mae": float(np.abs(err).mean()),
        "rmse": float(np.sqrt((err**2).mean())),
        "bias": float(err.mean()),
        "baseline_rmse": float(np.sqrt((berr**2).mean())),
    }


# --- export -----------------------------------------------------------------


def write_weights_bin(path: Path, W: np.ndarray, b: np.ndarray) -> int:
    """Flat little-endian float64: [W (3x16) row-major][b (3)]."""
    if W.shape != (N_OUTPUTS, N_FEATURES):
        raise DumpError(f"expected W of shape {(N_OUTPUTS, N_FEATURES)}, got {W.shape}")
    if b.shape != (N_OUTPUTS,):
        raise DumpError(f"expected b of shape {(N_OUTPUTS,)}, got {b.shape}")
    flat = np.concatenate([W.ravel(order="C"), b]).astype("<f8")
    if flat.size != N_WEIGHTS:
        raise DumpError(f"expected {N_WEIGHTS} weights, produced {flat.size}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(flat.tobytes(order="C"))
    return flat.size


def read_weights_bin(path: Path) -> tuple[np.ndarray, np.ndarray]:
    raw = np.frombuffer(Path(path).read_bytes(), dtype="<f8")
    if raw.size != N_WEIGHTS:
        raise DumpError(f"{path} holds {raw.size} float64, expected {N_WEIGHTS}")
    W = raw[: N_OUTPUTS * N_FEATURES].reshape(N_OUTPUTS, N_FEATURES)
    b = raw[N_OUTPUTS * N_FEATURES :]
    return W, b


def apply_weights(W: np.ndarray, b: np.ndarray, X: np.ndarray, mean: np.ndarray, std: np.ndarray) -> np.ndarray:
    return ((X - mean) / std) @ W.T + b


def write_yaml(path: Path, doc: dict) -> Path:
    """Minimal YAML emitter: the descriptor has no nesting and no quoting needs."""
    lines: list[str] = []

    def scalar(v) -> str:
        if isinstance(v, bool):
            return "true" if v else "false"
        if isinstance(v, float):
            return repr(v)
        if isinstance(v, int):
            return str(v)
        if v is None:
            return "null"
        s = str(v)
        if s == "" or any(ch in s for ch in ":#{}[],&*?|-<>=!%@`\"'\n") or s.strip() != s:
            return json.dumps(s, ensure_ascii=False)
        return s

    def emit(obj, indent: int):
        pad = "  " * indent
        if isinstance(obj, dict):
            for k, v in obj.items():
                if isinstance(v, (dict, list)) and v:
                    lines.append(f"{pad}{k}:")
                    emit(v, indent + 1)
                elif isinstance(v, (dict, list)):
                    lines.append(f"{pad}{k}: {'{}' if isinstance(v, dict) else '[]'}")
                else:
                    lines.append(f"{pad}{k}: {scalar(v)}")
        elif isinstance(obj, list):
            for v in obj:
                if isinstance(v, (dict, list)):
                    lines.append(f"{pad}-")
                    emit(v, indent + 1)
                else:
                    lines.append(f"{pad}- {scalar(v)}")
        else:
            lines.append(f"{pad}{scalar(obj)}")

    emit(doc, 0)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


# --- main -------------------------------------------------------------------


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dump", nargs="+", required=True, help="features_dump*.csv (globs ok)")
    ap.add_argument("--group-col", default=None)
    ap.add_argument("--out", default="models")
    ap.add_argument("--model-id", default="speed_residual_v1")
    ap.add_argument("--alpha", type=float, default=RIDGE_ALPHA)
    ap.add_argument("--seed", type=int, default=SEED)
    ap.add_argument("--folds", type=int, default=4)
    ap.add_argument("--author", default="")
    ap.add_argument("--data-revision", default="")
    ap.add_argument(
        "--drop-b-scale",
        action="store_true",
        help="remove b_scale from the feature vector (breaks the 16-feature contract)",
    )
    ap.add_argument(
        "--filter-wheels",
        action="store_true",
        help="keep only rows with wheels_valid == 1 and a_wheel_clipped == 0",
    )
    ap.add_argument(
        "--min-speed-kph",
        type=float,
        default=0.0,
        help="drop rows below this body speed, km/h (feature 'v')",
    )
    ap.add_argument(
        "--group-by-hash",
        action="store_true",
        help="group by a hash of the feature/target content instead of bag_id, "
        "so byte-identical re-dumps of one run share a fold",
    )
    ap.add_argument(
        "--exclude-bag-pattern",
        default=None,
        help="skip dump files whose name matches this substring",
    )
    ap.add_argument(
        "--max-clip-fraction",
        type=float,
        default=None,
        help="skip dump files whose share of target_clipped rows exceeds this",
    )
    ap.add_argument(
        "--clipped-rows",
        choices=("keep", "drop", "downweight"),
        default="keep",
        help="rows flagged by the target_clipped column: keep (default), drop, "
        "or downweight them, since a saturated target is an indicator rather "
        "than a correction",
    )
    ap.add_argument(
        "--clipped-weight",
        type=float,
        default=0.1,
        help="sample weight for target_clipped rows under --clipped-rows downweight",
    )
    ap.add_argument(
        "--no-clip",
        action="store_true",
        help="train on the raw targets instead of clipping to the runtime limits",
    )
    args = ap.parse_args()

    import datetime as _dt

    paths = resolve_dumps(args.dump)
    print(f"dump files: {len(paths)}")
    for p in paths:
        print(f"  {p.name}")

    # A run whose targets are mostly saturated is an indicator, not a
    # correction, and down-weighting rows inside it still leaves a large block
    # of noise. Drop the whole run instead.
    if args.max_clip_fraction is not None or args.exclude_bag_pattern:
        kept = []
        for p in paths:
            if args.exclude_bag_pattern and args.exclude_bag_pattern in p.name:
                print(f"  excluded by pattern: {p.name}")
                continue
            if args.max_clip_fraction is not None:
                import pandas as _pd

                if "target_clipped" in _pd.read_csv(p, nrows=0).columns:
                    frac = float(_pd.read_csv(p, usecols=["target_clipped"]).iloc[:, 0].mean())
                    if frac > args.max_clip_fraction:
                        print(
                            f"  excluded {p.name}: {100*frac:.1f}% of targets clipped "
                            f"(> {100*args.max_clip_fraction:.0f}%)"
                        )
                        continue
            kept.append(p)
        print(f"dump files after exclusion: {len(kept)}")
        paths = kept

    data = load_dumps(paths, args.group_col, group_by_hash=args.group_by_hash)
    X, Yraw, groups = data["X"], data["Y"], data["group"]
    available = data["available"]
    print(f"\nrows {X.shape[0]}, runs {len(set(groups.tolist()))}, features {X.shape[1]}")
    print(f"trainable targets: {', '.join(TARGET_NAMES[j] for j in available)}")

    if args.filter_wheels:
        import pandas as pd

        # The flag columns are not features, so keep them attached per source file.
        # Newer dumps dropped a_wheel_clipped; read whatever each file actually
        # has rather than dropping the whole file, which silently cost us a
        # 56k-row run before.
        parts = []
        for p in paths:
            head = pd.read_csv(p, nrows=0)
            have = [c for c in ("wheels_valid", "a_wheel_clipped") if c in head.columns]
            part = pd.read_csv(p, usecols=have).assign(__src__=p.name)
            if "wheels_valid" not in part.columns:
                part["wheels_valid"] = 1
            if "a_wheel_clipped" not in part.columns:
                part["a_wheel_clipped"] = 0
            parts.append(part[["wheels_valid", "a_wheel_clipped", "__src__"]])
        flags = pd.concat(parts, ignore_index=True)
        flags = flags[(flags["wheels_valid"] == 1) & (flags["a_wheel_clipped"] == 0)]
        import numpy as _np

        # rebuild the row mask positionally, file by file, in load order
        keep_mask = _np.zeros(X.shape[0], dtype=bool)
        pos = 0
        for p in paths:
            n_file = int((data["source"] == p.name).sum())
            n_keep = int((flags["__src__"] == p.name).sum())
            keep_mask[pos : pos + n_keep] = True
            pos += n_file
        before_total = X.shape[0]
        keep_mask = _np.zeros(before_total, dtype=bool)
        pos = 0
        for p in paths:
            n_file = int((data["source"] == p.name).sum())
            n_keep = int((flags["__src__"] == p.name).sum())
            keep_mask[pos : pos + n_keep] = True
            pos += n_file
        n_kept = int(keep_mask.sum())
        X, Yraw, groups = X[keep_mask], Yraw[keep_mask], groups[keep_mask]
        print(
            f"  filter wheels_valid==1 and a_wheel_clipped==0: "
            f"{n_kept} of {before_total} rows kept ({100*n_kept/before_total:.1f}%)"
        )
        if not len(X):
            raise SystemExit("no rows left after the wheel filter")

    # Rows whose target hit the export clip. Without the flag we cannot tell
    # them apart, so the default is to leave them alone and say so.
    clipped = data["clipped"]
    if clipped is None:
        if args.clipped_rows != "keep":
            raise SystemExit(
                "--clipped-rows needs a target_clipped column, which this dump lacks"
            )
        clipped = np.zeros(X.shape[0], dtype=bool)
    else:
        if not args.filter_wheels:
            clipped = clipped[: X.shape[0]]
        else:
            raise SystemExit(
                "target_clipped handling currently requires --filter-wheels, "
                "which re-indexes the rows"
            )
    if args.clipped_rows == "drop":
        n_before = len(X)
        keep = ~clipped
        X, Yraw, groups, clipped = X[keep], Yraw[keep], groups[keep], clipped[keep]
        print(
            f"  drop target_clipped: {len(X)} of {n_before} rows kept "
            f"({int(clipped.sum())} saturated rows removed)"
        )
        if not len(X):
            raise SystemExit("no rows left after dropping target_clipped rows")
    elif args.clipped_rows == "downweight":
        print(
            f"  downweight target_clipped: {int(clipped.sum())} rows to "
            f"weight {args.clipped_weight}"
        )

    if args.min_speed_kph > 0.0:
        v = X[:, FEATURE_NAMES.index("v")]
        keep = v >= args.min_speed_kph
        print(f"  drop v < {args.min_speed_kph} km/h: {int(keep.sum())} rows kept")
        X, Yraw, groups = X[keep], Yraw[keep], groups[keep]
        if not len(X):
            raise SystemExit("no rows left after the speed filter")

    if args.no_clip:
        Y = Yraw.copy()
        # a target with no data would carry NaN into the solver; it is not
        # trainable, so it is zeroed here and reported as such
        for j in range(N_OUTPUTS):
            if j not in available:
                Y[:, j] = 0.0
        target_report = {
            TARGET_COLUMNS[j]: (
                {
                    "trained": True,
                    "clipped": False,
                    "p1": float(np.percentile(Yraw[:, j], 1)),
                    "p99": float(np.percentile(Yraw[:, j], 99)),
                    "abs_max": float(np.abs(Yraw[:, j]).max()),
                }
                if j in available
                else {"trained": False}
            )
            for j in range(N_OUTPUTS)
        }
        print("  targets NOT clipped (--no-clip)")
    else:
        Y, target_report = prepare_targets(Yraw, available)

    r = target_report.get(TARGET_NAMES[0], {})
    if r.get("trained"):
        print(
            f"\ntarget_a_residual raw: p1 {r['p1']:+.4f}  p99 {r['p99']:+.4f}  "
            f"|max| {r['abs_max']:.4f}"
        )
        if "frac_over_limit" in r:
            print(
                f"  over +/-{A_RESIDUAL_LIMIT}: {r['frac_over_limit']*100:.2f}%"
                f"  -> {'CONFIRMED' if r['frac_over_limit'] == 0 else 'CLAMPING, tails are cut'}"
            )

    if 1 in available:
        leak = b_scale_leakage(X, Y)
        print(f"\nb_scale vs target_log_scale: pearson r = {leak['pearson_r']:+.4f} (n={leak['n']})")
        if abs(leak["pearson_r"]) > 0.5:
            print("  WARNING: strong coupling, so log_scale is partly readable from b_scale.")
            print("           Keep it as a diagnostic, or ask C++ to zero b_scale in the dump.")
    else:
        leak = {"pearson_r": None, "n": 0}
        print("\nb_scale vs target_log_scale: not checked, target_log_scale has no data")

    keep_idx = [i for i in range(N_FEATURES)]
    if args.drop_b_scale:
        drop = FEATURE_NAMES.index("b_scale")
        keep_idx = [i for i in keep_idx if i != drop]
        print(f"  b_scale dropped: training on {len(keep_idx)} features (contract violated)")
    else:
        print("  b_scale kept as a feature: the 16-feature contract is preserved")

    # --- grouped cross-validation, so the metrics come from runs the fit never saw
    folds, fold_runs = group_folds(groups, n_splits=args.folds, seed=args.seed)
    vi = FEATURE_NAMES.index("v")
    weights = regime_weights(X[:, vi])
    if args.clipped_rows == "downweight" and clipped.any():
        weights = weights * np.where(clipped, args.clipped_weight, 1.0)
    print(
        f"\n{len(list(folds))}-fold GroupKFold over {len(set(groups.tolist()))} runs "
        f"(alpha={args.alpha}, seed={args.seed})"
    )
    # Count the regimes from the speed, not from the weight value. With
    # --clipped-weight equal to STANDSTILL_WEIGHT a saturated moving row lands on
    # exactly the same weight as a standstill row, so a weight comparison put
    # 1.6M moving rows into the standstill count and the two splits disagreed.
    v_all = X[:, vi]
    n_moving = int((v_all > STOP_SPEED_KPH).sum())
    n_standstill = int((v_all <= STOP_SPEED_KPH).sum())
    if args.clipped_rows == "downweight" and abs(args.clipped_weight - STANDSTILL_WEIGHT) < 1e-12:
        print(
            "  note: --clipped-weight equals STANDSTILL_WEIGHT, so regime membership is "
            "reported from speed below, not from weight"
        )
    print(
        f"  weights: moving {MOVING_WEIGHT}, standstill {STANDSTILL_WEIGHT} "
        f"(v <= {STOP_SPEED_KPH} km/h); moving rows {n_moving}, "
        f"standstill {n_standstill}"
    )

    oof = np.full((X.shape[0], N_OUTPUTS), np.nan)
    per_fold = []
    for k, (tr, te) in enumerate(folds):
        Xtr, Ytr = X[tr][:, keep_idx], Y[tr]
        mean_k = Xtr.mean(axis=0)
        std_k = Xtr.std(axis=0)
        std_k[std_k < 1e-12] = 1.0
        mk = fit_linear(
            (Xtr - mean_k) / std_k, Ytr, alpha=args.alpha, seed=args.seed,
            sample_weight=weights[tr],
        )
        Wk, bk = mk.coef_.copy(), mk.intercept_.copy()
        for j in range(N_OUTPUTS):
            if j not in available:
                Wk[j, :] = 0.0
                bk[j] = 0.0
        oof[te] = apply_weights(Wk, bk, X[te][:, keep_idx], mean_k, std_k)
        per_fold.append({"fold": k, "held_out_runs": fold_runs[k], "rows": int(len(te))})
        print(f"  fold {k}: held out {fold_runs[k]}, {len(te)} rows")

    ok = np.isfinite(oof[:, 0])
    regimes = {
        "all": ok,
        "standstill (v<=1.8)": ok & (v_all <= STOP_SPEED_KPH),
        "moving (v>1.8)": ok & (v_all > STOP_SPEED_KPH),
    }
    metrics = {"folds": per_fold, "regimes": {}, "trained_outputs": [TARGET_NAMES[j] for j in available]}
    print("\nheld-out metrics (pooled over folds, out-of-fold predictions):")
    print(f"  {'regime':<22}{'n':>7}{'MAE':>10}{'RMSE':>10}{'bias':>10}{'base RMSE':>12}")
    for name, m in regimes.items():
        if not m.any():
            continue
        base = np.full(int(m.sum()), Y[m, 0].mean())
        e = evaluate(name, oof[m, 0], Y[m, 0], base)
        metrics["regimes"][name] = e
        print(
            f"  {name:<22}{e['n']:>7}{e['mae']:>10.4f}{e['rmse']:>10.4f}"
            f"{e['bias']:>+10.4f}{e['baseline_rmse']:>12.4f}"
        )

    # --- final artefact: fitted on every run ---------------------------------
    mean = X[:, keep_idx].mean(axis=0)
    std = X[:, keep_idx].std(axis=0)
    std[std < 1e-12] = 1.0
    final = fit_linear(
        (X[:, keep_idx] - mean) / std, Y, alpha=args.alpha, seed=args.seed,
        sample_weight=weights,
    )
    W = final.coef_.copy()
    bias = final.intercept_.copy()
    for j in range(N_OUTPUTS):
        if j not in available:
            W[j, :] = 0.0
            bias[j] = 0.0
            print(f"  {TARGET_COLUMNS[j]}: no data, weights emitted as zeros")

    out = Path(args.out)
    bin_path = out / "speed_residual.bin"
    n = write_weights_bin(bin_path, W, bias)
    print(f"\nwrote {bin_path}  ({n} float64, {bin_path.stat().st_size} bytes)")

    # reload and re-evaluate through the artefact, not through sklearn
    Wr, br = read_weights_bin(bin_path)
    check = apply_weights(Wr, br, X[:512, keep_idx], mean, std)
    direct = apply_weights(W, bias, X[:512, keep_idx], mean, std)
    diff = float(np.abs(check - direct).max())
    print(f"artefact round-trip: max |diff| = {diff:.3e}")
    if diff > 1e-10:
        raise SystemExit(f"export does not reproduce the model (max diff {diff:.3e} > 1e-10)")
    if not np.isfinite(check).all():
        raise SystemExit("exported model produces non-finite output")

    n_runs = int(len(set(groups.tolist())))
    cv_desc = (
        f"GroupKFold({len(per_fold)}) over {n_runs} runs"
        if n_runs > 1
        else f"leave-one-time-block-out({len(per_fold)}) within 1 run"
    )

    doc = {
        "schema_version": 1,
        "model_id": args.model_id,
        "created_utc": _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "author": args.author,
        "backend": "linear",
        "provenance": {
            "data_revision": args.data_revision,
            "sources": [p.name for p in paths],
            "n_rows_raw": int(X.shape[0]),
            "n_rows_used": int(X.shape[0]),
            "n_runs": n_runs,
            "cv": cv_desc,
            "filter_wheels": bool(args.filter_wheels),
        },
        "weights_file": bin_path.name,
        "weights_layout": "[W (3x16) row-major][b (3)], little-endian float64, 51 values",
        "apply": "y = W @ ((x - mean) / std) + b",
        "features": [FEATURE_NAMES[i] for i in keep_idx],
        "outputs": list(TARGET_NAMES),
        "norm": {
            "mean": [float(v) for v in mean],
            "std": [float(v) for v in std],
        },
        "limits": {
            "max_a_residual": A_RESIDUAL_LIMIT,
            "max_log_scale": LOG_SCALE_LIMIT,
            "mu_min": MU_MIN,
            "mu_max": MU_MAX,
        },
        "train": {
            "backend": "Ridge",
            "alpha": args.alpha,
            "seed": args.seed,
            "data_revision": args.data_revision,
            "b_scale_kept": not args.drop_b_scale,
            "bags": sorted(set(groups.tolist())),
            "cv": f"GroupKFold({len(per_fold)}) over runs",
            "rows_total": int(X.shape[0]),
            "rows_moving": int((weights > STANDSTILL_WEIGHT).sum()),
            "rows_standstill": int((weights < 1.0).sum()),
            "sample_weight_standstill": STANDSTILL_WEIGHT,
            "sample_weight_moving": MOVING_WEIGHT,
            "b_scale_log_scale_pearson_r": leak["pearson_r"],
            "notes": (
                "Trained on the C++ feature dump only. a_model is pure physics; slip_index "
                "is the C++ detector and is not blended with any Python model. grade is 0.0 "
                "throughout because the map is disabled."
            ),
        },
        "target_report": target_report,
        "validation_metrics": metrics,
    }
    yml = write_yaml(out / "ml_model.yaml", doc)
    print(f"wrote {yml}")
    (out / "validation_report.json").write_text(
        json.dumps({"metrics": metrics, "targets": target_report, "leakage": leak}, indent=2),
        encoding="utf-8",
    )
    print(f"wrote {out / 'validation_report.json'}")


if __name__ == "__main__":
    main()
