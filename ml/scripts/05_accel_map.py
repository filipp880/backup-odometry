import numpy as np

from odom_ml.data.build import load_labeled, manifest

HZ = 50.0
A_VALID = 2.0


def load_all(min_duration=30.0, vehicles=None):
    man = manifest(HZ)
    out = []
    for row in man["bags"]:
        if row["duration"] < min_duration:
            continue
        if vehicles and row["vehicle"] not in vehicles:
            continue
        d = load_labeled(row["bag_id"], HZ)
        out.append(d)
    return out, man


def robust_accel(a, w=11):
    x = a.copy()
    med = np.copy(x)
    for i in range(x.size):
        lo, hi = max(0, i - w // 2), min(x.size, i + w // 2 + 1)
        med[i] = np.median(x[lo:hi])
    return med


def main():
    bags, man = load_all(min_duration=60.0)
    print(f"bags used: {len(bags)} / {man['count']}, total {sum(b['t'][-1] for b in bags)/3600:.2f} h")

    U = np.concatenate([b["u"] for b in bags])
    V = np.concatenate([b["speed"] for b in bags])
    A = np.concatenate([b["accel"] for b in bags])
    A = robust_accel(A)
    W = np.concatenate([b["v_wheel_mean"] for b in bags])

    ok = np.isfinite(U) & np.isfinite(V) & np.isfinite(A) & (np.abs(A) < A_VALID)
    print(f"samples={U.size} valid={ok.mean():.3f}  |a| p50={np.median(np.abs(A[ok])):.3f} p99={np.percentile(np.abs(A[ok]),99):.3f}")
    print(f"speed vs wheel: corr={np.corrcoef(V[ok], W[ok])[0,1]:.5f}  rmse={np.sqrt(np.mean((V[ok]-W[ok])**2)):.4f} m/s")

    uu = np.clip(np.round(U).astype(int), -15, 15).astype(float)
    print("\nnotch histogram:")
    for u in range(-15, 16):
        n = int((uu == u).sum())
        if n:
            print(f"  u={u:+3d} n={n:8d} ({100*n/uu.size:5.2f}%)", end="")
            if u % 4 == 3 or u == 15:
                print()
            else:
                print("  ", end="")

    print("\naccel(u) median, split by vehicle (all speeds):")
    for veh in ("30618", "30639"):
        sel = [b for b in bags if str(b["vehicle"]) == veh]
        if not sel:
            continue
        Uv = np.concatenate([b["u"] for b in sel])
        Av = np.concatenate([robust_accel(b["accel"]) for b in sel])
        o = np.isfinite(Uv) & np.isfinite(Av) & (np.abs(Av) < A_VALID)
        print(f"  {veh}: n={o.sum()}")
        row = []
        for u in range(-15, 16):
            m = o & (np.clip(np.round(Uv), -15, 15) == u)
            row.append(f"{np.median(Av[m]):+.2f}" if m.sum() > 200 else "  .  ")
        print("    u=-15..15: " + " ".join(row))

    print("\ncoasting (u=0) decel vs speed:")
    for lo, hi in [(0, 1), (1, 3), (3, 5), (5, 8), (8, 11), (11, 15)]:
        m = ok & (uu == 0) & (V >= lo) & (V < hi)
        if m.sum() > 200:
            print(f"  v in [{lo},{hi}): n={m.sum():8d} a={np.median(A[m]):+.4f} m/s2")

    print("\naccel map a(u,v) median [m/s2] rows=u, cols=v bins")
    vbins = np.arange(0, 15.5, 1.5)
    print("  u\\v  " + "".join(f"{b:6.1f}" for b in vbins[:-1]))
    for u in range(-8, 9):
        cells = []
        for lo, hi in zip(vbins[:-1], vbins[1:]):
            m = ok & (uu == u) & (V >= lo) & (V < hi)
            cells.append(f"{np.median(A[m]):+6.2f}" if m.sum() > 300 else "     .")
        print(f"  {u:+3d}  " + "".join(cells))


if __name__ == "__main__":
    main()
