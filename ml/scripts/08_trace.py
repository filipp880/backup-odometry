import sys

import numpy as np

from odom_ml import config as C
from odom_ml.data.build import HZ, load_labeled
from odom_ml.estimator import EstimatorConfig, OdomEstimator
from odom_ml.models.traction import TractionModel

bag = sys.argv[1] if len(sys.argv) > 1 else "30618_01f73500"
model = TractionModel.from_npz(C.ARTIFACTS_DIR / "models" / "traction_v1" / "traction.npz")
d = load_labeled(bag, HZ)
est = OdomEstimator(model, EstimatorConfig())
t = d["t"]
v0 = float(np.nanmedian(d["speed"][t <= 2.0]))
est.init_from_gnss(v0, 0.0)
print("init v0", v0)

rows = []
for i in range(t.size):
    st = est.step(float(t[i]), d["u"][i], d["v_front"][i], d["v_rear"][i])
    if i % 25 == 0:
        rows.append((t[i], d["u"][i], d["v_front"][i], d["v_rear"][i], d["speed"][i], st.v, st.k, st.b, st.trust, st.slip))
arr = np.array(rows, dtype=np.float64)
print("   t      u     vf     vr    ref     est     k      b   trust  slip")
for r in arr[:25]:
    print("%6.1f %6.1f %6.2f %6.2f %6.2f %7.2f %6.3f %6.3f %6.2f %6.2f" % tuple(r))
print("...")
for r in arr[25:60:3]:
    print("%6.1f %6.1f %6.2f %6.2f %6.2f %7.2f %6.3f %6.3f %6.2f %6.2f" % tuple(r))
print("...")
for r in arr[-10:]:
    print("%6.1f %6.1f %6.2f %6.2f %6.2f %7.2f %6.3f %6.3f %6.2f %6.2f" % tuple(r))
ref = arr[:, 4]
est_v = arr[:, 5]
m = ref > 0.5
print("\nrmse est=%.3f wheel=%.3f" % (np.sqrt(np.mean((est_v[m] - ref[m]) ** 2)), np.sqrt(np.mean((arr[:, 2][m] - ref[m]) ** 2))))
print("k range", arr[:, 6].min(), arr[:, 6].max(), " b range", arr[:, 7].min(), arr[:, 7].max())
