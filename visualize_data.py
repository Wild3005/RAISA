import pandas as pd
import matplotlib.pyplot as plt

# =========================
# LOAD DATA
# =========================
df_a = pd.read_csv("./logger_csv/akhul_20260116_002411.csv")
df_b = pd.read_csv("./logger_csv/akhul_uwb_20260116_010010.csv")

def extract(df):
    return (
        df["robot_x"].to_numpy(),
        df["robot_y"].to_numpy(),
        df["human_x"].to_numpy(),
        df["human_y"].to_numpy(),
        df["target_x"].to_numpy(),
        df["target_y"].to_numpy()
    )

rx_a, ry_a, hx_a, hy_a, tx_a, ty_a = extract(df_a)
rx_b, ry_b, hx_b, hy_b, tx_b, ty_b = extract(df_b)

# =========================
# PLOT
# =========================
fig, axes = plt.subplots(1, 2, figsize=(14, 6), sharex=True, sharey=True)

# -------- Plot A --------
axes[0].plot(rx_a, ry_a, '-o', label="Robot")
axes[0].plot(hx_a, hy_a, '-s', label="Human")
axes[0].plot(tx_a, ty_a, '--x', label="Target")
axes[0].set_title("CAMERA")
axes[0].set_xlabel("X (m)")
axes[0].set_ylabel("Y (m)")
axes[0].grid(True)
axes[0].axis("equal")
axes[0].legend()

# -------- Plot B --------
axes[1].plot(rx_b, ry_b, '-o', label="Robot")
axes[1].plot(hx_b, hy_b, '-s', label="Human")
axes[1].plot(tx_b, ty_b, '--x', label="Target")
axes[1].set_title("UWB")
axes[1].set_xlabel("X (m)")
axes[1].grid(True)
axes[1].axis("equal")
axes[1].legend()

axes[0].set_aspect("equal", adjustable="box")
axes[1].set_aspect("equal", adjustable="box")

plt.tight_layout()
plt.show()
