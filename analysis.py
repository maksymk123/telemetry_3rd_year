"""
plot_rssi_vs_range.py
=====================
Plots theoretical Friis free-space RSSI against your measured values.

HOW TO USE:
1. Fill in your measured data in the MEASURED_DATA section below.
2. Run:  python plot_rssi_vs_range.py
3. Two files are saved:  rssi_vs_range.png  (for your dissertation)
                         rssi_vs_range.pdf  (vector, better for LaTeX)

DEPENDENCIES:
    pip install matplotlib numpy
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# =============================================================================
# YOUR MEASURED DATA — fill this in after field testing
# =============================================================================
# Each entry: (distance_m, rssi_dbm, snr_db, packets_received, packets_expected)
# If you didn't measure SNR or packet stats, just put None.

MEASURED_DATA = [
    # (distance_m, rssi_dbm,  snr_db,  rx,  expected)
    (  50,         -72.0,      9.5,    10,   10),
    ( 100,         -78.0,      8.0,    10,   10),
    ( 200,         -85.5,      5.5,    10,   10),
    ( 500,         -96.0,      2.0,     9,   10),
    (1000,         -103.5,    -1.5,     8,   10),
    # Add more rows as needed, e.g.:
    # (2000,  -112.0,  -4.0,   7,  10),
]

# =============================================================================
# SYSTEM PARAMETERS (match your hardware)
# =============================================================================
FREQ_HZ        = 868e6    # 868 MHz
TX_POWER_DBM   = 14       # Heltec SX1262 output power
TX_GAIN_DBI    = 2        # Heltec built-in antenna
RX_GAIN_DBI    = 2        # Heltec built-in antenna
RX_SENSITIVITY = -121     # SX1262 at SF7/BW250 (dBm)

# LoRa settings label (shown in plot legend/title)
LORA_CONFIG = "SF7 / BW250 / CR4/5"

# =============================================================================
# FRIIS CALCULATION
# =============================================================================
def friis_rssi(distance_m, freq_hz, pt_dbm, gt_dbi, gr_dbi):
    """Predicted received power (dBm) via Friis free-space path loss."""
    fspl = 20 * np.log10(distance_m) + 20 * np.log10(freq_hz) - 147.55
    return pt_dbm + gt_dbi + gr_dbi - fspl

# Fine-grained theoretical curve
d_theory = np.logspace(np.log10(10), np.log10(5000), 500)   # 10 m → 5 km
rssi_theory = friis_rssi(d_theory, FREQ_HZ, TX_POWER_DBM, TX_GAIN_DBI, RX_GAIN_DBI)

# Sensitivity limit line
sensitivity_line = np.full_like(d_theory, RX_SENSITIVITY)

# Max theoretical range (where Friis RSSI == sensitivity)
link_budget = TX_POWER_DBM + TX_GAIN_DBI + RX_GAIN_DBI - RX_SENSITIVITY
d_max_log = (link_budget - 20 * np.log10(FREQ_HZ) + 147.55) / 20
d_max_m   = 10 ** d_max_log

# Unpack measured data
meas_d    = np.array([r[0] for r in MEASURED_DATA])
meas_rssi = np.array([r[1] for r in MEASURED_DATA])
meas_snr  = [r[2] for r in MEASURED_DATA]
meas_rx   = [r[3] for r in MEASURED_DATA]
meas_exp  = [r[4] for r in MEASURED_DATA]
pdr       = [rx / ex * 100 if ex else None for rx, ex in zip(meas_rx, meas_exp)]

# Real-world path loss exponent fit (log-distance model)
# RSSI(d) = RSSI(d0) - 10*n*log10(d/d0)
# Fit n using linear regression on measured data
d0        = meas_d[0]
rssi_d0   = friis_rssi(d0, FREQ_HZ, TX_POWER_DBM, TX_GAIN_DBI, RX_GAIN_DBI)
X         = 10 * np.log10(meas_d / d0)
Y         = rssi_d0 - meas_rssi          # measured loss relative to Friis at d0
n_fit     = np.dot(X, Y) / np.dot(X, X)  # least-squares slope = path loss exponent

rssi_logdist = rssi_d0 - n_fit * 10 * np.log10(d_theory / d0)

# =============================================================================
# PLOT
# =============================================================================
fig, axes = plt.subplots(2, 1, figsize=(10, 9), gridspec_kw={"height_ratios": [3, 1]})
fig.patch.set_facecolor("#0d1117")

# ---- Top panel: RSSI vs Distance ----
ax = axes[0]
ax.set_facecolor("#161b22")

# Theoretical Friis
ax.semilogx(d_theory / 1000, rssi_theory, color="#58a6ff", linewidth=2,
            label="Friis free-space model (n = 2.0)", zorder=3)

# Log-distance fit
ax.semilogx(d_theory / 1000, rssi_logdist, color="#f0883e", linewidth=1.8,
            linestyle="--",
            label=f"Log-distance fit (n = {n_fit:.2f})", zorder=3)

# Sensitivity floor
ax.semilogx(d_theory / 1000, sensitivity_line, color="#ff7b72", linewidth=1.2,
            linestyle=":", label=f"RX sensitivity ({RX_SENSITIVITY} dBm)", zorder=2)

# Measured points
sc = ax.scatter(meas_d / 1000, meas_rssi, color="#3fb950", s=90, zorder=5,
                edgecolors="white", linewidths=0.6, label="Measured RSSI")

# Max range annotation
ax.axvline(d_max_m / 1000, color="#8b949e", linewidth=0.8, linestyle="-.")
ax.text(d_max_m / 1000 * 1.05, RX_SENSITIVITY + 3,
        f"Theoretical max\n{d_max_m/1000:.1f} km", color="#8b949e",
        fontsize=8, va="bottom")

# PDR labels on measured points
for i, (d, r, p) in enumerate(zip(meas_d, meas_rssi, pdr)):
    if p is not None:
        ax.annotate(f"PDR {p:.0f}%", xy=(d / 1000, r),
                    xytext=(8, -14), textcoords="offset points",
                    color="#3fb950", fontsize=7.5)

ax.set_xlabel("Distance (km)", color="#c9d1d9", fontsize=11)
ax.set_ylabel("RSSI (dBm)", color="#c9d1d9", fontsize=11)
ax.set_title(f"RSSI vs Distance — SX1262 @ 868 MHz, {LORA_CONFIG}",
             color="#e6edf3", fontsize=13, fontweight="bold", pad=12)

ax.tick_params(colors="#8b949e")
ax.xaxis.set_major_formatter(ticker.FuncFormatter(lambda x, _: f"{x:.2g}"))
ax.spines[:].set_color("#30363d")
ax.grid(True, which="both", color="#21262d", linewidth=0.6, linestyle="-")
ax.legend(facecolor="#161b22", edgecolor="#30363d", labelcolor="#c9d1d9",
          fontsize=9, loc="upper right")

ax.set_xlim([0.03, 6])
ax.set_ylim([-145, -55])

# ---- Bottom panel: SNR vs Distance ----
ax2 = axes[1]
ax2.set_facecolor("#161b22")

snr_vals = [s for s in meas_snr if s is not None]
snr_d    = [meas_d[i] / 1000 for i, s in enumerate(meas_snr) if s is not None]

ax2.semilogx(snr_d, snr_vals, color="#d2a8ff", linewidth=1.5,
             marker="o", markersize=6, markerfacecolor="white",
             markeredgecolor="#d2a8ff", label="Measured SNR")
ax2.axhline(0, color="#ff7b72", linewidth=0.9, linestyle=":",
            label="SNR = 0 dB threshold")

ax2.set_xlabel("Distance (km)", color="#c9d1d9", fontsize=11)
ax2.set_ylabel("SNR (dB)", color="#c9d1d9", fontsize=11)
ax2.set_title("SNR vs Distance", color="#e6edf3", fontsize=11, pad=8)
ax2.tick_params(colors="#8b949e")
ax2.xaxis.set_major_formatter(ticker.FuncFormatter(lambda x, _: f"{x:.2g}"))
ax2.spines[:].set_color("#30363d")
ax2.grid(True, which="both", color="#21262d", linewidth=0.6)
ax2.legend(facecolor="#161b22", edgecolor="#30363d", labelcolor="#c9d1d9",
           fontsize=9)
ax2.set_xlim([0.03, 6])

plt.tight_layout(pad=2.0)

# Save
plt.savefig("rssi_vs_range.png", dpi=200, bbox_inches="tight",
            facecolor=fig.get_facecolor())
plt.savefig("rssi_vs_range.pdf", bbox_inches="tight",
            facecolor=fig.get_facecolor())

print("Saved: rssi_vs_range.png  and  rssi_vs_range.pdf")
print(f"\nFitted path loss exponent n = {n_fit:.2f}")
print(f"  (free space = 2.0, urban typical = 2.7–3.5, heavy obstructions = 4+)")
print(f"\nTheoretical max range (SF7, free space): {d_max_m/1000:.1f} km")

plt.show()
