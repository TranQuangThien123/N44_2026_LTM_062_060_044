import csv
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
NETWORK_CSV = os.path.join(BASE_DIR, "network_report.csv")
ARQ_CSV = os.path.join(BASE_DIR, "arq_report.csv")
OUT_DIR = os.path.join(BASE_DIR, "charts")


def read_csv_rows(path):
    """Đọc toàn bộ dòng dữ liệu từ file CSV (UTF-8)."""
    if not os.path.exists(path):
        return []
    with open(path, "r", encoding="utf-8", errors="replace", newline="") as f:
        return list(csv.DictReader(f))


def shade_outages(ax, elapsed, statuses):
    """Tô nền đỏ nhạt các khoảng thời gian mất kết nối (Failsafe Timeout)."""
    in_outage = False
    start_t = 0.0
    labeled = False
    for t, st in zip(elapsed, statuses):
        is_lost = ("MẤT KẾT NỐI" in st) or ("TIMEOUT" in st)
        if is_lost and not in_outage:
            in_outage = True
            start_t = t
        elif not is_lost and in_outage:
            in_outage = False
            lbl = "Mất kết nối (>3s)" if not labeled else None
            ax.axvspan(start_t, t, color="#ff4d4d", alpha=0.18, label=lbl)
            labeled = True
    if in_outage and elapsed:
        lbl = "Mất kết nối (>3s)" if not labeled else None
        ax.axvspan(start_t, elapsed[-1], color="#ff4d4d", alpha=0.18, label=lbl)


def plot_network_reports(rows):
    """Vẽ bộ biểu đồ từ network_report.csv."""
    elapsed = [float(r["elapsed_sec"]) for r in rows]
    throughput = [float(r["throughput_kbps"]) for r in rows]
    pps = [float(r["packet_rate_pps"]) for r in rows]
    loss_rate = [float(r["loss_rate_percent"]) for r in rows]
    jitter = [float(r["jitter_ms"]) for r in rows]
    altitude = [float(r["altitude_m"]) for r in rows]
    statuses = [r.get("link_status", "") for r in rows]

    # 1. Biểu đồ Jitter & Packet Loss
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
    ax1.plot(elapsed, jitter, color="#1f77b4", linewidth=1.6, label="Jitter RFC 3550 (ms)")
    shade_outages(ax1, elapsed, statuses)
    ax1.set_ylabel("Jitter (ms)")
    ax1.set_title("Biến thiên độ trễ (Jitter) và Tỷ lệ mất gói (Packet Loss) theo thời gian")
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.legend(loc="upper right")

    ax2.plot(elapsed, loss_rate, color="#d62728", linewidth=1.6, label="Packet Loss (%)")
    shade_outages(ax2, elapsed, statuses)
    ax2.set_xlabel("Thời gian (giây)")
    ax2.set_ylabel("Tỷ lệ mất gói (%)")
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(loc="upper right")

    plt.tight_layout()
    out1 = os.path.join(OUT_DIR, "01_network_jitter_loss.png")
    fig.savefig(out1, dpi=180)
    plt.close(fig)
    print(f"[OK] Đã xuất: {out1}")

    # 2. Biểu đồ Throughput & Packet Rate
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
    ax1.plot(elapsed, throughput, color="#2ca02c", linewidth=1.6, label="Throughput (Kbps)")
    shade_outages(ax1, elapsed, statuses)
    ax1.set_ylabel("Băng thông (Kbps)")
    ax1.set_title("Thông lượng mạng (Throughput) và Tốc độ gói tin (PPS) theo thời gian")
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.legend(loc="upper right")

    ax2.plot(elapsed, pps, color="#9467bd", linewidth=1.6, label="Packet Rate (pps)")
    shade_outages(ax2, elapsed, statuses)
    ax2.set_xlabel("Thời gian (giây)")
    ax2.set_ylabel("Gói tin / giây (pps)")
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(loc="upper right")

    plt.tight_layout()
    out2 = os.path.join(OUT_DIR, "02_network_throughput_pps.png")
    fig.savefig(out2, dpi=180)
    plt.close(fig)
    print(f"[OK] Đã xuất: {out2}")

    # 3. Biểu đồ Độ cao UAV
    fig, ax = plt.subplots(figsize=(10, 3.8))
    ax.plot(elapsed, altitude, color="#ff7f0e", linewidth=1.8, label="Độ cao UAV (m)")
    shade_outages(ax, elapsed, statuses)
    ax.set_xlabel("Thời gian (giây)")
    ax.set_ylabel("Độ cao (m)")
    ax.set_title("Hồ sơ độ cao chuyến bay UAV theo thời gian")
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.legend(loc="upper right")

    plt.tight_layout()
    out3 = os.path.join(OUT_DIR, "03_uav_altitude.png")
    fig.savefig(out3, dpi=180)
    plt.close(fig)
    print(f"[OK] Đã xuất: {out3}")


def plot_arq_report(rows):
    """Vẽ biểu đồ hiệu năng Stop-and-Wait ARQ từ arq_report.csv."""
    labels = [f"#{i+1} {r['command_name']}" for i, r in enumerate(rows)]
    rtts = [float(r["rtt_ms"]) for r in rows]
    attempts = [int(r["attempt_count"]) for r in rows]
    results = [r["result"] for r in rows]
    colors = ["#2ca02c" if res == "ACCEPTED" else "#d62728" for res in results]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))

    bars1 = ax1.bar(labels, rtts, color=colors, alpha=0.88)
    ax1.set_ylabel("Thời gian phản hồi RTT (ms)")
    ax1.set_title("Độ trễ thực thi lệnh ARQ (RTT)")
    ax1.tick_params(axis="x", rotation=25)
    ax1.grid(True, axis="y", linestyle="--", alpha=0.5)
    for bar, val in zip(bars1, rtts):
        ax1.text(bar.get_x() + bar.get_width() / 2, val, f"{val:.0f}ms",
                 ha="center", va="bottom", fontsize=8)

    bars2 = ax2.bar(labels, attempts, color=colors, alpha=0.88)
    ax2.set_ylabel("Số lần gửi (1 = không retry)")
    ax2.set_ylim(0, max(attempts + [4]) + 0.8)
    ax2.set_title("Số lần truyền lại Stop-and-Wait ARQ")
    ax2.tick_params(axis="x", rotation=25)
    ax2.grid(True, axis="y", linestyle="--", alpha=0.5)
    for bar, val in zip(bars2, attempts):
        ax2.text(bar.get_x() + bar.get_width() / 2, val, f"{val}",
                 ha="center", va="bottom", fontsize=9)

    plt.tight_layout()
    out4 = os.path.join(OUT_DIR, "04_arq_performance.png")
    fig.savefig(out4, dpi=180)
    plt.close(fig)
    print(f"[OK] Đã xuất: {out4}")


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    net_rows = read_csv_rows(NETWORK_CSV)
    if net_rows:
        plot_network_reports(net_rows)
    else:
        print(f"[!] Chưa có dữ liệu trong {NETWORK_CSV}")

    arq_rows = read_csv_rows(ARQ_CSV)
    if arq_rows:
        plot_arq_report(arq_rows)
    else:
        print(f"[i] Chưa có {ARQ_CSV} (sẽ được tạo sau khi chạy mission_with_network_metrics.exe và gửi lệnh ARQ).")


if __name__ == "__main__":
    main()
