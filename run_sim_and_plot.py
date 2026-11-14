#!/usr/bin/env python3
"""
Chạy loạt mô phỏng GPSR (và DSR làm đường so sánh) rồi vẽ đồ thị theo cách trình bày của bài báo
(Karp & Kung, MobiCom 2000, mục 3).

Tái hiện:
  Hình 9  : tỉ lệ gói gửi thành công theo pause time; GPSR với B = 1.0 / 1.5 / 3.0 và DSR; 50 nút
  Hình 10 : overhead giao thức định tuyến theo pause time (GPSR: beacon; DSR: gói điều khiển)
  Hình 11 : phân bố số chặng vượt quá đường ngắn nhất thật (CHỈ GPSR), 50 nút, pause 0, B = 1.5
  Hình 12 : tỉ lệ gói gửi thành công khi 50 / 112 / 200 nút (cần --scaling)
  Hình 13 : overhead khi 50 / 112 / 200 nút, trục y log      (cần --scaling)
  Mục 3.6 : số neighbor trung bình trong bảng GPSR tại giây 300 (in ra, kèm số của bài)

LƯU Ý QUAN TRỌNG khi so với bài báo:
  - DSR ở đây là DSR của ns-3, KHÔNG phải DSR của ns-2 (CMU) mà bài dùng. Cache route, nghe lén
    và cách xử lý lỗi khác nhau, nên chỉ so xu hướng, không so từng con số.
  - Hình 11 và state per node của DSR không đo được (xem scenario).
  - Ns-3 dùng PHY/MAC khác ns-2 nên số tuyệt đối có thể lệch.

Cách dùng (chạy từ thư mục gốc ns-3, hoặc đặt --ns3-dir):
    python3 run_sim_and_plot.py --simtime 100 --runs 2 --b 1.5   # chạy thử nhanh
    python3 run_sim_and_plot.py --jobs 4                         # Hình 9, 10, 11
    python3 run_sim_and_plot.py --jobs 4 --scaling               # thêm Hình 12, 13 (rất lâu)
    python3 run_sim_and_plot.py --no-dsr                         # chỉ GPSR
    python3 run_sim_and_plot.py --plot-only                      # vẽ lại từ CSV

Kết quả ghi vào CSV sau mỗi lần chạy nên có thể dừng và chạy lại để tiếp tục.
"""

import argparse
import os
import signal
import subprocess
import sys
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import pandas as pd  # noqa: E402

RESULT_FIELDS = ["nNodes", "PauseTime", "RunIndex", "PDR_Gen", "PDR_Flow",
                 "Beacons", "AvgHops", "AvgDelayMs", "PDR_Conn"]
HIST_COLS = ["H0", "H1", "H2", "H3", "H4", "H5"]
RAW_COLUMNS = (["Protocol", "nNodes", "B", "PauseTime", "RunIndex", "PDR_Gen", "PDR_Flow",
                "PDR_Conn", "Beacons", "AvgHops", "AvgDelayMs", "AvgNeighbors"] + HIST_COLS)
KEY_COLUMNS = ["Protocol", "nNodes", "B", "PauseTime", "RunIndex"]

PAPER_PAUSE_TIMES = [0.0, 30.0, 60.0, 120.0]
PAPER_B_VALUES = [1.0, 1.5, 3.0]
PAPER_B_MAIN = 1.5
MARKERS = ["o", "s", "^", "D", "v"]


# ----------------------------------------------------------------------------
# Đọc đầu ra của scenario
# ----------------------------------------------------------------------------
def parse_output(text):
    """Trả về dict gồm RESULT (bắt buộc) + AvgNeighbors và H0..H5 nếu có; hoặc None."""
    out = None
    extra = {}
    for line in text.splitlines():
        line = line.strip()
        parts = line.split(",")
        try:
            if line.startswith("RESULT,") and len(parts) - 1 == len(RESULT_FIELDS):
                out = dict(zip(RESULT_FIELDS, (float(x) for x in parts[1:])))
            elif line.startswith("STATE,") and len(parts) == 5:
                extra["AvgNeighbors"] = float(parts[4])
            elif line.startswith("HOPHIST,") and len(parts) == 5 + len(HIST_COLS):
                for k, v in zip(HIST_COLS, parts[5:]):
                    extra[k] = float(v)
        except ValueError:
            return None
    if out is None:
        return None
    out.update(extra)
    return out


# ----------------------------------------------------------------------------
# Chạy mô phỏng
# ----------------------------------------------------------------------------
def build_jobs(args):
    jobs = []

    def add(proto, n, b, pauses, path_stats=False):
        for pause in pauses:
            for run in range(1, args.runs + 1):
                jobs.append({"Protocol": proto, "nNodes": n, "B": b, "PauseTime": pause,
                             "RunIndex": run,
                             "pathStats": path_stats and pause == 0.0 and b == PAPER_B_MAIN})

    for b in args.b:
        add("gpsr", 50, b, args.pause, path_stats=True)
    if not args.no_dsr:
        add("dsr", 50, 0.0, args.pause)
    if args.scaling:
        add("gpsr", 112, PAPER_B_MAIN, args.pause)
        add("gpsr", 200, PAPER_B_MAIN, [0.0])      # 200 nút: chỉ pause 0, như bài báo
        if not args.no_dsr:
            add("dsr", 112, 0.0, args.pause)
            add("dsr", 200, 0.0, [0.0])
    return jobs


def job_key(j):
    return (str(j["Protocol"]), int(j["nNodes"]), float(j["B"]),
            float(j["PauseTime"]), int(j["RunIndex"]))


def run_one(job, args):
    prog = (f"{args.scenario} --protocol={job['Protocol']} --nNodes={job['nNodes']}"
            f" --pauseTime={job['PauseTime']} --runIndex={job['RunIndex']}"
            f" --simTime={args.simtime}")
    if job["Protocol"] == "gpsr":
        prog += f" --helloInterval={job['B']}"
        if job["pathStats"]:
            prog += " --pathStats=1"
    if args.extra:
        prog += " " + args.extra
    cmd = [os.path.join(args.ns3_dir, "ns3"), "run", "--no-build", prog]
    proc = subprocess.Popen(cmd, cwd=args.ns3_dir, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, start_new_session=True)
    try:
        out, _ = proc.communicate(timeout=args.timeout if args.timeout > 0 else None)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.communicate()
        return None, "timeout"
    res = parse_output(out)
    if res is None:
        tail = " | ".join(out.strip().splitlines()[-3:])
        return None, f"không thấy dòng RESULT (rc={proc.returncode}): {tail}"
    return res, ""


def load_raw(path):
    if os.path.exists(path):
        df = pd.read_csv(path)
        missing = [c for c in RAW_COLUMNS if c not in df.columns]
        if not missing:
            return df
        print(f"! {path} thiếu cột {missing} (CSV từ phiên bản script cũ), bỏ qua file cũ.")
    return pd.DataFrame(columns=RAW_COLUMNS)


def run_all(args, raw_path):
    df = load_raw(raw_path)
    done = {job_key(r) for r in df.to_dict("records")}
    todo = [j for j in build_jobs(args) if job_key(j) not in done]
    total = len(todo)
    print(f"Còn {total} lần chạy (đã có {len(done)}). Scenario: {args.scenario}, "
          f"simTime={args.simtime}s, jobs={args.jobs}")
    if total == 0:
        return df

    # Build một lần rồi chạy --no-build để các tiến trình song song không giẫm lên nhau.
    print("Đang build ns-3 (./ns3 build) ...")
    rc = subprocess.run([os.path.join(args.ns3_dir, "ns3"), "build"], cwd=args.ns3_dir).returncode
    if rc != 0:
        sys.exit("Build thất bại, dừng.")

    lock = threading.Lock()
    new_rows = []
    finished = 0
    with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
        futs = {pool.submit(run_one, j, args): j for j in todo}
        for fut in as_completed(futs):
            j = futs[fut]
            res, err = fut.result()
            finished += 1
            tag = (f"[{finished}/{total}] {j['Protocol']} N={j['nNodes']} B={j['B']} "
                   f"pause={j['PauseTime']:g} run={j['RunIndex']}")
            if res is None:
                print(f"{tag}  LỖI: {err}")
                continue
            row = {c: res.get(c, float("nan")) for c in RAW_COLUMNS}
            row.update({"Protocol": j["Protocol"], "nNodes": int(j["nNodes"]), "B": j["B"],
                        "PauseTime": j["PauseTime"], "RunIndex": int(j["RunIndex"])})
            with lock:
                new_rows.append(row)
                pd.concat([df, pd.DataFrame(new_rows)], ignore_index=True)[RAW_COLUMNS] \
                    .to_csv(raw_path, index=False)
            print(f"{tag}  PDR_bai={res['PDR_Conn']:.2f}%  PDR_lich={res['PDR_Gen']:.2f}%  overhead={int(res['Beacons'])}")
    return pd.concat([df, pd.DataFrame(new_rows)], ignore_index=True) if new_rows else df


# ----------------------------------------------------------------------------
# Tổng hợp và vẽ
# ----------------------------------------------------------------------------
def summarize(raw):
    """Trung bình (và độ lệch chuẩn) qua các lần chạy cho mỗi (giao thức, nút, B, pause)."""
    agg = dict(
        Runs=("RunIndex", "count"),
        PDR_Conn=("PDR_Conn", "mean"), PDR_Conn_std=("PDR_Conn", "std"),
        PDR_Gen=("PDR_Gen", "mean"),
        PDR_Flow=("PDR_Flow", "mean"),
        Beacons=("Beacons", "mean"), Beacons_std=("Beacons", "std"),
        AvgHops=("AvgHops", "mean"), AvgDelayMs=("AvgDelayMs", "mean"),
        AvgNeighbors=("AvgNeighbors", "mean"),
    )
    for c in HIST_COLS:
        agg[c] = (c, "mean")
    s = raw.groupby(["Protocol", "nNodes", "B", "PauseTime"]).agg(**agg).reset_index()
    s["PDR_frac"] = s["PDR_Conn"] / 100.0           # bài báo dùng phân số 0..1
    s["PDR_frac_std"] = s["PDR_Conn_std"] / 100.0
    return s


def _line(ax, sub, ycol, label, marker, errbars, stdcol, style=":", color=None):
    sub = sub.sort_values("PauseTime")
    kw = dict(marker=marker, linestyle=style, label=label)
    if color:
        kw["color"] = color
    if errbars and stdcol in sub and sub[stdcol].notna().any():
        ax.errorbar(sub["PauseTime"], sub[ycol], yerr=sub[stdcol].fillna(0), capsize=3, **kw)
    else:
        ax.plot(sub["PauseTime"], sub[ycol], **kw)


def _finish(fig, ax, xlabel, ylabel, title, path):
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title, fontsize=10)
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"Đã vẽ: {path}")


def plot_fig9_10(s, out_dir, errbars):
    d = s[s["nNodes"] == 50]
    if d.empty:
        return
    g, ds = d[d["Protocol"] == "gpsr"], d[d["Protocol"] == "dsr"]
    for ycol, std, ylabel, fname, title in (
        ("PDR_frac", "PDR_frac_std", "Fraction data pkts delivered",
         "fig9_pdr_vs_pause_50nodes.png", "Packet delivery success rate, 50 nodes (cf. Fig. 9)"),
        ("Beacons", "Beacons_std", "Routing protocol overhead (pkts)",
         "fig10_overhead_vs_pause_50nodes.png", "Routing protocol overhead, 50 nodes (cf. Fig. 10)"),
    ):
        fig, ax = plt.subplots(figsize=(7, 4.5))
        for i, b in enumerate(sorted(g["B"].unique())):
            _line(ax, g[g["B"] == b], ycol, f"GPSR, B = {b:g}", MARKERS[i % 5], errbars, std)
        if not ds.empty:
            _line(ax, ds, ycol, "DSR (ns-3)", "x", errbars, std, style="-", color="black")
        if ycol == "PDR_frac":
            ax.set_ylim(min(0.97, d["PDR_frac"].min() - 0.005), 1.0)  # trục hẹp như Hình 9
        else:
            ax.set_ylim(bottom=0)
        ax.set_xlim(0, 120)
        _finish(fig, ax, "Pause time (s)", ylabel, title, os.path.join(out_dir, fname))


def plot_fig11(s, out_dir):
    d = s[(s["Protocol"] == "gpsr") & (s["nNodes"] == 50) & (s["B"] == PAPER_B_MAIN)
          & (s["PauseTime"] == 0.0)]
    d = d.dropna(subset=["H0"])
    if d.empty:
        return
    r = d.iloc[0]
    fig, ax = plt.subplots(figsize=(6.5, 4.2))
    labels = ["0", "1", "2", "3", "4", ">=5"]
    ax.bar(labels, [r[c] for c in HIST_COLS], color="#4c72b0", label="GPSR (ns-3)")
    ax.set_ylim(0, 1)
    ax.text(0.98, 0.78, "Bài báo: GPSR 97% ở '0'\n(DSR 84.9%, không đo ở đây)",
            transform=ax.transAxes, ha="right", fontsize=8)
    _finish(fig, ax, "Hop count over shortest-path", "Fraction data pkts delivered",
            "Path length beyond optimal, 50 nodes, pause 0 (cf. Fig. 11, GPSR only)",
            os.path.join(out_dir, "fig11_path_length_gpsr.png"))
    print(f"  GPSR giao đúng đường ngắn nhất: {100 * r['H0']:.1f}%  (bài báo: 97%)")


def plot_fig12_13(s, out_dir, errbars):
    g = s[(s["Protocol"] == "gpsr") & (s["B"] == PAPER_B_MAIN)]
    ds = s[s["Protocol"] == "dsr"]
    if g["nNodes"].nunique() < 2:
        return
    for ycol, std, ylabel, fname, logy, title in (
        ("PDR_frac", "PDR_frac_std", "Fraction data pkts delivered",
         "fig12_pdr_scaling.png", False, "Packet delivery success rate (cf. Fig. 12)"),
        ("Beacons", "Beacons_std", "Routing protocol overhead (pkts)",
         "fig13_overhead_scaling.png", True, "Routing protocol overhead (cf. Fig. 13)"),
    ):
        fig, ax = plt.subplots(figsize=(7, 4.5))
        for i, n in enumerate(sorted(g["nNodes"].unique())):
            _line(ax, g[g["nNodes"] == n], ycol, f"GPSR ({n} nodes), B = {PAPER_B_MAIN:g}",
                  MARKERS[i % 5], errbars, std)
            sub = ds[ds["nNodes"] == n]
            if not sub.empty:
                _line(ax, sub, ycol, f"DSR ({n} nodes, ns-3)", MARKERS[i % 5], errbars, std,
                      style="-")
        if logy:
            ax.set_yscale("log")
        ax.set_xlim(-5, 125)
        _finish(fig, ax, "Pause time (s)", ylabel, title, os.path.join(out_dir, fname))


def report_state(s):
    d = s[(s["Protocol"] == "gpsr")].dropna(subset=["AvgNeighbors"])
    d = d[d["PauseTime"] == 0.0]
    if d.empty:
        return
    print("\n=== State mỗi router GPSR (mục 3.6): neighbor trung bình tại giây 300, pause 0 ===")
    for _, r in d.sort_values(["nNodes", "B"]).iterrows():
        print(f"N={int(r['nNodes']):3d} B={r['B']:g}: {r['AvgNeighbors']:.1f} neighbor")
    print("Bài báo: ~20 neighbor với 50 nút (mục 3.2) và 26 với 200 nút (mục 3.6).")


def sanity_report(s, simtime):
    """GPSR gửi beacon định kỳ nên overhead tối đa ~ N x T / B."""
    g = s[s["Protocol"] == "gpsr"]
    print("\n=== Kiểm tra overhead beacon GPSR (cận trên ~ N x T / B) ===")
    for _, r in g.sort_values(["nNodes", "B", "PauseTime"]).iterrows():
        exp = r["nNodes"] * simtime / r["B"]
        print(f"N={int(r['nNodes']):3d} B={r['B']:g} pause={r['PauseTime']:5g}: "
              f"đo {r['Beacons']:10.0f} | cận trên {exp:10.0f} | tỉ lệ {r['Beacons'] / exp:5.2f}")
    print("Tỉ lệ < 1 là bình thường: nút đang gửi dữ liệu reset timer beacon (mục 2.1).")


def print_table(s):
    cols = ["Protocol", "nNodes", "B", "PauseTime", "Runs", "PDR_Conn", "PDR_Gen", "Beacons",
            "AvgHops", "AvgDelayMs"]
    print("\n=== Bảng tổng hợp (trung bình qua các lần chạy) ===")
    print(s[cols].to_string(index=False, float_format=lambda x: f"{x:.3f}"))
    print("PDR_Conn: như bài báo, chỉ tính gói có đường đi lúc gửi -> dùng để vẽ. PDR_Gen: trên mọi gói đã lên lịch (tham khảo)."
          "\nDSR: Beacons = số gói điều khiển DSR; AvgHops/AvgDelayMs = 0 (FlowMonitor không đo được).")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ns3-dir", default=".", help="thư mục gốc ns-3 (chứa ./ns3)")
    ap.add_argument("--scenario", default="scratch/gpsr-paper-scenario")
    ap.add_argument("--pause", type=float, nargs="+", default=PAPER_PAUSE_TIMES)
    ap.add_argument("--b", type=float, nargs="+", default=PAPER_B_VALUES,
                    help="chu kỳ beacon B (Hình 9, 10 dùng 1.0 1.5 3.0)")
    ap.add_argument("--runs", type=int, default=6, help="số lần chạy mỗi cấu hình (bài: 6)")
    ap.add_argument("--simtime", type=float, default=900.0)
    ap.add_argument("--scaling", action="store_true", help="thêm 112 và 200 nút (Hình 12, 13)")
    ap.add_argument("--no-dsr", action="store_true", help="không chạy DSR")
    ap.add_argument("--jobs", type=int, default=1, help="số tiến trình chạy song song")
    ap.add_argument("--timeout", type=float, default=0.0, help="giây/lần chạy, 0 = không giới hạn")
    ap.add_argument("--extra", default="", help='tham số thêm cho scenario, vd "--rtsCts=0"')
    ap.add_argument("--out-dir", default="results")
    ap.add_argument("--errorbars", action="store_true", help="vẽ thêm ±1 độ lệch chuẩn")
    ap.add_argument("--plot-only", action="store_true")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    raw_path = os.path.join(args.out_dir, "simulation_raw_results.csv")
    raw = load_raw(raw_path) if args.plot_only else run_all(args, raw_path)
    if raw.empty:
        sys.exit("Không có dữ liệu để vẽ.")

    s = summarize(raw)
    s.to_csv(os.path.join(args.out_dir, "simulation_summary.csv"), index=False)
    print_table(s)
    sanity_report(s, args.simtime)
    report_state(s)
    plot_fig9_10(s, args.out_dir, args.errorbars)
    plot_fig11(s, args.out_dir)
    plot_fig12_13(s, args.out_dir, args.errorbars)


if __name__ == "__main__":
    main()
