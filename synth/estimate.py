#!/usr/bin/env python3
"""Resource estimate per block with Yosys synth_xilinx (Artix-7 mapping).

These are open-source estimates; Vivado's numbers (and Fmax) are the
reference once the design has been implemented (see README).

usage: python3 synth/estimate.py        (run from the repository root)
"""
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
REPORTS = ROOT / "build" / "synth"
SOURCES = sorted(str(p) for d in ("ip/riscv", "ip/kmeans", "rtl") for p in (ROOT / d).glob("*.sv"))
SOURCES.append(str(ROOT / "fpga" / "top_nexys_a7.sv"))
FIRMWARE = ROOT / "build" / "firmware.hex"

BLOCKS = [
    ("RV32I 5-stage pipeline", "riscv_pipeline", ""),
    ("K-means accelerator (core, streamer, memories)", "kmeans_accel", ""),
    ("UART with FIFOs", "soc_uart", ""),
    ("System control", "soc_sysctl", ""),
    ("Whole board design", "top_nexys_a7", f'chparam -set INIT_FILE "{FIRMWARE}" top_nexys_a7;'),
]
CELLS = {"LUT": r"LUT[1-6]", "FF": r"FD[CPRSE]+", "LUTRAM": r"RAM(?:32M|64M|32X1D|64X1D|128X1D|256X1S)",
         "BRAM36": r"RAMB36E1", "BRAM18": r"RAMB18E1", "DSP": r"DSP48E1"}


def synth(top, extra):
    script = (f"read_verilog -defer -sv -I{ROOT}/ip/riscv {' '.join(SOURCES)}; {extra} "
              f"synth_xilinx -family xc7 -top {top} -flatten; stat")
    out = subprocess.run(["yosys", "-p", script], capture_output=True, text=True,
                         check=True, cwd=ROOT).stdout
    (REPORTS / f"{top}.log").write_text(out)
    stat = out[out.rindex("Printing statistics"):]
    return {k: sum(int(n) for _, n in re.findall(rf"^\s+({p})\s+(\d+)$", stat, re.M))
            for k, p in CELLS.items()}


def main():
    REPORTS.mkdir(parents=True, exist_ok=True)
    if not FIRMWARE.exists():
        raise SystemExit("build the firmware first: make -C sw")
    lines = ["| Block | LUTs | FFs | LUTRAM | BRAM36 | BRAM18 | DSP48E1 |",
             "|---|---|---|---|---|---|---|"]
    print("\n".join(lines), flush=True)
    for name, top, extra in BLOCKS:
        c = synth(top, extra)
        lines.append(f"| {name} | {c['LUT']} | {c['FF']} | {c['LUTRAM']} | {c['BRAM36']} "
                     f"| {c['BRAM18']} | {c['DSP']} |")
        print(lines[-1], flush=True)
    (REPORTS / "estimate.md").write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
