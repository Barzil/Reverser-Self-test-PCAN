"""Standalone entry point for reverser diagnostics (multi-step, pretty console)."""

import logging
import sys

from .pcan_reverser import (
    ReverserDiagnostics,
    ANALOG_CHANNEL_NAMES,
)

logging.basicConfig(level=logging.INFO)
LOG = logging.getLogger(__name__)


def _print_step_summary(results: dict) -> None:
    """Print per-step PASS/FAIL summary."""
    steps = [
        ("Connectivity", results["connectivity"]),
        ("Reset", results["reset"]),
        ("Configuration", results["configuration"]),
        ("Manual", results["manual"]),
        ("Operational", results["operational"]),
        ("Forward (all ON)", results["forward"]),
        ("Reverse (all OFF)", results["reverse"]),
        ("TRC_Supply_SNS Switch", results["trc_supply_switch"]),
    ]

    LOG.info("\n===== STEP SUMMARY =====")
    for idx, (name, res) in enumerate(steps, start=1):
        status = "PASS" if res.get("pass") else "FAIL"
        reason = res.get("reason") or ""
        LOG.info("%d) %s: %s%s", idx, name, status,
                 f" - {reason}" if reason else "")


def _print_channel_table(results: dict) -> None:
    """Print an ASCII table showing per-channel behavior across steps."""
    manual = results["manual"]
    forward = results["forward"]
    reverse = results["reverse"]
    trc_supply = results.get("trc_supply_switch", {})

    manual_vals = results.get("manual_values", {})
    fwd_vals = results.get("forward_values", {})
    rev_vals = results.get("reverse_values", {})
    trc_initial_mv = trc_supply.get("initial_voltage_mv")
    trc_final_mv = trc_supply.get("final_voltage_mv")

    manual_ok = manual.get("per_channel_ok", {})
    fwd_changed = forward.get("per_channel_changed", {})
    rev_back = reverse.get("per_channel_back", {})
    trc_pass = trc_supply.get("pass", False)

    header_channel = "Channel"
    header_manual = "Manual [mV]"
    header_forward = "Forward [mV]"
    header_reverse = "Reverse [mV]"
    header_trc = "TRC_Supply [mV]"
    header_m_ok = "Manual OK"
    header_f_ok = "Changed"
    header_r_ok = "Back"
    header_trc_ok = "TRC OK"
    header_step_m = "Step"
    header_step_f = "Step"
    header_step_r = "Step"
    header_step_trc = "Step"

    name_width = max(len(header_channel), max(len(ch) for ch in ANALOG_CHANNEL_NAMES), len("TRC_Supply_SNS Switch"))
    val_width = max(len(header_manual), len(header_forward), len(header_reverse), len(header_trc), 11)
    flag_width = max(len(header_m_ok), len(header_f_ok), len(header_r_ok), len(header_trc_ok))
    step_width = max(len(header_step_m), len("Manual"), len("Forward"), len("Reverse"), len("TRC Switch"))

    row_fmt = (
        f"{{:<{name_width}}} | {{:>{val_width}}} | {{:<{step_width}}} | "
        f"{{:>{val_width}}} | {{:<{step_width}}} | "
        f"{{:>{val_width}}} | {{:<{step_width}}} | "
        f"{{:>{val_width}}} | {{:<{step_width}}} | "
        f"{{:<{flag_width}}} | {{:<{flag_width}}} | {{:<{flag_width}}} | {{:<{flag_width}}}"
    )

    title = "Per-Channel Diagnostics"
    total_width = (name_width + 3) + 4 * (val_width + 3 + step_width + 3) + 4 * (flag_width + 3)
    border = "=" * max(len(title), total_width)

    lines = []
    lines.append(border)
    lines.append(f" {title}")
    lines.append(border)

    lines.append(
        row_fmt.format(
            header_channel,
            header_manual, header_step_m,
            header_forward, header_step_f,
            header_reverse, header_step_r,
            header_trc, header_step_trc,
            header_m_ok,
            header_f_ok,
            header_r_ok,
            header_trc_ok,
        )
    )

    sep = (
        "-" * name_width + "+-" +
        "-" * val_width + "+-" + "-" * step_width + "+-" +
        "-" * val_width + "+-" + "-" * step_width + "+-" +
        "-" * val_width + "+-" + "-" * step_width + "+-" +
        "-" * val_width + "+-" + "-" * step_width + "+-" +
        "-" * flag_width + "+-" +
        "-" * flag_width + "+-" +
        "-" * flag_width + "+-" +
        "-" * flag_width
    )
    lines.append(sep)

    # Add all analog channels
    for ch in ANALOG_CHANNEL_NAMES:
        mv_m = manual_vals.get(ch)
        mv_f = fwd_vals.get(ch)
        mv_r = rev_vals.get(ch)
        mv_m_str = "—" if mv_m is None else str(mv_m)
        mv_f_str = "—" if mv_f is None else str(mv_f)
        mv_r_str = "—" if mv_r is None else str(mv_r)
        trc_str = "—"

        m_ok = "PASS" if manual_ok.get(ch, False) else "FAIL"
        f_ok = "PASS" if fwd_changed.get(ch, False) else "FAIL"
        r_ok = "PASS" if rev_back.get(ch, False) else "FAIL"
        trc_ok = "—"

        lines.append(
            row_fmt.format(
                ch,
                mv_m_str, "Manual",
                mv_f_str, "Forward",
                mv_r_str, "Reverse",
                trc_str, "—",
                m_ok,
                f_ok,
                r_ok,
                trc_ok,
            )
        )

    # Add TRC_Supply_SNS Switch row
    trc_initial_str = "—" if trc_initial_mv is None else str(trc_initial_mv)
    trc_final_str = "—" if trc_final_mv is None else str(trc_final_mv)
    trc_ok_str = "PASS" if trc_pass else "FAIL"
    
    # Format: show initial/final voltages in the TRC_Supply column
    trc_voltage_str = "—"
    if trc_initial_mv is not None and trc_final_mv is not None:
        trc_voltage_str = f"{trc_initial_str}/{trc_final_str}"
    elif trc_initial_mv is not None:
        trc_voltage_str = str(trc_initial_mv)
    elif trc_final_mv is not None:
        trc_voltage_str = str(trc_final_mv)

    # Add TRC_Supply_SNS Switch row with PASS/FAIL status
    lines.append(
        row_fmt.format(
            "TRC_Supply_SNS Switch",
            "—", "—",
            "—", "—",
            "—", "—",
            trc_voltage_str, "TRC Switch",
            "—",
            "—",
            "—",
            trc_ok_str,  # This shows PASS or FAIL
        )
    )

    LOG.info("\n" + "\n".join(lines))


def main() -> int:
    diag = ReverserDiagnostics()
    try:
        diag.connect()
        LOG.info("Starting full reverser diagnostics...")
        results = diag.run_full_diagnostics()
    finally:
        diag.disconnect()

    _print_step_summary(results)
    _print_channel_table(results)

    final_pass = results.get("final_pass", False)
    LOG.info("\nFINAL RESULT: %s", "PASS" if final_pass else "FAIL")

    return 0 if final_pass else 1


if __name__ == "__main__":
    sys.exit(main())

