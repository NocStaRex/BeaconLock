"""
evaluation/report_generator.py
================================
Automated ISRO-compliant benchmark audit PDF using ReportLab.

Generates a single-page PDF summary report at session end.

Sections
--------
1. Header  — Logo text, session ID, timestamp.
2. Session Summary Table — key metrics grid.
3. Performance Matrix — pass/fail vs ISRO thresholds.
4. Failure Boundary Notes — honest edge-case documentation.
5. Footer — team / PS-ID.

ISRO SIH-2026 | PS-26169 | AlphaTrion (ID: 176697)
"""

from __future__ import annotations

import logging
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING, Optional

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from evaluation.logger import FrameLogger


# ── ISRO hard-limit thresholds (for pass/fail colouring) ─────────────────
ISRO_THRESHOLDS = {
    "rmse_px":       {"limit": 10.0,  "op": "<=", "unit": "px",  "label": "Tracking Error RMSE"},
    "rmse_mrad":     {"limit": 1.09,  "op": "<=", "unit": "mrad","label": "Angular Error RMSE"},
    "mean_fps":      {"limit": 20.0,  "op": ">=", "unit": "FPS", "label": "System Throughput"},
    "max_slew_deg_s":{"limit": 5.0,   "op": "<=", "unit": "°/s", "label": "Max Slew Rate"},
    "lock_rate":     {"limit": 95.0,  "op": ">=", "unit": "%",   "label": "Lock Retention Rate"},
}

FAILURE_BOUNDARY_NOTES = [
    "Severe fog (α≤0.3) + maximum jitter (±20px): centroid pull toward bright noise clusters; "
    "tracking error may exceed 15 px for beacon sizes < 8 px.",
    "Random-walk motion with rapid direction reversals at > 120 px/s: Kalman coast accuracy "
    "degrades to ~18 px at 10-frame occlusion.",
    "Salt-and-Pepper density ≥ 25%: background estimation (μ+3σ threshold) becomes unreliable; "
    "false-positive detections increase from < 0.1% to ~3%.",
]


class ReportGenerator:
    """
    ReportLab PDF audit report generator.

    Parameters
    ----------
    frame_logger : FrameLogger
        The active or closed logger whose CSV will be summarised.
    session_metadata : dict
        Extra key-value metadata (e.g. motion_type, atmospheric_mode).
    """

    def __init__(
        self,
        frame_logger: "FrameLogger",
        session_metadata: Optional[dict] = None,
    ) -> None:
        self._logger = frame_logger
        self._meta = session_metadata or {}

    def generate(self, output_path: str | Path) -> Path:
        """
        Compute statistics from the CSV and write the PDF.

        Parameters
        ----------
        output_path : str | Path
            Destination PDF file (parent dir created if missing).

        Returns
        -------
        Path to the generated PDF file.

        Raises
        ------
        ImportError   If ReportLab is not installed.
        RuntimeError  If no telemetry rows have been logged.
        """
        try:
            from reportlab.lib import colors
            from reportlab.lib.enums import TA_CENTER, TA_LEFT
            from reportlab.lib.pagesizes import A4
            from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
            from reportlab.lib.units import cm
            from reportlab.platypus import (
                HRFlowable, Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle,
            )
        except ImportError as e:
            raise ImportError(
                "ReportLab is required for PDF generation. "
                "Install it with: pip install reportlab"
            ) from e

        out = Path(output_path)
        out.parent.mkdir(parents=True, exist_ok=True)

        # ── Compute statistics ────────────────────────────────────────
        stats = self._logger.summary()
        if not stats:
            raise RuntimeError("No telemetry data — logger has zero rows.")

        # ── ReportLab story assembly ──────────────────────────────────
        doc = SimpleDocTemplate(
            str(out),
            pagesize=A4,
            rightMargin=2*cm, leftMargin=2*cm,
            topMargin=2*cm,   bottomMargin=2*cm,
        )

        styles = getSampleStyleSheet()
        style_title   = ParagraphStyle("title",   fontSize=18, spaceAfter=4,
                                        textColor=colors.HexColor("#0ea5e9"),
                                        fontName="Helvetica-Bold", alignment=TA_CENTER)
        style_sub     = ParagraphStyle("sub",     fontSize=11, spaceAfter=2,
                                        textColor=colors.HexColor("#64748b"),
                                        fontName="Helvetica", alignment=TA_CENTER)
        style_section = ParagraphStyle("section", fontSize=12, spaceBefore=12, spaceAfter=4,
                                        textColor=colors.HexColor("#1e293b"),
                                        fontName="Helvetica-Bold")
        style_note    = ParagraphStyle("note",    fontSize=8,  spaceAfter=4,
                                        textColor=colors.HexColor("#475569"),
                                        fontName="Helvetica", leading=11)

        # colour helpers
        GREEN  = colors.HexColor("#16a34a")
        RED    = colors.HexColor("#dc2626")
        AMBER  = colors.HexColor("#d97706")
        LBLUE  = colors.HexColor("#e0f2fe")
        DGRAY  = colors.HexColor("#1e293b")
        LGRAY  = colors.HexColor("#f1f5f9")

        story = []

        # ── Header ───────────────────────────────────────────────────
        story.append(Paragraph("🔒 BeaconLock — PAT Simulator Audit Report", style_title))
        story.append(Paragraph(
            f"ISRO SIH-2026 | PS-26169 | AlphaTrion (ID: 176697) | "
            f"Session: <b>{stats.get('session_id','–')}</b>",
            style_sub,
        ))
        story.append(Paragraph(
            f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
            style_sub,
        ))
        story.append(Spacer(1, 0.4*cm))
        story.append(HRFlowable(width="100%", thickness=1,
                                color=colors.HexColor("#cbd5e1")))
        story.append(Spacer(1, 0.3*cm))

        # ── Session summary table ──────────────────────────────────────
        story.append(Paragraph("1. Session Summary", style_section))

        motion_type = self._meta.get("motion_type", "—")
        atmos_mode  = self._meta.get("atmospheric", "clear")
        noise_sigma = self._meta.get("gaussian_sigma", "—")
        input_mode  = self._meta.get("input_mode", "Mode A – Synthetic")

        summary_rows = [
            ["Metric", "Value"],
            ["Session ID",         stats.get("session_id", "—")],
            ["Input Mode",         input_mode],
            ["Motion Type",        motion_type],
            ["Atmospheric Mode",   atmos_mode],
            ["Gaussian σ",         str(noise_sigma)],
            ["Total Frames",       str(stats.get("total_frames", "—"))],
            ["Session Duration",   f"{stats.get('duration_s',0):.2f} s"],
            ["Mean FPS",           f"{stats.get('mean_fps',0):.1f} FPS"],
            ["CSV Path",           stats.get("csv_path", "—")],
        ]

        t_summary = Table(summary_rows, colWidths=[5.5*cm, 11*cm])
        t_summary.setStyle(TableStyle([
            ("BACKGROUND",  (0,0), (-1,0), DGRAY),
            ("TEXTCOLOR",   (0,0), (-1,0), colors.white),
            ("FONTNAME",    (0,0), (-1,0), "Helvetica-Bold"),
            ("FONTSIZE",    (0,0), (-1,-1), 9),
            ("BACKGROUND",  (0,1), (-1,-1), LGRAY),
            ("ROWBACKGROUNDS", (0,1), (-1,-1), [colors.white, LGRAY]),
            ("GRID",        (0,0), (-1,-1), 0.5, colors.HexColor("#cbd5e1")),
            ("LEFTPADDING", (0,0), (-1,-1), 6),
            ("RIGHTPADDING",(0,0), (-1,-1), 6),
            ("TOPPADDING",  (0,0), (-1,-1), 4),
            ("BOTTOMPADDING",(0,0),(-1,-1), 4),
            ("ALIGN",       (0,0), (-1,-1), "LEFT"),
        ]))
        story.append(t_summary)
        story.append(Spacer(1, 0.4*cm))

        # ── Performance matrix ─────────────────────────────────────────
        story.append(Paragraph("2. ISRO Performance Benchmark Matrix", style_section))

        perf_rows = [["Metric", "ISRO Spec", "Measured", "Status"]]
        for key, spec in ISRO_THRESHOLDS.items():
            measured = stats.get(key, None)
            if measured is None:
                status_text = "N/A"
                status_col  = AMBER
            else:
                if spec["op"] == "<=":
                    passed = measured <= spec["limit"]
                else:
                    passed = measured >= spec["limit"]
                status_text = "PASS ✓" if passed else "FAIL ✗"
                status_col  = GREEN if passed else RED

            perf_rows.append([
                spec["label"],
                f"{spec['op']} {spec['limit']} {spec['unit']}",
                f"{measured:.3f} {spec['unit']}" if measured is not None else "—",
                Paragraph(f"<b>{status_text}</b>",
                          ParagraphStyle("s", fontSize=9, textColor=status_col,
                                         fontName="Helvetica-Bold")),
            ])

        t_perf = Table(perf_rows, colWidths=[5*cm, 4*cm, 4.5*cm, 3*cm])
        t_perf.setStyle(TableStyle([
            ("BACKGROUND",  (0,0), (-1,0), DGRAY),
            ("TEXTCOLOR",   (0,0), (-1,0), colors.white),
            ("FONTNAME",    (0,0), (-1,0), "Helvetica-Bold"),
            ("FONTSIZE",    (0,0), (-1,-1), 9),
            ("ROWBACKGROUNDS", (0,1), (-1,-1), [colors.white, LGRAY]),
            ("GRID",        (0,0), (-1,-1), 0.5, colors.HexColor("#cbd5e1")),
            ("LEFTPADDING", (0,0), (-1,-1), 6),
            ("RIGHTPADDING",(0,0), (-1,-1), 6),
            ("TOPPADDING",  (0,0), (-1,-1), 4),
            ("BOTTOMPADDING",(0,0),(-1,-1), 4),
            ("ALIGN",       (0,0), (-1,-1), "LEFT"),
            ("VALIGN",      (0,0), (-1,-1), "MIDDLE"),
        ]))
        story.append(t_perf)
        story.append(Spacer(1, 0.4*cm))

        # ── Failure boundary notes ─────────────────────────────────────
        story.append(Paragraph("3. Honest Failure Boundary Documentation", style_section))
        story.append(Paragraph(
            "The following edge cases were identified during stress-testing. "
            "These are documented openly per competitive requirement (no synthetic 100% pass metrics).",
            style_note,
        ))
        for i, note in enumerate(FAILURE_BOUNDARY_NOTES, 1):
            story.append(Paragraph(f"<b>{i}.</b> {note}", style_note))

        story.append(Spacer(1, 0.4*cm))
        story.append(HRFlowable(width="100%", thickness=1,
                                color=colors.HexColor("#cbd5e1")))
        story.append(Spacer(1, 0.2*cm))

        # ── Footer ────────────────────────────────────────────────────
        story.append(Paragraph(
            "AlphaTrion | Team ID: 176697 | SIH-2026 | PS-26169 | ISRO / Dept. of Space",
            style_sub,
        ))

        # ── Build PDF ─────────────────────────────────────────────────
        doc.build(story)
        logger.info("Audit report saved: %s", out)
        return out
