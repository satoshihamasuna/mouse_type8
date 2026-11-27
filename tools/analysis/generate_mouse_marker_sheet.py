"""Generate an A4 100%-scale printable PDF of mouse-top markers within 20 mm outer size (including white border).

Includes:
- DICT_4X4_50 ArUco markers (IDs 10/11 for Mouse A, 20/21 for Mouse B; avoiding board IDs 0-8)
  rendered as razor-sharp vector squares with exact outer cut crop marks (20 mm / 18 mm).
- High-contrast asymmetric B/W patterns within 20 mm outer size for motion-blur / 9x9 comparison.
- 100 mm horizontal and 50 mm vertical check rulers.

Print at 100% / Actual size (disable "Fit to page") on matte / non-glossy paper.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas


def aruco_grid_6x6(marker_id: int) -> np.ndarray:
    """Return a 6x6 boolean array (True = black cell, False = white cell) for DICT_4X4_50."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    bitmap = cv2.aruco.generateImageMarker(dictionary, marker_id, 6, borderBits=1)
    if bitmap.shape != (6, 6):
        raise ValueError(f"unexpected marker grid shape: {bitmap.shape}")
    # Verify exact decodability before placing into printable PDF
    padded = np.full((120, 120), 255, dtype=np.uint8)
    scaled = cv2.resize(bitmap, (72, 72), interpolation=cv2.INTER_NEAREST)
    padded[24:96, 24:96] = scaled
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    _, ids, _ = detector.detectMarkers(padded)
    if ids is None or int(ids.flatten()[0]) != marker_id:
        raise RuntimeError(f"ArUco self-check failed for ID {marker_id}")
    return bitmap < 128


def draw_outer_crop_marks(pdf: canvas.Canvas, x: float, y: float, outer_size: float) -> None:
    """Draw thin cut marks strictly outside the outer white-border square."""
    gap, length = 1.2 * mm, 3.8 * mm
    pdf.saveState()
    pdf.setStrokeColorRGB(0.25, 0.25, 0.25)
    pdf.setLineWidth(0.25)
    for px, direction in ((x, -1), (x + outer_size, 1)):
        pdf.line(px + direction * gap, y, px + direction * (gap + length), y)
        pdf.line(px + direction * gap, y + outer_size, px + direction * (gap + length), y + outer_size)
    for py, direction in ((y, -1), (y + outer_size, 1)):
        pdf.line(x, py + direction * gap, x, py + direction * (gap + length))
        pdf.line(x + outer_size, py + direction * gap, x + outer_size, py + direction * (gap + length))
    # Very faint dashed cut guide only at corners is omitted inside; keep white quiet zone pure white.
    pdf.restoreState()


def draw_front_indicator(pdf: canvas.Canvas, cx: float, top_y: float) -> None:
    """Place a small FRONT orientation triangle cleanly outside the white quiet zone."""
    pdf.saveState()
    pdf.setFillColorRGB(0.25, 0.25, 0.25)
    pdf.setStrokeColorRGB(0.25, 0.25, 0.25)
    base_y = top_y + 2.2 * mm
    p = pdf.beginPath()
    p.moveTo(cx, base_y + 2.0 * mm)
    p.lineTo(cx - 1.3 * mm, base_y)
    p.lineTo(cx + 1.3 * mm, base_y)
    p.close()
    pdf.drawPath(p, fill=1, stroke=0)
    pdf.setFont("Helvetica", 6)
    pdf.drawCentredString(cx, base_y + 2.7 * mm, "FRONT")
    pdf.restoreState()


def draw_vector_aruco(
    pdf: canvas.Canvas,
    outer_x: float,
    outer_y: float,
    outer_mm: float,
    body_mm: float,
    marker_id: int,
) -> None:
    """Draw white outer square (pure white) and crisp vector black ArUco cells centered inside."""
    grid = aruco_grid_6x6(marker_id)
    margin_mm = (outer_mm - body_mm) / 2.0
    cell_mm = body_mm / 6.0
    body_x = outer_x + margin_mm * mm
    body_y = outer_y + margin_mm * mm

    pdf.saveState()
    # Pure white background inside outer boundary
    pdf.setFillColorRGB(1, 1, 1)
    pdf.rect(outer_x, outer_y, outer_mm * mm, outer_mm * mm, stroke=0, fill=1)

    # Crisp black vector cells (row 0 is top in image coords -> top in PDF coords)
    pdf.setFillColorRGB(0, 0, 0)
    cell_pt = cell_mm * mm
    # Slight 0.02 pt overlap prevents PDF viewer hairline seams between adjacent black cells
    eps = 0.02
    for row in range(6):
        for col in range(6):
            if grid[row, col]:
                rx = body_x + col * cell_pt
                ry = body_y + (5 - row) * cell_pt
                pdf.rect(rx - eps / 2, ry - eps / 2, cell_pt + eps, cell_pt + eps, stroke=0, fill=1)
    pdf.restoreState()


def draw_bw_arrow_pattern(pdf: canvas.Canvas, outer_x: float, outer_y: float, outer_mm: float) -> None:
    """High-contrast black arrow + rear bar inside a 20 mm white square (large blur-tolerant features)."""
    cx = outer_x + (outer_mm / 2.0) * mm
    cy = outer_y + (outer_mm / 2.0) * mm
    pdf.saveState()
    pdf.setFillColorRGB(1, 1, 1)
    pdf.rect(outer_x, outer_y, outer_mm * mm, outer_mm * mm, stroke=0, fill=1)
    pdf.setFillColorRGB(0, 0, 0)
    # Forward-pointing large triangle (front tip at +7.5 mm from center)
    p = pdf.beginPath()
    p.moveTo(cx, cy + 7.5 * mm)
    p.lineTo(cx - 6.5 * mm, cy - 1.5 * mm)
    p.lineTo(cx + 6.5 * mm, cy - 1.5 * mm)
    p.close()
    pdf.drawPath(p, fill=1, stroke=0)
    # Thick stem / rear bar
    pdf.rect(cx - 3.0 * mm, cy - 7.5 * mm, 6.0 * mm, 6.5 * mm, stroke=0, fill=1)
    pdf.restoreState()


def draw_bw_two_dots_pattern(
    pdf: canvas.Canvas,
    outer_x: float,
    outer_y: float,
    outer_mm: float,
    *,
    inverted: bool = False,
) -> None:
    """High-contrast asymmetric 2-circle pattern (front phi=7.0 mm, rear phi=4.0 mm, span=10.0 mm)."""
    cx = outer_x + (outer_mm / 2.0) * mm
    cy = outer_y + (outer_mm / 2.0) * mm
    pdf.saveState()
    pdf.setFillColorRGB(1, 1, 1)
    pdf.rect(outer_x, outer_y, outer_mm * mm, outer_mm * mm, stroke=0, fill=1)
    if inverted:
        # 18 mm black inner square with 1 mm white outer border, white front/rear circles
        inner = (outer_mm - 2.0) * mm
        pdf.setFillColorRGB(0, 0, 0)
        pdf.rect(outer_x + 1.0 * mm, outer_y + 1.0 * mm, inner, inner, stroke=0, fill=1)
        pdf.setFillColorRGB(1, 1, 1)
    else:
        pdf.setFillColorRGB(0, 0, 0)
    # Front big dot (+4.5 mm), rear small dot (-5.5 mm) -> 10.0 mm center-to-center
    pdf.circle(cx, cy + 4.5 * mm, 3.5 * mm, stroke=0, fill=1)
    pdf.circle(cx, cy - 5.5 * mm, 2.0 * mm, stroke=0, fill=1)
    pdf.restoreState()


def draw_rulers(pdf: canvas.Canvas, page_w: float, page_h: float) -> None:
    """Draw 100 mm horizontal and 50 mm vertical check rulers for verifying 100% scale printing."""
    pdf.saveState()
    pdf.setStrokeColorRGB(0, 0, 0)
    pdf.setFillColorRGB(0, 0, 0)

    # Horizontal 100 mm ruler at bottom-right
    rx, ry = page_w - 115 * mm, 14 * mm
    pdf.setLineWidth(0.5)
    pdf.line(rx, ry, rx + 100 * mm, ry)
    for v in range(0, 101, 5):
        tick = 2.5 * mm if v % 10 == 0 else 1.3 * mm
        px = rx + v * mm
        pdf.line(px, ry - tick, px, ry + tick)
        if v % 10 == 0:
            pdf.setFont("Helvetica", 6)
            pdf.drawCentredString(px, ry + 3.2 * mm, str(v))
    pdf.setFont("Helvetica", 7)
    pdf.drawCentredString(rx + 50 * mm, ry - 5.5 * mm, "100 mm horizontal scale check ruler")

    # Vertical 50 mm ruler on right margin
    vx, vy = page_w - 11 * mm, 32 * mm
    pdf.line(vx, vy, vx, vy + 50 * mm)
    for v in range(0, 51, 10):
        py = vy + v * mm
        pdf.line(vx - 2.0 * mm, py, vx + 2.0 * mm, py)
        pdf.setFont("Helvetica", 6)
        pdf.drawRightString(vx - 2.8 * mm, py - 2.0, str(v))
    pdf.saveState()
    pdf.translate(vx + 4.0 * mm, vy + 25 * mm)
    pdf.rotate(90)
    pdf.setFont("Helvetica", 6.5)
    pdf.drawCentredString(0, 0, "50 mm vertical check")
    pdf.restoreState()

    pdf.restoreState()


def _draw_sheet_page(
    pdf: canvas.Canvas,
    page_w: float,
    page_h: float,
    page_title: str,
    page_subtitle: str,
    rows_spec: list,
    preview_png: Path | None = None,
) -> None:
    pdf.setFont("Helvetica-Bold", 12)
    pdf.drawString(16 * mm, page_h - 16 * mm, page_title)
    pdf.setFont("Helvetica", 8)
    pdf.drawString(16 * mm, page_h - 21.5 * mm, page_subtitle)

    left_margin = 18 * mm
    col_step = 43.5 * mm
    row_top_start = page_h - 33 * mm
    row_step = 59 * mm

    for r_idx, (title, outer_mm, items) in enumerate(rows_spec):
        header_y = row_top_start - r_idx * row_step
        pdf.setFont("Helvetica-Bold", 8.5)
        pdf.setFillColorRGB(0.1, 0.1, 0.1)
        pdf.drawString(16 * mm, header_y, title)

        pdf.saveState()
        pdf.setStrokeColorRGB(0.82, 0.82, 0.82)
        pdf.setLineWidth(0.4)
        pdf.line(16 * mm, header_y - 1.8 * mm, page_w - 18 * mm, header_y - 1.8 * mm)
        pdf.restoreState()

        outer_size = outer_mm * mm
        outer_y = header_y - 12.5 * mm - outer_size

        for c_idx, (kind, arg, body_mm, label) in enumerate(items):
            slot_cx = left_margin + c_idx * col_step + 20 * mm
            outer_x = slot_cx - outer_size / 2.0

            if kind == "aruco":
                draw_vector_aruco(pdf, outer_x, outer_y, outer_mm, body_mm, int(arg))
            elif kind == "arrow":
                draw_bw_arrow_pattern(pdf, outer_x, outer_y, outer_mm)
            elif kind == "dots":
                draw_bw_two_dots_pattern(pdf, outer_x, outer_y, outer_mm, inverted=bool(arg))

            draw_outer_crop_marks(pdf, outer_x, outer_y, outer_size)
            draw_front_indicator(pdf, slot_cx, outer_y + outer_size)

            pdf.setFont("Helvetica", 7.2)
            pdf.setFillColorRGB(0, 0, 0)
            pdf.drawCentredString(slot_cx, outer_y - 7.5 * mm, label)
            sub_txt = f"Cut: {outer_mm:g}x{outer_mm:g} mm" + (f" (body {body_mm:g}mm)" if kind == "aruco" else "")
            pdf.setFont("Helvetica", 6.2)
            pdf.setFillColorRGB(0.3, 0.3, 0.3)
            pdf.drawCentredString(slot_cx, outer_y - 10.7 * mm, sub_txt)

    pdf.setFont("Helvetica-Bold", 8.5)
    pdf.setFillColorRGB(0, 0, 0)
    pdf.drawString(16 * mm, 16 * mm, "Verify printed scale with ruler before cutting!")
    pdf.setFont("Helvetica", 7)
    pdf.drawString(16 * mm, 11.5 * mm, "Tip: Mount flat & matte. Do not trim white border inside crop marks.")
    draw_rulers(pdf, page_w, page_h)

    if preview_png is not None:
        preview_png.parent.mkdir(parents=True, exist_ok=True)
        dpi = 150
        px_per_mm = dpi / 25.4
        w_px, h_px = int(round(210 * px_per_mm)), int(round(297 * px_per_mm))
        img = Image.new("RGB", (w_px, h_px), (255, 255, 255))
        draw = ImageDraw.Draw(img)

        def to_px(x_pt: float, y_pt: float) -> tuple[int, int]:
            return int(round((x_pt / mm) * px_per_mm)), int(round(((page_h - y_pt) / mm) * px_per_mm))

        draw.rectangle([0, 0, w_px - 1, h_px - 1], outline=(180, 180, 180), width=2)
        for r_idx, (_, outer_mm, items) in enumerate(rows_spec):
            header_y = row_top_start - r_idx * row_step
            outer_size = outer_mm * mm
            outer_y = header_y - 12.5 * mm - outer_size
            for c_idx, (kind, arg, body_mm, _) in enumerate(items):
                slot_cx = left_margin + c_idx * col_step + 20 * mm
                outer_x = slot_cx - outer_size / 2.0
                x0, y1 = to_px(outer_x, outer_y)
                x1, y0 = to_px(outer_x + outer_size, outer_y + outer_size)
                draw.rectangle([x0, y0, x1, y1], outline=(200, 200, 200), width=1)
                if kind == "aruco":
                    grid = aruco_grid_6x6(int(arg))
                    margin = (outer_mm - body_mm) / 2.0 * mm
                    cell = (body_mm / 6.0) * mm
                    for rr in range(6):
                        for cc in range(6):
                            if grid[rr, cc]:
                                bx0, by1 = to_px(outer_x + margin + cc * cell, outer_y + margin + (5 - rr) * cell)
                                bx1, by0 = to_px(outer_x + margin + (cc + 1) * cell, outer_y + margin + (6 - rr) * cell)
                                draw.rectangle([bx0, by0, bx1, by1], fill=(0, 0, 0))
                elif kind == "arrow":
                    cx_mm, cy_mm = (outer_x / mm) + outer_mm / 2.0, (outer_y / mm) + outer_mm / 2.0
                    p1 = to_px(cx_mm * mm, (cy_mm + 7.5) * mm)
                    p2 = to_px((cx_mm - 6.5) * mm, (cy_mm - 1.5) * mm)
                    p3 = to_px((cx_mm + 6.5) * mm, (cy_mm - 1.5) * mm)
                    draw.polygon([p1, p2, p3], fill=(0, 0, 0))
                    s0, s1 = to_px((cx_mm - 3.0) * mm, (cy_mm - 7.5) * mm)
                    s2, s3 = to_px((cx_mm + 3.0) * mm, (cy_mm - 1.0) * mm)
                    draw.rectangle([s0, s3, s2, s1], fill=(0, 0, 0))
                elif kind == "dots":
                    cx_mm, cy_mm = (outer_x / mm) + outer_mm / 2.0, (outer_y / mm) + outer_mm / 2.0
                    if arg:
                        ix0, iy1 = to_px(outer_x + 1.0 * mm, outer_y + 1.0 * mm)
                        ix1, iy0 = to_px(outer_x + (outer_mm - 1.0) * mm, outer_y + (outer_mm - 1.0) * mm)
                        draw.rectangle([ix0, iy0, ix1, iy1], fill=(0, 0, 0))
                    fill = (255, 255, 255) if arg else (0, 0, 0)
                    for dy, rad in ((4.5, 3.5), (-5.5, 2.0)):
                        ax0, ay1 = to_px((cx_mm - rad) * mm, (cy_mm + dy - rad) * mm)
                        ax1, ay0 = to_px((cx_mm + rad) * mm, (cy_mm + dy + rad) * mm)
                        draw.ellipse([ax0, ay0, ax1, ay1], fill=fill)
        img.save(preview_png)


def create_mouse_marker_sheet(output_pdf: Path, preview_png: Path | None = None) -> None:
    output_pdf.parent.mkdir(parents=True, exist_ok=True)
    pdf = canvas.Canvas(str(output_pdf), pagesize=A4, pageCompression=1)
    page_w, page_h = A4

    subtitle = (
        "Print at 100% / Actual size (DISABLE Fit to page) on matte paper. Cut along outer crop marks. "
        "Board IDs 0-8 avoided (Mouse A=10/11, B=20/21)."
    )

    page1_rows = [
        (
            "Row 1: DICT_4X4_50  |  Outer 20.0 mm  |  Black body 15.0 mm (1 bit = 2.50 mm, white border = 2.50 mm/side)",
            20.0,
            [
                ("aruco", 10, 15.0, "ID 10  (Mouse A)"),
                ("aruco", 10, 15.0, "ID 10  (Mouse A spare)"),
                ("aruco", 20, 15.0, "ID 20  (Mouse B)"),
                ("aruco", 20, 15.0, "ID 20  (Mouse B spare)"),
            ],
        ),
        (
            "Row 2: DICT_4X4_50  |  Outer 20.0 mm  |  Black body 16.0 mm (1 bit = 2.67 mm, white border = 2.00 mm/side)",
            20.0,
            [
                ("aruco", 10, 16.0, "ID 10  (Mouse A, 16mm)"),
                ("aruco", 10, 16.0, "ID 10  (spare, 16mm)"),
                ("aruco", 20, 16.0, "ID 20  (Mouse B, 16mm)"),
                ("aruco", 20, 16.0, "ID 20  (spare, 16mm)"),
            ],
        ),
        (
            "Row 3: DICT_4X4_50  |  Outer 18.0 mm  |  Black body 13.5 mm (1 bit = 2.25 mm, white border = 2.25 mm/side)",
            18.0,
            [
                ("aruco", 10, 13.5, "ID 10  (A, 18mm)"),
                ("aruco", 11, 13.5, "ID 11  (A alt, 18mm)"),
                ("aruco", 20, 13.5, "ID 20  (B, 18mm)"),
                ("aruco", 21, 13.5, "ID 21  (B alt, 18mm)"),
            ],
        ),
        (
            "Row 4: High-Contrast Asymmetric B/W Patterns (Outer 20.0 mm)  |  For High-Blur & 9x9 Comparison (CNN/Template)",
            20.0,
            [
                ("arrow", None, 20.0, "BW Arrow (20mm)"),
                ("arrow", None, 20.0, "BW Arrow (spare)"),
                ("dots", False, 20.0, "2-Dot Black (7/4mm)"),
                ("dots", True, 20.0, "2-Dot White-on-Black"),
            ],
        ),
    ]

    page2_rows = [
        (
            "Row 1: DICT_4X4_50  |  Outer 22.0 mm  |  Black body 18.0 mm (1 bit = 3.00 mm, white border = 2.00 mm/side)",
            22.0,
            [
                ("aruco", 10, 18.0, "ID 10  (Mouse A, 18/22)"),
                ("aruco", 10, 18.0, "ID 10  (spare, 18/22)"),
                ("aruco", 20, 18.0, "ID 20  (Mouse B, 18/22)"),
                ("aruco", 20, 18.0, "ID 20  (spare, 18/22)"),
            ],
        ),
        (
            "Row 2: DICT_4X4_50  |  Outer 24.0 mm  |  Black body 20.0 mm (1 bit = 3.33 mm, white border = 2.00 mm/side)",
            24.0,
            [
                ("aruco", 10, 20.0, "ID 10  (Mouse A, 20/24)"),
                ("aruco", 10, 20.0, "ID 10  (spare, 20/24)"),
                ("aruco", 20, 20.0, "ID 20  (Mouse B, 20/24)"),
                ("aruco", 20, 20.0, "ID 20  (spare, 20/24)"),
            ],
        ),
        (
            "Row 3: DICT_4X4_50  |  Outer 24.0 mm  |  Black body 18.0 mm (1 bit = 3.00 mm, wide white border = 3.00 mm/side)",
            24.0,
            [
                ("aruco", 10, 18.0, "ID 10  (Mouse A, 18/24)"),
                ("aruco", 10, 18.0, "ID 10  (spare, 18/24)"),
                ("aruco", 20, 18.0, "ID 20  (Mouse B, 18/24)"),
                ("aruco", 20, 18.0, "ID 20  (spare, 18/24)"),
            ],
        ),
        (
            "Row 4: DICT_4X4_50 Alt IDs  |  Outer 24.0 mm  |  Black body 18.0 mm (18/24) & 20.0 mm (20/24)",
            24.0,
            [
                ("aruco", 11, 18.0, "ID 11  (A alt, 18/24)"),
                ("aruco", 11, 20.0, "ID 11  (A alt, 20/24)"),
                ("aruco", 21, 18.0, "ID 21  (B alt, 18/24)"),
                ("aruco", 21, 20.0, "ID 21  (B alt, 20/24)"),
            ],
        ),
    ]

    p1_preview = preview_png.with_stem(preview_png.stem + "_p1") if preview_png is not None else None
    p2_preview = preview_png.with_stem(preview_png.stem + "_p2") if preview_png is not None else None

    _draw_sheet_page(
        pdf,
        page_w,
        page_h,
        "mouse_type8 Top-Mount Marker Test Sheet [Page 1/2: Outer <= 20 mm]",
        subtitle,
        page1_rows,
        preview_png=p1_preview,
    )
    pdf.showPage()
    _draw_sheet_page(
        pdf,
        page_w,
        page_h,
        "mouse_type8 Top-Mount Marker Test Sheet [Page 2/2: Outer 22 mm & 24 mm]",
        subtitle,
        page2_rows,
        preview_png=p2_preview,
    )
    pdf.save()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("output/pdf/mouse_top_markers_test_sheet.pdf"),
        help="Output A4 PDF path",
    )
    parser.add_argument("--preview-png", type=Path, default=None, help="Optional layout preview PNG base path")
    args = parser.parse_args()
    create_mouse_marker_sheet(args.output, preview_png=args.preview_png)
    print(args.output.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
