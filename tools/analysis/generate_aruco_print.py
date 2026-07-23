"""Generate an A4 PDF containing precisely sized DICT_4X4_50 markers.

Print the resulting PDF at 100% / Actual size. Do not use "Fit to page".
"""
from __future__ import annotations

import argparse
from io import BytesIO
from pathlib import Path

import cv2
from PIL import Image
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas
from reportlab.lib.utils import ImageReader


def parse_ids(text: str) -> list[int]:
    result: list[int] = []
    for part in text.split(","):
        part = part.strip()
        if "-" in part:
            first, last = (int(value) for value in part.split("-", 1))
            result.extend(range(first, last + 1))
        else:
            result.append(int(part))
    if not result or any(marker_id < 0 or marker_id > 49 for marker_id in result):
        raise argparse.ArgumentTypeError("DICT_4X4_50 marker IDs must be in 0..49")
    return list(dict.fromkeys(result))


def marker_reader(marker_id: int, pixels: int = 800) -> ImageReader:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    bitmap = cv2.aruco.generateImageMarker(dictionary, marker_id, pixels, borderBits=1)
    buffer = BytesIO()
    Image.fromarray(bitmap).save(buffer, format="PNG")
    buffer.seek(0)
    return ImageReader(buffer)


def crop_marks(pdf: canvas.Canvas, x: float, y: float, size: float) -> None:
    gap, length = 1.5 * mm, 4 * mm
    pdf.setLineWidth(0.2)
    for px, direction in ((x, -1), (x + size, 1)):
        pdf.line(px + direction * gap, y, px + direction * (gap + length), y)
        pdf.line(px + direction * gap, y + size, px + direction * (gap + length), y + size)
    for py, direction in ((y, -1), (y + size, 1)):
        pdf.line(x, py + direction * gap, x, py + direction * (gap + length))
        pdf.line(x + size, py + direction * gap, x + size, py + direction * (gap + length))


def create_pdf(output: Path, ids: list[int], marker_size_mm: float) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    pdf = canvas.Canvas(str(output), pagesize=A4, pageCompression=1)
    page_w, page_h = A4
    size = marker_size_mm * mm
    cell_w, cell_h = 45 * mm, 50 * mm
    columns = 4
    rows = 5
    per_page = columns * rows

    for index, marker_id in enumerate(ids):
        slot = index % per_page
        if index and slot == 0:
            pdf.showPage()
        col, row = slot % columns, slot // columns
        grid_w, grid_h = columns * cell_w, rows * cell_h
        left = (page_w - grid_w) / 2
        top = page_h - (page_h - grid_h) / 2
        x = left + col * cell_w + (cell_w - size) / 2
        y = top - (row + 1) * cell_h + (cell_h - size) / 2 + 3 * mm

        # A white quiet zone is left around each marker by the cell spacing.
        pdf.drawImage(marker_reader(marker_id), x, y, width=size, height=size, mask="auto")
        crop_marks(pdf, x, y, size)
        pdf.setFont("Helvetica", 8)
        pdf.drawCentredString(x + size / 2, y - 5 * mm, f"DICT_4X4_50  ID {marker_id}")
        pdf.setFont("Helvetica", 7)
        pdf.drawCentredString(x + size / 2, y - 8.5 * mm, f"{marker_size_mm:g} x {marker_size_mm:g} mm")

    pdf.setFont("Helvetica-Bold", 10)
    pdf.drawString(15 * mm, 10 * mm, "Print at 100% / Actual size - disable Fit to page")
    # A ruler provides a quick physical verification after printing.
    ruler_x, ruler_y = page_w - 65 * mm, 12 * mm
    pdf.setLineWidth(0.4)
    pdf.line(ruler_x, ruler_y, ruler_x + 50 * mm, ruler_y)
    for value in range(0, 51, 10):
        px = ruler_x + value * mm
        pdf.line(px, ruler_y - 1.5 * mm, px, ruler_y + 1.5 * mm)
        pdf.setFont("Helvetica", 6)
        pdf.drawCentredString(px, ruler_y + 2.5 * mm, str(value))
    pdf.setFont("Helvetica", 7)
    pdf.drawCentredString(ruler_x + 25 * mm, ruler_y - 4 * mm, "50 mm check ruler")
    pdf.save()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ids", type=parse_ids, default=parse_ids("0"),
                        help='marker IDs, e.g. "0", "0,1,7", or "0-9"')
    parser.add_argument("--size-mm", type=float, default=20.0)
    parser.add_argument("--output", type=Path,
                        default=Path("output/pdf/aruco_DICT_4X4_50_20mm.pdf"))
    args = parser.parse_args()
    if not 5 <= args.size_mm <= 40:
        parser.error("--size-mm must be between 5 and 40")
    create_pdf(args.output, args.ids, args.size_mm)
    print(args.output.resolve())


if __name__ == "__main__":
    main()
