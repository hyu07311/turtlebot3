"""ArUco 마커 인쇄용 A4 PDF 생성 (DICT_4X4_50)

사용법:  python make_markers.py <마커크기mm> [ID ...]
예)      python make_markers.py 80            -> 80 mm, ID 0~3
         python make_markers.py 80 0 1 2 3 4  -> 80 mm, ID 0~4
크기에 맞춰 한 페이지에 들어갈 만큼 배치하고 자동으로 페이지를 나눈다.
인쇄 시 반드시 '실제 크기(100%)'로 인쇄하고, 50 mm 기준선을 자로 확인할 것.
"""
import sys
import cv2
from PIL import Image, ImageDraw, ImageFont

DPI = 300
A4_MM = (210, 297)
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def mm(v):
    return int(round(v / 25.4 * DPI))


def marker_img(mid, px):
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(DICT, mid, px)
    return cv2.aruco.drawMarker(DICT, mid, px)


def fonts():
    try:
        return ImageFont.truetype("arial.ttf", mm(4)), ImageFont.truetype("arial.ttf", mm(3))
    except OSError:
        f = ImageFont.load_default()
        return f, f


def new_page(size_mm):
    pg = Image.new("L", (mm(A4_MM[0]), mm(A4_MM[1])), 255)
    d = ImageDraw.Draw(pg)
    font, small = fonts()
    d.text((mm(15), mm(12)), f"ArUco DICT_4X4_50  |  marker size = {size_mm:g} mm "
           f"(black border outer edge)", font=font, fill=0)
    y = mm(22)  # 50 mm 인쇄 배율 확인선
    d.line([(mm(15), y), (mm(65), y)], fill=0, width=mm(0.4))
    for x in (15, 65):
        d.line([(mm(x), y - mm(2)), (mm(x), y + mm(2))], fill=0, width=mm(0.4))
    d.text((mm(70), y - mm(2)), "<- must measure exactly 50 mm", font=small, fill=0)
    return pg, d


def dashed_box(d, box):
    for i in range(box[0], box[2], mm(3)):
        d.line([(i, box[1]), (min(i + mm(1.5), box[2]), box[1])], fill=150, width=2)
        d.line([(i, box[3]), (min(i + mm(1.5), box[2]), box[3])], fill=150, width=2)
    for j in range(box[1], box[3], mm(3)):
        d.line([(box[0], j), (box[0], min(j + mm(1.5), box[3]))], fill=150, width=2)
        d.line([(box[2], j), (box[2], min(j + mm(1.5), box[3]))], fill=150, width=2)


def make_pages(size_mm, ids):
    margin_mm = max(5.0, size_mm / 6)            # 흰 여백(quiet zone) = 마커 1칸 이상
    cell_w = size_mm + 2 * margin_mm + 5
    cell_h = size_mm + 2 * margin_mm + 12        # 라벨 공간 포함
    area_w, area_h, top = A4_MM[0] - 30, A4_MM[1] - 45, 32
    cols, rows = max(1, int(area_w // cell_w)), max(1, int(area_h // cell_h))
    if size_mm + 2 * margin_mm > area_w:
        raise SystemExit(f"{size_mm} mm 는 A4 한 장에 들어가지 않습니다.")
    font, _ = fonts()
    px, mpx = mm(size_mm), mm(margin_mm)
    pages = []
    for start in range(0, len(ids), cols * rows):
        pg, d = new_page(size_mm)
        for k, mid in enumerate(ids[start:start + cols * rows]):
            cx = mm(15 + (k % cols + 0.5) * area_w / cols)
            cy = mm(top + (k // cols + 0.5) * min(cell_h, area_h / rows))
            pg.paste(Image.fromarray(marker_img(mid, px)), (cx - px // 2, cy - px // 2))
            box = [cx - px // 2 - mpx, cy - px // 2 - mpx, cx + px // 2 + mpx, cy + px // 2 + mpx]
            dashed_box(d, box)
            d.text((box[0], box[3] + mm(2)), f"ID {mid}", font=font, fill=0)
        pages.append(pg)
    return pages


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    size = float(sys.argv[1])
    ids = [int(v) for v in sys.argv[2:]] or [0, 1, 2, 3]
    pages = make_pages(size, ids)
    out = f"aruco_markers_{size:g}mm.pdf"
    pages[0].save(out, save_all=True, append_images=pages[1:], resolution=DPI)
    for n, p in enumerate(pages, 1):
        p.save(f"aruco_markers_{size:g}mm_p{n}.png", dpi=(DPI, DPI))
    print(f"saved: {out} ({len(pages)} page(s))")
