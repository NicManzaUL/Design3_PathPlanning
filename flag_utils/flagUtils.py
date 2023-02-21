import cv2
from PIL import Image
import numpy as np
import sys
sys.path.append("./")
from flag_utils.flag import Flag

def hexToCountry(hex):
    try:
        file = open("./drapeaux/listePaysEtCodeHexTabSeparated.txt", "r")
        for line in file.readlines():
            if (line[:2].strip() == hex):
                return line[2:].strip()
    except RuntimeError:
        return LookupError("l'image n'a pas été trouvée")

def countryToFlag(country):
    try:
        pil_image = Image.open(f"./drapeaux/Flag_{country}.gif")
        frame = pil_image.convert("RGB")

        frame.save("converted_image.jpeg")

        frame = cv2.imread("converted_image.jpeg")

        h = w = 96

        regions = [frame[0+10:h//3-10, 0+10:w//3-10], frame[0+10:h//3-10, w//3+10:2*w//3-10], frame[0+10:h//3-10, 2*w//3+10:w-10],
            frame[h//3+10:2*h//3-10, 0+10:w//3-10], frame[h//3+10:2*h//3-10, w//3+10:2*w//3-10], frame[h//3+10:2*h//3-10, 2*w//3+10:w-10],
            frame[2*h//3+10:h-10, 0+10:w//3-10], frame[2*h//3+10:h-10, w//3+10:2*w//3-10], frame[2*h//3+10:h-10, 2*w//3+10:w-10]]

        colors = [ _bgrToColor(tuple(np.uint8(cv2.mean(region))[:3])) for region in regions]

        return Flag(colors)

    except FileNotFoundError:
        return FileNotFoundError("l'image n'a pas été trouvée")


def _bgrToColor(bgr_color):
    if  (225 <= bgr_color[0] <= 235) & (235 <= bgr_color[1] <= 245) & (245 <= bgr_color[2] <= 255):
        return "VIDE"
    if (bgr_color[0] < 50) & (bgr_color[1] < 50) & (bgr_color[2] < 50):
        return "black"
    if (bgr_color[0] < 50) & (bgr_color[1] < 50) & (200 < bgr_color[2]):
        return "red"
    if (bgr_color[0] < 50) & (200 < bgr_color[1]) & (bgr_color[2] < 50):
        return "green"
    if (200 < bgr_color[0]) & (bgr_color[1] < 50) & (bgr_color[2] < 50):
        return "blue"
    if (bgr_color[0] < 50) & (200 < bgr_color[1]) & (200 < bgr_color[2]):
        return "yellow"
    if ( 235 < bgr_color[0] ) & (235 < bgr_color[1]) & (235 < bgr_color[2]):
        return "white"
    else:
        return ValueError("unknown color")