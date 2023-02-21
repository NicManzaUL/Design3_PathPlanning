VALID_FLAG_COLORS = ["black", "blue", "green", "red", "white", "yellow", "VIDE"]

class Flag:
    def __init__(self, colorList):
        if len(colorList) != 9:
            return ValueError("Un drapeau doit contenir 9 cases")
        for color in colorList:
            if color not in VALID_FLAG_COLORS:
                return ValueError (f"{color} ne fait pas partie de {VALID_FLAG_COLORS}")
        self.colorList = colorList

    def getColorList(self):
        return self.colorList