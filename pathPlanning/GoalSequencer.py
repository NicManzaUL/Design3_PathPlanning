import sys
sys.path.append("../")
from flag_utils.flag import Flag

def produceSequence(drapeau: Flag):
    sequence = []
    for i, color in enumerate(drapeau.getColorList()):
        if color != "VIDE":
            sequence.append((color, i))
    return reversed(sequence)