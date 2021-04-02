from math import sqrt

# ---------- With Integer ----------
def plusWithInteger(tupleA, sumer):
    return tuple(sumer + x for x in tupleA)


def minusWithInteger(tupleA, substracter):
    return tuple(-substracter + x for x in tupleA)


def timesWithInteger(tupleA, multipler):
    return tuple(multipler*x for x in tupleA)


def divideWithInteger(tupleA, divider):
    return tuple(1/divider*x for x in tupleA)


# ---------- With Tuple ----------
def plusWithTuple(tupleA, tupleB):
    return tuple(ele1 + ele2 for ele1, ele2 in zip(tupleA, tupleB))


def minusWithTuple(tupleA, tupleB):
    return tuple(ele1 - ele2 for ele1, ele2 in zip(tupleA, tupleB))


def timesWithTuple(tupleA, tupleB):
    return tuple(ele1 * ele2 for ele1, ele2 in zip(tupleA, tupleB))


def divideWithTuple(tupleA, tupleB):
    return tuple(ele1 / ele2 for ele1, ele2 in zip(tupleA, tupleB))

# ---------- Others ----------


def absoluteTuple(tupleA):
    return tuple(abs(x) for x in tupleA)


def calculateLength(tupleA):
    # (X, Y, Z)
    return sqrt(tupleA[0]**2 + tupleA[1]**2 + tupleA[2]**2)
