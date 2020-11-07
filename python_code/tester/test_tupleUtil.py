import sys
sys.path.append('../')
from tupleUtil import *

position2 = (5,2,4)
position = (1,2,3)

# ------------ WITH INTEGER ------------
def test_plusWithInteger():
    assert (plusWithInteger(position2, 10)) == (15.0, 12.0, 14.0)

def test_minusWithInteger():
    assert (minusWithInteger(position2, 10)) == (-5.0, -8.0, -6.0)

def test_timesWithInteger():
    assert (timesWithInteger(position2, 10)) == (50.0, 20.0, 40.0)

def test_divideWithInteger():
    assert (divideWithInteger(position2, 10)) == (5.0/10, 2.0/10, 4/10)

# ------------ WITH TUPLE ------------
def test_plusWithTuple():
    assert (plusWithTuple(position2, position)) == (6.0, 4.0, 7)

def test_minusWithTuple():
    assert (minusWithTuple(position2, position)) == (4.0, 0, 1.0)

def test_timesWithTuple():
    assert (timesWithTuple(position2, position)) == (5.0, 4.0, 12)

def test_divideWithTuple():
    assert (divideWithTuple(position2, position)) == (5.0, 1.0, 4/3)

# ------------ OTHERS ------------

def test_absoluteTuple():
    assert (absoluteTuple((-6,7,-10))) == (6,7,10)

