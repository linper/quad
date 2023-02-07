
from ctypes import *

lib = None
# testlib = CDLL('./mth.so')
# testlib.prompt(20))
# addTwoNumbers = clibrary.add
# addTwoNumbers.argtypes = [ctypes.c_int, ctypes.c_int]
# addTwoNumbers.restype = ctypes.c_int


# def area(p1, p2, p3):
# lib.area.argtypes = [c_bool, c_bool]
# lib.area.restype = c_float

# return liib.area(a, b)


def xor(a, b):
    lib.xor.argtypes = [c_bool, c_bool]
    lib.xor.restype = c_bool

    return lib.xor(a, b)


def init():
    global lib
    lib = CDLL('./mth.so')
