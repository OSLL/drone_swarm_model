#! /usr/bin/python3
# -*- coding: utf-8 -*-

## Unit tests for primitive_checker

import sys

sys.path.append("..")
from primitive_checker import PrimitiveChecker

empty_code = ["", "\t\t   \t\r\n      \r   \n\r"]

wrong_code = [
    "asdasd 19201 sj",
    "line1\nmove drone 0 0 0\nline3",
    "move drone_name 42069",
    "rotate drone_name 1 2 3 4 5 6",
    "move bull sh 1 t",
]


correct_code = [
    "move drone 0 0 0",
    "move_direct     drone 1.34 3.14 346\n move    drone 0120 874 1231",
    " rotate \t\t drone 1 2 3.5",
    "translate drone 1 2 3.5  4 5.76 6  ",
]


def primitive_check(code: str) -> bool:
    checker = PrimitiveChecker()
    return checker.check(code)


def test_empty_code():
    fail = False
    if primitive_check(empty_code[0]):
        print("TEST 1 FAILED: empty code (empty_code[0])", file=sys.stderr)
        fail = True
    if primitive_check(empty_code[1]):
        print(
            "TEST 1 FAILED: empty code with whitespaces (empty_code[1])",
            file=sys.stderr,
        )
        fail = True
    if not fail:
        print("TEST 1 OK")


def test_wrong_commands():
    fail = False
    if primitive_check(wrong_code[0]):
        print("TEST 2 FAILED: wrong code line (wrong_code[0])", file=sys.stderr)
        fail = True
    if primitive_check(wrong_code[1]):
        print(
            "TEST 2 FAILED: several wrong code lines (wrong_code[1])", file=sys.stderr
        )
        fail = True
    if primitive_check(wrong_code[2]):
        print("TEST 2 FAILED: not enough arguments (wrong_code[2])", file=sys.stderr)
        fail = True
    if primitive_check(wrong_code[3]):
        print("TEST 2 FAILED: too many arguments (wrong_code[3])", file=sys.stderr)
        fail = True
    if primitive_check(wrong_code[4]):
        print("TEST 2 FAILED: wrong arguments' format (wrong_code[4])", file=sys.stderr)
        fail = True
    if not fail:
        print("TEST 2 OK")


def test_correct_commands():
    fail = False
    if not primitive_check(correct_code[0]):
        print("TEST 3 FAILED: move to 0 0 0 (correct_code[0])", file=sys.stderr)
        fail = True
    if not primitive_check(correct_code[1]):
        print("TEST 3 FAILED: 2 moves (correct_code[1])", file=sys.stderr)
        fail = True
    if not primitive_check(correct_code[2]):
        print("TEST 3 FAILED: rotate (correct_code[2])", file=sys.stderr)
        fail = True
    if not primitive_check(correct_code[3]):
        print("TEST 3 FAILED: translate (correct_code[3])", file=sys.stderr)
        fail = True
    if not fail:
        print("TEST 3 OK")


test_empty_code()
test_wrong_commands()
test_correct_commands()
