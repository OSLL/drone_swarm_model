#! /usr/bin/python3
# -*- coding: utf-8 -*-
## Primitive check of code validity


## @brief Checks for code syntax validity
class PrimitiveChecker:

    def __init__(self) -> None:
        self._line_number = 0
        self._all_empty = True
        self._errors = []

    ## @brief Ensure that code line contains command with valid syntax
    # @param[in] line string with one command
    # @return True if line is valid, False otherwise
    def _check_line(self, line : str) -> bool:
        self._line_number += 1
        if not line:
            return True
        self._all_empty = False
        commands = ["move", "move_direct", "rotate", "rotate_direct", "translate", "translate_direct"]
        tokens = line.split()
        ok = True
        if tokens[0] not in commands:
            ok = False
        elif (tokens[0] == "move" or tokens[0] == "move_direct") and len(tokens) != 5:
            ok = False            # move[_direct] drone_name x y z
        elif (tokens[0] == "rotate" or tokens[0] == "rotate_direct") and len(tokens) != 5:
            ok = False            # rotate[_direct] drone_name yaw pitch roll
        elif (tokens[0] == "translate" or tokens[0] == "translate_direct") and len(tokens) != 8:
            ok = False            # translate[_direct] drone_name x y z yaw pitch roll
        if ok:
            try:
                all(map(float, tokens[2:]))
            except ValueError:
                ok = False
        if not ok:
            self._errors.append("Line " + str(self._line_number) + ": wrong syntax")
        return ok


    ## @brief Ensure that code is non-empty and contains only commands with valid syntax
    # @param[in] code multiline string with user's code
    # @return True if code is valid, False otherwise
    def check(self, code : str) -> bool:
        self._line_number = 0
        self._all_empty = True
        self._errors = []
        lines = list(map(lambda line : line.strip(), code.splitlines()))
        if len(lines) == 0:
            self._errors.append("Empty solution")
            return False
        ok = True
        for line in lines:
            ok = ok and self._check_line(line)
        if self._all_empty:
            self._errors.append("Empty solution")
            return False
        return ok


    def get_errors(self) -> list:
        return self._errors


if __name__ == '__main__':
    print("This module is not executable")
