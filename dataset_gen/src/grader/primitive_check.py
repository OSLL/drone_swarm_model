#! /usr/bin/python3
# -*- coding: utf-8 -*-
## Primitive check of code validity

## @brief Ensure that code line contains command with valid syntax
# @param[in] line string with one command
# @return True if line is valid, False otherwise
def _primitive_check_line(line : str) -> bool:
    if not line:
        return False
    tokens = line.split()
    if tokens[0] != "move" and tokens[0] != "rotate" and tokens[0] != "translate":
        return False
    if tokens[0] == "move" and len(tokens) != 5:        # move drone_name x y z
        return False
    if tokens[0] == "rotate" and len(tokens) != 5:      # rotate drone_name yaw pitch roll
        return False
    if tokens[0] == "translate" and len(tokens) != 8:   # translate drone_name x y z yaw pitch roll
        return False
    try:
        all(map(float, tokens[2:]))
        return True
    except ValueError:
        return False


## @brief Ensure that code is non-empty and contains only commands with valid syntax
# @param[in] code multiline string with user's code
# @return True if code is valid, False otherwise
def primitive_check(code : str) -> bool:
    lines = list(filter(lambda line : line.strip(), code.splitlines()))
    return all(map(_primitive_check_line, lines)) if lines else False
