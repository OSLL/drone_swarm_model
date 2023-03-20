#! /usr/bin/python3
# -*- coding: utf-8 -*-
## Primitive check of code validity

## @brief Ensure that code line contains command with valid syntax
# @param[in] line string with one command
# @return True if line is valid, False otherwise
def _primitive_check_line(line : str) -> bool:
    if not line:
        return False
    commands = ["move", "move_direct", "rotate", "rotate_direct", "translate", "translate_direct"]
    tokens = line.split()
    if tokens[0] not in commands:
        return False
    if (tokens[0] == "move" or tokens[0] == "move_direct") and len(tokens) != 5:
        return False            # move[_direct] drone_name x y z
    if (tokens[0] == "rotate" or tokens[0] == "rotate_direct") and len(tokens) != 5:
        return False            # rotate[_direct] drone_name yaw pitch roll
    if (tokens[0] == "translate" or tokens[0] == "translate_direct") and len(tokens) != 8:
        return False            # translate[_direct] drone_name x y z yaw pitch roll
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
