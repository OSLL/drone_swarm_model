from random import random

from grader_config import dist_grade, metric, pos_coeff, photo_coeff
from task_config import time_limit, x_range, y_range, z_range, barriers
from task_solution import positions, photos as made_photos, used_time


class Point:  # stub
    def __init__(self):
        self.x = random()
        self.y = random()
        self.z = random()


class SolutionReader:  # stub for reading from solution log file
    def __init__(self, drone, task):
        self.time = used_time
        self.photos = made_photos
        self.positions = positions


class Grader:
    def __init__(self, drones, task_id, grade_criteria):
        self.drones = drones
        self.task_id = task_id
        self.grade_criteria = grade_criteria
        self.metric = metric
        self.pos_coeff = pos_coeff
        self.photo_coeff = photo_coeff

    def __get_distance(self, p1: Point, p2: Point):
        x = p1.x - p2.x
        y = p1.y - p2.y
        z = p1.z - p2.z
        if self.metric == "chessboard":
            return max(abs(x), abs(y), abs(z))
        elif self.metric == "taxicab":
            return abs(x) + abs(y) + abs(z)
        elif self.metric != "plane":
            print("Unknown metric, proposed euclidean")  # raise?
        return (x**2 + y**2 + z**2)**0.5

    def __get_pos(self, drone):
        return Point()  # stub

    def __get_expected_pos(self, drone):
        return Point()  # stub

    def __get_photos_count(self, drone):
        return int(random()*5)  # stub

    def __get_expected_photos_count(self, drone):
        return 5  # stub

    def __grade_position(self):
        total_grade = 0
        for drone in self.drones:
            pos = self.__get_pos(drone)
            expected_pos = self.__get_expected_pos(drone)
            dist = self.__get_distance(pos, expected_pos)
            keys = list(filter(lambda key: dist <= key, dist_grade.keys()))
            key = min(keys) if keys else None
            grade = dist_grade[key] if key is not None else 0
            total_grade += grade
        return total_grade/len(self.drones)  # return avg

    def __grade_photo(self):
        total_grade = 0
        for drone in self.drones:
            count_photos = self.__get_photos_count(drone)
            expected_photos = self.__get_expected_photos_count(drone)
            grade = count_photos/expected_photos
            total_grade += grade
        return total_grade/len(self.drones)  # return avg

    def grade(self):
        used_pos_coeff = self.pos_coeff \
            if "position" in self.grade_criteria else 0
        used_photo_coeff = self.photo_coeff \
            if "photo" in self.grade_criteria else 0
        return (used_pos_coeff*self.__grade_position()
                + used_photo_coeff*self.__grade_photo()) \
            / (used_pos_coeff + used_photo_coeff)

    def check_solution(self, drone, task):
        solution = SolutionReader(drone, task)
        if not self.__check_time(solution):
            return False
        if not self.__check_borders(solution):
            return False
        if not self.__check_collision(solution):
            return False
        return True

    def __check_borders(self, solution):
        positions = solution.positions
        x = (min([i[0] for i in positions]), max([i[0] for i in positions]))
        if x[0] < x_range[0] or x[1] > x_range[1]:
            return False
        y = (min([i[1] for i in positions]), max([i[1] for i in positions]))
        if y[0] < y_range[0] or y[1] > y_range[1]:
            return False
        z = (min([i[2] for i in positions]), max([i[2] for i in positions]))
        if z[0] < z_range[0] or z[1] > z_range[1]:
            return False
        return True

    def __check_collision(self, solution):
        for position in solution.positions:
            for barrier in barriers:
                check = []
                for ind in range(3):
                    check.append(barrier["borders"][ind][0] - 0.5 < position[ind] <
                                 barrier["borders"][ind][1] + 0.5)  # drone size?
                if not all(check):
                    break
                equation = barrier["equation"]
                if abs(equation[0]*position[0] +
                       equation[1]*position[1] +
                       equation[2]*position[2] - equation[3]) < 0.5:
                    return False
        return True

    def __check_time(self, solution):
        return solution.time <= time_limit


for _ in range(3):
    grader = Grader(['1', '2', '3'], 1, ["photo", "position"])
    print(grader.grade())

