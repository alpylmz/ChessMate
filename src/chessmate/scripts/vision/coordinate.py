import cv2




# Takes a8 square x and y coordinates.
class Coordinate():

    def __init__(self, top_left_x_coordinate, top_left_y_coordinate,square_width):
        self.top_left_x_coordinate = top_left_x_coordinate
        self.top_left_y_coordinate = top_left_y_coordinate
        self.square_width = square_width
        self.get_square_coordinates()


    def get_square_as_string(self, i, j):
        row = 8 - i
        column = ' '
        if j == 0:
            column = "a"
        if j == 1:
            column = "b"
        if j == 2:
            column = "c"
        if j == 3:
            column = "d"
        if j == 4:
            column = "e"
        if j == 5:
            column = "f"
        if j == 6:
            column = "g"
        if j == 7:
            column = "h"

        return column + f'{row}'


    def get_square_coordinates(self):
        # coordinates["a8"] = [x, y]
        # coordinates["f4"] = [x, y]
        coordinates = dict()
        for i in range(8):
            x_coordinate = self.top_left_x_coordinate - self.square_width * i
            y_coordinate = self.top_left_y_coordinate
            for j in range(8):
                coordinate_list = [x_coordinate, y_coordinate]
                coordinates[self.get_square_as_string(i, j)] = coordinate_list
                y_coordinate -= self.square_width

        self.coordinates = coordinates


    def get_piece_coordinates(self, from_square, to_square):
        from_square_coordinates = self.coordinates[from_square]
        to_square_coordinates = self.coordinates[to_square]

        return from_square_coordinates, to_square_coordinates





