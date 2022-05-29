import cv2
import numpy as np
from math import cos,sin,sqrt,pi



class Coordinate():

    def __init__(self, a1_x_coordinate, a1_y_coordinate, a8_x_coordinate, a8_y_coordinate,h1_x_coordinate,h1_y_coordinate, h8_x_coordinate,h8_y_coordinate):
        self.a8_x_coordinate = a8_x_coordinate
        self.a8_y_coordinate = a8_y_coordinate
        self.h1_x_coordinate = h1_x_coordinate
        self.h1_y_coordinate = h1_y_coordinate
        self.a1_x_coordinate = a1_x_coordinate
        self.a1_y_coordinate = a1_y_coordinate
        self.h8_x_coordinate = h8_x_coordinate
        self.h8_y_coordinate = h8_y_coordinate
        #self.diagonal_x = self.h1_x_coordinate - self.a8_x_coordinate
        #self.diagonal_y = self.h1_y_coordinate - self.a8_y_coordinate
        #self.edge_length = sqrt(self.diagonal_y**2 + self.diagonal_x**2)/ sqrt(2)
        #self.square_length = self.edge_length / 7
        #self.a1_x_coordinate = a8_x_coordinate + ((self.diagonal_x * cos(pi / 4) - self.diagonal_y * sin(pi / 4)) / (self.edge_length * sqrt(2))) * self.edge_length
        #self.a1_y_coordinate = a8_y_coordinate + ((self.diagonal_x * sin(pi / 4) + self.diagonal_y * cos(pi / 4)) / (self.edge_length * sqrt(2))) * self.edge_length
        #self.h8_x_coordinate = a8_x_coordinate + ((self.diagonal_x * cos(- pi / 4) - self.diagonal_y * sin(- pi / 4)) / (self.edge_length * sqrt(2))) * self.edge_length
        #self.h8_y_coordinate = a8_y_coordinate + ((self.diagonal_x * sin(- pi / 4) + self.diagonal_y * cos(- pi / 4)) / (self.edge_length * sqrt(2))) * self.edge_length
        self.number_decrease_x = (self.a1_x_coordinate - self.a8_x_coordinate) / 7
        self.number_decrease_y = (self.a1_y_coordinate - self.a8_y_coordinate) / 7
        self.letter_increase_x = (self.h8_x_coordinate - self.a8_x_coordinate) / 7
        self.letter_increase_y = (self.h8_y_coordinate - self.a8_y_coordinate) / 7
        
        
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
            for j in range(8):
                coordinate_list = [0,0]
                coordinate_list[0] = self.a8_x_coordinate + (self.letter_increase_x * j) + (self.number_decrease_x * i)
                coordinate_list[1] = self.a8_y_coordinate + (self.letter_increase_y * j) + (self.number_decrease_y * i)
                coordinates[self.get_square_as_string(i, j)] = coordinate_list


        self.coordinates = coordinates



    def get_piece_coordinates(self, from_square, to_square):
        from_square_coordinates = self.coordinates[from_square]
        to_square_coordinates = self.coordinates[to_square]

        return from_square_coordinates, to_square_coordinates




if __name__ == "__main__":
    A8_X = 0.65078219
    A8_Y = 0.19939440
    H1_X = 0.32620260
    H1_Y = -0.1092154
    coordinate = Coordinate(A8_X, A8_Y, H1_X, H1_Y)
    print(coordinate.coordinates)
