# -*- coding: utf-8 -*-
"""
Created on May 24 2021

@author: Wei-Yu Chen
#Purpose: Calculate NAI matrix for matlab LP solver then plot the result from the LP solver
#Ref:
@INPROCEEDINGS{7743548,
  author={Bochkarev, Stanislav and Smith, Stephen L.},
  booktitle={2016 IEEE International Conference on Automation Science and Engineering (CASE)},
  title={On minimizing turns in robot coverage path planning},
  year={2016},
  volume={},
  number={},
  pages={1237-1242},
  doi={10.1109/COASE.2016.7743548}}
@misc{https://doi.org/10.48550/arxiv.2109.08185,
  doi = {10.48550/ARXIV.2109.08185},
  url = {https://arxiv.org/abs/2109.08185},
  author = {Ramesh, Megnath and Imeson, Frank and Fidan, Baris and Smith, Stephen L.},
  title = {Optimal Partitioning of Non-Convex Environments for Minimum Turn Coverage Planning},
  publisher = {arXiv},
  year = {2021},
  copyright = {arXiv.org perpetual, non-exclusive license}}

"""
import csv

import numpy as np
from scipy.io import savemat
import matplotlib.pyplot as plt
import pandas as pd


def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1


# a b are lists which contains start and end coordinates of a line
def neighborLine(a, b):
    return neighbors(a[1], b[0])


def neighbors(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def minus(a, b):
    return [a[0] - b[0], a[1] - b[1]]


def distParallelLine(a, b):
    if a[0][0] == a[1][0]:
        return abs(b[0][0] - a[0][0])
    elif a[0][1] == a[1][1]:
        return abs(b[0][1] - a[0][1])


# right first then down
class Grid:
    def __init__(self, start, end, unpassable):
        self.start = start
        self.end = end
        self.unpassable = unpassable
        self.gtsp_set = None
        self.squares = []
        self.grid_num = (end[1] - start[1] + 1) * (end[0] - start[0] + 1) - len(unpassable)
        self.naiH = []
        self.naiV = []
        self.resulting_grid = []
        self.xh = []
        self.transition_segements = []
        self.tsp_cost = []

    # generate NAI matrix for horizontal and vertical graph
    def findNAI(self):
        counter = 0
        for i in range(start[0], end[0] + 1):
            for j in range(start[1], end[1] + 1):
                if [i, j] not in self.unpassable:
                    point = Square([i, j], self.end, self.unpassable, counter)
                    self.squares.append(point)
                    naiH_row = []
                    naiV_row = []
                    for k in range(self.grid_num):
                        if point.right:
                            if counter == k:
                                naiH_row.append(1)
                            else:
                                naiH_row.append(0)
                        else:
                            naiH_row.append(0)
                        if point.down:
                            if counter == k:
                                naiV_row.append(1)
                            else:
                                naiV_row.append(0)
                        else:
                            naiV_row.append(0)
                    counter += 1
                    self.naiH.append(naiH_row)
                    self.naiV.append(naiV_row)

    # export matrix as matlab .mat file
    def exportNAI(self):
        nai = dict()
        nai['NAIH'] = np.array(self.naiH)
        nai['NAIV'] = np.array(self.naiV)
        savemat('NAI.mat', nai)

    def readCsv(self, file):
        f = open(file)
        mat = csv.reader(f)
        for i in mat:
            self.xh.append(int(i[0]))
        counter = 0
        for i in range(self.start[0], self.end[0] + 1):
            row = []
            for j in range(self.start[1], self.end[1] + 1):
                if [i, j] not in self.unpassable:
                    row.append(self.xh[counter])
                    counter += 1
                else:
                    row.append(-1)
            self.resulting_grid.append(row)
            # print(row)

    # plot a scatter plot from matlab generated results xh.csv
    def plotCsv(self):
        connection = []
        plt.gca().invert_yaxis()
        for i in self.gtsp_set:
            # print(i)
            for j in range(len(i) - 1):
                connection.append([[i[j][0], i[j + 1][0]], [i[j][1], i[j + 1][1]]])
                # print(connection[-1])
        for i in connection:
            plt.plot(i[1], i[0], color='r')
            plt.scatter(i[1], i[0], color='b')
        plt.savefig('lines.png')
        transition = []
        for i in self.transition_segements:
            for j in range(len(i) - 1):
                transition.append([[i[j][0], i[j + 1][0]], [i[j][1], i[j + 1][1]]])
                # print(connection[-1])
        for i in transition:
            plt.plot(i[1], i[0], color='y')
            plt.scatter(i[1], i[0], color='b')
        plt.savefig('segment.png')
        # plt.show()

    def parseLines(self):
        horizontal_lines = []
        vertical_lines = []
        for i in range(self.start[0], self.end[0] + 1):
            row = []
            for j in range(self.start[1], self.end[1] + 1):
                if not self.resulting_grid[i][j]:
                    if [i, j + 1] not in self.unpassable and j + 1 <= self.end[1]:
                        if not self.resulting_grid[i][j + 1]:
                            if [i, j] not in row:
                                row.append([i, j])
                            row.append([i, j + 1])
                        else:
                            if row:
                                horizontal_lines.append(row)
                                # print(row)
                            row = []
                    elif [i, j] == end:
                        # print(row)
                        horizontal_lines.append(row)
        for j in range(self.start[1], self.end[1] + 1):
            column = []
            for i in range(self.start[0], self.end[0] + 1):
                if self.resulting_grid[i][j]:
                    if [i + 1, j] not in self.unpassable and i + 1 <= self.end[0]:
                        if self.resulting_grid[i + 1][j]:
                            if [i, j] not in column:
                                column.append([i, j])
                            column.append([i + 1, j])
                        else:
                            if column or [i, j] == self.end:
                                vertical_lines.append(column)
                                # print(column)
                            column = []
                    elif [i, j] == end:
                        # print(column)
                        vertical_lines.append(column)
        self.gtsp_set = horizontal_lines + vertical_lines
        # print(len(self.gtsp_set))

    # lines are indexed 0 to n-1
    def possibleTransitions(self):
        transition = []
        for i in self.gtsp_set:
            # print(i)
            # 0 for horizontal 1 for vertical
            line_start = i[0]
            line_end = i[-1]
            line_orientation = abs(line_start[0] - line_end[0])
            if line_orientation:
                if line_start[1] + 1 <= self.end[1] and [line_start[0], line_start[1] + 1] not in self.unpassable:
                    transition.append([line_start, [line_start[0], line_start[1] + 1]])
                if line_start[1] - 1 >= self.start[1] and [line_start[0], line_start[1] - 1] not in self.unpassable:
                    transition.append([[line_start[0], line_start[1] - 1], line_start])
                if line_start[0] - 1 >= self.start[0] and [line_start[0] - 1, line_start[1]] not in self.unpassable:
                    transition.append([[line_start[0] - 1, line_start[1]], line_start])
                if line_end[1] + 1 <= self.end[1] and [line_end[0], line_end[1] + 1] not in self.unpassable:
                    transition.append([line_end, [line_end[0], line_end[1] + 1]])
                if line_end[0] + 1 <= self.end[0] and [line_end[0] + 1, line_end[1]] not in self.unpassable:
                    transition.append([line_end, [line_end[0] + 1, line_end[1]]])
                if line_end[1] - 1 >= self.start[1] and [line_end[0], line_end[1] - 1] not in self.unpassable:
                    transition.append([[line_end[0], line_end[1] - 1], line_end])
            else:
                if line_start[0] + 1 <= self.end[0] and [line_start[0] + 1, line_start[1]] not in self.unpassable:
                    transition.append([line_start, [line_start[0] + 1, line_start[1]]])
                if line_start[0] - 1 >= self.start[0] and [line_start[0] - 1, line_start[1]] not in self.unpassable:
                    transition.append([[line_start[0] - 1, line_start[1]], line_start])
                if line_start[1] - 1 >= self.start[1] and [line_start[0], line_start[1] - 1] not in self.unpassable:
                    transition.append([[line_start[0], line_start[1] - 1], line_start])
                if line_end[1] + 1 <= self.end[1] and [line_end[0], line_end[1] + 1] not in self.unpassable:
                    transition.append([line_end, [line_end[0], line_end[1] + 1]])
                if line_end[0] + 1 <= self.end[0] and [line_end[0] + 1, line_end[1]] not in self.unpassable:
                    transition.append([line_end, [line_end[0] + 1, line_end[1]]])
                if line_end[0] - 1 >= self.start[0] and [line_end[0] - 1, line_end[1]] not in self.unpassable:
                    transition.append([[line_end[0] - 1, line_end[1]], line_end])
        for i in transition:
            # print(i)
            if i not in self.transition_segements:
                # print(i)
                self.transition_segements.append(i)
        # print(len(self.transition_segements))

    def plotGTSP(self):
        f = open('GTSP_result.csv')
        mat = csv.reader(f)
        gtsp_result = []
        for i in mat:
            row = []
            for j in i:
                # print(j)
                row.append(int(j))
            gtsp_result.append(row)
            # print(row)
        connection = []
        for i in range(len(gtsp_result)):
            for j in range(len(gtsp_result[i])):
                if gtsp_result[i][j]:
                    front = self.squares[i].coord
                    back = self.squares[j].coord
                    connection.append([[front[0], back[0]], [front[1], back[1]]])
        plt.gca().invert_yaxis()
        for i in connection:
            plt.plot(i[1], i[0], color='r')
            plt.scatter(i[1], i[0], color='b')
        plt.savefig('GTSP.png')

    def createGTSPSet(self):
        gtsp = []
        for i in self.gtsp_set:
            gtsp.append([i[0], i[-1]])
            gtsp.append([i[-1], i[0]])
            # print(gtsp[-1])
            # print(gtsp[-2])
        seq = [1, 235, 4, 26, 5, 22, 7, 17, 13, 27, 16, 31, 12, 29, 9, 19]
        for i in seq:
            print(gtsp[i - 1])
        edge_weight_section = []
        for i in gtsp:
            row = []
            for j in gtsp:
                if not dot(minus(i[1], i[0]), minus(j[1], j[0])):
                    row.append(20)
                else:
                    row.append((self.end[0] - self.start[0] + 1 - distParallelLine(i, j)) * 20)
            edge_weight_section.append(row)
        mat = np.matrix(edge_weight_section)
        np.savetxt('edge_weight_section.txt', mat, fmt='%d')
        print(len(edge_weight_section))
        # for i in self.transition_segements:
        #     gtsp.append(i)
        #     gtsp.append([i[-1], i[0]])

    def findTSPCost(self):
        all_straight_connections = []
        not_connected_cost = 10000
        straight_connection_cost = 1
        transition_connection_cost = 100
        numbered = []
        for i in self.gtsp_set:
            number_row = []
            for j in range(len(i) - 1):
                all_straight_connections.append([i[j], i[j + 1]])
                # print(all_straight_connections[-1])
        for i in range(start[0], end[0] + 1):
            for j in range(start[1], end[1] + 1):
                row = []
                if [i, j] not in self.unpassable:
                    for k in range(start[0], end[0] + 1):
                        for l in range(start[1], end[1] + 1):
                            if [k, l] not in self.unpassable:
                                if [[i, j], [k, l]] in all_straight_connections:
                                    row.append(straight_connection_cost)
                                elif [[i, j], [k, l]] in self.transition_segements:
                                    row.append(transition_connection_cost)
                                else:
                                    row.append(not_connected_cost)
                    self.tsp_cost.append(row)
        OY_edge_df = pd.DataFrame({"Node 1": [], "Node 2": [], "edge": []})
        OY_edge_new_row = pd.DataFrame({"Node 1": ['d', 'd'], "Node 2": ['d', '0'], "edge": [0, 0]})
        OY_edge_df = pd.concat([OY_edge_df, OY_edge_new_row], ignore_index=True)
        # OY_edge_df = OY_edge_df.append({"Node 1": 'd',"Node 2": 'd',"edge":0},ignore_index=True)
        for i in range(len(self.tsp_cost)):
            OY_edge_new_row = pd.DataFrame({"Node 1": ['d'], "Node 2": [i], "edge": [10000]})
            OY_edge_df = pd.concat([OY_edge_df, OY_edge_new_row], ignore_index=True)

        for i in range(len(self.tsp_cost)):
            OY_edge_new_row = pd.DataFrame({"Node 1": [i], "Node 2": ['d'], "edge": [0]})
            OY_edge_df = pd.concat([OY_edge_df, OY_edge_new_row], ignore_index=True)
            for j in range(len(self.tsp_cost[i])):
                OY_edge_new_row = pd.DataFrame({"Node 1": [i], "Node 2": [j], "edge": [self.tsp_cost[i][j]]})
                OY_edge_df = pd.concat([OY_edge_df, OY_edge_new_row], ignore_index=True)
        OY_edge_df.to_csv('google_tsp.csv', index=False)

    def exportTSPCost(self):
        tsp_cost = dict()
        tsp_cost['tsp_cost'] = np.array(self.tsp_cost)
        savemat('tsp_cost.mat', tsp_cost)

    def createMovementMatrix(self):
        movement = []
        row = [0] * len(self.squares)
        for i in range(len(self.squares)):
            row[i] = 1
            for j in range(len(self.squares)):
                if not i == j:
                    row[j] = -1
                    movement.append(row)
                    # print(row)
                    row[j] = 0
                else:
                    movement.append([0] * len(self.squares))
            row[i] = 0
        # print(len(movement))


class Square:
    def __init__(self, coord, end, unpassable, number):
        self.coord = coord
        self.number = number
        self.right = None
        self.down = None
        self.orientation = 1
        self.connection_num = 0
        if self.coord[0] + 1 <= end[0] and ([self.coord[0] + 1, self.coord[1]]) not in unpassable:
            self.down = [self.coord[0] + 1, self.coord[1]]
        if self.coord[1] + 1 <= end[1] and ([self.coord[0], self.coord[1] + 1]) not in unpassable:
            self.right = [self.coord[0], self.coord[1] + 1]


start = [0, 0]
end = [9, 9]
unpassable = [[1, 4], [2, 4], [3, 4], [1, 5], [2, 5], [3, 5], [5, 7], [5, 8], [6, 7], [6, 8]]
# unpassable = [[5, 7], [5, 8], [6, 7], [6, 8]]
mode = 1
g = Grid(start, end, unpassable)
g.findNAI()
if not mode:
    g.exportNAI()
elif mode == 1:
    g.readCsv('xh.csv')
    g.parseLines()
    g.possibleTransitions()
    g.findTSPCost()
    g.exportTSPCost()
    g.createMovementMatrix()
    g.createGTSPSet()
    g.plotCsv()
elif mode == 2:
    g.plotGTSP()