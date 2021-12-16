import math
import time
from math import sqrt, pow, atan2, cos, sin, radians, degrees

from matplotlib import pyplot as plt
from matplotlib import colors
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg)
import numpy as np
import json
import tkinter as tk
import sys

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# Init subscribera
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('MinimalSubscriber')
        self.subscription = self.create_subscription(String,
                                                     'topic', self.listener_callback, 10)

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        global map, x_max
        action, coord, param = msg.data.split(';')
        print("Recieved action: " + str(action))
        if action == 'edit' and param != '':
            print("Now performing " + str(action) + " action.")
            coords = coord.split(',')
            for cord in coords:
                x, y = cord.split('-')
                map[x_max - int(x)][int(y)] = int(param)
            if int(param) == 1:
                global addedWall, coordWall
                addedWall = True
                coordWall = coords
        elif action == "move":
            print("Now performing " + str(action) + " action.")
            coords = coord.split(',')
            for cord in coords:
                x, y = cord.split('-')
                for i in range(len(map)):
                    for j in range(len(map[i])):
                        if map[i][j] in (2, 3):
                            map[i][j] = 0
                robot.setD_Goal(int(x), int(y))
                map[x_max - int(x)][int(y)] = 2
                print(robot.getD_Goal())
        elif action == 'quit':
            print("Now performing " + str(action) + " action.")
            sys.exit()


# Funkcia pre ziskanie mapy
def get_map(map):
    global x_max, y_max
    f = open(map)
    data = json.load(f)
    x_max, y_max = 0, 0
    for x in data['tiles']:
        if x['x_cord'] > x_max:
            x_max = x['x_cord']
        if x['y_cord'] > y_max:
            y_max = x['y_cord']
    data_list = []
    for x in range(x_max + 1):
        temp = []
        for y in range(y_max + 1):
            for tile in data['tiles']:
                if tile['x_cord'] == x and tile['x_cord'] == y:
                    temp.append(tile['occ'])
        data_list.insert(0, temp)
    return np.array(data_list, np.int32)

#D* Lite algorithm
class Node:
    def __init__(self, id):
        self.id = id
        self.parents = {}
        self.children = {}
        # g approximation
        self.g = float('inf')
        # rhs value
        self.rhs = float('inf')

    def __str__(self):
        return 'Node: ' + self.id + ' g: ' + str(self.g) + ' rhs: ' + str(self.rhs)

    def __repr__(self):
        return self.__str__()

    def update_parents(self, parents):
        self.parents = parents


class Graph:
    def __init__(self):
        self.graph = {}

    def __str__(self):
        msg = 'Graph:'
        for i in self.graph:
            msg += '\n  node: ' + i + ' g: ' + \
                   str(self.graph[i].g) + ' rhs: ' + str(self.graph[i].rhs)
        return msg

    def __repr__(self):
        return self.__str__()

    def setStart(self, id):
        if (self.graph[id]):
            self.start = id
        else:
            raise ValueError('start id not in graph')

    def setGoal(self, id):
        if (self.graph[id]):
            self.goal = id
        else:
            raise ValueError('goal id not in graph')


class GraphMap(Graph):
    def __init__(self, map):
        self.map = map
        self.x_dim, self.y_dim = map.shape
        self.x_max = self.x_dim - 1

        self.cells = list()
        for x in range(self.x_dim):
            row = list()
            for y in range(self.y_dim):
                row.append(0)
            self.cells.append(row)

        self.graph = {}

        self.generateGraphFromGrid()

    def __str__(self):
        msg = 'Graph:'
        for i in self.graph:
            msg += '\n  node: ' + i + ' g: ' + \
                   str(self.graph[i].g) + ' rhs: ' + str(self.graph[i].rhs) + \
                   ' neighbors: ' + str(self.graph[i].children)
        return msg

    def __repr__(self):
        return self.__str__()

    def printGrid(self):
        print('** GridWorld **')
        for row in self.cells:
            print(row)

    def printGValues(self):
        for j in range(self.y_dim):
            str_msg = ""
            for i in range(self.x_dim):
                node_id = 'x' + str(i) + 'y' + str(j)
                node = self.graph[node_id]
                if node.g == float('inf'):
                    str_msg += ' - '
                else:
                    str_msg += ' ' + str(node.g) + ' '
            print(str_msg)

    def generateGraphFromGrid(self):
        edge = 1
        for i in range(len(self.cells)):
            row = self.cells[self.x_max - i]
            for j in range(len(row)):
                if (self.map[i][j] != 1):
                    node = Node('x' + str(self.x_max - i) + 'y' + str(j))

                    if self.x_max - i > 0 and self.checkObsticle(self.x_max - i - 1, j):  # not top row
                        node.parents['x' + str(self.x_max - i - 1) + 'y' + str(j)] = edge
                        node.children['x' + str(self.x_max - i - 1) + 'y' + str(j)] = edge

                    if self.x_max - i + 1 < self.y_dim and self.checkObsticle(self.x_max - i + 1, j):  # not bottom row
                        node.parents['x' + str(self.x_max - i + 1) + 'y' + str(j)] = edge
                        node.children['x' + str(self.x_max - i + 1) + 'y' + str(j)] = edge

                    if j > 0 and self.checkObsticle(self.x_max - i, j - 1):  # not left col
                        node.parents['x' + str(self.x_max - i) + 'y' + str(j - 1)] = edge
                        node.children['x' + str(self.x_max - i) + 'y' + str(j - 1)] = edge

                    if j + 1 < self.x_dim and self.checkObsticle(self.x_max - i, j + 1):  # not right col
                        node.parents['x' + str(self.x_max - i) + 'y' + str(j + 1)] = edge
                        node.children['x' + str(self.x_max - i) + 'y' + str(j + 1)] = edge

                    if self.x_max - i + 1 < self.y_dim and j + 1 < self.x_dim and self.checkObsticle(self.x_max - i + 1,
                                                                                                     j + 1):  # Diagonaly rightTop
                        node.parents['x' + str(self.x_max - i + 1) + 'y' + str(j + 1)] = 1.4
                        node.children['x' + str(self.x_max - i + 1) + 'y' + str(j + 1)] = 1.4

                    if self.x_max - i + 1 < self.y_dim and j - 1 >= 0 and self.checkObsticle(self.x_max - i + 1,
                                                                                             j - 1):  # Diagonaly leftTop
                        node.parents['x' + str(self.x_max - i + 1) + 'y' + str(j - 1)] = 1.4
                        node.children['x' + str(self.x_max - i + 1) + 'y' + str(j - 1)] = 1.4

                    if self.x_max - i - 1 >= 0 and j + 1 < self.x_dim and self.checkObsticle(self.x_max - i - 1,
                                                                                             j + 1):  # Diagonaly rightBot
                        node.parents['x' + str(self.x_max - i - 1) + 'y' + str(j + 1)] = 1.4
                        node.children['x' + str(self.x_max - i - 1) + 'y' + str(j + 1)] = 1.4

                    if self.x_max - i - 1 >= 0 and j - 1 >= 0 and self.checkObsticle(self.x_max - i - 1,
                                                                                     j - 1):  # Diagonaly leftBot
                        node.parents['x' + str(self.x_max - i - 1) + 'y' + str(j - 1)] = 1.4
                        node.children['x' + str(self.x_max - i - 1) + 'y' + str(j - 1)] = 1.4

                    self.graph['x' + str(self.x_max - i) + 'y' + str(j)] = node

    def checkObsticle(self, i, j):
        if self.map[self.x_max - i][j] == 1:
            return False
        else:
            return True


def addNodeToGraph(graph, id, neighbors, edge=1):
    node = Node(id)
    for i in neighbors:
        node.parents[i] = edge
        node.children[i] = edge
    graph[id] = node
    return graph


def h_s(id, current):
    x_id, y_id = int(id.split('y')[0][1]), int(id.split('y')[1])
    x_curr, y_curr = int(current.split('y')[0][1]), int(current.split('y')[1])


    return abs(x_curr - x_id) + abs(y_curr - y_id)

def calculateKey(graph, id, s_current, km):
    return (
    min(graph.graph[id].g, graph.graph[id].rhs) + h_s(id, s_current) + km, min(graph.graph[id].g, graph.graph[id].rhs))


def updateVer(graph, U, id, curr, km):

    if id != graph.goal:
        min_rhs = float('inf')
        # print(graph.graph[id].children)
        for child in graph.graph[id].children:
            min_rhs = min(min_rhs, graph.graph[child].g + graph.graph[id].children[child])
        graph.graph[id].rhs = min_rhs

    id_in_queue = [item for item in U if id in item]

    if id_in_queue:
        U.remove(id_in_queue[0])

    if graph.graph[id].rhs != graph.graph[id].g:
        U.append([calculateKey(graph, id, curr, km), id])
        U.sort()


def computeShortestPath(graph, U, start, km):
    while (graph.graph[start].g != graph.graph[start].rhs or U[0][0] < calculateKey(graph, start, start, km)):
        # print(U)
        k_old = U[0][0]
        u = U.pop(0)[1]
        U.sort()

        if k_old < calculateKey(graph, u, start, km):
            U.append([calculateKey(graph, u, start, km), u])
            U.sort()
        elif graph.graph[u].g > graph.graph[u].rhs:
            graph.graph[u].g = graph.graph[u].rhs
            for parent in graph.graph[u].parents:
                updateVer(graph, U, parent, start, km)
        else:
            graph.graph[u].g = float('inf')
            updateVer(graph, U, u, start, km)
            for parent in graph.graph[u].parents:
                updateVer(graph, U, parent, start, km)


def get_path(graph, start):
    path = list()
    path.append(start)
    end = False
    while not end:
        g_s = list()
        names = list()
        for x in graph.graph[path[-1]].parents:
            g_s.append(graph.graph[x].g)
            names.append(x)
        path.append(names[g_s.index(min(g_s))])

        if graph.graph[path[-1]].g == 0:
            end = True
    return path


# Class pre robota, uchovavame udaje o rychlosti a pozicii
class Create_Robot():
    def __init__(self, map):
        self.linSpeed = 0
        self.angSpeed = 0
        self.theta = 90  # Uhol robota <-pi , pi> treba pripocitat 180 stupnov pre skutocnz uhol
        self.goalx = None
        self.goaly = None
        self.dgoalx = None
        self.dgoaly = None
        self.steering = 0
        f = open(map)
        data = json.load(f)
        for x in data['tiles']:
            if x['occ'] == 4:
                self.x = int(x['x_cord'])
                self.y = int(x['y_cord'])
        self.dist = 0.1

    def setGoal(self, x, y):
        self.goalx = x
        self.goaly = y

    def setD_Goal(self,x,y):
        self.dgoalx = x
        self.dgoaly = y

    def getGoal(self):
        return self.goalx, self.goaly

    def getD_Goal(self):
        return self.dgoalx, self.dgoaly

    def euclidean_distance(self):
        return sqrt(pow((self.goalx - self.x), 2) + pow((self.goaly - self.y), 2))

    def steering_angle(self):
        self.steering = atan2(self.goalx - self.x, self.goaly - self.y)
        print(self.steering)
        if self.steering != math.pi or self.steering != 0:
            self.steering = (self.steering * -1)
        if self.steering < 0:
            self.steering = ((self.steering * -1) + 2 * (self.steering + math.pi))

    def linear_vel(self, constatnt=1.5):
        self.linSpeed = constatnt * self.euclidean_distance()

    def angular_vel(self, constant=1):
        self.steering_angle()

        self.angSpeed = constant * (self.steering - radians(self.theta))

        # print("ANG SPEED RAW: " + str(self.angSpeed))
        if self.angSpeed > math.pi:
            temp = (self.angSpeed - math.pi)
            self.angSpeed = ((self.angSpeed * -1) + 2 * temp)

        if self.angSpeed < -math.pi:
            temp = (self.angSpeed + math.pi)
            self.angSpeed = ((self.angSpeed * -1) + temp)
        # print("ANG SPEED Changed: " + str(self.angSpeed))

    def move(self):
        global map, x_max
        map[x_max - round(self.x)][round(self.y)] = 3

        tempx = self.x - sin(self.angSpeed + radians(self.theta)) * self.linSpeed * 0.5
        tempy = self.y + cos(self.angSpeed + radians(self.theta)) * self.linSpeed * 0.5

        map[x_max - round(tempx)][round(tempy)] = 4

        self.x = tempx
        self.y = tempy
        self.theta = self.theta + degrees(self.angSpeed)
        if self.theta > 180:
            self.theta -= 180
        self.at_target()

    def at_target(self):
        global map
        if self.euclidean_distance() <= self.dist:
            self.setGoal(None, None)

            # for i in range(len(map)):
            #     for j in range(len(map[i])):
            #         if map[i][j] == 3:
            #             map[i][j] = 0


def get_tile_from_path(x):
    x_tile, y_tile = x.split('y')
    _, x_tile = x_tile.split('x')

    return  int(x_tile), int(y_tile)

# Hlavna class
class NMvR(tk.Frame):
    def __init__(self, window, map_name):
        # Definovanie okna a ziskanie mapy
        self.frame = tk.Frame(master=window)
        self.data = get_map(map_name)

        global map, robot, colors_L, path, addedWall, km
        km = 0
        addedWall = False
        path = list()
        # Definovanie mapy do globalnej premennej a robota
        map = self.data
        robot = Create_Robot(map_name)

        # Vytvorenie figure kde sa bude vzkreslovat
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        plt.xticks(np.arange(0, 25, 1.0))
        plt.yticks(np.arange(0, 25, 1.0))

        # Definovanie farieb
        cmap = colors.ListedColormap(colors_L)

        # nastavenie mapy na axis
        self.ax.pcolor(self.data[::-1], cmap=cmap, edgecolor='blue', linewidth=2)

        # Kreslenie samotneho canvasu na tkinter window
        self.canvas = FigureCanvasTkAgg(self.fig, master=window)
        self.canvas.draw()
        # self.canvas.get_tk_widget().pack()
        self.canvas.get_tk_widget().place(relx=0.18, rely=0.3, height=500, width=500, anchor='center')

        self.canvas_arrow = tk.Canvas(window, height=200, width=200)
        self.canvas_arrow.create_line(0, 0, 0, 0, tags=('line',), arrow='last', width=10, arrowshape=(16, 20, 6))
        self.canvas_arrow.place(relx=0.52, rely=0.65, anchor='se')

        # Refresh mapy kazdu 1s
        self.frame.after(1000, self.refresh_c)

    def refresh_c(self):
        global robot, map, path, addedWall, coordWall, graph, km
        goal = robot.getD_Goal()

        if (not path) and not None in goal:
            graph = GraphMap(map)

            start = "x" + str(round(robot.x)) + "y" + str(round(robot.y))
            goal = "x" + str(robot.dgoalx) + "y" + str(robot.dgoaly)

            km = 0
            graph.setStart(start)
            graph.setGoal(goal)
            graph.graph[goal].rhs = 0
            U = list()
            U.append([calculateKey(graph, goal, start, km), goal])

            computeShortestPath(graph, U, start, km)

            path = get_path(graph, start)
            path.pop(0)
            for x in path:
                x_tile, y_tile = get_tile_from_path(x)
                # print(x_tile,y_tile)
                if (x_tile,y_tile) not in [(robot.x, robot.y),(robot.dgoalx, robot.dgoaly)]:
                    map[24 -x_tile][y_tile] = 3
            x, y = get_tile_from_path(path[0])
            robot.setGoal(x,y)
            robot.setD_Goal(None, None)

        if not None in robot.getGoal():
            print("EUKL Distance: " + str(robot.euclidean_distance()))
            robot.linear_vel()
            robot.angular_vel()
            print("Lin speed: " + str(robot.linSpeed))
            print("Ang speed: " + str(robot.angSpeed))
            print("Robot uhol: " + str(robot.theta + 180))
            robot.move()
            print("########################################")
        elif None in robot.getGoal() and len(path)>1:
            print(path)
            path.pop(0)
            x, y = get_tile_from_path(path[0])
            robot.setGoal(x, y)
        elif None in robot.getGoal() and len(path) == 1:
            for i in range(len(map)):
                for j in range(len(map[i])):
                    if map[i][j] == 3:
                        map[i][j] = 0
            path = list()
        if addedWall:
            addedWall = False
            if graph:
                for i in range(len(map)):
                    for j in range(len(map[i])):
                        if map[i][j] == 3:
                            map[i][j] = 0
                graph = GraphMap(map)

                start = "x" + str(round(robot.x)) + "y" + str(round(robot.y))
                goal = path[-1]

                km += h_s(start, goal)
                graph.setStart(start)
                graph.setGoal(goal)
                graph.graph[goal].rhs = 0
                U = list()
                U.append([calculateKey(graph, goal, start, km), goal])

                computeShortestPath(graph, U, start, km)

                path = get_path(graph, start)
                path.pop(0)
                for x in path:
                    x_tile, y_tile = get_tile_from_path(x)
                    # print(x_tile,y_tile)
                    if (x_tile, y_tile) not in [(robot.x, robot.y), (robot.dgoalx, robot.dgoaly)]:
                        map[24 - x_tile][y_tile] = 3
                x, y = get_tile_from_path(path[0])
                robot.setGoal(x, y)
                x, y = get_tile_from_path(path[-1])
                map[24 - int(x)][int(y)] = 2
                robot.setD_Goal(None, None)


        cmap = colors.ListedColormap(colors_L)

        # nastavenie mapy na axis a kreslenie canvasu
        self.ax.pcolor(self.data[::-1], cmap=cmap, edgecolor='blue', linewidth=2)
        a = radians(robot.theta + 180)
        r = 50
        x0, y0 = (75, 100)
        x1 = x0
        y1 = y0
        x2 = x0 + -r * cos(a)
        y2 = y0 + -r * sin(a)  # musi byt minus lebo 0-0 je v pravo hore a nie v lavo dole v Tkinter
        self.canvas_arrow.coords("line", x1, y1, x2, y2)
        self.canvas.draw()
        self.frame.after(1000, self.refresh_c)


def startMap():
    window = tk.Tk()
    window.geometry("700x700")
    root = NMvR(window, str(sys.argv[1]))
    window.mainloop()


def subscriber():
    rclpy.init()

    minimalsubscriber = MinimalSubscriber()

    rclpy.spin(minimalsubscriber)

    minimalsubscriber.destroy_node()
    rclpy.shutdown()


def main(args=None):
    global colors_L
    # Definovanie farieb pre vykreslovanie ['prazdne pole','prekazka','robot','ciel']
    colors_L = ['white', 'black', 'yellow','green','red']

    thread_map = threading.Thread(target=startMap)
    thread_map.start()
    time.sleep(4)
    thread_sub = threading.Thread(target=subscriber())
    thread_sub.start()


if __name__ == '__main__':
    # main()
    print("Hello")
