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
                map[x_max-int(x)][int(y)] = int(param)
        elif action == "move":
            print("Now performing " + str(action) + " action.")
            coords = coord.split(',')
            for cord in coords:
                x, y = cord.split('-')
                for i in range(len(map)):
                    for j in range(len(map[i])):
                        if map[i][j] in (2, 3):
                            map[i][j] = 0
                robot.setGoal(int(x), int(y))
                map[x_max-int(x)][int(y)] = 2
                print(robot.getGoal())
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
    for x in range(x_max+1):
        temp = []
        for y in range(y_max+1):
            for tile in data['tiles']:
                if tile['x_cord'] == x and tile['x_cord'] == y:
                    temp.append(tile['occ'])
        data_list.insert(0,temp)
    return np.array(data_list, np.int32)


# Class pre robota, uchovavame udaje o rychlosti a pozicii
class Create_Robot():
    def __init__(self, map):
        self.linSpeed = 0
        self.angSpeed = 0
        self.theta = 90  #Uhol robota <-pi , pi> treba pripocitat 180 stupnov pre skutocnz uhol
        self.goalx = None
        self.goaly = None
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

    def getGoal(self):
        return self.goalx, self.goaly

    def euclidean_distance(self):
        return sqrt(pow((self.goalx - self.x), 2) + pow((self.goaly - self.y), 2))

    def steering_angle(self):
        # if (self.goaly - self.y)<0:
        #     return atan2(self.goalx - self.x, -(self.goaly - self.y))
        # else:
        self.steering = atan2(self.goalx - self.x, self.goaly - self.y)
        print(self.steering)
        if self.steering != math.pi or self.steering != 0:
            self.steering = (self.steering * -1)
        if self.steering < 0:
            self.steering = ((self.steering * -1) + 2*(self.steering + math.pi))


    def linear_vel(self, constatnt = 1.5):
        self.linSpeed = constatnt * self.euclidean_distance()

    def angular_vel(self, constant = 1):
        self.steering_angle()

        self.angSpeed = constant * (self.steering - radians(self.theta))

        # print("ANG SPEED RAW: " + str(self.angSpeed))
        if self.angSpeed > math.pi:
            temp = (self.angSpeed - math.pi)
            self.angSpeed = ((self.angSpeed * -1) + 2*temp)

        if self.angSpeed < -math.pi:
            temp = (self.angSpeed + math.pi)
            self.angSpeed = ((self.angSpeed * -1) + temp)
        # print("ANG SPEED Changed: " + str(self.angSpeed))
    def move(self):
        global map, x_max
        map[x_max-round(self.x)][round(self.y)] = 3

        tempx = self.x - sin(self.angSpeed + radians(self.theta)) * self.linSpeed * 0.1
        tempy = self.y + cos(self.angSpeed + radians(self.theta)) * self.linSpeed * 0.1

        map[x_max-round(tempx)][round(tempy)] = 4
        #
        self.x = tempx
        self.y = tempy
        self.theta = self.theta + degrees(self.angSpeed)
        if self.theta > 180:
            self.theta -= 180
        self.at_target()

    def at_target(self):
        global map
        if self.euclidean_distance()<=self.dist:
            self.setGoal(None, None)

            for i in range(len(map)):
                for j in range(len(map[i])):
                    if map[i][j] == 3:
                        map[i][j] = 0



# Hlavna class
class NMvR(tk.Frame):
    def __init__(self, window, map_name):
        # Definovanie okna a ziskanie mapy
        self.frame = tk.Frame(master=window)
        self.data = get_map(map_name)

        global map, robot, colors_L
        # Definovanie mapy do globalnej premennej a robota
        map = self.data
        robot = Create_Robot(map_name)

        # Vytvorenie figure kde sa bude vzkreslovat
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        plt.xticks(np.arange(0,25,1.0))
        plt.yticks(np.arange(0,25,1.0))

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
        self.canvas_arrow.create_line(0,0,0,0, tags=('line',), arrow='last', width=10, arrowshape=(16,20,6))
        self.canvas_arrow.place(relx=0.52, rely=0.65,anchor='se')

        # Refresh mapy kazdu 1s
        self.frame.after(200, self.refresh_c)

    def refresh_c(self):
        global robot
        goal = robot.getGoal()

        if not None in goal:
            print("EUKL Distance: " + str(robot.euclidean_distance()))
            robot.linear_vel()
            robot.angular_vel()
            print("Lin speed: " + str(robot.linSpeed))
            print("Ang speed: " + str(robot.angSpeed))
            print("Robot uhol: " + str(robot.theta + 180))
            robot.move()
            print("########################################")

        cmap = colors.ListedColormap(colors_L)

        # nastavenie mapy na axis a kreslenie canvasu
        self.ax.pcolor(self.data[::-1], cmap=cmap, edgecolor='blue', linewidth=2)
        a = radians(robot.theta + 180)
        r = 50
        x0, y0 = (75,100)
        x1 = x0
        y1 = y0
        x2 = x0 + -r * cos(a)
        y2 = y0 + -r * sin(a) # musi byt minus lebo 0-0 je v pravo hore a nie v lavo dole v Tkinter
        self.canvas_arrow.coords("line", x1, y1, x2, y2)
        self.canvas.draw()
        self.frame.after(200, self.refresh_c)


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
