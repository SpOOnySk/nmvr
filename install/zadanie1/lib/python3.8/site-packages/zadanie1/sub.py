import time
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

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('MinimalSubscriber')
        self.subscription = self.create_subscription(String,
        'topic',self.listener_callback, 10)

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        action, coord, param = msg.data.split(';')
        print("Recieved action: " + str(action))
        if action == 'edit':
            print("Now performing " + str(action) + " action.")
            coords = coord.split(',')
            global map
            for cord in coords:
                x, y = cord.split('-')
                map[int(x)][int(y)] = int(param)
        elif action == 'quit':
            print("Now performing " + str(action) + " action.")
            map = None
            sys.exit()


def get_map(map):
    f = open(map)
    data = json.load(f)
    x_max, y_max = 0,0
    for x in data['tiles']:
        if x['x_cord']>x_max:
            x_max = x['x_cord']
        if x['y_cord'] > y_max:
            y_max = x['y_cord']
    data_list = []
    for x in range(x_max):
        temp = []
        for y in range(y_max):
            for tile in data['tiles']:
                if tile['x_cord'] == x and tile['x_cord']==y:
                    temp.append(tile['occ'])
        data_list.append(temp)
    return np.array(data_list, np.int32)


class NMvR(tk.Frame):
    def __init__(self, window, map_name):
        self.frame = tk.Frame(master = window)
        self.data = get_map(map_name)
        global map
        map = self.data
        # print(self.data)
        self.fig, self.ax = plt.subplots(figsize = (10,10))

        cmap = colors.ListedColormap(['white','black'])

        self.ax.pcolor(self.data[::-1], cmap = cmap, edgecolor='blue', linewidth=2)

        # print("creating canvas")
        self.canvas = FigureCanvasTkAgg(self.fig, master=window)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack()
        # print(map)

        self.frame.after(5000, self.refresh_c)

    def refresh_c(self):
        if self.data is None:
            sys.exit()

        cmap = colors.ListedColormap(['white', 'black'])

        # self.data = get_new_map()
        self.ax.pcolor(self.data[::-1], cmap=cmap, edgecolor='blue', linewidth=2)
        self.canvas.draw()
        self.frame.after(5000, self.refresh_c)


def startMap():
    window = tk.Tk()
    root = NMvR(window, str(sys.argv[1]))
    window.mainloop()


def subscriber():
    rclpy.init()

    minimalsubscriber = MinimalSubscriber()

    rclpy.spin(minimalsubscriber)

    minimalsubscriber.destroy_node()
    rclpy.shutdown()


def main(args=None):
    thread_map = threading.Thread(target = startMap)
    thread_map.start()
    time.sleep(4)
    thread_sub = threading.Thread(target=subscriber())
    thread_sub.start()

if __name__=='__main__':
    main()
    # print(window)
    # root = NMvR(window)
    #
    # window.mainloop()
