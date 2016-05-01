import json
import copy
import sys
import math
import numpy as np

import matplotlib
from matplotlib.patches import Polygon as mPolygon
from matplotlib.collections import PatchCollection, LineCollection
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from shapely.geometry import Polygon, MultiPolygon, Point, LineString, MultiLineString


class Environment:

    def __init__(self, input_file):
        """
        Environment helper class for collision detection and matplotlib plotting.
        :param input_file: File name on input json.
        :return: None
        """
        self.plot_obstacles_polygon = []
        self.obs_list = []
        self.obs_polygon = MultiPolygon()       # Shapely object to store all polygons
        self.initial_state, self.goal_state = [], []
        self.initial_orientation = 0
        self.goal_orientation = 0
        self.resolution = 0                     # Dimension of the plane.
        self.car_data = None
        self.car_geom = np.array([])
        self.environment = {}

        self.read_env_from_file(input_file)

    def get_free_area(self):
        """
        To get the total free area.
        :return: Total area - obstacle area.
        """
        free_area = 0
        for op in self.obs_polygon:
            free_area += op.area
        return self.resolution**2 - free_area

    def read_env_from_file(self, input_file):
        """
        Read json input and sets the object variables.
        :param input_file: Input json file name.
        :return: None
        """
        try:
            with open(input_file, mode='r', encoding='utf-8') as a_file:
                self.environment = json.loads(a_file.read())

            # Making sure the required entities are defined in the input json file.

            self.initial_state, self.goal_state = self.environment['initial_state'], self.environment['goal_state']
            self.initial_orientation = math.radians(int(self.environment["initial_orientation"]))
            self.goal_orientation = math.radians(int(self.environment["goal_orientation"]))
            self.resolution = self.environment['resolution']
            temp_polygon_list = []

            for obs in self.environment['obstacles']:
                if not obs.get('shape') and obs.get('property') and obs['property'].get('vertices'):
                    print("Shape element not present for the obstacles")
                    continue
                if obs['shape'] == 'polygon':
                    polygon = mPolygon(np.array(obs['property']['vertices']), color='brown')
                    temp_polygon_list.append(Polygon(obs['property']['vertices']))
                    self.plot_obstacles_polygon.append(polygon)
                    self.obs_list.append(obs['property']['vertices'])
                else:
                    print("Undefined shape")
                    break
            self.car_data = self.environment["car"]
            self.obs_polygon = MultiPolygon(temp_polygon_list)

            self.car_geom = self.car_data["dimension"]["property"]["vertices"]

            car_shifted_axis_initial = self._change_axis(self.initial_orientation, self.initial_state)
            self.plot_obstacles_polygon.append(mPolygon(car_shifted_axis_initial, color='red'))

            for l in ["linear_velocity", "max_turning_angle", "front_axle_back_axle_distance", "dimension"]:
                if l not in self.environment["car"]:
                    raise KeyError
            if "vertices" not in self.environment["car"]["dimension"]["property"]:
                raise KeyError
            if "shape" not in self.environment["car"]["dimension"]:
                raise KeyError

        except KeyError:
            print("Invalid Environment definition")
            exit(1)
        except FileNotFoundError as fl:
            print("File not found for JSON ", fl)
            exit(1)
        except ValueError:
            print("Invalid JSON")
            exit(1)
        except Exception:
            print("Unable to process input file")
            exit(1)

    def _change_axis(self, theta, xy):
        """
        This is a rotation technique. The car object is defined in environment as if its center
        (center of rear axle) is super imposed in the origin of euclidean plane. To get the new position, i.e,
        the coordinates of its vertices the position of the car is at xy and rotated to theta radians,
        first we need to rotate the car theta radians from origin position, then add the translated xy
        coordinates to all the vertices.

        rot(theta) = [[cos(theta), -sin(theta)],
                        [sin(theta), cos(theta)]]
        point = [[vertex_point[0]],
                [vertex_point[1]]]
        coordinate after rotation, CR = rot(theta) x point
        after translation, AT = CR + xy
        :param theta: Orientation of car w.r.t x-axis.
        :param xy: Center of rear axle after translation.
        :return: nd.array of rotated and translated vertices.
        """
        if not isinstance(xy, np.ndarray):
            xy = np.array(xy)
        xy = xy.reshape(2, 1)
        out = list()
        rot = np.array([[math.cos(theta), -math.sin(theta)],
                        [math.sin(theta), math.cos(theta)]])
        for xy_in in self.car_geom:
            xy_in = np.array(xy_in).reshape(2, 1)
            out.append(np.dot(rot, xy_in)+xy)
        return np.array([o.reshape(1, 2)[0] for o in out])

    def is_within_boundary(self, xy):
        """
        Check if a point is inside boundary.
        :param xy: Position of the point
        :return: Boolean. True if point is within boundary and False if point is outside the boundary.
        """
        x, y = xy
        x_check = (x >= 0) and (x <= self.resolution)
        y_check = (y >= 0) and (y <= self.resolution)
        return x_check and y_check

    def is_car_within_boundary(self, xyo):
        """
        Check if a car is inside boundary.
        :param xyo: Position of the center of rear axle.
        :return: Boolean. True if car is within boundary and False if car is outside the boundary.
        """
        theta, xy = xyo[2], xyo[:2]
        poss_xy = [list(l) for l in self._change_axis(theta, xy)]
        min_x = min(poss_xy, key=lambda foo: foo[0])[0]
        min_y = min(poss_xy, key=lambda foo: foo[1])[1]
        max_x = max(poss_xy, key=lambda foo: foo[0])[0]
        max_y = max(poss_xy, key=lambda foo: foo[1])[1]

        x_check = (min_x >= 0) and (max_x <= self.resolution)
        y_check = (min_y >= 0) and (max_y <= self.resolution)
        return x_check and y_check

    def is_car_inside(self, xyo):
        """
        Check if the car is intersecting any obstacles.
        :param xyo: Position of the center of rear axle.
        :return: Boolean. True if car intersects any obstacle and False if car is not the boundary.
        """
        return Polygon(self._change_axis(xyo[2], xyo[:2])).intersects(self.obs_polygon)

    def is_point_inside(self, xy):
        """
        :param xy: tuple with x coordinate as first element and y coordinate as second element
        :return: True if the point is inside the obstacles and False if it isn't
        """
        return Point(xy[0], xy[1]).within(self.obs_polygon)

    def is_line_inside(self, xy_start, xy_end):
        # xy_start is tuple of (x, y) coordinate of one end of the line.
        # xy_end is tuple of (x, y) coordinate of the other end of line.
        line = LineString([xy_start, xy_end])
        return self.obs_polygon.contains(line) or self.obs_polygon.touches(line) or self.obs_polygon.crosses(line)

    def is_arc_inside(self, xy_center, radius, start_angle, end_angle, num=5):
        lines = list()
        for theta in np.linspace(start_angle, end_angle, num=num):
            x, y = xy_center[0] + radius*math.cos(theta), xy_center[1] + radius*math.sin(theta)
            lines.append((x, y))
        lines_list = [(lines[i], lines[i+1]) for i in range(len(lines) - 1)]
        multiline = [LineString(dt) for dt in lines_list]
        multiline_obj = MultiLineString(multiline)
        return self.obs_polygon.contains(multiline_obj) or self.obs_polygon.touches(multiline_obj) or self.obs_polygon.\
            crosses(multiline_obj)

    def draw_env(self, lines, path):
        """
        Method to draw an arrow in the environment.
        Format for lines and path is [[[x1, y1], [x2, y2]], [[x3, y3], [x4, y4]]], where
        [[x1, y1], [x2, y2]] is one line segment.
        :param lines: List of blue lines.
        :param path: List of black lines.
        :return:
        """
        fig, ax = plt.subplots()

        colors = np.random.rand(len(self.plot_obstacles_polygon))
        p = PatchCollection(self.plot_obstacles_polygon, cmap=matplotlib.cm.jet, alpha=0.4)
        p.set_array(np.array(colors))
        ax.add_collection(p)

        color_lines = np.tile((0, 1, 0, 1), (len(lines), 1))
        lc = LineCollection(lines, colors=color_lines, linewidths=2)
        ax.add_collection(lc)

        if path is not None:
            color_path = np.tile((1, 0, 0, 1), (len(path), 1))
            pt = LineCollection(path, colors=color_path, linewidths=2)
            ax.add_collection(pt)

        # plt.colorbar(p)
        plt.plot([self.initial_state[0]], [self.initial_state[1]], 'bs', self.goal_state[0], self.goal_state[1], 'g^')
        plt.axis([0, self.resolution, 0, self.resolution])
        plt.show()

    def animate_path(self, path, key_xy):
        fig, ax = plt.subplots()

        colors = np.random.rand(len(self.plot_obstacles_polygon))
        p = PatchCollection(self.plot_obstacles_polygon, cmap=matplotlib.cm.jet, alpha=0.4)
        p.set_array(np.array(colors))
        ax.add_collection(p)
        plt.colorbar(p)

        plt.plot([self.initial_state[0]], [self.initial_state[1]], 'bs', self.goal_state[0], self.goal_state[1], 'g^')
        plt.axis([0, self.resolution, 0, self.resolution])

        x_0, y_0 = key_xy(path[0])[0], key_xy(path[0])[1]
        x_1, y_1 = key_xy(path[0 + 1])[0], key_xy(path[0 + 1])[1]
        dx, dy = x_1 - x_0, y_0 - y_1
        qv = ax.quiver(x_0, y_0, dx, dy, angles='xy',scale_units='xy',scale=1)

        def animate(i):
            x_init, y_init =key_xy(path[i])[0], key_xy(path[i])[1]
            x_f, y_f = key_xy(path[i + 1])[0], key_xy(path[i + 1])[1]
            dx, dy = x_f - x_init, y_f - y_init
            qv.set_UVC(np.array(dx), np.array(dy))
            qv.set_offsets((x_init, y_init))
            return qv

        anim = animation.FuncAnimation(fig, animate, frames=range(0, len(path)-1), interval=500)
        plt.show()

    def get_apprx_visible_vertices(self, xy_robot):
        # To get visible vertices from robot point
        # xy_robot should be a tuple of (x, y) coordinate
        if self.is_point_inside(xy_robot):
            print("Invalid robot position")
            return None
        pool = copy.deepcopy(self.obs_list)
        pool.append([self.goal_state])
        visible_vertices, visible_lines = [], []

        for obj in pool:
            for vertex in obj:
                vertex = tuple(vertex)
                if vertex == xy_robot:
                    continue
                crosses, line = self.visibility_line(xy_robot, vertex)
                if not crosses:
                    visible_lines.append(line)
        visible_vertices.extend([x.xy[0][1], x.xy[1][1]] for x in visible_lines)
        return visible_vertices

    def get_actual_visible_vertices(self, xy_robot):
        if self.is_point_inside(xy_robot):
            print("Invalid robot position")
            return None
        pool = copy.deepcopy(self.obs_list)
        pool.append([self.goal_state])
        visible_vertices, line_robot_vertices = [], {}

        def line_slope(xy1, xy2):
            return (xy2[1] - xy1[1])/(xy2[0] - xy1[0]) if (xy2[0] - xy1[0]) != 0 else sys.maxsize

        for obj in pool:
            for vertex in obj:
                crosses, line = self.visibility_line(xy_robot, vertex)
                if not crosses:
                    if line_slope(xy_robot, vertex) in line_robot_vertices:
                        if line.length < line_robot_vertices[line_slope(xy_robot, vertex)].length:
                            line_robot_vertices[line_slope(xy_robot, vertex)] = line
                    else:
                        line_robot_vertices[line_slope(xy_robot, vertex)] = line
        visible_vertices.extend([x.xy[0][1], x.xy[1][1]] for x in line_robot_vertices.values())
        return visible_vertices

    def visibility_line(self, xy_start, xy_end):
        # Helper Method to check if the line is intersected by any obstacles.
        line = LineString([xy_start, xy_end])
        return self.obs_polygon.crosses(line) or self.obs_polygon.contains(line), line

    def __str__(self):
        return "Obstacle list: %s\nInitial State: %s\nGoal State: %s\nResolution: %d\n" \
               % ([cord.xy for cord in self.plot_obstacles_polygon], self.initial_state, self.goal_state, self.resolution)


