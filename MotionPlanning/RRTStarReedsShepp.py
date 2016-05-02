import random
import math
import json
import sys

import numpy as np

from ReedsSheppCar import ReedsShepp
from Graph import Graph
from Config import Environment

print_path = True


def rand_state(env):
    """
    The limits of this random function are determined by env.resolution and angular range.

    :return: nd.array(x, y, orientation)
    """
    x_r = random.randrange(0, env.resolution)
    y_r = random.randrange(0, env.resolution)
    o_r = random.uniform(-math.pi, math.pi)
    return np.array([x_r, y_r, o_r])


def update_tree(destination_data, steps_ut, tree_to_append, geom_skip_factor):
    """
    Helper class to keep track of lines to be drawn (tree_exploration/final_path lines). If the path is 's' or 'b' or 'f', then only the starting and end points of the line segment is tracked, otherwise for drawing the curve, the points are discretized based on geom_skip_factor value. This is to ensure memory optimization at the cost of final graph plot legibility.

    :param destination_data: ReedsShepp car object.
    :param steps_ut: nd.array Steps of xy coordinates
    :param tree_to_append: The tree list reference onto which the path needs to updated
    :param geom_skip_factor: Defines the precision of curves while drawing. Higher values results in course resolution.
    :return: None.
    """
    if destination_data.action in ['s', 'b', 'f']:
        tree_to_append.append([steps_ut[0][:2], steps_ut[-1][:2]])
    else:
        if geom_skip_factor > len(steps_ut):
            skip_factor = 1
        else:
            skip_factor = geom_skip_factor
        helper_list = [i for i in range(0, len(steps_ut), skip_factor)]
        if helper_list[-1] != len(steps_ut)-1:
            helper_list.append(len(steps_ut)-1)

        for i in range(len(helper_list)-1):
            step_index_1, step_index_2 = helper_list[i], helper_list[i+1]
            xy1 = steps_ut[step_index_1][:2]
            xy2 = steps_ut[step_index_2][:2]
            tree_to_append.append((xy1, xy2))


def add_graph(from_vertex, destination_data, steps_ag, cost, graph, tree_map, tree_lines, geom_skip_factor):
    """
    Helper method to add new edge or update them. This method also call update_tree.

    :param from_vertex: Car object
    :param destination_data: Car object
    :param cost: cost of destination vertex
    :param steps_ag: np.array of xyo
    :param geom_skip_factor: Defines the precision of curves while drawing. Higher values results in course resolution.
    :return: None
    """
    if from_vertex.xyo == destination_data.xyo:
        pass
    graph.add_edge(from_vertex.xyo, destination_data.xyo, cost, directed=True,
                   destination_data=destination_data,
                   set_destination_parent=True)
    tree_map[(from_vertex.xyo, destination_data.xyo)] = steps_ag

    update_tree(destination_data, steps_ag, tree_lines, geom_skip_factor)


def vertices_within_box(reference_vertex, graph, tolerance, gamma, D):
    """
    To get only the vertices within the reference vertex. Radius is defined by balanced box decomposition. For more information, refer http://www.cs.berkeley.edu/~pabbeel/cs287-fa15/optreadings/rrtstar.pdf.
    :param reference_vertex: Vertex object
    :return: List of nearest vertices within the computed radius.
    """
    len_v = len(graph.vertex_map)
    η = tolerance
    radius = max(gamma*((math.log(len_v, 2)/len_v)**(1/D)), η)
    out = []
    for v in graph.vertex_map.values():
        if v.data.xyo == reference_vertex.xyo:
            continue
        if np.linalg.norm(v.data.numpy_vector - reference_vertex.numpy_vector) < radius:
            out.append(v)
    return radius, out


def rrt_star_rs():
    env = Environment(sys.argv[1])

    try:
        data = dict()
        data['l'] = env.environment["car"]["front_axle_back_axle_distance"]
        data['Us'] = env.environment["car"]["linear_velocity"]
        data['Uphi'] = math.radians(env.environment["car"]["max_turning_angle"])
        data["ro_min"] = abs(data["l"]/math.tan(data["Uphi"]))
        data["delta_t"] = data["ro_min"]/env.environment["total_steps_for_complete_integration"]
        data["epsilon"] = env.environment["epsilon"]     # distance per step (cm, m)   # not required here though
        data["time_to_cover_entire_perimeter"] = 2*math.pi*data["ro_min"]/data["Us"]  # distance/velocity
        data["collision_check_skip_factor"] = env.environment["collision_check_skip_factor"]
        data["tolerance"] = env.environment["tolerance"]
        data["theta_norm"] = np.array(env.environment["theta_norm"])
        ReedsShepp.data = data
    except KeyError as k:
        print(k)
        print("ReedsShepp.DataNotDefinedException not defined")
        exit(3)

    rand_points = env.environment["random_points"]
    theta_norm = np.array(env.environment["theta_norm"])
    tolerance = env.environment["tolerance"]
    geom_skip_factor = env.environment["geom_skip_factor"]

    graph = Graph()
    tree_lines, path_lines = [], []
    tree_map = {}

    best_z_near_count = 0
    re_wiring_count = 0         # to keep track of the count of re-wirings made.

    D = 2   # Hausdorff dimension
    # Gamma is based on rrt* paper by Sertac Karaman and is http://acl.mit.edu/papers/Luders13_GNC.pdf pg. no. 10.
    gamma = 2*((1+1/D)**-1)*((env.get_free_area()/math.pi)**(1/D))

    def distance_fn(val, compare_with):
        """
        Weighted knn. val and compare_with should be of the form [x, y, orientation]

        :param val: First vector
        :param compare_with: Second vector
        :return: Weighted magnitude
        """
        if not isinstance(val, np.ndarray):
            val = np.array(val)
        if not isinstance(compare_with, np.ndarray):
            compare_with = np.array(compare_with)
        # if the theta value in any of the two list is negative, we have to convert it to positive.
        # this is ensure for example 178, -178 are taken cared of.
        if compare_with[2] < 0:
            compare_with[2] += math.pi
        if val[2] < 0:
            val[2] += math.pi
        return np.linalg.norm((val-compare_with)*theta_norm)

    def env_check(xyo):
        """
        To perform collision detection, checking if the car is within the boundary and to make the point is not added already in graph vertex to make sure the behaviour is predictable.

        :param xyo:
        :return:
        """
        return (not env.is_car_inside(xyo)) and env.is_car_within_boundary(xyo) and xyo not in graph.vertex_map

    def generate_rrt():
        """
        Driver method for running RRTStar algorithm.
        """
        x_initial = ReedsShepp(tuple(env.initial_state), env.initial_orientation, 0, data,
                               env_check=env_check, action="i")     # starting vertex
        graph.add_vertex(x_initial.xyo, x_initial)
        nonlocal best_z_near_count
        nonlocal re_wiring_count

        goal = np.array(env.goal_state + [env.goal_orientation])
        goal_xy = tuple(env.goal_state)
        goal_tuple = tuple(goal)

        for rp_count in range(0, rand_points):
            print("%d\r" % rp_count, end="")        # to keep track of current sampling number.
            if rp_count == rand_points-1:           # at the end, we adding goal as the random sample.
                x_rand = goal
            else:
                x_rand = rand_state(env)

            # Take the nearest vertex.
            z_nearest = min(graph.vertex_map.keys(), key=lambda val: distance_fn(val, x_rand))
            z_nearest = graph.vertex_map[z_nearest]
            # In the following steer method, within_tolerance=False since we want to go as close as possible and
            # not x_rand completely.
            steer_out = z_nearest.data.steer(x_rand, distance_fn, tolerance, within_tolerance=False)

            if steer_out is None:
                # if there is no possible motions for the z_nearest, then we can ignore that node for tree growth.
                continue

            min_route = steer_out[0]
            t_new, min_cost, min_index, steps = steer_out[1]
            z_new_xyo = steps[-1]
            action = "%s,%d" % (min_route, t_new)
            z_new = ReedsShepp(z_new_xyo[:2], z_new_xyo[2], 0, env_check=env_check, action=action)
            # radius - radius of the bounding circle
            # z_nearby - nearby vertices from z_new within a radius of "radius"
            radius, z_nearby = vertices_within_box(z_new, graph, tolerance, gamma, D)
            # print(radius, len(z_nearby))

            if z_nearest.data.xyo == z_new.xyo:
                # a sanity check in case z_nearest and z_new happen to be same
                continue

            c_min = min_cost + z_nearest.data.cost
            z_new.cost = c_min
            z_min = z_nearest
            z_min_steps = steps

            # keeping tracking of edge_cost is useful while updating the cost of children in re-wiring step.
            edge_cost = min_cost
            # best path for z_new (newly generated point)
            for z_near in z_nearby:
                z_near_steer_out = z_near.data.steer(z_new.numpy_vector, distance_fn, tolerance, within_tolerance=True)
                if z_near_steer_out is None:
                    # if not obstacle free and unable to reach
                    continue
                min_route = z_near_steer_out[0]
                min_t, min_value, min_index, steps = z_near_steer_out[1]
                if z_near.data.cost + min_value < c_min:
                    best_z_near_count += 1
                    # set min
                    c_min = z_near.data.cost + min_value
                    edge_cost = min_value
                    z_new.cost = c_min
                    z_new.action = "%s,%d" % (min_route, t_new)
                    z_min = z_near
                    z_min_steps = steps

            add_graph(z_min.data, z_new, z_min_steps, edge_cost, graph, tree_map, tree_lines, geom_skip_factor)

            # Re-Wiring step
            for z_near in z_nearby:
                if z_near == z_min:
                    continue
                z_near_steer_out = z_new.steer(z_near.data.numpy_vector, distance_fn, tolerance, within_tolerance=True)
                if z_near_steer_out is None:
                    # if not obstacle free and unable to reach
                    continue
                min_route = z_near_steer_out[0]
                min_t, min_value, min_index, steps = z_near_steer_out[1]
                if z_near.data.cost > z_new.cost + min_value:
                    re_wiring_count += 1
                    cost_difference = z_near.data.cost - (z_new.cost + min_value)
                    z_parent = z_near.pi
                    z_parent.remove_edge(z_near)
                    # z_near.cost = z_new.cost + min_value
                    add_graph(z_new, z_near.data, steps, min_value, graph, tree_map, tree_lines, geom_skip_factor)
                    z_near.data.action = "%s,%d" % (min_route, min_t)

                    def dfs_cost_update(vertex, cost):
                        # recursive cost update method using dfs traversal, though it doesn't matter which
                        # traversal method we are using.
                        vertex.data.cost -= cost
                        for adj_e in vertex.adj_vertices.values():
                            dfs_cost_update(adj_e.destination, cost)

                    dfs_cost_update(z_near, cost_difference)

        print("best_z_near_count", best_z_near_count, "re_wiring_count", re_wiring_count)

        ignore_theta = np.array([1, 1, 0])
        # We are simple connecting the goal to the nearest configuration from vertices based on x, y coordinates alone.
        # i.e ., not taking theta into consideration at the final step.
        goal_neigh = min(graph.vertex_map.keys(), key=lambda val: distance_fn(val*ignore_theta, goal*ignore_theta))

        control_input = []      # to store traversal input signal (i.e "st, 10  l, 20")
        complete_path = []      # to store the final traversal as line segments

        if goal_neigh is not None:
            if env.is_line_inside(goal_xy, goal_neigh):
                print("Not able to connect goal with near point")
                return None, None

            if goal_tuple not in graph.vertex_map:
                # create goal car object and connect it to the nearest config.
                goal = ReedsShepp(goal_xy, env.goal_orientation, 0, env_check=env_check, action="f")
                cost = np.linalg.norm((np.array(goal_neigh[:2])-np.array(goal.xy)))
                graph.add_edge(goal_neigh, goal.xyo, cost, destination_data=goal, directed=True,
                               set_destination_parent=True)

                tree_map[(goal_neigh, goal.xyo)] = np.array([goal_neigh, goal.xyo])
            else:
                goal = graph.vertex_map[goal_tuple]
            # traversal_path = graph.min_path(i_state, goal.xyo, return_as_list=True)
            traversal_path = graph.traverse_to(x_initial.xyo, goal.xyo)
            for i in range(0, len(traversal_path) - 1):
                lines = tree_map[(traversal_path[i].data.xyo, traversal_path[i+1].data.xyo)]
                update_tree(traversal_path[i].data, lines, path_lines, geom_skip_factor)

            traversal_path.reverse()
            for tp in traversal_path:
                control_input.append(graph.vertex_map[tp.name].data.action)
                complete_path.append(tp.name)
        else:
            print("Goal not near random points")
        return control_input, complete_path
    control, c_path = generate_rrt()
    if path_lines is None or len(path_lines) == 0:
        print("Path not found. Not enough random points")
    elif print_path:
        for x, y in zip(c_path, control):
            print(x, y)
        print("Path to be followed is", control[::-1])

    in_file_name = "_".join(sys.argv[1].split(".")[:-1])
    output_file_name = "%s_%d_graph.json" % (in_file_name, rand_points)
    with open(output_file_name, "w", encoding="utf-8") as out_file:
        # store the output in a new json file with the filename as mentioned in output_file_name
        data = env.environment
        data["stats"] = {"best_z_near_count": best_z_near_count, "re_wiring_count": re_wiring_count}
        data["control"] = control
        data["path_lines"] = [list(list(ll) for ll in l) for l in path_lines]
        data["tree_lines"] = [list(list(ll) for ll in l) for l in tree_lines]

        out_file.write(json.dumps(data, indent=4, sort_keys=True))

    # draw the final output
    env.draw_env(tree_lines, path_lines)


if __name__ == '__main__':
    rrt_star_rs()
