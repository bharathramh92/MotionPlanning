import math
import numpy as np
import matplotlib.pyplot as plt


class DataNotDefinedException(Exception):
    """
    static variable "data" a dictionary needs to be defined before using this class. "data" should have the following attributes.

    - data['l'] : distance between front_axle and rear axle of the car
    - data['Us'] : linear_velocity of the car
    - data['Uphi'] : maximum turning angle of the car (same of all turns)
    - data["ro_min"] : minimum turning radius, given by abs(data["l"]/math.tan(data["Uphi"]))
    - data["delta_t"] : h value for 4th order runge kutta integration
    - data["epsilon"] : step size for integration. Required for RRT and not for RRTStar
    - data["time_to_cover_entire_perimeter"] : time to cover entire perimeter for the car, given by  2*math.pi*data["ro_min"]/data["Us"]. i.e, distance/velocity.
    - data["collision_check_skip_factor"] : how often the collision check should be skipped while integrating.
    - data["tolerance"] : maximum tolerance between any match in configuration
    - data["theta_norm"] : [x, y, theta] normalization vector for the distance function.
    """
    pass


class ReedsShepp:
    plot = False

    data = None     # static variable to hold the car data. If undefined, raises exception.

    def __init__(self, xy, orientation, cost, variable_time_integration=True, env_check=None, action=None):
        """
        Car class to define ReedsShepp car object. Any mentioning of configuration mean [x, y, orientation] data
        structure.

        :param xy: xy coordinate tuple of the rear axle center
        :param orientation: orientation angle of the car w.r.t to x-axis
        :param cost: cost to reach the current configuration from initial configuration
        :param variable_time_integration: True if the integration needs to be performed over a variable time, and is used for RRT*. False if the integration step size is fixed, and is used for RRT.
        :param env_check: Function to check if the given configuration is intersecting an obstacle of not. Function should accept an argument of tuple/list/nd.array which states and center point of rear axle and orientation. (x, y, orientation) is the format.
        :param action: A String, which states on what action ("straight", "left", "right", "reverse", "left_reverse", "right_reverse") the current object is derived from the parent and the time for which the control was given.
        :return: None
        """
        if ReedsShepp.data is None:
            raise DataNotDefinedException
        for t in ["l", "Us", "Uphi", "ro_min", "delta_t", "epsilon", "time_to_cover_entire_perimeter", "collision_check_skip_factor"]:
            if t not in ReedsShepp.data:
                raise DataNotDefinedException
        self.xy = tuple(xy)
        self.orientation = self._roll_over(float(orientation))
        self.xyo = self.xy + (self.orientation, )
        self.numpy_vector = np.array(self.xyo)

        self.action = action
        self.cost = cost

        self.env_check = env_check

        self.steer_possibilities_vr = dict()
        self.steer_possibilities_fixed = dict()

        self.variable_time_integration = variable_time_integration

        if variable_time_integration:
            self._generate_steer_possibilities_vr()
        else:
            self._generate_steer_possibilities_fixed()

    def __repr__(self):
        return self.xyo

    def _roll_over(self, angle):
        """
        Converting an angle which is not bounded by any limit to an angle in -math.pi to math.pi range.
        eg: math.radians(182) = math.radians(-172)

        :param angle: in radians
        :return: Angle in -math.pi to math.pi
        """
        return ((np.array([angle]) + np.pi) % (2 * np.pi) - np.pi)[0]

    def _fn(self, t, data, us_fn, uphi_fn):
        """
        Function that defines the differential constraint for ReedsShepp car.Differential constraints for ReedsShepp Car is as mentioned in http://planning.cs.uiuc.edu/ch13.pdf pg. 6 of the pdf.

        :param t: time. Here this doesn't matter as the differential equation doesn't have a variable t.
        :param data: (x, y, orientation) tuple or list or nd.array.
        :param us_fn: Linear velocity in m/s or cm/s.
        :param uphi_fn: Turning angle of the car
        :return: Next state after integrating or time t.
        """
        x_dot = us_fn * math.cos(data[2])
        y_dot = us_fn * math.sin(data[2])
        theta_dot = us_fn * math.tan(uphi_fn) / self.data["l"]
        return np.array([x_dot, y_dot, theta_dot])

    def _runge_kutta_helper(self, ur_h, uphi_h, epsilon):
        """
        Helper function for runge-kutta 4th order integration.

        :param ur_h: Linear velocity control input. A number.
        :param uphi_h: Turning angle control input. A number.
        :param epsilon: Step size of integration.
        :return: None if no possible steps, else a numpy array of numpy array which gives the information of on x, y, orientation upon each integrating steps (h).
        """
        h = ReedsShepp.data["delta_t"]
        initial = np.array([self.xy[0], self.xy[1], self.orientation])
        current = initial.copy()
        steps = [initial]

        for i, t in enumerate(np.arange(h, epsilon, h)):
            k1 = self._fn(t, current, ur_h, uphi_h)
            k2 = self._fn(t + h / 2, current + 0.5 * h * k1, ur_h, uphi_h)
            k3 = self._fn(t + h / 2, current + 0.5 * h * k2, ur_h, uphi_h)
            k4 = self._fn(t + h, current + h * k3, ur_h, uphi_h)
            current += h*(k1 + 2*k2 + 2*k3 + k4)/6
            current[2] = self._roll_over(current[2])

            if i % self.data["collision_check_skip_factor"] == 0 and self.env_check and not self.env_check(tuple(current)):
                # do collision check only for some interval
                # upon env_check is true, integration method breaks.
                break
            steps.append(current.copy())

        steps = np.array(steps)

        if ReedsShepp.plot:
            plt.plot(steps[:, 0], steps[:, 1])
            plt.ylim([0, 100])
            plt.xlim([0, 100])
            plt.show()

        if len(steps) == 1:
            return None
        else:
            return steps

    def _runge_kutta_fixed(self, us_rk, uphi_rk):
        """
        4th order runge-kutta integration method with fixed step size given in data["epsilon"]. Note that the integration breaks once the car hits an obstacle. Return next state after integrating over the step size.

        :param us_rk: Linear velocity control input. A number.
        :param uphi_rk: Turning angle control input. A number.
        :return: None if no possible steps, else a numpy array of numpy array which gives the information of on x, y, orientation upon each integrating steps (h).
        """
        return self._runge_kutta_helper(us_rk, uphi_rk, self.data["epsilon"])

    def _runge_kutta_variable_step(self, Us_rk, Uphi_rk):
        """
        4th order runge-kutta integration method step size limited to data["time_to_cover_entire_perimeter"]. Note that the integration breaks once the car hits an obstacle. Return next state after integrating over the step size.

        :param us_rk: Linear velocity control input. A number.
        :param uphi_rk: Turning angle control input. A number.
        :return: None if no possible steps, else a numpy array of numpy array which gives the information of on x, y, orientation upon each integrating steps (h).
        """
        return self._runge_kutta_helper(Us_rk, Uphi_rk, self.data["time_to_cover_entire_perimeter"])

    def _generate_steer_possibilities_vr(self):
        """
        Function to generate all possible 6 steer methods whose integration time is limited to data["time_to_cover_entire_perimeter"] for the car at this current configuration. This function updates self.steer_possibilities_vr dictionary.

        :return: None
        """
        st = self._runge_kutta_variable_step(ReedsShepp.data["Us"], 0)
        b = self._runge_kutta_variable_step(-ReedsShepp.data["Us"], 0)

        r = self._runge_kutta_variable_step(ReedsShepp.data["Us"], -ReedsShepp.data["Uphi"])
        rb = self._runge_kutta_variable_step(-ReedsShepp.data["Us"], -ReedsShepp.data["Uphi"])

        l = self._runge_kutta_variable_step(ReedsShepp.data["Us"], ReedsShepp.data["Uphi"])
        lb = self._runge_kutta_variable_step(-ReedsShepp.data["Us"], ReedsShepp.data["Uphi"])

        if st is not None:
            self.steer_possibilities_vr['st'] = st
        if b is not None:
            self.steer_possibilities_vr['b'] = b
        if r is not None:
            self.steer_possibilities_vr['r'] = r
        if rb is not None:
            self.steer_possibilities_vr['rb'] = rb
        if l is not None:
            self.steer_possibilities_vr['l'] = l
        if lb is not None:
            self.steer_possibilities_vr['lb'] = lb

    def _generate_steer_possibilities_fixed(self):
        """
        Function to generate all possible 6 steer methods whose integration time is limited to data["epsilon"] for the car at this current configuration. This function updates self.steer_possibilities_fixed dictionary.

        :return: None
        """
        st = self._runge_kutta_fixed(ReedsShepp.data["Us"], 0)
        b = self._runge_kutta_fixed(-ReedsShepp.data["Us"], 0)

        r = self._runge_kutta_fixed(ReedsShepp.data["Us"], -ReedsShepp.data["Uphi"])
        rb = self._runge_kutta_fixed(-ReedsShepp.data["Us"], -ReedsShepp.data["Uphi"])

        l = self._runge_kutta_fixed(ReedsShepp.data["Us"], ReedsShepp.data["Uphi"])
        lb = self._runge_kutta_fixed(-ReedsShepp.data["Us"], ReedsShepp.data["Uphi"])

        if st is not None:
            self.steer_possibilities_fixed['st'] = st
        if b is not None:
            self.steer_possibilities_fixed['b'] = b
        if r is not None:
            self.steer_possibilities_fixed['r'] = r
        if rb is not None:
            self.steer_possibilities_fixed['rb'] = rb
        if l is not None:
            self.steer_possibilities_fixed['l'] = l
        if lb is not None:
            self.steer_possibilities_fixed['lb'] = lb

    def steer(self, z_end, distance_fn, tolerance_steer, within_tolerance=False):
        """
        Obtains the best possible steer method to reach z_end configuration from the current configuration. This method takes the best of best configuration of all possible steer types ("straight", "left", etc). Note that this method considers variable integration. Raises exception if self.variable_time_integration is False.

        :type tolerance_steer: maximum tolerence for the distance_fn.
        :param z_end: The destination configuration.
        :param distance_fn: Distance function for determining the closest neighbour.
        :param within_tolerance: boolean. True if the destination needs to be reached within the tolerant limit, and False if the car needs to reach z_end as close as possible.
        :return: None if no possible motions else, a tuple of following format. (min_steer_name, (min_t, min_value, min_index, steps)), where min_steer_name is the name of the action, min_t is the integration time, min_value is the value from distance fuction between z_end and config at min_t and steps is the nd.array for all the intermediate configurations.
        """
        if not self.variable_time_integration:
            raise EnvironmentError

        if len(self.steer_possibilities_vr) == 0:
            return None

        if not isinstance(z_end, np.ndarray):
            z_end = np.array(z_end)

        min_possible_path = {}

        for k, v in self.steer_possibilities_vr.items():
            min_index = 0
            min_value = distance_fn(v[0], z_end)
            for vv in range(1, len(v)):
                dist = distance_fn(v[vv], z_end)
                if dist < min_value:
                    min_value = dist
                    min_index = vv
            min_possible_path[k] = (min_index, min_value)

        min_min_possible_path = min(min_possible_path.items(), key=lambda kv: kv[1][1])

        min_steer_name = min_min_possible_path[0]
        min_index = min_min_possible_path[1][0]
        distance_between_nearest_and_z_end = min_min_possible_path[1][1]
        min_t = (min_index + 1)*ReedsShepp.data["delta_t"]
        min_value = min_t*ReedsShepp.data["delta_t"]
        steps = self.steer_possibilities_vr[min_steer_name][:min_index+1]

        if within_tolerance and distance_between_nearest_and_z_end > tolerance_steer:
            return None
        else:
            return min_steer_name, (min_t, min_value, min_index, steps)

    def get_next_best_state(self, nearest_conf, distance_function):
        """
         Obtains the best possible steer method to reach nearest_conf configuration  as close as possible from the current configuration. Note that this method considers fixed integration. Raises exception if self.variable_time_integration is True.

        :param nearest_conf: The destination configuration.
        :param distance_function: Distance function for determining the closest neighbour.
        :return: None if no possible motions else, a tuple with control name at first index (0) and np.array of next possible configurations at second index (1).
        """
        if self.variable_time_integration:
            raise EnvironmentError

        if len(self.steer_possibilities_fixed) == 0:
            return None

        final_possible_position = {k: v[-1] for k, v in self.steer_possibilities_fixed.items()}

        key, val = min(final_possible_position.items(), key=lambda v: distance_function(v[1], nearest_conf))
        return key, self.steer_possibilities_fixed[key]

if __name__ == "__main__":
    # for standalone testing
    ReedsShepp.plot = False

    data_setter = dict()
    data_setter['l'] = 25.62
    data_setter['Us'] = 1
    data_setter['Uphi'] = math.radians(50)
    data_setter["ro_min"] = abs(data_setter["l"]/math.tan(data_setter["Uphi"]))
    data_setter["delta_t"] = data_setter["ro_min"]/45
    data_setter["epsilon"] = 10     # distance per step (cm, m)   # not required here though
    data_setter["time_to_cover_entire_perimeter"] = 2*math.pi*data_setter["ro_min"]/data_setter["Us"]  # distance/velocity
    data_setter["collision_check_skip_factor"] = 5
    data_setter["theta_norm"] = np.array([1, 1, 5])

    ReedsShepp.data = data_setter

    tolerance = 1

    np.set_printoptions(suppress=True)

    def distance_fn_test(val, compare_with):
        if not isinstance(val, np.ndarray):
            val = np.array(val)
        if not isinstance(compare_with, np.ndarray):
            compare_with = np.array(compare_with)
        if compare_with[2] < 0:
            compare_with[2] += math.pi
        if val[2] < 0:
            val[2] += math.pi
        return np.linalg.norm((val-compare_with)*ReedsShepp.data["theta_norm"])

    dc_vr_test = ReedsShepp((50, 50), math.pi / 2, 0)

    z_end_main = np.array([50.2, 59, math.pi / 2 + 0.1])     # s
    z_end_main = np.array([50.1, 39, math.pi / 2 + 0.1])     # b
    z_end_main = np.array([59, 59, 0])                   # r
    z_end_main = np.array([59, 42, math.pi])             # rb
    z_end_main = np.array([42, 59, math.pi])             # l
    z_end_main = np.array([42, 42, 0.2])                 # lb
    #
    print(dc_vr_test.steer(z_end_main, distance_fn_test, tolerance, within_tolerance=True))
    print(dc_vr_test.steer(z_end_main, distance_fn_test, tolerance, within_tolerance=False))

    dc_fixed_test = ReedsShepp((50, 50), math.pi / 2, 0, variable_time_integration=False)
    print(dc_fixed_test.get_next_best_state(z_end_main, distance_fn_test))

