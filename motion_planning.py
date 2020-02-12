import argparse
import time
import msgpack
from enum import Enum, auto
import random
import numpy as np
import copy

from skimage.morphology import medial_axis, erosion
from skimage.util import invert
from visualize_grid import plot_graph, plot_path
import matplotlib.pyplot as plt


from medial_graph import skeleton_to_graph, simplify_graph
from astar import astar_graph_wrapper
from grid import create_grid, read_lat_lon
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local, local_to_global


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


def prepare_medial_axis_graph(
    TARGET_ALTITUDE: float, SAFETY_DISTANCE: float, plot=False, tol=3.0
):
    """
    Prepare the graph abstracted from the medial transforma
    This takes a while so we do it ahead of time
    :return:
    """
    data = np.loadtxt("colliders.csv", delimiter=",", dtype="Float64", skiprows=2)

    # Define a grid for a particular altitude and safety margin around obstacles
    grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, 0)
    inv_grid = invert(grid)
    # let's make the boundaries to obstacles too so we don't stick to them
    inv_grid[0, :] = 0
    inv_grid[-1, :] = 0
    inv_grid[:, 0] = 0
    inv_grid[:, -1] = 0

    # this is more elegant as doesn't rely on us knowing building
    # representations as blocks with centers
    for _ in range(int(SAFETY_DISTANCE * 1.5)):
        inv_grid = erosion(inv_grid)

    skeleton = medial_axis(inv_grid)
    g = skeleton_to_graph(skeleton)
    g2 = simplify_graph(copy.deepcopy(g), tol=tol)
    if plot:
        print("plotting...")
        plt.imshow(invert(inv_grid) + grid, origin="lower")
        plot_graph(g2, "b")
    print("returning medial graph")

    return g2


class MotionPlanning(Drone):
    def __init__(self, connection, TARGET_ALTITUDE, SAFETY_DISTANCE, graph, plot=False):
        super().__init__(connection)

        self.plot = plot
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)
        self.g2 = graph
        self.TARGET_ALTITUDE = TARGET_ALTITUDE
        self.SAFETY_DISTANCE = SAFETY_DISTANCE

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if (
                np.linalg.norm(self.target_position[0:2] - self.local_position[0:2])
                < 1.0
            ):
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print("target position", self.target_position)
        self.cmd_position(
            self.target_position[0],
            self.target_position[1],
            self.target_position[2],
            self.target_position[3],
        )

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")

        self.target_position[2] = self.TARGET_ALTITUDE

        # DONE: read lat0, lon0 from colliders into floating point values
        lat0, lon0 = read_lat_lon("colliders.csv")

        # DONE: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # DONE: retrieve current global position
        gp = self.global_position

        # DONE: convert to current local position using global_to_local()
        lp = global_to_local(gp, self.global_home)

        print(
            "global home {0}, position {1}, local position {2}".format(
                self.global_home, self.global_position, self.local_position
            )
        )
        # Read in obstacle map
        data = np.loadtxt("colliders.csv", delimiter=",", dtype="Float64", skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(
            data, self.TARGET_ALTITUDE, self.SAFETY_DISTANCE
        )
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        # DONE: convert start position to current position rather than map center
        grid_start = (
            int(-north_offset + self.local_position[0]),
            int(-east_offset + self.local_position[1]),
        )

        # Set goal as some arbitrary position on the grid
        while True:
            grid_target = (
                random.randint(0, grid.shape[0] - 1),
                random.randint(0, grid.shape[1] - 1),
            )
            if grid[grid_target[0], grid_target[1]] == 0:
                break

        pre_grid_goal = [
            north_offset + grid_target[0],
            east_offset + grid_target[1],
            self.TARGET_ALTITUDE,
        ]
        lat_lon_goal = local_to_global(pre_grid_goal, self.global_home)
        # DONE: adapt to set goal as latitude / longitude position and convert
        grid_goal = global_to_local(lat_lon_goal, self.global_home)
        # now apply the offsets and round so it's in grid index coordinates
        grid_goal = (int(grid_goal[0]) - north_offset, int(grid_goal[1]) - east_offset)
        # Run A* to find a path from start to goal
        # DONE: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # added
        # or move to a different search space such as a graph (not done here)

        # DONE (if you're feeling ambitious): Try a different approach altogether!
        print("Local Start and Goal: ", grid_start, grid_goal)

        start_ne_g, goal_ne_g, path, cost = astar_graph_wrapper(
            self.g2, grid_start, grid_goal
        )
        print("planning done!")
        path = [p[0] for p in path]
        if self.plot:
            print("plotting path on graph...")
            plot_path(grid_start, grid_goal, path)
            plt.show()
            print("done!")

        # Convert path to waypoints
        waypoints = [
            [p[0] + north_offset, p[1] + east_offset, self.TARGET_ALTITUDE, 0]
            for p in path
        ]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=5760, help="Port number")
    parser.add_argument(
        "--host", type=str, default="127.0.0.1", help="host address, i.e. '127.0.0.1'"
    )
    args = parser.parse_args()

    TARGET_ALTITUDE = 5
    SAFETY_DISTANCE = 5
    # graph creation takes a bit of time, leading to potential timeout errors from the drone
    # so we do it ahead of time
    plot = False
    graph = prepare_medial_axis_graph(TARGET_ALTITUDE, SAFETY_DISTANCE, plot, tol=4)

    conn = MavlinkConnection("tcp:{0}:{1}".format(args.host, args.port), timeout=300)

    drone = MotionPlanning(conn, TARGET_ALTITUDE, SAFETY_DISTANCE, graph, plot)
    time.sleep(1)

    drone.start()
