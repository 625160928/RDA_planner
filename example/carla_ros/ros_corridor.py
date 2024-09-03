from ir_sim.env import EnvBase
import numpy as np
from RDA_planner.mpc import MPC
from collections import namedtuple
from gctl.curve_generator import curve_generator
from map_process import read_json_file
import yaml
# map_config = 'corridor.yaml'
# map_config = './example/carla_ros/writeYamlData.yaml'
map_config = './map/writeYamlData.yaml'


# start and goal point of the robot
start_point = np.array([[0], [20], [0]])
goal_point = np.array([[60], [20], [0]])


with open(map_config, 'r', encoding='utf-8') as f:
    map_data= yaml.load(f.read(), Loader=yaml.FullLoader)
    # print(map_data['robot']['state'],map_data['robot']['goal'])
    start_point = np.array([[map_data['robot']['state'][0]], [map_data['robot']['state'][1]], [map_data['robot']['state'][2]]])
    goal_point = np.array([[map_data['robot']['goal'][0]], [map_data['robot']['goal'][1]], [map_data['robot']['goal'][2]]])
    print(map_data['world']['width'],map_data['world']['height'])
# start_point = np.array([[map_data['robot']['state']], [20], [0]])
# goal_point = np.array([[60], [20], [0]])
print(start_point,goal_point)

point_list = [start_point, goal_point]

# generate dubins curve
cg = curve_generator()
ref_path_list = cg.generate_curve('dubins', point_list, 0.1, 5)

# init simulated environment
robot_init_point = np.zeros((4, 1))
robot_init_point[0:3] = ref_path_list[0][0:3]

env = EnvBase(map_config, save_ani=False, full=False, display=True)
car = namedtuple('car', 'G h cone_type wheelbase max_speed max_acce dynamics')  # robot information

env.draw_trajectory(ref_path_list, traj_type='-k')

if __name__ == '__main__':


    # obs_list = env.get_obstacle_list()
    robot_info = env.get_robot_info()
    car_tuple = car(robot_info.G, robot_info.h, robot_info.cone_type, robot_info.wheelbase, [10, 1], [10, 0.5], 'acker')
    mpc_opt = MPC(car_tuple, ref_path_list, sample_time=env.step_time, max_edge_num=4, max_obs_num=6)

    for i in range(500):

        obs_list = env.get_obstacle_list()
        opt_vel, info = mpc_opt.control(env.robot.state, 4, obs_list)

        env.draw_trajectory(info['opt_state_list'], 'r', refresh=True)

        env.step(opt_vel)
        env.render(show_traj=True, show_trail=True)

        if info['arrive']:
            print('arrive at the goal')

        if env.done():
            print('Done')
            break

    env.end(ani_name='corridor', show_traj=True, show_trail=True, ending_time=10, ani_kwargs={'subrectangles': True})

