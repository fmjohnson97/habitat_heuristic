import torch
import argparse
import habitat
import habitat_sim
import cv2
import random
import numpy as np

from habitat.sims.habitat_simulator.actions import HabitatSimActions
from habitat_sim.utils.settings import default_sim_settings, make_cfg

from habitat_utils import get_split_files

def getArgs():
    parser = argparse.ArgumentParser()

    # Superglue arguments
    parser.add_argument('--nms_radius', type=int, default=4, help='SuperPoint Non Maximum Suppression (NMS) radius (Must be positive)')
    parser.add_argument('--sinkhorn_iterations', type=int, default=20, help='Number of Sinkhorn iterations performed by SuperGlue')
    parser.add_argument('--match_threshold', type=float, default=0.2, help='SuperGlue match threshold')
    parser.add_argument('--superglue', choices={'indoor', 'outdoor'}, default='indoor', help='SuperGlue weights')
    parser.add_argument('--max_keypoints', type=int, default=1024, help='Maximum number of keypoints detected by Superpoint (\'-1\' keeps all keypoints)')
    parser.add_argument('--keypoint_threshold', type=float, default=0.005, help='SuperPoint keypoint detector confidence threshold')
    parser.add_argument('--x_size', type=int, default=640, help='x dimension of the image')
    parser.add_argument('--y_size', type=int, default=480, help='y dimension of the image')

    return parser.parse_args()



if __name__ == '__main__':
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    test_scene_files = get_split_files(split='test',env='gibson')
    test_scene = random.choice(list(test_scene_files['all']))
    sim_settings = {"scene_dataset_config_file": "default",
                    "scene":test_scene,
                    "default_agent":0,
                    "sensor_height":1.5,
                    "width":640,
                    "height":480,
                    "hfov": 90,
                    "zfar": 1000.0,
                    "color_sensor": True,
                    "semantic_sensor": False,
                    "depth_sensor": False,
                    "ortho_rgba_sensor": False,
                    "ortho_depth_sensor": False,
                    "ortho_semantic_sensor": False,
                    "fisheye_rgba_sensor": False,
                    "fisheye_depth_sensor": False,
                    "fisheye_semantic_sensor": False,
                    "equirect_rgba_sensor": False,
                    "equirect_depth_sensor": False,
                    "equirect_semantic_sensor": False,
                    "seed": 1,
                    "physics_config_file": "data/default.physics_config.json",
                    "enable_physics": True}
    habitat_cfg = make_cfg(sim_settings)
    sim = habitat.Simulator(habitat_cfg)
    # initialize an agent
    agent = sim.initialize_agent(sim_settings["default_agent"])
    # Set agent state
    agent_state = habitat_sim.AgentState()
    #TODO: change this to a random place in the environment bounds
    agent_state.position = np.array([-0.6, 0.0, 0.0])  # in world space
    agent.set_state(agent_state)
    # Get agent state
    agent_state = agent.get_state()
    print("agent_state: position", agent_state.position, "rotation", agent_state.rotation)
    # obtain the default, discrete actions that an agent can perform
    # default action space contains 3 actions: move_forward, turn_left, and turn_right
    action_names = list(habitat_cfg.agents[sim_settings["default_agent"]].action_space.keys())
    print("Discrete action space: ", action_names)

    print('finished')