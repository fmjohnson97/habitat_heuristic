import torch
import argparse
import habitat

from habitat.sims.habitat_simulator.actions import HabitatSimActions
import cv2

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

    env = habitat.Env(config=habitat.get_config("/home/faith"))
    print('finished')