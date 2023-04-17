import torch
import cv2
import numpy as np

from matplotlib import pyplot as plt
from matterport_test import getArgs
from SuperGluePretrainedNetwork.models.matching import Matching
from SuperGluePretrainedNetwork.models.superpoint import SuperPoint

class Worker(object):
    def __init__(self, args, device):
        self.superglue_config = {
            'superpoint': {
                'nms_radius': args.nms_radius,
                'keypoint_threshold': args.keypoint_threshold,
                'max_keypoints': args.max_keypoints
            },
            'superglue': {
                'weights': args.superglue,
                'sinkhorn_iterations': args.sinkhorn_iterations,
                'match_threshold': args.match_threshold,
            }
        }
        self.device = device
        self.superglue = Matching(self.superglue_config).eval().to(device)
        self.x_size = args.x_size
        self.y_size = args.y_size
        self.x_threshold = args.x_size//10
        self.y_threshold = args.y_size // 10

    @torch.no_grad()
    def identifyGoalPatch(self, obs, goal):
        # superglue code borrowed from here: https://github.com/magicleap/SuperGluePretrainedNetwork
        # get superglue to find salient points in obs and goal and match them
        #TODO: may need to resize the image to 480x640?
        # data = {'image0':torch.tensor([obs]).byte().unsqueeze(1).to(self.device), 'image1':torch.ByteTensor([goal]).byte().unsqueeze(1).to(self.device)}
        data = {'image0':torch.from_numpy(obs/255.).float()[None, None].to(self.device),
                'image1':torch.from_numpy(goal/255.).float()[None, None].to(self.device)}
        pred = self.superglue(data)
        pred = {k: v[0].cpu().numpy() for k, v in pred.items()}
        kpts0, kpts1 = pred['keypoints0'], pred['keypoints1']
        matches, conf = pred['matches0'], pred['matching_scores0']
        valid = [i for i in range(len(matches)) if matches[i] > -1 and conf[i]>=0.75]
        mkptsOBS = kpts0[valid]
        # mkptsGOAL = kpts1[matches[valid]]
        # mconf = conf[valid]

        # find center of goal keypoint matches and return
        goal_center = np.round(np.mean(mkptsOBS, axis=0)).astype(int)
        return goal_center

    def moveTowardGoal(self, goal_center):
        moves = []
        #check if should turn left or right or not
        if abs(goal_center[0]-self.x_size//2)> self.x_threshold:
            if goal_center[0]-self.x_size//2 < 0:
                moves.append('turn left')
            else:
                moves.append('turn right')

        #TODO: check this logic!!! stop condition could be weird. what if item is on ground? should really checkf or depth instead.
        #maybe pass in obs???

        #check if should move forward or stop
        if goal_center[1]-self.y_size//2 > self.y_threshold:
            moves.append('stop')
        else:
            moves.append('forward')

        return moves

    def step(self, obs, goal):
        goal_center = self.identifyGoalPatch(obs, goal)
        moves = self.moveTowardGoal(goal_center)
        #TODO: actually execute the moves in the environment
        return moves


class MidManager(object):
    def __init__(self, args, device):
        self.superpoint_config = {
            'nms_radius': args.nms_radius,
            'keypoint_threshold': args.keypoint_threshold,
            'max_keypoints': args.max_keypoints
        }
        self.superpoint = SuperPoint(self.superpoint_config).to(device)
        self.device = device

    def getPotentialGoalKeypoints(self, obs):
        # superpoint code borrowed from here: https://github.com/magicleap/SuperGluePretrainedNetwork
        breakpoint()
        data = {'image0': torch.from_numpy(obs / 255.).float()[None, None].to(self.device)}
        pred = self.superpoint({'image': data['image0']})
        kpts = pred['keypoints'][0]
        scores = pred['scores'][0]
        descripts = pred['descriptors'][0]


    def narrowDownKeypoints(self, obs, keypoints):
        breakpoint()
        # cluster the points into objects based on distance??? kmeans???
        # in theory the

    def step(self, obs):
        potentialGoalKeypoints = self.getPotentialGoalKeypoints(obs)
        goalimg = self.narrowDownKeypoints(obs, potentialGoalKeypoints)
        return goalimg

if __name__ == '__main__':
    args = getArgs()
    device = torch.device('cpu')
    testimg = cv2.cvtColor(cv2.imread('test.png'), cv2.COLOR_BGR2GRAY)
    goalimg = cv2.cvtColor(cv2.imread('goal.png'), cv2.COLOR_BGR2GRAY)

    #test mid level manager
    midMan = MidManager(args, device)
    mangoalimg = midMan.step(testimg)
    plt.imshow(mangoalimg)
    plt.show()

    #test worker
    worker = Worker(args, device)
    moves = worker.step(testimg, goalimg)
    print(moves)
