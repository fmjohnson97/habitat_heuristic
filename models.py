import torch
import cv2
import numpy as np

from matplotlib import pyplot as plt
from sklearn.cluster import KMeans, AgglomerativeClustering
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
        if obs.shape[-1]>1:
            obs = cv2.cvtColor(obs,cv2.COLOR_RGB2GRAY)
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
                moves.append('turn_left')
            else:
                moves.append('turn_right')

        #TODO: check this logic!!! stop condition could be weird. what if item is on ground? should really checkf or depth instead.
        #maybe pass in obs???

        #check if should move forward or stop
        if goal_center[1]-self.y_size//2 > self.y_threshold:
            moves.append('stop') #TODO: should this just be an empty string instead?
        else:
            moves.append('move_forward')

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
        self.distance_threshold = args.cluster_dist_thresh

    @torch.no_grad()
    def getPotentialGoal(self, obs, graph=None):
        # superpoint code borrowed from here: https://github.com/magicleap/SuperGluePretrainedNetwork
        if obs.shape[-1]>1:
            obs = cv2.cvtColor(obs,cv2.COLOR_RGB2GRAY)
        data = {'image0': torch.from_numpy(obs / 255.).float()[None, None].to(self.device)}
        pred = self.superpoint({'image': data['image0']})
        kpts = pred['keypoints'][0]
        scores = pred['scores'][0]
        # descripts = pred['descriptors'][0]

        #only take the ones with scores above the mean; scores tell how "point-ish" a keypoint is
        avg = torch.mean(scores)
        valid = scores>avg
        mkpts = kpts[valid]
        mkpts = mkpts.cpu()
        
        # TODO: check to see if matching features between real goal and current obs
        # and if so, pick subgoal from features that overlap
        
        # TODO: if no goal features, can A) pick largest cluster of points and go there? B) pick furthest away set of 
        # points and go there? not quite sure actually
        
        if graph is None: #or change this to if goal is not found in graph?
            #find portion of image with highest density of dots (for now)
            #TODO: arbitrarily picked 20 to best fit the test case; maybe should change that???
            clustering = AgglomerativeClustering(None, linkage='single', distance_threshold=self.distance_threshold)
            clusters = clustering.fit_predict(mkpts)
            largest_cluster = max(set(clusters), key=list(clusters).count)
            print('largest cluster:', largest_cluster)
            # plt.imshow(obs)
            # for i in range(max(clusters)):
            #     inds=[clusters==i]
            #     print(sum(sum(inds)))
            #     plt.scatter(mkpts[inds[0],0], mkpts[inds[0],1])
            # plt.savefig('gibson_superglue_test.png')
            # plt.show()
            points = mkpts[clusters==largest_cluster]
            min_coords = torch.min(points, axis=0)[0].int()
            max_coords = torch.max(points, axis=0)[0].int()
            goal_img = obs[min_coords[1]:max_coords[1],min_coords[0]:max_coords[0]]
            # plt.imshow(goal_img)
            # plt.show()
            return goal_img
        else:
            return None
            #TODO: figure out how to incorporate graph knowledge of goal location, robot location, and image
            # learned from the videos???

    def step(self, obs):
        goalimg = self.getPotentialGoal(obs)
        return goalimg

class Manager(object):
    def __init__(self, args, device, graph):
        self.device = device
        if type(graph)==str:
            pass
            # TODO: read graph files
        else:
            self.graph = graph

    def

if __name__ == '__main__':
    args = getArgs()
    device = torch.device('cpu')
    testimg = cv2.cvtColor(cv2.imread('test.png'), cv2.COLOR_BGR2GRAY)
    goalimg = cv2.cvtColor(cv2.imread('goal.png'), cv2.COLOR_BGR2GRAY)

    #test upper level manager


    #test mid level manager
    midMan = MidManager(args, device)
    goalimg = midMan.step(testimg)
    plt.imshow(goalimg)
    plt.show()

    #test worker
    worker = Worker(args, device)
    moves = worker.step(testimg, goalimg)
    print(moves)
