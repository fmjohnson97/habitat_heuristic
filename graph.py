import json

import cv2


class Graph():
    def __init__(self, goal_location=None):
        self.nodes = []
        self.edges = []
        self.current_node = 0
        self.goal_location = goal_location

    def addNode(self, img_path, location, orientation, depth_path=None):
        node = Node(img_path, location, orientation, depth_path)
        self.nodes.append(node)
        self.current_node = len(self.nodes) - 1
        self.updateEdges()

    def updateEdges(self):
        if len(self.edges) == 0:
            self.edges.append([0])
        elif len(self.edges[0]) != len(self.nodes):
            for i in range(len(self.edges)):
                self.edges[i].append(0)
            self.edges.append([0] * len(self.nodes))
            if len(self.nodes) > 1:
                self.edges[-1][-2] = 1
                self.edges[-2][-1] = 1
        # TODO: is there some smarter way to make these graphs more connected?

    def save(self, save_path):
        data = {}
        for i, node in enumerate(self.nodes):
            data['node' + str(i)] = node.getSaveDict()
        data['edges'] = self.edges
        data['goal_location'] = self.goal_location
        with open(save_path, 'w') as f:
            json.dump(data, f)


class Node():
    def __init__(self, img_path, location, orientation=None, depth_path=None):
        self.img_path = img_path
        self.depth_path = depth_path
        self.location = location
        # the orientation of the robot when the image was taken
        self.orientation = orientation

    def getImage(self):
        if self.img_path:
            return cv2.cvtColor(cv2.imread(self.img_path), cv2.COLOR_BGR2RGB)
        else:
            return None

    def getDepth(self):
        if self.depth_path:
            return cv2.imread(self.depth_path)  # TODO: might have to cast this as grayscale??? bc single value?
        else:
            return None

    def getSaveDict(self):
        return {'img_path': self.img_path,
                'depth_path': self.depth_path,
                'location': self.location,
                'orientation': self.orientation}
