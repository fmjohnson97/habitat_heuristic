import cv2

class Graph():
    def __init__(self):
        self.nodes = []
        self.edges = []
        self.current_location = 0
        self.goal_location = None

    def addNode(self, img_path, location, orientation):
        node = Node(img_path, location, orientation)
        self.nodes.append(node)
        self.updateEdges()

    def updateEdges(self):
        if len(self.edges[0])!= len(self.nodes):
            for i in range(len(self.edges)):
                self.edges[i].append(0)
            self.edges.append([0]*len(self.nodes))
            if len(self.nodes)>1:
                self.edges[-1][-2]=1
                self.edges[-2][-1]=1

        #TODO: is there some smarter way to make these graphs more connected?




def Node():
    def __init__(self, img_path, location, orientation):
        self.img_path = img_path
        self.location = location
        # the orientation of the robot when the image was taken
        self.orientation = orientation

    def getImage(self):
        return cv2.cvtColor(cv2.imread(self.img_path),cv2.COLOR_BGR2RGB)

