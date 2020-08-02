import matplotlib.pyplot as plt
from graph import Graph
class Detector(object):
    def __init__(self):
        super().__init__()

    def Detect(self,graph,debug = False):
        l = [[],[]]
        #i am not sure we still need the ellipse to fit the size
        #this case i use the orginal controid
        for cell in graph.cells.values():
            l[0].append(cell.COM())
            l[1].append(cell.id)
            if debug:
                plt.plot(cell.COM()[0],cell.COM()[1],marker = "o")
                plt.draw()
        
        if debug:
            plt.clf()
            plt.show()
        return l

if __name__ == "__main__":
    img = plt.imread("mask_DIC/mask008.tif")
    plt.imshow(img)
    plt.draw()
    size = img.shape
    width = size[0]
    height  = size[1]

    detector = Detector()
    graph = Graph(img,height,width)
    
    centers = detector.Detect(graph,True)
