import matplotlib.pyplot as plt
import cv2
import numpy as np
class Cell():
    def __init__(self,id,empty,graph):
        super().__init__()
        self.id = id
        self.pixelx = []
        self.pixely = []
        self.pairs = []
        self.controid = None
        self.empty = empty
        self.graph = graph


    def addPixel(self,x,y):
        self.pixelx.append(x)
        self.pixely.append(y)
        self.pairs.append((x,y))
        self.empty[x][y] = 1

    def COM(self):
        M = cv2.moments(self.empty)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return (cX,cY)
        # Cx = int ((max(self.pixelx) + min(self.pixelx))/2)
        # Cy = int ((max(self.pixely) + min(self.pixely))/2)
        # print(len(self.pixelx))
        # print(len(self.pixelx)) 
        # return (Cx,Cy)
    def COM1(self):
        Cx = int ((max(self.pixelx) + min(self.pixelx))/2)
        Cy = int ((max(self.pixely) + min(self.pixely))/2)

        return (Cx,Cy)
    def showM(self):
        plt.imshow(self.empty)
        plt.show()

    def Ellipse(self):
        ret = cv2.convertScaleAbs(self.empty)
        contours, hierarchy = cv2.findContours(ret, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        ellipse = cv2.fitEllipse(contours[0])
        axis = ellipse[1]
        a = max(axis)/2
        b = min(axis)/2

        self.ecc = np.sqrt(np.square(a)-np.square(b))/a
        img = cv2.ellipse(np.ones(self.empty.shape), ellipse, (255,255,255),-1)
        img_binary = cv2.threshold(img, 50, 1, cv2.THRESH_BINARY)[1]
        return img_binary

  
    def show(self,m):
        plt.imshow(m)
        plt.show()