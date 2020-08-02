import cv2
import matplotlib.pyplot as plt
from matplotlib.animation import Animation
import sys
import os
from cell import Cell
from graph import Graph
import math
import numpy as np
from tracker import Tracker
from detector import Detector



def main():
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    img = plt.imread("mask_DIC/mask000.tif")

    size = img.shape
    width = size[0]
    height  = size[1]

    tracker = Tracker(30,10,0)
    detector = Detector()


    l = []
    for root, dirs, files in os.walk("mask_DIC"):
        for file in files:
            l.append(file)

    l =sorted(l)

    first=-1
    second=0

    lx1 = []
    ly1 = []
    lx2 = []
    ly2 = []

    for name in l:
        
        imgN = os.path.join("mask_DIC/" + name)
        print(imgN)
        img= plt.imread(imgN)

       
        if second == 32:
            break    
        if second ==7:
            print("it's 4")


        graph = Graph(img,height,width)
        
        centers = detector.Detect(graph)

        tracker.Update(centers)


        for i in range(len(tracker.tracks)):
            if(len(tracker.tracks[i].trace)>1):
                # for j in range(len(tracker.tracks[i].trace)-1):
                    
                x1 = tracker.tracks[i].trace[-2][0]
                lx1.append(x1)
                y1 = tracker.tracks[i].trace[-2][1]
                ly1.append(y1)
                x2 = tracker.tracks[i].trace[-1][0]
                lx2.append(x2)
                y2 = tracker.tracks[i].trace[-1][1]

                if x1==x2 and y1==y2:
                    continue
                
                ax.plot( [x1,x2],[y1,y2],[first,second] )
                plt.draw()
                
                if Gap(tracker.tracks[i].trace):
                    print("find gap")
                    
      
        first +=1
        second +=1
    plt.show()

                

   
def Gap(trace):
    if len(trace)<4:
        return False
    x3 = trace[-2][0]
    y3 = trace[-2][1]
    x4 = trace[-3][0]
    x4 = trace[-3][1]
    
    if(x3==x4 and y3==y4):
        return True
        



if __name__ == "__main__":
    main()