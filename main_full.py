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

    tracker = Tracker(50,3,0)
    detector = Detector()


    l = []
    for root, dirs, files in os.walk("mask_DIC"):
        for file in files:
            l.append(file)

    l =sorted(l)

    first=-1
    second=0

    for name in l:
        
        imgN = os.path.join("mask_DIC/" + name)
        print(imgN)
        img= plt.imread(imgN)

       
        if second == 4:
            break    
             

        graph = Graph(img,height,width)
        
        centers = detector.Detect(graph)

        tracker.Update(centers)

   
        for i in range(len(tracker.tracks)):
            if(len(tracker.tracks[i].trace)>1):
                # for j in range(len(tracker.tracks[i].trace)-1):
                    
                x1 = tracker.tracks[i].trace[-2][0]
                y1 = tracker.tracks[i].trace[-2][1]
                x2 = tracker.tracks[i].trace[-1][0]
                y2 = tracker.tracks[i].trace[-1][1]

                if x1==x2 and y1==y2:
                    continue
                
                ax.plot( [x1,x2],[y1,y2],[first,second] )
                plt.draw()
                    
                # x1 = tracker.tracks[i].trace[-2][0]
                # y1 = tracker.tracks[i].trace[-2][1]
                # x2 = tracker.tracks[i].trace[-1][0]
                # y2 = tracker.tracks[i].trace[-1][1] 
                
      
        first +=1
        second +=1
    plt.show()

                

   





if __name__ == "__main__":
    main()