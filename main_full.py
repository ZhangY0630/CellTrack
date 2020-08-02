import cv2
import matplotlib.pyplot as plt
from matplotlib.animation import Animation
from matplotlib.animation import FuncAnimation
import copy
import sys
import os
from cell import Cell
from graph import Graph
import math
import numpy as np
from tracker import Tracker ,Track
from detector import Detector
import argparse


def config_parse():
    parser = argparse.ArgumentParser()
    parser.add_argument('--plot',default=False,action='store_true')
    parser.add_argument('--traj' ,default=False,action='store_true')
    parser.add_argument('--num',type=int,default=5)
    args = parser.parse_args()
    return args




def update(i,imgs,axs,Clist,tracker):
    axs.cla()
    label = f"Timestep {i}"
    axs.set_xlabel(label)
    img = imgs[i]
    axs.imshow(img)
    Cells = Clist[i][0]

    for c in Cells:
        plt.plot(c[0], c[1], marker="o")
        plt.draw()

    t = tracker[i]
    for track in t:
        if i ==0:
            lastx = track.trace[-1][0]
            lasty = track.trace[-1][1]
            id = track.trackID
            axs.text(lastx, lasty, str(id))
            plt.draw()
            continue
        if track.mitosisFrame !=-1:
            if i>=track.end:
                continue
        if(len(track.trace)>1):
            # for j in range(len(tracker.tracks[i].trace)-1):

            x1 = track.trace[-2][0]
            y1 = track.trace[-2][1]

            x2 = track.trace[-1][0]
            y2 = track.trace[-1][1]

            if x1==x2 and y1==y2:
                continue

            lastx = track.trace[-1][0]
            lasty = track.trace[-1][1]
            id = track.trackID
            axs.text(lastx, lasty, str(id))
            plt.draw()





    return axs

def MitosisRecovery(tracker,plt_trace):
    maxD = 50

    for track in tracker.tracks:

        if track.recovery:
            continue

        distlist = []
        location = []
        #any frame which is not at the beginning
        if track.frame[0]==0:
            continue
        startFrame = track.frame[0]
        previousFrame = startFrame - 1
        startloc = np.array(track.trace[0])
        for othertrack in tracker.tracks:
            if track !=othertrack:
                loc_i = othertrack.findFrame(previousFrame)

                if loc_i !=-1:
                    loc = np.array(othertrack.trace[loc_i])
                    dist = np.linalg.norm(loc - startloc)
                    if 10<dist<=maxD:
                        distlist.append(dist)
                        location.append(othertrack)
        if len(distlist) ==0:
            continue
        min_inx = np.argmin(distlist)
        ori = location[min_inx]
        #approximation

        for stage in plt_trace:
            for trace in stage:
                if trace.trackID == ori.trackID:
                    if max(trace.frame) >= startFrame:

                        trace.mitosisFrame = previousFrame-2
                        trace.end = startFrame


        track1 = Track(ori.trace[startFrame],tracker.trackID)
        tracker.trackID +=1
        track1.trace = ori.trace[startFrame:]
        track1.frame = ori.frame[startFrame:]
        track1.recovery = True
        tracker.tracks.append(track1)
        f = track1.frame[0]
        count = 0
        for i in range(len(plt_trace)):
            if i == f:
                track2= copy.deepcopy(track1)
                track2.trace = track1.trace[:i]
                track2.frame = track1.frame[:i]
                plt_trace[i].append(track2)
                count+=1
            if count == len(track1.frame):
                break
            f=track1.frame[count]


        print(f"{track.trackID}->{ori.trackID}")


        return plt_trace




def main():
    fig = plt.figure()
    args = config_parse()
    imgs = []
    if not (args.traj or args.plot):
        print("Please choose plot method")
        return

    if args.traj:
        ax = plt.axes(projection='3d')
    else:
        fig,axs = plt.subplots()
    img = plt.imread("mask_DIC/mask000.tif")

    size = img.shape
    width = size[0]
    height  = size[1]
    #we cant delete
    tracker = Tracker(30,100,0)
    detector = Detector()


    l = []
    for root, dirs, files in os.walk("mask_DIC"):
        for file in files:
            l.append(file)

    l =sorted(l)
    
    #original file
    ori = []
    for root, dirs, files in os.walk("Sequence 1"):
        for file in files:
            ori.append(file)
    ori = sorted(ori)

    first=-1
    second=0
    #center list
    Clist = []
    plt_trace = []

    for name in l:
        
        imgN = os.path.join("mask_DIC/" + name)
        print(imgN)
        img= plt.imread(imgN)
    

       

        if second ==7:
            print("it's 4")


        graph = Graph(img,height,width)
        
        centers = detector.Detect(graph)
        Clist.append(centers)
        tracker.Update(centers,second)

        if args.traj:
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
        if args.plot:
            oriN = os.path.join("Sequence 1/" + ori[second])
            originImg = plt.imread(oriN)
            imgs.append(originImg)
            elements = []
            plt_trace.append(copy.deepcopy(tracker.tracks))


                    
      
        first +=1
        second +=1

        if second == args.num:
            break

    plt_trace = MitosisRecovery(tracker,plt_trace)

    if args.plot:
        ani = FuncAnimation(fig,update,fargs=(imgs,axs,Clist,plt_trace),interval=500,frames=second)
    plt.show()
    done = False
    while done:
        choice = input("> ")
        qlist= ['speed','total','net']
        if choice in qlist:
            id = int(input("ID: "))
            frame = int(input("which frame u are: "))
            if frame <1:
                print("the frame has to be great than 0")
                continue

            t = tracker.findTrack(id)
            if t == None:
                print("Dont have this cell")
                continue
            if len(t.frame)==1:
                print("Sorry, this cell only appear once")


            absolute_val_array = np.abs(np.array(t.frame) - frame)
            smallest_difference_inx = absolute_val_array.argmin()
            closet_frame = t.frame[smallest_difference_inx]
            if closet_frame != frame:
                print(f"Sorry we can't find {id} in frame {frame}, the closet frame is {closet_frame}")
            if choice == "speed":
                pre_frame = t.frame[closet_frame-1]
                pre_loc = np.array(t.trace[smallest_difference_inx-1])
                cur_loc = np.array( t.trace[smallest_difference_inx])
                dist = np.linalg.norm(cur_loc-pre_loc)
                speed = dist/(frame-pre_frame)
                print(f"The {id} at frame {frame} has a speed {speed} pixel/frame")
            if choice == "total":
                t.printTrace(smallest_difference_inx)
                dist = t.totalDistance(smallest_difference_inx)
                print(f"It has travelled {dist} in total")
            if choice == "net":
                loc = np.array( t.trace[smallest_difference_inx] )
                start = np.array(t.trace[0])

                dist = np.linalg.norm(loc-start)
                print(f"The net distance is {dist}")




        if choice == "q":
            done=False


                

   




if __name__ == "__main__":
    main()