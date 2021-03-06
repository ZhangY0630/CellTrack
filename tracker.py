import numpy as np
from kalman_filter import KalmanFilter
from scipy.optimize import linear_sum_assignment

class Track(object):
    
    def __init__(self,prediction,trackID,cellID):
        self.trackID = trackID
        self.KF = KalmanFilter()
        self.prediction = np.array(prediction)
        self.skipped_frame = 0
        self.trace = []
        self.frame = []
        self.mitosisFrame = -1
        self.end = -1
        self.parent = None
        self.recovery = False
        self.cellID = cellID
        
    def printTrace(self, num):
        print("It has travelled:",end=' ')
        for i in range(num+1):
            loc = self.trace[i]
            x = loc[0]
            y = loc[1]
            print(f"({x},{y})", end="")
            if i == num:
                break
            print("->",end="")
        print("")

    def findFrame(self,num):
        for i in range(len(self.frame)):
            if self.frame[i] == num:
                return i
        return -1




    def totalDistance(self,num):
        sum = 0
        pre = np.array(self.trace[0])
        for i in range(num+1):
            loc = np.array(self.trace[i])
            sum += np.linalg.norm(loc-pre)
            pre = loc
        return sum

    def addFrame(self,i):
        self.frame.append(i)

class Tracker(object):
    def __init__(self, thresh,max_frames_to_skip, id):
        super().__init__()
        self.max_frames_to_skip = max_frames_to_skip
        self.thresh = thresh
        self.trackID = id
        self.tracks = []

    def findTrack(self,id):
        for track in self.tracks:
            if track.trackID==id:
                return track
        return None

    def Update(self,observation,frame):
        detections = np.array(observation[0])
        ids = np.array(observation[1])
        if (len(self.tracks)==0):
            for i in range(len( detections)):
                detection = detections[i]
                id = ids[i]
                track = Track(detection,self.trackID,id)
                track.addFrame(frame)
                self.trackID += 1
                self.tracks.append(track)

    #prediction vs detected controids
        N = len(self.tracks)
        M = len(detections)
        cost = np.zeros((N,M))
        for i in range(N):
            for j in range(M):
                #got problem
                try:
                    if (self.tracks[i].prediction.shape ==(2,)):
                        a = self.tracks[i].prediction
                    else:
                        a = self.tracks[i].prediction[0]
                    b=  detections[j]
                    diff = a-b
                    distance = np.sqrt(diff[0]*diff[0] +diff[1]*diff[1])
                    cost[i][j] =distance
                except:
                    pass

        #average the cost
        cost = (0.5)*cost
        assignment = []
        #assign the correct detected measument to the predict tracks
        for _ in range(N):
            assignment.append(-1)
        row_ind,col_ind = linear_sum_assignment(cost)
        for i in range(len(row_ind)):
            assignment[row_ind[i]] = col_ind[i]

        #identify tracks with no assignment, if any
        un_assigned_tracks = []
        for i in range(len(assignment)):
            if(assignment[i] != -1):
                if (cost[i][assignment[i]] > self.thresh):
                    assignment[i] = -1
                    un_assigned_tracks.append(i)

            else:
                self.tracks[i].skipped_frame +=1

        #update kalman filter

        for i in range(len(assignment)):
            self.tracks[i].KF.predict()

            if (assignment[i] != -1):

                #clear the cumulative skip frame
                self.tracks[i].skipped_frame = 0
                self.tracks[i].prediction = self.tracks[i].KF.correct(detections[assignment[i]],True)
            else:
                self.tracks[i].prediction = self.tracks[i].KF.correct(np.array([[0], [0]]),False)


            # self.tracks[i].trace.append(self.tracks[i].prediction)
            if assignment[i] == -1:
                self.tracks[i].trace.append(self.tracks[i].trace[-1])
            else:
                if frame!=0:
                    self.tracks[i].addFrame(frame)
                self.tracks[i].trace.append(detections[assignment[i]])
            self.tracks[i].KF.lastResult = self.tracks[i].prediction



        # if trackers are not detected for a long time, remove it
        del_tracks = []
        for track in self.tracks:
            if (track.skipped_frame > self.max_frames_to_skip):
                del_tracks.append(track)
# because here delete the list , whole structure move forward
        if len(del_tracks) > 0:
            for track in del_tracks:
                # if track.trackID < len(self.tracks):
                self.tracks.remove(track)
                if(track.trackID in assignment):
                        assignment.remove(track.trackID)
              
        # lets check for the detections that dont belong to any track
        un_assigned_detection = []
        for i in range(len(detections)):
            if i not in assignment:
                un_assigned_detection.append(i)

        #for this detection start new tracks
        if (len(un_assigned_detection)!=0):
            for i in un_assigned_detection:
                track = Track(detections[i],self.trackID,ids[i])
                track.addFrame(frame)
                track.trace.append(detections[i])
                self.trackID +=1
                self.tracks.append(track)




        


