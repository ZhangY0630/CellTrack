import numpy as np
from kalman_filter import KalmanFilter
from scipy.optimize import linear_sum_assignment

class Track(object):
    
    def __init__(self,prediction,trackID):
        self.trackID = trackID
        self.KF = KalmanFilter()
        self.prediction = np.array(prediction)
        self.skipped_frame = 0
        self.trace = []

class Tracker(object):
    def __init__(self, thresh,max_frames_to_skip, id):
        super().__init__()
        self.max_frames_to_skip = max_frames_to_skip
        self.thresh = thresh
        self.trackID = id
        self.tracks = []

    def Update(self,observation):
        detections = np.array(observation[0])
        if (len(self.tracks)==0):
            for detection in detections:
                track = Track(detection,self.trackID)
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

        # if trackers are not detected for a long time, remove it
        del_tracks = []
        for track in self.tracks:
            if (track.skipped_frame > self.max_frames_to_skip):
                del_tracks.append(track)

        if len(del_tracks) > 0:
            for track in del_tracks:
                if track.trackID < len(self.tracks):
                    self.tracks.remove(track)
                    if(track.trackID in assignment):
                        assignment.remove(track.trackID)
                else:
                    print("ID is greater than the lenth")
        # lets check for the detections that dont belong to any track
        un_assigned_detection = []
        for i in range(len(detections)):
            if i not in assignment:
                un_assigned_detection.append(i)

        #for this detection start new tracks
        if (len(un_assigned_detection)!=0):
            for i in un_assigned_detection:
                track = Track(i,self.trackID)
                self.trackID +=1
                self.tracks.append(track)

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
                self.tracks[i].trace.append(detections[assignment[i]])
            self.tracks[i].KF.lastResult = self.tracks[i].prediction


        


