#!/usr/bin/env python
import roslib
roslib.load_manifest('roscv')
import sys
import os
import rospy
import cv2
import numpy as np

class calibrate(object):
    
    def __init__(self, args):
        self.camera = None
        self.distortion = None
        self.r = None
        self.t = None
        self.e = None
        self.f = None
        self.debug_dir = 1
        self.square_size = .025
        self.left_images = self.getImageFromFile(str(args[0]))
        self.right_images = self.getImageFromFile(str(args[1]))
        
    def getImageFromFile(self, filename):
        return [line.strip() for line in open(os.path.join(os.path.dirname(os.path.realpath(__file__)), filename))]
        
    def detect_squares(self, images):
        pattern_size = (7, 6)
        pattern_points = np.zeros( (np.prod(pattern_size), 3), np.float32 )
        pattern_points[:,:2] = np.indices(pattern_size).T.reshape(-1, 2)
        pattern_points *= self.square_size
    
        obj_points = []
        img_points = []
        h, w = 0, 0
        for fn in images:
            print 'processing %s...' % fn,
            img = cv2.imread(os.path.join(os.path.dirname(os.path.realpath(__file__)), "cal", fn), 0)
            h, w = img.shape[:2]
            found, corners = cv2.findChessboardCorners(img, pattern_size)
            if found:
                term = ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1 )
                cv2.cornerSubPix(img, corners, (5, 5), (-1, -1), term)
            if self.debug_dir:
                vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                cv2.drawChessboardCorners(vis, pattern_size, corners, found)
            if not found:
                print 'chessboard not found'
                continue
            img_points.append(corners.reshape(-1, 2))
            obj_points.append(pattern_points)
            
            print 'ok'
        print "Calibration is a go... please wait.."
        camera_matrix = np.zeros((3, 3))
        dist_coefs = np.zeros(4)
        img_n = len(img_points)
        rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w, h), camera_matrix, dist_coefs)
        return (obj_points, img_points, (w, h), camera_matrix, dist_coefs, rvecs, tvecs)
        """print "RMS:", rms
        print "camera matrix:\n", camera_matrix
        print "distortion coefficients: ", dist_coefs.ravel()"""
    
    def calibrate(self):
        left = self.detect_squares(self.left_images)
        right = self.detect_squares(self.right_images)
        print "Sweet camera calibration is done. \n now for the stereo data. please wait..."
        (retval, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2,
          R, T, E, F) = cv2.stereoCalibrate(left[0], left[1], right[1], left[2],
                                            left[3], left[4], right[3], right[4])
        self.retval = retval
        self.size = left[3]
        self.rvects = (left[5], right[5])
        self.tvects = (left[6], right[6])
        self.camera = (left[1], right[1])#(cameraMatrix1, cameraMatrix2)
        self.distortion = (left[2], right[2])#(distCoeffs1, distCoeffs2)
        self.r = R
        self.t = T
        self.e = E
        self.f = F
        self.save()
        print "running stereo rectify in case it is needed later"
        R1, R2, P1, P2, Q, valid1, valid2 = cv2.stereoRectify(self.camera[0], 
                        self.distortion[0], self.camera[1], self.distortion[1],
                        left[3], self.r, self.t)
        
        
    def save(self):
        if self.camera == None:
            raise Exception("Camera has not been calibrated yet")
        #convert all the data to numpy arrays
        camera1 = np.array(self.camera[0])
        camera2 = np.array(self.camera[1])
        distCoeffs1 = np.array(self.distortion[0])
        distCoeffs2 = np.array(self.distortion[1])
        retval = np.array(self.retval)
        R = np.array(self.r)
        T = np.array(self.t)
        E = np.array(self.e)
        F = np.array(self.f)
        
        #write the numpy arrays to files
        np.save(os.path.join(os.path.dirname(os.path.realpath(__file__)), "cal/camera1"), camera1)
        np.save(os.path.join(os.path.dirname(os.path.realpath(__file__)), "cal/camera2"), camera2)
        np.save(os.path.join(os.path.dirname(os.path.realpath(__file__)), "cal/distCoeffs1"), distCoeffs1)
        np.save(os.path.join(os.path.dirname(os.path.realpath(__file__)), "cal/distCoeffs2"), distCoeffs2)
        np.save(os.path.join(os.path.dirname(os.path.realpath(__file__)), "cal/retval"), retval)
        np.save(os.path.join(os.path.dirname(os.path.realpath(__file__)), "cal/r"), R)
        np.save(os.path.join(os.path.dirname(os.path.realpath(__file__)), "cal/t"), T)
        np.save(os.path.join(os.path.dirname(os.path.realpath(__file__)), "cal/e"), E)
        np.save(os.path.join(os.path.dirname(os.path.realpath(__file__)), "cal/f"), F)
        
    def load(self):
        try:
            self.retval = np.fromfile(os.path.join(os.path.dirname(os.path.realpath(__file__)), "cal/retval.npy"))
            self.camera = (np.fromfile(os.path.join(os.path.dirname(os.path.realpath(__file__)), "cal/camera1.npy")),
                           np.fromfile(os.path.join(os.path.dirname(os.path.realpath(__file__)), "cal/camera2.npy")))
            self.distortion = (np.fromfile(os.path.join(os.path.dirname(os.path.realpath(__file__)), "cal/distCoeffs1.npy")),
                           np.fromfile(os.path.join(os.path.dirname(os.path.realpath(__file__)), "cal/distCoeffs2.npy")))
            self.r = np.fromfile(os.path.join(os.path.dirname(os.path.realpath(__file__)), "cal/r.npy"))
            self.t = np.fromfile(os.path.join(os.path.dirname(os.path.realpath(__file__)), "cal/t.npy"))
            self.e = np.fromfile(os.path.join(os.path.dirname(os.path.realpath(__file__)), "cal/e.npy"))
            self.f = np.fromfile(os.path.join(os.path.dirname(os.path.realpath(__file__)), "cal/f.npy"))
            left = cv2.imread(os.path.join(os.path.dirname(os.path.realpath(__file__)), "cal/left-0001.png"))
            right = cv2.imread(os.path.join(os.path.dirname(os.path.realpath(__file__)), "cal/right-0001.png"))
            left = cv2.undistort(left, self.camera[0], self.distortion[0])
            right = cv2.undistort(right, self.camera[1], self.distortion[1])
            cv2.imshow("test", left)
            cv2.imshow("test", right)
            cv2.waitKey(1)
            return True
        except IOError, e:
            print e
            return False
def main(args):
    cal = calibrate(args[1:])
    if not cal.load():
        cal.calibrate()
    print "done"
    
if __name__ == '__main__':
    main(sys.argv)
