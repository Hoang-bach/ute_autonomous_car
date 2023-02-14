import time
import numpy as np
import cv2
import torch
from model.UNET import build_unet
from model.CNN import Network
from sklearn.ensemble import RandomForestRegressor
from weights.loadWeights import weights
from BusinessAnalysis.BASegmentations import segmentation
from BusinessAnalysis.BADetections import detection
from BusinessAnalysis.BAController import Controller
from BusinessAnalysis.BAImageProcessing import imageProcessing
from Datasets.Dataloader import Map

# from sklearn.ensemble import RandomForestRegressor

global M


def main():
    global M
    currentAngle = 0
    currentSpeed = 0
    sendBackSpeed = 0
    sendBackAngle = 0
    MAX_SPEED = 50
    delaySign = None
    flag = None
    t0 = 0
    pretrainedModel = weights()
    predictedUNET = pretrainedModel.modelUNET()
    predictedYOLOv5m = pretrainedModel.modelYOLOv5m()
    predictedCNN = pretrainedModel.modelCNN()
    start = time.time()

    # x = np.array([-12, -1 ,-0.25, 0,
    #                 0.25, 1, 12])
    # y = np.array([0, 10, 25, 55,  
    #                 25, 10 , 0])

    # x = x.reshape(-1,1)
    M = Map(currentAngle, currentSpeed, sendBackAngle, sendBackSpeed)
    try:
        while True:
            image, currentAngle, currentSpeed, sendBackAngle, sendBackSpeed = M.Connect()
            '''-------------------------Work Space----------------------------'''
            # cv2.imshow("Lmao", image)
            modelUNET = segmentation(image)
            pretrainedUNET = modelUNET.predict(predictedUNET)
            IP = imageProcessing(pretrainedUNET)
            pretrainedUNET = IP.removeSmallContours()
            cv2.imshow("", predictedUNET)
            modelYOLOv5m = detection(image)
            sign = modelYOLOv5m.predict(predictedYOLOv5m, predictedCNN)
            print('Sign: ', sign)
            preTime = time.time()
            '''-------------------------Controller----------------------------'''
            

            # balance = Controller(pretrainedUNET, start, sendBackSpeed, sign, preTime)

            if sign:
                
                ####
                if sign == 'straight':
                    pretrainedUNET = IP.ROIStraight()
                elif sign == 'turnright':
                    pretrainedUNET = IP.ROITurnRight()
                elif sign == 'turnleft':
                    pretrainedUNET = IP.ROITurnLeft()
                elif sign == 'nostraight':
                    # pretrainedUNET = IP.ROINoStraight()
                    balance = Controller(pretrainedUNET, start, sendBackSpeed, sign, preTime)
                    Min, Max = balance.checkLane()
                    if Min <= 10 and Max <= 150:
                        pretrainedUNET = IP.ROITurnLeft()
                        flag = 'left' 
                    elif Max >= 150 and Min >= 10:
                        pretrainedUNET = IP.ROITurnRight()
                        flag = 'right'
                elif sign == 'noright':
                    pretrainedUNET = IP.ROINoRight()
                elif sign == 'noleft':
                    pretrainedUNET = IP.ROINoLeft()
                # else:
                    # error = balance.obstacleAvoiding()
                ####
                balance = Controller(pretrainedUNET, start, sendBackSpeed, sign, preTime)
                error = balance.computeError()
                sendBackAngle = - balance.PIDController(error) * 20 / 60
                sendBackSpeed = 10
                delaySign = sign
                t0 = time.time()

                
            else:
                delayTime = 3

                if delaySign:
                    if delaySign == 'straight':
                        pretrainedUNET = IP.ROIStraight()
                    elif delaySign == 'turnright':
                        pretrainedUNET = IP.ROITurnRight()
                    elif delaySign == 'turnleft':
                        pretrainedUNET = IP.ROITurnLeft()
                    elif delaySign == 'nostraight':
                        if flag == 'right':
                            pretrainedUNET = IP.ROITurnRight()
                        elif flag == 'left':
                            pretrainedUNET = IP.ROITurnLeft()
                    elif delaySign == 'noright':
                        pretrainedUNET = IP.ROINoRight()
                    elif delaySign == 'noleft':
                        pretrainedUNET = IP.ROINoLeft()
                    # else:
                        # error = balance.obstacleAvoiding()
                    balance = Controller(pretrainedUNET, start, sendBackSpeed, sign, preTime)
                    # Min, Max = balance.checkLane()
                    error = balance.computeError()
                    if time.time() - t0 >= delayTime:
                        delaySign = None
                        flag = None
                    sendBackAngle = - balance.PIDController(error) * 23 / 60
                    sendBackSpeed = 20
                else: 
                    balance = Controller(pretrainedUNET, start, sendBackSpeed, sign, preTime)
                    error = balance.computeError()
                    sendBackAngle = - balance.PIDController(error) * 20 / 60
                    # regressor = RandomForestRegressor(n_estimators = 300, random_state = 0)
                    # regressor.fit(np.array(x), np.array(y))
                    # sendBackSpeed = np.mean(regressor.predict([[sendBackAngle]]))
                    sendBackSpeed = 30
                
                
               

            cv2.imshow("Segmentation", pretrainedUNET)
            print('Delay Sign: ', delaySign)



            
            print('==============================================')
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        M.socketClose()


if __name__ == '__main__':
    main()
