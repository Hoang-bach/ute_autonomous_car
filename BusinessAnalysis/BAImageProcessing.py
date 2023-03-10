import cv2
import numpy as np
import time
import matplotlib.pyplot as plt


class imageProcessing:
    def __init__(self, mask):
        self.mask = mask

    def gaussianBlur(self):
        blur = cv2.GaussianBlur(self.mask, (5, 5), 0)
        return blur

    def canny(self):
        return cv2.Canny(self.gaussianBlur(), 40, 100)

    def ROI(self):
        height = self.canny().shape[0]
        width = self.canny().shape[1]
        polygon = np.array([
            [(0, height), (20, 40), (0, 40), (0, 20), (width, 20), (width, 40), (130, 40), (width, height)]
        ])
        imgMask = np.zeros_like(self.canny())
        cv2.fillPoly(imgMask, polygon, 255)
        cropped_image = cv2.bitwise_and(self.canny(), imgMask)
        return cropped_image

    def ROITurnRight(self):
        height = self.canny().shape[0]
        width = self.canny().shape[1]
        polygon = np.array([
            [(0, 0), (0, 50), (90, 0)]
        ])
        polygon2 = np.array([
            [(0, 0), (0, 9), (width, 9), (width, 0)]
        ])
        cv2.fillPoly(self.mask, polygon, 0)
        cv2.fillPoly(self.mask, polygon2, 0)
        return self.mask

    def ROITurnLeft(self):
        height = self.canny().shape[0]
        width = self.canny().shape[1]
        polygon = np.array([
            [(70, 0), (width, 0), (width, 50)]
        ])
        polygon2 = np.array([
            [(0, 0), (0, 9), (width, 9), (width, 0)]
        ])
        cv2.fillPoly(self.mask, polygon, 0)
        cv2.fillPoly(self.mask, polygon2, 0)
        return self.mask

    def ROIStraight(self):
        height = self.canny().shape[0]
        width = self.canny().shape[1]
        polygon = np.array([
            [(70, 0), (width, 0), (width, 50)]
        ])
        polygon2 = np.array([
            [(0, 0), (0, 50), (90, 0)]
        ])
        cv2.fillPoly(self.mask, polygon, 0)
        cv2.fillPoly(self.mask, polygon2, 0)
        return self.mask

    def removeSmallContours(self):
        image_binary = np.zeros((self.mask.shape[0], self.mask.shape[1]), np.uint8)
        contours = cv2.findContours(self.mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]
        masked = cv2.drawContours(image_binary, [max(contours, key=cv2.contourArea)], -1, (255, 255, 255), -1)
        image_remove = cv2.bitwise_and(self.mask, self.mask, mask=masked)
        return image_remove

    def ROINoStraight(self):
        height = self.canny().shape[0]
        width = self.canny().shape[1]
        polygon = np.array([
            [(0, 0), (0, 10), (width, 10), (width, 0)]
        ])
        cv2.fillPoly(self.mask, polygon, 0)
        return self.mask

    # def ROINoStraightNoRight(self):
    #     height = self.canny().shape[0]
    #     width = self.canny().shape[1]
    #     polygon = np.array([
    #         [(0, 0), (0, 12), (width, 12), (width, 0)]
    #     ])
    #     polygon2 = np.array([
    #         [(70, 0), (width, 0), (width, 50)]
    #     ])
    #     cv2.fillPoly(self.mask, polygon, 0)
    #     cv2.fillPoly(self.mask, polygon2, 0)
    #     return self.mask

    # def ROINoStraightNoLeft(self):
    #     height = self.canny().shape[0]
    #     width = self.canny().shape[1]
    #     polygon = np.array([
    #         [(0, 0), (0, 12), (width, 12), (width, 0)]
    #     ])
    #     polygon2 = np.array([
    #         [(0, 0), (0, 50), (90, 0)]
    #     ])
    #     cv2.fillPoly(self.mask, polygon, 0)
    #     cv2.fillPoly(self.mask, polygon2, 0)
    #     return self.mask
        

    def ROINoRight(self):
        height = self.canny().shape[0]
        width = self.canny().shape[1]
        polygon = np.array([
            [(70, 0), (width, 0), (width, 50)]
        ])
        cv2.fillPoly(self.mask, polygon, 0)
        return self.mask

    def ROINoLeft(self):
        height = self.canny().shape[0]
        width = self.canny().shape[1]
        polygon = np.array([
            [(0, 0), (0, 50), (90, 0)]
        ])
        cv2.fillPoly(self.mask, polygon, 0)
        return self.mask
