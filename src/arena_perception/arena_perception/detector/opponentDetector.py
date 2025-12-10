import numpy as np
import cv2

import heapq

class MedianEstimator:
    def __init__(self):
        self.low = []  # Max-heap (invert values)
        self.high = [] # Min-heap

    def add(self, num):
        heapq.heappush(self.low, -num)
        heapq.heappush(self.high, -heapq.heappop(self.low))

        if len(self.low) < len(self.high):
            heapq.heappush(self.low, -heapq.heappop(self.high))

    def get_median(self):
        if len(self.low) == len(self.high):
            return (-self.low[0] + self.high[0]) / 2
        return -self.low[0]


class DiffDetector:
    def __init__(self):
        self.static_back = None
        self.img = None

    def track(self, image_gray, ignore_polygon, debug_image=None):
        # Our operations on the frame come here
        image_gray = cv2.GaussianBlur(image_gray, (11, 11), 25)
        if self.static_back is None:
            self.static_back = image_gray
            # return

        # cv2.imshow('static_back', self.static_back)

        # Difference between static background
        # and current frame(which is GaussianBlur)
        diff_frame = cv2.absdiff(self.static_back, image_gray)
        # cv2.imshow('diff_frame', diff_frame)

        # If change in between static background and
        # current frame is greater than 30 it will show white color(255)
        thresh_frame = cv2.threshold(diff_frame, 30, 255, cv2.THRESH_BINARY)[1]
        # thresh_frame = cv2.dilate(thresh_frame, None, iterations=2)
        if debug_image is not None:
            cv2.imshow('DiffDetector: thresh_frame', thresh_frame)

        if ignore_polygon is not None:
            # Mask shapes from thresh frame
            cv2.fillPoly(thresh_frame, [np.array(ignore_polygon).astype(np.int32)], 0)
            if debug_image is not None:
                cv2.imshow('DiffDetector: thresh_frame+ignore', thresh_frame)

        # Finding contour of moving object
        cnts, _ = cv2.findContours(thresh_frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # print(f"Number of contours: {len(cnts)}")
        bounding_rect = []
        for contour in cnts:
            if cv2.contourArea(contour) < 10 * 10:
                continue

            rect = np.array(cv2.boundingRect(contour))
            (x, y, w, h) = rect
            # print(f"Bounding box: x,y:{x}, {y}; w,h:{w}, {h}")
            # Draw bounding box rectangle in red (not rotated)
            if debug_image is not None:
                cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 0, 255), 1)

            # Draw min area rectangle in blue
            # rect = cv2.minAreaRect(contour)
            # cv2.drawContours(frame, [cv2.boxPoints(rect).astype(int)], -1, (255, 0, 0), 3)

            ###
            bounding_rect.append(rect)
            i, j = (0, 1)
            while i < len(bounding_rect):
                while j < len(bounding_rect):
                    inter = self.intersect(bounding_rect[i], bounding_rect[j])
                    if inter is not None:
                        bounding_rect[i] = inter
                        bounding_rect.pop(j)
                        break
                    j += 1
                if j >= len(bounding_rect):
                    i += 1
                    j = i + 1

        if debug_image is not None:
            # Draw individual contours in yellow
            for individual_contour in bounding_rect:
                (x, y, w, h) = individual_contour
                cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 255, 255), 1)

            # # Draw all contours in green
            # cv2.drawContours(frame, cnts, -1, (0, 255, 0), 3)
            cv2.imshow("DiffDetector Debug Image", debug_image)

        self.static_back = image_gray

        return bounding_rect

    # define a function that checks if two rectangles defined by (x,y,w,h) intersect
    def intersect(self, r1, r2, expand=10):
        def expand_rect(r, e):
            return np.array([r[0] - e, r[1] - e, r[2] + e, r[3] + e])
        r1 = expand_rect(r1, expand)
        r2 = expand_rect(r2, expand)
        x1, y1, w1, h1 = r1
        x2, y2, w2, h2 = r2
        if x1 < x2 + w2 and x1 + w1 > x2 and y1 < y2 + h2 and y1 + h1 > y2:
            return np.array([min(x1,x2), min(y1,y2), max(-x1 + x2 + w2, x1 + w1 - x2), max(-y1 + y2 + h2, y1 + h1 - y2)])
        return None

    def area(self, r):
        return r[2] * r[3]

arrow_forward_x = 0.2 * np.array([[0, 0, 0],
                [1, 0, 0],
                [0.75, 0.25, 0],
                [1, 0, 0],
                [0.75, -0.25, 0],
                [1, 0, 0]])

def draw_outline(outline, image, text=None, color=(0, 255, 0)):
    for i in range(len(outline)):
        pt1 = tuple(outline[i].astype(int))
        pt2 = tuple(outline[(i + 1) % len(outline)].astype(int))
        cv2.line(image, pt1, pt2, color, 2)
        cv2.putText(image, f"{i}", pt1,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    if text:
        cv2.putText(image, text, outline[0].astype(int),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

class OpponentDetector():
    def __init__(self, options: dict):
        self.diff_detector = DiffDetector()
        self.detection_last = None

    def detect(self, image_gray, robot_detection, debug_image=None):
        # bounding_rects = self.diff_detector.track(image_gray.copy(),
        #                                                 robot_detection['robotOutlineInCamera2D'] if robot_detection is not None else None,
        #                                                 debug_image)
        bounding_rects = self.diff_detector.track(image_gray.copy(),
                                                        robot_detection['robotOutlineInCamera2D'] if robot_detection is not None else None)
        def get_contour_largest_area(contours):
            if len(contours) == 0:
                return None
            largest_area = 0.0
            largest_ind = 0
            # print(f'Contours: {contours}')
            for i in range(len(contours)):
                # print(f'Contours[i]: {contours[i]}')
                (x, y, w, h) = contours[i]
                area = w * h
                if area > largest_area:
                    largest_ind = i
                    largest_area = area
            return np.array(contours[largest_ind])

        opponent_contour = get_contour_largest_area(bounding_rects)
        if opponent_contour is None:
            if debug_image is not None:
                if self.detection_last is not None:
                    cv2.circle(debug_image, self.detection_last['opponentInCamera2D'].astype(int), 5, (0, 0, 255), 2)
                cv2.imshow("Opponent pos", debug_image)
            return self.detection_last

        detection = {}
        detection['opponentBBoxInCamera2D'] = opponent_contour
        # print(f'opponentContour {opponent_contour}')
        (x, y, w, h) = opponent_contour
        detection['opponentInCamera2D'] = np.array([x + w / 2, y + h / 2])

        if debug_image is not None:
            print(f'opponentInCamera2D: {detection["opponentInCamera2D"].astype(int)}')
            cv2.circle(debug_image, detection['opponentInCamera2D'].astype(int), 5, (0, 255, 0), 2)
            cv2.imshow("Opponent pos", debug_image)

        self.detection_last = detection
        return detection