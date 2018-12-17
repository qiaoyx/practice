import cv2
import numpy as np
import matplotlib.pyplot as plt

img = cv2.imread("/home/qiaoyx/Desktop/image.png")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray, 50, 200, 3)
# cv2.imshow('image', img)
plt.subplot(121)
plt.imshow(edges, 'gray')
plt.xticks([]), plt.yticks([])

# hough transform


def linesP():
    lines = cv2.HoughLinesP(
        edges, 1, np.pi / 180, 30, minLineLength=60, maxLineGap=10)
    lines1 = lines[:, 0, :]
    for x1, y1, x2, y2 in lines1:
        cv2.line(img, (x1, y1), (x2, y2), (0, 0, 254), 2)


def lines():
    lines = cv2.HoughLines(edges, 1, np.pi / 180, 118)
    lines1 = lines[:, 0, :]
    for rho, theta in lines1:
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 1)


linesP()
# lines()
plt.subplot(122), plt.imshow(img)
plt.xticks([]), plt.yticks([])
plt.show()
