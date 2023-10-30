# importing the Libraries
import numpy as np
import cv2
import imutils
# reading image by using path
args_image ="E:/manpic.jpg"
image = cv2.imread(args_image)
#original
image=cv2.resize(image,(250,250))
cv2.imshow("Original Image", image)
# blur
ksize = (10, 10)
blurimage = cv2.blur(image, ksize) 
blurimage = cv2.resize(blurimage, (250, 250))
cv2.imshow('blur_image', blurimage) 
#gray
grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) grayImage = cv2.resize(grayImage, (250, 250))
cv2.imshow("gray", grayImage)29
#grayblur
grayImageBlur = cv2.blur(grayImage,(9,9)) grayImageBlur = cv2.resize(grayImageBlur, (250, 250))
cv2.imshow("grayBlur", grayImageBlur)
#Motion Blur
kernel = np.ones((3,3), np.uint8)
size = 15
kernel_motion_blur = np.zeros((size, size))
kernel_motion_blur[int((size-1)/2), :] = np.ones(size)
kernel_motion_blur = kernel_motion_blur / size
# applying the kernel to the input image
output = cv2.filter2D(image, -2, kernel_motion_blur)
output = cv2.resize(output, (250, 250))
cv2.imshow('Motion Blur', output)
#Cartoon
# Edges
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
gray = cv2.medianBlur (gray, 5)
edges = cv2.adaptiveThreshold (gray, 
255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 9, 9) # Cartoonization
color=cv2.bilateralFilter(image, 9, 250, 250) cartoon = cv2.bitwise_and(color, color, mask=edges)
#blur
edges = cv2.resize(edges, (250, 250)) cartoon = cv2.resize(cartoon, (250, 250))
cv2.imshow("Cartoon", cartoon)
cv2.imshow("edges", edges)
#edges
edgedImage = cv2.Canny(grayImage, 100, 300, 3) edgedImage = cv2.resize(edgedImage, (250, 250))
cv2.imshow("Edge Detected Image", edgedImage)
#dilation
image_dilation = cv2.dilate(image, kernel, iterations=1) image_dilation = cv2.resize(image_dilation, (250, 250)) cv2.imshow('Dilation', image_dilation)30
#erosion
image_erosion = cv2.erode(image, kernel, iterations=1) image_erosion = cv2.resize(image_erosion, (250, 250)) cv2.imshow('Erosion', image_erosion)
# negative image
img_neg = cv2.bitwise_not(image) cv2.imshow('negative',img_neg)
cv2.waitKey(0)
cv2.destroyAllWindows()