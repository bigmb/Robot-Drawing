#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Nov  3 16:48:53 2018

@author: bigmb
"""
import numpy as np

from collections import deque
import cv2
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import PoseStamped

image = cv2.imread("gitphotos/1.jpg", 1)
image = cv2.resize(image,None,fx=0.25,fy=0.25)

lower = [1, 0, 20]
upper = [60, 40, 200]

lower = np.array(lower, dtype="uint8")
upper = np.array(upper, dtype="uint8")

mask = cv2.inRange(image, lower, upper)
output = cv2.bitwise_and(image, image, mask=mask)

ret,thresh = cv2.threshold(mask, 40, 255, 0)
im2,contours,hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

if len(contours) != 0:
    cv2.drawContours(output, contours, -1, 255, 3)

    c = max(contours, key = cv2.contourArea)


plt.imshow(output, cmap = 'gray', interpolation = 'bicubic')
plt.xticks([]), plt.yticks([])  # to hide tick values on X and Y axis
#plt.show()
#cv2.imwrite( "./Gray_Image.jpg", output );

img1 = output.copy()
img1 = img1[:,:,0]
plt.imshow(img1, cmap = 'gray', interpolation = 'bicubic')
plt.xticks([]), plt.yticks([])  # to hide tick values on X and Y axis
#plt.show()

edges = cv2.Canny(img1,100,200)
plt.imshow(edges, cmap = 'gray', interpolation = 'bicubic')
plt.xticks([]), plt.yticks([])  # to hide tick values on X and Y axis
#plt.show()


a = np.mat(edges)
count = 0
i=0
j=0
for i in range(224):
    for j in range(224):
        if a[i,j]==255:
            count = count + 1
#
#m=2
#n=2
##    
#b = np.mat(a)
#ib=0
#jb=0
#for ib in range(224):
#    for jb in range(224):
#        b[ib,jb]=0


            
#def draw(a,b,m,n):
#    #for m in range(223):
#     #   for n in range(223):
#            if a(m-1,n-1)==255 or a(m,n-1)==255 or a(m-1,n)==255 or a(m+1,n-1)==255 or a(m-1,n+1)==255 or a(m,n+1)==255 or a(m+1,n)==255 or a(m+1,n+1)==255:
#                gotoP(m,n)
#            else:
#                n = n+1
#                
#         #reach(a,m,n,size(I));
##        #servoWrite(a,7,92);
##        #pause(0.01);
#    else:
#        
##   r=size(I);
##   I(m,n)=0;
##  if m-1>0&&n-1>0&&m<r(1,1)&&n<r(1,2)
##      for i=m-1:m+1
##          for j=n-1:n+1
##              if I(i,j)==1
##                 I=draw(a,I,i,j); 
##              end
##          end
##      end
##   end
##end
##    
#
#def gotoP(m,n):
#    reach(m,n)
#    #pendrop
#    start(m,n)
#    #if a(x-1,y-1)==255 or a(x,y-1)==255 or a(x-1,y)==255 or a(x+1,y-1)==255 or a(x-1,y+1)==255 or a(x,y+1)==255 or a(x+1,y)==255 or a(x+1,y+1)==255:
#    if a(m-1,n-1)==255 or a(m,n-1)==255 or a(m-1,n)==255 or a(m+1,n-1)==255 or a(m-1,n+1)==255 or a(m,n+1)==255 or a(m+1,n)==255 or a(m+1,n+1)==255:
#        for m in range(224):
#        start(m,n)
#         
#    else:
#        #pentakeoff
#        
#def draw2(m,n):
#     if a(m-1,n-1)==255:
#         #pendrop
#         draw2(m-1,n-1)
#    else if a(m,n-1)==255:
#        #pendrop
#        draw2(m,n-1)
           
#a = a[:,:,0]
ib=0
jb=0
for ib in range(224):
    for jb in range(224):
        if a[ib,jb]==255:
            a[ib,jb]=1
  
graph = a    
len_x = graph.shape[0]
len_y = graph.shape[1] 
a = a.tolist()
#visited = np.empty([269,269])

visited = []

def get_siblings(x, y):
    out = [[x+1, y], [x+1, y], [x-1, y], [x, y-1],
            [x+1, y+1], [x+1, y-1], [x-1, y-1], [x-1, y+1]]


    def validity(x, y):
        if x < 0 or x >= len_x:
            return False

        if y < 0 or y >= len_y:
            return False

        return True

    out = [c for c in out if validity(*c)]

    return [co_ords for co_ords in out if co_ords and a[co_ords[0]][co_ords[1]] == 1]


stack = deque()


def BFS(node):
    if node in visited:
        return

    stack.append(node)
    lands = []

    while len(stack):
        cnode = stack.pop()

        if cnode in visited:
            continue

        lands.append(cnode)
        siblings = get_siblings(*cnode)
        siblings = [s for s in siblings if s not in visited]

        visited.append(cnode)
        stack.extend(siblings)

    return lands


def main():
    islands_ = [BFS([xx, yy]) for xx in range(len_x) for yy in range(len_y) if a[xx][yy] == 1]
    return [i for i in islands_ if i]


islands = main()
#
#f = open("myfile", "w")
#f.write("\n".join(map(lambda x: str(x), islands)))
#f.close()
#
#with open("myfile", "r") as ins:
#    array = []
#    for line in ins:
#        array.append(line)
        
#array2 = np.asarray(array)
array3 = np.asarray(islands)     

#print ('Number of points are = ', len(islands))
#for i, v in enumerate(islands): print ("Points {}: {}".format(i+1, v))
        
rospy.init_node('image_node')
pose_publish = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=10)
goal = PoseStamped()

arr=0
jarr=0
for arr in range(array3.size):
    for jarr in range(len(array3[arr])):
        goal.pose.position.x = array3[arr][jarr][0]/50.0
        goal.pose.position.y = array3[arr][jarr][1]/50.0
        pose_publish.publish(goal)
        rospy.sleep(1.5)
    rospy.sleep(3)
