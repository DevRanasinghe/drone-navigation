import cv2
import matplotlib.pyplot as plt

#vid = cv2.VideoCapture('/home/pi/Videos/Blue_Sky_and_Clouds_Timelapse_0892__Videvo_preview.mp4')   
vid = cv2.VideoCapture('https://192.168.137.161:8080/video')
#vid = cv2.VideoCapture(0)
while(True):
 
 ret,frame = vid.read()
 #print(ret)
 
 if ret is True:
  cv2.imshow("frame",frame)
 else:
  break   
 #plt.imshow(frame)
 
 #plt.show()
 
 key = cv2.waitKey(1)
 if key == 27:
   break
 
vid.release()
cv2.destroyAllWindows()
  
 