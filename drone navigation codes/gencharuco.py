import cv2
import cv2.aruco as aruco

gridboard = aruco.CharucoBoard_create(
        squaresX=5, 
        squaresY=7, 
        squareLength=0.04, 
        markerLength=0.02, 
        dictionary=aruco.Dictionary_get(aruco.DICT_5X5_1000))

# Create an image from the gridboard
img = gridboard.draw(outSize=(988, 1400))
cv2.imwrite("test_charuco.jpg", img)

# Display the image to us
cv2.imshow('Gridboard', img)
# Exit on any key
cv2.waitKey(0)
cv2.destroyAllWindows()