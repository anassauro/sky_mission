import RPi.GPIO as GPIO
import cv2 as cv

GPIO.setmode(GPIO.BOARD)

trigpin=3
GPIO.setup (trigpin, GPIO.IN)

cam = cv.VideoCapture(0)
ret, image = cam.read()

triggered=0
imagenum=0
imagemax=100

if ret:
	while imagenum<imagemax:
		input=GPIO.input(trigpin)
		if not triggered and input==1:
			triggered=1
			#cv.imshow('SnapshotTest',image)
			#cv.waitKey(0)
			#cv.destroyWindow('SnapshotTest')
			import cv2 as cv
			#cam=cv.VideoCapture(0)
			ret, image=cam.read()
			cv.imwrite('/home/skyrats/Img/teste%d.jpg'% (imagenum),image)
			print("Foto %d tirada" %imagenum)
			imagenum+=1
		elif triggered and input==0:
			triggered=0
else:
	print('Erro de Câmera')
