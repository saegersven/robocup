from picamera import PiCamera

def capture(path, res):
	camera = PiCamera()
	camera.resolution = res
	camera.capture(path)