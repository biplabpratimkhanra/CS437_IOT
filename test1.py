from detect4ver2 import run

for i in range(25):

    a = run("efficientdet_lite0.tflite", 2, 0.4, 0, 640, 480)

    print(a)