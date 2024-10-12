#import picar_4wd as fc

# Stop the program if the ESC key is pressed.

import sys
import time

import cv2
import mediapipe as mp

from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from utils import visualize
from picamera2 import Picamera2

# Global variables to calculate FPS
COUNTER, FPS = 0, 0
START_TIME = time.time()
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640,480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

def get_category_name(detection_results):
    if detection_results and detection_results[0].detections:
        objectName= detection_results[0].detections[0].categories[0].category_name
        #if objectName is not None
        #code logic for stop sign

        if objectName is not None:
            if objectName=="stop sign":
                return 0
            elif objectName=="traffic light":
                return 1
            else:
                return 3
         




def run(model: str, max_results: int, score_threshold: float, 
        camera_id: int, width: int, height: int) -> int:
    


  row_size = 50  
  left_margin = 24  
  text_color = (0, 0, 255) 
  font_size = 1
  font_thickness = 1
  fps_avg_frame_count = 10

  detection_frame = None
  detection_result_list = []

  def save_result(result: vision.ObjectDetectorResult, unused_output_image: mp.Image, timestamp_ms: int):
      global FPS, COUNTER, START_TIME

      if COUNTER % fps_avg_frame_count == 0:
          elapsed_time = time.time() -START_TIME
          if elapsed_time > 0:
              FPS =fps_avg_frame_count / elapsed_time
          START_TIME=time.time()

      detection_result_list.append(result)
      COUNTER += 1

  #  detection model
  base_options = python.BaseOptions(model_asset_path=model)
  options = vision.ObjectDetectorOptions(base_options=base_options,
                                         running_mode=vision.RunningMode.LIVE_STREAM,
                                         max_results=max_results, score_threshold=score_threshold,
                                         result_callback=save_result)
  detector = vision.ObjectDetector.create_from_options(options)

  while True:
      im= picam2.capture_array()  
      image = cv2.flip(im, -1)

      mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)

      detector.detect_async(mp_image, time.time_ns() // 1_000_000)

      current_frame = image

      if detection_result_list:
# uncomment this part if you want to have a preview window

       # current_frame = visualize(current_frame, detection_result_list[0])
      #  detection_frame = current_frame
        catchValue = get_category_name(detection_result_list)
        if catchValue is not None:
            return catchValue
        
        detection_result_list.clear()
       

      if detection_frame is not None:
        cv2.imshow('object_detection', detection_frame)
    
#     if cv2.waitKey(1) == 27:
#       break
#     cv2.waitKey(0)
#       time.sleep(3)
#       break
  detector.close()
  cap.release()

  cv2.destroyAllWindows()


def main():
  #remove the print satement
  return (run("efficientdet_lite0.tflite", 2, 0.4, 0, 640, 480))

if __name__ == '__main__':
  main()
  
