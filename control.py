import cv2
import picar_4wd as fc

from mapping import Mapper,MapCoordinate,AngleDeg
from road_sign_detector import Detector, RoadSign
from imageio.plugins._bsdf import save

TURN_FACT = .01
FW_FACT = .04
UPDATE_FREQUENCY = 4
im_res = (320,240)
save_movie = True
sign_detector = Detector()

def stop_on_sign() -> None:
    _, frame = stream.read()
    bb = sign_detector.detected(frame,RoadSign.Stop)
    while len(bb[roadSign.Stop]) > 0:
        time.sleep(0.5)
        if save_movie:
            for(x,y,w,h) in bb[roadSign.Stop]:
                cv2.rectangle(frame, (x,y), (x+w,y+h),RoadSign.Stop.value,2)
                cv2.putText(frame, 'STOP', (x,y), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale = 1.0, color = RoadSign.Stop.value)
                out.write(frame)
        _, frame = stream.read()
        bb = sign_detector.detected(frame,RoadSign.Stop)
        print('Stop')
    if save_movie:
        out.write(frame)

if __name__ == '__main__':
    output_path = f"./run {datetime.datetime.now().strftime('%H:%M:%S')}"
    os.makedirs(output_path,exist_ok=True)

    #initiate mapper
    mapper = Mapper(output_path=output_path)
    car_position = MapCoordinate(0,0)
    car_heading : AngleDeg = 0
    destination_a = MapCoordinate(50,80)
    destination_b = MapCoordinate(-60,120)

    stream = cv2.VideoCapture(0)
    stream.set(3,im_res[0])
    stream.set(4,im_res[1])

    if save_movie:
        out = cv2.VideoWriter(os.path.join(output_path,'movie.avi'), cv2.VideoWriter_fourcc(*'XVID'),20,im_res)
    (grabbed, frame) = stream.read()

    if grabbed:
        print('camera ok')
        print(f"image resolution : {frame.shape[:2]} pixels")
    else:
        raise "problem with camera"
    
    counter = 0
    for dest in [destination_a,destination_b]:
        mapper.update_grid(car_position,car_heading)
        maneuvars = mapper.plot_navigation_plan(destination)
        new_destination = True

        while len(maneuvars) > 0:
            stop_on_sign()

            maneuvar = maneuvars.pop(0)
            if maneuvar.heading != car_heading:
                delta_ang = maneuvar.heading - car_heading
                if delta_ang > 0:
                    fc.turn_right(1)
                else:
                    fc.turn_left(1)
                time.sleep(TURN_FACT * abs(delta_ang))
                fc.stop()
                car_heading = maneuvar.heading
            stop_on_sign()

            fc.forward(1)
            time.sleep(FW_FACT * maneuvar.forward)
            fc.stop()
            stop_on_sign()

            car_position = maneuvar.end_position

            counter += 1
            if (counter >= UPDATE_FREQUENCY or new_destination) and len(maneuvars) > 0:
                counter = 0
                new_destination = False
