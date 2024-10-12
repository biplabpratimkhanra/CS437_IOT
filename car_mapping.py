import picar_4wd as fc
import sys
import signal
import numpy as np
import time

# Car class definition
class Car():
    def __init__(self):
        self.turnRadius = 13.97
        self.radiusWheel = 3.175
        self.orientation = 90
        self.position = (100, 99)
        self.aRange = 144
        self.stepAmt = 18
        self.us_step = self.stepAmt
        self.angle = np.array([i for i in range(-int(self.aRange / 2), int(self.aRange / 2) + self.stepAmt, self.stepAmt)])
        self.scan = []

        self.angleDist = [0,0]
        self.currAngle = 0
        self.mAngle = self.aRange/2
        self.lAngle = self.aRange/2
        self.scanList = []
        self.numScans = 0

        self.env_grid = np.zeros((100, 200))

        fc.left_rear_speed.start()
        fc.right_rear_speed.start()
        signal.signal(signal.SIGINT, self.sig_int_handler)

    def sig_int_handler(self, signal, frame):
        fc.stop()
        fc.right_rear_speed.deinit()
        fc.left_rear_speed.deinit()
        sys.exit(0)

    def move_distance(self, dist_to_travel, power_val):
        dist_to_travel = dist_to_travel * 1.5

        time.sleep(1)
        curr_dist_traveled = 0
        fc.forward(power)

        dist_increment_val = fc.speed_val() * 0.1
        for dist_val in range(0, dist_to_travel, dist_increment_val):
            # waiting till we reach the approximated distance
            time.sleep(0.002)
            print(dist_val)

        fc.stop()

        prev_x_coord = self.position[0]
        prev_y_coord = self.position[1]

        x_coord = prev_x_coord + dist_to_travel * np.cos(self.orientation)
        y_coord = prev_y_coord + dist * np.sin(self.orientation)
        self.position = (x_coord, y_coord)

    def move_angle(self, angle, directional_power):
        arc_length = np.radians(np.abs(angle)) * self.turnRadius

        left_dist = 0
        right_dist = 0
        time.sleep(0.5)
        if angle >= 0:
            fc.turn_right(directional_power)
        else:
            fc.turn_left(directional_power)

        while np.abs(right_dist) <= arc_length and np.abs(left_dist) <= arc_length:
            time.sleep(0.002)
            left_dist += fc.left_rear_speed() * 0.002
            right_dist += fc.right_rear_speed() * 0.002

        fc.stop()

        self.orientation = (self.orientation - angle) % 360

    def map_orientation(self, angle, power):
        if angle == self.orientation:
            return
        turningDeg = self.orientation - angle
        if 180 < np.abs(turningDeg):
            if turningDeg >= 0:
                turningDeg = (360 - turningDeg) * -1
        else:
            turningDeg += 360
        self.move_angle(turningDeg, power)
        self.orientation = angle
    
    def verify_step(self):
        scans = []
        steps = self.angle.copy()
        checkScan = False
        if self.mAngle == self.currAngle:
            steps = np.flip(steps)
            checkScan = True
        for angle in steps:
            self.currAngle = angle
            scans.append(fc.get_distance_at(angle))
        if checkScan == True:
            scans.reverse()
        return scans

    def mapping(self):
        scansCount = 1
        sins = np.sin(np.radians(self.orientation + self.angle))
        cos = np.cos(np.radians(self.orientation + self.angle))
        arrScan = np.array(self.verify_step())
        while scansCount <= 2:
            final = 0
            fRow = -1
            fCol = -1
            for i in range(len(arrScan)):
                if arrScan[i] <= 0:
                    continue
                c = int(self.position[0] - (cos[i] * arrScan[i] / 5))
                r = int(self.position[1] - (sins[i] * arrScan[i] / 5))
                if r >= 100 or r < 0:
                    continue
                if c >= 200 or c < 0:
                    continue
                if arrScan[i] < 100 and arrScan[i] > 0:
                    self.env_grid[r][c] = 1
                    if final == 1:
                        if c != fCol:
                            curve = (r - fRow) / (c - fCol)  # updating the first col in the mapping grid
                            if c > fCol:
                                first = c
                            else:
                                first = last_col
                            if c == last_col:
                                second = last_col
                            else:
                                second = c
                            for i in range(first, second):
                                self.env_grid[int(fRow + curve * (i - last_col))][i] = 1
                        else:
                            for i in range(fRow, r):
                                self.env_grid[i][c] = 1
                    fRow = r
                    last_col = c
                    final = 1

                else:
                    final = 0
            scansCount += 1
        self.numScans += 1
        np.savetxt("map_{scansCount}.csv".format(scansCount=self.numScans), self.env_grid, delimiter=",", fmt='%i')

    def get_env_grid(self):
        return self.env_grid

    def get_x(self):
        return self.position[0]
    
    def get_y(self):
        return self.position[0]

if __name__ == "__main__":
    car = Car()
    car.mapping()  # This will simulate mapping and save the grid to a file