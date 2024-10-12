import numpy as np
import time

class Car:
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
        self.env_grid = np.zeros((100, 200))
        self.numScans = 0
    
    def sig_int_handler(self, signal, frame):
        print("Signal interrupt received. Stopping the car.")
        self.stop()
        sys.exit(0)

    def stop(self):
        print("Stopping the car.")

    def forward(self, power_val):
        print(f"Moving forward with power {power_val}.")

    def turn_left(self, power_val):
        print(f"Turning left with power {power_val}.")

    def turn_right(self, power_val):
        print(f"Turning right with power {power_val}.")

    def get_distance_at(self, angle):
        # Simulate a distance sensor value
        return np.random.randint(50, 200)

    def get_x(self):
        return self.position[0]

    def get_y(self):
        return self.position[1]

    def verify_step(self):
        scans = []
        steps = self.angle.copy()
        checkScan = False
        if self.mAngle == self.currAngle:
            steps = np.flip(steps)
            checkScan = True
        for angle in steps:
            self.currAngle = angle
            scans.append(self.get_distance_at(angle))
        if checkScan:
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
                r = int(self.position[0] - (cos[i] * arrScan[i] / 5))
                c = int(self.position[1] - (sins[i] * arrScan[i] / 5))
                if r >= 100 or r < 0:
                    continue
                if c >= 200 or c < 0:
                    continue
                if arrScan[i] < 100 and arrScan[i] > 0:
                    self.env_grid[r][c] = 1
                    if final == 1:
                        if c != fCol:
                            curve = (r - fRow) / (c - fCol)  # updating the first col in the mapping grid
                            first = c if c > fCol else last_col
                            second = last_col if c == last_col else c
                            for i in range(first, second):
                                self.env_grid[int(fRow + curve * (i - last_col))][i] = 1
                        else:
                            for i in range(fRow, r):
                                self.env_grid[i][c] = 1
                    fRow = r
                    fCol = c
                    final = 1
            scansCount += 1
            self.numScans += 1
            np.savetxt("map_{}.csv".format(scansCount=self.numScans), self.env_grid, delimiter=",", fmt='%i')

    def get_env_grid(self):
        return self.env_grid

# Example usage:
if __name__ == "__main__":
    car = Car()
    car.mapping()  # This will simulate mapping and save the grid to a file
