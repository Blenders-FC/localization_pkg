import math

# Constants
FIELD_WIDTH_CM = 900
FIELD_HEIGHT_CM = 600
LEFT_POST_Y_SUP = 170
RIGHT_POST_Y_SUB = 430

LEFT_POST = (0, RIGHT_POST_Y_SUB)
RIGHT_POST = (0, LEFT_POST_Y_SUP)
LEFT_GOALS = [LEFT_POST, RIGHT_POST]
RIGHT_GOALS = [(FIELD_WIDTH_CM, LEFT_POST_Y_SUP), (FIELD_WIDTH_CM, RIGHT_POST_Y_SUB)]

# Map quadrant to base robot orientation (in degrees)
base_headings = {
    1: 180,  # facing left
    2:   0,  # facing right
    3:   0,  # facing right
    4: 180   # facing left
}


class InitialPoseEstimator:
    def __init__(self, quadrant: int, post: "Calc_Post"):
        assert quadrant in [1, 2, 3, 4], "Quadrant must be 1–4"
        self.quadrant = quadrant
        self.post = post

    def get_robot_position(self):
        # Decide which post is expected based on quadrant
        post_index = 0 if self.quadrant % 2 == 1 else 1

        # Choose goal side
        post_coord = RIGHT_GOALS[post_index] if self.quadrant in [1, 4] else LEFT_GOALS[post_index]
        print(f"post coord: {post_coord}")

        post_x, post_y = post_coord
        d = self.post.distance
        relative_angle_deg = math.degrees(self.post.angle)

        # Robot's base heading in the global frame per quadrant
        base_heading = base_headings[self.quadrant]
        global_angle_rad = math.radians(base_heading + relative_angle_deg)

        # Compute robot position
        robot_x = post_x + d * math.cos(global_angle_rad)

        # Use quadrant to decide Y sign
        if self.quadrant in [1, 3]:
            robot_y = post_y + d * math.sin(global_angle_rad)
        else:  # Quadrant 2 or 4
            robot_y = post_y - d * math.sin(global_angle_rad)

        return robot_x, robot_y


















import cv2
import numpy as np
import math

SCALE = 1  # 1 cm = 5 pixels

FIELD_WIDTH_CM = 900
FIELD_HEIGHT_CM = 600

IMG_WIDTH = FIELD_WIDTH_CM * SCALE
IMG_HEIGHT = FIELD_HEIGHT_CM * SCALE

LEFT_POST_X_SUP = 0
LEFT_POST_Y_SUP = 170
RIGHT_POST_X_SUB = IMG_WIDTH - 1
RIGHT_POST_Y_SUB = 430


# Convert cm to image pixel coordinates
cm_to_px = lambda x_cm, y_cm: (int(x_cm * SCALE), int(y_cm * SCALE))



class SoccerFieldCV:
    def __init__(self):
        self.image = np.zeros((IMG_HEIGHT, IMG_WIDTH, 3), dtype=np.uint8)

    def draw(self):
        self.image[:] = (0, 128, 0)  # green background
        self.draw_boundaries()
        self.draw_center_circle()
        self.draw_penalty_area()
        self.draw_goal_area()
        self.draw_penalty_points()
        self.draw_goals()
        self.draw_center_line()
        return self.image

    def draw_boundaries(self):
        cv2.rectangle(self.image, (0, 0), (IMG_WIDTH - 1, IMG_HEIGHT - 1), (255, 255, 255), 2)

    def draw_center_line(self):
        cv2.line(self.image, (IMG_WIDTH // 2, 0), (IMG_WIDTH // 2, IMG_HEIGHT), (255, 255, 255), 2)

    def draw_center_circle(self, radius_cm=75):
        center = cm_to_px(FIELD_WIDTH_CM // 2, FIELD_HEIGHT_CM // 2)
        radius_px = int(radius_cm * SCALE)
        cv2.circle(self.image, center, radius_px, (255, 255, 255), 2)
        cv2.circle(self.image, cm_to_px(450, 300), 5, (255, 255, 255), -1)

    def draw_penalty_area(self):
        cv2.rectangle(self.image, cm_to_px(0, 50), cm_to_px(200, 550), (255, 255, 255), 2)
        cv2.rectangle(self.image, cm_to_px(700, 50), cm_to_px(IMG_WIDTH, 550), (255, 255, 255), 2)

    def draw_goal_area(self):
        cv2.rectangle(self.image, cm_to_px(0, 150), cm_to_px(100, 450), (255, 255, 255), 2)
        cv2.rectangle(self.image, cm_to_px(800, 150), cm_to_px(IMG_WIDTH, 450), (255, 255, 255), 2)

    def draw_penalty_points(self):
        left_point = cm_to_px(150, 300)
        right_point = cm_to_px(750, 300)
        cv2.circle(self.image, left_point, 5, (255, 255, 255), -1)
        cv2.circle(self.image, right_point, 5, (255, 255, 255), -1)

    def draw_goals(self):
        # Goals on left and right in magenta
        for y_cm in range(LEFT_POST_Y_SUP, RIGHT_POST_Y_SUB):
            pt1 = cm_to_px(LEFT_POST_X_SUP, y_cm)
            pt2 = cm_to_px(RIGHT_POST_X_SUB, y_cm)
            self.image[pt1[1], pt1[0]] = (255, 0, 255)
            self.image[pt2[1], pt2[0]] = (255, 0, 255)
            # line width
            self.image[pt1[1], pt1[0] + 1] = (255, 0, 255)
            self.image[pt2[1], pt2[0] - 1] = (255, 0, 255)

        # Posts
        for y_cm in [LEFT_POST_Y_SUP, RIGHT_POST_Y_SUB]:
            for x_cm in [LEFT_POST_X_SUP, RIGHT_POST_X_SUB]:
                pt = cm_to_px(x_cm, y_cm)
                cv2.circle(self.image, pt, 5, (255, 0, 0), -1)  # Yellow posts


class Localization:
    def __init__(self, post_1, post_2, two_goals=True) -> None:
        self._post_1 = post_1
        self._post_2 = post_2
        self._two_goals = two_goals

        self.inital_entry = True
        self.x_position = None
        self.y_position = None
        self._nearest_goal_side = None
        self._y_goal = 300  # 3.0 m
        self._side = 0  # 0: Right |  1: Left

    def get_position(self):
        if self._two_goals:
            robot_coord = self._calc_two_goals()
            if robot_coord[0] != None and robot_coord[1] != None:
                return robot_coord
            else:
                return None

    def _calc_two_goals(self):
        robot_x = None; robot_y = None
        post_1_x = LEFT_POST_X_SUP; post_1_y = LEFT_POST_Y_SUP
        post_2_x = LEFT_POST_X_SUP; post_2_y = RIGHT_POST_Y_SUB

        init_robot_x = abs(self._post_1.distance * math.cos(self._post_1.angle)) + post_1_x
        comp_robot_x = abs(self._post_2.distance * math.cos(self._post_2.angle)) + post_2_x
        # print(f"hueeeeeeeeee {init_robot_x}")
        # print(f"hueeeeeeeeee {comp_robot_x}")
        if self._post_1.distance > self._post_2.distance:
            if self._post_1.angle > 0 and self._post_2.angle > 0:
                init_robot_y = abs(self._post_1.distance * math.sin(self._post_1.angle) + post_1_y)
                comp_robot_y = abs(self._post_2.distance * math.sin(self._post_2.angle) + post_2_y)
            elif self._post_1.angle > 0 and self._post_2.angle < 0:
                # NOT VALIDATED YET
                init_robot_y = abs(self._post_1.distance * math.sin(self._post_1.angle) + post_1_y)
                comp_robot_y = abs(self._post_2.distance * math.sin(self._post_2.angle) + post_2_y)
            elif self._post_1.angle < 0 and self._post_2.angle > 0:
                # NOT VALIDATED YET
                init_robot_y = abs(self._post_1.distance * math.sin(self._post_1.angle) - post_1_y)
                comp_robot_y = abs(self._post_2.distance * math.sin(self._post_2.angle) - post_2_y)
            else:
                # NOT VALIDATED YET
                init_robot_y = abs(self._post_1.distance * math.sin(self._post_1.angle) - post_1_y)
                comp_robot_y = abs(self._post_2.distance * math.sin(self._post_2.angle) - post_2_y)
        else:
            if self._post_1.angle > 0 and self._post_2.angle > 0:
                print(self._post_1.distance * math.sin(self._post_1.angle))
                print(self._post_2.distance * math.sin(self._post_2.angle))
                print(post_1_y)
                print(post_2_y)
                init_robot_y = abs(self._post_1.distance * math.sin(self._post_1.angle) + post_2_y)
                comp_robot_y = abs(self._post_2.distance * math.sin(self._post_2.angle) + post_1_y)
            elif self._post_1.angle > 0 and self._post_2.angle < 0:
                # NOT VALIDATED YET
                init_robot_y = abs(self._post_1.distance * math.sin(self._post_1.angle) + post_1_y)
                comp_robot_y = abs(self._post_2.distance * math.sin(self._post_2.angle) + post_2_y)
            elif self._post_1.angle < 0 and self._post_2.angle > 0:
                # NOT VALIDATED YET
                init_robot_y = abs(self._post_1.distance * math.sin(self._post_1.angle) - post_1_y)
                comp_robot_y = abs(self._post_2.distance * math.sin(self._post_2.angle) - post_2_y)
            else:
                init_robot_y = abs(self._post_1.distance * math.sin(self._post_1.angle) + post_1_y)
                comp_robot_y = abs(self._post_2.distance * math.sin(self._post_2.angle) + post_2_y)
        # print(f"despciywcbuéeees {init_robot_y}")
        # print(f"despciywcbuéeees {comp_robot_y}")

        if abs(init_robot_x - comp_robot_x) < 30:
            robot_x = abs(init_robot_x + comp_robot_x)/2
        if abs(init_robot_y - comp_robot_y) < 30:
            robot_y = abs(init_robot_y + comp_robot_y)/2
        
        return (robot_x, robot_y)


class Robot(Localization):
    def __init__(self, field, size):
        post_1 = Calc_Post(distance=319.83, angle=0.5626)
        post_2 = Calc_Post(distance=508.55, angle=1.0098)
        super().__init__(post_1, post_2)  # 90 - robot angle

        self.field = field
        self.x_cm = None  # 450  # default in the center
        self.y_cm = None  # 300
        self.size = int(size / 2)

    def set_position(self, x_cm, y_cm):
        self.x_cm = x_cm
        self.y_cm = y_cm

    def draw(self, image):
        # Robot sees a goalpost at 400 cm, 45° to the left
        post_measurement = Calc_Post(distance=497, angle=math.radians(20))

        # We're assuming robot is in quadrant 2 (top-left)
        estimator = InitialPoseEstimator(quadrant=3, post=post_measurement)
        x_cm, y_cm = estimator.get_robot_position()

        print(f"Estimated position: x = {x_cm:.2f} cm, y = {y_cm:.2f} cm")
        cv2.circle(image, (round(x_cm), round(y_cm)), self.size, (0, 0, 255), -1)  # Red robot


        # """Draws the robot at its current position."""
        # #self.two_goals = False
        # if self.get_position() is None:
        #     print("ERRRORR")
        # else:
        #     self.x_cm, self.y_cm = self.get_position() # (50, 30)
        
        # if self.x_cm is not None and self.y_cm is not None:
        #     x_px, y_px = cm_to_px(round(self.x_cm), round(self.y_cm))
        #     print(f'Position Point: ({x_px}, {y_px})\n')
        #     cv2.circle(image, (x_px, y_px), self.size, (0, 0, 255), -1)  # Red robot


class Calc_Post():
    def __init__(self, distance, angle):
        self.distance = distance
        self.angle = angle


def main_loop():
    field = SoccerFieldCV()
    robot = Robot(field=field, size=20)

    while True:
        frame = field.draw()
        robot.draw(frame)

        cv2.imshow("Soccer Field", frame)
        key = cv2.waitKey(100)

        if key == ord('q'):
            break
        elif key == ord('w'):
            robot.y_cm -= 1
        elif key == ord('s'):
            robot.y_cm += 1
        elif key == ord('a'):
            robot.x_cm -= 1
        elif key == ord('d'):
            robot.x_cm += 1

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main_loop()
