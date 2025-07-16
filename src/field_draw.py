import curses
import time
import math

class SoccerField:
    def __init__(self, stdscr):
        self.field_height = 60  # dm
        self.field_width = 180  # 1/2dm == 90m
        self._stdscr = stdscr
        self._init_colors()
        self.conditions = {
            "boundaries": lambda x, y: (y == 0 or y == self.field_height - 1                 # Top and bottom boundary
                                        or x == 0 or x == self.field_width - 1               # Side boundaries
                                        ),
            "penalty_area": lambda x, y: ((y in {5, 55} and (x < 41 or x > 139))             # Top and bottom boundary
                                          or (x in {40, 140} and 5 < y < 55)                 # Initial boundary
                                          ),
            "goal_area": lambda x, y: ((y in {15, 45} and (x < 11 or x > 169))               # Top and bottom boundary
                                       or (x in {10, 170} and 15 < y < 45)                   # Initial boundary
                                       ),
            "penalty_point": lambda x, y: ((y == 30 and (27 < x < 33 or 147 < x < 153)) 
                                           or (x in {30, 150} and 28 < y < 32)
                                           ),
            "goals": lambda x, y: ((17 < y < 43) and (x == 0 or x == 179)
                                   ),
            "posts": lambda x, y: ((y == 18 or y == 42) and (x == 0 or x == 179)
                                     ),
            "center_line": lambda x: x == self.field_width // 2,
        }


    def draw(self):
        """Draws the complete field including boundaries and center circle."""
        self._stdscr.clear()
        self._draw_boundaries()
        self._draw_center_circle()
        self._draw_goals()
        self._stdscr.refresh()


    def _draw_boundaries(self):
        """Draw the field boundaries and other features."""
        for y in range(self.field_height):
            for x in range(self.field_width):
                if self.conditions["boundaries"](x, y) or self.conditions["center_line"](x):
                    self._stdscr.addch(y, x, '*', curses.color_pair(1))
                elif self.conditions["penalty_area"](x, y):
                    self._stdscr.addch(y, x, '*', curses.color_pair(1))
                elif self.conditions["goal_area"](x, y):
                    self._stdscr.addch(y, x, '*', curses.color_pair(1))
                elif self.conditions["penalty_point"](x, y):
                    self._stdscr.addch(y, x, '*', curses.color_pair(1))
                else:
                    self._stdscr.addch(y, x, ' ', curses.color_pair(4))


    def _draw_center_circle(self, radius=7):
        """Draw an ASCII circle at the center of the field."""
        h, k = self.field_height // 2, self.field_width // 2
        for y in range(h - radius, h + radius + 1):
            for x in range(k - radius * 2, k + radius * 2 + 1):
                adjusted_x = (x - k) * 0.5
                distance = math.sqrt(adjusted_x ** 2 + (y - h) ** 2)
                if (radius - 0.35) < distance < (radius + 0.35):
                    self._stdscr.addch(y, x, '*', curses.color_pair(1))


    def _draw_goals(self):
        """Remark the goals zone in yellow"""
        for y in range(self.field_height):
            for x in range(self.field_width):
                if self.conditions["posts"](x, y):
                    self._stdscr.addch(y, x, '*', curses.color_pair(5))
                elif self.conditions["goals"](x, y):
                    self._stdscr.addch(y, x, '*', curses.color_pair(2))


    def _init_colors(self):
        """Initialize color pairs."""
        curses.start_color()
        curses.init_pair(1, curses.COLOR_WHITE, curses.COLOR_WHITE)         # Field elements
        curses.init_pair(2, curses.COLOR_MAGENTA, curses.COLOR_MAGENTA)     # Goals
        curses.init_pair(3, curses.COLOR_RED, curses.COLOR_RED)             # Robot
        curses.init_pair(4, curses.COLOR_GREEN, curses.COLOR_GREEN)         # Field
        curses.init_pair(5, curses.COLOR_YELLOW, curses.COLOR_YELLOW)       # Goals posts



class Localization:
    def __init__(self, post_1, post_2, two_goals=True) -> None:
        self._post_1 = post_1
        self._post_2 = post_2
        self._two_goals = two_goals

        self.inital_entry = True
        self.x_position = None
        self.y_position = None
        self._nearest_goal_side = None
        self._y_goal = 30  # 3.0 m
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
        post_1_x = 0; post_1_y = 18
        post_2_x = 0; post_2_y = 42
        init_robot_x = abs(self._post_1.distance * math.cos(self._post_1.angle)) + post_1_x
        comp_robot_x = abs(self._post_2.distance * math.cos(self._post_2.angle)) + post_2_x
        if self._post_1.distance > self._post_2.distance:
            if self._post_1.angle > 0 and self._post_2.angle > 0:
                init_robot_y = abs(self._post_1.distance * math.sin(self._post_1.angle) + post_1_y)
                comp_robot_y = abs(self._post_2.distance * math.sin(self._post_2.angle) + post_2_y)
            elif self._post_1.angle > 0 and self._post_2.angle < 0:
                init_robot_y = abs(self._post_1.distance * math.sin(self._post_1.angle) + post_1_y)
                comp_robot_y = abs(self._post_2.distance * math.sin(self._post_2.angle) + post_2_y)
            elif self._post_1.angle < 0 and self._post_2.angle > 0:
                init_robot_y = abs(self._post_1.distance * math.sin(self._post_1.angle) - post_1_y)
                comp_robot_y = abs(self._post_2.distance * math.sin(self._post_2.angle) - post_2_y)
            else:
                init_robot_y = abs(self._post_1.distance * math.sin(self._post_1.angle) - post_1_y)
                comp_robot_y = abs(self._post_2.distance * math.sin(self._post_2.angle) - post_2_y)
        else:
            if self._post_1.angle > 0 and self._post_2.angle > 0:
                init_robot_y = abs(self._post_1.distance * math.sin(self._post_1.angle) - post_1_y)
                comp_robot_y = abs(self._post_2.distance * math.sin(self._post_2.angle) - post_2_y)
            elif self._post_1.angle > 0 and self._post_2.angle < 0:
                init_robot_y = abs(self._post_1.distance * math.sin(self._post_1.angle) + post_1_y)
                comp_robot_y = abs(self._post_2.distance * math.sin(self._post_2.angle) + post_2_y)
            elif self._post_1.angle < 0 and self._post_2.angle > 0:
                init_robot_y = abs(self._post_1.distance * math.sin(self._post_1.angle) - post_1_y)
                comp_robot_y = abs(self._post_2.distance * math.sin(self._post_2.angle) - post_2_y)
            else:
                print(self._post_1.distance)
                print(self._post_1.angle)
                print(post_1_y)
                init_robot_y = abs(self._post_1.distance * math.sin(self._post_1.angle) + post_1_y)
                comp_robot_y = abs(self._post_2.distance * math.sin(self._post_2.angle) + post_2_y)
        print(f"despciywcbuéeees {init_robot_y}")
        print(f"despciywcbuéeees {comp_robot_y}")

        if abs(init_robot_x - comp_robot_x) < 3:
            robot_x = abs(init_robot_x + comp_robot_x)/2
        if abs(init_robot_y - comp_robot_y) < 3:
            robot_y = abs(init_robot_y + comp_robot_y)/2
        
        return (robot_x, robot_y)


class Robot(Localization):
    def __init__(self, field):
        post_2 = Calc_Post(distance=43.863, angle=-2.324)
        post_1 = Calc_Post(distance=31.048, angle=-2.881)
        super().__init__(post_1, post_2, True)  # 90 - robot angle
        self.field = field
        self.x = None
        self.y = None


    def _move(self):
        """Moves the robot diagonally down-right across the field."""
        self.x = (self.x + 1) % (self.field.field_width - 1)
        self.y = (self.y + 1) % (self.field.field_height - 1)


    def draw(self):
        """Draws the robot at its current position."""
        #self.two_goals = False
        if self.get_position() is None:
            print("ERRRORR")
        else:
            self.x, self.y = self.get_position() # (50, 30)
        if self.x is not None and self.y is not None:
            self.x = round(self.x); self.y = round(self.y)
            print(f'Position Point: ({self.x}, {self.y})\n')
            self.field._stdscr.addch(self.y, self.x, '*', curses.color_pair(3))


class Calc_Post():
    def __init__(self, distance, angle):
        self.distance = distance
        self.angle = angle


def main(stdscr):
    # Initialize curses settings
    curses.curs_set(0)  # Hide the cursor
    stdscr.nodelay(1)   # Don't wait for user input to continue the loop
    stdscr.timeout(100) # Refresh every 100 ms

    # Initialize the soccer field and robot
    field = SoccerField(stdscr)
    robot = Robot(field)

    # Main loop
    while True:
        # Draw field and robot
        field.draw()
        robot.draw()
        
        # Move the robot
        # robot.move()

        # Sleep briefly to control movement speed
        time.sleep(0.1)

        # Stop the loop with 'q' key
        key = stdscr.getch()
        if key == ord('q'):
            break

# Run the curses application
curses.wrapper(main)
