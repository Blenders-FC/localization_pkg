import cv2
import numpy as np
import yaml
import math
import collections
import os

# Map settings
RESOLUTION = 0.01
MAP_WIDTH = 500
MAP_HEIGHT = 500

# Modes
DRAW_OBSTACLE = 1
DRAW_MARK = 2
DRAW_ERASE = 3
DRAW_RECT = 4
DRAW_CIRC = 5
DRAW_LINE = 6
DRAW_LINE_CONT = 7
current_draw_mode = DRAW_OBSTACLE
last_draw_color_mode = DRAW_OBSTACLE

# Colors
COLOR_FREE = 255
COLOR_OBSTACLE = 0
COLOR_MARK = 128

# Map image and history
map_img = np.full((MAP_HEIGHT, MAP_WIDTH), COLOR_FREE, dtype=np.uint8)
undo_stack = []
redo_stack = []

# State
shape_start_point = None
preview_point = None
manual_size = None
current_mouse_pos = None
show_coords = True
snap_angle_mode = False

def save_undo_state():
    undo_stack.append(map_img.copy())
    if len(undo_stack) > 50:
        undo_stack.pop(0)
    redo_stack.clear()

def undo():
    if undo_stack:
        redo_stack.append(map_img.copy())
        map_img[:] = undo_stack.pop()
        print("Undo performed.")
    else:
        print("Nothing to undo.")

def redo():
    if redo_stack:
        undo_stack.append(map_img.copy())
        map_img[:] = redo_stack.pop()
        print("Redo performed.")
    else:
        print("Nothing to redo.")

def get_draw_color():
    return COLOR_OBSTACLE if last_draw_color_mode == DRAW_OBSTACLE else COLOR_MARK

def snap_point_to_angle(start, current, length_override=None):
    dx = current[0] - start[0]
    dy = current[1] - start[1]
    angle = math.atan2(dy, dx)
    snapped_angle = round(angle / (math.pi / 4)) * (math.pi / 4)
    length = math.hypot(dx, dy) if length_override is None else length_override
    snapped_x = int(round(start[0] + length * math.cos(snapped_angle)))
    snapped_y = int(round(start[1] + length * math.sin(snapped_angle)))
    return (snapped_x, snapped_y)

def draw_preview(img):
    if shape_start_point and preview_point:
        color = get_draw_color()
        end_point = preview_point
        if current_draw_mode in [DRAW_LINE, DRAW_LINE_CONT]:
            if snap_angle_mode:
                length = manual_size[0] if manual_size else None
                end_point = snap_point_to_angle(shape_start_point, preview_point, length_override=length)
            elif manual_size:
                angle = math.atan2(preview_point[1]-shape_start_point[1], preview_point[0]-shape_start_point[0])
                length = manual_size[0]
                end_point = (int(round(shape_start_point[0] + length * math.cos(angle))),
                             int(round(shape_start_point[1] + length * math.sin(angle))))
        elif current_draw_mode == DRAW_RECT and manual_size:
            end_point = (shape_start_point[0]+manual_size[0], shape_start_point[1]+manual_size[1])
        elif current_draw_mode == DRAW_CIRC and manual_size:
            end_point = (shape_start_point[0]+manual_size[0], shape_start_point[1])

        if current_draw_mode == DRAW_RECT:
            cv2.rectangle(img, shape_start_point, end_point, color, 1)
        elif current_draw_mode == DRAW_CIRC:
            radius = int(np.hypot(end_point[0]-shape_start_point[0], end_point[1]-shape_start_point[1]))
            cv2.circle(img, shape_start_point, radius, color, 1)
        elif current_draw_mode in [DRAW_LINE, DRAW_LINE_CONT]:
            cv2.line(img, shape_start_point, end_point, color, 1)

def draw_mouse_info(img):
    if current_mouse_pos and show_coords:
        x, y = current_mouse_pos
        wx, wy = x*RESOLUTION, y*RESOLUTION
        cv2.putText(img, f"Pixel: ({x},{y})  World: ({wx:.2f}, {wy:.2f}) m", (10,20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, 100, 1, cv2.LINE_AA)
        if shape_start_point:
            dist_px = np.hypot(x - shape_start_point[0], y - shape_start_point[1])
            dist_m = dist_px * RESOLUTION
            cv2.putText(img, f"Distance: {dist_px:.1f} px / {dist_m:.3f} m", (10,40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, 100, 1, cv2.LINE_AA)
        cv2.drawMarker(img, current_mouse_pos, 100, cv2.MARKER_CROSS, 10, 1)

def ask_manual_size():
    global manual_size
    try:
        if current_draw_mode == DRAW_RECT:
            w = int(input("Rectangle width (px): "))
            h = int(input("Rectangle height (px): "))
            manual_size = (w, h)
        elif current_draw_mode == DRAW_CIRC:
            r = int(input("Circle radius (px): "))
            manual_size = (r,)
        elif current_draw_mode in [DRAW_LINE, DRAW_LINE_CONT]:
            l = int(input("Line length (px): "))
            manual_size = (l,)
    except:
        print("Invalid input.")

def apply_shape(x, y):
    global shape_start_point, manual_size
    color = get_draw_color()
    end_point = (x, y)

    if current_draw_mode in [DRAW_LINE, DRAW_LINE_CONT]:
        if snap_angle_mode:
            length = manual_size[0] if manual_size else None
            end_point = snap_point_to_angle(shape_start_point, (x,y), length_override=length)
        elif manual_size:
            angle = math.atan2(y - shape_start_point[1], x - shape_start_point[0])
            length = manual_size[0]
            end_point = (int(round(shape_start_point[0] + length * math.cos(angle))),
                         int(round(shape_start_point[1] + length * math.sin(angle))))

    if current_draw_mode == DRAW_RECT:
        if manual_size:
            end_point = (shape_start_point[0]+manual_size[0], shape_start_point[1]+manual_size[1])
        cv2.rectangle(map_img, shape_start_point, end_point, color, -1)
        print(f"Drew rectangle from {shape_start_point} to {end_point}")
    elif current_draw_mode == DRAW_CIRC:
        radius = manual_size[0] if manual_size else int(np.hypot(end_point[0]-shape_start_point[0], end_point[1]-shape_start_point[1]))
        cv2.circle(map_img, shape_start_point, radius, color, -1)
        print(f"Drew circle at {shape_start_point} with radius {radius}")
    elif current_draw_mode == DRAW_LINE:
        cv2.line(map_img, shape_start_point, end_point, color, 1)
        print(f"Drew line from {shape_start_point} to {end_point}")
    elif current_draw_mode == DRAW_LINE_CONT:
        cv2.line(map_img, shape_start_point, end_point, color, 1)
        print(f"Drew line from {shape_start_point} to {end_point}")
        shape_start_point = end_point
    manual_size = None
    if current_draw_mode != DRAW_LINE_CONT:
        shape_start_point = None


def mouse_callback(event, x, y, flags, param):
    global shape_start_point, preview_point, current_mouse_pos
    current_mouse_pos = (x, y)

    if event == cv2.EVENT_LBUTTONDOWN:
        if current_draw_mode in [DRAW_RECT, DRAW_CIRC, DRAW_LINE, DRAW_LINE_CONT]:
            if shape_start_point is None:
                shape_start_point = (x, y)
            else:
                save_undo_state()
                apply_shape(x, y)
                preview_point = None
        else:
            save_undo_state()
            if current_draw_mode == DRAW_OBSTACLE:
                cv2.circle(map_img, (x,y), 3, COLOR_OBSTACLE, -1)
            elif current_draw_mode == DRAW_MARK:
                cv2.circle(map_img, (x,y), 3, COLOR_MARK, -1)
            elif current_draw_mode == DRAW_ERASE:
                cv2.circle(map_img, (x,y), 5, COLOR_FREE, -1)

    elif event == cv2.EVENT_MOUSEMOVE:
        if shape_start_point and current_draw_mode in [DRAW_RECT, DRAW_CIRC, DRAW_LINE, DRAW_LINE_CONT]:
            preview_point = (x,y)
        elif flags & cv2.EVENT_FLAG_LBUTTON:
            if current_draw_mode == DRAW_OBSTACLE:
                cv2.circle(map_img, (x,y), 3, COLOR_OBSTACLE, -1)
            elif current_draw_mode == DRAW_MARK:
                cv2.circle(map_img, (x,y), 3, COLOR_MARK, -1)
            elif current_draw_mode == DRAW_ERASE:
                cv2.circle(map_img, (x,y), 5, COLOR_FREE, -1)

def confirm_and_edit_yaml(default_name):
    meta = {
        'image': default_name,
        'resolution': RESOLUTION,
        'origin': [0.0,0.0,0.0],
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
        'negate': 0
    }
    while True:
        img = np.full((250,500,3),255,np.uint8)
        lines = [
            f"MAP SAVE SETTINGS:",
            f"image: {meta['image']}",
            f"resolution: {meta['resolution']}",
            f"origin: {meta['origin']}",
            f"occupied_thresh: {meta['occupied_thresh']}",
            f"free_thresh: {meta['free_thresh']}",
            f"negate: {meta['negate']}",
            "y: confirm, n: edit"
        ]
        y=30
        for l in lines:
            cv2.putText(img, l, (10,y), cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,0,0),1,cv2.LINE_AA)
            y+=30
        cv2.imshow("Save Confirmation", img)
        key = cv2.waitKey(0)&0xFF
        if key==ord('y'):
            cv2.destroyWindow("Save Confirmation")
            return meta
        elif key==ord('n'):
            cv2.destroyWindow("Save Confirmation")
            try:
                n = input(f"Image name [{meta['image']}]: ").strip()
                if n: meta['image']=n
                r = input(f"Resolution [{meta['resolution']}]: ").strip()
                if r: meta['resolution']=float(r)
                o = input(f"Origin x y yaw [{meta['origin']}]: ").strip()
                if o:
                    parts = [float(x) for x in o.split()]
                    if len(parts)==3: meta['origin']=parts
                ot = input(f"Occupied thresh [{meta['occupied_thresh']}]: ").strip()
                if ot: meta['occupied_thresh']=float(ot)
                ft = input(f"Free thresh [{meta['free_thresh']}]: ").strip()
                if ft: meta['free_thresh']=float(ft)
                neg = input(f"Negate [{meta['negate']}]: ").strip()
                if neg: meta['negate']=int(neg)
            except:
                print("Invalid input, try again.")
        else:
            print("Press y or n.")

def save_map_and_yaml(default_name="map.png"):
    meta = confirm_and_edit_yaml(default_name)

    # Paths relative to where the script runs (src/)
    maps_dir = os.path.join("..", "maps")
    os.makedirs(maps_dir, exist_ok=True)

    image_name = meta['image']
    image_path = os.path.join(maps_dir, image_name)
    yaml_name = os.path.splitext(image_name)[0] + ".yaml"
    yaml_path = os.path.join(maps_dir, yaml_name)

    # YAML content points to maps/filename.png (relative to yaml file location)
    meta_clean = {
        'image': image_name,
        'resolution': meta['resolution'],
        'origin': meta['origin'],
        'occupied_thresh': meta['occupied_thresh'],
        'free_thresh': meta['free_thresh'],
        'negate': meta['negate']
    }

    # Save files
    cv2.imwrite(image_path, map_img)
    with open(yaml_path, 'w') as f:
        yaml.dump(meta_clean, f, default_flow_style=None, sort_keys=False)

    print(f"Saved image to {image_path}")
    print(f"Saved YAML to {yaml_path}")

def show_instructions():
    img = np.full((300,500,3),255,np.uint8)
    lines=[
        "o: obstacle  m: mark  e: erase",
        "r: rect  c: circ  l: line  b: cont-line",
        "a: snap angle  i: manual size",
        "u: undo  z: redo  k: toggle coords",
        "s: save  q: quit"
    ]
    y=30
    for l in lines:
        cv2.putText(img,l,(10,y),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,0,0),1,cv2.LINE_AA)
        y+=30
    cv2.imshow("Instructions", img)

def main():
    global current_draw_mode,last_draw_color_mode,shape_start_point,preview_point,show_coords,snap_angle_mode
    cv2.namedWindow("Map Editor")
    cv2.setMouseCallback("Map Editor", mouse_callback)
    show_instructions()
    while True:
        disp = map_img.copy()
        draw_preview(disp)
        draw_mouse_info(disp)
        cv2.imshow("Map Editor", disp)
        k = cv2.waitKey(10)&0xFF
        if k == 27:  # Esc key
            if shape_start_point:
                print("Shape drawing cancelled.")
                shape_start_point = None
                preview_point = None
                snap_angle_mode = False
        elif k==ord('q'):
            break
        elif k==ord('o'):
            current_draw_mode=DRAW_OBSTACLE;last_draw_color_mode=DRAW_OBSTACLE;shape_start_point=None;preview_point=None
        elif k==ord('m'):
            current_draw_mode=DRAW_MARK;last_draw_color_mode=DRAW_MARK;shape_start_point=None;preview_point=None
        elif k==ord('e'):
            current_draw_mode=DRAW_ERASE;shape_start_point=None;preview_point=None
        elif k==ord('r'):
            current_draw_mode=DRAW_RECT;shape_start_point=None;preview_point=None
        elif k==ord('c'):
            current_draw_mode=DRAW_CIRC;shape_start_point=None;preview_point=None
        elif k==ord('l'):
            current_draw_mode=DRAW_LINE;shape_start_point=None;preview_point=None;snap_angle_mode=False
        elif k==ord('b'):
            current_draw_mode=DRAW_LINE_CONT;shape_start_point=None;preview_point=None;snap_angle_mode=False
        elif k==ord('a'):
            if current_draw_mode in [DRAW_LINE,DRAW_LINE_CONT]:
                snap_angle_mode=True
        elif k==ord('i'):
            ask_manual_size()
        elif k==ord('u'):
            undo()
        elif k==ord('z'):
            redo()
        elif k==ord('k'):
            show_coords=not show_coords
        elif k==ord('s'):
            save_map_and_yaml()

    cv2.destroyAllWindows()

if __name__=="__main__":
    main()
