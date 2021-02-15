from inputs import get_gamepad

import cv2
import numpy as np

button_dict = {
    'ABS_X': [0, 0],
    'ABS_Y': [0, 0]
}

if __name__ == '__main__':
    winname = 'plot'
    cv2.namedWindow(winname)
    bg = np.zeros((500, 500, 3), dtype=np.uint8)
    bg = cv2.circle(bg, (250, 250), 100, (140, 140, 140))

    l_circle = [250, 250]

    while 1:
        plot = bg.copy()
        text_color = (160, 160, 160)
        plot = cv2.putText(plot, 'ABS_Y {}'.format(
            button_dict['ABS_Y'][0]), (200, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color)
        plot = cv2.putText(plot, 'ABS_Y {}'.format(
            button_dict['ABS_Y'][1]), (200, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color)
        plot = cv2.putText(plot, 'ABS_X {}'.format(
            button_dict['ABS_X'][0]), (50, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color)
        plot = cv2.putText(plot, 'ABS_X {}'.format(
            button_dict['ABS_X'][1]), (355, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color)

        plot = cv2.circle(plot, tuple(l_circle), 10, (0, 0, 200))

        cv2.imshow(winname, plot)
        cv2.waitKey(1)
        events = get_gamepad()
        for e in events:
            print(e.ev_type, e.code, e.state)
            if e.code == 'ABS_X' or e.code == 'ABS_Y':
                butt = button_dict[e.code]
                butt[0] = min(butt[0], e.state)
                butt[1] = max(butt[1], e.state)
                x_min = button_dict['ABS_X'][0]
                x_max = button_dict['ABS_X'][1]
                y_min = button_dict['ABS_Y'][0]
                y_max = button_dict['ABS_Y'][1]
                l_circle[0] = int(e.state / max(abs(x_min), abs(x_max)) * 100) + \
                    250 if e.code == 'ABS_X' and max(
                        abs(x_min), abs(x_max)) > 0 else l_circle[0]
                l_circle[1] = int(e.state / max(abs(y_min), abs(y_max)) * 100) + \
                    250 if e.code == 'ABS_Y' and max(
                        abs(y_min), abs(y_max)) else l_circle[1]

        cv2.imshow(winname, plot)
        cv2.waitKey(1)
