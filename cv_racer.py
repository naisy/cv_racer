"""
Script to drive a keras TF model with the Virtual Race Environment.
Usage:
    cv_racer.py (--host=<ip_address>) (--name=<car_name>)
    
Options:
    -h --help        Show this screen.
"""

import os
import numpy as np
import json
import time
from io import BytesIO
import base64
import re
import socket
import select
from threading import Thread

from docopt import docopt
from cv_functions import *
from cv_lane_detection import *

# Server port
PORT = 9091

def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def replace_float_notation(string):
    """
    Replace unity float notation for languages like
    French or German that use comma instead of dot.
    This convert the json sent by Unity to a valid one.
    Ex: "test": 1,2, "key": 2 -> "test": 1.2, "key": 2
    :param string: (str) The incorrect json string
    :return: (str) Valid JSON string
    """
    regex_french_notation = r'"[a-zA-Z_]+":(?P<num>[0-9,E-]+),'
    regex_end = r'"[a-zA-Z_]+":(?P<num>[0-9,E-]+)}'

    for regex in [regex_french_notation, regex_end]:
        matches = re.finditer(regex, string, re.MULTILINE)

        for match in matches:
            num = match.group('num').replace(',', '.')
            string = string.replace(match.group('num'), num)
    return string


class SDClient:
    def __init__(self, host, port, poll_socket_sleep_time=0.05):
        self.msg = None
        self.host = host
        self.port = port
        self.poll_socket_sleep_sec = poll_socket_sleep_time

        # the aborted flag will be set when we have detected a problem with the socket
        # that we can't recover from.
        self.aborted = False
        self.connect()

        """
        FOR LANE DETECTION
        """
        self.steering_icon = cv2.imread("steering.png", -1)
        self.steering = 0.0
        self.throttle = 0.0
        self.max_throttle = 0.3
        self.x_meter = 15
        self.y_meter = 20
        self.frame_cols, self.frame_rows = (160*3, 120*3)
        self.fontFace = cv2.FONT_HERSHEY_SIMPLEX
        self.fontScale = self.frame_rows/1000.0
        if self.fontScale < 0.4:
            self.fontScale = 0.4
        self.fontThickness = 1 + int(self.fontScale)
        self.max_steering_angle = 46
        sample_str='Sample strings'
        [(text_width, text_height), baseLine] = cv2.getTextSize(text=sample_str, fontFace=self.fontFace, fontScale=self.fontScale, thickness=self.fontThickness)
        self.x_left = int(baseLine)
        self.y_top = int(baseLine)


    def connect(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # connecting to the server 
        print("connecting to", self.host, self.port)
        self.s.connect((self.host, self.port))

        # time.sleep(pause_on_create)
        self.do_process_msgs = True
        self.th = Thread(target=self.proc_msg, args=(self.s,))
        self.th.start()


    def send(self, m):
        self.msg = m

    def send_now(self, msg):
        print("sending now:", msg)
        self.s.sendall(msg.encode("utf-8"))

    def on_msg_recv(self, j):
        # print("got:", j['msg_type'])
        # we will always have a 'msg_type' and will always get a json obj
        pass


    def stop(self):
        # signal proc_msg loop to stop, then wait for thread to finish
        # close socket
        self.do_process_msgs = False
        self.th.join()
        self.s.close()


    def proc_msg(self, sock):
        '''
        This is the thread message loop to process messages.
        We will send any message that is queued via the self.msg variable
        when our socket is in a writable state. 
        And we will read any messages when it's in a readable state and then
        call self.on_msg_recv with the json object message.
        '''
        sock.setblocking(0)
        inputs = [ sock ]
        outputs = [ sock ]
        partial = []

        while self.do_process_msgs:
            # without this sleep, I was getting very consistent socket errors
            # on Windows. Perhaps we don't need this sleep on other platforms.
            time.sleep(self.poll_socket_sleep_sec)

            if True: #try:
                # test our socket for readable, writable states.
                readable, writable, exceptional = select.select(inputs, outputs, inputs)

                for s in readable:
                    # print("waiting to recv")
                    try:
                        data = s.recv(1024 * 64)
                    except ConnectionAbortedError:
                        print("socket connection aborted")
                        self.do_process_msgs = False
                        break
                    
                    # we don't technically need to convert from bytes to string
                    # for json.loads, but we do need a string in order to do
                    # the split by \n newline char. This seperates each json msg.
                    data = data.decode("utf-8")
                    msgs = data.split("\n")

                    for m in msgs:
                        if len(m) < 2:
                            continue
                        last_char = m[-1]
                        first_char = m[0]
                        # check first and last char for a valid json terminator
                        # if not, then add to our partial packets list and see
                        # if we get the rest of the packet on our next go around.                
                        if first_char == "{" and last_char == '}':
                            # Replace comma with dots for floats
                            # useful when using unity in a language different from English
                            m = replace_float_notation(m)
                            j = json.loads(m)
                            self.on_msg_recv(j)
                        else:
                            partial.append(m)
                            if last_char == '}':
                                if partial[0][0] == "{":
                                    assembled_packet = "".join(partial)
                                    assembled_packet = replace_float_notation(assembled_packet)
                                    j = json.loads(assembled_packet)
                                    self.on_msg_recv(j)
                                else:
                                    print("failed packet.")
                                partial.clear()
                        
                for s in writable:
                    if self.msg != None:
                        # print("sending", self.msg)
                        s.sendall(self.msg.encode("utf-8"))
                        self.msg = None
                if len(exceptional) > 0:
                    print("problems w sockets!")

            #except Exception as e:
            #    print("Exception:", e)
            #    self.aborted = True
            #    self.on_msg_recv({"msg_type" : "aborted"})
            #    break


def cv_racer(img, steering_icon, steering, throttle, max_throttle, x_meter, y_meter, frame_cols, frame_rows, fontFace, fontScale, fontThickness, max_steering_angle, x_left, y_top):
    img = cv2.resize(img, (160*3, 120*3))
    """
    LANE DETECTION
    """
    is_pass = False
    lane_image = None
    try:
        # detect lane
        is_pass, \
        panel_rows, panel_left_row1, lane_image, \
        tilt1_deg,tilt2_deg,angle1_deg,angle2_deg,curve1_r,curve2_r, \
        meters_from_center, \
        tilt_deg = lane_detection(img, x_meter, y_meter, frame_cols, frame_rows, fontFace, fontScale, fontThickness)

        if is_pass:
            if np.abs(tilt2_deg - tilt1_deg) >= 5.0 and max_throttle > 0 and throttle > 0.2:
                throttle -= 0.01
            elif throttle > max_throttle:
                throttle -= 0.01
            elif throttle < max_throttle:
                throttle += 0.01
    except:
        import traceback
        traceback.print_exc()
        pass

    if not is_pass:
        # Lane detection failed.
        if throttle > 0:
            throttle -= 0.03
        if throttle < 0:
            throttle = 0.0

    """
    STEERING
    """
    '''
    About left, right
      tilt_deg: + is left, - is right
      angle_deg: - is left, + is right
      meters_from_center: + car is left side, - car is right side
      steering_angle: + is left, - is right
    '''
    ########################################
    # steering angle
    ########################################
    steering_angle = -1.0*tilt_deg + 1*meters_from_center # decided by feeling
    print("meters from center: {}".format(meters_from_center))


    # ajust steering angle
    if steering_angle > max_steering_angle:
        steering_angle = max_steering_angle
    if steering_angle < -1*max_steering_angle:
        steering_angle = -1*max_steering_angle

    # set steering. -1.0 to 1.0
    steering = (float)steering_angle/(float)self.max_steering_angle


    """
    DRAW
    """
    if lane_image is None or not is_pass:
        panel_rows = new_rgb(frame_rows, frame_cols)
        img = cv2.hconcat([panel_rows, img])
        lane_image = copy.deepcopy(img)
        print("lane failed")
    else:
        print("lane success")
        # panel_center_row1に文字を描く
        panel_center_row1 = new_rgb(int(frame_rows/3), int(frame_cols/3))
        display_str = []
        display_str.append("STEERING:{:.1f}".format(steering_angle))
        display_str.append("MAX_THROTTLE:{}".format(max_throttle))
        display_str.append("THROTTLE:{:.2f}".format(throttle))

        end_x, end_y = draw_text(panel_center_row1, display_str, color=(0,255,255), start_x=x_left, start_y=y_top, fontFace=fontFace, fontScale=fontScale, fontThickness=fontThickness)
        if meters_from_center is not None:
            display_str = []
            if meters_from_center >= 0:
                display_str.append("center:"+str(round(meters_from_center*100,2))+"cm right")
                color = (0,255,0)
            else:
                display_str.append("center:"+str(round(meters_from_center*100,2))+"cm left")
                color = (255,20,147)
            end_x, end_y = draw_text(panel_center_row1, display_str, color, start_x=x_left, start_y=end_y, fontFace=fontFace, fontScale=fontScale, fontThickness=fontThickness)
        # panelに絵を描く
        panel_right_row1 = draw_steering(steering_icon, steering_angle, throttle, frame_cols, frame_rows)
        panel_right_row1 = draw_speed(panel_right_row1, speed=int(throttle*100), min_value=0, max_value=100)
        panel_row1 = cv2.hconcat([panel_left_row1,panel_center_row1])
        panel_row1 = cv2.hconcat([panel_row1,panel_right_row1])
        panel_rows = cv2.vconcat([panel_row1,panel_rows])
        panel_rows = cv2.resize(panel_rows, (frame_cols, frame_rows))
        lane_image = cv2.hconcat([panel_rows,lane_image])

    
    cv2.imshow("cv result", lane_image)
    cv2.waitKey(1)
    return steering, throttle


class CVRaceClient(SDClient):
    

    def __init__(self, address, conf, poll_socket_sleep_time=0.01):
        super().__init__(*address, poll_socket_sleep_time=poll_socket_sleep_time)
        self.last_image = None
        self.car_loaded = False
        self.conf = conf

    def on_msg_recv(self, json_packet):
        #print("got", json_packet['msg_type'])

        if json_packet['msg_type'] == "need_car_config":
            self.send_config(self.conf)

        if json_packet['msg_type'] == "car_loaded":
            self.car_loaded = True
        
        if json_packet['msg_type'] == "telemetry":
            imgString = json_packet["image"]
            decoded_data = base64.b64decode(imgString)
            np_data = np.fromstring(decoded_data, np.int8)
            self.last_image = cv2.imdecode(np_data, cv2.IMREAD_UNCHANGED)
            #print("got a new image")

    def extract_keys(self, dct, lst):
        ret_dct = {}
        for key in lst:
            if key in dct:
                ret_dct[key] = dct[key]
        return ret_dct

    def send_controls(self, steering, throttle):
        print("sending controls", steering, throttle)
        p = { "msg_type" : "control",
                "steering" : steering.__str__(),
                "throttle" : throttle.__str__(),
                "brake" : "0.0" }
        msg = json.dumps(p)
        self.send(msg)

    def send_config(self, conf):
        self.set_car_config(conf)
        self.set_racer_bio(conf)
        cam_config = self.extract_keys(conf, ["img_w", "img_h", "img_d", "img_enc", "fov", "fish_eye_x", "fish_eye_y", "offset_x", "offset_y", "offset_z", "rot_x"])
        self.send_cam_config(**cam_config)

    def set_car_config(self, conf):
        if "body_style" in conf :
            self.send_car_config(conf["body_style"], conf["body_rgb"], conf["car_name"], conf["font_size"])

    def set_racer_bio(self, conf):
        self.conf = conf
        if "bio" in conf :
            self.send_racer_bio(conf["racer_name"], conf["car_name"], conf["bio"], conf["country"])

    def send_car_config(self, body_style, body_rgb, car_name, font_size):
        """
        # body_style = "donkey" | "bare" | "car01" choice of string
        # body_rgb  = (128, 128, 128) tuple of ints
        # car_name = "string less than 64 char"
        """
        msg = {'msg_type': 'car_config',
            'body_style': body_style,
            'body_r' : body_rgb[0].__str__(),
            'body_g' : body_rgb[1].__str__(),
            'body_b' : body_rgb[2].__str__(),
            'car_name': car_name,
            'font_size' : font_size.__str__() }
        self.blocking_send(msg)
        time.sleep(0.1)

    def send_racer_bio(self, racer_name, car_name, bio, country):
        # body_style = "donkey" | "bare" | "car01" choice of string
        # body_rgb  = (128, 128, 128) tuple of ints
        # car_name = "string less than 64 char"
        msg = {'msg_type': 'racer_info',
            'racer_name': racer_name,
            'car_name' : car_name,
            'bio' : bio,
            'country' : country }
        self.blocking_send(msg)
        time.sleep(0.1)

    def send_cam_config(self, img_w=0, img_h=0, img_d=0, img_enc=0, fov=0, fish_eye_x=0, fish_eye_y=0, offset_x=0, offset_y=0, offset_z=0, rot_x=0):
        """ Camera config
            set any field to Zero to get the default camera setting.
            offset_x moves camera left/right
            offset_y moves camera up/down
            offset_z moves camera forward/back
            rot_x will rotate the camera
            with fish_eye_x/y == 0.0 then you get no distortion
            img_enc can be one of JPG|PNG|TGA
        """
        msg = {"msg_type" : "cam_config",
               "fov" : str(fov),
               "fish_eye_x" : str(fish_eye_x),
               "fish_eye_y" : str(fish_eye_y),
               "img_w" : str(img_w),
               "img_h" : str(img_h),
               "img_d" : str(img_d),
               "img_enc" : str(img_enc),
               "offset_x" : str(offset_x),
               "offset_y" : str(offset_y),
               "offset_z" : str(offset_z),
               "rot_x" : str(rot_x) }
        self.blocking_send(msg)
        time.sleep(0.1)

    def blocking_send(self, p):
        msg = json.dumps(p)
        self.send_now(msg)

    def update(self):
        if self.last_image is not None:
            try:
                steering, throttle = cv_racer(self.last_image, self.steering_icon, self.steering, self.throttle, self.max_throttle, self.x_meter, self.y_meter, self.frame_cols, self.frame_rows, self.fontFace, self.fontScale, self.fontThickness, self.max_steering_angle, self.x_left, self.y_top)
                self.steering = steering
                self.throttle = throttle
                self.send_controls(steering, throttle)
            except:
                import traceback
                traceback.print_exc()
                print("cv_racer error!:")


def race(host, name):

    conf = { "body_style" : "donkey", 
        "body_rgb" : (255, 0, 0),
        "car_name" : name,
        "racer_name" : "naisy_cv",
        "country" : "Japan",
        "bio" : "I am OpenCV.",
        "font_size" : "75"
        }

    # Create client
    client = CVRaceClient(address=(host, PORT), conf=conf)

    # load scene
    msg = '{ "msg_type" : "load_scene", "scene_name" : "mountain_track" }'
    client.send(msg)
    time.sleep(1.0)

    # Car config
    
    client.send(msg)
    time.sleep(0.2)

    try:
        while True:
            client.update()
            time.sleep(0.1)

    except KeyboardInterrupt:
        pass

    client.stop()

if __name__ == '__main__':
    args = docopt(__doc__)
    race(host = args['--host'], name = args['--name'])

