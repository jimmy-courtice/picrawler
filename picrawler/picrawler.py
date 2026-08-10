from robot_hat import Robot, utils

import os
import time
import math

class Picrawler(Robot):
    """
    Customize motion by editing MoveList in this file — those definitions
    *are* the defaults (stand, sit, forward, …). There is no parallel gait layer.

    smooth_segments controls XYZ midpoints between keyframes in do_step.
    Dense motions (e.g. dance) automatically skip midpoints.
    """
    A = 48
    B = 78
    C = 33
    OFFSET_FILE = os.path.expanduser('~/.config/.picrawler.config')
    # Machine-specific walk steer (same idea as servo cali — not a code default)
    YAW_TRIM_FILE = os.path.expanduser('~/.config/picrawler_yaw_trim')
    PIN_LIST = [9, 10, 11, 3, 4, 5, 0, 1, 2, 6, 7, 8]
    _GAIT_MOTIONS = frozenset({
        "forward", "backward", "turn left", "turn right",
        "turn left angle", "turn right angle",
    })
    _POSE_MOTIONS = frozenset({"sit", "stand"})
    # Above this keyframe count, do_action forces segments=1 (dance, etc.)
    _DENSE_KEYFRAME_THRESHOLD = 40
    # Skip XYZ midpoints when max foot travel is below this (mm)
    _TINY_POSE_DELTA_MM = 3.0

    @classmethod
    def load_yaw_trim(cls):
        """Read persisted yaw trim from YAW_TRIM_FILE (0 if missing/invalid)."""
        try:
            with open(cls.YAW_TRIM_FILE, "r", encoding="utf-8") as f:
                return float(f.read().strip().split()[0])
        except (OSError, ValueError, IndexError):
            return 0.0

    def save_yaw_trim(self, value=None):
        """Persist yaw trim for this robot (default: current self.yaw_trim)."""
        if value is not None:
            self.yaw_trim = float(value)
        path = self.YAW_TRIM_FILE
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            f.write("%s\n" % self.yaw_trim)
        return self.yaw_trim

    def __init__(
        self,
        pin_list=PIN_LIST,
        init_angles=None,
        smooth_segments=2,
        yaw_trim=None,
        yaw_trim_interval=None,
    ):
        """
        :param smooth_segments: XYZ midpoints between keyframes when do_step
            plays a pose list. 1 = original snap, 2 = one midpoint (default).
        :param yaw_trim: left/right steer on forward/backward. None (default)
            loads from ~/.config/picrawler_yaw_trim if present, else 0.
            Pass an explicit number to override for this session only.
            Positive = steer left (if it pulls right). Negative = steer right.
        :param yaw_trim_interval: ignored (kept for old call sites).
        """
        utils.reset_mcu()
        time.sleep(0.2)

        super().__init__(pin_list, db=self.OFFSET_FILE, name='picrawler', init_angles=init_angles)

        self.move_list = self.MoveList()
        self.move_list_add = {
            'my action': None
        }

        # Names only — never pre-evaluate stand/sit (that stuck z_current on sit).
        self.step_list = {
            "stand": "stand",
            "sit": "sit",
        }

        self.smooth_segments = max(1, int(smooth_segments))
        self._segments_override = None
        if yaw_trim is None:
            self.yaw_trim = self.load_yaw_trim()
        else:
            self.yaw_trim = float(yaw_trim)
        self.yaw_trim_interval = yaw_trim_interval  # unused
        self.stand_position = 0
        self.direction = [
            1,1,-1,
            1,1,1,
            1,1,-1,
            1,1,1,
        ]

        self.current_coord = [[60, 0, -30], [60, 0, -30], [60, 0, -30], [60, 0, -30]]
        self.coord_temp = [[60, 0, -30], [60, 0, -30], [60, 0, -30], [60, 0, -30]]
        self.move_list.z_current = self.move_list.Z_DEFAULT
        self.move_list.ready_state = 1

    def coord2polar(self, coord):
        x,y,z = coord
        
        L = math.sqrt(x**2+y**2+z**2)
        if L == 0:
            L = 0.1
        if L < self.C:
            temp = self.C/L
            x = temp * x
            y = temp * y
            z = temp * z           
        elif L > (self.A+self.B+self.C):
            temp = (self.A+self.B+self.C)/L
            x = temp * x
            y = temp * y
            z = temp * z   

        self.coord_temp.append([x,y,z])

        w = math.sqrt(math.pow(x,2) + math.pow(y,2))
        v = w - self.C
        u = math.sqrt(math.pow(z,2) + math.pow(v,2))
        u = max(30, min(91.58, u))
        cos_angle1 = (self.B**2 + self.A**2 - u**2) / (2 * self.B * self.A)
        beta = math.acos(cos_angle1)

        angle1 = math.atan2(z, v)
        angle2 = math.acos((self.A**2 + u**2 - self.B**2)/(2*self.A*u))
        alpha = angle2 + angle1

        gamma = math.atan2(y, x)

        alpha = 90 - alpha / math.pi * 180
        beta = beta / math.pi * 180 - 90
        gamma = -(gamma / math.pi * 180 - 45) 

        return [round(alpha,4), round(beta,4), round(gamma,4)]

    def polar2coord(self, angles):
        alpha, beta, gamma = angles

        L1 = math.sqrt(self.A**2+self.B**2-2*self.A*self.B*math.cos((90+alpha)/180*math.pi))
        angle = math.acos((self.A**2+L1**2-self.B**2)/(2*self.A*L1))*180/math.pi
        angle = 90 - beta - angle
        L = L1*math.cos(angle*math.pi/180) + self.C

        x = L*math.sin((45+gamma)*math.pi/180)
        y = L*math.cos((45+gamma)*math.pi/180)
        z = L1*math.sin(angle*math.pi/180)
    
        return [round(x,4),round(y,4),round(z,4)]

    def limit(self,min,max,x):
        if x > max:
            return max
        elif x < min:
            return min
        else:
            return x

    def limit_angle(self,angles):
        alpha, beta, gamma = angles
        # print('input: %s'%angles)
        # limit 
        limit_flag = False
        # alpha
        temp = self.limit(-90,90,alpha)
        if temp != alpha:
            alpha = temp
            limit_flag = True
        # beta
        temp = self.limit(-10,90,beta)
        if temp != beta:
            beta = temp
            limit_flag = True
        # gamma
        temp = self.limit(-60,60,gamma)
        if temp != gamma:
            gamma = temp
            limit_flag = True
        #return
        # print('output: %s'%[alpha,beta,gamma])
        return limit_flag,[alpha,beta,gamma]

    @staticmethod
    def _smoothstep(t):
        """Ease in/out for segment interpolation (0..1 → 0..1)."""
        return t * t * (3.0 - 2.0 * t)

    def _lerp_point(self, a, b, t):
        return [a[0] + (b[0] - a[0]) * t,
                a[1] + (b[1] - a[1]) * t,
                a[2] + (b[2] - a[2]) * t]

    def _pose_max_delta(self, a, b):
        """Largest foot travel (mm) between two 4-leg poses."""
        best = 0.0
        for i in range(4):
            dx = a[i][0] - b[i][0]
            dy = a[i][1] - b[i][1]
            dz = a[i][2] - b[i][2]
            d = math.sqrt(dx * dx + dy * dy + dz * dz)
            if d > best:
                best = d
        return best

    def _do_step_raw(self, coords, speed=50, israise=False):
        """Single pose → IK → servos (no segment interpolation)."""
        angles_temp = []
        for coord in coords:
            alpha, beta, gamma = self.coord2polar(coord)
            angles_temp.append([beta, alpha, gamma])
        self.coord_temp = list.copy(coords)
        self.set_angle(angles_temp, speed, israise)

    def _mark_standing(self):
        self.move_list.z_current = self.move_list.Z_DEFAULT
        self.move_list.ready_state = 1

    def _return_to_rest(self, speed=50):
        """Square rest (all legs Y=45) after a motion."""
        ml = self.move_list
        self._mark_standing()
        self.do_step(ml._square_pose(ml.Z_DEFAULT), speed=speed)
        self.stand_position = 0
        ml.stand_position = 0

    def _bias_gait_pose(self, pose, travel_motion):
        """Asymmetric stride so forward/backward steers without a turn wiggle.

        Leg order: RF, LF, LR, RR. Positive yaw_trim lengthens right-side Y
        and shortens left-side Y (steer left). Backward inverts.
        """
        b = self.yaw_trim
        if not b:
            return pose
        if travel_motion == "backward":
            b = -b
        # Gain: sweet spot between “still drifts right” and “hard left”
        b *= 2.0
        out = []
        for i, (x, y, z) in enumerate(pose):
            side = 1.0 if i in (0, 3) else -1.0
            if y > 0:
                y = y + side * b
            out.append([x, y, z])
        return out

    def do_action(self, motion_name, step=1, speed=50):
        spaced = motion_name.replace("_", " ")
        under = motion_name.replace(" ", "_")

        # sit / stand — same path as do_step('sit'/'stand')
        if under in self._POSE_MOTIONS or spaced in self._POSE_MOTIONS:
            pose_name = "sit" if under == "sit" or spaced == "sit" else "stand"
            self.do_step(pose_name, speed=speed)
            return

        is_gait = spaced in self._GAIT_MOTIONS
        ml = self.move_list

        try:
            if is_gait:
                self._mark_standing()
                self.do_step(ml._diagonal_pose(ml.Z_DEFAULT), speed=speed)

            for _ in range(step):
                ml.stand_position = self.stand_position
                if is_gait:
                    self.stand_position = self.stand_position + 1 & 1
                self._mark_standing()
                # MoveList.__getitem__ maps spaces → underscores
                action = ml[spaced if spaced in self._GAIT_MOTIONS else motion_name]
                override = None
                if len(action) >= self._DENSE_KEYFRAME_THRESHOLD:
                    override = 1
                prev = self._segments_override
                self._segments_override = override
                try:
                    for pose in action:
                        if spaced in ("forward", "backward") and self.yaw_trim:
                            pose = self._bias_gait_pose(pose, spaced)
                        self.do_step(pose, speed=speed)
                finally:
                    self._segments_override = prev

            # All non-pose motions finish in square rest
            self._return_to_rest(speed=speed)
        except AttributeError:
            try:
                for _ in range(step):
                    action_add = self.move_list_add[motion_name]
                    if action_add is None:
                        raise KeyError(motion_name)
                    override = 1 if len(action_add) >= self._DENSE_KEYFRAME_THRESHOLD else None
                    prev = self._segments_override
                    self._segments_override = override
                    try:
                        for pose in action_add:
                            self.do_step(pose, speed=speed)
                    finally:
                        self._segments_override = prev
                self._return_to_rest(speed=speed)
            except KeyError:
                print("No such action")

    def set_angle(self, angles_list, speed=50, israise=False):
        translate_list = []
        results = []
        for angles in angles_list:
            result, angles = self.limit_angle(angles)
            translate_list += angles
            results.append(result)
        
        if True in results:
            if israise == True:
                raise ValueError('\033[1;35mCoordinates out of controllable range.\033[0m')
            else:
                try:
                    # print('\033[1;35mCoordinates out of controllable range.\033[0m')
                    coords = []
                    # Calculate coordinates 
                    for i in range(4):
                        coords.append(self.polar2coord([translate_list[i*3],translate_list[i*3+1],translate_list[i*3+2]]))
                    self.current_coord = list.copy(coords)
                except Exception as e:
                    print('re : %s'%e)
        else:
            self.current_coord = list.copy(self.coord_temp)

        self.servo_move(translate_list, speed)  
        return list.copy(translate_list)

    def do_step(self, _step, speed=50, israise=False):
        if isinstance(_step, str):
            name = _step.replace(" ", "_")
            if name in ("stand", "sit"):
                frames = self.move_list[name]
                for one_step in frames:
                    self.do_step(one_step, speed=speed, israise=israise)
                if name == "stand":
                    self._mark_standing()
                    self.stand_position = 0
                    self.move_list.stand_position = 0
                else:
                    self.move_list.z_current = self.move_list.Z_UP
            else:
                print("The name of gait is not in the default gait dictionary")
        elif isinstance(_step, list):
            segs = self.smooth_segments
            if self._segments_override is not None:
                segs = self._segments_override
            start = self.current_step_all_leg_value()
            end = [list(map(float, leg)) for leg in _step]
            if segs <= 1 or self._pose_max_delta(start, end) < self._TINY_POSE_DELTA_MM:
                self._do_step_raw(end, speed=speed, israise=israise)
                return
            for i in range(1, segs + 1):
                t = self._smoothstep(i / float(segs))
                mid = [self._lerp_point(start[j], end[j], t) for j in range(4)]
                self._do_step_raw(mid, speed=speed, israise=israise)
        else:
            print("The \"_step\" parameter is wrong.")
            return


    def current_step_all_leg_angle(self):
        return list.copy(self.servo_positions)

    def add_action(self,action_name, action_list):
        self.move_list_add[action_name] = action_list


    def cali_helper_web(self, leg, pos, enter):
        step=0.2
        cali_position = []
        cali_coord = [[60, 0, -30], [60, 0, -30], [60, 0, -30], [60, 0, -30]]

        for coord in cali_coord: # each servo motion
            alpha, beta, gamma = self.coord2polar(coord)
            cali_position += [beta, alpha, gamma]

        cali_position = [cali_position[i] + self.offset[i] for i in range(12)]
        # print("cali_position:",cali_position)

        positive_list = [
            [1, -1, -1, 1, 1, -1],
            [1, -1, 1, -1, 1, -1],
            [-1, 1, 1, -1, 1, -1],
            [-1, 1, -1, 1, 1, -1],
        ]
        
        offset = list.copy(self.offset)
        leg = leg - 1
        if pos == 'up':
            self.current_coord[leg][1] += step * positive_list[leg][0]
        elif pos == 'down':
            self.current_coord[leg][1] += step * positive_list[leg][1]
        elif pos == 'left':
            self.current_coord[leg][0] += step * positive_list[leg][2]
        elif pos == 'right':
            self.current_coord[leg][0] += step * positive_list[leg][3]
        elif pos == 'high':
            self.current_coord[leg][2] += step * positive_list[leg][4]
        elif pos == 'low':
            self.current_coord[leg][2] += step * positive_list[leg][5]
        
        for coord in self.current_coord:
            coord[0] = max(40, min(80, coord[0]))
            coord[1] = max(-20, min(20, coord[1]))
            coord[2] = max(-50, min(-10, coord[2]))
        self.do_step(self.current_coord, speed=100)
        current_position = list.copy(self.do_step(self.current_coord, speed=100))
        # print('current_position: %s'%current_position)
        if enter == 1:
            tmp = [current_position[i] - cali_position[i] + offset[i] for i in range(len(current_position))]
            offset[leg*3:(leg + 1)*3] = tmp[leg*3:(leg + 1)*3]
            self.current_coord[leg] = [60, 0, -30]
            self.set_offset(offset)
            self.do_step(self.current_coord, speed=100)


    class MoveList(dict):
        """
        === Customize poses/gaits HERE (these replace stock defaults) ===

        Leg order each frame: [right front, left front, left rear, right rear]
        Each leg is [x, y, z] in mm (local frame).

        stand  — default rest: ALL legs at (45, 45, Z_DEFAULT)
        sit    — park: all legs at (45, 45, Z_UP)
        forward / backward / turn_* — walk cycles (do_action eases through
            diagonal gait stance, then returns to square rest)
        """
        LENGTH_SIDE = 77
        X_DEFAULT = 45
        X_TURN = 70
        X_START = 0
        Y_DEFAULT = 45
        Y_TURN = 130
        Y_WAVE =120
        Y_START = 0 
        Z_DEFAULT = -50
        Z_UP = -30
        Z_WAVE = 60
        Z_TURN = -40
        Z_PUSH = -76
         
        # temp length
        TEMP_A = math.sqrt(pow(2 * X_DEFAULT + LENGTH_SIDE, 2) + pow(Y_DEFAULT, 2))
        TEMP_B = 2 * (Y_START + Y_DEFAULT) + LENGTH_SIDE
        TEMP_C = math.sqrt(pow(2 * X_DEFAULT + LENGTH_SIDE, 2) + pow(2 * Y_START + Y_DEFAULT + LENGTH_SIDE, 2))
        TEMP_ALPHA = math.acos((pow(TEMP_A, 2) + pow(TEMP_B, 2) - pow(TEMP_C, 2)) / 2 / TEMP_A / TEMP_B)
        # site for turn
        TURN_X1 = (TEMP_A - LENGTH_SIDE) / 2
        TURN_Y1 = Y_START + Y_DEFAULT / 2
        TURN_X0 = TURN_X1 - TEMP_B * math.cos(TEMP_ALPHA)
        TURN_Y0 = TEMP_B * math.sin(TEMP_ALPHA) - TURN_Y1 - LENGTH_SIDE

        def __init__(self, *args, **kwargs):
            dict.__init__(self, *args, **kwargs)
            self.z_current = self.Z_UP
            self.stand_position = 0
            self.recovery_step = []
            self.ready_state = 0
            self.angle = 30
   
        def __getitem__(self, item):
            return eval("self.%s"%item.replace(" ", "_"))
        
        def turn_angle_coord(self, angle):
            a = math.atan(self.Y_DEFAULT/(self.X_DEFAULT+self.LENGTH_SIDE/2))
            angle1 = a/math.pi*180
            r1 = math.sqrt(pow(self.Y_DEFAULT,2)+ pow(self.X_DEFAULT+ self.LENGTH_SIDE/2, 2))
            x1 = r1* math.cos((angle1-angle)* math.pi/180)- self.LENGTH_SIDE/2
            y1 = r1* math.sin((angle1-angle)* math.pi/180)
            # print(x1,y1)
            
            x2 = (self.X_DEFAULT+ self.LENGTH_SIDE/2)* math.cos(angle*math.pi/180)- self.LENGTH_SIDE/2
            y2 = (self.X_DEFAULT+ self.LENGTH_SIDE/2)* math.sin(angle*math.pi/180)
            # print(x2,y2)
            
            b = math.atan((self.X_DEFAULT+self.LENGTH_SIDE/2)/(self.Y_DEFAULT+ self.LENGTH_SIDE))
            angle2 = b/math.pi*180
            r2 = math.sqrt(pow(self.X_DEFAULT+ self.LENGTH_SIDE/2, 2)+ pow(self.Y_DEFAULT+ self.LENGTH_SIDE,2))
            x3 = r2*math.sin((angle2-angle)* math.pi/180) - self.LENGTH_SIDE/2
            y3 = r2*math.cos((angle2-angle)*math.pi/180)- self.LENGTH_SIDE

            x3 += 10
            # print(x3,y3)
            return [x1,y1,x2,y2,x3,y3]
        
        # 装饰器封装函数,判断是否站立
        def check_stand(func):
            def wrapper(self):
                _action = []
                if not self.is_stand():
                    _action += self.stand
                _action += func(self)
                return _action
            return wrapper
        
        # 装饰器封装函数，装饰器简化步态的0，1两种状态转化，状态0为2，3脚y轴为0，状态1为1，4脚y轴为0 mode为2种转化方式，mode0为1，2交换3，4交换，mode1为1，3交换2，4交换
        def normal_action(mode):
            def wrapper1(func):
                def wrapper2(self):
                    _action = []
                    if self.stand_position == 0:
                        _action += func(self)
                    else:
                        temp = func(self)
                        new_step = []
                        for step in temp:
                            if mode == 0:
                                new_step = [step[1], step[0], step[3], step[2]]
                            elif mode == 1:
                                new_step = [step[2], step[3], step[0], step[1]]
                            _action += [new_step]
                    return _action
                return wrapper2
            return wrapper1
        
        def _square_pose(self, z):
            """Rest stance: all four legs at (X=45, Y=45, z)."""
            x, y = self.X_DEFAULT, self.Y_DEFAULT
            return [[x, y, z], [x, y, z], [x, y, z], [x, y, z]]

        def _diagonal_pose(self, z):
            """Gait footprint: RF/RR long (Y=45), LF/LR short (Y=0)."""
            return [
                [self.X_DEFAULT, self.Y_DEFAULT, z],
                [self.X_DEFAULT, self.Y_START, z],
                [self.X_DEFAULT, self.Y_START, z],
                [self.X_DEFAULT, self.Y_DEFAULT, z],
            ]

        @property
        @normal_action(0)
        def sit(self):
            """Park: all legs at Y=45, raised (Z_UP)."""
            self.z_current = self.Z_UP
            return [self._square_pose(self.z_current)]

        @property
        @normal_action(0)
        def stand(self):
            """Default rest: all legs at (45, 45, Z_DEFAULT)."""
            _stand = []
            if self.ready_state == 0:
                _stand += self.ready
            self.z_current = self.Z_DEFAULT
            z = self.z_current
            _stand += [
                self._square_pose(z * 0.35),
                self._square_pose(z * 0.55),
                self._square_pose(z * 0.75),
                self._square_pose(z * 0.9),
                self._square_pose(z),
            ]
            return _stand
        
        @property
        def ready(self):
            _ready = [self._square_pose(self.z_current)]
            self.ready_state = 1
            return _ready
          

        def is_sit(self):
            return self.z_current == self.Z_UP
            
        def is_stand(self):
            tmp = self.z_current == self.Z_DEFAULT
            # print("is stand? %s"%tmp)
            return tmp
        
        @property
        @check_stand
        @normal_action(0)
        def forward(self):
            return [
                [[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_TURN, self.Y_START,self.Z_UP],[self.X_DEFAULT, self.Y_START, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current]],
                [[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT*2,self.Z_UP],[self.X_DEFAULT, self.Y_START, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current]],
                [[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT*2,self.z_current],[self.X_DEFAULT, self.Y_START, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current]],
                [[self.X_DEFAULT, self.Y_START, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT,self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT*2, self.z_current]],
                
                [[self.X_DEFAULT, self.Y_START, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT,self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT*2, self.Z_UP]],
                [[self.X_DEFAULT, self.Y_START, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT,self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_TURN, self.Y_START, self.Z_UP]],
                [[self.X_DEFAULT, self.Y_START, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT,self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_START, self.z_current]],
            ]
        
        @property
        @check_stand
        @normal_action(0)
        def backward(self):
            return [
                [[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_START,self.z_current],[self.X_TURN, self.Y_START, self.Z_UP],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current]],
                [[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_START,self.z_current],[self.X_DEFAULT, self.Y_DEFAULT*2, self.Z_UP],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current]],
                [[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_START,self.z_current],[self.X_DEFAULT, self.Y_DEFAULT*2, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current]],
                [[self.X_DEFAULT, self.Y_DEFAULT*2, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT,self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_START, self.z_current]],
                [[self.X_DEFAULT, self.Y_DEFAULT*2, self.Z_UP],[self.X_DEFAULT, self.Y_DEFAULT,self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_START, self.z_current]],
                [[self.X_TURN, self.Y_START, self.Z_UP],[self.X_DEFAULT, self.Y_DEFAULT,self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_START, self.z_current]],
                [[self.X_DEFAULT, self.Y_START, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT,self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_START, self.z_current]],
            ]
        
       
        @property
        @check_stand
        @normal_action(1)
        def turn_left(self):
            return [
                [[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_START,self.z_current],[self.X_TURN, self.Y_START, self.Z_UP],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current]],
                [[self.TURN_X1, self.TURN_Y1, self.z_current],[self.TURN_X1, self.TURN_Y1, self.z_current],[self.TURN_X0, self.TURN_Y0, self.Z_UP],[self.TURN_X0, self.TURN_Y0, self.z_current]],
                [[self.TURN_X1, self.TURN_Y1, self.z_current],[self.TURN_X1, self.TURN_Y1, self.z_current],[self.TURN_X0, self.TURN_Y0, self.z_current],[self.TURN_X0, self.TURN_Y0, self.z_current]],
                
                [[self.TURN_X1, self.TURN_Y1, self.z_current],[self.TURN_X1, self.TURN_Y1, self.z_current],[self.TURN_X0, self.TURN_Y0, self.z_current],[self.TURN_X0, self.TURN_Y0, self.Z_UP]],
                [[self.X_DEFAULT, self.Y_START, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_TURN, self.Y_START, self.Z_UP]],
                [[self.X_DEFAULT, self.Y_START, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_START, self.z_current]],
            ]

        @property
        @check_stand
        @normal_action(1)
        def turn_right(self):
            return [
                [[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_TURN, self.Y_START,self.Z_UP],[self.X_DEFAULT, self.Y_START, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current]],
                [[self.TURN_X0, self.TURN_Y0, self.z_current],[self.TURN_X0, self.TURN_Y0, self.Z_UP],[self.TURN_X1, self.TURN_Y1, self.z_current],[self.TURN_X1, self.TURN_X1, self.z_current]],
                [[self.TURN_X0, self.TURN_Y0, self.z_current],[self.TURN_X0, self.TURN_Y0, self.z_current],[self.TURN_X1, self.TURN_Y1, self.z_current],[self.TURN_X1, self.TURN_X1, self.z_current]],
                [[self.TURN_X0, self.TURN_Y0, self.Z_UP],[self.TURN_X0, self.TURN_Y0, self.z_current],[self.TURN_X1, self.TURN_Y1, self.z_current],[self.TURN_X1, self.TURN_X1, self.z_current]],
                [[self.X_TURN, self.Y_START, self.Z_UP],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_START, self.z_current]],
                [[self.X_DEFAULT, self.Y_START, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_START, self.z_current]],
                
            ]
        
        @property
        def push_up(self):
            _push_up = []
            if not self.is_sit():
                _push_up += self.sit
            _push_up += [
                [[self.X_TURN, self.Y_START, self.Z_TURN],[self.X_TURN, self.Y_START, self.Z_TURN],[self.X_START, self.Y_TURN, self.Z_TURN],[self.X_START, self.Y_TURN,self.Z_TURN]],
                [[self.X_TURN, self.Y_START, self.Z_PUSH],[self.X_TURN, self.Y_START, self.Z_PUSH],[self.X_START, self.Y_TURN, self.Z_TURN],[self.X_START, self.Y_TURN,self.Z_TURN]],
                [[self.X_TURN, self.Y_START, self.Z_TURN],[self.X_TURN, self.Y_START, self.Z_TURN],[self.X_START, self.Y_TURN, self.Z_TURN],[self.X_START, self.Y_TURN,self.Z_TURN]],
                [[self.X_TURN, self.Y_START, self.Z_PUSH],[self.X_TURN, self.Y_START, self.Z_PUSH],[self.X_START, self.Y_TURN, self.Z_TURN],[self.X_START, self.Y_TURN,self.Z_TURN]],
                [[self.X_TURN, self.Y_START, self.Z_TURN],[self.X_TURN, self.Y_START, self.Z_TURN],[self.X_START, self.Y_TURN, self.Z_TURN],[self.X_START, self.Y_TURN,self.Z_TURN]],
                [[self.X_TURN, self.Y_START, self.Z_PUSH],[self.X_TURN, self.Y_START, self.Z_PUSH],[self.X_START, self.Y_TURN, self.Z_TURN],[self.X_START, self.Y_TURN,self.Z_TURN]],
                [[self.X_TURN, self.Y_START, self.Z_TURN],[self.X_TURN, self.Y_START, self.Z_TURN],[self.X_START, self.Y_TURN, self.Z_TURN],[self.X_START, self.Y_TURN,self.Z_TURN]],
            ]
            if self.stand_position == 0:
                _push_up.append([[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_TURN, self.Y_START,self.z_current],[self.X_TURN, self.Y_START, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current]])
            else:
                _push_up.append([[self.X_TURN, self.Y_START,self.z_current], [self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_TURN, self.Y_START, self.z_current]])
            return _push_up
        
        @property
        @check_stand
        @normal_action(0)
        def wave(self):
            return [
                [[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_TURN, self.Y_START,self.Z_UP],[self.X_DEFAULT, self.Y_START, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current]],
                [[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_START, self.Y_WAVE,self.Z_WAVE],[self.X_DEFAULT, self.Y_START, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current]],
                [[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_START, self.Y_WAVE,self.Z_UP],[self.X_DEFAULT, self.Y_START, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current]],
                [[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_START, self.Y_WAVE,self.Z_WAVE],[self.X_DEFAULT, self.Y_START, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current]],
                [[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_START, self.Y_WAVE,self.Z_UP],[self.X_DEFAULT, self.Y_START, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current]],
                [[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_START, self.Y_WAVE,self.Z_WAVE],[self.X_DEFAULT, self.Y_START, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current]],
                [[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_START, self.Y_WAVE,self.Z_UP],[self.X_DEFAULT, self.Y_START, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current]],
                
                [[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_TURN, self.Y_START,self.Z_UP],[self.X_DEFAULT, self.Y_START, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current]],
                [[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_START,self.z_current],[self.X_DEFAULT, self.Y_START, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current]],
            ]
        
        @property
        @check_stand
        @normal_action(1)
        def look_left(self):
            li = self.turn_angle_coord(self.angle)
            temp_x1 = li[0:2]
            temp_x1.append(self.z_current)
            temp_x2 = li[2:4]
            temp_x2.append(self.z_current)
            temp_x3 = li[4:6]
            temp_x3.append(self.z_current)
            return [
                [[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_START,self.z_current],[self.X_TURN, self.Y_START, self.Z_UP],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current]],
                [temp_x1, temp_x2,[self.X_TURN, self.Y_START, self.Z_UP],temp_x3]
            ]
            
        @property
        @check_stand
        @normal_action(1)
        def look_right(self):
            li = self.turn_angle_coord(self.angle)
            temp_x1 = li[0:2]
            temp_x1.append(self.z_current)
            temp_x2 = li[2:4]
            temp_x2.append(self.z_current)
            temp_x3 = li[4:6]
            temp_x3.append(self.z_current)
            return [
                [
                    [self.X_DEFAULT, self.Y_DEFAULT, self.z_current],
                    [self.X_TURN, self.Y_START,self.Z_UP],
                    [self.X_DEFAULT, self.Y_START, self.z_current],
                    [self.X_DEFAULT, self.Y_DEFAULT, self.z_current]
                ],
                [temp_x3, [self.X_TURN, self.Y_START, self.Z_UP], temp_x2, temp_x1]
            ]
        
        @property
        @check_stand
        @normal_action(1)
        def turn_left_angle(self):
            li = self.turn_angle_coord(self.angle)
            temp_x1 = li[0]
            temp_y1 = li[1]
            temp_x2 = li[2]
            temp_y2 = li[3]
            temp_x3 = li[4]
            temp_y3 = li[5]
            return [
                [[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_START,self.z_current],[self.X_TURN, self.Y_START, self.Z_UP],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current]],
                [[temp_x1, temp_y1, self.z_current], [temp_x2, temp_y2, self.z_current],[self.X_TURN, self.Y_START, self.Z_UP],[temp_x3, temp_y3, self.z_current]],
                [[temp_x1, temp_y1, self.z_current], [temp_x2, temp_y2, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[temp_x3, temp_y3, self.z_current]],
                [[temp_x1, temp_y1, self.z_current], [temp_x2, temp_y2, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[temp_x3, temp_y3, self.Z_UP]],
                [[temp_x1, temp_y1, self.z_current], [temp_x2, temp_y2, self.z_current],[self.X_TURN, self.Y_DEFAULT, self.z_current],[self.X_TURN, self.Y_START, self.Z_UP]],
                [[temp_x1, temp_y1, self.z_current], [temp_x2, temp_y2, self.z_current],[self.X_TURN, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_START, self.z_current]]
            ]
            
        @property
        @check_stand
        @normal_action(1)
        def turn_right_angle(self):
            li = self.turn_angle_coord(self.angle)
            temp_x1 = li[0]
            temp_y1 = li[1]
            temp_x2 = li[2]
            temp_y2 = li[3]
            temp_x3 = li[4]
            temp_y3 = li[5]
            return [
                [[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_TURN, self.Y_START,self.Z_UP],[self.X_DEFAULT, self.Y_START, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current]],
                [[temp_x3,temp_y3, self.z_current], [self.X_TURN, self.Y_START, self.Z_UP], [temp_x2, temp_y2, self.z_current], [temp_x1, temp_y1, self.z_current]],
                [[temp_x3,temp_y3, self.z_current], [self.X_DEFAULT, self.Y_DEFAULT, self.z_current], [temp_x2, temp_y2, self.z_current], [temp_x1, temp_y1, self.z_current]],
                [[temp_x3,temp_y3, self.Z_UP], [self.X_DEFAULT, self.Y_DEFAULT, self.z_current], [temp_x2, temp_y2, self.z_current], [temp_x1, temp_y1, self.z_current]],
                [[self.X_TURN, self.Y_START, self.Z_UP], [self.X_DEFAULT, self.Y_DEFAULT, self.z_current], [temp_x2, temp_y2, self.z_current], [temp_x1, temp_y1, self.z_current]],
                [[self.X_DEFAULT, self.Y_START, self.z_current], [self.X_DEFAULT, self.Y_DEFAULT, self.z_current], [temp_x2, temp_y2, self.z_current], [temp_x1, temp_y1, self.z_current]],
            ]
            
        
        @property
        @check_stand
        @normal_action(0)
        def look_up(self):
            return [
                [[self.X_DEFAULT, self.Y_DEFAULT, self.Z_DEFAULT],[self.X_DEFAULT, self.Y_START,self.Z_DEFAULT],[self.X_TURN, self.Y_START, self.Z_UP],[self.X_DEFAULT, self.Y_DEFAULT, self.Z_UP]],
            ]
            
        @property
        @check_stand
        @normal_action(0)
        def look_down(self):
            return [
                [[self.X_DEFAULT, self.Y_DEFAULT, self.Z_UP],[self.X_TURN, self.Y_START,self.Z_UP],[self.X_DEFAULT, self.Y_START, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current]],
            ]
        
        def rotate_body_absolute_x(self, degree_x):
            degree_x = degree_x * math.pi / 180
            dz = (self.LENGTH_SIDE / 2 + self.Y_DEFAULT) * math.sin(degree_x)
            dy = (self.LENGTH_SIDE / 2 + self.Y_DEFAULT) * (1 - math.cos(degree_x))
            return [[self.X_DEFAULT, self.Y_DEFAULT - dy, self.Z_DEFAULT + dz],[self.X_DEFAULT, self.Y_DEFAULT - dy, self.Z_DEFAULT - dz],[self.X_DEFAULT, self.Y_DEFAULT - dy, self.Z_DEFAULT - dz],[self.X_DEFAULT, self.Y_DEFAULT - dy, self.Z_DEFAULT + dz]]
        
        
        def rotate_body_absolute_y(self, degree_y):
            degree_y = degree_y * math.pi / 180
            dz = (self.LENGTH_SIDE / 2 + self.X_DEFAULT) * math.sin(degree_y)
            dx = (self.LENGTH_SIDE / 2 + self.X_DEFAULT) * (1 - math.cos(degree_y))
            # print("dz = %d"%dz)
            # print("dx = %d"%dx)
            return [[self.X_DEFAULT- dx, self.Y_DEFAULT, self.Z_DEFAULT + dz], [self.X_DEFAULT- dx, self.Y_DEFAULT, self.Z_DEFAULT + dz],[self.X_DEFAULT- dx, self.Y_DEFAULT, self.Z_DEFAULT - dz],[self.X_DEFAULT- dx, self.Y_DEFAULT, self.Z_DEFAULT - dz]]
        
        
        def  move_body_absolute(self, x, y, z):
            return [[self.X_DEFAULT - x,self.Y_DEFAULT - y,self.Z_TURN - z],[self.X_DEFAULT + x,self.Y_DEFAULT - y,self.Z_TURN - z],[self.X_DEFAULT + x,self.Y_DEFAULT + y,self.Z_TURN - z],[self.X_DEFAULT - x,self.Y_DEFAULT + y,self.Z_TURN - z]]
        
        
        def to_rad(self, deg):
            return deg * math.pi / 180
        
        @property
        def dance(self):
            _dance = []
            if not self.is_sit():
                _dance += self.sit
            _dance += [
                [[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current],[self.X_DEFAULT, self.Y_DEFAULT, self.z_current]],
            ]
            for i in range(0, 360, 5):
                _dance.append(self.move_body_absolute(40 * math.sin(self.to_rad(i)), 40 * math.cos(self.to_rad(i)), 0))
            for i in range(360, 0, -5):
                _dance.append(self.move_body_absolute(40 * math.sin(self.to_rad(i)), 40 * math.cos(self.to_rad(i)), 0))
            _dance.append(self.rotate_body_absolute_x(-20))
            _dance.append(self.rotate_body_absolute_x(20))
            _dance.append(self.move_body_absolute(0, 0, 0))
            _dance.append(self.rotate_body_absolute_y(-20))
            _dance.append(self.rotate_body_absolute_y(20))
            for j in range(0, 3):
                for i in range(0, 360, 3):
                    _dance.append(self.move_body_absolute(40 * math.sin(self.to_rad(i)), 40 * math.cos(self.to_rad(i)), (i / 360.0 + j) * 15))
            for j in range(3, 0, -1):
                for i in range(0, 360, 3):
                    _dance.append(self.move_body_absolute(40 * math.sin(self.to_rad(i)), 40 * math.cos(self.to_rad(i)), ((360 - i) / 360.0 + j - 1) * 15))
            _dance.append(self.move_body_absolute(0, 0, 0))
            return _dance



    def do_single_leg(self,leg,coodinate=[50,50,-33],speed=50):
        target_coord = self.current_step_all_leg_value()
        target_coord[leg] = coodinate
        self.do_step(target_coord,speed)
 

    def current_step_leg_value(self,leg):
        return list.copy(self.current_coord[leg])
        
    def current_step_all_leg_value(self):
        return list.copy(self.current_coord)

    def mix_step(self,basic_step,leg,coodinate=[50,50,-33]):
        # Pay attention to adding list(), otherwise the address pointer is returned
        new_step = list(basic_step)
        new_step[leg] = coodinate
        return list(new_step)

  
    
   
    
   