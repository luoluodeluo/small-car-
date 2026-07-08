from machine import Pin, PWM, ADC
import time
import math

# ===================== 编码器类 =====================
class Encoder:
    def __init__(self, pinA, pinB):
        self.pinA = Pin(pinA, Pin.IN, Pin.PULL_UP)
        self.pinB = Pin(pinB, Pin.IN, Pin.PULL_UP)
        self.count = 0
        self.last_count = 0
        self.speed = 0
        self.last_time = time.ticks_ms()
        
        self.pinA.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._irq_handler)
        
    def _irq_handler(self, pin):
        a_val = self.pinA.value()
        b_val = self.pinB.value()
        if a_val == b_val:
            self.count += 1
        else:
            self.count -= 1
    
    def update_speed(self):
        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, self.last_time) / 1000.0
        if dt > 0:
            delta_count = self.count - self.last_count
            self.speed = delta_count / dt
            self.last_count = self.count
            self.last_time = current_time
        return self.speed
    
    def get_count(self): return self.count
    def reset(self):
        self.count = 0
        self.last_count = 0
        self.speed = 0

# ===================== PID控制器【极致防抖参数】 =====================
class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0, output_limits=(-1023, 1023)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.output_limits = output_limits
        self._integral = 0
        self._last_error = 0
        self._last_time = time.ticks_ms()
        self._last_output = 0.0
        self._output_smoothing = 0.22  # 更大平滑，输出无突变
    
    def update(self, measured_value, dt=None):
        if dt is None:
            current_time = time.ticks_ms()
            dt = time.ticks_diff(current_time, self._last_time) / 1000.0
            self._last_time = current_time
        if dt <= 0: dt = 0.005
        error = self.setpoint - measured_value
        p_term = self.kp * error

        # 仅偏移很大时才积分，直线完全关闭积分
        if abs(error) < 1.0:
            self._integral += error * dt
        else:
            self._integral *= 0.65
        self._integral = max(-120, min(120, self._integral))
        i_term = self.ki * self._integral

        derivative = (error - self._last_error) / dt
        d_term = self.kd * derivative
        output = p_term + i_term + d_term

        output = max(self.output_limits[0], min(self.output_limits[1], output))
        output = self._last_output * self._output_smoothing + output * (1 - self._output_smoothing)
        self._last_output = output
        self._last_error = error
        return output
    
    def reset(self):
        self._integral = 0
        self._last_error = 0
        self._last_output = 0.0

# ===================== 光电采样（统一阈值100） =====================
class PhotoelectricSampler:
    def __init__(self):
        self.adc_pins = {'left1':27,'left2':33,'mid':32,'right2':35,'right1':34}
        self.sensor_thresholds = [140, 140, 140, 140, 140]
        self.adc_objects = {}
        self._init_adc()
    def _init_adc(self):
        print("初始化ADC通道...")
        for name,pin in self.adc_pins.items():
            adc = ADC(Pin(pin))
            adc.atten(ADC.ATTN_11DB)
            adc.width(ADC.WIDTH_12BIT)
            self.adc_objects[name] = adc
            print(f"  {name} GPIO{pin} OK")
    def read_all(self):
        res = {}
        for name,adc in self.adc_objects.items():
            val = adc.read()
            res[name] = {"raw":val,"volt":val/4095*3.6}
        return res
    def get_line_position(self):
        sensors = self.read_all()
        names = ['left1','left2','mid','right2','right1']
        binary = []
        for i,n in enumerate(names):
            th = self.sensor_thresholds[i]
            binary.append(1 if sensors[n]["raw"]>th else 0)
        weights = [-2,-1,0,1,2]
        pos,total = 0,0
        for i,v in enumerate(binary):
            if v:
                pos += weights[i]
                total += abs(weights[i])
        pos = pos/total if total>0 else 0
        return pos,binary,sensors

# ===================== 电机控制 =====================
class MotorController:
    def __init__(self, base_speed=650, min_speed=480, max_speed=800):
        self.L1=PWM(Pin(15,0),freq=20000,duty=0)
        self.L2=PWM(Pin(13,0),freq=20000,duty=0)
        self.R1=PWM(Pin(14,0),freq=20000,duty=0)
        self.R2=PWM(Pin(25,0),freq=20000,duty=0)
        self.MAX_SPEED=max_speed
        self.MIN_SPEED=min_speed
        self.BASE_SPEED=base_speed
        self.curL,self.curR = 0.0,0.0
        self.tarL,self.tarR = 0.0,0.0
        self.acceleration = 0.95
        self.encL = Encoder(16,17)
        self.encR = Encoder(18,19)
        self.use_encoder=False
        self.car_stop()
        print(f"电机初始化 min:{min_speed} base:{base_speed} max:{max_speed}")
    def _limit_min(self,sp):
        if sp>0 and sp<self.MIN_SPEED: return self.MIN_SPEED
        if sp<0 and sp>-self.MIN_SPEED: return -self.MIN_SPEED
        return sp
    def _set_motor(self,l,r):
        l = max(-1023, min(1023, l))
        r = max(-1023, min(1023, r))
        l = self._limit_min(max(-self.MAX_SPEED, min(self.MAX_SPEED,l)))
        r = self._limit_min(max(-self.MAX_SPEED, min(self.MAX_SPEED,r)))
        self.tarL,self.tarR = l,r
        self.curL += (self.tarL - self.curL)*self.acceleration
        self.curR += (self.tarR - self.curR)*self.acceleration
        la,ra = int(abs(self.curL)), int(abs(self.curR))
        la = max(0, min(1023, la))
        ra = max(0, min(1023, ra))
        self.L1.duty(la if self.curL>=0 else 0)
        self.L2.duty(la if self.curL<0 else 0)
        self.R1.duty(ra if self.curR>=0 else 0)
        self.R2.duty(ra if self.curR<0 else 0)
    def set_speeds(self,l,r): self._set_motor(l,r)
    def car_stop(self):
        self.L1.duty(0);self.L2.duty(0);self.R1.duty(0);self.R2.duty(0)
        self.curL=self.curR=self.tarL=self.tarR=0.0

# ===================== 循迹逻辑【轮速公式修复防外侧甩弯，PID防抖】 =====================
class LineFollower:
    def __init__(self, base=650, min_sp=480, max_sp=800):
        self.sensor = PhotoelectricSampler()
        self.motor = MotorController(base, min_sp, max_sp)
        # 极致防抖PID
        self.pid = PIDController(kp=1.0, ki=0.008, kd=7.0, output_limits=(-650,650))
        self.no_line_counter = 0
        self.last_pos = 0.0
        self.lastL,self.lastR = float(base),float(base)
        self.running = False
        self.motor.car_stop()

    def get_track_mode(self, bin_arr):
        L1, L2, M, R2, R1 = bin_arr
        mode = "Straight"
        speed_scale = 1.0
        diff = 0
        sharp_corner = False

        # 左右直角 完全不变
        if (L1 and L2 and M) and not R2 and not R1:
            mode = "LeftRightAngle"
            speed_scale = 0.6
            diff = -320
            sharp_corner = True
        elif (M and R2 and R1) and not L1 and not L2:
            mode = "RightRightAngle"
            speed_scale = 0.6
            diff = 320
            sharp_corner = True
        # 单点大弯参数不变
        elif L1 == 1 and L2 == 0 and M == 0 and R2 == 0 and R1 == 0:
            mode = "BigLeft"
            speed_scale = 0.72
            diff = -240
        elif R1 == 1 and L1 == 0 and L2 == 0 and M == 0 and R2 == 0:
            mode = "BigRight"
            speed_scale = 0.72
            diff = 240
        # 小幅单边弯不变
        elif L2 == 1 and L1 == 0 and M == 0 and R2 == 0 and R1 == 0:
            mode = "SlightLeft"
            speed_scale = 0.94
            diff = -130
        elif R2 == 1 and L1 == 0 and L2 == 0 and M == 0 and R1 == 0:
            mode = "SlightRight"
            speed_scale = 0.94
            diff = 130
        # 五路全黑十字
        elif sum(bin_arr) == 5:
            mode = "CrossRoad"
            speed_scale = 1.0
            diff = 0
        # 中间单独直行
        elif M == 1 and L1 == 0 and L2 == 0 and R2 == 0 and R1 == 0:
            mode = "Straight"
            speed_scale = 1.0
            diff = 0
        # 混合曲线不变
        else:
            mode = "MixedLine"
            speed_scale = 0.94
            if self.last_pos < -0.1:
                diff = -70
            elif self.last_pos > 0.1:
                diff = 70
            else:
                diff = 0
        return mode, speed_scale, diff, sharp_corner

    def follow_line(self):
        pos, bin_arr, sens = self.sensor.get_line_position()
        total_bin = sum(bin_arr)
        L1, L2, M, R2, R1 = bin_arr

        # 丢线容错保持480帧，搜寻幅度不变
        if total_bin == 0:
            self.no_line_counter += 1
            if self.no_line_counter > 480:
                self.motor.car_stop()
                return "stop", pos, bin_arr, sens, 0, 0, 0, 0, "Stop", "LostTimeout"
            search_sp = self.motor.MIN_SPEED + 50
            turn_add = 320
            if self.last_pos < -0.3:
                self.motor.set_speeds(search_sp, search_sp + turn_add)
                return "search_left", pos, bin_arr, sens, search_sp, search_sp+turn_add, 0,0,"SearchLeft","Lost_TurnLeft"
            else:
                self.motor.set_speeds(search_sp + turn_add, search_sp)
                return "search_right", pos, bin_arr, sens, search_sp+turn_add, search_sp,0,0,"SearchRight","Lost_TurnRight"

        self.last_pos = pos
        self.no_line_counter = 0
        track_mode, speed_scale, base_diff, sharp = self.get_track_mode(bin_arr)
        corr = self.pid.update(pos)
        corr = max(-400, min(400, corr))

        base_sp = self.motor.BASE_SPEED * speed_scale
        # ========== 修复：弯道固定差值与PID反向，防止甩外侧 ==========
        # 原：L = base_sp + base_diff + corr
        # 现：固定diff与PID修正反向叠加，内侧不会过慢
        L = base_sp + base_diff - corr
        R = base_sp - base_diff + corr

        alpha = 0.86
        L = self.lastL * (1 - alpha) + L * alpha
        R = self.lastR * (1 - alpha) + R * alpha
        self.lastL, self.lastR = L, R

        maxA = self.motor.MAX_SPEED
        L = max(self.motor.MIN_SPEED, min(maxA, L))
        R = max(self.motor.MIN_SPEED, min(maxA, R))

        self.motor.set_speeds(int(L), int(R))
        diff_val = L - R
        act = "FWD"
        if diff_val < -20:
            act = "SL" if diff_val < -60 else "L"
        elif diff_val > 20:
            act = "SR" if diff_val > 60 else "R"
        return act, pos, bin_arr, sens, L, R, 0, corr, act, track_mode

    def run(self, print_int=1):
        print("小车2秒后启动 Ctrl+C停止")
        time.sleep(2)
        self.running=True
        last_print = time.time()
        try:
            while self.running:
                try:
                    act, pos, bin_arr, sens, L, R, clv, corr, actname, info = self.follow_line()
                except ValueError as e:
                    print("【警告】速度超限，重置PID:", e)
                    self.pid.reset()
                    self.motor.set_speeds(self.motor.BASE_SPEED, self.motor.BASE_SPEED)
                    self.no_line_counter = 0
                    continue
                if time.time() - last_print >= print_int:
                    l1,l2,m,r2,r1 = [sens[x]["raw"] for x in ["left1","left2","mid","right2","right1"]]
                    bw = ["B" if v else "W" for v in bin_arr]
                    print(f"\n【状态】{actname} | 赛道:{info} pos={pos:.2f} PID={corr:.1f} L={int(L)} R={int(R)}")
                    print(f"传感器 L1:{l1} L2:{l2} M:{m} R2:{r2} R1:{r1} 黑线:{bw}")
                    last_print = time.time()
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("\n手动停止")
        finally:
            self.running=False
            self.motor.car_stop()
            print("小车已停机")

def main():
    MIN_SPEED = 480
    BASE_SPEED = 650
    MAX_SPEED = 800
    car = LineFollower(base=BASE_SPEED, min_sp=MIN_SPEED, max_sp=MAX_SPEED)
    car.run(print_int=1)

if __name__ == "__main__":
    main()