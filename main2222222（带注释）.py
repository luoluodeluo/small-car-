from machine import Pin, PWM, ADC
import time
import math

# ===================== 正交编码器类：读取电机脉冲、计算转速 =====================
class Encoder:
    def __init__(self, pinA, pinB):
        # AB相引脚，开启内部上拉输入
        self.pinA = Pin(pinA, Pin.IN, Pin.PULL_UP)
        self.pinB = Pin(pinB, Pin.IN, Pin.PULL_UP)
        self.count = 0                # 编码器总脉冲计数
        self.last_count = 0           # 上一次测速时的脉冲值
        self.speed = 0                # 实时转速（脉冲/秒）
        self.last_time = time.ticks_ms()  # 上次测速时间戳
        
        # A相上升/下降沿触发外部中断，用于正交解码
        self.pinA.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._irq_handler)
    
    # 编码器中断回调函数，简易AB相正交解码
    def _irq_handler(self, pin):
        a_val = self.pinA.value()
        b_val = self.pinB.value()
        # AB电平相同：正转，计数+1；不同：反转，计数-1
        if a_val == b_val:
            self.count += 1
        else:
            self.count -= 1
    
    # 更新并返回当前轮速（脉冲每秒）
    def update_speed(self):
        current_time = time.ticks_ms()
        # 计算与上次测速的时间差，转换为秒
        dt = time.ticks_diff(current_time, self.last_time) / 1000.0
        if dt > 0:
            delta_count = self.count - self.last_count
            self.speed = delta_count / dt
            self.last_count = self.count
            self.last_time = current_time
        return self.speed
    
    # 获取当前总脉冲计数
    def get_count(self): 
        return self.count
    
    # 重置编码器计数、速度缓存
    def reset(self):
        self.count = 0
        self.last_count = 0
        self.speed = 0

# ===================== PID控制器：黑线位置闭环纠偏，带防抖优化 =====================
class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0, output_limits=(-1023, 1023)):
        self.kp = kp                  # 比例系数：快速修正偏离
        self.ki = ki                  # 积分系数：消除直道静态偏移
        self.kd = kd                  # 微分系数：抑制抖动、缓和过弯冲击
        self.setpoint = setpoint      # 目标值：黑线居中时pos=0
        self.output_limits = output_limits  # PID输出上下限
        
        self._integral = 0            # 积分累加缓存
        self._last_error = 0          # 上一帧偏差，用于微分项计算
        self._last_time = time.ticks_ms()
        self._last_output = 0.0       # 上一轮PID输出，用于平滑滤波
        self._output_smoothing = 0.22 # 输出一阶平滑系数，越大防抖越强
    
    # PID核心计算函数，返回修正量
    def update(self, measured_value, dt=None):
        # 自动计算帧间隔时间dt（秒）
        if dt is None:
            current_time = time.ticks_ms()
            dt = time.ticks_diff(current_time, self._last_time) / 1000.0
            self._last_time = current_time
        # 防止时间差为0导致除零报错
        if dt <= 0: 
            dt = 0.005
        
        error = self.setpoint - measured_value  # 计算当前偏差
        p_term = self.kp * error                # 比例项
        
        # 分段积分策略：偏差小时正常积分，大偏差衰减积分防饱和甩尾
        if abs(error) < 1.0:
            self._integral += error * dt
        else:
            self._integral *= 0.65
        # 积分限幅，防止积分累积过大
        self._integral = max(-120, min(120, self._integral))
        i_term = self.ki * self._integral       # 积分项
        
        # 微分项：偏差变化率，抑制震荡
        derivative = (error - self._last_error) / dt
        d_term = self.kd * derivative
        
        # PID总输出
        output = p_term + i_term + d_term
        # 硬限幅输出，防止超出电机可调范围
        output = max(self.output_limits[0], min(self.output_limits[1], output))
        # 一阶低通平滑输出，消除电机突变抖动
        output = self._last_output * self._output_smoothing + output * (1 - self._output_smoothing)
        
        self._last_output = output
        self._last_error = error
        return output
    
    # 清空PID缓存，异常时重置使用
    def reset(self):
        self._integral = 0
        self._last_error = 0
        self._last_output = 0.0

# ===================== 五路红外灰度传感器采集类 =====================
class PhotoelectricSampler:
    def __init__(self):
        # 五路传感器对应GPIO引脚：左1、左2、中间、右2、右1
        self.adc_pins = {'left1':27,'left2':33,'mid':32,'right2':35,'right1':34}
        # 五路传感器统一黑白阈值：AD值大于阈值判定为黑线
        self.sensor_thresholds = [140, 140, 140, 140, 140]
        self.adc_objects = {}  # 存放每个通道ADC实例
        self._init_adc()
    
    # 初始化所有ADC通道
    def _init_adc(self):
        print("初始化ADC通道...")
        for name,pin in self.adc_pins.items():
            adc = ADC(Pin(pin))
            adc.atten(ADC.ATTN_11DB)    # 电压量程0~3.6V，适配红外模块
            adc.width(ADC.WIDTH_12BIT)  # 12位ADC，数值范围0~4095
            self.adc_objects[name] = adc
            print(f"  {name} GPIO{pin} OK")
    
    # 一次性读取五路传感器原始AD值与换算电压
    def read_all(self):
        res = {}
        for name,adc in self.adc_objects.items():
            val = adc.read()
            res[name] = {"raw":val,"volt":val/4095*3.6}
        return res
    
    # 计算黑线中心位置pos，返回位置、黑白二值数组、原始传感器数据
    def get_line_position(self):
        sensors = self.read_all()
        names = ['left1','left2','mid','right2','right1']
        binary = []
        # 根据阈值生成黑白二值数组：1=黑线，0=白纸
        for i,n in enumerate(names):
            th = self.sensor_thresholds[i]
            binary.append(1 if sensors[n]["raw"]>th else 0)
        # 五路传感器权重：左负右正，中间为0
        weights = [-2,-1,0,1,2]
        pos,total = 0,0
        # 加权平均计算黑线偏移位置
        for i,v in enumerate(binary):
            if v:
                pos += weights[i]
                total += abs(weights[i])
        # 存在黑线则加权平均，无黑线位置置0
        pos = pos/total if total>0 else 0
        return pos,binary,sensors

# ===================== 双电机驱动控制类：PWM调速、正反转、平滑加减速 =====================
class MotorController:
    def __init__(self, base_speed=650, min_speed=480, max_speed=800):
        # 左轮PWM：L1前进，L2后退
        self.L1=PWM(Pin(15,0),freq=20000,duty=0)
        self.L2=PWM(Pin(13,0),freq=20000,duty=0)
        # 右轮PWM：R1前进，R2后退
        self.R1=PWM(Pin(14,0),freq=20000,duty=0)
        self.R2=PWM(Pin(25,0),freq=20000,duty=0)
        self.MAX_SPEED=max_speed   # 电机最大限速
        self.MIN_SPEED=min_speed   # 电机最小启动速度（死区补偿）
        self.BASE_SPEED=base_speed # 直行基准速度
        self.curL,self.curR = 0.0,0.0  # 当前实时左右轮速度
        self.tarL,self.tarR = 0.0,0.0  # 目标左右轮速度
        self.acceleration = 0.95       # 速度平滑系数，实现缓加速缓减速
        # 左右轮编码器实例，预留轮速闭环功能（当前未启用）
        self.encL = Encoder(16,17)
        self.encR = Encoder(18,19)
        self.use_encoder=False
        self.car_stop()
        print(f"电机初始化 min:{min_speed} base:{base_speed} max:{max_speed}")
    
    # 最小速度死区处理：速度非0但过小则强制拉到最小启动速度
    def _limit_min(self,sp):
        if sp>0 and sp<self.MIN_SPEED: 
            return self.MIN_SPEED
        if sp<0 and sp>-self.MIN_SPEED: 
            return -self.MIN_SPEED
        return sp
    
    # 底层电机调速核心函数，处理限幅、平滑、正反转输出
    def _set_motor(self,l,r):
        # 限制PWM数值范围-1023~1023
        l = max(-1023, min(1023, l))
        r = max(-1023, min(1023, r))
        # 限速+死区补偿
        l = self._limit_min(max(-self.MAX_SPEED, min(self.MAX_SPEED,l)))
        r = self._limit_min(max(-self.MAX_SPEED, min(self.MAX_SPEED,r)))
        self.tarL,self.tarR = l,r
        # 一阶平滑过渡，避免速度突变
        self.curL += (self.tarL - self.curL)*self.acceleration
        self.curR += (self.tarR - self.curR)*self.acceleration
        # 取绝对值作为PWM占空比
        la,ra = int(abs(self.curL)), int(abs(self.curR))
        la = max(0, min(1023, la))
        ra = max(0, min(1023, ra))
        # 正负速度区分正反转输出
        self.L1.duty(la if self.curL>=0 else 0)
        self.L2.duty(la if self.curL<0 else 0)
        self.R1.duty(ra if self.curR>=0 else 0)
        self.R2.duty(ra if self.curR<0 else 0)
    
    # 外部调用接口：设置左右轮目标速度
    def set_speeds(self,l,r): 
        self._set_motor(l,r)
    
    # 整车停机：所有PWM置0，清空速度缓存
    def car_stop(self):
        self.L1.duty(0);self.L2.duty(0);self.R1.duty(0);self.R2.duty(0)
        self.curL=self.curR=self.tarL=self.tarR=0.0

# ===================== 循迹主逻辑类：赛道识别、PID复合调速、丢线自救 =====================
class LineFollower:
    def __init__(self, base=650, min_sp=480, max_sp=800):
        self.sensor = PhotoelectricSampler() # 实例化五路灰度传感器
        self.motor = MotorController(base, min_sp, max_sp) # 实例化电机控制器
        # PID纠偏参数，输出限制±650
        self.pid = PIDController(kp=1.0, ki=0.008, kd=7.0, output_limits=(-650,650))
        self.no_line_counter = 0    # 丢线连续计数
        self.last_pos = 0.0         # 上一帧黑线偏移位置
        self.lastL,self.lastR = float(base),float(base) # 上一帧左右轮速度缓存
        self.running = False        # 小车运行状态标志
        self.motor.car_stop()
    
    # 根据五路黑白二值数组识别赛道类型，返回速度缩放、基础转向差值、急弯标记
    def get_track_mode(self, bin_arr):
        L1, L2, M, R2, R1 = bin_arr
        mode = "Straight"
        speed_scale = 1.0
        diff = 0
        sharp_corner = False
        # 左直角弯：左三路黑线，右侧无黑线
        if (L1 and L2 and M) and not R2 and not R1:
            mode = "LeftRightAngle"
            speed_scale = 0.6
            diff = -320
            sharp_corner = True
        # 右直角弯：右三路黑线，左侧无黑线
        elif (M and R2 and R1) and not L1 and not L2:
            mode = "RightRightAngle"
            speed_scale = 0.6
            diff = 320
            sharp_corner = True
        # 大左弯：仅最左侧传感器检测黑线
        elif L1 == 1 and L2 == 0 and M == 0 and R2 == 0 and R1 == 0:
            mode = "BigLeft"
            speed_scale = 0.72
            diff = -240
        # 大右弯：仅最右侧传感器检测黑线
        elif R1 == 1 and L1 == 0 and L2 == 0 and M == 0 and R2 == 0:
            mode = "BigRight"
            speed_scale = 0.72
            diff = 240
        # 小幅左弯：仅左二传感器黑线
        elif L2 == 1 and L1 == 0 and M == 0 and R2 == 0 and R1 == 0:
            mode = "SlightLeft"
            speed_scale = 0.94
            diff = -130
        # 小幅右弯：仅右二传感器黑线
        elif R2 == 1 and L1 == 0 and L2 == 0 and M == 0 and R1 == 0:
            mode = "SlightRight"
            speed_scale = 0.94
            diff = 130
        # 十字路口：五路全部检测到黑线
        elif sum(bin_arr) == 5:
            mode = "CrossRoad"
            speed_scale = 1.0
            diff = 0
        # 标准直道：仅中间传感器黑线
        elif M == 1 and L1 == 0 and L2 == 0 and R2 == 0 and R1 == 0:
            mode = "Straight"
            speed_scale = 1.0
            diff = 0
        # 混合曲线：多传感器不规则黑线组合
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
    
    # 单帧循迹核心执行函数，返回整车状态与调试数据
    def follow_line(self):
        pos, bin_arr, sens = self.sensor.get_line_position()
        total_bin = sum(bin_arr)
        L1, L2, M, R2, R1 = bin_arr
        # 丢线处理：无任何传感器检测到黑线
        if total_bin == 0:
            self.no_line_counter += 1
            # 连续480帧丢线判定彻底丢失，自动停机
            if self.no_line_counter > 480:
                self.motor.car_stop()
                return "stop", pos, bin_arr, sens, 0, 0, 0, 0, "Stop", "LostTimeout"
            # 短时丢线：低速搜寻黑线
            search_sp = self.motor.MIN_SPEED + 50
            turn_add = 320
            # 根据上一次黑线位置决定搜寻转向
            if self.last_pos < -0.3:
                self.motor.set_speeds(search_sp, search_sp + turn_add)
                return "search_left", pos, bin_arr, sens, search_sp, search_sp+turn_add, 0,0,"SearchLeft","Lost_TurnLeft"
            else:
                self.motor.set_speeds(search_sp + turn_add, search_sp)
                return "search_right", pos, bin_arr, sens, search_sp+turn_add, search_sp,0,0,"SearchRight","Lost_TurnRight"
        # 正常识别到黑线，重置丢线计数，保存当前偏移位置
        self.last_pos = pos
        self.no_line_counter = 0
        # 获取赛道类型、速度缩放、基础转向差值
        track_mode, speed_scale, base_diff, sharp = self.get_track_mode(bin_arr)
        # PID计算位置修正量，限制最大修正幅度±400
        corr = self.pid.update(pos)
        corr = max(-400, min(400, corr))
        # 当前帧基准速度
        base_sp = self.motor.BASE_SPEED * speed_scale
        # 修复弯道甩外侧公式：基础转向差值与PID修正反向叠加
        L = base_sp + base_diff - corr
        R = base_sp - base_diff + corr
        # 多帧速度平滑滤波，进一步防抖
        alpha = 0.86
        L = self.lastL * (1 - alpha) + L * alpha
        R = self.lastR * (1 - alpha) + R * alpha
        self.lastL, self.lastR = L, R
        # 限制速度在最小~最大区间
        maxA = self.motor.MAX_SPEED
        L = max(self.motor.MIN_SPEED, min(maxA, L))
        R = max(self.motor.MIN_SPEED, min(maxA, R))
        # 输出速度到电机
        self.motor.set_speeds(int(L), int(R))
        # 判断整车行驶动作标识
        diff_val = L - R
        act = "FWD"
        if diff_val < -20:
            act = "SL" if diff_val < -60 else "L"
        elif diff_val > 20:
            act = "SR" if diff_val > 60 else "R"
        return act, pos, bin_arr, sens, L, R, 0, corr, act, track_mode
    
    # 小车主运行循环
    def run(self, print_int=1):
        print("小车2秒后启动 Ctrl+C停止")
        time.sleep(2)
        self.running=True
        last_print = time.time()
        try:
            while self.running:
                try:
                    # 执行一帧循迹逻辑
                    act, pos, bin_arr, sens, L, R, clv, corr, actname, info = self.follow_line()
                except ValueError as e:
                    # 捕获速度计算异常，重置PID防止卡死
                    print("【警告】速度超限，重置PID:", e)
                    self.pid.reset()
                    self.motor.set_speeds(self.motor.BASE_SPEED, self.motor.BASE_SPEED)
                    self.no_line_counter = 0
                    continue
                # 定时打印调试信息
                if time.time() - last_print >= print_int:
                    l1,l2,m,r2,r1 = [sens[x]["raw"] for x in ["left1","left2","mid","right2","right1"]]
                    bw = ["B" if v else "W" for v in bin_arr]
                    print(f"\n【状态】{actname} | 赛道:{info} pos={pos:.2f} PID={corr:.1f} L={int(L)} R={int(R)}")
                    print(f"传感器 L1:{l1} L2:{l2} M:{m} R2:{r2} R1:{r1} 黑线:{bw}")
                    last_print = time.time()
                time.sleep(0.01) # 循环周期10ms
        except KeyboardInterrupt:
            # 捕获Ctrl+C手动停止
            print("\n手动停止")
        finally:
            # 退出循环后安全停机
            self.running=False
            self.motor.car_stop()
            print("小车已停机")

# 程序入口函数
def main():
    # 整车速度参数配置
    MIN_SPEED = 480    # 电机最小启动速度
    BASE_SPEED = 650   # 直行基础速度
    MAX_SPEED = 800    # 电机最大限速
    # 实例化循迹小车
    car = LineFollower(base=BASE_SPEED, min_sp=MIN_SPEED, max_sp=MAX_SPEED)
    # 启动运行，每秒打印一次调试信息
    car.run(print_int=1)

# 程序启动入口
if __name__ == "__main__":
    main()