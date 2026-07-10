from machine import Pin, PWM, ADC
import time

# ===================== PID控制器【优化防抖参数】 =====================
class PIDController:
    """
    分段积分防抖PID控制器，用于黑线位置闭环纠偏
    kp：比例系数，ki：积分系数，kd：微分系数
    setpoint：目标值，黑线居中时pos=0
    output_limits：PID输出修正量上下限
    """
    def __init__(self, kp, ki, kd, setpoint=0, output_limits=(-1023, 1023)):
        self.kp = kp                  # 比例系数：快速修正黑线偏移
        self.ki = ki                  # 积分系数：消除直道长期静态偏移
        self.kd = kd                  # 微分系数：抑制车身抖动、缓冲过弯冲击
        self.setpoint = setpoint      # 目标位置：黑线居中偏移为0
        self.output_limits = output_limits  # PID输出上下限，防止修正过猛
        
        self._integral = 0            # 积分累加缓存，累计误差消除静差
        self._last_error = 0          # 上一帧偏差，用于计算微分项
        self._last_time = time.ticks_ms() # 上一次PID计算的毫秒时间戳
        self._last_output = 0.0       # 上一轮PID输出，用于平滑滤波
        self._output_smoothing = 0.30 # PID输出平滑系数，数值越大防抖越强，响应越慢

    def update(self, measured_value, dt=None):
        """
        PID核心计算函数
        measured_value：当前传感器检测到的黑线偏移pos
        dt：外部传入时间间隔，不传则自动计算帧间隔
        return：PID速度修正量
        """
        # 外部未传入时间间隔，则自动计算本次与上次PID的时间差
        if dt is None:
            current_time = time.ticks_ms()
            dt = time.ticks_diff(current_time, self._last_time) / 1000.0
            self._last_time = current_time
        # 防止dt=0触发除零报错，强制最小周期0.005秒
        if dt <= 0:
            dt = 0.005
        
        # 计算偏差：目标位置 - 当前实际黑线位置
        error = self.setpoint - measured_value
        p_term = self.kp * error  # 比例项，偏差越大修正力度越大

        # 分段积分逻辑：大幅偏离时衰减积分，防止积分饱和甩尾
        if abs(error) < 1.0:
            # 偏差很小（接近中线），正常累加积分消除直道静差
            self._integral += error * dt
        else:
            # 偏差大（弯道大幅偏移），积分衰减65%，避免积分爆炸
            self._integral *= 0.65
        # 积分限幅，强制锁定在-120~120之间，防止积分饱和
        self._integral = max(-120, min(120, self._integral))
        i_term = self.ki * self._integral  # 积分项输出

        # 微分项：偏差变化速率，抑制震荡、车身抖动
        derivative = (error - self._last_error) / dt
        d_term = self.kd * derivative
        
        # 比例+积分+微分 合成原始修正量
        output = p_term + i_term + d_term

        # 硬限幅：修正值不能超出设定上下限
        output = max(self.output_limits[0], min(self.output_limits[1], output))
        # 一阶低通平滑滤波，消除修正值突变，电机运转更顺滑
        output = self._last_output * self._output_smoothing + output * (1 - self._output_smoothing)
        
        # 缓存本次数据，供下一帧计算使用
        self._last_output = output
        self._last_error = error
        return output

    def reset(self):
        """清空PID全部缓存，异常报错时重置控制器"""
        self._integral = 0
        self._last_error = 0
        self._last_output = 0.0

# ===================== 五路红外灰度光电采样类 =====================
class PhotoelectricSampler:
    """五路灰度传感器采集，计算黑线加权偏移位置pos"""
    def __init__(self):
        # 五路传感器GPIO映射：左1、左2、中间、右2、右1
        self.adc_pins = {'left1':27,'left2':33,'mid':32,'right2':35,'right1':34}
        # 五路传感器黑白判定阈值：AD原始值>140判定为黑线
        self.sensor_thresholds = [140, 140, 140, 140, 140]
        self.adc_objects = {} # 存储每一路ADC实例对象
        self._init_adc()      # 执行ADC通道初始化

    def _init_adc(self):
        """初始化所有ADC通道配置，12位0~3.6V量程"""
        print("初始化ADC通道...")
        for name,pin in self.adc_pins.items():
            adc = ADC(Pin(pin))
            adc.atten(ADC.ATTN_11DB)    # 电压衰减，量程0~3.6V适配红外模块
            adc.width(ADC.WIDTH_12BIT)  # 12位ADC，数值范围0~4095
            self.adc_objects[name] = adc
            print(f"  {name} GPIO{pin} OK")

    def read_all(self):
        """一次性读取五路传感器原始AD值+换算电压"""
        res = {}
        for name,adc in self.adc_objects.items():
            val = adc.read() # 读取0~4095原始模拟数值
            # 换算对应电压：12bit满量程3.6V
            res[name] = {"raw":val,"volt":val/4095*3.6}
        return res

    def get_line_position(self):
        """
        核心函数：计算黑线中心偏移位置pos
        return pos(偏移值), binary(黑白二值数组), sensors(原始传感器数据)
        pos < 0：黑线偏左；pos = 0：黑线居中；pos > 0：黑线偏右
        """
        sensors = self.read_all()
        names = ['left1','left2','mid','right2','right1']
        binary = []
        # 循环五路传感器，生成黑白二值数组：1=黑线，0=白纸
        for i,n in enumerate(names):
            th = self.sensor_thresholds[i]
            binary.append(1 if sensors[n]["raw"]>th else 0)
        # 五路权重：左侧负、右侧正，中间0，用于加权计算偏移量
        weights = [-2,-1,0,1,2]
        pos,total = 0,0
        # 加权平均计算黑线位置
        for i,v in enumerate(binary):
            if v: # 当前传感器检测到黑线
                pos += weights[i]
                total += abs(weights[i])
        # 存在黑线则加权平均；无黑线pos=0
        pos = pos/total if total>0 else 0
        return pos,binary,sensors

# ===================== 电机控制（移除编码器相关代码） =====================
class MotorController:
    """双路直流电机PWM驱动控制器，带速度平滑、最小速度死区补偿"""
    def __init__(self, base_speed=650, min_speed=480, max_speed=800):
        # 左轮电机：L1前进，L2后退
        self.L1 = PWM(Pin(15,0), freq=20000, duty=0)
        self.L2 = PWM(Pin(13,0), freq=20000, duty=0)
        # 右轮电机：R1前进，R2后退
        self.R1 = PWM(Pin(14,0), freq=20000, duty=0)
        self.R2 = PWM(Pin(25,0), freq=20000, duty=0)
        
        self.MAX_SPEED = max_speed   # 电机最大限速，防止飞车
        self.MIN_SPEED = min_speed   # 电机最小启动速度（低速无力死区补偿）
        self.BASE_SPEED = base_speed # 直行基准速度
        self.curL, self.curR = 0.0, 0.0 # 当前实时左右轮速度
        self.tarL, self.tarR = 0.0, 0.0 # 目标左右轮速度
        self.acceleration = 0.88       # 速度平滑系数，数值越大加减速越柔和
        self.car_stop() # 上电初始化先停机
        print(f"电机初始化 min:{min_speed} base:{base_speed} max:{max_speed}")

    def _limit_min(self,sp):
        """最小速度死区处理：低速时强制拉到最小启动速度，解决电机不动问题"""
        # 正向速度小于最小启动值，强制拉到MIN_SPEED
        if sp>0 and sp<self.MIN_SPEED:
            return self.MIN_SPEED
        # 反向速度大于负最小启动值，强制拉到-MIN_SPEED
        if sp<0 and sp>-self.MIN_SPEED:
            return -self.MIN_SPEED
        return sp

    def _set_motor(self,l,r):
        """底层电机调速核心：限幅、平滑过渡、输出PWM占空比"""
        # PWM数值硬限幅-1023~1023
        l = max(-1023, min(1023, l))
        r = max(-1023, min(1023, r))
        # 叠加最大限速+最小速度死区补偿
        l = self._limit_min(max(-self.MAX_SPEED, min(self.MAX_SPEED,l)))
        r = self._limit_min(max(-self.MAX_SPEED, min(self.MAX_SPEED,r)))
        # 保存目标速度
        self.tarL, self.tarR = l, r
        # 一阶平滑过渡，避免速度突变抖动
        self.curL += (self.tarL - self.curL) * self.acceleration
        self.curR += (self.tarR - self.curR) * self.acceleration
        # 取速度绝对值作为PWM占空比
        la, ra = int(abs(self.curL)), int(abs(self.curR))
        la = max(0, min(1023, la))
        ra = max(0, min(1023, ra))
        # 正负速度区分正反转输出
        self.L1.duty(la if self.curL >= 0 else 0)
        self.L2.duty(la if self.curL < 0 else 0)
        self.R1.duty(ra if self.curR >= 0 else 0)
        self.R2.duty(ra if self.curR < 0 else 0)

    def set_speeds(self,l,r):
        """对外调用接口：设置左右轮目标速度"""
        self._set_motor(l,r)

    def car_stop(self):
        """整车停机：全部PWM置0，清空速度缓存"""
        self.L1.duty(0)
        self.L2.duty(0)
        self.R1.duty(0)
        self.R2.duty(0)
        self.curL=self.curR=self.tarL=self.tarR=0.0

# ===================== 循迹主逻辑类（整车控制核心） =====================
class LineFollower:
    """循迹主程序，包含赛道识别、PID复合调速、丢线搜寻容错"""
    def __init__(self, base=650, min_sp=480, max_sp=800):
        self.sensor = PhotoelectricSampler() # 实例化五路灰度传感器
        self.motor = MotorController(base, min_sp, max_sp) # 实例化电机控制器
        # PID调参：降低kp、提升kd，专门解决直道车身抖动
        self.pid = PIDController(kp=0.65, ki=0.006, kd=10.0, output_limits=(-650,650))
        self.no_line_counter = 0    # 连续丢线计数
        self.last_pos = 0.0         # 上一帧黑线偏移位置
        self.lastL, self.lastR = float(base), float(base) # 上一帧左右轮速度缓存
        self.running = False        # 小车运行开关标志
        self.motor.car_stop()

    def get_track_mode(self, bin_arr):
        """
        根据五路黑白二值数组识别赛道类型
        return mode赛道名称, speed_scale速度缩放系数, base_diff基础转向差值
        """
        L1, L2, M, R2, R1 = bin_arr
        mode = "Straight"
        speed_scale = 1.0
        diff = 0

        # 左直角弯：左三路黑线，右侧无黑线，减速、加大转向力度防甩弯
        if (L1 and L2 and M) and not R2 and not R1:
            mode = "LeftRightAngle"
            speed_scale = 0.55
            diff = -360
        # 右直角弯：右三路黑线，左侧无黑线
        elif (M and R2 and R1) and not L1 and not L2:
            mode = "RightRightAngle"
            speed_scale = 0.55
            diff = 360
        # 大左弯：仅最左侧传感器黑线
        elif L1 == 1 and L2 == 0 and M == 0 and R2 == 0 and R1 == 0:
            mode = "BigLeft"
            speed_scale = 0.68
            diff = -280
        # 大右弯：仅最右侧传感器黑线
        elif R1 == 1 and L1 == 0 and L2 == 0 and M == 0 and R2 == 0:
            mode = "BigRight"
            speed_scale = 0.68
            diff = 280
        # 小幅左弯：仅左二传感器黑线
        elif L2 == 1 and L1 == 0 and M == 0 and R2 == 0 and R1 == 0:
            mode = "SlightLeft"
            speed_scale = 0.88
            diff = -160
        # 小幅右弯：仅右二传感器黑线
        elif R2 == 1 and L1 == 0 and L2 == 0 and M == 0 and R1 == 0:
            mode = "SlightRight"
            speed_scale = 0.88
            diff = 160
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
            # 根据上一帧偏移设置轻微转向补偿
            if self.last_pos < -0.1:
                diff = -100
            elif self.last_pos > 0.1:
                diff = 100
            else:
                diff = 0
        return mode, speed_scale, diff

    def follow_line(self):
        """单帧循迹执行函数，每一帧循环调用一次"""
        pos, bin_arr, sens = self.sensor.get_line_position()
        total_bin = sum(bin_arr)
        L1, L2, M, R2, R1 = bin_arr

        # ---------------- 丢线容错逻辑 ----------------
        if total_bin == 0: # 所有传感器都没检测到黑线
            self.no_line_counter += 1
            # 连续480帧无黑线，判定彻底丢失，自动停机
            if self.no_line_counter > 480:
                self.motor.car_stop()
                return "stop", pos, bin_arr, sens, 0, 0, 0, "Stop", "LostTimeout"
            # 短时丢线：低速搜寻黑线
            search_sp = self.motor.MIN_SPEED + 50
            turn_add = 320
            # 根据上一次黑线位置判断搜寻方向
            if self.last_pos < -0.3:
                self.motor.set_speeds(search_sp, search_sp + turn_add)
                return "search_left", pos, bin_arr, sens, search_sp, search_sp+turn_add, "SearchLeft","Lost_TurnLeft"
            else:
                self.motor.set_speeds(search_sp + turn_add, search_sp)
                return "search_right", pos, bin_arr, sens, search_sp+turn_add, search_sp, "SearchRight","Lost_TurnRight"

        # 正常识别到黑线，重置丢线计数器，保存当前偏移
        self.last_pos = pos
        self.no_line_counter = 0
        # 获取当前赛道类型、速度缩放系数、基础转向差值
        track_mode, speed_scale, base_diff = self.get_track_mode(bin_arr)
        # PID计算黑线偏移修正量
        corr = self.pid.update(pos)
        # 限制PID最大修正幅度±300，防止修正过猛甩车
        corr = max(-300, min(300, corr))

        # 当前赛道基准速度 = 基础速度 * 弯道减速系数
        base_sp = self.motor.BASE_SPEED * speed_scale
        # 核心防甩弯公式：固定弯道差值与PID修正反向叠加，抑制过弯外侧甩飞
        L = base_sp + base_diff - corr
        R = base_sp - base_diff + corr

        # 多帧速度平滑滤波，进一步降低车身抖动
        alpha = 0.75
        L = self.lastL * (1 - alpha) + L * alpha
        R = self.lastR * (1 - alpha) + R * alpha
        # 缓存本轮速度，下一帧平滑使用
        self.lastL, self.lastR = L, R

        # 速度限制在最小~最大区间内
        maxA = self.motor.MAX_SPEED
        L = max(self.motor.MIN_SPEED, min(maxA, L))
        R = max(self.motor.MIN_SPEED, min(maxA, R))

        # 输出速度到电机
        self.motor.set_speeds(int(L), int(R))
        # 根据左右轮速度差判断行驶动作
        diff_val = L - R
        act = "FWD"
        if diff_val < -20:
            act = "SL" if diff_val < -60 else "L"
        elif diff_val > 20:
            act = "SR" if diff_val > 60 else "R"
        # 返回全部调试数据供串口打印
        return act, pos, bin_arr, sens, L, R, corr, act, track_mode

    def run(self, print_int=1):
        """小车主运行循环
        print_int：打印调试信息的间隔（单位秒）
        """
        print("小车2秒后启动 Ctrl+C停止")
        time.sleep(2) # 上电延时2秒，方便摆放小车
        self.running = True
        last_print = time.time() # 记录上次打印时间
        try:
            while self.running: # 无限循环循迹
                try:
                    # 执行一帧循迹逻辑
                    act, pos, bin_arr, sens, L, R, corr, actname, info = self.follow_line()
                except ValueError as e:
                    # 捕获计算异常，重置PID防止卡死
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
                time.sleep(0.01) # 循环周期10ms，控制运行帧率
        except KeyboardInterrupt:
            # 捕获键盘Ctrl+C手动停止
            print("\n手动停止")
        finally:
            # 无论如何退出循环，都会执行停机保护
            self.running = False
            self.motor.car_stop()
            print("小车已停机")

# ===================== 程序入口函数 =====================
def main():
    # 整车速度参数配置
    MIN_SPEED = 480    # 电机最小启动速度
    BASE_SPEED = 650   # 直行基准速度
    MAX_SPEED = 800    # 电机最大限速
    # 实例化循迹小车对象
    car = LineFollower(base=BASE_SPEED, min_sp=MIN_SPEED, max_sp=MAX_SPEED)
    # 启动运行，每秒打印1次调试信息
    car.run(print_int=1)

# Python程序启动判断：文件直接运行时执行main()
if __name__ == "__main__":
    main()
