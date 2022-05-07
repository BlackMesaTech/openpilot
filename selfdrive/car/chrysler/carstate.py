from cereal import car
from common.conversions import Conversions as CV
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.chrysler.values import DBC, STEER_THRESHOLD


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["Transmission_Status"]["Gear_State"]

  def update(self, cp, cp_cam):

    ret = car.CarState.new_message()

    # lock info 
    ret.doorOpen = any([cp.vl["BCM_1"]["Driver_Door_Ajar"],
                        cp.vl["BCM_1"]["Passenger_Door_Ajar"],
                        cp.vl["BCM_1"]["Left_Rear_Door_Ajar"],
                        cp.vl["BCM_1"]["Right_Rear_Door_Ajar"]])
    ret.seatbeltUnlatched = cp.vl["ORC_1"]['Driver_Seatbelt_Status'] == 1 #1 is unbuckled

    # brake pedal
    ret.brakePressed = cp.vl["ESP_1"]['Brake_Pedal_State'] ==1  # Physical brake pedal switch
    ret.brake = 0

    # gas pedal
    ret.gas = cp.vl["ECM_5"]["Accelerator_Position"]
    ret.gasPressed = ret.gas > 1e-5

    # car speed
    ret.vEgoRaw = cp.vl["ESP_8"]["Vehicle_Speed"] * CV.KPH_TO_MS # This is the actual speed the DASM looks at to determine the minimum speed. 
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = not ret.vEgoRaw > 0.001
    ret.wheelSpeeds = self.get_wheel_speeds(
    cp.vl["ESP_6"]["Wheel_RPM_Front_Left"],
    cp.vl["ESP_6"]["Wheel_RPM_Rear_Right"],
    cp.vl["ESP_6"]["Wheel_RPM_Rear_Left"],
    cp.vl["ESP_6"]["Wheel_RPM_Front_Right"],
    unit=1,
    )
    #ret.aEgo = cp.vl["ESP_4"]["Acceleration"] #m/s2
    #ret.yawRate = cp.vl["ESP_4"]["Yaw_Rate"] #deg/s

    # button presses
    ret.leftBlinker = (cp.vl["Steering_Column_Commands"]["Turn_Signal_Status"] == 1)
    ret.rightBlinker = (cp.vl["Steering_Column_Commands"]["Turn_Signal_Status"] == 2)
    ret.genericToggle = bool(cp.vl["Steering_Column_Commands"]["High_Beam_Lever_Status"])
    ret.espDisabled = (cp.vl["Center_Stack_1"]["Traction_Button"] == 1) #button is pressed. This doesn't mean ESP is diabled.
 
    # steering wheel  
    ret.steeringAngleDeg = cp.vl["Steering_Column_Angle_Status"]["Steering_Wheel_Angle"]
    ret.steeringRateDeg = cp.vl["Steering_Column_Angle_Status"]["Steering_Rate"]
    ret.steeringTorque = cp.vl["EPS_2"]["Steering_Column_Torque"] #applied by driver
    ret.steeringTorqueEps = cp.vl["EPS_2"]["EPS_Motor_Torque"] #applied EPS motor
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    self.frame = int(cp.vl["EPS_2"]["COUNTER"])
    steer_state = cp.vl["EPS_2"]["LKAS_STATE"]
    ret.steerFaultPermanent = steer_state == 4 or (steer_state == 0 and ret.vEgo > self.CP.minSteerSpeed)
    
    # gear
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(cp.vl["Transmission_Status"]["Gear_State"], None))

    # Cruise Control
    ret.cruiseState.enabled = cp.vl["DAS_3"]["ACC_Engaged"] == 1 #Bit to determine if ACC is engaged.
    ret.cruiseState.standstill = cp.vl["DAS_3"]["ACC_StandStill"] == 1 #Bit to determine if the vehicle is standing still.
    ret.cruiseState.speed = cp.vl["DAS_4"]["ACC_Set_Speed"] * CV.KPH_TO_MS
    # ACC_Activation_Status is a three bit msg, 0 is off, 1 and 2 are Non-ACC mode, 3 and 4 are ACC mode
    ret.cruiseState.available = cp.vl["DAS_4"]['ACC_Activation_Status'] in [3, 4]  #3 ACCOn and 4 ACCSet
    ret.cruiseState.nonAdaptive = cp.vl["DAS_4"]["ACC_Activation_Status"] in (1, 2) #1 NormalCCOn and 2 NormalCCSet
    self.button_counter = cp.vl["Cruise_Control_Buttons"]["COUNTER"]

    # blindspot sensors
    if self.CP.enableBsm:
      ret.leftBlindspot = cp.vl["BSM_1"]["Blind_Spot_Monitor_Left"] == 1
      ret.rightBlindspot = cp.vl["BSM_1"]["Blind_Spot_Monitor_Right"] == 1 

    self.lkas_counter = cp_cam.vl["LKAS_COMMAND"]["COUNTER"]
    self.lkas_car_model = cp_cam.vl["DAS_6"]["CAR_MODEL"]
    self.lkas_status_ok = cp_cam.vl["LKAS_HEARTBIT"]["LKAS_STATUS_OK"]

    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address
      ("Gear_State", "Transmission_Status"), #Gear Position
      ("Vehicle_Speed", "ESP_8"),#Vehicle Speed
      #("Acceleration", "ESP_4"),#Acceleration Rate
      #("Yaw_Rate", "ESP_4"),#Yaw Rate
      ("Wheel_RPM_Front_Left", "ESP_6"),#FL Wheel Speed
      ("Wheel_RPM_Front_Right", "ESP_6"),#FR Wheel Speed
      ("Wheel_RPM_Rear_Left", "ESP_6"),#RL Wheel Speed
      ("Wheel_RPM_Rear_Right", "ESP_6"),#RR Wheel Speed
      ("Accelerator_Position", "ECM_5"), #Accelerator Position
      ("Brake_Pedal_State", "ESP_1"),#Brake Pedal Pressed
      ("Steering_Wheel_Angle", "Steering_Column_Angle_Status"),#Steering Angle
      ("Steering_Rate", "Steering_Column_Angle_Status"),#Steering rate
      ("Steering_Column_Torque", "EPS_2"),#EPS Driver applied torque
      ("EPS_Motor_Torque", "EPS_2"),#EPS Motor Torque output
      ("LKAS_STATE", "EPS_2"),#LKAS State  
      ("COUNTER", "EPS_2"),#EPS Counter  
      ("Traction_Button", "Center_Stack_1"),#Traction Control Button
      ("Turn_Signal_Status", "Steering_Column_Commands"),#Blinker 
      ("High_Beam_Lever_Status", "Steering_Column_Commands"),#High Beam Lever
      ("COUNTER", "Cruise_Control_Buttons"),#ACC Counter Button
      ("Driver_Door_Ajar", "BCM_1"),#driver Door
      ("Passenger_Door_Ajar", "BCM_1"),#Passenger Door
      ("Left_Rear_Door_Ajar", "BCM_1"),#Driver Rear Door
      ("Right_Rear_Door_Ajar", "BCM_1"),#Passenger Rear Door
      ("Driver_Seatbelt_Status", "ORC_1"), #Driver Sear Belt
      ("ACC_Engaged", "DAS_3"),#ACC Engaged
      ("ACC_StandStill", "DAS_3"),#ACC Engaged
      ("ACC_Set_Speed", "DAS_4"),
      ("ACC_Activation_Status", "DAS_4"),
    ]

    checks = [
      # sig_address, frequency
      ("Transmission_Status", 50),
      ("ESP_1", 50),
      ("ESP_4", 50),
      ("ESP_6", 50),
      ("ESP_8", 50),
      ("ECM_5", 50),
      ("Steering_Column_Angle_Status", 100),
      ("EPS_2", 100),
      ("Center_Stack_1", 1),
      ("Steering_Column_Commands", 10),
      ("Cruise_Control_Buttons", 50),
      ("BCM_1", 1),
      ("ORC_1", 1),
      ("DAS_3", 50),
      ("DAS_4", 50),
    ]

    if CP.enableBsm:
      signals += [
        ("Blind_Spot_Monitor_Left", "BSM_1"),
        ("Blind_Spot_Monitor_Right", "BSM_1"),
      ]
      checks.append(("BSM_1", 2))

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = [
      # sig_name, sig_address
      ("COUNTER", "LKAS_COMMAND"),
      ("CAR_MODEL", "DAS_6"),
      ("LKAS_STATUS_OK", "LKAS_HEARTBIT")
    ]
    checks = [
      ("LKAS_COMMAND", 100),
      ("LKAS_HEARTBIT", 10),
      ("DAS_6", 4),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 2)
