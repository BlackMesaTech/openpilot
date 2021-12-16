from cereal import car
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.chrysler.values import DBC, STEER_THRESHOLD, CAR
class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["Transmission_Status"]["Gear_State"]
  
  def update(self, cp, cp_cam):

    ret = car.CarState.new_message() 
  
  # car speed
    ret.vEgoRaw = cp.vl["ESP_8"]["Vehicle_Speed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = not ret.vEgoRaw > 0.001
    ret.wheelSpeeds.fl = cp.vl["ESP_6"]["Wheel_RPM_Front_Left"]
    ret.wheelSpeeds.rr = cp.vl["ESP_6"]["Wheel_RPM_Rear_Right"]
    ret.wheelSpeeds.rl = cp.vl["ESP_6"]["Wheel_RPM_Rear_Left"]
    ret.wheelSpeeds.fr = cp.vl["ESP_6"]["Wheel_RPM_Front_Right"]
    #ret.aEgo = cp.vl["ESP_4"]["Acceleration"] #m/s2
    #ret.yawRate = cp.vl["ESP_4"]["Yaw_Rate"] #deg/s
    
  # gas pedal
    ret.gas = cp.vl["ECM_5"]["Accelerator_Position"]
    ret.gasPressed = ret.gas > 45 # up from 5
  
  # brake pedal
    ret.brakePressed = cp.vl["ESP_1"]['Brake_Pedal_State'] ==1  # Physical brake pedal switch
    ret.brake = 0

  # steering wheel  
    ret.steeringAngleDeg = cp.vl["Steering_Column_Angle_Status"]["Steering_Wheel_Angle"]
    ret.steeringRateDeg = cp.vl["Steering_Column_Angle_Status"]["Steering_Rate"]
    ret.steeringTorque = cp.vl["EPS_2"]["Steering_Column_Torque"]
    ret.steeringTorqueEps = cp.vl["EPS_2"]["EPS_Motor_Torque"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    ret.espDisabled = (cp.vl["Center_Stack_1"]["Traction_Button"] == 1) #button is pressed. This doesn't mean ESP is diabled.
    self.frame = int(cp.vl["EPS_2"]["COUNTER"])

  # cruise state  
    self.steer_command_bit = cp_cam.vl["LKAS_COMMAND"]['LKAS_CONTROL_BIT'] 
    self.lkas_counter = cp_cam.vl["LKAS_COMMAND"]["COUNTER"]
    self.autoHighBeamBit = cp_cam.vl["DAS_6"]['Auto_High_Beam'] #Auto High Beam isn't Located in this message on chrysler or jeep currently located in 729 message
    self.lkas_car_model = cp_cam.vl["DAS_6"]["CAR_MODEL"] 

    if self.CP.carFingerprint in (CAR.PACIFICA_2017_HYBRID, CAR.PACIFICA_2018_HYBRID, CAR.PACIFICA_2019_HYBRID, CAR.PACIFICA_2018, CAR.PACIFICA_2020, CAR.JEEP_CHEROKEE_2019, CAR.JEEP_CHEROKEE):
      ret.cruiseState.enabled = cp.vl["DAS_3"]["ACC_Engaged"] == 1  # ACC is green.
      ret.cruiseState.speed = cp.vl["DAS_4"]["ACC_Set_Speed"] * CV.KPH_TO_MS
      # ACC_Activation_Status is a three bit msg, 0 is off, 1 and 2 are Non-ACC mode, 3 and 4 are ACC mode
      ret.cruiseState.available = cp.vl["DAS_4"]['ACC_Activation_Status'] in [3, 4]  #3 ACCOn and 4 ACCSet
      ret.cruiseState.nonAdaptive = cp.vl["DAS_4"]["ACC_Activation_Status"] in [1, 2] #1 NormalCCOn and 2 NormalCCSet
      #ret.cruiseState.speedOffset = ret.cruiseState.speed - ret.vEgo
      self.dashboard = cp.vl["DAS_4"]  
      self.steer_state = cp.vl["EPS_2"]["Torque_Overlay_Status"]
      ret.steerError = self.steer_state == 4 or (self.steer_state == 0 and ret.vEgo > self.CP.minSteerSpeed)

    if self.CP.carFingerprint in (CAR.RAM_1500, CAR.RAM_2500):
      ret.cruiseState.enabled = cp_cam.vl["DAS_3"]["ACC_Engaged"] == 1  # ACC is green.
      ret.cruiseState.speed = cp_cam.vl["DAS_4"]["ACC_Set_Speed"] * CV.KPH_TO_MS
      # ACC_Activation_Status is a three bit msg, 0 is off, 1 and 2 are Non-ACC mode, 3 and 4 are ACC mode
      ret.cruiseState.available = True #cp_cam.vl["DAS_4"]['ACC_Activation_Status'] in [3, 4]  #3 ACCOn and 4 ACCSet
      ret.cruiseState.nonAdaptive = cp_cam.vl["DAS_4"]["ACC_Activation_Status"] in [1, 2] #1 NormalCCOn and 2 NormalCCSet
      #ret.cruiseState.speedOffset = ret.cruiseState.speed - ret.vEgo
      self.dashboard = cp_cam.vl["DAS_4"]
      ret.steerError = cp_cam.vl["LKAS_COMMAND"]["LKAS_ERROR"]==1


  # gear
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(cp.vl["Transmission_Status"]["Gear_State"], None))

  # button presses
    ret.leftBlinker = cp.vl["Steering_Column_Commands"]["Turn_Signal_Status"] == 1
    ret.rightBlinker = cp.vl["Steering_Column_Commands"]["Turn_Signal_Status"] == 2
    ret.genericToggle = bool(cp.vl["Steering_Column_Commands"]["High_Beam_Lever_Status"])
 
  # lock info 
    ret.doorOpen = any([cp.vl["BCM_1"]["Driver_Door_Ajar"],
                        cp.vl["BCM_1"]["Passenger_Door_Ajar"],
                        cp.vl["BCM_1"]["Left_Rear_Door_Ajar"],
                        cp.vl["BCM_1"]["Right_Rear_Door_Ajar"]])
    ret.seatbeltUnlatched = cp.vl["ORC_1"]['Driver_Seatbelt_Status'] == 1 #1 is unbuckled

  # blindspot sensors
    if self.CP.enableBsm:
      ret.leftBlindspot = cp.vl["BSM_1"]["Blind_Spot_Monitor_Left"] == 1
      ret.rightBlindspot = cp.vl["BSM_1"]["Blind_Spot_Monitor_Right"] == 1    

    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("Gear_State", "Transmission_Status", 0), #Gear Position
      ("Vehicle_Speed", "ESP_8", 0),#Vehicle Speed
      ("Acceleration", "ESP_4", 0),#Acceleration Rate
      ("Yaw_Rate", "ESP_4", 0),#Yaw Rate
      ("Wheel_RPM_Front_Left", "ESP_6", 0),#FL Wheel Speed
      ("Wheel_RPM_Front_Right", "ESP_6", 0),#FR Wheel Speed
      ("Wheel_RPM_Rear_Left", "ESP_6", 0),#RL Wheel Speed
      ("Wheel_RPM_Rear_Right", "ESP_6", 0),#RR Wheel Speed
      ("Accelerator_Position", "ECM_5", 0), #Accelerator Position
      ("Brake_Pedal_State", "ESP_1", 0),#Brake Pedal Pressed
      ("Steering_Wheel_Angle", "Steering_Column_Angle_Status", 0),#Steering Angle
      ("Steering_Rate", "Steering_Column_Angle_Status", 0),#Steering rate
      ("Steering_Column_Torque", "EPS_2", 0),#EPS Driver applied torque
      ("EPS_Motor_Torque", "EPS_2", 0),#EPS Motor Torque output
      ("Torque_Overlay_Status", "EPS_2", 1),
      ("Traction_Button", "Center_Stack_1", 0),#Traction Control Button
      ("Turn_Signal_Status", "Steering_Column_Commands", 0),#Blinker 
      ("High_Beam_Lever_Status", "Steering_Column_Commands", 0),#High Beam Lever
      ("ACC_Accel", "Cruise_Control_Buttons", 0),#ACC Accel Button
      ("ACC_Decel", "Cruise_Control_Buttons", 0),#ACC Decel Button
      ("ACC_Cancel", "Cruise_Control_Buttons", 0),#ACC Cancel Button
      ("ACC_Distance_Inc", "Cruise_Control_Buttons", 0),#ACC Distance Increase Button
      ("Driver_Door_Ajar", "BCM_1", 0),#driver Door
      ("Passenger_Door_Ajar", "BCM_1", 0),#Passenger Door
      ("Left_Rear_Door_Ajar", "BCM_1", 0),#Driver Rear Door
      ("Right_Rear_Door_Ajar", "BCM_1", 0),#Passenger Rear Door
      ("Driver_Seatbelt_Status", "ORC_1", 0), #Driver Sear Belt
      ("COUNTER", "EPS_2", -1),#EPS Counter  
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
    ]

    if CP.enableBsm:
      signals += [
        ("Blind_Spot_Monitor_Left", "BSM_1", 0),
        ("Blind_Spot_Monitor_Right", "BSM_1", 0),
      ]
      checks += [("BSM_1", 2)]

    if CP.carFingerprint in (CAR.PACIFICA_2017_HYBRID, CAR.PACIFICA_2018_HYBRID, CAR.PACIFICA_2019_HYBRID, CAR.PACIFICA_2018, CAR.PACIFICA_2020, CAR.JEEP_CHEROKEE_2019, CAR.JEEP_CHEROKEE):
      signals += [
        ("ACC_Engaged", "DAS_3", 0),#ACC Engaged
        ("ACC_Set_Speed", "DAS_4", -1),
        ("ACC_Activation_Status", "DAS_4", -1),
      ]
      checks += [("DAS_3", 50),
        ("DAS_4", 50),]


    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("LKAS_CONTROL_BIT", "LKAS_COMMAND", 0),
      ("COUNTER", "LKAS_COMMAND", 0),
      ("LKAS_ERROR", "LKAS_COMMAND", 0),
      ("Auto_High_Beam", "DAS_6", -1), 
      ("CAR_MODEL", "DAS_6", -1),
      ("ACC_Engaged", "DAS_3", 0),#ACC Engaged
      ("ACC_Set_Speed", "DAS_4", -1),
      ("ACC_Activation_Status", "DAS_4", -1),
    ]
    checks = [
      ("LKAS_COMMAND", 50),
      ("DAS_6", 15),
      ("DAS_3", 50),
      ("DAS_4", 50),
    ]

    if CP.carFingerprint in (CAR.RAM_1500, CAR.RAM_2500):
      signals += [
        ("ACC_Engaged", "DAS_3", 0),#ACC Engaged
        ("ACC_Set_Speed", "DAS_4", -1),
        ("ACC_Activation_Status", "DAS_4", -1),
      ]
      checks += [
        ("DAS_3", 50),
        ("DAS_4", 50),
        ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 2)