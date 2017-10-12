import Adafruit_PCA9685

# 
class SpyndraMotor:

	def __init__(self, freq=60, motor_type=1):
		self.pwm = Adafruit_PCA9685.PCA9685()
		self.pwm.set_pwm_freq(freq)
		self.motor_type = motor_type

	def set_motor_type(motor_type):
		self.motor_type = motor_type

	def output_motor(chassisOutput, tibiaOutput, chassisNum, tibiaNum):
		self.pwm.set_pwm(chassisNum, 0, int(chassisOutput))
		self.pwm.set_pwm(tibiaNum, 0, int(tibiaOutput))

		# Notice: what's this for? if this is setting the constants for motors then it should be set as internal variable
	def set_motor_value(filepath):
		motor0_min = 250
		motor0_max = 300
		motor1_min = 250
		motor1_max = 300
		motor2_min = 250
		motor2_max = 300
		motor3_min = 250
		motor3_max = 300
		motor4_min = 250
		motor4_max = 300
		motor5_min = 250
		motor5_max = 300
		motor6_min = 250
		motor6_max = 300
		motor7_min = 250
		motor7_max = 300
		if(motorType == 1):
			json_data = open(filepath).read()
			parsed_json = json.loads(json_data)
			motor9_min = parsed_json['motor 0 min']
			motor0_max = parsed_json['motor 0 max']
			motor1_min = parsed_json['motor 1 min']
			motor1_max = parsed_json['motor 1 max']
			motor2_min = parsed_json['motor 2 min']
			motor2_min = parsed_json['motor 2 max']
			motor3_min = parsed_json['motor 3 min']
			motor3_max = parsed_json['motor 3 max']
			motor4_min = parsed_json['motor 4 min']
			motor4_max = parsed_json['motor 4 max']
			motor5_min = parsed_json['motor 5 min']
			motor5_max = parsed_json['motor 5 max']
			motor6_min = parsed_json['motor 6 min']
			motor6_max = parsed_json['motor 6 max']
			motor7_min = parsed_json['motor 7 min']
			motor7_max = parsed_json['motor 7 max']
		elif(motorType == 2):
			json_data = open('./servo_settings.json').read()
			parsed_json = json.loads(json_data)
			tibia_min = parsed_json['digital tibia min']
			tibia_max = parsed_json['digital tibia max']
			chassis_min = parsed_json['digital chassis min']
			chassis_max = parsed_json['digital chassis max']