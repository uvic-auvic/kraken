import time
import math

class PID():
	def __init__(self, current, target, kp, ki, kd):
		self.__current = current
		self.__target = target
		self.__kp = kp
		self.__ki = ki
		self.__kd = kd
		self.__lower = -math.inf
		self.__upper = math.inf
		

		self.__diff = target - current
		self.__P = 0
		self.__I = 0
		self.__D = 0
		self.__output = 0
		
		self.__dt = 0
		self.__last_time = time.time()
		

	def set_current(self, current):
		self.__current = current

	def get_current(self):
		return self.__current

	def set_target(self, target):
		self.__target = target

	def get_target(self):
		return self.__target

	def set_params(self, kp=None, ki=None, kd=None):
		if kp is not None:
			self.__kp = kp
		if ki is not None:
			self.__ki = ki
		if kd is not None:
			self.__kd = kd

	def get_params(self):
		return (self.__kp, self.__ki, self.__kd)

	def get_values(self):
		return str(round(self.__P, 2)) + "\t" + str(round(self.__I, 2)) + "\t" + str(round(self.__D, 2))

	def set_i_bounds(self, lower=-math.inf, upper=math.inf):
		self.__lower = lower
		self.__upper = upper

	def get_output(self):
		return self.__output

	def calculate(self, current):
		self.__current = current
		prev_diff = self.__diff
		self.__dt = time.time() - self.__last_time
		self.__last_time = time.time()

		self.__diff = self.__target - self.__current
	

		self.__P = self.__kp * self.__diff

		self.__I += self.__ki * self.__diff * self.__dt

		if self.__I > self.__upper:
			self.__I = self.__upper
		elif self.__I < self.__lower:
			self.__I = self.__lower

		self.__D = self.__kd * (self.__diff - prev_diff) / self.__dt

		output = self.__P + self.__I + self.__D

		self.__output = output

		#print(output)
		#print(self.__diff)

		return output

		
		