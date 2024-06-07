import serial
import time
import struct

"""
Message format:

Preamble: 0xfa
Bus identifier: 0xff
Message identifier:

Error: 0x42
Warning: 0x43
MTData2: 0x36

Length: Length of data field

Data:

Temerature: 081y
UTC Time: 1010
Packet Counter: 1020
Sample Time Fine: 1060
Sample Time Coarse: 1070
Quaternion: 201y
Rotation Matrix: 202y
Euler Angles: 203y
Baro Pressure: 301y
DeltaV: 401y
Acceleration: 402y
Free Acceleration: 403y
AccelerationHR: 404y
Rate of Term: 802y
DeltaQ: 803y
RateOfTurnHR: 804y
Magnetic Field: C02y
Status Byte: e010
Status Word: e020

y value: Logical or of formats

Precision:
0x0: Float32
0x1: Fp1220
0x2: Fp1632
0x3: Float64

Coordinate system:
0x0: ENU
0x4: NED
0x8: NWU


Checksum: Sum of all message bytes excluding preamble, including the checksum, lower byte == 0

"""

type_lookup = {
"081": "Temperature",
"101": "UTC Time",
"102": "Packet Counter",
"106": "Sample Time Fine",
"107": "Sample Time Coarse",
"201": "Quaternion",
"202": "Rotation Matrix",
"203": "Euler Angles",
"301": "Baro Pressure",
"401": "DeltaV",
"402": "Acceleration",
"403": "Free Acceleration",
"404": "AccelerationHR",
"802": "Rate of Turn",
"803": "DeltaQ",
"804": "RateOfTurnHR",
"c02": "Magnetic Field",
"e01": "Status Byte",
"e02": "Status Word"
}

# Open a serial port. You may have to change the first parameter
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

# If the user closes the program, close the serial port
def signal_handler(sig, frame):
	ser.close()
	sys.exit(0)

# Read a byte from serial
def get_byte():
	return int.from_bytes(ser.read(), byteorder="big")

# Convert a list of 4 ints to a float
def list_to_float32(byte_list):
	hex_string = ""
	hex_string += str(hex(byte_list[0]))[2:].zfill(2)
	hex_string += str(hex(byte_list[1]))[2:].zfill(2)
	hex_string += str(hex(byte_list[2]))[2:].zfill(2)
	hex_string += str(hex(byte_list[3]))[2:].zfill(2)

	byte = bytes.fromhex(hex_string)

	return struct.unpack('!f', byte)[0]

# Convert a list of 8 ints to a double
def list_to_float64(byte_list):
	hex_string = ""
	hex_string += str(hex(byte_list[0]))[2:].zfill(2)
	hex_string += str(hex(byte_list[1]))[2:].zfill(2)
	hex_string += str(hex(byte_list[2]))[2:].zfill(2)
	hex_string += str(hex(byte_list[3]))[2:].zfill(2)
	hex_string += str(hex(byte_list[4]))[2:].zfill(2)
	hex_string += str(hex(byte_list[5]))[2:].zfill(2)
	hex_string += str(hex(byte_list[6]))[2:].zfill(2)
	hex_string += str(hex(byte_list[7]))[2:].zfill(2)

	byte = bytes.fromhex(hex_string)

	return struct.unpack('!d', byte)[0]

# Given a 2-byte type code, parse the type as specified above
def parse_type(type_list):
	# String of hex characters
	type_string = str(hex(type_list[0]))[2:].zfill(2)
	type_string += str(hex(type_list[1]))[2:]

	data_format = int(type_string[-1], 16) & 3 # (0011)
	coord_format = int(type_string[-1], 16) & 12 # (1100)
	type_code = type_string[:-1]
	if data_format == 0:
		length = 4
	elif data_format == 1:
		length = 4
	elif data_format == 2:
		length = 6
	elif data_format == 3:
		length = 8

	if coord_format == 4:
		coord = "NED"
	elif coord_format == 8:
		coord = "NWU"
	else:
		coord = "ENU"

	type_dict = {}
	type_dict["name"] = type_lookup[type_code]
	type_dict["length"] = length
	type_dict["format"] = data_format
	type_dict["system"] = coord

	return type_dict


while True:
	if get_byte() != int(0xfa): # Preamble
		continue
	if get_byte() != int(0xff): # BID
		#print("Test")
		continue
		

	checksum = 255 # Keep a running sum of bytes to calculate checksum (255 for BID)

	mid = get_byte()
	if mid == int(0x42):
		print("Error")
		continue
	if mid != int(0x36):
		print("Unknown MID")
		continue

	checksum += 54

	length = get_byte() # Length of data section
	#print(length)
	checksum += length
	state = 0 # 0: type, 1: length, 2: data

	type_list = [] # List containing two bytes that make up the type
	bytes_list = [] # List containing the bytes for the current value
	data_point_list = [] # List containing the values for the current data point
	point_length = 0 # Length of the data point
	value_counter = 0 # Counter for each value
	byte_counter = 0 # Counter for each byte in a value
	data_type = {} # Dict containing name, length of each value in bytes, and system (eg. ENU)
	data = {} # Dict containing parsed data
	for i in range(length):
		current_byte = get_byte()
		#print(hex(current_byte))
		checksum += current_byte

		if state == 0: # Type (2 bytes)
			type_list.append(current_byte)
			if len(type_list) == 2:
				state = 1
				#print(type_list)
				data_type = parse_type(type_list)
				type_list = []

		elif state == 1: # Length (1 byte)
			point_length = current_byte
			state = 2
			value_counter = 0
			byte_counter = 0
			bytes_list = []

		elif state == 2: # Data (Length bytes)
			bytes_list.append(current_byte)
			name = data_type["name"]
			value_length = data_type["length"]
			data_format = data_type["format"]
			num_values = int(point_length/value_length)
			byte_counter += 1
			if byte_counter == value_length:
				byte_counter = 0
				value_counter += 1
				if data_format == 0:
					data_point_list.append(list_to_float32(bytes_list))
				if data_format == 3:
					data_point_list.append(list_to_float64(bytes_list))
				
				bytes_list = []

			if value_counter == num_values:
				state = 0
				data[name] = tuple(data_point_list)
				data_point_list = []


	checksum += get_byte()
	if checksum % 256 == 0:
		print(data)
