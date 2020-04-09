#!/usr/bin/python

from struct import unpack
from binascii import unhexlify
import json
import re
import sys
import random
import binascii
import codecs

if __name__ == "__main__" or sys.version_info[0] < 3:
	from constants import constants
else:
	from .constants import constants

class PARSE_ERROR:
	WRONG_SIZE = "wrong size packet"
	INVALID_MSG_TYPE = "invalid message type"
	INVALID_SAT_STATE = "invalid sat state"
	INVALID_ECODE = "invalid error code(s)"
	INVALID_ELOC = "invalid error location(s)"

INVALID_STR = "[invalid]"

# constant helpers
DATA_SECTION_START_BYTE = constants["DATA_SECTION_START_BYTE"]
IDLE_BATCHES_PER_PACKET	= constants["IDLE_BATCHES_PER_PACKET"]
ATTITUDE_BATCHES_PER_PACKET	=	constants["ATTITUDE_BATCHES_PER_PACKET"]
FLASHBURST_BATCHES_PER_PACKET =	constants["FLASHBURST_BATCHES_PER_PACKET"]
FLASHCMP_BATCHES_PER_PACKET	=	constants["FLASHCMP_BATCHES_PER_PACKET"]
LOWPOWER_BATCHES_PER_PACKET	=	constants["LOWPOWER_BATCHES_PER_PACKET"]
ERROR_TIME_BUCKET_SIZE = constants["ERROR_TIME_BUCKET_SIZE"]
Ms_and_Bs = constants["Ms_and_Bs"]

def get_line_m_from_signal(sig):
	try:
		return Ms_and_Bs[constants["line_m_from_signal"][sig]]
	except KeyError:
		return -1
def get_line_b_from_signal(sig):
	try:
		return Ms_and_Bs[constants["line_b_from_signal"][sig]]
	except KeyError:
		return -1
def get_ELOC_name(eloc):
	try:
		return constants["ELOC_name"][eloc]
	except IndexError:
		return INVALID_STR
def get_ECODE_name(ecode):
	try:
		return constants["ECODE_name"][ecode]
	except IndexError:
		return INVALID_STR

def untruncate(val, sig):
	u16 = (val) << 8
	# note: it is not desirable to cast to an int before the subtraction here (it's a bug),
	# but we're doing so to keep python3 compatible with python2 and js
	return int(int(u16 / get_line_m_from_signal(sig)) - get_line_b_from_signal(sig))

def get_bit(byte, i):
	return byte & (1<<i) > 0

def hex_to_int_le(hexstr):
	try:
		return unpack('<i', unhexlify(hexstr))[0]
	except TypeError:
		return -1

def int_to_hex(intval):
	return hex(intval)[2:] # remove "0x"

def left_shift_8(byte):
	return byte << 8

def hex_string_byte_to_signed_int(byte):
	return int(byte + '000000', 16) >> 24

def mag_raw_to_mG(raw):
	return round((raw/1090.)*1000.,2)
def acc_raw_to_g(raw):
	return round(raw/16384.,2)
def gyro_raw_to_dps(raw):
	return round(raw/131.,2)

def ir_raw_to_C(raw):
	return round(raw*.02 - 273.15, 2)
def ad590_mV_to_C(mV):
	return round((mV * 0.1286) - 107.405, 0)

def l_sns_mV_to_mA(mV):
	return round((mV - 985) * 2, 0)

def lfbsns_mV_to_mA(mV):
	return round((mV - 980) * 50, 0)
def lfbosns_mV_to_mA(mV):
	return round(mV * 71.43, 0)
def led_sns_mV_to_mA(mV):
	return round(mV / .03, 0)

def get_sat_state(val):
	try:
		return {
			0:	'INITIAL',
			1:	'ANTENNA DEPLOY',
			2:	'HELLO WORLD',
			3:	'IDLE NO FLASH',
			4:	'IDLE FLASH',
			5:	'LOW POWER'
		}[val]
	except KeyError:
		return INVALID_STR

def get_message_type(val):
	try:
		return {
			0:	'IDLE',
			1:	'ATTITUDE',
			2:	'FLASH BURST',
			3:	'FLASH CMP',
			4:	'LOW POWER'
		}[val]
	except KeyError:
		return INVALID_STR

def is_hex_str(ps):
	""" Returns whether the packet is only hexedecimal data """
	try:
		int(ps, 16)
		return True
	except ValueError:
		return False

def parse_preamble(ps):
	preamble = {}
	errs = []
	preamble['callsign'] = unhexlify(ps[0:12]).decode("ascii", errors="replace")
	preamble['timestamp'] = hex_to_int_le(ps[12:20])

	msg_op_states = int(ps[20:22],16)
	preamble['message_type'] = get_message_type(msg_op_states & 0x07) #get_bit(msg_op_states, 7)+get_bit(msg_op_states, 6)+get_bit(msg_op_states, 5)
	if preamble['message_type'] == INVALID_STR:
		errs.append(PARSE_ERROR.INVALID_MSG_TYPE)
	preamble['satellite_state'] = get_sat_state((msg_op_states >> 3) & 0x07) #get_bit(msg_op_states, 4)+get_bit(msg_op_states, 3)+get_bit(msg_op_states, 2)
	if preamble['satellite_state'] == INVALID_STR:
		errs.append(PARSE_ERROR.INVALID_SAT_STATE)

	preamble['FLASH_KILLED'] = get_bit(msg_op_states, 6)
	preamble['MRAM_CPY'] = get_bit(msg_op_states, 7)

	preamble['bytes_of_data'] = int(ps[22:24], 16)
	preamble['num_errors'] = int(ps[24:26], 16)

	return preamble, errs

def parse_current_info(ps):
	current_info = {}
	current_info['time_to_flash'] = int(ps[26:28], 16)
	current_info['boot_count'] = int(ps[28:30], 16)
	current_info['L1_REF'] = untruncate(int(ps[30:32], 16), "S_LREF")
	current_info['L2_REF'] = untruncate(int(ps[32:34], 16), "S_LREF")
	current_info['L1_SNS'] = l_sns_mV_to_mA(untruncate(hex_string_byte_to_signed_int(ps[34:36]), "S_L_SNS"))
	current_info['L2_SNS'] = l_sns_mV_to_mA(untruncate(hex_string_byte_to_signed_int(ps[36:38]), "S_L_SNS"))
	current_info['L1_TEMP'] = ad590_mV_to_C(untruncate(hex_string_byte_to_signed_int(ps[38:40]), "S_L_TEMP"))
	current_info['L2_TEMP'] = ad590_mV_to_C(untruncate(hex_string_byte_to_signed_int(ps[40:42]), "S_L_TEMP"))
	current_info['PANELREF'] = int((untruncate(int(ps[42:44], 16), "S_PANELREF")-130)*5580/1000)
	current_info['L_REF'] = int((untruncate(int(ps[44:46], 16), "S_LREF")-50)*2717/1000)

	bat_digsigs_1 = int(ps[46:48],16)
	bat_digsigs_2 = int(ps[48:50],16)
	parse_dig_sigs(bat_digsigs_1, bat_digsigs_2, current_info)

	current_info['LF1REF'] = untruncate(int(ps[50:52], 16), "S_LF_VOLT")
	current_info['LF2REF'] = untruncate(int(ps[52:54], 16), "S_LF_VOLT")
	current_info['LF3REF'] = untruncate(int(ps[54:56], 16), "S_LF_VOLT")
	current_info['LF4REF'] = untruncate(int(ps[56:58], 16), "S_LF_VOLT")

	return current_info

def parse_attitude_data(ps):
	data = []
	start = DATA_SECTION_START_BYTE
	for i in range(0, ATTITUDE_BATCHES_PER_PACKET):
		cur = {}
		cur['IR_FLASH_OBJ'] = ir_raw_to_C(hex_to_int_le(ps[start:start+4]+'0000'))
		cur['IR_SIDE1_OBJ'] = ir_raw_to_C(hex_to_int_le(ps[start+4:start+8]+'0000'))
		cur['IR_SIDE2_OBJ'] = ir_raw_to_C(hex_to_int_le(ps[start+8:start+12]+'0000'))
		cur['IR_RBF_OBJ'] = ir_raw_to_C(hex_to_int_le(ps[start+12:start+16]+'0000'))
		cur['IR_ACCESS_OBJ'] = ir_raw_to_C(hex_to_int_le(ps[start+16:start+20]+'0000'))
		cur['IR_TOP1_OBJ'] = ir_raw_to_C(hex_to_int_le(ps[start+20:start+24]+'0000'))

		pd_1 = int(ps[start+24:start+26],16)
		pd_2 = int(ps[start+26:start+28],16)

		cur['PD_FLASH'] = (pd_1 >> 6) & (0x03)
		cur['PD_SIDE1'] = (pd_1 >> 4) & (0x03)
		cur['PD_SIDE2'] = (pd_1 >> 2) & (0x03)
		cur['PD_ACCESS'] = (pd_1 >> 0) & (0x03)
		cur['PD_TOP1'] = (pd_2 >> 6) & (0x03)
		cur['PD_TOP2'] = (pd_2 >> 4) & (0x03)

		cur['accelerometer1X'] = acc_raw_to_g(untruncate(int(ps[start+28:start+30], 16), "S_ACCEL"))*-1
		cur['accelerometer1Z'] = acc_raw_to_g(untruncate(int(ps[start+30:start+32], 16), "S_ACCEL"))*-1
		cur['accelerometer1Y'] = acc_raw_to_g(untruncate(int(ps[start+32:start+34], 16), "S_ACCEL"))

		cur['accelerometer2X'] = acc_raw_to_g(untruncate(int(ps[start+34:start+36], 16), "S_ACCEL"))*-1
		cur['accelerometer2Z'] = acc_raw_to_g(untruncate(int(ps[start+36:start+38], 16), "S_ACCEL"))*-1
		cur['accelerometer2Y'] = acc_raw_to_g(untruncate(int(ps[start+38:start+40], 16), "S_ACCEL"))

		cur['gyroscopeX'] = gyro_raw_to_dps(untruncate(int(ps[start+40:start+42], 16), "S_GYRO"))*-1
		cur['gyroscopeZ'] = gyro_raw_to_dps(untruncate(int(ps[start+42:start+44], 16), "S_GYRO"))*-1
		cur['gyroscopeY'] = gyro_raw_to_dps(untruncate(int(ps[start+44:start+46], 16), "S_GYRO"))

		cur['magnetometer1Z'] = mag_raw_to_mG(untruncate(int(ps[start+46:start+48], 16), "S_MAG"))
		cur['magnetometer1X'] = mag_raw_to_mG(untruncate(int(ps[start+48:start+50], 16), "S_MAG"))*-1
		cur['magnetometer1Y'] = mag_raw_to_mG(untruncate(int(ps[start+50:start+52], 16), "S_MAG"))*-1

		cur['magnetometer2Z'] = mag_raw_to_mG(untruncate(int(ps[start+52:start+54], 16), "S_MAG"))
		cur['magnetometer2X'] = mag_raw_to_mG(untruncate(int(ps[start+54:start+56], 16), "S_MAG"))*-1
		cur['magnetometer2Y'] = mag_raw_to_mG(untruncate(int(ps[start+56:start+58], 16), "S_MAG"))*-1

		cur['timestamp'] = hex_to_int_le(ps[start+58:start+66])
		cur['data_hash'] = ps[start:start+66]

		data.append(cur)
		start += 66
	return data

def parse_idle_data(ps):
	data = []
	start = DATA_SECTION_START_BYTE
	for i in range(0, IDLE_BATCHES_PER_PACKET):
		cur = {}
		event_history = int(ps[start:start+2],16)
		parse_event_history(event_history, cur)

		cur['L1_REF'] = untruncate(int(ps[start+2:start+4], 16), "S_LREF")
		cur['L2_REF'] = untruncate(int(ps[start+4:start+6], 16), "S_LREF")
		cur['L1_SNS'] = l_sns_mV_to_mA(untruncate(hex_string_byte_to_signed_int(ps[start+6:start+8]), "S_L_SNS"))
		cur['L2_SNS'] = l_sns_mV_to_mA(untruncate(hex_string_byte_to_signed_int(ps[start+8:start+10]), "S_L_SNS"))
		cur['L1_TEMP'] = ad590_mV_to_C(untruncate(hex_string_byte_to_signed_int(ps[start+10:start+12]), "S_L_TEMP"))
		cur['L2_TEMP'] = ad590_mV_to_C(untruncate(hex_string_byte_to_signed_int(ps[start+12:start+14]), "S_L_TEMP"))
		cur['PANELREF'] = int((untruncate(int(ps[start+14:start+16], 16), "S_PANELREF")-130)*5580/1000)
		cur['L_REF'] = int((untruncate(int(ps[start+16:start+18], 16), "S_LREF")-50)*2717/1000)

		bat_digsigs_1 = int(ps[start+18:start+20],16)
		bat_digsigs_2 = int(ps[start+20:start+22],16)
		parse_dig_sigs(bat_digsigs_1, bat_digsigs_2, cur)

		cur['RAD_TEMP'] = int(untruncate(int(ps[start+22:start+24], 16), "S_RAD_TEMP")/10)
		cur['IMU_TEMP'] = (untruncate(int(ps[start+24:start+26], 16), "S_IMU_TEMP")) / 333.87 + 21

		cur['IR_FLASH_AMB'] = ir_raw_to_C(untruncate(int(ps[start+26:start+28], 16), "S_IR_AMB"))
		cur['IR_SIDE1_AMB'] = ir_raw_to_C(untruncate(int(ps[start+28:start+30], 16), "S_IR_AMB"))
		cur['IR_SIDE2_AMB'] = ir_raw_to_C(untruncate(int(ps[start+30:start+32], 16), "S_IR_AMB"))
		cur['IR_RBF_AMB'] = ir_raw_to_C(untruncate(int(ps[start+32:start+34], 16), "S_IR_AMB"))
		cur['IR_ACCESS_AMB'] = ir_raw_to_C(untruncate(int(ps[start+34:start+36], 16), "S_IR_AMB"))
		cur['IR_TOP1_AMB'] = ir_raw_to_C(untruncate(int(ps[start+36:start+38], 16), "S_IR_AMB"))

		cur['timestamp'] = hex_to_int_le(ps[start+38:start+46])
		cur['data_hash'] = ps[start:start+46]

		data.append(cur)
		start += 46
	return data

def parse_flash_burst_data(ps):
	data = {}
	burst = [dict() for x in range(FLASHBURST_BATCHES_PER_PACKET)]
	start = DATA_SECTION_START_BYTE
	data['data_hash'] = ps[start:start+302]
	for i in range(0, FLASHBURST_BATCHES_PER_PACKET):
		burst[i]['LED1TEMP'] = ad590_mV_to_C(untruncate(hex_string_byte_to_signed_int(ps[start:start+2]), "S_LED_TEMP_FLASH"))
		burst[i]['LED2TEMP'] = ad590_mV_to_C(untruncate(hex_string_byte_to_signed_int(ps[start+2:start+4]), "S_LED_TEMP_FLASH"))
		burst[i]['LED3TEMP'] = ad590_mV_to_C(untruncate(hex_string_byte_to_signed_int(ps[start+4:start+6]), "S_LED_TEMP_FLASH"))
		burst[i]['LED4TEMP'] = ad590_mV_to_C(untruncate(hex_string_byte_to_signed_int(ps[start+6:start+8]), "S_LED_TEMP_FLASH"))
		start += 8

	for i in range(0, FLASHBURST_BATCHES_PER_PACKET):
		burst[i]['LF1_TEMP'] = ad590_mV_to_C(untruncate(hex_string_byte_to_signed_int(ps[start:start+2]), "S_LF_TEMP"))
		burst[i]['LF3_TEMP'] = ad590_mV_to_C(untruncate(hex_string_byte_to_signed_int(ps[start+2:start+4]), "S_LF_TEMP"))
		start += 4

	for i in range(0, FLASHBURST_BATCHES_PER_PACKET):
		burst[i]['LFB1SNS'] = lfbsns_mV_to_mA(untruncate(hex_string_byte_to_signed_int(ps[start:start+2]), "S_LF_SNS_FLASH"))
		burst[i]['LFB1OSNS'] = lfbosns_mV_to_mA(untruncate(hex_string_byte_to_signed_int(ps[start+2:start+4]), "S_LF_OSNS_FLASH"))
		burst[i]['LFB2SNS'] = lfbsns_mV_to_mA(untruncate(hex_string_byte_to_signed_int(ps[start+4:start+6]), "S_LF_SNS_FLASH"))
		burst[i]['LFB2OSNS'] = lfbosns_mV_to_mA(untruncate(hex_string_byte_to_signed_int(ps[start+6:start+8]), "S_LF_OSNS_FLASH"))
		start += 8

	for i in range(0, FLASHBURST_BATCHES_PER_PACKET):
		burst[i]['LF1REF'] = untruncate(hex_string_byte_to_signed_int(ps[start:start+2]), "S_LF_VOLT")
		burst[i]['LF2REF'] = untruncate(hex_string_byte_to_signed_int(ps[start+2:start+4]), "S_LF_VOLT")
		burst[i]['LF3REF'] = untruncate(hex_string_byte_to_signed_int(ps[start+4:start+6]), "S_LF_VOLT")
		burst[i]['LF4REF'] = untruncate(hex_string_byte_to_signed_int(ps[start+6:start+8]), "S_LF_VOLT")
		start += 8

	for i in range(0, FLASHBURST_BATCHES_PER_PACKET):
		burst[i]['LED1SNS'] = led_sns_mV_to_mA(untruncate(hex_string_byte_to_signed_int(ps[start:start+2]), "S_LED_SNS"))
		burst[i]['LED2SNS'] = led_sns_mV_to_mA(untruncate(hex_string_byte_to_signed_int(ps[start+2:start+4]), "S_LED_SNS"))
		burst[i]['LED3SNS'] = led_sns_mV_to_mA(untruncate(hex_string_byte_to_signed_int(ps[start+4:start+6]), "S_LED_SNS"))
		burst[i]['LED4SNS'] = led_sns_mV_to_mA(untruncate(hex_string_byte_to_signed_int(ps[start+6:start+8]), "S_LED_SNS"))
		start += 8

	for i in range(0, FLASHBURST_BATCHES_PER_PACKET):
		burst[i]['gyroscopeX'] = gyro_raw_to_dps(untruncate(int(ps[start:start+2], 16), "S_GYRO"))*-1
		burst[i]['gyroscopeZ'] = gyro_raw_to_dps(untruncate(int(ps[start+2:start+4], 16), "S_GYRO"))*-1
		burst[i]['gyroscopeY'] = gyro_raw_to_dps(untruncate(int(ps[start+4:start+6], 16), "S_GYRO"))
		start += 6
	data['burst'] = burst
	data['timestamp'] = hex_to_int_le(ps[start:start+8])
	return data

def parse_flash_cmp_data(ps):
	data = []
	start = DATA_SECTION_START_BYTE
	for i in range(0, FLASHCMP_BATCHES_PER_PACKET):
		cur = {}
		cur['LED1TEMP'] = ad590_mV_to_C(untruncate(hex_string_byte_to_signed_int(ps[start:start+2]), "S_LED_TEMP_FLASH"))
		cur['LED2TEMP'] = ad590_mV_to_C(untruncate(hex_string_byte_to_signed_int(ps[start+2:start+4]), "S_LED_TEMP_FLASH"))
		cur['LED3TEMP'] = ad590_mV_to_C(untruncate(hex_string_byte_to_signed_int(ps[start+4:start+6]), "S_LED_TEMP_FLASH"))
		cur['LED4TEMP'] = ad590_mV_to_C(untruncate(hex_string_byte_to_signed_int(ps[start+6:start+8]), "S_LED_TEMP_FLASH"))
		cur['LF1_TEMP'] = ad590_mV_to_C(untruncate(hex_string_byte_to_signed_int(ps[start+8:start+10]), "S_LF_TEMP"))
		cur['LF3_TEMP'] = ad590_mV_to_C(untruncate(hex_string_byte_to_signed_int(ps[start+10:start+12]), "S_LF_TEMP"))

		cur['LFB1SNS'] = lfbsns_mV_to_mA(untruncate(hex_string_byte_to_signed_int(ps[start+12:start+14]), "S_LF_SNS_FLASH"))
		cur['LFB1OSNS'] = lfbosns_mV_to_mA(untruncate(hex_string_byte_to_signed_int(ps[start+14:start+16]), "S_LF_OSNS_FLASH"))
		cur['LFB2SNS'] = lfbsns_mV_to_mA(untruncate(hex_string_byte_to_signed_int(ps[start+16:start+18]), "S_LF_SNS_FLASH"))
		cur['LFB2OSNS'] = lfbosns_mV_to_mA(untruncate(hex_string_byte_to_signed_int(ps[start+18:start+20]), "S_LF_OSNS_FLASH"))

		cur['LF1REF'] = untruncate(hex_string_byte_to_signed_int(ps[start+20:start+22]), "S_LF_VOLT")
		cur['LF2REF'] = untruncate(hex_string_byte_to_signed_int(ps[start+22:start+24]), "S_LF_VOLT")
		cur['LF3REF'] = untruncate(hex_string_byte_to_signed_int(ps[start+24:start+26]), "S_LF_VOLT")
		cur['LF4REF'] = untruncate(hex_string_byte_to_signed_int(ps[start+26:start+28]), "S_LF_VOLT")

		cur['LED1SNS'] = led_sns_mV_to_mA(untruncate(hex_string_byte_to_signed_int(ps[start+28:start+30]), "S_LED_SNS"))
		cur['LED2SNS'] = led_sns_mV_to_mA(untruncate(hex_string_byte_to_signed_int(ps[start+30:start+32]), "S_LED_SNS"))
		cur['LED3SNS'] = led_sns_mV_to_mA(untruncate(hex_string_byte_to_signed_int(ps[start+32:start+34]), "S_LED_SNS"))
		cur['LED4SNS'] = led_sns_mV_to_mA(untruncate(hex_string_byte_to_signed_int(ps[start+34:start+36]), "S_LED_SNS"))

		cur['magnetometer1Z'] = mag_raw_to_mG(untruncate(int(ps[start+36:start+38], 16), "S_MAG"))
		cur['magnetometer1X'] = mag_raw_to_mG(untruncate(int(ps[start+38:start+40], 16), "S_MAG"))*-1
		cur['magnetometer1Y'] = mag_raw_to_mG(untruncate(int(ps[start+40:start+42], 16), "S_MAG"))*-1

		cur['timestamp'] = hex_to_int_le(ps[start+42:start+50])
		cur['data_hash'] = ps[start:start+50]
		data.append(cur)
		start += 50
	return data

def parse_low_power_data(ps):
	data = []
	start = DATA_SECTION_START_BYTE
	for i in range(0, LOWPOWER_BATCHES_PER_PACKET):
		cur = {}
		event_history = int(ps[start:start+2],16)
		parse_event_history(event_history, cur)

		cur['L1_REF'] = untruncate(int(ps[start+2:start+4], 16), "S_LREF")
		cur['L2_REF'] = untruncate(int(ps[start+4:start+6], 16), "S_LREF")
		cur['L1_SNS'] = l_sns_mV_to_mA(untruncate(hex_string_byte_to_signed_int(ps[start+6:start+8]), "S_L_SNS"))
		cur['L2_SNS'] = l_sns_mV_to_mA(untruncate(hex_string_byte_to_signed_int(ps[start+8:start+10]), "S_L_SNS"))
		cur['L1_TEMP'] = ad590_mV_to_C(untruncate(hex_string_byte_to_signed_int(ps[start+10:start+12]), "S_L_TEMP"))
		cur['L2_TEMP'] = ad590_mV_to_C(untruncate(hex_string_byte_to_signed_int(ps[start+12:start+14]), "S_L_TEMP"))
		cur['PANELREF'] = int((untruncate(int(ps[start+14:start+16], 16), "S_PANELREF")-130)*5580/1000)
		cur['L_REF'] = int((untruncate(int(ps[start+16:start+18], 16), "S_LREF")-50)*2717/1000)

		bat_digsigs_1 = int(ps[start+18:start+20],16)
		bat_digsigs_2 = int(ps[start+20:start+22],16)
		parse_dig_sigs(bat_digsigs_1, bat_digsigs_2, cur)

		cur['IR_FLASH_OBJ'] = ir_raw_to_C(hex_to_int_le(ps[start+22:start+26]+'0000'))
		cur['IR_SIDE1_OBJ'] = ir_raw_to_C(hex_to_int_le(ps[start+26:start+30]+'0000'))
		cur['IR_SIDE2_OBJ'] = ir_raw_to_C(hex_to_int_le(ps[start+30:start+34]+'0000'))
		cur['IR_RBF_OBJ'] = ir_raw_to_C(hex_to_int_le(ps[start+34:start+38]+'0000'))
		cur['IR_ACCESS_OBJ'] = ir_raw_to_C(hex_to_int_le(ps[start+38:start+42]+'0000'))
		cur['IR_TOP1_OBJ'] = ir_raw_to_C(hex_to_int_le(ps[start+42:start+46]+'0000'))

		cur['gyroscopeX'] = gyro_raw_to_dps(untruncate(int(ps[start+46:start+48], 16), "S_GYRO"))*-1
		cur['gyroscopeZ'] = gyro_raw_to_dps(untruncate(int(ps[start+48:start+50], 16), "S_GYRO"))*-1
		cur['gyroscopeY'] = gyro_raw_to_dps(untruncate(int(ps[start+50:start+52], 16), "S_GYRO"))

		cur['timestamp'] = hex_to_int_le(ps[start+52:start+60])
		cur['data_hash'] = ps[start:start+60]

		data.append(cur)
		start += 60
	return data

# common parsers
def parse_event_history(event_history, obj):
	obj['ANTENNA_DEPLOYED'] = get_bit(event_history, 1)
	obj['LION_1_CHARGED'] = get_bit(event_history, 2)
	obj['LION_2_CHARGED'] = get_bit(event_history, 3)
	obj['LIFEPO4_B1_CHARGED'] = get_bit(event_history, 4)
	obj['LIFEPO4_B2_CHARGED'] = get_bit(event_history, 5)
	obj['FIRST_FLASH'] = get_bit(event_history, 6)
	obj['PROG_MEM_REWRITTEN'] = get_bit(event_history, 7)


def parse_dig_sigs(bat_digsigs_1, bat_digsigs_2, obj):
	# invert some "that" are active LOW
	obj['L1_RUN_CHG'] = get_bit(bat_digsigs_1, 0)
	obj['L2_RUN_CHG'] = get_bit(bat_digsigs_1, 1)
	obj['LF_B1_RUN_CHG'] = get_bit(bat_digsigs_1, 2)
	obj['LF_B2_RUN_CHG'] = get_bit(bat_digsigs_1, 3)
	obj['LF_B2_CHGN'] = not get_bit(bat_digsigs_1, 4)
	obj['LF_B2_FAULTN'] = not get_bit(bat_digsigs_1, 5)
	obj['LF_B1_FAULTN'] = not get_bit(bat_digsigs_1, 6)
	obj['LF_B1_CHGN'] = not get_bit(bat_digsigs_1, 7)

	obj['L2_ST'] = get_bit(bat_digsigs_2, 0)
	obj['L1_ST'] = get_bit(bat_digsigs_2, 1)
	obj['L1_DISG'] = not get_bit(bat_digsigs_2, 2)
	obj['L2_DISG'] = not get_bit(bat_digsigs_2, 3)
	obj['L1_CHGN'] = not get_bit(bat_digsigs_2, 4)
	obj['L1_FAULTN'] = not get_bit(bat_digsigs_2, 5)
	obj['L2_CHGN'] = not get_bit(bat_digsigs_2, 6)
	obj['L2_FAULTN'] = not get_bit(bat_digsigs_2, 7)

def getErrorStartByte(message_type):
	if (message_type == 'IDLE'):
		return 190*2
	elif (message_type == 'ATTITUDE'):
		return 194*2
	elif (message_type == 'FLASH BURST'):
		return 180*2
	elif (message_type == 'FLASH CMP'):
		return 179*2
	elif (message_type == 'LOW POWER'):
		return 179*2
	return -1

def getNumErrorsInPacket(message_type):
	if (message_type == 'IDLE'):
		return 11
	elif (message_type == 'ATTITUDE'):
		return 9
	elif (message_type == 'FLASH BURST'):
		return 14
	elif (message_type == 'FLASH CMP'):
		return 14
	elif (message_type == 'LOW POWER'):
		return 14
	return -1

def convert_error_timestamp(timestamp_data, packet_timestamp):
	# see https://github.com/BrownaSpaceEngineering/EQUiSatOS/blob/master/EQUiSatOS/EQUiSatOS/src/data_handling/package_transmission.c#L209
	return packet_timestamp - ERROR_TIME_BUCKET_SIZE*timestamp_data

def parse_errors(ps, message_type, packet_timestamp):
	errors = []
	invalid_ecode = False
	invalid_eloc = False
	start = getErrorStartByte(message_type)
	num_errors_in_packet = getNumErrorsInPacket(message_type)
	for i in range(0, num_errors_in_packet):
		cur = {}
		cur['error_code'] = int(ps[start:start+2],16) & 0x7F
		cur['priority_bit'] = get_bit(int(ps[start:start+2],16),7)
		cur['error_location'] = int(ps[start+2:start+4],16)
		cur['timestamp'] = convert_error_timestamp(int(ps[start+4:start+6],16), packet_timestamp)
		cur['error_code_name'] = get_ECODE_name(cur['error_code'])
		if cur['error_code_name'] == INVALID_STR:
			invalid_ecode = True
		cur['error_location_name'] = get_ELOC_name(cur['error_location'])
		if cur['error_location_name'] == INVALID_STR:
			invalid_eloc = True
		# represent packet uniquely as its raw bytes, except use the fully qualified timestamp
		cur['data_hash'] = ps[start:start+4] + int_to_hex(cur['timestamp'])
		errors.append(cur)
		start += 6

	parse_errs = []
	if invalid_eloc:
		parse_errs.append(PARSE_ERROR.INVALID_ELOC)
	if invalid_ecode:
		parse_errs.append(PARSE_ERROR.INVALID_ECODE)
	return errors, parse_errs

def parse_data_section(message_type, ps):
	if (message_type == 'IDLE'):
		return parse_idle_data(ps)
	elif (message_type == 'ATTITUDE'):
		return parse_attitude_data(ps)
	elif (message_type == 'FLASH BURST'):
		return parse_flash_burst_data(ps)
	elif (message_type == 'FLASH CMP'):
		return parse_flash_cmp_data(ps)
	elif (message_type == 'LOW POWER'):
		return parse_low_power_data(ps)
	return {}


def parse_packet(ps):
	# with or without parity bytes
	if (len(ps) != 510 and len(ps) != 446):
		return {}, [PARSE_ERROR.WRONG_SIZE]

	# ensure that it is a string
	if type(ps) == bytes:
		ps = ps.decode('ascii')

	packet = {}
	parse_errs = []
	packet['preamble'], preamble_err = parse_preamble(ps)
	parse_errs.append(preamble_err)
	packet['current_info'] = parse_current_info(ps)
	# add the timestamp, for consistency
	packet['current_info']['timestamp'] = packet['preamble']['timestamp']

	message_type = packet['preamble']['message_type']
	if message_type != INVALID_STR:
		packet['data'] = parse_data_section(message_type, ps)
		packet['errors'], error_err = parse_errors(ps, message_type, packet['preamble']['timestamp'])
		parse_errs.append(error_err)
	else:
		packet['data'] = {}
		packet['errors'] = {}

	# enforcing more consistency
	packet['error_codes'] = packet['errors']

	return packet, parse_errs

def find_packets(file):
	with open(file, 'r') as f:
		dump_str = f.read().replace('\n', '')
		packets = re.findall("(574c39585a45.{498})", dump_str)
		return packets

def gen_random_buf():
	return binascii.hexlify(bytearray([random.randint(0, 255) for i in range(255)]))

def main():
	attitude = "574c39585a457d6e000021a5092702dfde585104042754e0f1aeb1b1b2e339ba39bf39af39a839173a5609823f80823f817f7f80777879777879e46a0000dd39bd39cb39af39b7390b3a5609823f81823f807f7f8077787977787970660000d439bb39c639a139ac39033a5609823f80823f807f7f80777879777879fc610000cf39b439bf399b39a639ff395609823f81823f807f7f80777879777879885d0000d539ac39ac399b39a939fb395609823f80823f807f7f8077787977787914590000a732529b2a569c2a5608150008155a9c295a9b305e9b305ea23e5e0000b8bf966e88d0864f8bc4b68a23f6a54b585f5f843d9dded0c2e252bdbe1ebd85"
	idle = "574c39585a455136000020a10b1302dee4515d04042854f0b2afb3aeb13edfe3515f04042854f0b28f5a5757585657588d3400003ee2df5d5104042854f0e18f5a575758565758453100003edfe3515c04042854f0b28f5a575758565758fd2d00003ee1e4515f04042854f0b28f5a575758565758b52a00003ee3e05e5104042855f0e18f5a5757585657586d2700003edfe3515c04042854f0b28f5a575758565758252400003edfe337600404274ef0b28f5a575758565758dd20000008152a9c292a9b302e9b302ea23e2ec63e2ec63e2ea23e2e9b29019b291a9b2a0230f07b4a31312c9cf5121ed6feccc6d0181e9ebe63eba5e6b3d895eeb9f5c2f1"
	fb1 = "574c39585a454100000022970e3b02e2e05c5104042855f0e1b1b4b1b404040404040404040404040404040404040404040304040404040404040404040404040404040404040403060303d039c846d139c945d139c945d139c9450306030303060303b1b4b1b4a2a2a2a2a2a1a0a2a1a29ea1a2a29ea1afb3b3b2b1b3b0b30202020044634e3f47564e3f5b58493f42534e3c02020200020202007f7f807f7f807f7f807f7f807f7f807f7f807f7f80400000009b30009b3000a23e00c63e00c63e00a23e009b30009b3000a23e00c63e00c63e00a23e009b30009b300000c51d74120214a6769f810b5aa75f29027a5b147de21add293392058b7ef1cc0d"
	fb2 = "574c39585a454100000022970e3b02e2e05c5104042855f0e1b1b4b1b404040404040404040404040404040404040404040404040404040404040404040404040404040404040403060303d039c846d139c945d139c945d039c9450306030303060303b1b3b1b4a2a2a3a0a2a1a1a1a1a0a1a19fa19ea1b0b3afb2b2b3aeb4000000005365493f445b473f42564937425d493f00000000000000007f7f807f7f807f7f807f7f807f7f807f7f807f7f8043000000a23e00c63e00c63e00a23e009b30009b3000a23e00c63e00c63e00a23e009b30009b3000a23e00c63e000086738d6760a85099c7a5b6e5b992a95cc963c77022f07115f2c0e14e89cc0d1a"
	fc1 = "574c39585a45d903000023960e2702e1e05a5104042855f0e1b1b3afb339483b300404a72ea138a3a4a3a439483b30777879c703000039483b300404a72ea138a3a4a3a439483b30777879c703000039483b300404a72ea138a3a4a3a439483b30777879c703000039483b300404a72ea138a3a4a3a439483b30777879c703000039483b300404a72ea138a3a4a3a439483b30777879c703000039483b300404a72ea138a3a4a3a439483b30777879c7030000a23e039b30039b3003a23e03c63e03c63e03a23e039b30039b3003a23e03c63e03c63e03a23e039b30030000152fafcdb4ee495ef2969c2216be05da81ca5049c402dbbcd21726b2101c06a4"
	fc2 = "574c39585a458971000023960e2702e1dd605104042854f0e1adb1b0b23d483b300404a92ca238a3a3a2a43d483b30777879877000003a483b300404a82da336a3a3a1a43a483b30777879c76c00003844392d0404a92ca334a3a3a2a33844392d777879076900003947392d0404aa2ba335a2a3a1a43947392d777879476500003341362a0404aa2ba632a2a3a0a33341362a777879876100003541372b0404aa2ba532a3a3a0a33541372b777879c75d0000a23e60c63e60c63e60a23e609b291c9b294c9b2aff9c2a0c9c291ca732549b2a589c2a5808150108155c00002f20f771dba3610d533bf1305a4f516b748f0bcbb7a94be21c791449407d0df8"
	lp1 = "574c39585a45660000002c960e13018f38585904040057faffe7e7e7e71e9037585904040057f2f69a399a3987399b39a6398a397f7f80650000001ea327585904040058f2f69d399d398d39a039a8398c397f7f80510000001eb617585904040058f2f69839983989399a39a8398c397f7f803f0000001eb617585904040058f2f69e399e398c399739ab3989397f7f803e0000001eb716585904040058f2f6983998398c399839a8398c397f7f803d000000b44b00b34b00c64000a24000a240000815000815009b1aff9b2bff9b2aff9b1a000e02009b2b009b2a0000000000000000000000000000000000000000000000000000000000000000000000"
	lp2 = "574c39585a45f60b00002c960eff01b7c5585904040057e8ffe7e747471eb8c5585904040057e8f6a839a839a439ac39b739a4397f7f80f50b00001ec1c5585804040058e8f6a839a839a339a939ba39a4397f7f80e10b00001eb8c5585904040057e8f6a839a839a439ac39b739a4397f7f80f50b00001ec1c5585804040058e8f6a839a839a339a939ba39a4397f7f80e10b00001ecbc5585804040057e8f6a439a439a339a839ba39a0397f7f80cd0b00009b30079b31079c3107254607a73307a72907364e08b44c08ab34083b4b080815089b1aff9b2bff9b2aff00000000000000000000000000000000000000000000000000000000000000000000"
	test = "574c39585a45671600002c9618ff04e1e25d5803032856f0e2c6b2a0b20ee3e35d5804042856f0e200000000c339b8390000b5397f7f80661600000ee3e35d5804042856f0e200000000c339b8390000b5397f7f80661600000ee3e35d5804042856f0e200000000c339b8390000b5397f7f80661600000ee3e35d5804042856f0e200000000c339b8390000b5397f7f80661600000ee3e35d5804042856f0e200000000c339b8390000b5397f7f80661600009c30009b2a009b2f009c2e009b2a009c30009b2f009c2e00573b00323d00323d06573b07b54a090e050000003f73fed7e2e664ec3eea86bc64849d141afd525558ca00a32d87879a23043592"
	packets = [attitude, idle, fb1, fb2, fc1, fc2, lp1, lp2, test]

	if (len(sys.argv) < 2):
		for pkt in packets:
			print(json.dumps(parse_packet(pkt)[0]))
		# for i in range(10000):
		# 	print(parse_packet(gen_random_buf()))
	else:
		for x in sys.argv[1:]:
			packets = find_packets(x)
			for packet in packets:
				print(parse_packet(packet))

if __name__ == "__main__":
	main()
