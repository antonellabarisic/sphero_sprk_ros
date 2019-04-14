#!/usr/bin/python
""" This module is used for utility and helper functions. """

from bluepy.btle import Scanner


def search_for_sphero(second_time=1):
	""" Search for nearby Spheros """
	scanner = Scanner()
	devices = scanner.scan(second_time)
	sphero_list = []
	for dev in devices:
		#get the complete local name
		local_name = dev.getValueText(9)
		if local_name != None:
			if local_name.startswith('SK-'):
				sphero_list.append(dev.addr)
	return sphero_list

def from_bytes (data, big_endian = False):
    """ Convert bytes to int """
    if isinstance(data, str):
    	data = bytearray(data)
    if big_endian:
        data = reversed(data)
    num = 0
    for offset, byte in enumerate(data):
        num += byte << (offset * 8)
    return num

def cal_packet_checksum(arr):
	""" 
	Calculate packet checksum which is the modulo 256 sum of all the bytes
	from the DID through the end of the data payload.
	"""
	value=0
	for a in arr:
		for i in range(0,len(a)):
			if isinstance(a[i], str):
				value += ord(a[i])
			elif isinstance(a[i],int):
				value+=a[i]
	return 255-(value%256)
