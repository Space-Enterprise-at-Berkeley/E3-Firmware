from common import *
from uproto_reader import JSONCDecoder
import os
# grab device-packet hierarchy

default_spec_path = "../universalproto/"

def get_packet_hierarchy(spec_path = default_spec_path):

    reader = JSONCDecoder()

    # check existence of packets.jsonc
    if (not os.path.exists(os.path.join(spec_path, "packets.jsonc"))):
        print("Error: packets.jsonc not found in " + spec_path + ". Do you have universalproto cloned in the same directory as this repo?")
        exit(1)

    # Each packet has a set of allowed packets
    packets_for_packet_group = reader.decode(open(os.path.join(spec_path, "packets.jsonc"), "r").read())
    
    return packets_for_packet_group

def get_config(config_name = "config.jsonc", spec_path = default_spec_path):

    reader = JSONCDecoder()

    # check existence of config.jsonc
    if (not os.path.exists(os.path.join(spec_path, config_name))):
        print("Error: " + config_name + " not found in " + spec_path + ". Do you have universalproto cloned in the same directory as this repo?")
        exit(1)

    packets_for_packet_group = reader.decode(open(os.path.join(spec_path, config_name), "r").read())
    
    return packets_for_packet_group

def get_payloads_and_enums(spec_path = default_spec_path):
    reader = JSONCDecoder()

    # check existence of types.jsonc
    if (not os.path.exists(os.path.join(spec_path, "types.jsonc"))):
        print("Error: types.jsonc not found in " + spec_path + ". Do you have universalproto cloned in the same directory as this repo?")
        exit(1)
        
    data = reader.decode(open(os.path.join(spec_path, "types.jsonc"), "r").read())

    tbk = []
    enums = {}

    for key in data.keys():
        if (type(data[key]) == dict):
            tbk.append(key)
            enums[key] = data[key]

    [data.pop(key) for key in tbk]

    return data, enums
