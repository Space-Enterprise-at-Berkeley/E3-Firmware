from common import *
from uproto_reader import JSONCDecoder
import os
# grab device-packet hierarchy

default_spec_path = "proto/universalproto/"

def get_packet_hierarchy(spec_path = default_spec_path):

    reader = JSONCDecoder()

    # Each packet has a set of allowed packets
    packets_for_packet_group = reader.decode(open(os.path.join(spec_path, "packets.jsonc"), "r").read())
    
    return packets_for_packet_group

def get_payloads_and_enums(spec_path = default_spec_path):
    reader = JSONCDecoder()
    data = reader.decode(open(os.path.join(spec_path, "types.jsonc"), "r").read())

    tbk = []
    enums = {}

    for key in data.keys():
        if (type(data[key]) == dict):
            tbk.append(key)
            enums[key] = data[key]

    [data.pop(key) for key in tbk]

    return data, enums
