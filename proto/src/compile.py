from common import *
from read_json import get_packet_hierarchy, get_payloads_and_enums, get_config
from fields import Fields
import os
import glob

def get_packet_header_str(packet_name, packet_id, packet_fields, kill_hdr):
    # template
    template = """
#pragma once

#include "common.h"

KillHeader

#define PACKET_ID_PacketName packet_id

class PacketPacketName 
{
public:

    void writeRawPacket(Comms::Packet *packet)
    {
        packet->len = 0;
        packet->id = packet_id;
[PACKET_ADD_VALUES]
    }

    static PacketPacketName fromRawPacket(Comms::Packet *packet)
    {
[PACKET_GET_VALUES]
    }

    [BUILDER]

    uint8_t getId() const
    {
        return id;
    }

    using Builder = Builder_<false_list>;

    packet_fields

private:

    uint8_t id = packet_id;

    PacketPacketName(packet_args)
        : packet_constructor_inline
    {}
};

"""
    arg_name_str = packet_fields.get_constructor_args_str()
    
    inline_val_str = packet_fields.get_constructor_inline_str()

    field_decl_str = packet_fields.get_fields_decl_str()

    template = substitute(template, [
        ("PacketName", packet_name),
        ("packet_id", str(packet_id)),
        ("packet_args", arg_name_str),
        ("packet_constructor_inline", inline_val_str),
        ("packet_fields", field_decl_str),
        ("[BUILDER]", packet_fields.build_Builder("Packet"+packet_name).replace("\n","    \n")),
        ("false_list", ", ".join(["false"] * packet_fields.num_fields)),
        ("KillHeader", kill_hdr),
        ("[PACKET_ADD_VALUES]", packet_fields.build_packet_add_values()),
        ("[PACKET_GET_VALUES]", packet_fields.build_packet_read_values(packet_name))
    ])

        # no fields...
    if (packet_fields.num_fields == 0):
        template = template.replace("template<>", "").replace("<>", "").replace(" :","").replace("static_", "//static_")

    return template 

def get_packet_file_name(packet_name):
    return f"Packet_{packet_name}.h"

def make_group_header(packet_group_key, packets_for_packet_group):
    
    packet_group_header = "#pragma once\n"

    for packet_name in (packets_for_packet_group[packet_group_key].keys()):
        packet_group_header += f'#include "{get_packet_file_name(packet_name)}"\n'
    
    return packet_group_header


# In; dict of packet_group : {packet1 : {id : [ID], payload : [PAYLOAD]...}...}
# In: dict of payloads; {[NAME] : {type : [TYPE], symbol : [NAME]... }... }
# Out: dict of {}
def create_packet_headers(packet_list, payloads, enums, config, build_path = "proto/include"):
    packet_structs = {}

    for file in glob.glob(os.path.join(build_path, "*")):
        os.remove(file)
    
    # os.system(f"cp ../common/* {build_path}")
    common_header_path = os.path.join(build_path, "common.h")
    with open(common_header_path, "w") as common_file:
        common_file.write("""
#pragma once
#include <cinttypes>
#include <array>
#include <EspComms.h>
""")
        
        common_file.write(f"#define PACKET_SPEC_VERSION {config['version']}\n")

        for enum_name in enums:
            common_file.write(f"\ntypedef enum {enum_name} {{\n")
            enum = enums[enum_name]
            for value_name in enum:
                common_file.write(f"    {value_name} = {enum[value_name]},\n")
            common_file.write(f"}} {enum_name};\n")
        
        for board in config:
            if board == "deviceIds" or board == "version":
                continue
            board_config = config[board]
            for channel in board_config:
                channel_number = channel["channel"]
                channel_name = channel["measure"]
                common_file.write(f"""
#define CHANNEL_{channel_name} {channel_number}
#ifdef BOARD_{board}
#define IS_BOARD_FOR_{channel_name} true
#else
#define IS_BOARD_FOR_{channel_name} false
#endif
""")



    all_rw = []

    for packet_def in packet_list:
        allowed = packet_def['writes'] + packet_def['reads']
        if (type(allowed) != str):
            for i in allowed:
                if (i.replace("*","") not in all_rw):
                    all_rw.append(i.replace("*",""))

    for packet_def in packet_list:


        if ("payload" in packet_def.keys() and packet_def["payload"]):
            packet_payload = payloads[packet_def["payload"]]
        else:
            packet_payload = {}

        # Now struct has id and payload corresponding to the desired thingiemabobber
        packet_fields = Fields(packet_payload)

        # Make the packet
        packet_name = packet_def['name']
        packet_id = packet_def["id"]

        kill_hdr = ""

        allowed = packet_def['writes'] + packet_def['reads']
        if (type(allowed) != str):
            allowed_rw = [i.replace("*", '') for i in allowed]
            disallowed_rw = [i if not (i in allowed_rw) else None for i in all_rw ]

            if "" not in allowed_rw:
                for live in allowed_rw:
                    if (live):
                        kill_hdr += f"#ifndef BOARD_{live}\n"
                kill_hdr += "#error\n"
                for live in allowed_rw:
                    if (live):
                        kill_hdr += f"#endif\n"

        packet_header_str = get_packet_header_str(packet_name, packet_id, packet_fields, kill_hdr)

        packet_header_path = os.path.join(build_path, get_packet_file_name(packet_name))
        open(packet_header_path, "w+").write(packet_header_str)
            


def make_headers():
    
    packets_for_packet_group = get_packet_hierarchy()
    
    payloads, enums = get_payloads_and_enums()

    config = get_config()

    packet_structs = {}

    create_packet_headers(packets_for_packet_group, payloads, enums, config)

make_headers()
