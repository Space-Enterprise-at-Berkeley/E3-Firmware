from common import *

type_lut = {
    "u8":"uint8_t",
    "u16":"uint16_t",
    "u32":"uint32_t",
    "u64":"uint64_t",
    "f32":"float"
}

packet_adder_lut = {
    "uint8_t":"packetAddUint8",
    "uint16_t":"packetAddUint16",
    "uint32_t":"packetAddUint32",
    "uint64_t":"packetAddUint64",
    "float":"packetAddFloat"
}

packet_reader_lut = {
    "uint8_t":"packetGetUint8",
    "uint16_t":"packetGetUint16",
    "uint32_t":"packetGetUint32",
    "uint64_t":"packetGetUint64",
    "float":"packetGetFloat"
}

size_lut = {
    "uint8_t":1,
    "uint16_t":2,
    "uint32_t":4,
    "uint64_t":8,
    "float":4
}

def try_lut(val):
    if (val in type_lut.keys()):
        return type_lut[val]
    return val

class Fields():

    # assumes fields has type like 
    #                             [ { "array": 8, "enum": "ACActuatorStates", "type": "u8", "symbol": "states" }, 
    #                               { "array": 8, "enum": "ACActuatorStates", "type": "u8", "symbol": "states" }
    #                             ]

    def __init__(self, fields):

        self.type = []  # Initialize as empty list
        self.enum = []  # Initialize as empty list
        self.array = []  # Initialize as empty list
        self.symbol = []
        self.msymbol = []
        self.noted_type = []
        
        for field in fields:
            self.type.append(try_lut(field["type"]))  # Append to type list
            name = field["symbol"]
            self.symbol.append(name[0].upper() + name[1:])  # cvt to CamelCase
            self.msymbol.append("m_" + self.symbol[-1])  # Append to msymbol list

            self.enum.append(field["enum"]) if "enum" in field.keys() else self.enum.append(None)  # Append to enum list
            self.array.append(field["array"]) if "array" in field.keys() else self.array.append(1)  # Append to array list

        
            # Noted type is what is written as the dtype; uint8_t run() vs RunState run() for RunState as enum
            self.noted_type.append(self.enum[-1] if self.enum[-1] else self.type[-1])

            if (self.array[-1] != 1):
                self.noted_type[-1] = f"std::array<{self.noted_type[-1]}, {self.array[-1]}>"

        self.num_fields = len(self.enum)

    def build_builder_type_str(self, names):
        template_str = ", ".join([str(b).lower() for b in names])
        return f"Builder_<{template_str}>"

    def build_getter(self, idx):

        template = """ 
    Type getSymbol() const {
        return this->Msymbol;
    }
            """
        return substitute(template, [("Type", self.noted_type[idx]),
                                              ("Symbol", self.symbol[idx]),
                                              ("Msymbol", self.msymbol[idx])])

    def build_setter(self, idx):
        name_arr = [f"f{i}set" if i != idx else "true" for i in range(self.num_fields)]
        ret_type = self.build_builder_type_str(name_arr)
        arg_name_str = ", ".join([f"this->{self.msymbol[i]}" if i != idx else "input" for i in range(self.num_fields)])

        template = """
    Ret_type withSymbol(Type input) const 
    {
        static_assert(! fIdxset, "Cannot set field 'Symbol', it is already set");
        return Ret_type(Args);
    }

        """
        return substitute(template, [("Ret_type", ret_type),
                                            ("Type", self.noted_type[idx]),
                                            ("Symbol", self.symbol[idx]),
                                            ("Args", arg_name_str),
                                            ("Idx", str(idx))])
    
    def get_constructor_args_str(self):
        return ", ".join([f"{self.noted_type[i]} {self.symbol[i]}In" for i in range(self.num_fields)])

    def get_constructor_inline_str(self):
        return ", ".join([f"{self.msymbol[i]}({self.symbol[i]}In)" for i in range(self.num_fields)])
    
    def get_fields_decl_str(self):
        return "\n    ".join([f"{self.noted_type[i]} {self.msymbol[i]};" for i in range(self.num_fields)])

    def build_Builder(self, target_packet):
        template_arr = [f"bool f{i}set" for i in range(self.num_fields)]
        
        arg_name_str = self.get_constructor_args_str()
        
        inline_val_str = self.get_constructor_inline_str()

        field_decl_str = self.get_fields_decl_str()
        
        fields_name_str = ", ".join([f"this->{self.msymbol[i]}" for i in range(self.num_fields)])

        template = """ 
template<template_arr>
class Builder_
{
    private:
    field_decl_str

    public:
    Builder_(arg_name_str)
        : inline_val_str
    {}

    Builder_() {}

    target_packet build() const
    {        
        static_assert(all_true, "All fields must be set before building."); // Added static assert
        return target_packet(fields_name_str);
    }

        """
        template = substitute(template,[
                                    ("template_arr", ", ".join(template_arr)),
                                    ("arg_name_str",arg_name_str),
                                    ("inline_val_str",inline_val_str),
                                    ("field_decl_str",field_decl_str),
                                    ("all_true", " && ".join(template_arr).replace("bool", "")),
                                    ("target_packet", target_packet),
                                    ("fields_name_str", fields_name_str)])
        
        for i in range(self.num_fields):

            template += self.build_setter(i)

        template += "\n};"
        
        return template
    
    def build_packet_add_values(self):
        print()
        print(self.type)
        print(self.enum)
        print(self.array)
        print(self.symbol)
        print(self.msymbol)
        print(self.noted_type)
        return "\n".join(["        " + self.get_packet_value_adder(i) for i in range(self.num_fields)])

    def get_packet_value_adder(self, i):
        if self.array[i] > 1:
            return f"for (size_t i = 0; i < {self.array[i]}; i ++) {{\n            Comms::{packet_adder_lut[self.type[i]]}(packet, {self.msymbol[i]}.at(i));\n        }}"
        return f"Comms::{packet_adder_lut[self.type[i]]}(packet, {self.msymbol[i]});"

    def build_packet_read_values(self, class_name):
        output = []
        pos = 0
        for i in range(self.num_fields):
            o, size = self.get_packet_value_reader(i, pos)
            output.append(o)
            pos += size
        params = [f"v_{self.msymbol[i]}" for i in range(self.num_fields)]
        output.append(f"        return Packet{class_name}({', '.join(params)});")
        return "\n".join(output)

    def get_packet_value_reader(self, i, start):
        element_size = size_lut.get(self.type[i], 1)
        cast_start = f"static_cast<{self.enum[i]}>(" if self.enum[i] != None else ""
        cast_end = ")" if self.enum[i] != None else ""
        if self.array[i] > 1:
            return f"""        {self.noted_type[i]} v_{self.msymbol[i]};
        for (size_t i = {start}; i < {start + self.array[i] * element_size}; i += {element_size}) {{
            v_{self.msymbol[i]}[i] = {cast_start}Comms::{packet_reader_lut[self.type[i]]}(packet, i){cast_end};
        }}
""", self.array[i] * element_size
        return f"        {self.noted_type[i]} v_{self.msymbol[i]} = {cast_start}Comms::{packet_reader_lut[self.type[i]]}(packet, {start}){cast_end};", element_size

def test_fields():
    a = Fields([
    { "enum": "SystemMode", "type": "u8", "symbol": "systemMode" },
    { "type": "f32", "symbol": "burnTime" },
    { "type": "u8", "symbol": "nitrousEnable" },
    { "type": "u8", "symbol": "ipaEnable" }
    ])
    print(a.build_Builder("Launch"))
