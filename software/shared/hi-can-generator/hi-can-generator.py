import sys
import json
import re


def parse_enum_class(enum_class, key):
    parsed_dict = {}
    for line in enum_class.split("\n"):
        if not (len(line) == 0 or line.isspace()):
            stripped_line = line.lstrip().split(",")[0]
            name, id_string = stripped_line.split(" = ")
            id_number = int(id_string, 0)
            parsed_dict.update({id_number: {key: name}})
    return parsed_dict


def get_enum_class(token, name):
    enum_string = (
        token.split(f"enum class {name}")[1]
        .split("{")[1]
        .split("}")[0]
    )
    return enum_string


def get_id(token, name):
    id_number = int(token.split(f" {name}_ID = ")[1].split(";")[0], 0)
    return id_number


def main():
    arguments = sys.argv
    argc = len(arguments)
    if argc < 2:
        raise Exception(f"ERROR: Please supply the file path of hi_can_address.hpp. Args are: {arguments}")

    hi_can_address_filepath = sys.argv[1]
    output_json_filepath = sys.argv[2] if argc >= 3 else None

    hi_can_address_content = ""

    try:
        with open(hi_can_address_filepath, "r") as hi_can_address_file:
            hi_can_address_content = hi_can_address_file.read()
    except Exception as err:
        print(f"ERROR while opening and reading hi_can_address.hpp at {hi_can_address_filepath}: {err}")
        sys.exit(1)

    if hi_can_address_content == "":
        raise Exception("ERROR hi_can_address.hpp file is empty")


    hi_can_address_content_no_comments = re.sub(
        r"//.*?$|/\*.*?\*/", "", hi_can_address_content, flags=re.MULTILINE | re.DOTALL # use regex to remove single line and multi-line comments
    )

    hi_can_address_tokenized = hi_can_address_content_no_comments.split("namespace ")
    system = {}
    systems = {}
    system_name = ""
    system_id = -1
    subsystem_id = 0
    device_id = 0

    for token in hi_can_address_tokenized:
        if "legacy" in token:
            break
        elif " GROUP_ID" in token:
            group_name = token.split()[0]
            group_id = get_id(token, "GROUP")
            system[system_id][subsystem_id][device_id].update({group_id: {"GROUP_NAME": group_name}})
            if "enum class parameter" in token:
                parameters = parse_enum_class(get_enum_class(token, "parameter"), "PARAMETERS")
                system[system_id][subsystem_id][device_id][group_id].update(parameters)
        elif " DEVICE_ID" in token:
            device_name = token.split()[0]
            device_id = get_id(token, "DEVICE")
            system[system_id][subsystem_id].update({device_id: {"DEVICE_NAME": device_name}})
            if "enum class group" in token:
                groups = parse_enum_class(get_enum_class(token, "group"), "GROUP_NAME")
                system[system_id][subsystem_id][device_id].update(groups)
                parameters_for_groups = {}
                
                # Check for multiple parameter enums within the device namespace
                if token.split("enum class group")[1].find("enum class ") != -1:
                    for parameter_enum in token.split("enum class group")[1].split("enum class ")[1:]:
                        parameters = parse_enum_class(parameter_enum.split("{")[1].split("}")[0], "PARAMETER_NAME")
                        parameter_name = parameter_enum.split()[0].upper()
                        parameters_for_groups[parameter_name.split("_")[0]] = parameters

                    # Generalised Parameter Matching
                    for key, group in groups.items():
                        group_prefix = group["GROUP_NAME"].upper().split("_")[0]
                        
                        # Match prefix (e.g., BANK_1 -> BANK)
                        if group_prefix in parameters_for_groups:
                            group.update(parameters_for_groups[group_prefix])
                        # Match specific actuator mappings (e.g., LIFT/TILT -> ACTUATOR)
                        elif group_prefix in ["LIFT", "TILT", "JAWS"] and "ACTUATOR" in parameters_for_groups:
                            group.update(parameters_for_groups["ACTUATOR"])
                        # Handle Magnet shorthand
                        elif group_prefix == "MAGN" and "MAGNET" in parameters_for_groups:
                            group.update(parameters_for_groups["MAGNET"])

                system[system_id][subsystem_id][device_id].update(groups)
        elif " SUBSYSTEM_ID" in token:
            subsystem_name = token.split()[0]
            subsystem_id = get_id(token, "SUBSYSTEM")
            system[system_id].update({subsystem_id: {"SUBSYSTEM_NAME": subsystem_name}})
            if system_name == "drive":
                device_id = 0
                system[system_id][subsystem_id].update({device_id: {"DEVICE_NAME": "vesc"}})
                groups = parse_enum_class(get_enum_class(token, "command_id"), "GROUP_NAME")
                system[system_id][subsystem_id][device_id].update(groups)
        elif " SYSTEM_ID" in token:
            if system_id != -1:
                systems.update(system)
            system_name = token.split()[0]
            system_id = get_id(token, "SYSTEM")
            system = {system_id: {"SYSTEM_NAME": system_name}}
        else:
            if "SYSTEM_ADDRESS_BITS" in token:
                systems["SYSTEM_ADDRESS_BITS"] = token.split("SYSTEM_ADDRESS_BITS = ")[1].split(";")[0]
            if "SUBSYSTEM_ADDRESS_BITS" in token:
                systems["SUBSYSTEM_ADDRESS_BITS"] = token.split("SUBSYSTEM_ADDRESS_BITS = ")[1].split(";")[0]
            if "DEVICE_ADDRESS_BITS" in token:
                systems["DEVICE_ADDRESS_BITS"] = token.split("DEVICE_ADDRESS_BITS = ")[1].split(";")[0]
            if "GROUP_ADDRESS_BITS" in token:
                systems["GROUP_ADDRESS_BITS"] = token.split("GROUP_ADDRESS_BITS = ")[1].split(";")[0]
            if "PARAM_ADDRESS_BITS" in token:
                systems["PARAMETER_ADDRESS_BITS"] = token.split("PARAM_ADDRESS_BITS = ")[1].split(";")[0]

    # Build flat lookup table
    flat_lut = {}
    sub_bits = int(systems["SUBSYSTEM_ADDRESS_BITS"])
    dev_bits = int(systems["DEVICE_ADDRESS_BITS"])
    grp_bits = int(systems["GROUP_ADDRESS_BITS"])
    par_bits = int(systems["PARAMETER_ADDRESS_BITS"])

    for sys_id, sys_val in systems.items():
        if not isinstance(sys_id, int): continue
        sys_name = sys_val["SYSTEM_NAME"]

        for sub_id, sub_val in sys_val.items():
            if not isinstance(sub_id, int): continue
            sub_name = sub_val["SUBSYSTEM_NAME"]

            for dev_id, dev_val in sub_val.items():
                if not isinstance(dev_id, int): continue
                dev_name = dev_val.get("DEVICE_NAME") or dev_val.get("DEVICES")
                if dev_name is None: continue

                for grp_id, grp_val in dev_val.items():
                    if not isinstance(grp_id, int): continue
                    grp_name = grp_val["GROUP_NAME"]
                    has_parameter = False

                    for par_id, par_val in grp_val.items():
                        if not isinstance(par_id, int) or not isinstance(par_val, dict):
                            continue
                        par_name = par_val.get("PARAMETER_NAME")
                        if par_name is None: continue
                        
                        has_parameter = True
                        can_id = (
                            (sys_id << (sub_bits + dev_bits + grp_bits + par_bits))
                            | (sub_id << (dev_bits + grp_bits + par_bits))
                            | (dev_id << (grp_bits + par_bits))
                            | (grp_id << par_bits)
                            | par_id
                        )

                        flat_lut[f"0x{can_id:08x}"] = {
                            "system": sys_name,
                            "subsystem": sub_name,
                            "device": dev_name,
                            "group": grp_name,
                            "parameter": par_name,
                        }

                    if not has_parameter:
                        can_id = (
                            (sys_id << (sub_bits + dev_bits + grp_bits + par_bits))
                            | (sub_id << (dev_bits + grp_bits + par_bits))
                            | (dev_id << (grp_bits + par_bits))
                            | (grp_id << par_bits)
                        )
                        flat_lut[f"0x{can_id:08x}"] = {
                            "system": sys_name,
                            "subsystem": sub_name,
                            "device": dev_name,
                            "group": grp_name,
                            "parameter": None,
                        }

    systems_json = json.dumps(flat_lut, indent=2)
    print(systems_json)

    if output_json_filepath:
        with open(output_json_filepath, "w") as f:
            f.write(systems_json)
        print(f"JSON written to {output_json_filepath}")


if __name__ == "__main__":
    main()