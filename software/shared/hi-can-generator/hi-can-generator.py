import sys
import json

def parse_enum_class(enum_class, key):
    parsed_dict = {}
    # Split the lines apart
    for line in enum_class.split("\n"):
        # Disregard empty strings and whitespace
        if not (len(line) == 0 or line.isspace()):
            # Get rid of leading whitespace and trailing commas
            stripped_line = line.lstrip().split(",")[0]
            # The name and ID are split by an equals sign
            name, id_string = stripped_line.split(" = ")
            id_number = int(id_string, 0)
            # Add the pair to a dict
            parsed_dict.update({id_number: {key: name}})
    return parsed_dict


def get_enum_class(token, name):
    enum_string = (  # Get everything after the enum class
        token.split(f"enum class {name}")[1]
        # Get everything after the first '{'
        .split("{")[1]
        # Get everything before the first '}'
        .split("}")[0]
    )
    return enum_string


# This is converted from string to int to ensure that hex values are shown
# properly even though it will get converted back to a string when converting
# the dict to a json **Cries in inefficiency of strings**
def get_id(token, name):
    id_number = int(token.split(f" {name}_ID = ")[1].split(";")[0], 0)
    return id_number


arguments = sys.argv
argc = len(arguments)
if argc == 2:
    hi_can_address_filepath = sys.argv[1]
else:
    raise Exception(
        "ERROR: Please supply the file path of hi_can_address.hpp. Args are",
        arguments,
    )

hi_can_address_content = ""

try:
    with open(hi_can_address_filepath, "r") as hi_can_address_file:
        hi_can_address_content = hi_can_address_file.read()
except Exception as err:
    print(
        "ERROR while opening and reading hi_can_address.hpp at ",
        hi_can_address_filepath,
        " error is: ",
        err,
    )
    quit

if hi_can_address_content == "":
    raise Exception("ERROR hi_can_address.hpp file is empty")

# We need to get rid of comments in case stuff like "namespace legacy"
# gets messed up with a comment that include the word legacy
hi_can_address_content_no_comments = "\n".join(
    [line for line in hi_can_address_content.split("\n") if line.lstrip()[0:2] != "//"]
)

hi_can_address_tokenized = hi_can_address_content_no_comments.split("namespace ")
system = {}
systems = {}
system_name = ""
system_id = -1
subsystem_id = 0
device_id = 0

for token in hi_can_address_tokenized:
    # Legacy is a bitch and a half to parse so imma ignore it 🖕
    if token.find("legacy") != -1:
        break
    elif token.find(" GROUP_ID") != -1:
        group_name = token.split()[0]
        group_id = get_id(token, "GROUP")
        group = {group_id: {"GROUP_NAME": group_name}}
        system[system_id][subsystem_id][device_id].update(group)
        if token.find("enum class parameter") != -1:
            parameters = parse_enum_class(
                get_enum_class(token, "parameter"),
                "PARAMETERS",
            )
            system[system_id][subsystem_id][device_id][group_id].update(parameters)
    elif token.find(" DEVICE_ID") != -1:  # We're in a device namespace
        device_name = token.split()[0]
        device_id = get_id(token, "DEVICE")
        device = {device_id: {"DEVICE_NAME": device_name}}
        system[system_id][subsystem_id].update(device)
        if token.find("enum class group") != -1:
            groups = parse_enum_class(
                get_enum_class(token, "group"),
                "GROUP_NAME",
            )
            system[system_id][subsystem_id][device_id].update(groups)
            parameters_for_groups = {}
            if token.split("enum class group")[1].find("enum class ") != -1:
                for parameter_enum in token.split("enum class group")[1].split(
                    "enum class "
                )[1:]:
                    parameters = parse_enum_class(
                        parameter_enum.split("{")[1].split("}")[0], "PARAMETER_NAME"
                    )
                    parameter_name = parameter_enum.split()[0]
                    parameters_for_groups[parameter_name.upper().split("_")[0]] = (
                        parameters
                    )
                if list(system.values())[0]["SYSTEM_NAME"] == "excavation":
                    for key, group in groups.items():
                        type_of_parameter = group["GROUP_NAME"][0:4]
                        if type_of_parameter == "BANK":
                            group.update(parameters_for_groups["BANK"])
                        elif (
                            type_of_parameter == "LIFT"
                            or type_of_parameter == "TILT"
                            or type_of_parameter == "JAWS"
                        ):
                            group.update(parameters_for_groups["ACTUATOR"])
                        elif type_of_parameter == "MAGN":
                            group.update(parameters_for_groups["MAGNET"])
            system[system_id][subsystem_id][device_id].update(groups)
    elif token.find(" SUBSYSTEM_ID") != -1:  # We're in a SUBSYSTEM namespace.
        subsystem_name = token.split()[0]
        subsystem_id = get_id(token, "SUBSYSTEM")
        subsystem = {subsystem_id: {"SUBSYSTEM_NAME": subsystem_name}}
        system[system_id].update(subsystem)
        if system_name == "drive":
            # The VESCs are weird and I don't want to rewrite the code for them
            # The VESCs want the wheel id as the last byte, but we've
            # called it the device and just converted it to the last byte.
            # It's inconsistent with the rest of the naming, so I have to
            # do this part manually
            device_id = 0
            device = {device_id: {"DEVICE_NAME": "vesc"}}
            system[system_id][subsystem_id].update(device)
            groups = parse_enum_class(
                get_enum_class(token, "command_id"),
                "GROUP_NAME",
            )
            system[system_id][subsystem_id][device_id].update(groups)
            parameters = parse_enum_class(
                get_enum_class(token, "device"),
                "PARAMETERS",
            )
        elif token.find("enum class device") != -1:
            # This namespace has an "enum class device", so we should go
            # through the enum to find each device
            devices = parse_enum_class(
                get_enum_class(token, "device"),
                "DEVICES",
            )
            system[system_id][subsystem_id].update(dict(devices))
    elif token.find(" SYSTEM_ID") != -1:
        # We're in a SYSTEM namespace. Note the space at the front ensures
        # that we don't have a false positive for SUBSYSTEM_ID
        if system_id != -1:
            systems.update(system)
        system_name = token.split()[0]
        system_id = get_id(token, "SYSTEM")
        system = {system_id: {"SYSTEM_NAME": system_name}}
    else:
        if token.find(" SYSTEM_ADDRESS_BITS") != -1:
            system_address_bits = token.split("SYSTEM_ADDRESS_BITS = ")[1].split(";")[0]
            systems.update({"SYSTEM_ADDRESS_BITS": system_address_bits})

        if token.find(" SUBSYSTEM_ADDRESS_BITS") != -1:
            system_address_bits = token.split("SUBSYSTEM_ADDRESS_BITS = ")[1].split(
                ";"
            )[0]
            systems.update({"SUBSYSTEM_ADDRESS_BITS": system_address_bits})
        if token.find("DEVICE_ADDRESS_BITS") != -1:
            system_address_bits = token.split("DEVICE_ADDRESS_BITS = ")[1].split(";")[0]
            systems.update({"DEVICE_ADDRESS_BITS": system_address_bits})
        if token.find("GROUP_ADDRESS_BITS") != -1:
            system_address_bits = token.split("GROUP_ADDRESS_BITS = ")[1].split(";")[0]
            systems.update({"GROUP_ADDRESS_BITS": system_address_bits})
        if token.find("PARAM_ADDRESS_BITS") != -1:
            system_address_bits = token.split("PARAM_ADDRESS_BITS = ")[1].split(";")[0]
            systems.update({"PARAMETER_ADDRESS_BITS": system_address_bits})


flat_lut = {}
sys_bits = int(systems["SYSTEM_ADDRESS_BITS"])
sub_bits = int(systems["SUBSYSTEM_ADDRESS_BITS"])
dev_bits = int(systems["DEVICE_ADDRESS_BITS"])
grp_bits = int(systems["GROUP_ADDRESS_BITS"])
par_bits = int(systems["PARAMETER_ADDRESS_BITS"])


for sys_id, sys_val in systems.items():
    if not isinstance(sys_id, int):
        continue
    sys_name = sys_val["SYSTEM_NAME"]
    is_legacy_sys = "legacy" in sys_name.lower()

    for sub_id, sub_val in sys_val.items():
        if not isinstance(sub_id, int):
            continue
        sub_name = sub_val["SUBSYSTEM_NAME"]

        for dev_id, dev_val in sub_val.items():
            if not isinstance(dev_id, int):
                continue
            dev_name = dev_val.get("DEVICE_NAME") or dev_val.get("DEVICES")
            if dev_name is None:
                continue

            for grp_id, grp_val in dev_val.items():
                if not isinstance(grp_id, int):
                    continue
                grp_name = grp_val["GROUP_NAME"]

                has_parameter = False

                for par_id, par_val in grp_val.items():
                    if not isinstance(par_id, int):
                        continue
                    if not isinstance(par_val, dict):
                        continue
                    par_name = par_val.get("PARAMETER_NAME")
                    if par_name is None:
                        continue
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

                if not has_parameter: # Sometimes there are no parameters? K watevrrrr just ignore em
                    can_id = (
                        (sys_id << (sub_bits + dev_bits + grp_bits + par_bits))
                        | (sub_id << (dev_bits + grp_bits + par_bits))
                        | (dev_id << (grp_bits + par_bits))
                        | (grp_id << par_bits)
                    )
                    constructor = (
                        f"{'legacy::address_t' if is_legacy_sys else 'standard_address_t'}"
                        f"({sys_id}, {sub_id}, {dev_id}, {grp_id}, 0)"
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