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
            name, id_str = stripped_line.split(" = ")
            id_num = int(id_str, 0)
            # Add the pair to a dict
            parsed_dict.update({id_num: {key: name}})
    return parsed_dict


def get_enum_class(token, name):
    enum_str = (  # Get everything after the enum class
        token.split(f"enum class {name}")[1]
        # Get everything after the first '{'
        .split("{")[1]
        # Get everything before the first '}'
        .split("}")[0]
    )
    return enum_str


# This is converted from string to int to ensure that hex values are shown
# properly even though it will get converted back to a string when converting
# the dict to a json **Cries in inefficiency**
def get_id(token, name):
    id_num = int(token.split(f" {name}_ID = ")[1].split(";")[0], 0)
    return id_num


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
    # Implementing legacy will be a pain, because it has different sizes for
    # the address sections. I'm skipping it for now
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
                for param_enum in token.split("enum class group")[1].split(
                    "enum class "
                )[1:]:
                    parameters = parse_enum_class(
                        param_enum.split("{")[1].split("}")[0], "PARAMETER_NAME"
                    )
                    parameter_name = param_enum.split()[0]
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
systems_json = json.dumps(systems)
print(systems_json)
