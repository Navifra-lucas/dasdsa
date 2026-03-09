#!/usr/bin/env python3
import re
import sys
import yaml
from collections import OrderedDict

# OrderedDict를 일반 dict처럼 출력하게 만드는 클래스
class OrderedDumper(yaml.SafeDumper):
    pass

def _dict_representer(dumper, data):
    return dumper.represent_dict(data.items())

OrderedDumper.add_representer(OrderedDict, _dict_representer)

def parse_msg_file(msg_path):
    pattern = re.compile(r'^int64\s+([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(\d+)$')
    enum_map = OrderedDict()
    with open(msg_path, 'r') as f:
        for line in f:
            line = line.strip()
            match = pattern.match(line)
            if match:
                name, value = match.groups()
                enum_map[name] = int(value)
    return enum_map

def generate_yaml(enum_name, enum_map, output_path):
    data = OrderedDict()
    data[enum_name] = enum_map
    with open(output_path, 'w') as f:
        yaml.dump(data, f, Dumper=OrderedDumper, default_flow_style=False, sort_keys=False)

def main():
    if len(sys.argv) != 4:
        sys.exit(1)

    msg_file = sys.argv[1]
    enum_name = sys.argv[2]
    yaml_file = sys.argv[3]

    enum_map = parse_msg_file(msg_file)
    generate_yaml(enum_name, enum_map, yaml_file)

if __name__ == "__main__":
    main()
