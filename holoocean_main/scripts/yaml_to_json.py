import yaml
import json
import sys

def convert_yaml_to_json(yaml_file, json_file):
    with open(yaml_file, 'r') as yf:
        data = yaml.safe_load(yf)
    
    print("running in converter")
    
    with open(json_file, 'w') as jf:
        json.dump(data, jf, indent=4)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: yaml_to_json.py <input_yaml_file> <output_json_file>")
        sys.exit(1)
    
    yaml_file = sys.argv[1]
    json_file = sys.argv[2]
    
    convert_yaml_to_json(yaml_file, json_file)
