import os
import re

def get_turtlebot_namespace(variable_name):
    home_directory = os.path.expanduser("~")
    bashrc_path = os.path.join(home_directory, '.bashrc')

    variable_value = None

    try:
        with open(bashrc_path, 'r') as bashrc_file:
            for line in bashrc_file:
                match = re.match(r'^\s*export\s+' + variable_name + r'\s*=\s*(.*)\s', line)
                if match:
                    variable_value = match.group(1).strip()
                    break
    except FileNotFoundError:
        print('bashrc file not found')

    return variable_value

if __name__ == '__main__':
    variable_name = "TURTLEBOT_NAMESPACE"
    value = get_turtlebot_namespace(variable_name)

    if value:
        print(f"{variable_name} = {value}")
    else:
        print(f"{variable_name} = not found in bashrc")
