import os

def get_turtlebot_namespace(variable_name):
    home_directory = os.path.expanduser("~")
    bashrc_path = os.path.join(home_directory, '.bashrc')

    variable_value = None

    try:
        with open(bashrc_path, 'r') as bashrc_file:
            for line in bashrc_file:
                if line.startswith('export' + variable_name + '='):
                    variable_value = line.split('=')[1].strip()
                    break
    except FileNotFoundError:
        print('bashrc file not found')

    return variable_value

if __name__ == '__main__':
    variable_name = "MY_VARIABLE"
    value = get_turtlebot_namespace(variable_name)

    if value:
        print(f"{variable_name} = {value}")
    else:
        print(f"{variable_name}= not found in bashrc")
