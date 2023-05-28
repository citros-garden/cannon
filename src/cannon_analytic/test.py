import ast


# def extract_contents(node):
    # try:
        # if isinstance(node, ast.List):
            # return [extract_contents(item) for item in node.elts]
        # elif isinstance(node, ast.Tuple):
            # return tuple(extract_contents(item) for item in node.elts)
        # elif isinstance(node, ast.Dict):
            # return {
                # extract_contents(key): extract_contents(value)
                # for key, value in zip(node.keys, node.values)
            # }
        # elif isinstance(node, ast.Call):
            # pos_args = tuple(extract_contents(arg) for arg in node.args)
            # kw_args = tuple(f"{kw.arg}={extract_contents(kw.value)}" for kw in node.keywords)
            # if isinstance(node.func, ast.Name):
                # return f"<Call: {node.func.id}{pos_args+kw_args}>"
            # elif isinstance(node.func, ast.Attribute):
                # return f"<Call: {node.func.attr}{pos_args+kw_args}>"
            # else:
                # return "<Call>"
        # elif isinstance(node, ast.BinOp):
            # return extract_contents(node.left) + extract_contents(node.right)
        # else:
            # if not isinstance(node, ast.Name):
                # return ast.literal_eval(node)  
            # else: 
                # return global_scope.get(node.id, None)
    # except Exception as e:
        # return None
# 
# 
# 
# Read the contents of the setup.py file
# with open('src/cannon_analytic/setup.py', 'r') as f:
    # source = f.read()
# 
# Define a global scope dictionary to evaluate global variables
# global_scope = {}
# 
# Parse the source code using ast
# tree = ast.parse(source)
# 
# Execute the global variables assignment statements
# for node in tree.body:
    # if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
        # target_name = node.targets[0].id
        # target_value = None
        # if isinstance(node.value, ast.Str):
            # target_value = ast.literal_eval(node.value)
        # elif isinstance(node.value, ast.BinOp):
            # target_value = extract_contents(node.value.left) + extract_contents(node.value.right)
        # else:
            # target_value = node.value
        # 
        # global_scope[target_name] = target_value
# 
# F#ind the setup() function call
# setup_call = None
# for node in ast.walk(tree):
    # if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == 'setup':
        # setup_call = node
        # break
# 
# if setup_call is not None:
    # Extract the parameter names and their values
    # parameters = {}
    # for keyword in setup_call.keywords:
        # parameter_name = keyword.arg
# 
        # try:
            # if isinstance(keyword.value, ast.Str):
                # parameter_value = keyword.value.s
            # elif isinstance(keyword.value, ast.Num):
                # parameter_value = keyword.value.n
            # elif isinstance(keyword.value, ast.Name):
                # parameter_value = global_scope.get(keyword.value.id)
            # elif isinstance(keyword.value, ast.List):
                # parameter_value = [extract_contents(item) for item in keyword.value.elts]
            # elif isinstance(keyword.value, ast.Tuple):
                # parameter_value = tuple(extract_contents(item) for item in keyword.value.elts)
            # elif isinstance(keyword.value, ast.Dict):
                # parameter_value = {extract_contents(key): extract_contents(value)
                                #    for key, value in zip(keyword.value.keys, keyword.value.values)}
            # elif isinstance(keyword.value, ast.BinOp):
                # parameter_value = extract_contents(keyword.value.left) + extract_contents(keyword.value.right)
            # elif isinstance(keyword.value, ast.Call):
                # pos_args = tuple(extract_contents(arg) for arg in keyword.value.args)
                # kw_args = tuple(f"{kw.arg}={extract_contents(kw.value)}" for kw in keyword.value.keywords)
                # if isinstance(keyword.value.func, ast.Name):
                    # parameter_value = f"{keyword.value.func.id}{pos_args+kw_args}"
                # elif isinstance(keyword.value.func, ast.Attribute):
                    # parameter_value = f"{keyword.value.func.attr}{pos_args+kw_args}"
            # handle boolean values
            # elif isinstance(keyword.value, ast.Constant):
                # parameter_value = keyword.value.s
            # else:
                # Handle other cases as needed
                # parameter_value = None
        # except Exception as e:
            # parameter_value = None
# 
        # parameters[parameter_name] = parameter_value
# 
    # Print the parameter names and their values
    # for name, value in parameters.items():
        # print(f'{name}: {value}')
# 
    # node_name = parameters['entry_points']['console_scripts'][0].split('=')[0].strip()
    # entry_point = parameters['entry_points']['console_scripts'][0].split('=')[1].strip()
    # file_name = f"{parameters['entry_points']['console_scripts'][0].split('=')[1].strip().split(':')[0].split('.')[1]}.py" 
    # print(f"\n node_name = {node_name}\n entry_point = {entry_point}\n file_name = {file_name}")
    # 
# else:
    # print('setup() function call not found in the setup.py file')
# 
# 
# 

def extract_contents(node, global_scope):
    """
    Recursive helper function for parsing a variety of objects such as lists, function calls etc.
    Does not handle every case, such as nested functions etc. Returns None on failure.
    """
    if isinstance(node, ast.Str):
        return node.s
    elif isinstance(node, ast.Num):
        return node.n
    elif isinstance(node, ast.Name):
        return global_scope.get(node.id)
    elif isinstance(node, ast.List):
        return [extract_contents(item, global_scope) for item in node.elts]
    elif isinstance(node, ast.Tuple):
        return tuple(extract_contents(item, global_scope) for item in node.elts)
    elif isinstance(node, ast.Dict):
        return {extract_contents(key, global_scope): extract_contents(value, global_scope)
                for key, value in zip(node.keys, node.values)}
    elif isinstance(node, ast.BinOp):
        return extract_contents(node.left, global_scope) + extract_contents(node.right, global_scope)
    elif isinstance(node, ast.Call):
        pos_args = tuple(extract_contents(arg, global_scope) for arg in node.args)
        kw_args = tuple(f"{kw.arg}={extract_contents(kw.value, global_scope)}" for kw in node.keywords)
        if isinstance(node.func, ast.Name):
            return f"{node.func.id}{pos_args+kw_args}"
        elif isinstance(node.func, ast.Attribute):
            return f"{node.func.attr}{pos_args+kw_args}"
    elif isinstance(node, ast.Constant):
        return node.s
    else:
        return None

def extract_setup_parameters(tree, global_scope):
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == 'setup':
            parameters = {}
            for keyword in node.keywords:
                parameter_name = keyword.arg
                parameter_value = extract_contents(keyword.value, global_scope)
                parameters[parameter_name] = parameter_value
            return parameters

    return None

def populate_global_scope(tree):
    global_scope = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            target_name = node.targets[0].id
            target_value = extract_contents(node.value, global_scope)
            global_scope[target_name] = target_value
    return global_scope

def main():
    with open('src/cannon_analytic/setup.py', 'r') as f:
        source = f.read()

    tree = ast.parse(source)
    global_scope = populate_global_scope(tree)
    parameters = extract_setup_parameters(tree, global_scope)
    
    if parameters is not None:
        for name, value in parameters.items():
            print(f'{name}: {value}')

        node_name = parameters['entry_points']['console_scripts'][0].split('=')[0].strip()
        entry_point = parameters['entry_points']['console_scripts'][0].split('=')[1].strip()
        file_name = f"{entry_point.split(':')[0].split('.')[1]}.py" 
        print(f"\n node_name = {node_name}\n entry_point = {entry_point}\n file_name = {file_name}")



def get_file_hierarchy(root_dir, file_list):
    import os
    hierarchy = {}
    file_contents = []
    file_count = {file_name: 0 for file_name in file_list}
    for dirpath, dirnames, filenames in os.walk(root_dir):
        dir_structure = hierarchy
        subdir_path = dirpath[len(root_dir):].split(os.sep)[1:]
        for subdir in subdir_path:
            if subdir not in dir_structure:
                dir_structure[subdir] = {}
            dir_structure = dir_structure[subdir]
        for filename in filenames:
            file_path = os.path.join(dirpath, filename)
            relative_path = file_path[len(root_dir):]
            dir_structure[filename] = relative_path
            if filename in file_list:
                file_count[filename] += 1
                if file_count[filename] > 1:
                    print(f"Duplicate copies of '{filename}' were found.")
                    file_contents[file_list.index(filename)] = f"Duplicate copies of '{filename}' were found."
                else:
                    with open(file_path, 'r') as file:
                        file_contents.append(file.read())
    return hierarchy, file_contents


if __name__ == "__main__":
    #main()
    hierarchy, file_contents = get_file_hierarchy("/workspaces/cannon/src/cannon_analytic", ["setup.py"])
    print(hierarchy)
