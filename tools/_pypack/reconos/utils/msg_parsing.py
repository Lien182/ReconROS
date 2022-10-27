import copy as cp
import re
import os

"""
takes in a list of paths and returns a dictionary with msg definitions
"""
def parse_msg_lib(paths):
    msg_lib = _parse_msg_directory(paths[0])
    for path in paths[1:]:
        temp_lib = _parse_msg_directory(path)
        msg_lib.update(temp_lib)
    return _add_local_key(msg_lib)

"""
takes in a msg description as a dictionary, replaces any nested messages
and sorts into arrays and primitives
"""
def build_msg_dict(msg, msg_lib, primitives_lib):
    unrolled = _deep_replace(msg, msg_lib)
    return _sort_into_datatypes(unrolled, primitives_lib)

"""
Message definitions can contain other msgs either with a path (e.g. geometry_msgs/Pose),
or w/o a path (Pose). This function takes entries with a path and duplicates them w/o
path
"""
def _add_local_key(msg_lib):
    msg_lib_temp = {}
    for key in msg_lib.keys():
        msg_lib_temp[key] = msg_lib[key]
        local_key = re.sub(r'^.*?/', '', key)
        if(local_key != key):
            msg_lib_temp[local_key] = msg_lib[key]
    
    return msg_lib_temp


def _parse_msg_directory(path):
    msg_paths = _get_all_msg_paths(path)
    msg_names = _get_msg_names_from_paths(path + "/", msg_paths)
    msg_datatypes = _get_msg_datatypes_from_paths(path + "/", msg_paths)
    msg_includes = _get_msg_include_paths_from_paths(path + "/", msg_paths)
    msg_lib = {}
    for c,msg_path in enumerate(msg_paths):
        msg_lib[msg_names[c]] = _parse_msg_from_msgfile(msg_path)
        msg_lib[msg_names[c]]["datatype"] = msg_datatypes[c]
        msg_lib[msg_names[c]]["include"] = msg_includes[c]
    return msg_lib

def _get_all_msg_paths(path):
    paths = []
    for root, dirs, files in os.walk(path):
        for file in files:
            if file.endswith(".msg"):
                paths.append(os.path.join(root, file))
    return paths

def _get_msg_names_from_paths(prefix, msg_paths):
    msg_names = []
    for path in msg_paths:
        temp = path.replace(prefix, "")
        temp = temp.replace("msg/", "")
        temp = temp.replace(".msg", "")
        msg_names.append(temp)
    return msg_names

def _get_msg_datatypes_from_paths(prefix, msg_paths):
    msg_datatypes = []
    for path in msg_paths:
        temp = path.replace(prefix, "")
        temp = temp.replace("msgs/", "msgs__")
        temp = temp.replace("msg/", "msg__")
        temp = temp.replace(".msg", "")
        temp2 = "#_" + temp
        msg_datatypes.append(temp2)
    return msg_datatypes

def _get_msg_include_paths_from_paths(prefix, msg_paths):
    msg_includes = []
    for path in msg_paths:
        temp = path.replace(prefix, "")
        temp = temp.replace(".msg", ".h")
        temp2 = "##_" + temp
        temp2 = temp2.lower()
        msg_includes.append(temp2)
    return msg_includes

'''
Parses a .msg file into a dictionary. field names are used as keys,
field types are values. Comments, leading and trailing white spaces
and new lines are skipped
'''
def _parse_msg_from_msgfile(msgpath):
    with open(msgpath) as f:
        lines = [line.rstrip().lstrip().partition('#')[0] for line in f]
        lines = [line for line in lines if len(line)>0]
        
    msg_dict = {}
    for line in lines:
        #datatype, name = str.split(line)
        l = str.split(line)
        datatype = l[0]
        name = l[1]
        msg_dict[name] = datatype
    return msg_dict


'''
Expects a non-nested dict. Apply flatten_dict before
Replaces any value in msg_dict that has a corresponding key in msg_lib, except for arrays.
Values in sub-dictionaries are not replaced
'''
def _replace_values(msg, msg_lib):
    msg_dict = cp.deepcopy(msg)
    for key in msg_dict.keys():
        if(msg_dict[key] in msg_lib.keys()):
            msg_dict[key] = msg_lib[msg_dict[key]]
    return msg_dict
            

'''
Removes all values that are dictionaries and copies the removed dictonaries' entries to dict_.
This only removes one layer of nested dictionaries
'''
def _remove_dicts(dict_):
    d = cp.deepcopy(dict_)
    for key in dict_.keys():
        if (type(dict_[key]) == dict):
            for sub_key in dict_[key].keys():
                new_key =  key + "." + sub_key
                d[new_key] = dict_[key][sub_key]
            del d[key]
    return d


'''
Remove all layers of nesting in dict
'''
def _flatten_dict(dict_):
    while( any(isinstance(i,dict) for i in dict_.values()) ):
        dict_ = _remove_dicts(dict_)
    return dict_


'''
Expects a non-nested dict. Apply flatten_dict before
Replaces any array denoted by [] in msg that has a corresponding key in msg_lib.
Arrays of primitive data types are not replaced!
If no array size is given, a size of 1 is inferred.
Values in sub-dictionaries are not replaced
'''
def _replace_arrays(msg, msg_lib):
    msg_temp = cp.deepcopy(msg)
    for key in msg.keys():
        if(msg[key][-1]=="]"):
            m = re.findall(r'\[[\s+]*(-?\d+)\s*\]',msg[key])
            
            if((m is None) or (len(m) == 0)):
                print("No array size given in msg definition for {}. Inferring 1 as array size".format(msg[key]))
                print(msg[key])
                m = 1
                
            else:
                print(m)
                m = int(m[0])
                print("Found array of size " + str(m))
            temp = re.sub("[\[].*?[\]]", "", msg[key])
            if(temp in msg_lib.keys()):
                del msg_temp[key]
                for k in range(m):
                    new_key = key + "_" + str(k)
                    msg_temp[new_key] = msg_lib[temp]
            else:
                print("No match found in msg_lib for key {}".format(msg[key]))

    return msg_temp

'''
Replaces any value of msg that has a corresponding definition in msg_lib.
This works for arbitrary nested dicts
'''
def _deep_replace(msg, msg_lib):
    msg = _flatten_dict(msg)
    while(set( [re.sub("[\[].*?[\]]", "", val) for val in msg.values() if type(val)!=dict] ) & set(list(msg_lib.keys()))):
        msg = _replace_values(msg, msg_lib)
        msg = _flatten_dict(msg)
        msg = _replace_arrays(msg, msg_lib)
        msg = _flatten_dict(msg)
    return msg

"""
for datatype[], returns 1
for datatype[x], returns x
"""
def _parse_array_size(string):
    
    m = re.findall(r'\[[\s+]*(-?\d+)\s*\]',string)
    if((m is None) or (len(m) == 0)):
        print("No array size given in msg definition for {}. Inferring 1 as array size".format(string))
        m = 10
                
    else:
        print(m)
        m = int(m[0])
        print("Found array of size " + str(m))
        
    return m

"""
for (int_t/uint_t)8-64 returns 8-64,
for string returns 8
"""
def _parse_dwidth(string):

    if(string.startswith("string")):
        width = 8
    else:
        try:
            width = int(re.findall(r'\d+', string)[0])
        except IndexError:
            print("Data width could not be determined for array of type " + string)
            width = 0

    return width
    
    
"""
Sorts a given dictionary into primitives and arrays/strings
Assumes that arrays of ros msgs have been replaced and flattened already
Arrays of primitive data types get replaced
"""
def _sort_into_datatypes(msg, primitive_lib):
    sorted_dict = {}
    
    primitives = []
    arrays = []
    arrays_8 = []
    arrays_16 = []
    arrays_32 = []
    arrays_64 = []
    
    datatype_found = False
    includepath_found = False

    num_msg_elems = 0
     
    for key in msg.keys():
        if(msg[key][-1]=="]" or msg[key] == "string"): # array or string
            print("array found")
            m = _parse_array_size(msg[key])
            num_msg_elems += (m + 2)    # + 2 for size and capacity
            array_dict = {}
            array_dict["name"] = key
            array_dict["size"] = primitive_lib["size_t"]
            array_dict["capacity"] = primitive_lib["size_t"]
            array_dict["dtype"] = re.sub("[\[].*?[\]]", "", msg[key])
            array_dict["dwidth"] = _parse_dwidth(array_dict["dtype"])
            array_dict["num_elems"] = m
            if(array_dict["dwidth"] == 8):
                arrays_8.append(array_dict)
            elif(array_dict["dwidth"] == 16):
                arrays_16.append(array_dict)
            elif(array_dict["dwidth"] == 32):
                arrays_32.append(array_dict)
            elif(array_dict["dwidth"] == 64):
                arrays_64.append(array_dict)
            else:
                arrays.append(array_dict)
            
            
        else: # primitive data type
            if(msg[key] in primitive_lib.keys()):
                primitive = {}
                num_msg_elems += 1
                primitive["name"] = key
                primitive["dtype"] = msg[key]
                primitives.append(primitive)
            else:
                print("no match found for {} in primitive_lib".format(msg[key]))
                
            if(msg[key][0:2] == "#_" and not datatype_found):
                print("msg datatype: ", msg[key])
                sorted_dict["datatype"] = msg[key].replace("#_", "")
                datatype_found = True

            if(msg[key][0:3] == "##_" and not includepath_found):
                print("msg include path: ", msg[key])
                sorted_dict["include"] = msg[key].replace("##_", "")
                includepath_found = True
    
    sorted_dict["Primitives"] = primitives
    sorted_dict["Arrays"] = arrays
    sorted_dict["num_msg_elems"] = num_msg_elems
        
    return sorted_dict


primitive_lib = {}

primitive_lib["uint8"] = "uint8"
primitive_lib["uint16"] = "uint16"
primitive_lib["uint32"] = "uint32"
primitive_lib["uint64"] = "uint64"
primitive_lib["size_t"] = "uint32"

primitive_lib["int8"] = "int8"
primitive_lib["int16"] = "int16"
primitive_lib["int32"] = "int32"
primitive_lib["int64"] = "int64"

primitive_lib["float32"] = "float"
primitive_lib["float64"] = "double"