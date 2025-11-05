#!/usr/bin/env python
import os
import sys
import rospy
from os.path import abspath

if sys.version_info[0] >= 3:
    # Python 3
    import importlib
    importlib.reload(sys)
    # sys.setdefaultencoding('utf-8') # python3默认为utf-8，无需特殊设置
else:
    # Python 2
    reload(sys)
    sys.setdefaultencoding('utf-8')

fprint_file = 'RRLDR_fprintf.log'
bin_file = 'RRLDR_binId4'


def find_file(dir, name):
    for root, dir, filenames in os.walk(dir):
        for filename in filenames:
            if filename.startswith(name):
                return os.path.join(root, filename)
    return ""


def get_location():
    location = rospy.get_param('~replay/log_directory')
    return location


if __name__ == "__main__":
    try:
        rospy.init_node("bin47_converter", anonymous=True)
        file_prefix = get_location()

        if find_file(file_prefix, fprint_file) != "":
            print("bin47 file has already convert to fprintf file.")
            exit(0)

        bin_file_location = find_file(file_prefix, bin_file)

        if bin_file_location == "":
            print("location does not contain bin47 file.")
            exit(1)

        print("Found bin47 file, converting bin file to text file.")
        print(bin_file_location)

        location = sys.argv[0]
        location = location.replace("bin47_converter.py", "Bin47ToText")
        os.system(location + " " + bin_file_location)
    except rospy.ROSInterruptException:
        print("ROSInterruptException")
