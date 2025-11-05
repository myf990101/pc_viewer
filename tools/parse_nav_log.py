import os
import sys


def parse(nav_log):
    key_words = 'Robot Tilt Status '

    with open(nav_log + '.fprintf', 'w') as fw:
        with open(nav_log, 'r') as fr:
            for line in fr.readlines():
                if (key_words in line):
                    timestamp = line.split(' ')[1]
                    value = line.split(' ')[19]
                    if (value == '1'):
                        fw.write(timestamp + ' robotTilt true \n')
                    else:
                        fw.write(timestamp + ' robotTilt false \n')


if __name__ == '__main__':
    nav_log = sys.argv[1]
    parse(nav_log)
