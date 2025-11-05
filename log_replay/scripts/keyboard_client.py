#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
from pynput import keyboard
from pynput.keyboard import Key, KeyCode
from std_msgs.msg import UInt8
import sys
import signal

HELP = u"按键功能列表： \n \
                up :     (1)增加播放速度 \n \
              down :     (2)降低播放速度 \n \
              left :     (3)回退10秒    \n \
                f1 :     (0,1) 暂停/继续 \n \
                f2 :     启动/关闭键盘控制 \n \
                f3 :     打印本帮助 \n \
                r  :     (11,12)实时/极速播放 \n \
		"

node_name = 'keypress_publisher'
publish_topic = 'pressed_key'


class KeyboardPublisher:
    def __init__(self):
        # logging the node start
        rospy.loginfo("Starting client for keyboard ops")
        rospy.init_node(node_name, anonymous=True)
        self.key_map = {
            Key.up: 2,
            Key.down: 3,
            Key.left: 4,
            Key.right: 5,
            Key.f3: 7,
        }
        self.current_pause = False
        self.current_real_time = True
        self.state = True
        self.listen_on = True

        self.publisher = rospy.Publisher('/{}/{}'.format(node_name, publish_topic), UInt8, queue_size=20)
        print(HELP)
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def keycode_action(self, key_code):
        send_msg = UInt8()
        send_msg.data = key_code
        self.publisher.publish(send_msg)

    def on_press(self, key):
        return self.state

    def on_release(self, key):
        if rospy.is_shutdown(): return False
        if key == Key.f2:
            self.listen_on = not self.listen_on
            print('Keyboard enable {}'.format(self.listen_on))

        if not self.listen_on:
            return True

        if key == Key.f1:
            if self.current_pause:
                self.keycode_action(1)
            else:
                self.keycode_action(0)
            self.current_pause = not self.current_pause
            print('pause: {}'.format(self.current_pause))
        elif key == Key.f3:
            print(HELP)
        elif key == KeyCode.from_char('r'):
            if self.current_real_time:
                self.keycode_action(12)
            else:
                self.keycode_action(11)
            self.current_real_time = not self.current_real_time
            print('real time: {}'.format(self.current_real_time))
        else:
            self.keycode_action(self.key_map.get(key, 100))
        return True


if __name__ == '__main__':
    keyboard_publisher = KeyboardPublisher()
    rospy.spin()
