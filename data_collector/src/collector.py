#!/usr/bin/python
# -*- coding:utf-8 -*-

import os
import rospy
import rosbag
import time
from sensor_msgs.msg import PointCloud2


class Lidar(object):
    def __init__(self, top_dir):
        self.data_file = os.path.join(top_dir, "lidar.bag")
        self.record = rosbag.Bag(self.data_file, 'w')

    def record_lidar(self, topic, msg):
        self.record.write(topic, msg)

    def close(self):
        self.record.close()


class APP(object):
    def __init__(self):
        rospy.init_node("data_collector", log_level=rospy.INFO)
        self._initial_()

        self.lidar_record = Lidar(self.full_path)
        self.myTest_record = my_Test(self.full_path)

        self._add_sub()

    def _initial_(self):
        storage = rospy.get_param("~file_path", default="/data")
        if not os.path.exists(storage):
            try:
                os.mkdir(storage)
            except Exception as e:
                rospy.ERROR("could not make dataSet dir: {} , please check it".format(storage))
                rospy.ERROR(e)
                exit(1)
        dir_name = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
        self.full_path = os.path.join(storage, dir_name)
        try:
            os.mkdir(self.full_path)
        except Exception as e:
            rospy.ERROR("could not make dataSet dir: {} , please check it".format(self.full_path))
            rospy.ERROR(e)
            exit(1)

    def _add_sub(self):
        # self.lidar_sub = rospy.Subscriber("/lidar", PointCloud2, self._lidar_cb)
        self.myTest_sub = rospy.Subscriber("/NDT_attr", AttrMsg, self._myTest_cb)

    def _lidar_cb(self, msg):
        self.lidar_record.record_lidar("/lidar", msg)

    def run(self):
        rospy.spin()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.lidar_record.close()


if __name__ == '__main__':
    with APP() as app:
        app.run()
