from sps_common_msgs.srv import ParamsData
from sps_common_msgs.srv import SetParamData
import json
import rclpy
import os
from threading import Lock
from rclpy.node import Node


class ParamsManager(Node):

    paramsfile_lock_ = Lock()
    home = os.getenv("HOME")
    params_file = os.path.join(home, ".hari_params.json")

    global obstacle_avoidance_waiting_seconds
    global drop_switch

    def __init__(self):
        super().__init__('params_manager')
        self.srv_params = self.create_service(
            ParamsData, '/params_data', self.params_data_callback)

        self.srv_param = self.create_service(
            SetParamData, '/param_data', self.set_param_data_callback)

        json_object = self.get_params_from_file()
        value = {}
        self.set_params(value, json_object)

    def set_param_data_callback(self, request, response):
        self.get_logger().info('Set Param data: node_name: %s ' % request.node_name)
        self.get_logger().info('Set Param data: param_name: %s' % request.param_name)
        self.get_logger().info('Set Param data: param_data: %s' % request.param_data)

        cmd = "ros2 param set " + request.node_name + " " + request.param_name + " " + request.param_data
        self.get_logger().info('Set Param data: cmd: %s' % cmd)
        os.system(cmd)

        return response

    def save_params_file(self, value):
        self.paramsfile_lock_.acquire()
        self.get_logger().info("save_params_file ReadFile %s loading ..." % self.params_file)
        with open(self.params_file, "w") as fout:
            json_str = json.dumps(value, indent=4)
            fout.write(json_str)

        os.sync()
        self.paramsfile_lock_.release()

    def get_params_from_file(self):
        self.get_logger().info("get_hari_params_from_file %s loading ..." % self.params_file)
        try:
            with open(self.params_file, "r") as f:
                root = json.load(f)
                params = json.dumps(root, indent=4)
                self.get_logger().info("params_file content: \n%s" % params)
        except FileNotFoundError:
            self.get_logger().warn("ReadFile %s load ERROR." % self.params_file)
            root = {}

        return root

    def set_params(self, value, json_object):
        self.drop_switch = json_object.get("visualAntiDropEnable")
        if self.drop_switch is not None:
            value["visualAntiDropEnable"] = self.drop_switch
            cmd = "ros2 param set /cloudminds/TaskManager main_drop_switch " + \
                str(self.drop_switch)
            os.system(cmd)
            self.get_logger().info('Set main_drop_switch : %s ' % self.drop_switch)

        self.obstacle_avoidance_waiting_seconds = json_object.get(
            "obstacleAvoidanceWaitingSeconds")
        if self.obstacle_avoidance_waiting_seconds is not None:
            value["obstacleAvoidanceWaitingSeconds"] = self.obstacle_avoidance_waiting_seconds
            self.get_logger().info('Set ParamsManager : ' +
                                   str(self.obstacle_avoidance_waiting_seconds))

    def parse_json(self, json_params):
        try:
            json_object = json.loads(json_params)
            self.get_logger().info("Request json_params string ")
        except json.decoder.JSONDecodeError:
            self.get_logger().warn("Request json_params string invalid")
        else:
            value = self.get_params_from_file()

            self.set_params(value, json_object)
            self.save_params_file(value)

    def params_data_callback(self, request, response):
        self.get_logger().info('Set ParamsManager ctrl_cmd: %d ' % request.ctrl_cmd)
        if request.ctrl_cmd == 0:
            self.parse_json(request.json_params)

        elif request.ctrl_cmd == 1:
            value = self.get_params_from_file()
            root = {}
            root["action"] = "getSpsParams"
            root["param"] = value
            response.json_report = json.dumps(root)
            self.get_logger().info("params_file json_report: \n%s" % response.json_report)

        return response


def main():
    rclpy.init()

    params_manager_node = ParamsManager()

    rclpy.spin(params_manager_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
