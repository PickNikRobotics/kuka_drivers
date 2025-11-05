# Copyright 2022 Márton Antal
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.from launch import LaunchDescription

import socket
import sys
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def create_rsi_xml_rob(act_joint_pos, timeout_count, ipoc):
    q = act_joint_pos
    dof = len(q)
    robot_dof = min(dof, 6)
    external_dof = dof - robot_dof if dof > 6 else 0
    
    root = ET.Element("Rob", {"TYPE": "KUKA"})
    ET.SubElement(
        root, "RIst", {"X": "0.0", "Y": "0.0", "Z": "0.0", "A": "0.0", "B": "0.0", "C": "0.0"}
    )
    
    # Robot axes (A1-A6)
    robot_attribs = {f"A{i+1}": str(q[i]) for i in range(robot_dof)}
    ET.SubElement(root, "AIPos", robot_attribs)
    
    # External axes (E1-EN)
    if external_dof > 0:
        external_attribs = {f"E{i+1}": str(q[robot_dof + i]) for i in range(external_dof)}
        ET.SubElement(root, "EIPos", external_attribs)
    
    ET.SubElement(root, "Delay", {"D": str(timeout_count)})
    ET.SubElement(root, "IPOC").text = str(ipoc)
    return ET.tostring(root, encoding="utf-8", method="xml").replace(b" />", b"/>")


def parse_rsi_xml_sen(data):
    root = ET.fromstring(data)
    
    # Parse robot axes (A1-A6)
    AK = root.find("AK").attrib
    robot_correction = np.array([AK[f"A{i+1}"] for i in range(len(AK))]).astype(np.float64)
    
    # Parse external axes (E1-EN) if present
    EK = root.find("EK")
    if EK is not None:
        external_correction = np.array([EK.attrib[f"E{i+1}"] for i in range(len(EK.attrib))]).astype(np.float64)
        desired_joint_correction = np.concatenate([robot_correction, external_correction])
    else:
        desired_joint_correction = robot_correction
    
    IPOC = root.find("IPOC").text
    stop_flag = root.find("Stop").text

    return desired_joint_correction, int(IPOC), bool(int(stop_flag))


class RSISimulator(Node):
    cycle_time = 0.04
    timeout_count = 0
    ipoc = 0
    rsi_ip_address_ = "127.0.0.1"
    rsi_port_address_ = 59152
    rsi_send_name_ = "IamFree"
    rsi_act_pub_ = None
    rsi_cmd_pub_ = None
    node_name_ = "rsi_simulator_node"
    socket_ = None

    def __init__(self, node_name):
        super().__init__(node_name)
        self.node_name_ = node_name
        self.timer = self.create_timer(self.cycle_time, self.timer_callback)
        self.declare_parameter("rsi_ip_address", "127.0.0.1")
        self.declare_parameter("rsi_port", 59152)
        self.declare_parameter("rsi_send_name", "IamFree")
        self.declare_parameter("dof", 6)

        self.rsi_ip_address_ = (
            self.get_parameter("rsi_ip_address").get_parameter_value().string_value
        )
        self.rsi_port_address_ = self.get_parameter("rsi_port").get_parameter_value().integer_value
        self.rsi_send_name_ = (
            self.get_parameter("rsi_send_name").get_parameter_value().string_value
        )

        dof = self.get_parameter("dof").get_parameter_value().integer_value
        self.robot_dof = min(dof, 6)
        self.external_dof = min(dof - 6, 6) if dof > 6 else 0

        # Initialize joint positions
        self.act_joint_pos = np.zeros(dof)
        self.act_joint_pos[:6] = np.array([0, -90, 90, 0, 90, 0])  # Initialize robot joints to home
        self.initial_joint_pos = self.act_joint_pos.copy()
        self.des_joint_correction_absolute = np.zeros(dof)

        self.rsi_act_pub_ = self.create_publisher(String, self.node_name_ + "/rsi/state", 1)
        self.rsi_cmd_pub_ = self.create_publisher(String, self.node_name_ + "/rsi/command", 1)
        self.get_logger().info(f"rsi_ip_address: {self.rsi_ip_address_}")
        self.get_logger().info(f"rsi_port: {self.rsi_port_address_}")
        self.get_logger().info(f"dof: {dof} (robot: {self.robot_dof}, external: {self.external_dof})")

        self.socket_ = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.get_logger().info(f"{self.node_name_}, Successfully created socket")
        self.socket_.settimeout(self.cycle_time)

    def timer_callback(self):
        if self.timeout_count == 100:
            self.get_logger().fatal(f"{self.node_name_} Timeout count of 100 exceeded")
            # sys.exit()
        try:
            msg = create_rsi_xml_rob(self.act_joint_pos, self.timeout_count, self.ipoc)
            self.rsi_act_pub_.publish(msg)
            self.socket_.sendto(msg, (self.rsi_ip_address_, self.rsi_port_address_))
            recv_msg, _ = self.socket_.recvfrom(1024)
            self.rsi_cmd_pub_.publish(recv_msg)
            self.get_logger().warn(f"msg: {recv_msg}")
            des_joint_correction_absolute, ipoc_recv, stop_flag = parse_rsi_xml_sen(recv_msg)
            self.get_logger().warn(f"des_joint_correction_absolute: {des_joint_correction_absolute}")
            if ipoc_recv == self.ipoc:
                self.act_joint_pos = self.initial_joint_pos + des_joint_correction_absolute
            else:
                self.get_logger().warn(f"{self.node_name_}: Packet is late")
                self.get_logger().warn(
                    f"{self.node_name_}: sent ipoc: {self.ipoc}, received: {ipoc_recv}"
                )
                if self.ipoc != 0:
                    self.timeout_count += 1
            self.ipoc += 1
            if stop_flag:
                self.on_shutdown()
                sys.exit()
        except OSError:
            if self.ipoc != 0:
                self.timeout_count += 1
                self.get_logger().warn(f"{self.node_name_}: Socket timed out")

    def on_shutdown(self):
        self.socket_.close()
        self.get_logger().info("Socket closed.")


def main():
    node_name = "rsi_simulator_node"

    rclpy.init()
    node = RSISimulator(node_name)
    node.get_logger().info(f"{node_name}: Started")

    rclpy.spin(node)
    node.on_shutdown()
    node.get_logger().info(f"{node_name}: Shutting down")


if __name__ == "__main__":
    main()
