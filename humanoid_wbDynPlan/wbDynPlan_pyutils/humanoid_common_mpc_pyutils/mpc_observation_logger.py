

import rclpy
from rclpy.node import Node
from ocs2_ros2_msgs.msg import MpcObservation
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import pandas as pd
from datetime import datetime


class MpcObservationLogger(Node):

    def __init__(self):
        super().__init__("mpc_observation_subscriber")
        print("Setting up MPC observation logger...")
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        self.subscription = self.create_subscription(
            MpcObservation,
            "humanoid/mpc_observation",
            self.listener_callback,
            qos_profile,
        )
        self.subscription  # prevent unused variable warning

        self.logger_cols = [
            "h_x",
            "h_y",
            "h_z",
            "L_x",
            "L_y",
            "L_z",
            "p_x",
            "p_y",
            "p_z",
            "euler_z",
            "euler_y",
            "euler_x",
            "q_j_l_hip_y",
            "q_j_l_hip_x",
            "q_j_l_hip_z",
            "q_j_l_upperknee_y",
            "q_j_l_lowerknee_y",
            "q_j_l_ankle_y",
            "q_j_l_ankle_x",
            "q_j_r_hip_y",
            "q_j_r_hip_x",
            "q_j_r_hip_z",
            "q_j_r_upperknee_y",
            "q_j_r_lowerknee_y",
            "q_j_r_ankle_y",
            "q_j_r_ankle_x",
            "q_j_spine_y",
            "q_j_spine_z",
            "q_j_l_shoulder_y",
            "q_j_l_shoulder_x",
            "q_j_l_shoulder_z",
            "q_j_l_elbow_y",
            "q_j_l_elbow_z",
            "q_j_r_shoulder_y",
            "q_j_r_shoulder_x",
            "q_j_r_shoulder_z",
            "q_j_r_elbow_y",
            "q_j_r_elbow_z",
            "q_j_l_wrist_x",
            "q_j_l_wrist_y",
            "q_j_neck_z",
            "q_j_neck_y",
            "q_j_neck_x",
            "q_j_r_wrist_x",
            "q_j_r_wrist_y",
            "F_l_x",
            "F_l_y",
            "F_l_z",
            "M_l_x",
            "M_l_y",
            "M_l_z",
            "F_r_x",
            "F_r_y",
            "F_r_z",
            "M_r_x",
            "M_r_y",
            "M_r_z",
            "qd_j_l_hip_y",
            "qd_j_l_hip_x",
            "qd_j_l_hip_z",
            "qd_j_l_upperknee_y",
            "qd_j_l_lowerknee_y",
            "qd_j_l_ankle_y",
            "qd_j_l_ankle_x",
            "qd_j_r_hip_y",
            "qd_j_r_hip_x",
            "qd_j_r_hip_z",
            "qd_j_r_upperknee_y",
            "qd_j_r_lowerknee_y",
            "qd_j_r_ankle_y",
            "qd_j_r_ankle_x",
            "qd_j_spine_y",
            "qd_j_spine_z",
            "qd_j_l_shoulder_y",
            "qd_j_l_shoulder_x",
            "qd_j_l_shoulder_z",
            "qd_j_l_elbow_y",
            "qd_j_l_elbow_z",
            "qd_j_r_shoulder_y",
            "qd_j_r_shoulder_x",
            "qd_j_r_shoulder_z",
            "qd_j_r_elbow_y",
            "qd_j_r_elbow_z",
            "qd_j_l_wrist_x",
            "qd_j_l_wrist_y",
            "qd_j_neck_z",
            "qd_j_neck_y",
            "qd_j_neck_x",
            "qd_j_r_wrist_x",
            "qd_j_r_wrist_y",
            "time",
        ]
        self.data_df = pd.DataFrame()

    def listener_callback(self, msg):
        print(msg.time)
        print(type(msg.state))
        mpc_obs_arr = np.array(msg.state.value)
        mpc_obs_arr = np.append(mpc_obs_arr, np.zeros(7))
        mpc_obs_arr = np.append(mpc_obs_arr, msg.input.value)
        mpc_obs_arr = np.append(mpc_obs_arr, np.zeros(7))
        mpc_obs_arr = np.append(mpc_obs_arr, msg.time)
        new_df = pd.DataFrame(mpc_obs_arr.reshape(1, -1), columns=self.logger_cols)
        self.data_df = pd.concat([self.data_df, new_df], ignore_index=True)

    def save_log(self):
        log_name = (
            "mpc_observation_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
        )
        print("Saving log:", log_name)
        self.data_df.to_csv(log_name, index=False)
        print("Done saving log")


def main(args=None):
    rclpy.init(args=args)
    subscriber = MpcObservationLogger()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.save_log()
        subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
