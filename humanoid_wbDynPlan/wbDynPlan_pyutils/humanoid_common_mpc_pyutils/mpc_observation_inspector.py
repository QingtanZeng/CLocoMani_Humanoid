

import pandas as pd
import vpselector


def main():
    plot_config_dict = {
        "x_axis_col": "time",
        "sub_plt1_data": [
            "h_x",
            "h_y",
            "h_z",
            "L_x",
            "L_y",
            "L_z",
        ],
        "sub_plt2_data": [
            "p_x",
            "p_y",
            "p_z",
            "euler_z",
            "euler_y",
            "euler_x",
        ],
        "sub_plt3_data": [
            "q_j_l_hip_y",
            "q_j_l_hip_x",
            "q_j_l_hip_z",
            "q_j_l_upperknee_y",
            "q_j_l_lowerknee_y",
            "q_j_l_ankle_y",
            "q_j_l_ankle_x",
        ],
        "sub_plt4_data": [
            "q_j_r_hip_y",
            "q_j_r_hip_x",
            "q_j_r_hip_z",
            "q_j_r_upperknee_y",
            "q_j_r_lowerknee_y",
            "q_j_r_ankle_y",
            "q_j_r_ankle_x",
        ],
    }
    log_name = "mpc_observation_20240701_093238"
    data_df = pd.read_csv(log_name + ".csv")
    print(data_df.columns)
    print(data_df)

    selected_df = vpselector.data_selection.select_visual_data(
        data_df, plot_config_dict
    )
    print("Selected dataframe:")
    selected_df.self.data_df.to_csv(log_name + "_cropped.csv", index=False)


if __name__ == "__main__":
    main()
