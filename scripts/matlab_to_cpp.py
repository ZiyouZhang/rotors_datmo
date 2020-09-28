"""
/**
 * @file matlab_to_cpp.py
 * @author Ziyou Zhang (ziyou.zhang@outlook.com)
 * @brief Code for easy conversion of matlab synbolic differentiation result to cpp code.
 * @version 0.1
 * @date 2020-09-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */
"""

new_code = ""

with open("/home/ziyou/catkin_ws/src/rotors_simulator/rotors_datmo/src/visual_ekf.cpp", "r") as f:
    for line in f:
        new_line = line

        new_line = new_line.replace("C_WO1_1", "C_WO(0, 0)")
        new_line = new_line.replace("C_WO1_2", "C_WO(0, 1)")
        new_line = new_line.replace("C_WO1_3", "C_WO(0, 2)")
        new_line = new_line.replace("C_WO2_1", "C_WO(1, 0)")
        new_line = new_line.replace("C_WO2_2", "C_WO(1, 1)")
        new_line = new_line.replace("C_WO2_3", "C_WO(1, 2)")
        new_line = new_line.replace("C_WO3_1", "C_WO(2, 0)")
        new_line = new_line.replace("C_WO3_2", "C_WO(2, 1)")
        new_line = new_line.replace("C_WO3_3", "C_WO(2, 2)")

        new_line = new_line.replace("v1", "state.v_O.x()")
        new_line = new_line.replace("v2", "state.v_O.y()")
        new_line = new_line.replace("v3", "state.v_O.z()")

        new_line = new_line.replace("w1", "state.omega_O.x()")
        new_line = new_line.replace("w2", "state.omega_O.y()")
        new_line = new_line.replace("w3", "state.omega_O.z()")

        new_line = new_line.replace("q1", "state.q_WO.w()")
        new_line = new_line.replace("q2", "state.q_WO.x()")
        new_line = new_line.replace("q3", "state.q_WO.y()")
        new_line = new_line.replace("q4", "state.q_WO.z()")

        new_code += new_line

print(new_code)

with open("/home/ziyou/catkin_ws/src/rotors_simulator/rotors_datmo/src/visual_ekf.cpp", "w") as f:
    f.write(new_code)