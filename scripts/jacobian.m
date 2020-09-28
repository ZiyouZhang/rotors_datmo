"""
/**
 * @file jacobian.m
 * @author Ziyou Zhang (ziyou.zhang@outlook.com)
 * @brief The matlab code for symbolic jacobian differentiation.
 * @version 0.1
 * @date 2020-09-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */
"""

r = sym('r',[3 1], 'real');
q = sym('q', [4 1], 'real');
v = sym('v', [3 1], 'real');
w = sym('w', [3 1], 'real');

C_WO = sym('C_WO', [3 3], 'real');
% g = sym('g', [3 1], 'real');
% I = sym('I', [3 3], 'real');
g = [0; 0; -9.81];

aux = sym('aux', 'real');
% I = [0.5/12, 0, 0;
%     0, 0.5/12, 0;
%     0, 0, 0.5/12];
I = [aux, 0, 0;
    0, aux, 0;
    0, 0, aux];

t = sym('t', 'real');

delta_r_1 = C_WO * v;
delta_q_1 = (t * 0.5 * quatmultiply(q.', [w; 0].')).';
delta_v_1 = t * (inv(C_WO) * g - cross(w, v));
delta_w_1 = t * inv(I) * (cross(w, I*w));

r_k_temp = r + delta_r_1;
q_k_temp = q + delta_q_1;
v_k_temp = v + delta_v_1;
w_k_temp = w + delta_w_1;

delta_r_2 = C_WO * v_k_temp;
delta_q_2 = (t * 0.5 * quatmultiply(q_k_temp.', [0; w_k_temp].')).';
delta_v_2 = t * (inv(C_WO) * g - cross(w_k_temp, v_k_temp));
delta_w_2 = t * inv(I) * (cross(w_k_temp, I*w_k_temp));

r_k = r + 0.5 * (delta_r_1 + delta_r_2);
q_k = q + 0.5 * (delta_q_1 + delta_q_2);
v_k = v + 0.5 * (delta_v_1 + delta_v_2);
w_k = w + 0.5 * (delta_w_1 + delta_w_2);

disp('r_k');
disp(jacobian(r_k, r));
disp(jacobian(r_k, q));
disp(jacobian(r_k, v));
disp(jacobian(r_k, w));

disp('q_k');
disp(jacobian(q_k, r));
disp(jacobian(q_k, q));
disp(jacobian(q_k, v));
disp(jacobian(q_k, w));

disp('v_k');
disp(jacobian(v_k, r));
disp(jacobian(v_k, q));
disp(jacobian(v_k, v));
disp(jacobian(v_k, w));

disp('w_k');
disp(jacobian(w_k, r));
disp(jacobian(w_k, q));
disp(jacobian(w_k, v));
disp(jacobian(w_k, w));
