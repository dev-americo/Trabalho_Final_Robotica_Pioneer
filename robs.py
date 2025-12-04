import numpy as np
import math


v_r_max = 01.5
v_l_max = 01.5
w = 0.381
delta_t = 0.1
Kp_theta = 2.0

v_max_roda = 1.5
theta_dot_max = 2 * v_max_roda / w


pose_inicial = np.array([0.0,0.0,0.0])
pose_final = np.array([2.0,3.0])

delta_x = pose_final[0] - pose_inicial[0]
delta_y = pose_final[1] - pose_inicial[1]
theta_alinhamento = math.atan2(delta_y, delta_x)


k = 0
pose_array = np.array([pose_inicial])

erro_orientacao = theta_alinhamento - pose_array[k, 2]

erro_orientacao = math.atan2(np.sin(erro_orientacao), np.cos(erro_orientacao))

while np.abs(erro_orientacao)  > np.radians(1.0):
    delta_v = v_r_max - v_l_max
    v = (1/2)*delta_v

    theta_dot_control = Kp_theta * erro_orientacao
    
    # Limita a velocidade angular máxima
    theta_ponto = np.clip(theta_dot_control, -theta_dot_max, theta_dot_max)
    
    nova_pose = np.array([
        pose_array[k, 0]  + v *np.cos(pose_array[k, 2]) *  delta_t,
        pose_array[k, 1]  + v *np.sin(pose_array[k, 2]) * delta_t,
        pose_array[k, 2]  + theta_ponto*delta_t
    ])

    pose_array = np.append(pose_array, [nova_pose], axis=0)
    k += 1

    
    erro_orientacao = theta_alinhamento - pose_array[k, 2]


    

p_final_homogeneo = np.array([pose_final[0], pose_final[1], 1.0])

x_robo = pose_array[k,0]
y_robo = pose_array[k,1]
theta_robo = pose_array[k,2]

t_inicial = np.array([
    [np.cos(theta_robo), -np.sin(theta_robo), x_robo],
    [np.sin(theta_robo),  np.cos(theta_robo), y_robo],
    [0.0,                     0.0,                    1.0   ]
])

t_inverso = np.linalg.inv(t_inicial)

p_alvo = t_inverso @ p_final_homogeneo
erro_x_atual = p_alvo[0]

while np.abs(erro_x_atual) > 0.1:
    theta_dot_control = Kp_theta * erro_orientacao
    
    # Limita a velocidade angular máxima
    theta_ponto = np.clip(theta_dot_control, -theta_dot_max, theta_dot_max)

    nova_pose = np.array([
        pose_array[k, 0]  + v_max_roda *np.cos(pose_array[k, 2]) *  delta_t,
        pose_array[k, 1]  + v_max_roda *np.sin(pose_array[k, 2]) * delta_t,
        pose_array[k, 2] 
    ])

    pose_array = np.append(pose_array, [nova_pose], axis=0)
    k += 1

    x_robo = pose_array[k, 0]
    y_robo = pose_array[k, 1]
    theta_robo = pose_array[k, 2]

    t_inicial = np.array([
        [np.cos(theta_robo), -np.sin(theta_robo), x_robo],
        [np.sin(theta_robo),  np.cos(theta_robo), y_robo],
        [0.0,                     0.0,                    1.0   ]
    ])

    t_inverso = np.linalg.inv(t_inicial)

    p_alvo = t_inverso @ p_final_homogeneo
    erro_x_atual = p_alvo[0]

    print("\n--- Resultado do Alinhamento ---")
    print(f"Pose final: X={pose_array[-1, 0]:.2f}, Y={pose_array[-1, 1]:.2f}, Theta={np.degrees(pose_array[-1, 2]):.2f} graus") 
