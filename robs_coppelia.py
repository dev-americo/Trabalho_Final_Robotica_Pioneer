"""
path_Pioneer.py

Controle de Navegação em Duas Fases (Alinhamento + Movimento Linear).
Implementação do controle fornecido pelo usuário dentro do loop do CoppeliaSim.
"""

import math
import sim
import numpy as np # Import necessário para o código de controle

class Pioneer():
    """
    Classe principal para controlar o robô Pioneer no CoppeliaSim.
    """

    def __init__(self):
        """ Inicializações de variáveis globais e constantes do controlador. """
        self.y_out = []
        self.x_out = []
        
        # --- CONSTANTES DO ROBÔ E DO CONTROLADOR ---
        self.w = 0.381                 # Distância entre rodas (Eixo, L)
        self.v_max_roda = 1.5          # Velocidade linear máxima da roda
        self.theta_dot_max = 2 * self.v_max_roda / self.w # Velocidade angular máxima (omega_max)
        self.Kp_theta = 2.0            # Ganho Proporcional para Orientação
        self.v_l_max = 1.5
        self.v_r_max = 1.5
        
        # --- VARIÁVEIS DE ESTADO E META ---
        self.pose_final = np.array([1.0, 3.0])  # Posição final desejada (X, Y)
        self.control_phase = 'ALIGNMENT'        # 'ALIGNMENT' ou 'LINEAR'
        self.Min_error_distance = 0.1           # Distância mínima para critério de parada

    def connect_Pioneer(self, port):
        # ... (Função connect_Pioneer permanece a mesma) ...
        """
        Função usada para se comunicar com o CoppeliaSim.
        """
        # Conexão com coppeliaSim
        sim.simxFinish(-1)
        clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
        if clientID == 0:
            print("Conectado à porta", port)
        else:
            print("Não foi possível conectar à porta", port)
            return None, None, None, None, None

        # Obtenção dos handles dos objetos
        returnCode, robot = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx', sim.simx_opmode_blocking)
        returnCode, MotorE = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
        returnCode, MotorD = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)
        returnCode, ball = sim.simxGetObjectHandle(clientID, 'ball', sim.simx_opmode_blocking) # Mantido para referência

        return clientID, robot, MotorE, MotorD, ball


    def Robot_Pioneer(self, deltaT):
        """
        Função principal que contém o loop de simulação e o controlador em duas fases.
        """
        
        # Obtenção dos objetos
        (clientID, robot, motorE, motorD, ball) = self.connect_Pioneer(19999)

        if clientID is None:
            return

        # Inicialização de variáveis de controle e loop
        Number_Iterations = 0
        a = 1 # Condição para simulação: 1 = rodando, 0 = parar
        
        # Variáveis de velocidade (iniciais)
        vl, vd = 0.0, 0.0

        if (sim.simxGetConnectionId(clientID) != -1):
            
            # Loop de Simulação
            while (a == 1):

                # 1. Configuração e Leitura Inicial
                if Number_Iterations <= 1:
                    # Leitura inicial de streaming (modo síncrono é geralmente preferido)
                    s, positiona = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
                    s, orientation = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
                    
                    # Definição da pose inicial (robot [0,0,0]) e cálculo do ângulo alvo (theta_alinhamento)
                    pose_inicial = np.array([positiona[0], positiona[1], orientation[2]])
                    delta_x = self.pose_final[0] - pose_inicial[0]
                    delta_y = self.pose_final[1] - pose_inicial[1]
                    self.theta_alinhamento = math.atan2(delta_y, delta_x)
                    
                    # NOVO: Posiciona o 'ball' na pose final para visualização
                    target_position = [self.pose_final[0], self.pose_final[1], 0.0] # X, Y, Z (no chão)
                    sim.simxSetObjectPosition(clientID, ball, -1, target_position, sim.simx_opmode_blocking)
                    print(f"Alvo 'ball' posicionado em X={target_position[0]}, Y={target_position[1]} para visualização.")
                    
                    # Zera motores
                    sim.simxSetJointTargetVelocity(clientID, motorE, 0, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(clientID, motorD, 0, sim.simx_opmode_blocking)

                # 2. Loop de execução principal
                else:
                    # Obter Posições Atuais do Robô
                    s, positiona = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
                    returnCode, orientation = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
                    
                    x_robo = positiona[0]
                    y_robo = positiona[1]
                    theta_robo = orientation[2] # Orientação do robô no eixo Z
                    
                    
                    # --- CÓDIGO DE CONTROLE EM DUAS FASES ---
                    
                    if self.control_phase == 'ALIGNMENT':
                        
                        # CÁLCULO DE ERRO DE ORIENTAÇÃO
                        erro_orientacao = self.theta_alinhamento - theta_robo
                        # Normaliza o erro entre [-pi, pi]
                        erro_orientacao = math.atan2(np.sin(erro_orientacao), np.cos(erro_orientacao))

                        if np.abs(erro_orientacao) > np.radians(1.0):
                            
                            # CONTROLE PURELY ROTACIONAL (v=0, pois v_r_max = v_l_max)
                            delta_v = self.v_r_max - self.v_l_max
                            v = (1/2)*delta_v # Deve ser 0.0
                            
                            theta_dot_control = self.Kp_theta * erro_orientacao
                            
                            # Limita a velocidade angular máxima (theta_ponto)
                            theta_ponto = np.clip(theta_dot_control, -self.theta_dot_max, self.theta_dot_max)

                            # CINEMÁTICA INVERSA: Converte (v, theta_ponto) para (vl, vd)
                            vl = v - (theta_ponto * self.w) / 2
                            vd = v + (theta_ponto * self.w) / 2
                            
                            print(f"FASE ALINHAMENTO: Erro Theta={np.degrees(erro_orientacao):.2f}°")

                        else:
                            # TRANSIÇÃO PARA MOVIMENTO LINEAR
                            self.control_phase = 'LINEAR'
                            vl, vd = 0.0, 0.0
                            print("\n--- TRANSIÇÃO: MOVIMENTO LINEAR ---")

                    elif self.control_phase == 'LINEAR':
                        
                        # TRANSFORMAÇÃO DE POSE PARA CÁLCULO DE ERRO LOCAL (erro_x_atual)
                        # NOTA: Esta parte AGORA usa o self.pose_final=[2.0, 3.0], 
                        # o que é o alvo CORRETO e está visualmente marcado pelo 'ball'.
                        p_final_homogeneo = np.array([self.pose_final[0], self.pose_final[1], 1.0])

                        t_inicial = np.array([
                            [np.cos(theta_robo), -np.sin(theta_robo), x_robo],
                            [np.sin(theta_robo),  np.cos(theta_robo), y_robo],
                            [0.0,                     0.0,                    1.0   ]
                        ])

                        t_inverso = np.linalg.inv(t_inicial)
                        p_alvo = t_inverso @ p_final_homogeneo
                        erro_x_atual = p_alvo[0] # Erro no eixo X local (distância a percorrer)

                        if np.abs(erro_x_atual) > self.Min_error_distance:
                            
                            # MOVIMENTO PURAMENTE LINEAR
                            vl = self.v_max_roda
                            vd = self.v_max_roda
                            print(f"FASE LINEAR: Erro X local={erro_x_atual:.2f}m")

                        else:
                            # OBJETIVO ALCANÇADO: PARAR
                            vl, vd = 0.0, 0.0
                            a = 0 # Parar a simulação
                            print("\n--- OBJETIVO ALCANÇADO ---")
                            print(f"Pose Final: X={x_robo:.2f}, Y={y_robo:.2f}, Theta={np.degrees(theta_robo):.2f}°")


                    # Fim do bloco de controle
                    
                    # 3. Enviar velocidades para o CoppeliaSim
                    sim.simxSetJointTargetVelocity(clientID, motorE, vl, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(clientID, motorD, vd, sim.simx_opmode_blocking)

                # 4. Atualizar iteração e armazenar dados (opcional)
                Number_Iterations = Number_Iterations + 1
                self.y_out.append(positiona[1])
                self.x_out.append(positiona[0])
                
            # Fim do loop principal


if __name__ == "__main__":
    crb01 = Pioneer()
    deltaT = 0.05 
    crb01.Robot_Pioneer(deltaT)