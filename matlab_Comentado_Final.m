%Limpando as variáveis
clear all 
close all 
clc 

fprintf('\n');
fprintf('Modelo de locomocao do robo com rodas diferenciais\n');
fprintf('\n');

% Parametros do robo Pioneer 
velocidade_roda_direita_maxima = 1.5;       % velocidade maxima da roda direita m/s
velocidade_roda_esquerda_maxima = 1.5;      % velocidade maxima da roda esquerda m/s 
velocidade_roda_direita_girar = velocidade_roda_direita_maxima*0.2;        % velocidade da roda direita para girar em m/s
velocidade_roda_esquerda_girar = velocidade_roda_esquerda_maxima*0.2;      % velocidade da roda esquerda para girar em m/s
eixo = 0.381;                               % distancia entre as rodas em m
delta_t = 0.1;                              % passo de tempo discreto em s

fprintf('\n');
fprintf('velocidade_roda_direita_maxima: %.3f\n', velocidade_roda_direita_maxima);
fprintf('velocidade_roda_esquerda_maxima: %.3f\n', velocidade_roda_esquerda_maxima);
fprintf('\n');
fprintf('velocidade_roda_direita_girar: %.3f\n', velocidade_roda_direita_girar);
fprintf('velocidade_roda_esquerda_girar: %.3f\n', velocidade_roda_esquerda_girar);
fprintf('\n');
fprintf('Tamanho do Eixo: %.3f\n', eixo);
fprintf('\n');
fprintf('Delta t: %.3f\n', delta_t);
fprintf('\n');

% Equacoes basicas de movimento do robo
velocidade_maxima = (1/2)*(velocidade_roda_direita_maxima+velocidade_roda_esquerda_maxima);              % velocidade a frente, eixo x 
velocidade_angular_maxima = (velocidade_roda_direita_maxima - velocidade_roda_esquerda_maxima)/eixo;     % velocidade angular 


% Pontos inicial e final do robo
fprintf('\n');
pose_inicial = input('Coloque a pose inicial do robô [x0,y0,theta0]: ');    % pose inicial do robo
Movimenta(pose_inicial, velocidade_roda_direita_girar, velocidade_roda_esquerda_girar, eixo, delta_t, velocidade_maxima, velocidade_roda_direita_maxima, velocidade_roda_esquerda_maxima)

function Movimenta(pose_inicial, velocidade_roda_direita_girar, velocidade_roda_esquerda_girar, eixo, delta_t, velocidade_maxima, velocidade_roda_direita_maxima, velocidade_roda_esquerda_maxima)
    ponto_final = input('Coloque o ponto final do robô [xFinal, yFinal]: ');    % pose final do robo 
    fprintf('\n');

    % Calculos para encontrar a diferenca x,y,theta inicial final
    delta_x = ponto_final(1,1) - pose_inicial(1,1);  % diferenca em x
    delta_y = ponto_final(1,2) - pose_inicial(1,2);  % diferenca em y
    theta_alinhamento = atan2d(delta_y,delta_x);     % arco tangente em graus

    % Grafico inicial
    xlim([-1 4]),ylim([-1 4]);
    grid on 
    drawnow;

    % Codigo para o robo Virar o angulo necessario 
    k = 1;                                         % inicializando o contador de tempo discreto 
    pose(k,:) = pose_inicial;                      % inicializando a pose do robo
    erro(k,1:2) = ponto_final - pose(k,1:2);       % erro em x e y
    erro(k,3) = theta_alinhamento - pose(k,3);     % erro em theta
    velocidade_roda_direita_girar_precisao = velocidade_roda_direita_girar * 0.1;         % reduz a velocidade da roda direita
    velocidade_roda_esquerda_girar_precisao = velocidade_roda_esquerda_girar * 0.1;       % reduz a velocidade da roda esquerda

    while (erro(k,3) > 1.5 && erro(k,3) < 358.5) || (erro(k,3) < -1.5 && erro(k,3) > -358.5)  % enquanto o erro angular for maior que 2 graus

        %grafico incremental
        cla;    
        hold on 
        quiver(pose(k,1),pose(k,2),eixo*cosd(pose(k,3)),eixo*sind(pose(k,3)),0,'b'); 
        quiver(pose(k,1),pose(k,2),-eixo*sind(pose(k,3)),eixo*cosd(pose(k,3)),0,'r'); 
        pause(0.1)
        drawnow;

        if erro(k,3) > 5 
            delta_v = velocidade_roda_direita_girar - velocidade_roda_esquerda_girar;                       % diferenca de velocidade entre as rodas
            v = (1/2)*delta_v;                                                                              % velocidade
            theta_ponto = rad2deg((velocidade_roda_direita_girar + velocidade_roda_esquerda_girar)/eixo);   % velocidade angular

        else % se o erro for pequeno, diminui a velocidade para maior precisão
            delta_v = (velocidade_roda_direita_girar_precisao) - (velocidade_roda_esquerda_girar_precisao);                   % diferença de velocidade entre as rodas
            v = (1/2)*delta_v;                                                                                                % velocidade
            theta_ponto = rad2deg((velocidade_roda_direita_girar_precisao + velocidade_roda_esquerda_girar_precisao)/eixo);   % velocidade angular
        end

        pose(k+1,1) = pose(k,1) + cosd(pose(k,3))*(v*delta_t);
        pose(k+1,2) = pose(k,2) + sind(pose(k,3))*(v*delta_t);
        pose(k+1,3) = pose(k,3) + theta_ponto*delta_t; 

        erro(k+1,1:2) = ponto_final - pose(k+1,1:2); 
        erro(k+1,3) = theta_alinhamento - pose(k+1,3); 

        k = k+1;

        pose(k, 3) = mod(pose(k, 3), 360);  % Mantém o ângulo entre 0 e 360 graus
        erro(k, 3) = mod(erro(k, 3), 360);  

        % Mostra erros e poses atuais
        fprintf('\n');
        fprintf('-------------------------\n');
        fprintf('Iteração: %d\n', k);
        fprintf('Pose atual: x: %.2f, y: %.2f, theta: %.2f°\n', pose(k,1), pose(k,2), pose(k,3));
        fprintf('Erro atual: erro_x: %.2f, erro_y: %.2f, erro_theta: %.2f°\n', erro(k,1), erro(k,2), erro(k,3));
        fprintf('-------------------------\n');
        fprintf('\n');

    end

    % Matriz posição final
    ponto_final = [ponto_final(1); ponto_final(2); 1];

    % Posição do robô (após giro)
    x_robo = pose(k,1);
    y_robo = pose(k,2);
    theta_robo = pose(k,3);

    % Tranformação homogêna (verifica o ponto no X do robô)
    Transformacao_inicial = [ cosd(theta_robo), -sind(theta_robo), x_robo;
                            sind(theta_robo),  cosd(theta_robo), y_robo;
                            0,                 0,                1       ];

    % Inverte a matriz            
    Transformacao_inverso = inv(Transformacao_inicial); 

    %Encontra Ponto P em X do robô
    Ponto_alvo = Transformacao_inverso * ponto_final;
    erro_x_atual = Ponto_alvo(1); % erro em X

    % Movimento em linha reta até o ponto final
    theta_ponto= rad2deg((velocidade_roda_direita_maxima -velocidade_roda_esquerda_maxima)/eixo); 

    while erro_x_atual > 0.12
        
        % atualização das poses
        pose(k+1,1) = pose(k,1) + cosd(pose(k,3)) * (velocidade_maxima * delta_t);
        pose(k+1,2) = pose(k,2) + sind(pose(k,3)) * (velocidade_maxima * delta_t);
        pose(k+1,3) = pose(k,3) + theta_ponto * delta_t; 

        % incrementando pose
        x_robo_novo = pose(k+1,1);
        y_robo_novo = pose(k+1,2);
        theta_robo_novo = pose(k+1,3);
        
        % transformação homogenea
        Transformacao_inicial_novo = [ cosd(theta_robo_novo), -sind(theta_robo_novo), x_robo_novo;
                                    sind(theta_robo_novo),  cosd(theta_robo_novo), y_robo_novo;
                                                        0,                      0,            1 ];
        
        % inversa
        Transformacao_inverso_novo = inv(Transformacao_inicial_novo); 

        % Ponto em relacao ao robo em X
        Ponto_alvo_novo = Transformacao_inverso_novo * ponto_final;
        
        % erro em X
        erro_x_atual = Ponto_alvo_novo(1); 

        k = k+1; 

        fprintf('\n');
        fprintf('-------------------------\n');
        fprintf('Iteração: %d\n', k);
        fprintf('Transformação Atual:\n');
        disp(Transformacao_inicial_novo);
        fprintf('Erro em X atual: %.2f\n', erro_x_atual);
        fprintf('Theta: %.2f°\n', pose(k,3));
        fprintf('-------------------------\n');
        fprintf('\n');

        %Grafico
        cla;    
        hold on 
        quiver(pose(k,1),pose(k,2),eixo*cosd(pose(k,3)),eixo*sind(pose(k,3)),0,'b'); 
        quiver(pose(k,1),pose(k,2),-eixo*sind(pose(k,3)),eixo*cosd(pose(k,3)),0,'r'); 
        pause(0.1)
    
    end

    Repetir = input('Deseja movimentar para outro ponto? (1-Sim / 0-Não): ');

    if Repetir == 1
        Movimenta(pose(k,:), velocidade_roda_direita_girar, velocidade_roda_esquerda_girar, eixo, delta_t, velocidade_maxima, velocidade_roda_direita_maxima, velocidade_roda_esquerda_maxima);
    else
        fprintf('Movimentação concluída.\n');
    end
end
