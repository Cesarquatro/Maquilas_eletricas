close all; clc; clear;
%% Parâmetros do Motor e Sistema
% Parâmetros fornecidos na tabela
Hp = 10;             % Potência do motor em HP
Vl_line = 400;       % Tensão de linha em V
v_phase = 230.94;    % Tensão de fase em V
f = 60;              % Frequência em Hz
Eff = 0.885;         % Eficiência
Ef_75 = 0.885;       % Eficiência a 75% da carga
Eff_50 = 0.875;      % Eficiência a 50% da carga
Pff = 0.87;          % Fator de potência
Pf_75 = 0.810;       % Fator de potência a 75% da carga
Pf_50 = 0.710;       % Fator de potência a 50% da carga
Ir = 14;             % Corrente nominal do rotor em A
Ts = 210;            % Torque de partida em %
Tm = 250;            % Torque máximo
T_nom = 20.6;        % Torque nominal

StartingTorque = Ts / 100 * T_nom;  % Cálculo do torque de partida

% Parâmetros do circuito equivalente
I_75 = 11.2655;      % Corrente a 75% da carga
I_50 = 8.6660;       % Corrente a 50% da carga
rt = 1.0347;         % Razão de transformação
K = 249.4435;        % Constante de torque
rm = 1282.8556;      % Resistência do motor
Ior = 0.1800;        % Corrente de rotor bloqueado
Or = 0.5156;         % Rotor bloqueado
O_75 = 0.6266;       % 75% do bloqueio do rotor
O_50 = 0.7813;       % 50% do bloqueio do rotor
B = 1.1851;          % Controle do motor
Im = 5.7176;         % Medida
xm = 40.3913;        % Medida
A1 = 12.0000;        % Medida
I_2 = 12.0584;       % Medida
Pf_2 = 0.9952;       % Medida
r2 = 0.4134;         % Medida
r1 = 0.6214;         % Medida
C_ = 0.5445;         % Medida
x2 = 0.7591;         % Medida
x1 = 0.5063;         % Medida
P = 2;               % Medida

E_2 = abs((r2 + x2*1i) * I_2);  % Calcula E2

%% Velocidade Síncrona e Nominal
n_sync = 120 * f / P;              % Velocidade Síncrona em RPM
w_sync = n_sync * 2 * pi / 60;     % Velocidade Síncrona em rad/s
V = 3515;                          % Velocidade nominal em RPM

%% Calcula a Tensão e Impedância de Thévenin
v_th = v_phase * (xm / sqrt(r1^2 + (x1 + xm)^2));  % Tensão de Thévenin
z_th = ((1i*xm) * (r1 + 1i*x1)) / (r1 + 1i*(x1 + xm));  % Impedância de Thévenin
r_th = real(z_th);                % Parte real da impedância de Thévenin
x_th = imag(z_th);                % Parte imaginária da impedância de Thévenin

%% Velocidade de Parada e Escorregamento
nm_stop = 875;                     % Velocidade mecânica de parada em RPM
s_stop = 1 - nm_stop / n_sync;     % Escorregamento correspondente

%% Cálculo do Torque e Corrente em Função da Velocidade
velocidade = linspace(0, n_sync, 101);  % Vetor de velocidades

t_ind1 = zeros(1, length(velocidade)); % Inicializa vetor de torque
i_pu_normalized = zeros(1, length(velocidade));  % Inicializa vetor de corrente

for ii = 1:length(velocidade)
    s = (n_sync - velocidade(ii)) / n_sync;  % Calcula o escorregamento
    if s == 0
        s = 1e-6;  % Evita divisão por zero
    end
    % Calcula o torque eletromagnético em p.u.
    t_ind1(ii) = (3 * v_th^2 * r2 / s) / (w_sync * ((r_th + r2/s)^2 + (x_th + x2)^2));
    
    % Calcula a corrente de estator em p.u.
    i_pu_normalized(ii) = v_th / sqrt((r_th + r2/s)^2 + (x_th + x2)^2);
end

% Normaliza o torque em p.u.
t_ind1_pu = (t_ind1 / T_nom) / 1.4937;

% Normaliza a corrente em p.u.
i_pu_normalized = (i_pu_normalized / Ir) / 5.2688;

%% Plotagem dos Resultados
figure;
yyaxis left;
plot(velocidade / n_sync * 100, t_ind1_pu, 'r', 'LineWidth', 2.0, 'LineStyle', ':');
ylabel('Torque (p.u.)');
ylim([min(t_ind1_pu), max(t_ind1_pu) * 1.01]);
title('Torque e Corrente vs. Velocidade - Simulação MATLAB');
grid on;

yyaxis right;
plot(velocidade / n_sync * 100, i_pu_normalized, 'b', 'LineWidth', 2.0);
ylabel('Corrente (p.u.)');
xlabel('Velocidade Síncrona (%)');
legend({'Torque', 'Corrente'}, 'Location', 'southwest', 'FontSize', 12);
