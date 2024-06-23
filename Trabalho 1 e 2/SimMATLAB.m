close all; clc; clear;
%% 

% Parâmetros fornecidos na tabela
Hp = 10;
Vl_line = 400;
v_phase = 230.94;
f = 60;
Eff = 0.885;
Ef_75 = 0.885;
Eff_50 = 0.875;
Pff = 0.87;
Pf_75 = 0.810;
Pf_50 = 0.710;
Ir = 14;
Ts = 210;
Tm = 250;
T_nom = 20.6; % Torque nominal = reted torque

StartingTorque = Ts / 100 * T_nom;

I_75 = 11.2655;
I_50 = 8.6660;
rt = 1.0347;
K = 249.4435;
rm = 1282.8556;
Ior = 0.1800;
Or = 0.5156;
O_75 = 0.6266;
O_50 = 0.7813;
B = 1.1851;
Im = 5.7176;
xm = 40.3913;
A1 = 12.0000;
I_2 = 12.0584;
Pf_2 = 0.9952;
r2 = 0.4134;
r1 = 0.6214;
C_ = 0.5445;
x2 = 0.7591;
x1 = 0.5063;
P = 2;

E_2 = abs((r2 + x2*1i) * I_2);

% Velocidade Sincrona
n_sync = 120 * f / P; % Velocidade Síncrona em RPM
w_sync = n_sync * 2 * pi / 60; % Velocidade Síncrona em rad/s

% Velocidade Nominal
V = 3515; % rpm

%% Calcula a tensão e a impedância de Thévenin com as Equações
v_th = v_phase * (xm / sqrt(r1^2 + (x1 + xm)^2));
z_th = ((1i*xm) * (r1 + 1i*x1)) / (r1 + 1i*(x1 + xm));
r_th = real(z_th);
x_th = imag(z_th);

% Definindo a velocidade mecânica de parada (875 rpm)
nm_stop = 875;
s_stop = 1 - nm_stop / n_sync;

% Calcula o escorregamento e a velocidade mecânica correspondente
%s = (1:-0.001:s_stop); % Escorregamento
%nm = (1 - s) * n_sync; % Velocidade mecânica
velocidade = linspace(0, n_sync, 101);


% Calcula o torque eletromagnético para a resistência de rotor original:
t_ind1 = zeros(1, length(velocidade));

teste = zeros(1, length(velocidade));
for ii = 1:length(velocidade)
    s = (n_sync - velocidade(ii))/n_sync;
    if s == 0
        s = 1e-6; % Evitar divisão por zero
    end
    t_ind1(ii) = (3 * v_th^2 * r2 / s) /(w_sync * ((r_th + r2/s)^2 + (x_th + x2)^2)) ;
end
%t_ind1(1) = torque_partida_desejado;
% Normalizando o torque (conjugado) em unidades por unidade (p.u.)
t_ind1_pu = (t_ind1/T_nom) / 1.4937;

% Calcula a corrente de estator em p.u. para cada velocidade
i_pu = zeros(1, length(s));
for ii = 1:length(velocidade)
    s = (n_sync - velocidade(ii))/n_sync;
    if s == 0
        s = 1e-6; % Evitar divisão por zero
    end
    i_pu(ii) = v_th / sqrt((r_th + r2/s)^2 + (x_th + x2)^2);
end

% Normaliza a corrente em p.u.
i_pu_normalized = (i_pu / Ir) /5.2688;

% Plota a curva de torque x velocidade e corrente x velocidade no mesmo gráfico
figure;
yyaxis left;
plot(velocidade / n_sync * 100, t_ind1_pu, Color = 'r', LineWidth = 2.0, LineStyle=':');
ylabel('Torque (p.u.)');
ylim([min(t_ind1_pu), max(t_ind1_pu)*1.01])
title('Torque e Corrente vs velocidade do MIT - Simulação MATLAB');
grid on;

yyaxis right;
plot(velocidade / n_sync * 100, i_pu_normalized, 'Color', 'b', 'LineWidth', 2.0);
ylabel('\bf Corrente (p.u.)');
%ylim([min(t_ind1_pu), max(t_ind1_pu)*1.01])
xlabel('n_{sync} (%)')


legend({'\tau', 'I'}, Location = 'southwest', FontSize=20);
