close all; clc; clear;
% Cesar Augusto Mendes Cordeiro da Silva
% João Paulo Souza da Silva
% João Victor Ferro Amorosini 
% Thiago Daniel Leptokarydis
%% Ex 10) Considere os circuitos da Figura 4, com os seguintes valores: 
%  a) V=180sin⁡(ωt);
%  b) R = 0.3; 
%  c) L = 20 mH; 
%  d) C = 100 μF. 
%  Considere ω=2π60 rad/s. Crie um script no MATLAB que realize as
%  seguintes funções:

%  a) Apresente o valor da impedância equivalente vista pela fonte, no
%  formato polar (módulo e fase), para os circuitos a) e b).

%  b) Apresente o valor da reatância indutiva XL e XC.

%  c) Para cada circuito, crie uma janela de plotagem, considerando 5 
%  ciclos de V. Cada janela de plotagem conterá dois gráficos, que estarão
%  dispostos lado a lado. Na janela da esquerda, plote iR, iL, iC conjunta-
%  mente com V. Na janela da direita, plote VR, VL e Vc  conjuntamente com 
%  V. Em cada uma dessas janelas, crie dois eixos de ordenadas: um para V 
%  e outro para o grupo de variáveis calculadas em cada caso. Formate o 
%  gráfico convenientemente, adicionando título, nome para os eixos e 
%  legendas.
%% 
% Variaveis do circuito:
V_amplitude = 180; % [V]
R = 0.3;           % [Ω] 
L = 20e-3;         % [H]
C = 100e-6;        % [F]
f = 60;            % [f]
w = 2*pi*f;        % [rad/s]

%% a)
% Impedância relativa vista pela fonte (série)
z_eq_serie = R + 1i * w * L + 1 / (1i * w * C);
fprintf('Impedância relativa (série):\n %.2f + %.2fi\n', real(z_eq_serie), imag(z_eq_serie));

% Impedância relativa vista pela fonte (paralelo)
z_eq_paralelo = 1 / ((1 / R) + (1 / (1i * w * L)) + (1 / (1 / (1i * w * C))));
fprintf('Impedância relativa (paralelo):\n %.2f + %.2fi\n', real(z_eq_paralelo), imag(z_eq_paralelo));

%% b)
% Reatância indutiva e capacitiva
XL = w * L;
XC = 1 / (w * C);

fprintf('Reatância indutiva:\n %.2f\n', XL);
fprintf('Reatância capacitiva:\n %.2f\n', XC);

%% c)
% t para 5 ciclos do sinal de tensão V
t = linspace(0, 5 * (1 / f), 1000); % 1000 pontos para 5 ciclos

% Vin -> V(t)
V = V_amplitude * sin(w*t);

% Circuito RLC Série - Correntes: 
iR_serie = V / (sqrt(R^2 + (XL - XC)^2));
iL_serie = V / (sqrt(R^2 + (XL - XC)^2));
iC_serie = V / (sqrt(R^2 + (XL - XC)^2));

% Circuito RLC Série - Tensões: 
VR_serie = iR_serie * R;
VL_serie = iL_serie * XL;
VC_serie = iC_serie * XC;

% Circuito RLC Paralelo - Correntes: 
iR_paralelo = V / R;
iL_paralelo = V / XL;
iC_paralelo = V / XC;

% Circuito RLC Paralelo - Correntes: 
VR_paralelo = V;
VC_paralelo = V;
VL_paralelo = V;

% Circuito RLC Série - plot:
figure(1);
sgtitle('Circuito RLC Série');

% Circuito RLC Série - Corrente e Tensão
subplot(1, 2, 1);
yyaxis left
plot(t, V, 'k');
hold on;

plot(t, abs(iR_serie), 'r-');
plot(t, abs(iL_serie), 'Color', '#42bd59', LineStyle='-.');
plot(t, abs(iC_serie), 'b--');

ylabel('V [V]');
yyaxis right
ylabel('i [A]')
title('i(t) e V(t)');
xlabel('t (s)');
legend('V', '|iR|', '|iL|', '|iC|');

% Circuito RLC Série - Tensão e Tensão
subplot(1, 2, 2);
plot(t, V, 'k');
hold on;

plot(t, abs(VR_serie), 'r-');
plot(t, abs(VL_serie), 'Color', '#42bd59', LineStyle='-.');
plot(t, abs(VC_serie), 'b--');

ylabel('V [V]');
title('Vin(t) e V(t)');
xlabel('t [s]');
legend('V', '|VR|', '|VL|', '|VC|');

% Circuito RLC Paralelo - plot:
figure(2);
sgtitle('Circuito RLC Paralelo');

% Circuito RLC Paralelo - Corrente e Tensão
subplot(1, 2, 1);
yyaxis left
plot(t, V, 'k');
hold on;
plot(t, abs(iR_paralelo), 'r-');
plot(t, abs(iL_paralelo), 'Color', '#42bd59', LineStyle='-.');
plot(t, abs(iC_paralelo), 'b--');

ylabel('V [V]');
title('i(t) e V(t)');
yyaxis right
ylabel('i [A]');
xlabel('t [s]');
legend('V', '|iR|', '|iL|', '|iC|');

% Circuito RLC Paralelo - Tensão e Tensão
subplot(1, 2, 2);
plot(t, V, 'k');
hold on;

plot(t, abs(VR_paralelo), 'r-');
plot(t, abs(VL_paralelo), 'Color', '#42bd59', LineStyle='-.');
plot(t, abs(VC_paralelo), 'b--');

ylabel('V [V]');
title('Vin(t) e V(t)');
xlabel('t [s]');
legend('V', '|VR|', '|VL|', '|VC|');