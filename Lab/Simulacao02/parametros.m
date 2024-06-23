close all; clc; clear;
% Cesar Augusto Mendes Cordeiro da Silva	211270121
% João Paulo Souza da Silva				    221270671
% João Victor Ferro Amorosini				221270817
% Thiago Daniel Leptokarydis				221270851

%%
T = 10;
t = 0:0.001:10;

tau1 = 30e-3;
k1 = 2;

tau2 = 500e-3;
k2 = 1;

KAO = T / (2 * k1 * tau1); % Amplitude Ótima
TnAO = T;

KSO = T / (2 * k2 * tau2); % Simetrico Ótimo
TnSO = 4 * tau2;

% GsAO = zeros(length(t));
% GsSO = zeros(length(t));
% for i = 1:length(t)
%     GsAO(i) = KAO * (1 + (1 / TnAO * t(i)));
%     GsSO(i) = KSO * (1 + (1 / TnSO * t(i)));
% end
% 
% step(GsAo+GsSO)