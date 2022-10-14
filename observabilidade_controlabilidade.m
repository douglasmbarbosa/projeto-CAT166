clear 
close all
clc 

%Dados utilizados no artigo

r1=0.05; r2=0.07; m=2;
J1=m*r1^2/2;
J2=m*r2^2/2;
k=4;
k1=4; k2=4;k3=4;
T=1/J1;

%Passando as matrizes de estados

A = [0 0 1 0; 0 0 0 1; -(k1+k2)/J1 k2/J1 0 0; k2/J2 -(k2+k3)/J2 0 0];
B = [0 ; 0 ; T ; 0];
%C = [1 0 0 0];
C = [0 1 0 0];
D = [0];

%Obtendo função de transferência

[num,den] = ss2tf(A,B,C,D);

FT = tf(num,den)

n1 = dcgain(FT(1))
FT(1)= FT(1)*(1/n1)


%Obtendo matriz de controlabilidade (matriz 4x4)
Pc = [B A*B A^2*B A^3*B]

%Obtendo matriz de observabilidade (matriz 4x4)
Po = [C ; C*A ; C*A^2 ; C*A^3]

%Calculando os determinantes

det_c = det(Pc)
det_o = det(Po)

if det_c == 0 & det_o == 0
    disp("O sistema não é controlável e, também, não é observável")
elseif det_c == 0 & det_o ~= 0
    disp("O sistema não é controlável, mas é observável")
elseif det_c ~= 0 & det_o == 0
    disp("O sistema é controlável, mas não é observável")
elseif det_c ~= 0 & det_o ~= 0
    disp("O sistema é controlável e, também, é observável")
else 
    disp("Indefinido!")
end

%Controlando o sistema (Estabilidade)

%step(FT,4) %Sistema antes da matriz K
hold on

%Aplicando matriz K para para controle

sys = ss(A,B,C,D);
p = [-2,-4,-12,-8];
K = place(A,B,p);
Acl = A-B*K;
syscl = ss(Acl,B,C,D);
Pcl = pole(syscl)

%Estabilizando em 1

n = dcgain(syscl(1))
syscl(1)= syscl(1)*(1/n)

step(syscl)


