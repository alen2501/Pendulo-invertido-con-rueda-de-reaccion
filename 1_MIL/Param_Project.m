% Fichero que contiene los parámetros de un pendulo invertido con rueda de 
% reacción y su servomotor, así como las matrices A, B, C y D, y los 
% vectores de estado iniciales, x0 = x(t=0), correspondientes al
% modelo de simulación.
%
% Revisado el 06 de junio de 2026; Alen Garcia
% Actualizado al Modelo Dinámico Acoplado y Hardware Real (DFRobot FIT0493)
clc; clear; close all;

%% 1. PARÁMETROS FÍSICOS (Inspirado en el Paper del ITBA)
g  = 9.81;           % Constante gravitatoria [m/s^2]

% --- La Barra (Plástico 3D, ligero y corto como recomienda el paper) ---
mb = 0.1;           % 100 gramos (PLA/PETG con poco relleno)
L_barra = 0.207;      % 20 centímetros (Similar a los 20.7 cm del paper)
lb = L_barra / 2;    % Centro de masa de la barra

% --- El Motor (DFRobot FIT0493 con reductora 34:1) ---
mm = 0.098;          % 98 gramos (Según hoja de datos del FIT0493) 
L  = L_barra;        % Montado arriba

% --- La Rueda de Reacción (Impresa 3D + Tuercas en el borde exterior) ---
mr = 0.12;           % 120 gramos (30g plástico + 90g de tuercas metálicas en el borde)
radio_rueda = 0.105;  % 8 centímetros de radio (16 cm de diámetro total)

% --- Parámetros Eléctricos del Motor (Con Reductora) ---
Ra = 2.18;           % Resistencia de inducido [ohm] (V/Istall = 12/5.5)
La = 0.001;          % Inductancia [H] (Mantenemos un valor bajo típico)

% Al tener reductora 34:1, consideramos los valores mecánicos equivalentes 
% ya medidos a la salida del eje para simplificar el modelo:
kt = 0.214;          % Constante de par multiplicada por reductora [N·m/A] (Tau_stall / I_stall)
kb = 0.327;          % Constante contraelectromotriz [V/(rad/s)] (V / w_noload)

%% 2. CÁLCULO DE INERCIAS (Modificado para rueda tipo anillo)
% El paper demuestra que la masa debe ir en el borde. 
% La inercia de un anillo es M*R^2 (el doble que un disco macizo 1/2*M*R^2)
Ir = mr * radio_rueda^2; 

% Inercias respecto al PIVOTE base (Teorema de Steiner)
Ib_pivote = (1/3) * mb * L_barra^2;  
Im_pivote = mm * L^2;                
Ir_pivote = mr * L^2;                

% Inercia Total y Momento Gravitatorio
J = Ib_pivote + Im_pivote + Ir_pivote;
Mg = (mb * lb) + (mm * L) + (mr * L);

%% 3. MODELO DE SIMULACIÓN
% Vector de entrada: u = [va]'
% Vector de salida:  y = [theta, theta', wr, ia]'

% Matriz A (Dinámica de la planta acoplada)
mA = [       0,   1,        0,                 0;
      (Mg*g)/J,   0,        0,             -kt/J;
     -(Mg*g)/J,   0,        0,  (kt/Ir) + (kt/J); 
             0,   0,   -kb/La,            -Ra/La ];

% Matriz B (Efecto del actuador)
mB = [0; 
      0; 
      0; 
      1/La];

% Matriz C (Sensores: theta y wr)
mC = eye(4);

% Matriz D (Transmisión directa)
mD = [0; 0; 0; 0];

% Condiciones iniciales: 
% x(t=0) = [theta(t=0) theta'(t=0) wr(t=0) ia(t=0)]'
x0 = [0 0 0 0]';

%% 4. MODELO DE CONTROL (Observador de Kalman y LQR)
% Se asume La = 0 -> ia = (va - kb*wr)/Ra (Ecuación puramente algebraica)
% Vector de estados: x = [theta, theta', wr]'
% Vector de entrada: u = [va]
% Vector de salida:  y = [theta, theta', wr]'

% Matriz A reducida
mA_con = [       0, 1,                                       0;
          (Mg*g)/J, 0,                          (kt*kb)/(J*Ra);
         -(Mg*g)/J, 0, -((kt*kb)/(Ir*Ra)) - ((kt*kb)/(J*Ra)) ];

% Matriz B reducida
mB_con = [                                0; 
                                 -kt/(J*Ra); 
          (kt/(Ir*Ra)) + (kt/(J*Ra)) ];

% Matriz C reducida
mC_con = [    1, 0,       0;
              0, 1,       0;
              0, 0,       1;
              0, 0, -kb/Ra ];

% Matriz D reducida
mD_con = [    0; 
              0; 
              0; 
           1/Ra ];

% Condiciones iniciales: 
% x_red(t=0) = [theta(t=0) theta'(t=0) wr(t=0)]'
x0_con = [0 0 0]';