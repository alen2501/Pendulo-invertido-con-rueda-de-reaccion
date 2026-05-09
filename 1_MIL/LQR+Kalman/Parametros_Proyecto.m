% Fichero que contiene los parámetros de un pendulo invertido con rueda de 
% reacción y su servomotor, así como las matrices A, B, C y D, y los 
% vectores de estado iniciales, x0 = x(t=0), correspondientes al
% modelo de simulación.
%
% Revisado el 24 de abril de 2026; Alen Garcia
% Actualizado al Modelo Dinámico Acoplado

clc; clear; close all;

%% 1. PARÁMETROS FÍSICOS (Inspirado en el Paper del ITBA)
g  = 9.81;           % Constante gravitatoria [m/s^2]

% --- La Barra (Plástico 3D, ligero y corto como recomienda el paper) ---
mb = 0.06;           % 60 gramos (PLA/PETG con poco relleno)
L_barra = 0.20;      % 20 centímetros (Similar a los 20.7 cm del paper)
lb = L_barra / 2;    % Centro de masa de la barra

% --- El Motor (Ej: Pololu #4842 con reductora ~10:1) ---
mm = 0.095;          % 95 gramos 
L  = L_barra;        % Montado arriba

% --- La Rueda de Reacción (Impresa 3D + Tuercas en el borde exterior) ---
mr = 0.12;           % 120 gramos (30g plástico + 90g de tuercas metálicas en el borde)
radio_rueda = 0.08;  % 8 centímetros de radio (16 cm de diámetro total)

% --- Parámetros Eléctricos del Motor (Con Reductora) ---
Ra = 2.4;            % Resistencia de inducido [ohm]
La = 0.001;          % Inductancia [H]
% Al tener reductora ~10:1, el par se multiplica por 10 y la velocidad baja.
kt = 0.075;          % Constante de par multiplicada por la reductora [N·m/A]
kb = 0.11;           % Constante contraelectromotriz [V/(rad/s)]

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
%% 3. MODELO EN ESPACIO DE ESTADOS
% Vector de estados: x = [theta, theta', wr, ia]'
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