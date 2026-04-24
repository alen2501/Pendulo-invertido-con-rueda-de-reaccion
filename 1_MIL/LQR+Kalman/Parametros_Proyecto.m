% Fichero que contiene los parámetros de un pendulo invertido con rueda de 
% reacción y su servomotor, así como las matrices A, B, C y D, y los 
% vectores de estado iniciales, x0 = x(t=0), correspondientes al
% modelo de simulación.
%
% Revisado el 24 de abril de 2026; Alen Garcia
% Actualizado al Modelo Dinámico Acoplado

clc; clear; close all;

%% 1. PARÁMETROS FÍSICOS
g  = 9.81;           % Constante gravitatoria [m/s^2]

% --- La Barra ---
mb = 0.2;            % Masa de la barra [kg]
L_barra = 0.2;       % Longitud total de la barra [m]
lb = L_barra / 2;    % Distancia del pivote al Centro de Masa de la barra [m]

% --- El Motor  ---
mm = 0.15;           % Masa del estator del motor [kg]
L  = L_barra;        % Distancia del pivote al motor [m]

% --- La Rueda de Reacción ---
mr = 0.1;            % Masa de la rueda [kg]
radio_rueda = 0.05;  % Radio geométrico de la rueda [m]

% --- Parámetros Eléctricos del Motor ---
Ra = 2.0;            % Resistencia de inducido [ohm]
La = 0.001;          % Inductancia de inducido [H]
kt = 0.05;           % Constante de par [N·m/A]
kb = 0.05;           % Constante de fuerza contraelectromotriz [V/(rad/s)]

%% 2. CÁLCULO DE INERCIAS Y MOMENTOS
% Inercia de la rueda respecto a su PROPIO eje de rotación (Disco macizo)
Ir = (1/2) * mr * radio_rueda^2; 

% Inercias respecto al PIVOTE base (Teorema de Steiner)
Ib_pivote = (1/3) * mb * L_barra^2;  % Inercia de la barra desde su extremo
Im_pivote = mm * L^2;                % Inercia del estator trasladado al pivote
Ir_pivote = mr * L^2;                % Inercia de la rueda trasladada al pivote (como masa puntual)

% Inercia Total del sistema respecto al pivote (J)
J = Ib_pivote + Im_pivote + Ir_pivote;

% Momento Gravitatorio estático total equivalente (Mg)
% Suma de (Masa * Distancia al pivote) de cada elemento
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