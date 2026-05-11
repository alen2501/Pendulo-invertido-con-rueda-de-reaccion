% Diseño de un controlador LQR+Kalman
%
% Revisado el 11 de mayo de 2026; Alen Garcia

%% Parámetros del Modelo
if(exist('mA','var')==0) % Para no repetir indefinidamente la carga de variables
    % Script con parámetros del sistema
    Parametros_Proyecto
    mA=mA_con; mB=mB_con; mC=mC_con; mD=mD_con;
end
deg_inicial=15;
%% Comprobación de Controlabilidad y de Observabilidad
% Obtenemos el número de estados (n=4) leyendo las filas de mA
n_estados = size(mA, 2); 

if(rank(ctrb(mA, mB)) ~= n_estados)
    fprintf('\n¡OJO! Sistema NO CONTROLABLE!!!\n\n')
    return
else
    fprintf('El sistema es CONTROLABLE.\n')
end

if(rank(obsv(mA, mC)) ~= n_estados)
    fprintf('\n¡OJO! Sistema NO OBSERVABLE!!!\n\n')
    return
else
    fprintf('El sistema es OBSERVABLE.\n\n')
end

%% Diseño del controlador LQR
% Q (Castigo del estado) y R (Castigo de energía)
% Regla de Bryson: La regla dice que el peso óptimo es la inversa del 
% cuadrado del máximo valor tolerable para esa variable.

%Maximo de 10 grados en theta
max_theta = 10 * (pi / 180);%
q1= 1/(max_theta^2);     
% Maximo 5 grad/s en theta
max_vel = 5 * (pi / 180);
q2= 1/(max_vel^2);      
% Maximo 1000 rpm en wr
max_wr = 1000 * (2*pi / 60);
q3= 1/(max_wr^2);        
% Maximo 12 V en va
va_max = 1; % se limita mucho va para evitar oscilaciones gigantes
r1=1/(va_max^2);

%  Sintonización basada en la Regla de Bryson
Q = diag([q1, q2, q3]);    % Pesos de: [theta, theta', wr]
R = r1;                    % Peso del Voltaje

% Cálculo de la ganancia K
K = lqr(mA, mB, Q, R);

% Comprobación de estabilidad (Polos en Lazo Cerrado)
polos_lc = eig(mA - mB*K)
disp('Polos del sistema controlado:');
disp(polos_lc);

%% Periodo de muestreo
% Criterio teniendo en cuenta pd(1)
w_max_lc = max([abs(imag(polos_lc)); abs(real(polos_lc))]);
h_recomendado = (2*pi) / (w_max_lc * 20);

fprintf('Criterio de muestreo:\n')
fprintf('  Frecuencia dominante : %.1f rad/s\n', w_max_lc)
fprintf('  h recomendado        : %.5f s  (%.0f Hz mínimo)\n', ...
        h_recomendado, 1/h_recomendado)

h = 0.0001;   % [s] — ajustar si el warning se activa

if h > h_recomendado
    warning(['h = %.4f s SUPERA h_recomendado = %.5f s.\n' ...
             'Riesgo de aliasing. Reduce h o simplifica el modelo ' ...
             '(3 estados, despreciando La).'], h, h_recomendado)
else
    fprintf('  h = %.4f s cumple el criterio.\n', h)
end
fprintf('\n')
%% Discretización del Modelo de Control
PlantaD=c2d(ss(mA,mB,mC,mD),h,'zoh');
mG=PlantaD.A; mH=PlantaD.B;
% Alternativamente, de acuerdo a las expresiones de las transparencias del tema
% mG=expm(mA*h); syms t; mH=double(int(expm(mA*t)*mB, t, 0, h));

%% Ganancia del control por realimentación de estados
Kd=dlqr(mG,mH,Q,R);

% Comprobación de estabilidad (Polos en Lazo Cerrado)
disp('Polos del sistema controlado:');
disp(eig(mG - mH*Kd));

fprintf('K  continuo: '); disp(K)
fprintf('Kd discreto: '); disp(Kd)

%% Filtro de Kalman

% Extraemos solo las filas que corresponden a sensores reales (theta y wr)
mC_medido = [1, 0, 0;  % Sensor 1: IMU (theta)
             0, 0, 1]; % Sensor 2: Encoder (wr)

% Aumentamos un poco la confianza en el modelo
Q_kalman = diag([1e-4, 1e-4, 1e-3]); 

% Reducimos un poco la desconfianza en los sensores
R_kalman = diag([1e-3, 1e-2]);

% 1. Calculamos la ganancia del filtro
[Ld ~, ~] = dlqe(mG, eye(3), mC_medido, Q_kalman, R_kalman);

% 2. Calculamos los polos EXACTOS del observador
polos_K_exactos = eig(mG - Ld * mC_medido);
polos_K_abs = abs(polos_K_exactos); % Sacamos la magnitud

% 3. Comprobación estricta (max() de un vector 4x1 da un solo número)
max_polo_K = max(polos_K_abs);
max_polo_LQR = max(abs(eig(mG - mH*Kd)));

disp('--- RESULTADOS DEL OBSERVADOR ---');
fprintf('Polo más lento del LQR: %.4f\n', max_polo_LQR);
fprintf('Polo más lento del Kalman: %.4f\n', max_polo_K);

if max_polo_K >= 1
    warning('¡EL FILTRO ES INESTABLE! (Polos >= 1). Ajusta q_val o r_val.');
elseif max_polo_K > max_polo_LQR
    warning('El filtro es estable, pero más LENTO que el LQR. El péndulo podría caerse.');
else
    disp('¡ÉXITO VERDADERO! El filtro es estable y más rápido que el controlador.');
end

disp('Magnitud de los 3 polos del Kalman (Todos deben ser < 1):');
disp(polos_K_abs);