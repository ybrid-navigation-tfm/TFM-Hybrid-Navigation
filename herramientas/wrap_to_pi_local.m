function angulo = wrap_to_pi_local(angulo)
% Normaliza ángulos al intervalo [-pi, pi].
%
%   angulo = WRAP_TO_PI_LOCAL(angulo)
%
%   Normaliza uno o varios ángulos (en radianes) para que pertenezcan al
%   intervalo [-pi, pi].
%
%   Esta función es equivalente a wrapToPi de MATLAB, pero no requiere la
%   Robotics System Toolbox ni Mapping Toolbox, por lo que el proyecto es
%   completamente autónomo.
%
%   Entrada:
%       angulo : escalar, vector o matriz de ángulos [rad]
%
%   Salida:
%       angulo : mismo tamaño que la entrada, con todos los valores
%                normalizados en [-pi, pi].
%
%   Ejemplos:
%
%       wrap_to_pi_local(pi)
%       %  3.1416
%
%       wrap_to_pi_local(3*pi)
%       %  3.1416
%
%       wrap_to_pi_local(-4*pi/3)
%       %  2.0944
%
%       theta = [0 pi 3*pi -5*pi/2];
%       theta = wrap_to_pi_local(theta);
%
%   Se utiliza en:
%       - modelo_robot.m
%       - mpc.m
%       - apf.m
%       - actualizar_obstaculos.m
%       - cualquier integración cinemática del robot.

%% Validación

if ~isnumeric(angulo) || ~isreal(angulo)
    error('wrap_to_pi_local:EntradaNoValida', ...
        'La entrada debe ser un valor numérico real.');
end

%% Normalización

angulo = atan2(sin(angulo), cos(angulo));

end