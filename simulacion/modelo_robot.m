function [estadoSiguiente, controlAplicado, info] = modelo_robot( ...
    estadoActual, controlSolicitado, Ts, robot)
% Actualiza el estado del robot mediante el modelo de uniciclo.
%
%   estadoSiguiente = MODELO_ROBOT( ...
%       estadoActual,controlSolicitado,Ts,robot)
%
%   [estadoSiguiente,controlAplicado,info] = MODELO_ROBOT( ...
%       estadoActual,controlSolicitado,Ts,robot)
%
%   Integra durante un periodo de muestreo el modelo cinematico discreto:
%
%       x(k+1)     = x(k)     + Ts*v(k)*cos(theta(k))
%       y(k+1)     = y(k)     + Ts*v(k)*sin(theta(k))
%       theta(k+1) = theta(k) + Ts*w(k)
%
%   La orientacion final se normaliza al intervalo [-pi,pi].
%
%   Entradas:
%       estadoActual
%           Estado del robot:
%
%               [x y theta]
%
%           x, y  : posicion en el plano                     [m]
%           theta : orientacion                              [rad]
%
%       controlSolicitado
%           Accion solicitada por el controlador:
%
%               [v w]
%
%           v : velocidad lineal                             [m/s]
%           w : velocidad angular                            [rad/s]
%
%       Ts
%           Periodo de muestreo                              [s]
%
%       robot
%           Estructura obtenida mediante configuracion_robot.m.
%
%   Salidas:
%       estadoSiguiente
%           Estado actualizado [x y theta].
%
%       controlAplicado
%           Control realmente aplicado despues de imponer los limites
%           cinematicos definidos en robot.limites.
%
%       info
%           Estructura de diagnostico:
%
%               .controlSolicitado
%               .controlAplicado
%               .controlSaturado
%               .saturacionVelocidadLineal
%               .saturacionVelocidadAngular
%               .derivadaEstado
%               .incrementoPosicion
%               .incrementoOrientacion
%               .distanciaRecorridaPaso
%               .metodoIntegracion
%
%   La saturacion constituye la ultima proteccion del modelo fisico. Los
%   controladores APF y MPC deben generar normalmente controles dentro de
%   los limites, pero MODELO_ROBOT impide que una orden fuera de rango se
%   aplique durante la simulacion.
%
%   Para el registro experimental debe guardarse controlAplicado, ya que
%   representa la accion que realmente recibe el robot.
%
%   Ejemplo:
%
%       cfg = parametros_generales("batch");
%       robot = configuracion_robot();
%
%       estado = [1 1 0];
%       control = [0.8 0];
%
%       [estadoSiguiente,controlAplicado,info] = modelo_robot( ...
%           estado,control,cfg.sim.Ts,robot);
%
%       % estadoSiguiente = [1.12 1.00 0.00]
%
%   Esta funcion utiliza:
%       - wrap_to_pi_local.m

%% Validacion y normalizacion de entradas
[estadoActual, controlSolicitado, Ts, limitesControl, ...
    metodoIntegracion] = validar_entradas( ...
        estadoActual,controlSolicitado,Ts,robot);

% El estado interno se mantiene siempre con orientacion normalizada.
estadoActual(3) = wrap_to_pi_local(estadoActual(3));

%% Aplicacion de los limites cinematicos
controlAplicado = min( ...
    max(controlSolicitado,limitesControl(1,:)), ...
    limitesControl(2,:));

tolerancia = 1e-12*max(1,max(abs([ ...
    controlSolicitado,controlAplicado,limitesControl(:).'])));

saturacion = abs(controlAplicado-controlSolicitado) > tolerancia;

%% Modelo cinematico de uniciclo
x = estadoActual(1);
y = estadoActual(2);
theta = estadoActual(3);

v = controlAplicado(1);
w = controlAplicado(2);

derivadaEstado = [
    v*cos(theta), ...
    v*sin(theta), ...
    w
];

%% Integracion mediante Euler explicito
estadoSiguiente = [
    x + Ts*v*cos(theta), ...
    y + Ts*v*sin(theta), ...
    wrap_to_pi_local(theta + Ts*w)
];

%% Informacion de diagnostico
incrementoPosicion = ...
    estadoSiguiente(1:2)-estadoActual(1:2);

incrementoOrientacion = wrap_to_pi_local( ...
    estadoSiguiente(3)-estadoActual(3));

info = struct();

info.estadoActual = estadoActual;
info.estadoSiguiente = estadoSiguiente;

info.controlSolicitado = controlSolicitado;
info.controlAplicado = controlAplicado;

info.controlSaturado = any(saturacion);
info.saturacionVelocidadLineal = saturacion(1);
info.saturacionVelocidadAngular = saturacion(2);

info.derivadaEstado = derivadaEstado;
info.incrementoPosicion = incrementoPosicion;
info.incrementoOrientacion = incrementoOrientacion;
info.distanciaRecorridaPaso = norm(incrementoPosicion);

info.periodoMuestreo = Ts;
info.metodoIntegracion = metodoIntegracion;
info.modelo = "uniciclo";
end

%% ========================================================================
% VALIDACION
% ========================================================================

function [estado,control,Ts,limites,metodo] = validar_entradas( ...
    estado,control,Ts,robot)
%VALIDAR_ENTRADAS Comprueba la coherencia del modelo y de sus entradas.

%% Estructura del robot
if ~isstruct(robot) || ~isscalar(robot)
    error('modelo_robot:RobotNoValido', ...
        'robot debe ser la estructura obtenida con configuracion_robot.m.');
end

camposRobot = {'tipo','estado','control','limites','movimiento'};

for i = 1:numel(camposRobot)
    if ~isfield(robot,camposRobot{i})
        error('modelo_robot:ConfiguracionIncompleta', ...
            'Falta el campo robot.%s.',camposRobot{i});
    end
end

if string(robot.tipo) ~= "uniciclo"
    error('modelo_robot:ModeloNoSoportado', ...
        'Esta funcion implementa exclusivamente el modelo de uniciclo.');
end

if ~isstruct(robot.estado) || ...
        ~isfield(robot.estado,'dimension') || ...
        robot.estado.dimension ~= 3
    error('modelo_robot:EstadoConfiguradoNoValido', ...
        'El modelo de uniciclo debe tener tres variables de estado.');
end

if ~isstruct(robot.control) || ...
        ~isfield(robot.control,'dimension') || ...
        robot.control.dimension ~= 2
    error('modelo_robot:ControlConfiguradoNoValido', ...
        'El modelo de uniciclo debe tener dos entradas de control.');
end

%% Estado
if ~isnumeric(estado) || ~isreal(estado) || ...
        numel(estado) ~= robot.estado.dimension || ...
        any(~isfinite(estado(:)))
    error('modelo_robot:EstadoNoValido', ...
        'estadoActual debe ser un vector real y finito [x y theta].');
end

estado = reshape(double(estado),1,3);

%% Control
if ~isnumeric(control) || ~isreal(control) || ...
        numel(control) ~= robot.control.dimension || ...
        any(~isfinite(control(:)))
    error('modelo_robot:ControlNoValido', ...
        'controlSolicitado debe ser un vector real y finito [v w].');
end

control = reshape(double(control),1,2);

%% Periodo de muestreo
if ~isnumeric(Ts) || ~isscalar(Ts) || ~isreal(Ts) || ...
        ~isfinite(Ts) || Ts <= 0
    error('modelo_robot:TsNoValido', ...
        'Ts debe ser un escalar real, finito y positivo.');
end

Ts = double(Ts);

%% Limites cinematicos
camposLimites = {'vMin','vMax','wMin','wMax'};

for i = 1:numel(camposLimites)
    if ~isfield(robot.limites,camposLimites{i})
        error('modelo_robot:LimitesIncompletos', ...
            'Falta robot.limites.%s.',camposLimites{i});
    end
end

limites = [
    robot.limites.vMin, robot.limites.wMin;
    robot.limites.vMax, robot.limites.wMax
];

if ~isnumeric(limites) || ~isreal(limites) || ...
        any(~isfinite(limites(:))) || ...
        limites(1,1) > limites(2,1) || ...
        limites(1,2) > limites(2,2)
    error('modelo_robot:LimitesNoValidos', ...
        'Los limites cinematicos del robot no son coherentes.');
end

limites = double(limites);

%% Metodo de integracion
if ~isstruct(robot.movimiento) || ...
        ~isfield(robot.movimiento,'modeloDiscreto')
    error('modelo_robot:MetodoAusente', ...
        'Falta robot.movimiento.modeloDiscreto.');
end

metodo = string(robot.movimiento.modeloDiscreto);

if metodo ~= "Euler_explicito"
    error('modelo_robot:MetodoNoSoportado', ...
        'Esta version requiere el metodo "Euler_explicito".');
end
end
