function robot = configuracion_robot()
% Define el modelo y los limites fisicos del robot.
%
%   robot = CONFIGURACION_ROBOT()
%
%   El robot se modela como un robot movil terrestre de tipo uniciclo,
%   compatible con la formulacion utilizada por los controladores APF y MPC:
%
%       estado  x = [px py theta]
%       control u = [v w]
%
%       px(k+1)    = px(k)    + Ts*v(k)*cos(theta(k))
%       py(k+1)    = py(k)    + Ts*v(k)*sin(theta(k))
%       theta(k+1) = theta(k) + Ts*w(k)
%
%   donde:
%       px, py : posicion en el plano                         [m]
%       theta  : orientacion                                  [rad]
%       v      : velocidad lineal                             [m/s]
%       w      : velocidad angular                            [rad/s]
%
%   El periodo de muestreo Ts no se define aqui, sino en
%   parametros_generales.m, para que sea identico en todas las
%   arquitecturas experimentales.
%
%   Esta funcion no contiene la posicion inicial ni la meta. Esos datos
%   pertenecen al escenario y se obtienen mediante escenarios.m.
%
%   Ejemplo:
%
%       cfg = parametros_generales("visual");
%       escenario = escenarios("media");
%       robot = configuracion_robot();
%
%       estado = escenario.inicio;
%       control = [robot.limites.vMax 0];
%
%       estadoSiguiente = [
%           estado(1) + cfg.sim.Ts*control(1)*cos(estado(3)), ...
%           estado(2) + cfg.sim.Ts*control(1)*sin(estado(3)), ...
%           atan2(sin(estado(3) + cfg.sim.Ts*control(2)), ...
%                 cos(estado(3) + cfg.sim.Ts*control(2)))
%       ];

%% Identificacion del modelo
robot = struct();

robot.nombre = "Robot movil terrestre";
robot.tipo = "uniciclo";
robot.descripcion = [ ...
    "Modelo cinematico planar con velocidad lineal y velocidad angular. " ...
    "Se utiliza de forma comun en RRT*+APF, RRT*+MPC y PRM+MPC."];

%% Variables del modelo
robot.estado.nombre = ["x", "y", "theta"];
robot.estado.dimension = 3;
robot.estado.unidades = ["m", "m", "rad"];

robot.control.nombre = ["v", "w"];
robot.control.dimension = 2;
robot.control.unidades = ["m/s", "rad/s"];

%% Geometria
% El robot se representa mediante un disco para planificacion, deteccion
% de colisiones y visualizacion.
robot.geometria.forma = "circulo";
robot.geometria.radio = 0.25;          % [m]
robot.geometria.diametro = 2*robot.geometria.radio;
robot.geometria.huella = "disco";

%% Limites cinematicos
% Valores conservados de las implementaciones funcionales de referencia.
robot.limites.vMin = 0.00;             % [m/s]
robot.limites.vMax = 0.80;             % [m/s]
robot.limites.wMin = -1.40;            % [rad/s]
robot.limites.wMax = 1.40;             % [rad/s]

%% Condiciones operativas
robot.movimiento.permiteRetroceso = false;
robot.movimiento.giroEnElSitio = true;
robot.movimiento.modeloDiscreto = "Euler_explicito";
robot.movimiento.normalizacionAngular = "atan2_sin_cos";

%% Valores iniciales de control
robot.controlInicial = [0 0];
robot.controlParada = [0 0];

%% Convenciones de seguridad
% Los margenes adicionales no se incluyen en el radio fisico, porque se
% definen de forma comun en parametros_generales.m.
robot.seguridad.radioFisico = robot.geometria.radio;
robot.seguridad.incluirMargenExterno = true;
robot.seguridad.origenMargen = "parametros_generales";

%% Validacion interna
validar_configuracion(robot);
end

%% ========================================================================
% FUNCIONES LOCALES
% ========================================================================

function validar_configuracion(robot)
%VALIDAR_CONFIGURACION Comprueba la coherencia interna del modelo.

if robot.tipo ~= "uniciclo"
    error('configuracion_robot:ModeloNoValido', ...
        'El modelo experimental debe ser de tipo uniciclo.');
end

if robot.estado.dimension ~= numel(robot.estado.nombre) || ...
        robot.estado.dimension ~= numel(robot.estado.unidades)
    error('configuracion_robot:EstadoIncoherente', ...
        'La definicion del estado no es coherente.');
end

if robot.control.dimension ~= numel(robot.control.nombre) || ...
        robot.control.dimension ~= numel(robot.control.unidades)
    error('configuracion_robot:ControlIncoherente', ...
        'La definicion del control no es coherente.');
end

if ~isscalar(robot.geometria.radio) || ...
        ~isfinite(robot.geometria.radio) || robot.geometria.radio <= 0
    error('configuracion_robot:RadioNoValido', ...
        'El radio del robot debe ser un escalar positivo.');
end

if robot.limites.vMin < 0 || ...
        robot.limites.vMax <= robot.limites.vMin
    error('configuracion_robot:VelocidadLinealNoValida', ...
        'Los limites de velocidad lineal no son validos.');
end

if robot.limites.wMin >= 0 || robot.limites.wMax <= 0 || ...
        abs(robot.limites.wMin + robot.limites.wMax) > 1e-12
    error('configuracion_robot:VelocidadAngularNoValida', ...
        'Los limites angulares deben ser simetricos respecto de cero.');
end

if ~isequal(size(robot.controlInicial),[1 2]) || ...
        ~isequal(size(robot.controlParada),[1 2])
    error('configuracion_robot:ControlInicialNoValido', ...
        'Los controles inicial y de parada deben tener formato 1 x 2.');
end

if any(robot.controlInicial < ...
        [robot.limites.vMin robot.limites.wMin]) || ...
        any(robot.controlInicial > ...
        [robot.limites.vMax robot.limites.wMax])
    error('configuracion_robot:ControlInicialFueraLimites', ...
        'El control inicial queda fuera de los limites cinematicos.');
end
end
