function graficos = dibujar_robot(graficos, estadoRobot, robot)
% Dibuja la representacion grafica inicial del robot movil.
%
%   graficos = DIBUJAR_ROBOT(graficos,estadoRobot,robot)
%
%   Completa la figura creada por inicializar_figura.m mediante dos objetos
%   graficos persistentes:
%
%       - un disco azul que representa la huella fisica del robot;
%       - una flecha negra que indica su orientacion actual.
%
%   La funcion crea ambos objetos una unica vez. actualizar_graficos.m
%   modificara posteriormente sus propiedades XData, YData, UData y VData,
%   sin utilizar clf ni reconstruir la figura en cada iteracion.
%
%   Entradas:
%       graficos
%           Estructura obtenida mediante inicializar_figura.m. Debe
%           contener, como minimo:
%
%               graficos.activa
%               graficos.inicializada
%               graficos.ejes
%               graficos.capas.robot
%               graficos.objetos
%               graficos.estilo
%
%       estadoRobot
%           Estado actual del robot con formato:
%
%               [x y theta]
%
%           x, y  : posicion del centro del robot             [m]
%           theta : orientacion                               [rad]
%
%       robot
%           Estructura obtenida mediante configuracion_robot.m. Se utiliza:
%
%               robot.tipo
%               robot.geometria.forma
%               robot.geometria.radio
%
%   Salida:
%       graficos
%           Misma estructura de entrada, ampliada con:
%
%               graficos.objetos.robot
%               graficos.objetos.orientacionRobot
%               graficos.robotDibujado
%               graficos.radioRobot
%
%   Apariencia:
%       robot         disco azul semitransparente;
%       contorno      negro;
%       orientacion   flecha negra desde el centro del robot.
%
%   Modo batch:
%       Si graficos.activa es false, no se crea ningun objeto grafico y la
%       funcion devuelve inmediatamente la estructura recibida.
%
%   Ejemplo:
%
%       cfg = parametros_generales("visual");
%       escenario = escenarios("media");
%       robot = configuracion_robot();
%
%       graficos = inicializar_figura( ...
%           escenario,cfg,"RRT* + MPC");
%
%       graficos = dibujar_entorno( ...
%           graficos,escenario);
%
%       graficos = dibujar_robot( ...
%           graficos,escenario.inicio,robot);
%
%   Esta funcion no actualiza el camino global, la trayectoria ejecutada,
%   el planificador ni los obstaculos dinamicos.

%% Validacion y normalizacion
[graficos, estadoRobot, robot] = ...
    validar_entradas(graficos,estadoRobot,robot);

if ~graficos.activa
    return;
end

capas = graficos.capas;
estilo = graficos.estilo;
objetos = graficos.objetos;

%% Eliminacion segura de una representacion anterior
% Permite volver a llamar a la funcion sin superponer varios robots.
if isfield(objetos,'robot')
    eliminar_manejadores(objetos.robot);
end

if isfield(objetos,'orientacionRobot')
    eliminar_manejadores(objetos.orientacionRobot);
end

%% Geometria del robot
centro = estadoRobot(1:2);
orientacion = estadoRobot(3);
radio = robot.geometria.radio;

[xRobot,yRobot] = coordenadas_circulo( ...
    centro,radio,estilo.anguloCirculo);

%% Disco que representa la huella fisica
objetos.robot = patch( ...
    'Parent',capas.robot, ...
    'XData',xRobot, ...
    'YData',yRobot, ...
    'FaceColor',estilo.colorRobot, ...
    'FaceAlpha',estilo.alphaRobot, ...
    'EdgeColor','k', ...
    'LineWidth',1.0, ...
    'Tag','robot');

%% Flecha de orientacion
% La longitud coincide con el radio fisico para conservar la apariencia de
% los codigos funcionales de referencia.
componenteX = radio*cos(orientacion);
componenteY = radio*sin(orientacion);

flecha = quiver( ...
    graficos.ejes, ...
    centro(1), ...
    centro(2), ...
    componenteX, ...
    componenteY, ...
    0, ...
    'Color',estilo.colorOrientacionRobot, ...
    'LineWidth',estilo.anchoOrientacion, ...
    'MaxHeadSize',0.9);

flecha.Parent = capas.robot;
flecha.Tag = 'orientacion_robot';

objetos.orientacionRobot = flecha;

%% Incorporacion de los objetos a la salida
graficos.objetos = objetos;
graficos.robotDibujado = true;
graficos.radioRobot = radio;
graficos.estadoRobotInicial = estadoRobot;

% El renderizado se difiere hasta el drawnow del bucle principal.
end

%% ========================================================================
% UTILIDADES GRAFICAS
% ========================================================================

function [x,y] = coordenadas_circulo(centro,radio,angulos)
%COORDENADAS_CIRCULO Calcula el contorno circular del robot.

x = centro(1)+radio*cos(angulos);
y = centro(2)+radio*sin(angulos);
end

function eliminar_manejadores(manejadores)
%ELIMINAR_MANEJADORES Borra objetos graficos validos sin generar errores.

if isempty(manejadores)
    return;
end

manejadores = manejadores(isgraphics(manejadores));

if ~isempty(manejadores)
    delete(manejadores);
end
end

%% ========================================================================
% VALIDACION
% ========================================================================

function [graficos,estadoRobot,robot] = ...
    validar_entradas(graficos,estadoRobot,robot)
%VALIDAR_ENTRADAS Comprueba el contrato minimo del modulo.

%% Estructura grafica
if ~isstruct(graficos) || ~isscalar(graficos)
    error('dibujar_robot:GraficosNoValidos', ...
        ['graficos debe ser la estructura obtenida mediante ' ...
         'inicializar_figura.m.']);
end

camposGraficos = { ...
    'activa', ...
    'inicializada', ...
    'ejes', ...
    'capas', ...
    'objetos', ...
    'estilo'};

for i = 1:numel(camposGraficos)
    if ~isfield(graficos,camposGraficos{i})
        error('dibujar_robot:GraficosIncompletos', ...
            'Falta graficos.%s.',camposGraficos{i});
    end
end

if ~islogical(graficos.activa) || ~isscalar(graficos.activa)
    error('dibujar_robot:IndicadorVisualNoValido', ...
        'graficos.activa debe ser un escalar logico.');
end

% En modo batch no existen ejes ni capas graficas y no deben exigirse.
if graficos.activa
    if ~islogical(graficos.inicializada) || ...
            ~isscalar(graficos.inicializada) || ...
            ~graficos.inicializada
        error('dibujar_robot:FiguraNoInicializada', ...
            ['La figura debe crearse primero mediante ' ...
             'inicializar_figura.m.']);
    end

    if ~isscalar(graficos.ejes) || ...
            ~isgraphics(graficos.ejes,'axes')
        error('dibujar_robot:EjesNoValidos', ...
            'graficos.ejes no contiene unos ejes validos.');
    end

    if ~isstruct(graficos.capas) || ...
            ~isfield(graficos.capas,'robot') || ...
            ~isscalar(graficos.capas.robot) || ...
            ~isgraphics(graficos.capas.robot,'hggroup')
        error('dibujar_robot:CapaRobotNoValida', ...
            'Falta una capa grafica valida en graficos.capas.robot.');
    end

    camposEstilo = { ...
        'anguloCirculo', ...
        'colorRobot', ...
        'alphaRobot', ...
        'colorOrientacionRobot', ...
        'anchoOrientacion'};

    for i = 1:numel(camposEstilo)
        if ~isfield(graficos.estilo,camposEstilo{i})
            error('dibujar_robot:EstiloIncompleto', ...
                'Falta graficos.estilo.%s.',camposEstilo{i});
        end
    end

    validar_estilo(graficos.estilo);
end

%% Estado del robot
if ~isnumeric(estadoRobot) || ~isreal(estadoRobot) || ...
        numel(estadoRobot) ~= 3 || ...
        any(~isfinite(estadoRobot(:)))
    error('dibujar_robot:EstadoNoValido', ...
        'estadoRobot debe ser un vector real y finito [x y theta].');
end

estadoRobot = reshape(double(estadoRobot),1,3);
estadoRobot(3) = atan2( ...
    sin(estadoRobot(3)),cos(estadoRobot(3)));

%% Configuracion del robot
if ~isstruct(robot) || ~isscalar(robot) || ...
        ~isfield(robot,'tipo') || ...
        ~isfield(robot,'geometria') || ...
        ~isstruct(robot.geometria) || ...
        ~isfield(robot.geometria,'forma') || ...
        ~isfield(robot.geometria,'radio')
    error('dibujar_robot:RobotNoValido', ...
        ['robot debe ser la estructura obtenida mediante ' ...
         'configuracion_robot.m.']);
end

if string(robot.tipo) ~= "uniciclo"
    error('dibujar_robot:ModeloNoSoportado', ...
        'La visualizacion actual admite el modelo de uniciclo.');
end

if string(robot.geometria.forma) ~= "circulo"
    error('dibujar_robot:GeometriaNoSoportada', ...
        'La visualizacion actual requiere una geometria circular.');
end

radio = robot.geometria.radio;

if ~isnumeric(radio) || ~isscalar(radio) || ...
        ~isreal(radio) || ~isfinite(radio) || radio <= 0
    error('dibujar_robot:RadioNoValido', ...
        'robot.geometria.radio debe ser un escalar positivo.');
end

robot.geometria.radio = double(radio);
end

function validar_estilo(estilo)
%VALIDAR_ESTILO Comprueba los campos utilizados por el dibujo del robot.

angulos = estilo.anguloCirculo;

if ~isnumeric(angulos) || ~isreal(angulos) || ...
        numel(angulos) < 3 || any(~isfinite(angulos(:)))
    error('dibujar_robot:AngulosCirculoNoValidos', ...
        'graficos.estilo.anguloCirculo debe ser un vector numerico valido.');
end

validar_color(estilo.colorRobot,'colorRobot');
validar_color( ...
    estilo.colorOrientacionRobot,'colorOrientacionRobot');

if ~isnumeric(estilo.alphaRobot) || ...
        ~isscalar(estilo.alphaRobot) || ...
        ~isreal(estilo.alphaRobot) || ...
        ~isfinite(estilo.alphaRobot) || ...
        estilo.alphaRobot < 0 || estilo.alphaRobot > 1
    error('dibujar_robot:TransparenciaNoValida', ...
        'graficos.estilo.alphaRobot debe pertenecer al intervalo [0,1].');
end

if ~isnumeric(estilo.anchoOrientacion) || ...
        ~isscalar(estilo.anchoOrientacion) || ...
        ~isreal(estilo.anchoOrientacion) || ...
        ~isfinite(estilo.anchoOrientacion) || ...
        estilo.anchoOrientacion <= 0
    error('dibujar_robot:AnchoOrientacionNoValido', ...
        'graficos.estilo.anchoOrientacion debe ser positivo.');
end
end

function validar_color(color,nombre)
%VALIDAR_COLOR Comprueba un triplete RGB en el intervalo [0,1].

if ~isnumeric(color) || ~isreal(color) || ...
        numel(color) ~= 3 || any(~isfinite(color(:))) || ...
        any(color(:) < 0) || any(color(:) > 1)
    error('dibujar_robot:ColorNoValido', ...
        'graficos.estilo.%s debe ser un triplete RGB valido.',nombre);
end
end
