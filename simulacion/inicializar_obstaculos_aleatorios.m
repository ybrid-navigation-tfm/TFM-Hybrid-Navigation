function [obstaculos, info] = inicializar_obstaculos_aleatorios( ...
    escenario, robot, cfg, flujoAleatorio)
%INICIALIZAR_OBSTACULOS_ALEATORIOS Aleatoriza posiciones iniciales.
%
%   obstaculos = INICIALIZAR_OBSTACULOS_ALEATORIOS( ...
%       escenario,robot,cfg,flujoAleatorio)
%
%   [obstaculos,info] = INICIALIZAR_OBSTACULOS_ALEATORIOS(...)
%
%   Genera una realizacion aleatoria de las POSICIONES INICIALES de los
%   obstaculos dinamicos conservando:
%
%       - el numero de obstaculos del escenario;
%       - sus identificadores;
%       - sus radios;
%       - sus velocidades iniciales y, por tanto, la rapidez nominal.
%
%   La funcion NO sustituye escenarios.m. Parte de:
%
%       escenario.obstaculosDinamicos
%
%   y devuelve un vector alternativo que puede utilizarse solamente cuando
%   se desea ejecutar el modo aleatorio.
%
%   Por defecto, cada obstaculo se desplaza aleatoriamente alrededor de su
%   posicion de referencia dentro de un radio de 2 m. Esto introduce
%   variabilidad sin convertir cada repeticion en un escenario totalmente
%   diferente. El radio puede modificarse mediante:
%
%       cfg.aleatoriedadObstaculos.radioPerturbacionPosicion
%
%   Tambien puede utilizarse muestreo uniforme en todo el mapa mediante:
%
%       cfg.aleatoriedadObstaculos.modoPosicion = "uniforme_mapa";
%
%   Restricciones de las posiciones generadas:
%
%       - permanecen dentro de los limites;
%       - no solapan obstaculos estaticos;
%       - no solapan otros obstaculos dinamicos;
%       - no bloquean inicialmente al robot;
%       - no ocupan inicialmente la region de meta.
%
%   IMPORTANTE PARA LA REPRODUCIBILIDAD:
%   flujoAleatorio debe ser un RandStream independiente del generador que
%   utilizan RRT* o PRM. De este modo las trayectorias de los obstaculos no
%   dependen del numero de llamadas a rand realizadas por el planificador.
%
%   Ejemplo:
%
%       flujoObstaculos = RandStream( ...
%           'mt19937ar','Seed',100000+cfg.semilla);
%
%       [obstaculosDinamicos,infoIni] = ...
%           inicializar_obstaculos_aleatorios( ...
%               escenario,robot,cfg,flujoObstaculos);
%
%   Esta funcion no modifica el generador aleatorio global de MATLAB.

%% Validacion
[escenario,robot,cfg,flujoAleatorio,parametros] = ...
    validar_entradas(escenario,robot,cfg,flujoAleatorio);

obstaculos = escenario.obstaculosDinamicos;
numeroDinamicos = numel(obstaculos);

posicionesReferencia = zeros(numeroDinamicos,2);
posicionesGeneradas = zeros(numeroDinamicos,2);
intentosPorObstaculo = zeros(numeroDinamicos,1);

for i = 1:numeroDinamicos
    posicionesReferencia(i,:) = obstaculos(i).pos;
end

%% Generacion secuencial de posiciones validas
for i = 1:numeroDinamicos
    radio = obstaculos(i).radio;
    posicionReferencia = posicionesReferencia(i,:);

    encontrada = false;

    for intento = 1:parametros.maxIntentosPorObstaculo
        candidato = muestrear_posicion( ...
            posicionReferencia,radio,escenario.limites, ...
            flujoAleatorio,parametros);

        if posicion_valida( ...
                candidato,radio,i,obstaculos, ...
                posicionesGeneradas,escenario,robot,cfg,parametros)

            obstaculos(i).pos = candidato;
            posicionesGeneradas(i,:) = candidato;
            intentosPorObstaculo(i) = intento;
            encontrada = true;
            break;
        end
    end

    if ~encontrada
        error('inicializar_obstaculos_aleatorios:SinPosicionValida', ...
            ['No se pudo generar una posicion valida para D%d tras %d ' ...
             'intentos. Reduzca el radio de perturbacion o los margenes.'], ...
            i,parametros.maxIntentosPorObstaculo);
    end
end

%% Informacion
info = struct();
info.exito = true;
info.modoPosicion = parametros.modoPosicion;
info.radioPerturbacionPosicion = parametros.radioPerturbacionPosicion;
info.numeroObstaculos = numeroDinamicos;
info.posicionesReferencia = posicionesReferencia;
info.posicionesGeneradas = posicionesGeneradas;
info.desplazamientosIniciales = posicionesGeneradas-posicionesReferencia;
info.intentosPorObstaculo = intentosPorObstaculo;
info.maxIntentosPorObstaculo = parametros.maxIntentosPorObstaculo;
info.margenInicialEstatico = parametros.margenInicialEstatico;
info.margenInicialDinamico = parametros.margenInicialDinamico;
info.margenInicialRobot = parametros.margenInicialRobot;
info.margenInicialMeta = parametros.margenInicialMeta;
info.velocidadesInicialesConservadas = true;
end

%% ========================================================================
% MUESTREO
% ========================================================================

function candidato = muestrear_posicion( ...
    referencia,radio,limites,flujo,parametros)

switch parametros.modoPosicion
    case "alrededor_referencia"
        angulo = 2*pi*rand(flujo);
        distancia = parametros.radioPerturbacionPosicion*sqrt(rand(flujo));
        candidato = referencia + ...
            distancia*[cos(angulo) sin(angulo)];

    case "uniforme_mapa"
        candidato = [ ...
            limites(1)+radio + ...
                rand(flujo)*(limites(2)-limites(1)-2*radio), ...
            limites(3)+radio + ...
                rand(flujo)*(limites(4)-limites(3)-2*radio)];

    otherwise
        error('inicializar_obstaculos_aleatorios:ModoNoValido', ...
            'Modo de posicion no reconocido: %s.',parametros.modoPosicion);
end
end

function valida = posicion_valida( ...
    p,radio,indice,obstaculos,posicionesGeneradas, ...
    escenario,robot,cfg,parametros)

valida = false;
limites = escenario.limites;

%% Limites
if p(1)-radio < limites(1) || p(1)+radio > limites(2) || ...
        p(2)-radio < limites(3) || p(2)+radio > limites(4)
    return;
end

%% Obstaculos estaticos
for j = 1:size(escenario.obstaculosEstaticos,1)
    distanciaCentro = distancia_punto_rectangulo_local( ...
        p,escenario.obstaculosEstaticos(j,:));

    if distanciaCentro <= ...
            radio+parametros.margenInicialEstatico
        return;
    end
end

%% Robot en la posicion inicial
distanciaInicio = norm(p-escenario.inicio(1:2));

if distanciaInicio <= ...
        radio+robot.geometria.radio+parametros.margenInicialRobot
    return;
end

%% Region de meta
distanciaMeta = norm(p-escenario.meta);

if distanciaMeta <= ...
        radio+cfg.navegacion.radioMeta+parametros.margenInicialMeta
    return;
end

%% Otros obstaculos dinamicos ya colocados
for j = 1:indice-1
    radioJ = obstaculos(j).radio;

    if norm(p-posicionesGeneradas(j,:)) <= ...
            radio+radioJ+parametros.margenInicialDinamico
        return;
    end
end

valida = true;
end

function d = distancia_punto_rectangulo_local(p,r)
xmin = r(1);
ymin = r(2);
xmax = xmin+r(3);
ymax = ymin+r(4);

dx = max([xmin-p(1),0,p(1)-xmax]);
dy = max([ymin-p(2),0,p(2)-ymax]);

d = hypot(dx,dy);
end

%% ========================================================================
% VALIDACION Y PARAMETROS
% ========================================================================

function [escenario,robot,cfg,flujo,parametros] = ...
    validar_entradas(escenario,robot,cfg,flujo)

if ~isstruct(escenario) || ~isscalar(escenario) || ...
        ~all(isfield(escenario,{ ...
            'limites','inicio','meta', ...
            'obstaculosEstaticos','obstaculosDinamicos'}))
    error('inicializar_obstaculos_aleatorios:EscenarioNoValido', ...
        'escenario debe proceder de escenarios.m.');
end

if ~isstruct(robot) || ~isscalar(robot) || ...
        ~isfield(robot,'geometria') || ...
        ~isfield(robot.geometria,'radio')
    error('inicializar_obstaculos_aleatorios:RobotNoValido', ...
        'robot debe proceder de configuracion_robot.m.');
end

if ~isstruct(cfg) || ~isscalar(cfg) || ...
        ~isfield(cfg,'navegacion') || ...
        ~isfield(cfg.navegacion,'radioMeta') || ...
        ~isfield(cfg,'seguridad')
    error('inicializar_obstaculos_aleatorios:ConfiguracionNoValida', ...
        'cfg debe proceder de parametros_generales.m.');
end

if ~isa(flujo,'RandStream')
    error('inicializar_obstaculos_aleatorios:FlujoNoValido', ...
        ['flujoAleatorio debe ser un objeto RandStream independiente ' ...
         'del generador utilizado por los planificadores.']);
end

parametros = struct();

parametros.modoPosicion = "alrededor_referencia";
parametros.radioPerturbacionPosicion = 2.0;
parametros.maxIntentosPorObstaculo = 5000;

parametros.margenInicialEstatico = cfg.seguridad.margenEstatico;
parametros.margenInicialDinamico = cfg.seguridad.margenDinamico;
parametros.margenInicialRobot = cfg.seguridad.margenDinamico;
parametros.margenInicialMeta = cfg.seguridad.margenDinamico;

if isfield(cfg,'aleatoriedadObstaculos') && ...
        isstruct(cfg.aleatoriedadObstaculos)

    alea = cfg.aleatoriedadObstaculos;

    if isfield(alea,'modoPosicion')
        parametros.modoPosicion = ...
            lower(strtrim(string(alea.modoPosicion)));
    end

    if isfield(alea,'radioPerturbacionPosicion')
        parametros.radioPerturbacionPosicion = ...
            double(alea.radioPerturbacionPosicion);
    end

    if isfield(alea,'maxIntentosPorObstaculo')
        parametros.maxIntentosPorObstaculo = ...
            double(alea.maxIntentosPorObstaculo);
    end

    if isfield(alea,'margenInicialEstatico')
        parametros.margenInicialEstatico = ...
            double(alea.margenInicialEstatico);
    end

    if isfield(alea,'margenInicialDinamico')
        parametros.margenInicialDinamico = ...
            double(alea.margenInicialDinamico);
    end

    if isfield(alea,'margenInicialRobot')
        parametros.margenInicialRobot = ...
            double(alea.margenInicialRobot);
    end

    if isfield(alea,'margenInicialMeta')
        parametros.margenInicialMeta = ...
            double(alea.margenInicialMeta);
    end
end

if ~any(parametros.modoPosicion == ...
        ["alrededor_referencia","uniforme_mapa"])
    error('inicializar_obstaculos_aleatorios:ModoPosicionNoValido', ...
        ['cfg.aleatoriedadObstaculos.modoPosicion debe ser ' ...
         '"alrededor_referencia" o "uniforme_mapa".']);
end

if ~es_positivo(parametros.radioPerturbacionPosicion)
    error('inicializar_obstaculos_aleatorios:RadioPerturbacionNoValido', ...
        'El radio de perturbacion debe ser positivo.');
end

if ~es_entero_positivo(parametros.maxIntentosPorObstaculo)
    error('inicializar_obstaculos_aleatorios:IntentosNoValidos', ...
        'maxIntentosPorObstaculo debe ser un entero positivo.');
end

margenes = [ ...
    parametros.margenInicialEstatico, ...
    parametros.margenInicialDinamico, ...
    parametros.margenInicialRobot, ...
    parametros.margenInicialMeta];

if any(~isfinite(margenes)) || any(margenes < 0)
    error('inicializar_obstaculos_aleatorios:MargenNoValido', ...
        'Los margenes iniciales deben ser finitos y no negativos.');
end
end

function tf = es_positivo(x)
tf = isnumeric(x) && isscalar(x) && isreal(x) && ...
    isfinite(x) && x > 0;
end

function tf = es_entero_positivo(x)
tf = es_positivo(x) && x == floor(x);
end
