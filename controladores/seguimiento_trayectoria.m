function [objetivoLocal, caminoActualizado, info] = ...
    seguimiento_trayectoria( ...
        estadoRobot, caminoGlobal, meta, cfg, caminoParcialPermitido)
%SEGUIMIENTO_TRAYECTORIA Selecciona un objetivo local con progreso monotono.
%
%   [objetivoLocal,caminoActualizado,info] = ...
%       seguimiento_trayectoria(estadoRobot,caminoGlobal,meta,cfg)
%
%   La funcion mantiene una politica sencilla:
%       1. conserva solamente la parte restante del camino;
%       2. no vuelve a buscar segmentos antiguos de una polilinea;
%       3. actualiza el primer punto con la posicion real del robot;
%       4. selecciona un objetivo adelantado mediante look-ahead.
%
%   La quinta entrada se conserva por compatibilidad. En la arquitectura
%   principal RRT*+APF se proporciona false porque RRT* entrega caminos
%   completos hasta la region de meta.

if nargin < 5 || isempty(caminoParcialPermitido)
    caminoParcialPermitido = false;
end

[posicionRobot,camino,meta,parametros,caminoParcialPermitido] = ...
    validar_entradas( ...
        estadoRobot,caminoGlobal,meta,cfg,caminoParcialPermitido);

escala = max(1,max(abs([posicionRobot(:);meta(:);camino(:)])));
tol = 1e-10*escala;

objetivoLocal = posicionRobot;
caminoActualizado = zeros(0,2);

info = crear_info_base( ...
    posicionRobot,meta,parametros,caminoParcialPermitido);

info.distanciaMeta = norm(posicionRobot-meta);
info.metaAlcanzada = ...
    info.distanciaMeta <= parametros.radioMeta+tol;

if info.metaAlcanzada
    objetivoLocal = meta;
    caminoActualizado = quitar_repetidos( ...
        [posicionRobot;meta],tol);

    info.exito = true;
    info.caminoUtilizable = true;
    info.motivo = "meta_alcanzada";
    info.objetivoLocal = objetivoLocal;
    info.objetivoEsMeta = true;
    info.objetivoEsFinalCamino = true;
    info.caminoActualizado = caminoActualizado;
    info.numeroWaypointsSalida = size(caminoActualizado,1);
    info.distanciaRobotObjetivo = info.distanciaMeta;
    info.distanciaAlCamino = 0;
    return;
end

if isempty(camino)
    info.motivo = "camino_vacio";
    return;
end

camino = quitar_repetidos(camino(:,1:2),tol);

if size(camino,1) < 2
    info.motivo = "camino_sin_segmentos";
    return;
end

info.numeroWaypointsEntrada = size(camino,1);
info.distanciaAlCamino = ...
    distancia_a_polilinea(posicionRobot,camino,tol);

distanciaUltimoMeta = norm(camino(end,:)-meta);
caminoTerminaEnMeta = ...
    distanciaUltimoMeta <= parametros.radioMeta+tol;

info.caminoTerminaEnMeta = caminoTerminaEnMeta;
info.distanciaUltimoWaypointMeta = distanciaUltimoMeta;
info.caminoParcial = ~caminoTerminaEnMeta;

if ~caminoTerminaEnMeta && ~caminoParcialPermitido
    info.motivo = "camino_no_termina_en_meta";
    return;
end

% Elimina solamente tramos iniciales ya superados. No se proyecta sobre
% toda la ruta, por lo que no puede saltar a una rama antigua o cruzada.
numeroDescartados = 0;

while size(camino,1) > 2
    a = camino(1,:);
    b = camino(2,:);
    ab = b-a;
    den = dot(ab,ab);

    alcanzado = ...
        norm(posicionRobot-b) <= parametros.tolWaypoint+tol;

    if den <= tol^2
        sobrepasado = true;
    else
        t = dot(posicionRobot-a,ab)/den;
        sobrepasado = t >= 1;
    end

    if ~(alcanzado || sobrepasado)
        break;
    end

    camino(1,:) = [];
    numeroDescartados = numeroDescartados+1;
end

camino(1,:) = posicionRobot;
camino = quitar_repetidos(camino,tol);

if size(camino,1) < 2
    info.motivo = "camino_agotado";
    return;
end

if ~caminoTerminaEnMeta && ...
        norm(posicionRobot-camino(end,:)) <= ...
            parametros.tolWaypoint+tol

    objetivoLocal = camino(end,:);
    caminoActualizado = zeros(0,2);

    info.exito = true;
    info.caminoUtilizable = false;
    info.motivo = "fin_camino_parcial_alcanzado";
    info.finCaminoParcialAlcanzado = true;
    info.objetivoLocal = objetivoLocal;
    info.objetivoEsFinalCamino = true;
    info.numeroWaypointsDescartadosAntesProyeccion = ...
        numeroDescartados;
    return;
end

[objetivoLocal,lookaheadAplicado,objetivoEsFinal] = ...
    objetivo_lookahead(camino,parametros.lookahead,tol);

caminoActualizado = camino;

info.exito = true;
info.caminoUtilizable = true;
info.motivo = "objetivo_local_seleccionado";
info.objetivoLocal = objetivoLocal;
info.objetivoEsFinalCamino = objetivoEsFinal;
info.objetivoEsMeta = objetivoEsFinal && caminoTerminaEnMeta;
info.objetivoInterpolado = ~objetivoEsFinal;
info.distanciaRobotObjetivo = ...
    norm(objetivoLocal-posicionRobot);
info.lookaheadAplicado = lookaheadAplicado;
info.caminoActualizado = caminoActualizado;
info.numeroWaypointsSalida = size(caminoActualizado,1);
info.numeroWaypointsDescartadosAntesProyeccion = ...
    numeroDescartados;
info.longitudRestante = longitud_polilinea(caminoActualizado);
end

%% ========================================================================
% FUNCIONES LOCALES
% ========================================================================

function [posicion,camino,meta,parametros,parcial] = ...
    validar_entradas(estadoRobot,camino,meta,cfg,parcial)

if ~isnumeric(estadoRobot) || ~isreal(estadoRobot) || ...
        numel(estadoRobot) < 2 || any(~isfinite(estadoRobot(:)))
    error('seguimiento_trayectoria:EstadoNoValido', ...
        'estadoRobot debe contener al menos [x y].');
end

estadoRobot = reshape(double(estadoRobot),1,[]);
posicion = estadoRobot(1:2);

if isempty(camino)
    camino = zeros(0,2);
elseif ~isnumeric(camino) || ~isreal(camino) || ...
        size(camino,2) < 2 || any(~isfinite(camino(:)))
    error('seguimiento_trayectoria:CaminoNoValido', ...
        'caminoGlobal debe ser una matriz numerica N x 2.');
else
    camino = double(camino(:,1:2));
end

if ~isnumeric(meta) || ~isreal(meta) || ...
        numel(meta) ~= 2 || any(~isfinite(meta(:)))
    error('seguimiento_trayectoria:MetaNoValida', ...
        'meta debe tener formato [x y].');
end

meta = reshape(double(meta),1,2);

if ~isstruct(cfg) || ~isfield(cfg,'navegacion')
    error('seguimiento_trayectoria:ConfiguracionNoValida', ...
        'cfg debe proceder de parametros_generales.m.');
end

campos = {'radioMeta','lookahead','tolWaypoint'};
for i = 1:numel(campos)
    if ~isfield(cfg.navegacion,campos{i}) || ...
            ~isnumeric(cfg.navegacion.(campos{i})) || ...
            ~isscalar(cfg.navegacion.(campos{i})) || ...
            ~isfinite(cfg.navegacion.(campos{i})) || ...
            cfg.navegacion.(campos{i}) < 0
        error('seguimiento_trayectoria:ParametroNoValido', ...
            'cfg.navegacion.%s no es valido.',campos{i});
    end
end

if cfg.navegacion.radioMeta <= 0 || ...
        cfg.navegacion.lookahead <= 0
    error('seguimiento_trayectoria:ParametroNoPositivo', ...
        'radioMeta y lookahead deben ser positivos.');
end

if ~(islogical(parcial) && isscalar(parcial))
    if isnumeric(parcial) && isscalar(parcial) && ...
            isfinite(parcial) && any(parcial == [0 1])
        parcial = logical(parcial);
    else
        error('seguimiento_trayectoria:IndicadorParcialNoValido', ...
            'caminoParcialPermitido debe ser logico.');
    end
end

parametros = struct();
parametros.radioMeta = double(cfg.navegacion.radioMeta);
parametros.lookahead = double(cfg.navegacion.lookahead);
parametros.tolWaypoint = double(cfg.navegacion.tolWaypoint);
end

function info = crear_info_base(posicion,meta,parametros,parcial)

info = struct();
info.exito = false;
info.caminoUtilizable = false;
info.motivo = "";
info.posicionRobot = posicion;
info.meta = meta;
info.distanciaMeta = norm(posicion-meta);
info.metaAlcanzada = false;
info.objetivoLocal = posicion;
info.objetivoEsMeta = false;
info.objetivoEsFinalCamino = false;
info.objetivoInterpolado = false;
info.distanciaRobotObjetivo = 0;
info.distanciaAlCamino = NaN;
info.lookaheadSolicitado = parametros.lookahead;
info.lookaheadAplicado = 0;
info.numeroWaypointsEntrada = 0;
info.numeroWaypointsSalida = 0;
info.numeroWaypointsDescartadosAntesProyeccion = 0;
info.caminoTerminaEnMeta = false;
info.distanciaUltimoWaypointMeta = NaN;
info.caminoParcialPermitido = parcial;
info.caminoParcial = false;
info.finCaminoParcialAlcanzado = false;
info.longitudRestante = NaN;
info.caminoActualizado = zeros(0,2);
end

function camino = quitar_repetidos(camino,tol)

if isempty(camino)
    camino = zeros(0,2);
    return;
end

conservar = true(size(camino,1),1);

for i = 2:size(camino,1)
    conservar(i) = norm(camino(i,:)-camino(i-1,:)) > tol;
end

camino = camino(conservar,:);
end

function [objetivo,distanciaAplicada,esFinal] = ...
    objetivo_lookahead(camino,lookahead,tol)

longitudes = hypot( ...
    diff(camino(:,1)),diff(camino(:,2)));

longitudTotal = sum(longitudes);
distanciaAplicada = min(lookahead,longitudTotal);

if longitudTotal <= tol || ...
        distanciaAplicada >= longitudTotal-tol
    objetivo = camino(end,:);
    esFinal = true;
    return;
end

acumulada = 0;

for i = 1:numel(longitudes)
    L = longitudes(i);

    if L <= tol
        continue;
    end

    if acumulada+L >= distanciaAplicada-tol
        fraccion = (distanciaAplicada-acumulada)/L;
        fraccion = min(max(fraccion,0),1);
        objetivo = camino(i,:)+ ...
            fraccion*(camino(i+1,:)-camino(i,:));
        esFinal = false;
        return;
    end

    acumulada = acumulada+L;
end

objetivo = camino(end,:);
esFinal = true;
end

function d = distancia_a_polilinea(p,camino,tol)

if size(camino,1) < 2
    d = norm(p-camino(1,:));
    return;
end

d = inf;

for i = 1:size(camino,1)-1
    a = camino(i,:);
    b = camino(i+1,:);
    ab = b-a;
    den = dot(ab,ab);

    if den <= tol^2
        q = a;
    else
        t = dot(p-a,ab)/den;
        t = min(max(t,0),1);
        q = a+t*ab;
    end

    d = min(d,norm(p-q));
end
end

function L = longitud_polilinea(camino)

if size(camino,1) < 2
    L = 0;
else
    L = sum(hypot(diff(camino(:,1)),diff(camino(:,2))));
end
end
