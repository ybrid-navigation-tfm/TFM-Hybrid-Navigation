function [obstaculos, info] = actualizar_obstaculos_aleatorios( ...
    obstaculos, obstaculosEstaticos, limites, Ts, ...
    flujoAleatorio, cfg)
%ACTUALIZAR_OBSTACULOS_ALEATORIOS Movimiento con rebotes aleatorios.
%
%   obstaculos = ACTUALIZAR_OBSTACULOS_ALEATORIOS( ...
%       obstaculos,obstaculosEstaticos,limites,Ts, ...
%       flujoAleatorio,cfg)
%
%   [obstaculos,info] = ACTUALIZAR_OBSTACULOS_ALEATORIOS(...)
%
%   Es una alternativa PARALELA a actualizar_obstaculos.m. La funcion
%   determinista original no se modifica y puede seguir utilizandose.
%
%   Entre colisiones, el movimiento sigue siendo:
%
%       pos(k+1) = pos(k) + Ts*vel(k)
%
%   La diferencia aparece cuando un obstaculo choca contra:
%
%       - los limites;
%       - un obstaculo estatico;
%       - otro obstaculo dinamico.
%
%   Primero se calcula una direccion de rebote fisicamente saliente y
%   despues se introduce una perturbacion angular aleatoria alrededor de
%   esa direccion. La rapidez del obstaculo se conserva.
%
%   Por defecto:
%
%       desviacionMaximaRebote = pi/3
%
%   es decir, hasta +/-60 grados respecto al rebote determinista. Puede
%   modificarse mediante:
%
%       cfg.aleatoriedadObstaculos.desviacionMaximaRebote
%
%   IMPORTANTE:
%   flujoAleatorio debe ser el MISMO objeto RandStream utilizado en
%   inicializar_obstaculos_aleatorios.m durante una ejecucion. Debe ser
%   independiente del generador global utilizado por RRT* o PRM.
%
%   De esta forma, utilizando la misma semilla de obstaculos, las tres
%   arquitecturas reciben exactamente la misma realizacion dinamica.

%% Validacion
[obstaculos,obstaculosEstaticos,limites,Ts, ...
    flujoAleatorio,parametros] = validar_entradas( ...
        obstaculos,obstaculosEstaticos,limites,Ts, ...
        flujoAleatorio,cfg);

if isempty(obstaculos)
    info = crear_info(0);
    return;
end

numeroObstaculos = numel(obstaculos);
posicionesAnteriores = zeros(numeroObstaculos,2);

for i = 1:numeroObstaculos
    posicionesAnteriores(i,:) = obstaculos(i).pos;
end

info = crear_info(numeroObstaculos);

%% Movimiento, limites y estaticos
for i = 1:numeroObstaculos
    posicionAnterior = posicionesAnteriores(i,:);
    velocidad = obstaculos(i).vel;
    radio = obstaculos(i).radio;

    posicionPropuesta = posicionAnterior + Ts*velocidad;

    % Limites. Puede haber contacto con dos lados en una esquina.
    [posicionPropuesta,velocidad,nCambiosLimites] = ...
        resolver_limites_aleatorios( ...
            posicionAnterior,posicionPropuesta,velocidad, ...
            radio,limites,Ts,flujoAleatorio,parametros);

    info.numeroRebotesLimites = ...
        info.numeroRebotesLimites+nCambiosLimites;

    % Obstaculos estaticos.
    for j = 1:size(obstaculosEstaticos,1)
        [hayColision,normal] = circulo_rectangulo( ...
            posicionPropuesta,radio,obstaculosEstaticos(j,:));

        if hayColision && dot(velocidad,normal) < 0
            velocidadReflejada = reflejar_velocidad(velocidad,normal);

            velocidad = perturbar_rebote( ...
                velocidadReflejada,normal,flujoAleatorio, ...
                parametros.desviacionMaximaRebote);

            posicionPropuesta = posicionAnterior + Ts*velocidad;

            % Fallback determinista si la perturbacion mantiene el contacto.
            [sigueColisionando,~] = circulo_rectangulo( ...
                posicionPropuesta,radio,obstaculosEstaticos(j,:));

            if sigueColisionando
                velocidad = velocidadReflejada;
                posicionPropuesta = posicionAnterior + Ts*velocidad;
            end

            info.numeroRebotesEstaticos = ...
                info.numeroRebotesEstaticos+1;

            info.reboteAleatorioPorObstaculo(i) = ...
                info.reboteAleatorioPorObstaculo(i)+1;
        end
    end

    % Proteccion numerica dentro del mapa.
    posicionPropuesta(1) = min(max( ...
        posicionPropuesta(1),limites(1)+radio),limites(2)-radio);

    posicionPropuesta(2) = min(max( ...
        posicionPropuesta(2),limites(3)+radio),limites(4)-radio);

    obstaculos(i).pos = posicionPropuesta;
    obstaculos(i).vel = velocidad;
end

%% Contactos entre obstaculos dinamicos
for i = 1:numeroObstaculos-1
    for j = i+1:numeroObstaculos
        [hayColision,normal,distancia] = circulo_circulo( ...
            obstaculos(i).pos,obstaculos(i).radio, ...
            obstaculos(j).pos,obstaculos(j).radio);

        if ~hayColision
            continue;
        end

        velocidadI = obstaculos(i).vel;
        velocidadJ = obstaculos(j).vel;
        velocidadRelativa = velocidadI-velocidadJ;

        % Con la convencion de circulo_circulo, valor negativo implica
        % aproximacion mutua.
        if dot(velocidadRelativa,normal) >= 0
            corregir = true;
        else
            corregir = false;

            % Resultado elastico determinista de masas iguales.
            componenteNormalI = dot(velocidadI,normal)*normal;
            componenteNormalJ = dot(velocidadJ,normal)*normal;

            velocidadIDeterminista = ...
                velocidadI-componenteNormalI+componenteNormalJ;

            velocidadJDeterminista = ...
                velocidadJ-componenteNormalJ+componenteNormalI;

            [velocidadINueva,velocidadJNueva,aleatorioValido] = ...
                perturbar_colision_dinamica( ...
                    velocidadIDeterminista,velocidadJDeterminista, ...
                    normal,flujoAleatorio, ...
                    parametros.desviacionMaximaRebote, ...
                    parametros.maxIntentosDireccion);

            if aleatorioValido
                obstaculos(i).vel = velocidadINueva;
                obstaculos(j).vel = velocidadJNueva;
            else
                obstaculos(i).vel = velocidadIDeterminista;
                obstaculos(j).vel = velocidadJDeterminista;
            end

            info.numeroRebotesDinamicos = ...
                info.numeroRebotesDinamicos+1;

            info.reboteAleatorioPorObstaculo(i) = ...
                info.reboteAleatorioPorObstaculo(i)+1;

            info.reboteAleatorioPorObstaculo(j) = ...
                info.reboteAleatorioPorObstaculo(j)+1;
        end

        % Correccion geometrica del posible solapamiento.
        distanciaMinima = ...
            obstaculos(i).radio+obstaculos(j).radio;

        solapamiento = distanciaMinima-distancia;

        if solapamiento > 0
            correccionPosicion = 0.5*solapamiento*normal;

            obstaculos(i).pos = ...
                obstaculos(i).pos+correccionPosicion;

            obstaculos(j).pos = ...
                obstaculos(j).pos-correccionPosicion;
        elseif corregir
            % No hace falta modificar velocidades si ya se separan.
        end
    end
end

%% Resumen
info.numeroRebotesTotales = ...
    info.numeroRebotesLimites + ...
    info.numeroRebotesEstaticos + ...
    info.numeroRebotesDinamicos;

info.desviacionMaximaRebote = ...
    parametros.desviacionMaximaRebote;
end

%% ========================================================================
% REBOTES CONTRA LIMITES
% ========================================================================

function [posicion,velocidad,numeroCambios] = ...
    resolver_limites_aleatorios( ...
        posicionAnterior,posicion,velocidad,radio,limites,Ts, ...
        flujo,parametros)

numeroCambios = 0;

% Lado izquierdo.
if posicion(1)-radio <= limites(1) && velocidad(1) < 0
    normal = [1 0];
    velocidad = rebote_aleatorio( ...
        velocidad,normal,flujo,parametros);
    posicion = posicionAnterior+Ts*velocidad;
    numeroCambios = numeroCambios+1;
end

% Lado derecho.
if posicion(1)+radio >= limites(2) && velocidad(1) > 0
    normal = [-1 0];
    velocidad = rebote_aleatorio( ...
        velocidad,normal,flujo,parametros);
    posicion = posicionAnterior+Ts*velocidad;
    numeroCambios = numeroCambios+1;
end

% Lado inferior.
if posicion(2)-radio <= limites(3) && velocidad(2) < 0
    normal = [0 1];
    velocidad = rebote_aleatorio( ...
        velocidad,normal,flujo,parametros);
    posicion = posicionAnterior+Ts*velocidad;
    numeroCambios = numeroCambios+1;
end

% Lado superior.
if posicion(2)+radio >= limites(4) && velocidad(2) > 0
    normal = [0 -1];
    velocidad = rebote_aleatorio( ...
        velocidad,normal,flujo,parametros);
    posicion = posicionAnterior+Ts*velocidad;
    numeroCambios = numeroCambios+1;
end

posicion(1) = min(max(posicion(1),limites(1)+radio),limites(2)-radio);
posicion(2) = min(max(posicion(2),limites(3)+radio),limites(4)-radio);
end

function velocidadNueva = rebote_aleatorio( ...
    velocidad,normal,flujo,parametros)

velocidadReflejada = reflejar_velocidad(velocidad,normal);

velocidadNueva = perturbar_rebote( ...
    velocidadReflejada,normal,flujo, ...
    parametros.desviacionMaximaRebote);
end

function velocidadReflejada = reflejar_velocidad(velocidad,normal)
velocidadReflejada = ...
    velocidad-2*dot(velocidad,normal)*normal;
end

%% ========================================================================
% PERTURBACION ANGULAR
% ========================================================================

function velocidadNueva = perturbar_rebote( ...
    velocidadBase,normal,flujo,desviacionMaxima)

rapidez = norm(velocidadBase);

if rapidez <= eps
    velocidadNueva = velocidadBase;
    return;
end

anguloBase = atan2(velocidadBase(2),velocidadBase(1));

for intento = 1:30
    delta = (2*rand(flujo)-1)*desviacionMaxima;
    angulo = anguloBase+delta;

    candidata = rapidez*[cos(angulo) sin(angulo)];

    % La nueva velocidad debe salir de la superficie.
    if dot(candidata,normal) > 1e-12*max(1,rapidez)
        velocidadNueva = candidata;
        return;
    end
end

% Fallback seguro: rebote determinista.
velocidadNueva = velocidadBase;
end

function [vi,vj,valida] = perturbar_colision_dinamica( ...
    viBase,vjBase,normal,flujo,desviacionMaxima,maxIntentos)

rapidezI = norm(viBase);
rapidezJ = norm(vjBase);

vi = viBase;
vj = vjBase;
valida = false;

for intento = 1:maxIntentos
    viCandidata = rotar_aleatoriamente( ...
        viBase,rapidezI,flujo,desviacionMaxima);

    vjCandidata = rotar_aleatoriamente( ...
        vjBase,rapidezJ,flujo,desviacionMaxima);

    % Tras el choque, la velocidad relativa debe ser separadora.
    if dot(viCandidata-vjCandidata,normal) > ...
            1e-12*max(1,rapidezI+rapidezJ)

        vi = viCandidata;
        vj = vjCandidata;
        valida = true;
        return;
    end
end
end

function v = rotar_aleatoriamente( ...
    vBase,rapidez,flujo,desviacionMaxima)

if rapidez <= eps
    v = vBase;
    return;
end

anguloBase = atan2(vBase(2),vBase(1));
delta = (2*rand(flujo)-1)*desviacionMaxima;
angulo = anguloBase+delta;

v = rapidez*[cos(angulo) sin(angulo)];
end

%% ========================================================================
% INFORMACION
% ========================================================================

function info = crear_info(numeroObstaculos)
info = struct();
info.numeroRebotesLimites = 0;
info.numeroRebotesEstaticos = 0;
info.numeroRebotesDinamicos = 0;
info.numeroRebotesTotales = 0;
info.reboteAleatorioPorObstaculo = zeros(numeroObstaculos,1);
info.desviacionMaximaRebote = NaN;
end

%% ========================================================================
% VALIDACION
% ========================================================================

function [obstaculos,estaticos,limites,Ts,flujo,parametros] = ...
    validar_entradas(obstaculos,estaticos,limites,Ts,flujo,cfg)

if ~isstruct(obstaculos)
    error('actualizar_obstaculos_aleatorios:ObstaculosNoValidos', ...
        'obstaculos debe ser un vector de estructuras.');
end

for i = 1:numel(obstaculos)
    if ~all(isfield(obstaculos(i),{'pos','vel','radio'}))
        error('actualizar_obstaculos_aleatorios:CampoAusente', ...
            'Faltan campos en el obstaculo dinamico %d.',i);
    end

    obstaculos(i).pos = reshape(double(obstaculos(i).pos),1,2);
    obstaculos(i).vel = reshape(double(obstaculos(i).vel),1,2);
    obstaculos(i).radio = double(obstaculos(i).radio);

    if any(~isfinite([ ...
            obstaculos(i).pos,obstaculos(i).vel,obstaculos(i).radio])) || ...
            obstaculos(i).radio <= 0
        error('actualizar_obstaculos_aleatorios:DinamicoNoValido', ...
            'El obstaculo dinamico %d no es valido.',i);
    end
end

if isempty(estaticos)
    estaticos = zeros(0,4);
elseif ~isnumeric(estaticos) || size(estaticos,2) ~= 4 || ...
        any(~isfinite(estaticos(:))) || ...
        any(estaticos(:,3:4) <= 0,'all')
    error('actualizar_obstaculos_aleatorios:EstaticosNoValidos', ...
        'Los obstaculos estaticos deben tener formato N x 4.');
else
    estaticos = double(estaticos);
end

if ~isnumeric(limites) || numel(limites) ~= 4 || ...
        any(~isfinite(limites(:)))
    error('actualizar_obstaculos_aleatorios:LimitesNoValidos', ...
        'limites debe ser [xmin xmax ymin ymax].');
end

limites = reshape(double(limites),1,4);

if limites(1) >= limites(2) || limites(3) >= limites(4)
    error('actualizar_obstaculos_aleatorios:OrdenLimitesNoValido', ...
        'Debe cumplirse xmin < xmax e ymin < ymax.');
end

if ~isnumeric(Ts) || ~isscalar(Ts) || ...
        ~isreal(Ts) || ~isfinite(Ts) || Ts <= 0
    error('actualizar_obstaculos_aleatorios:TsNoValido', ...
        'Ts debe ser un escalar positivo.');
end

Ts = double(Ts);

if ~isa(flujo,'RandStream')
    error('actualizar_obstaculos_aleatorios:FlujoNoValido', ...
        'flujoAleatorio debe ser un objeto RandStream.');
end

parametros = struct();
parametros.desviacionMaximaRebote = pi/3;
parametros.maxIntentosDireccion = 40;

if isstruct(cfg) && isfield(cfg,'aleatoriedadObstaculos') && ...
        isstruct(cfg.aleatoriedadObstaculos)

    alea = cfg.aleatoriedadObstaculos;

    if isfield(alea,'desviacionMaximaRebote')
        parametros.desviacionMaximaRebote = ...
            double(alea.desviacionMaximaRebote);
    end

    if isfield(alea,'maxIntentosDireccion')
        parametros.maxIntentosDireccion = ...
            double(alea.maxIntentosDireccion);
    end
end

if ~isnumeric(parametros.desviacionMaximaRebote) || ...
        ~isscalar(parametros.desviacionMaximaRebote) || ...
        ~isfinite(parametros.desviacionMaximaRebote) || ...
        parametros.desviacionMaximaRebote < 0 || ...
        parametros.desviacionMaximaRebote > pi/2
    error('actualizar_obstaculos_aleatorios:DesviacionNoValida', ...
        ['desviacionMaximaRebote debe pertenecer al intervalo ' ...
         '[0,pi/2].']);
end

if ~isnumeric(parametros.maxIntentosDireccion) || ...
        ~isscalar(parametros.maxIntentosDireccion) || ...
        ~isfinite(parametros.maxIntentosDireccion) || ...
        parametros.maxIntentosDireccion < 1 || ...
        parametros.maxIntentosDireccion ~= ...
            floor(parametros.maxIntentosDireccion)
    error('actualizar_obstaculos_aleatorios:IntentosNoValidos', ...
        'maxIntentosDireccion debe ser un entero positivo.');
end
end
