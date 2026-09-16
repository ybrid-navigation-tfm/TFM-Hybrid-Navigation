function [distanciaMinima, detalle] = distancia_seguridad( ...
    trayectoria, historialObstaculos, escenario, robot, cfg)
% Calcula la distancia minima de seguridad del robot.
%
%   distanciaMinima = DISTANCIA_SEGURIDAD( ...
%       trayectoria,historialObstaculos,escenario,robot,cfg)
%
%   [distanciaMinima,detalle] = DISTANCIA_SEGURIDAD( ...
%       trayectoria,historialObstaculos,escenario,robot,cfg)
%
%   Calcula, para cada muestra de la simulacion, la menor distancia libre
%   entre la superficie del robot y:
%
%       - los limites del mapa;
%       - los obstaculos estaticos rectangulares;
%       - los obstaculos dinamicos circulares.
%
%   La metrica principal es:
%
%       d_min = min_k d_k
%
%   donde:
%
%       d_k = min(d_limites(k),d_estaticos(k),d_dinamicos(k))
%
%   La distancia se mide entre superficies:
%
%       distancia > 0   existe separacion
%       distancia = 0   existe contacto
%       distancia < 0   existe solapamiento o penetracion
%
%   Por tanto, un valor MAYOR representa una navegacion MAS SEGURA.
%
%   Entradas:
%       trayectoria
%           Historial N x M, con M >= 2. Las dos primeras columnas son
%           las posiciones [x y] del robot. Se admite directamente el
%           historial N x 3 de estados [x y theta].
%
%       historialObstaculos
%           Historial numerico N x D x 2 con las posiciones de los D
%           obstaculos dinamicos:
%
%               historialObstaculos(k,i,:) = [x_i(k) y_i(k)]
%
%           La fila temporal k debe corresponder al mismo instante que
%           trayectoria(k,:). Si el escenario no contiene obstaculos
%           dinamicos, puede proporcionarse [].
%
%       escenario
%           Estructura obtenida mediante escenarios.m. Debe contener:
%
%               .limites
%               .obstaculosEstaticos
%               .obstaculosDinamicos
%
%       robot
%           Estructura obtenida mediante configuracion_robot.m.
%
%       cfg
%           Estructura obtenida mediante parametros_generales.m. Se usan:
%
%               cfg.sim.Ts
%               cfg.metricas.distanciaRiesgo
%
%   Salidas:
%       distanciaMinima
%           Menor distancia libre registrada durante la ejecucion [m].
%
%       detalle
%           Estructura con informacion adicional:
%
%               .calculable
%               .motivo
%               .distanciaMinima
%               .distanciaPorPaso
%               .distanciaLimitesPorPaso
%               .distanciaEstaticosPorPaso
%               .distanciaDinamicosPorPaso
%               .pasoCritico
%               .tiempoCritico
%               .tipoCritico
%               .idCritico
%               .indiceCritico
%               .umbralRiesgo
%               .riesgoPorPaso
%               .numeroPasosRiesgo
%               .numeroEpisodiosRiesgo
%               .colisionPorPaso
%               .numeroPasosColision
%               .numeroEpisodiosColision
%               .penetracionMaxima
%
%   La distancia de seguridad es una distancia FISICA. No se restan los
%   margenes cfg.seguridad.margenEstatico ni margenDinamico, ya que estos
%   pertenecen a la planificacion y al control. Una situacion de riesgo se
%   declara cuando:
%
%       distanciaPorPaso <= cfg.metricas.distanciaRiesgo
%
%   Ejemplo:
%
%       cfg = parametros_generales("batch");
%       escenario = escenarios("media");
%       robot = configuracion_robot();
%
%       % trayectoriaEjecutada: N x 3
%       % historialDinamicos:   N x escenario.nDinamicos x 2
%
%       [dMin,info] = distancia_seguridad( ...
%           trayectoriaEjecutada,historialDinamicos, ...
%           escenario,robot,cfg);
%
%       fprintf('Distancia minima: %.3f m\n',dMin);
%       fprintf('Situaciones de riesgo: %d\n', ...
%           info.numeroEpisodiosRiesgo);
%
%   Esta funcion utiliza:
%       - detectar_colisiones.m

%% Validacion y normalizacion
[trayectoria, historialObstaculos, escenario, robot, cfg] = ...
    validar_entradas( ...
        trayectoria,historialObstaculos,escenario,robot,cfg);

numeroPasos = size(trayectoria,1);
numeroDinamicos = numel(escenario.obstaculosDinamicos);

detalle = estructura_detalle_vacia(numeroPasos,cfg);

if numeroPasos == 0
    distanciaMinima = NaN;
    detalle.motivo = "trayectoria_vacia";
    return;
end

%% Memoria para las distancias instantaneas
distanciaPorPaso = inf(numeroPasos,1);
distanciaLimitesPorPaso = inf(numeroPasos,1);
distanciaEstaticosPorPaso = inf(numeroPasos,1);
distanciaDinamicosPorPaso = inf(numeroPasos,1);

tipoMasCercanoPorPaso = strings(numeroPasos,1);
idMasCercanoPorPaso = strings(numeroPasos,1);
indiceMasCercanoPorPaso = nan(numeroPasos,1);

colisionPorPaso = false(numeroPasos,1);

%% Evaluacion de cada instante de la ejecucion
dinamicosPaso = escenario.obstaculosDinamicos;

for k = 1:numeroPasos

    for i = 1:numeroDinamicos
        dinamicosPaso(i).pos = reshape( ...
            historialObstaculos(k,i,:),1,2);
    end

    [hayColision,infoInstantanea] = detectar_colisiones( ...
        trayectoria(k,:), ...
        robot, ...
        escenario.obstaculosEstaticos, ...
        dinamicosPaso, ...
        escenario.limites);

    distanciaPorPaso(k) = ...
        infoInstantanea.distanciaLibreMinima;

    distanciaLimitesPorPaso(k) = ...
        infoInstantanea.distanciaLibreLimites;

    if ~isempty(infoInstantanea.distanciasLibresEstaticos)
        distanciaEstaticosPorPaso(k) = min( ...
            infoInstantanea.distanciasLibresEstaticos);
    end

    if ~isempty(infoInstantanea.distanciasLibresDinamicos)
        distanciaDinamicosPorPaso(k) = min( ...
            infoInstantanea.distanciasLibresDinamicos);
    end

    tipoMasCercanoPorPaso(k) = ...
        string(infoInstantanea.tipoMasCercano);

    idMasCercanoPorPaso(k) = ...
        string(infoInstantanea.idMasCercano);

    indiceMasCercanoPorPaso(k) = ...
        infoInstantanea.indiceMasCercano;

    colisionPorPaso(k) = hayColision;
end

%% Metrica principal
[distanciaMinima,pasoCritico] = min(distanciaPorPaso);

%% Situaciones de riesgo
umbralRiesgo = cfg.metricas.distanciaRiesgo;

escala = max(1,max(abs([ ...
    distanciaPorPaso(isfinite(distanciaPorPaso)); ...
    umbralRiesgo])));

tolerancia = 1e-12*escala;

riesgoPorPaso = ...
    distanciaPorPaso <= umbralRiesgo+tolerancia;

numeroPasosRiesgo = nnz(riesgoPorPaso);
numeroEpisodiosRiesgo = contar_episodios(riesgoPorPaso);

numeroPasosColision = nnz(colisionPorPaso);
numeroEpisodiosColision = contar_episodios(colisionPorPaso);

%% Primeras apariciones
primerPasoRiesgo = primer_indice(riesgoPorPaso);
primerPasoColision = primer_indice(colisionPorPaso);

%% Resultado detallado
detalle.calculable = true;
detalle.motivo = "correcto";

detalle.distanciaMinima = distanciaMinima;
detalle.distanciaPorPaso = distanciaPorPaso;

detalle.distanciaLimitesPorPaso = ...
    distanciaLimitesPorPaso;

detalle.distanciaEstaticosPorPaso = ...
    distanciaEstaticosPorPaso;

detalle.distanciaDinamicosPorPaso = ...
    distanciaDinamicosPorPaso;

detalle.distanciaMinimaLimites = ...
    minimo_finito(distanciaLimitesPorPaso);

detalle.distanciaMinimaEstaticos = ...
    minimo_finito(distanciaEstaticosPorPaso);

detalle.distanciaMinimaDinamicos = ...
    minimo_finito(distanciaDinamicosPorPaso);

detalle.pasoCritico = pasoCritico;
detalle.tiempoCritico = (pasoCritico-1)*cfg.sim.Ts;

detalle.tipoCritico = ...
    tipoMasCercanoPorPaso(pasoCritico);

detalle.idCritico = ...
    idMasCercanoPorPaso(pasoCritico);

detalle.indiceCritico = ...
    indiceMasCercanoPorPaso(pasoCritico);

detalle.tipoMasCercanoPorPaso = ...
    tipoMasCercanoPorPaso;

detalle.idMasCercanoPorPaso = ...
    idMasCercanoPorPaso;

detalle.indiceMasCercanoPorPaso = ...
    indiceMasCercanoPorPaso;

detalle.umbralRiesgo = umbralRiesgo;
detalle.riesgoPorPaso = riesgoPorPaso;
detalle.numeroPasosRiesgo = numeroPasosRiesgo;
detalle.numeroEpisodiosRiesgo = numeroEpisodiosRiesgo;
detalle.porcentajePasosRiesgo = ...
    100*numeroPasosRiesgo/numeroPasos;

detalle.primerPasoRiesgo = primerPasoRiesgo;
detalle.primerTiempoRiesgo = ...
    indice_a_tiempo(primerPasoRiesgo,cfg.sim.Ts);

detalle.cumpleUmbralRiesgo = ...
    distanciaMinima > umbralRiesgo+tolerancia;

detalle.colisionPorPaso = colisionPorPaso;
detalle.colisionOcurrida = any(colisionPorPaso);
detalle.numeroPasosColision = numeroPasosColision;
detalle.numeroEpisodiosColision = numeroEpisodiosColision;

detalle.primerPasoColision = primerPasoColision;
detalle.primerTiempoColision = ...
    indice_a_tiempo(primerPasoColision,cfg.sim.Ts);

detalle.penetracionMaxima = max(0,-distanciaMinima);

detalle.numeroPasos = numeroPasos;
detalle.periodoMuestreo = cfg.sim.Ts;
detalle.tiempos = (0:numeroPasos-1)'*cfg.sim.Ts;

detalle.unidad = "m";
detalle.criterio = "mayor_valor_mayor_seguridad";
end

%% ========================================================================
% FUNCIONES LOCALES
% ========================================================================

function detalle = estructura_detalle_vacia(numeroPasos,cfg)
%ESTRUCTURA_DETALLE_VACIA Inicializa la salida auxiliar.

detalle = struct();

detalle.calculable = false;
detalle.motivo = "";

detalle.distanciaMinima = NaN;
detalle.distanciaPorPaso = nan(numeroPasos,1);

detalle.distanciaLimitesPorPaso = nan(numeroPasos,1);
detalle.distanciaEstaticosPorPaso = nan(numeroPasos,1);
detalle.distanciaDinamicosPorPaso = nan(numeroPasos,1);

detalle.distanciaMinimaLimites = NaN;
detalle.distanciaMinimaEstaticos = NaN;
detalle.distanciaMinimaDinamicos = NaN;

detalle.pasoCritico = NaN;
detalle.tiempoCritico = NaN;
detalle.tipoCritico = "";
detalle.idCritico = "";
detalle.indiceCritico = NaN;

detalle.tipoMasCercanoPorPaso = strings(numeroPasos,1);
detalle.idMasCercanoPorPaso = strings(numeroPasos,1);
detalle.indiceMasCercanoPorPaso = nan(numeroPasos,1);

detalle.umbralRiesgo = cfg.metricas.distanciaRiesgo;
detalle.riesgoPorPaso = false(numeroPasos,1);
detalle.numeroPasosRiesgo = 0;
detalle.numeroEpisodiosRiesgo = 0;
detalle.porcentajePasosRiesgo = NaN;
detalle.primerPasoRiesgo = NaN;
detalle.primerTiempoRiesgo = NaN;
detalle.cumpleUmbralRiesgo = false;

detalle.colisionPorPaso = false(numeroPasos,1);
detalle.colisionOcurrida = false;
detalle.numeroPasosColision = 0;
detalle.numeroEpisodiosColision = 0;
detalle.primerPasoColision = NaN;
detalle.primerTiempoColision = NaN;

detalle.penetracionMaxima = NaN;

detalle.numeroPasos = numeroPasos;
detalle.periodoMuestreo = cfg.sim.Ts;
detalle.tiempos = (0:numeroPasos-1)'*cfg.sim.Ts;

detalle.unidad = "m";
detalle.criterio = "mayor_valor_mayor_seguridad";
end

function numero = contar_episodios(indicador)
%CONTAR_EPISODIOS Cuenta transiciones de false a true.

indicador = logical(indicador(:));

if isempty(indicador)
    numero = 0;
    return;
end

inicios = indicador & [true; ~indicador(1:end-1)];
numero = nnz(inicios);
end

function indice = primer_indice(indicador)
%PRIMER_INDICE Devuelve el primer indice verdadero o NaN.

indice = find(indicador,1,'first');

if isempty(indice)
    indice = NaN;
end
end

function tiempo = indice_a_tiempo(indice,Ts)
%INDICE_A_TIEMPO Convierte un indice de muestra en tiempo simulado.

if isnan(indice)
    tiempo = NaN;
else
    tiempo = (indice-1)*Ts;
end
end

function valor = minimo_finito(datos)
%MINIMO_FINITO Devuelve NaN cuando una categoria no existe.

datos = datos(isfinite(datos));

if isempty(datos)
    valor = NaN;
else
    valor = min(datos);
end
end

function [trayectoria,historial,escenario,robot,cfg] = ...
    validar_entradas(trayectoria,historial,escenario,robot,cfg)
%VALIDAR_ENTRADAS Comprueba la coherencia temporal y geometrica.

%% Trayectoria
if ~isnumeric(trayectoria) || ~isreal(trayectoria)
    error('distancia_seguridad:TrayectoriaNoValida', ...
        'La trayectoria debe ser una matriz numerica real.');
end

if ~ismatrix(trayectoria) || ...
        (~isempty(trayectoria) && size(trayectoria,2) < 2)
    error('distancia_seguridad:DimensionTrayectoriaNoValida', ...
        'La trayectoria debe tener formato N x M con M >= 2.');
end

if any(~isfinite(trayectoria(:)))
    error('distancia_seguridad:TrayectoriaNoFinita', ...
        ['La trayectoria contiene NaN o Inf. Recorte primero las filas ' ...
         'no utilizadas del historial preasignado.']);
end

trayectoria = double(trayectoria);
numeroPasos = size(trayectoria,1);

%% Escenario
if ~isstruct(escenario) || ~isscalar(escenario)
    error('distancia_seguridad:EscenarioNoValido', ...
        'escenario debe ser la estructura obtenida mediante escenarios.m.');
end

camposEscenario = { ...
    'limites', ...
    'obstaculosEstaticos', ...
    'obstaculosDinamicos'};

for i = 1:numel(camposEscenario)
    if ~isfield(escenario,camposEscenario{i})
        error('distancia_seguridad:EscenarioIncompleto', ...
            'Falta escenario.%s.',camposEscenario{i});
    end
end

if ~isnumeric(escenario.limites) || ...
        numel(escenario.limites) ~= 4 || ...
        any(~isfinite(escenario.limites(:))) || ...
        escenario.limites(1) >= escenario.limites(2) || ...
        escenario.limites(3) >= escenario.limites(4)
    error('distancia_seguridad:LimitesNoValidos', ...
        'escenario.limites debe ser [xmin xmax ymin ymax].');
end

escenario.limites = reshape( ...
    double(escenario.limites),1,4);

estaticos = escenario.obstaculosEstaticos;

if isempty(estaticos)
    escenario.obstaculosEstaticos = zeros(0,4);
elseif ~isnumeric(estaticos) || ~isreal(estaticos) || ...
        size(estaticos,2) ~= 4 || ...
        any(~isfinite(estaticos(:))) || ...
        any(estaticos(:,3:4) <= 0,'all')
    error('distancia_seguridad:EstaticosNoValidos', ...
        'Los obstaculos estaticos deben tener formato N x 4.');
else
    escenario.obstaculosEstaticos = double(estaticos);
end

dinamicos = escenario.obstaculosDinamicos;

if ~isstruct(dinamicos)
    error('distancia_seguridad:DinamicosNoValidos', ...
        'escenario.obstaculosDinamicos debe ser un vector de estructuras.');
end

for i = 1:numel(dinamicos)
    if ~isfield(dinamicos(i),'radio') || ...
            ~isnumeric(dinamicos(i).radio) || ...
            ~isscalar(dinamicos(i).radio) || ...
            ~isreal(dinamicos(i).radio) || ...
            ~isfinite(dinamicos(i).radio) || ...
            dinamicos(i).radio <= 0
        error('distancia_seguridad:RadioDinamicoNoValido', ...
            'El radio del obstaculo dinamico %d no es valido.',i);
    end

    dinamicos(i).radio = double(dinamicos(i).radio);
end

escenario.obstaculosDinamicos = dinamicos;
numeroDinamicos = numel(dinamicos);

%% Historial de obstaculos dinamicos
if numeroDinamicos == 0
    if isempty(historial)
        historial = zeros(numeroPasos,0,2);
    elseif ~isnumeric(historial) || ...
            size(historial,1) ~= numeroPasos || ...
            size(historial,2) ~= 0
        error('distancia_seguridad:HistorialDinamicoNoValido', ...
            'El escenario no contiene obstaculos dinamicos.');
    end
else
    if ~isnumeric(historial) || ~isreal(historial) || ...
            size(historial,1) ~= numeroPasos || ...
            size(historial,2) ~= numeroDinamicos || ...
            size(historial,3) ~= 2 || ...
            any(~isfinite(historial(:)))
        error('distancia_seguridad:HistorialDinamicoNoValido', ...
            ['historialObstaculos debe tener dimensiones ' ...
             'N x numeroDinamicos x 2 y coincidir temporalmente ' ...
             'con la trayectoria.']);
    end

    historial = double(historial);
end

%% Robot
if ~isstruct(robot) || ~isscalar(robot) || ...
        ~isfield(robot,'geometria') || ...
        ~isstruct(robot.geometria) || ...
        ~isfield(robot.geometria,'radio') || ...
        ~isnumeric(robot.geometria.radio) || ...
        ~isscalar(robot.geometria.radio) || ...
        ~isreal(robot.geometria.radio) || ...
        ~isfinite(robot.geometria.radio) || ...
        robot.geometria.radio <= 0
    error('distancia_seguridad:RobotNoValido', ...
        'robot debe proceder de configuracion_robot.m.');
end

%% Configuracion
if ~isstruct(cfg) || ~isscalar(cfg) || ...
        ~isfield(cfg,'sim') || ...
        ~isstruct(cfg.sim) || ...
        ~isfield(cfg.sim,'Ts') || ...
        ~isnumeric(cfg.sim.Ts) || ...
        ~isscalar(cfg.sim.Ts) || ...
        ~isreal(cfg.sim.Ts) || ...
        ~isfinite(cfg.sim.Ts) || ...
        cfg.sim.Ts <= 0
    error('distancia_seguridad:TsNoValido', ...
        'Falta un cfg.sim.Ts positivo.');
end

if ~isfield(cfg,'metricas') || ...
        ~isstruct(cfg.metricas) || ...
        ~isfield(cfg.metricas,'distanciaRiesgo') || ...
        ~isnumeric(cfg.metricas.distanciaRiesgo) || ...
        ~isscalar(cfg.metricas.distanciaRiesgo) || ...
        ~isreal(cfg.metricas.distanciaRiesgo) || ...
        ~isfinite(cfg.metricas.distanciaRiesgo) || ...
        cfg.metricas.distanciaRiesgo < 0
    error('distancia_seguridad:UmbralRiesgoNoValido', ...
        'cfg.metricas.distanciaRiesgo debe ser no negativo.');
end
end
