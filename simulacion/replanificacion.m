function [debeReplanificar, info] = replanificacion( ...
    camino, estadoRobot, pasoActual, robot, ...
    obstaculosEstaticos, obstaculosDinamicos, limites, cfg, ...
    siguientePasoPermitido)
% Decide si debe recalcularse la trayectoria global.
%
%   debe = REPLANIFICACION(camino,estadoRobot,pasoActual,robot, ...
%       obstaculosEstaticos,obstaculosDinamicos,limites,cfg)
%
%   [debe,info] = REPLANIFICACION(...,siguientePasoPermitido)
%
%   Esta funcion aplica el mismo criterio a RRT*+APF, RRT*+MPC y
%   PRM+MPC. No ejecuta ningun planificador: solo devuelve true o false.
%
%   Se solicita replanificacion cuando:
%       - no existe un camino utilizable;
%       - una arista del camino deja de ser segura;
%       - se alcanza el periodo comun cfg.replan.periodo.
%
%   Para los obstaculos dinamicos se utiliza la prediccion:
%
%       pFutura = pActual + cfg.prediccion.pasos*cfg.sim.Ts*velocidad
%
%   El argumento opcional siguientePasoPermitido evita reintentos en cada
%   paso despues de un fallo de planificacion. Su valor por defecto es 1.
%
%   La estructura info indica la causa de la decision y, cuando existe un
%   bloqueo, el segmento y el obstaculo responsables.
%
%   Utilidades empleadas:
%       - segmento_rectangulo.m
%       - distancia_segmentos.m

if nargin < 9 || isempty(siguientePasoPermitido)
    siguientePasoPermitido = 1;
end

validar_entradas( ...
    camino,estadoRobot,pasoActual,robot, ...
    obstaculosEstaticos,obstaculosDinamicos,limites,cfg, ...
    siguientePasoPermitido);

centroRobot = reshape(double(estadoRobot),1,[]);
centroRobot = centroRobot(1:2);
modo = lower(string(cfg.replan.modo));

%% 1. Camino inexistente
caminoAusente = isempty(camino) || size(camino,1) < 2;

%% 2. Replanificacion periodica
% Produce los pasos 1, 1+periodo, 1+2*periodo, ...
periodica = mod(pasoActual-1,cfg.replan.periodo) == 0;

%% 3. Bloqueo de la trayectoria
detalleBloqueo = bloqueo_vacio();

usaBloqueo = any(modo == [ ...
    "bloqueo_o_periodico", ...
    "solo_bloqueo"]);

if ~caminoAusente && usaBloqueo
    caminoEvaluado = double(camino(:,1:2));

    % La primera arista comienza siempre en la posicion real del robot.
    caminoEvaluado(1,:) = centroRobot;

    detalleBloqueo = camino_bloqueado( ...
        caminoEvaluado,robot.geometria.radio, ...
        obstaculosEstaticos,obstaculosDinamicos,limites,cfg);
end

%% 4. Aplicacion del modo configurado
switch modo
    case "bloqueo_o_periodico"
        solicitada = caminoAusente || ...
            detalleBloqueo.bloqueo || periodica;

    case "solo_bloqueo"
        solicitada = caminoAusente || detalleBloqueo.bloqueo;

    case "solo_periodico"
        solicitada = caminoAusente || periodica;

    otherwise
        error('replanificacion:ModoNoValido', ...
            'Modo de replanificacion no reconocido: %s.',modo);
end

permitida = pasoActual >= siguientePasoPermitido;
debeReplanificar = solicitada && permitida;

%% 5. Motivo de la decision
if caminoAusente
    motivoSolicitado = "camino_ausente";
elseif detalleBloqueo.bloqueoLimites
    motivoSolicitado = "bloqueo_limite";
elseif detalleBloqueo.bloqueoEstatico
    motivoSolicitado = "bloqueo_estatico";
elseif detalleBloqueo.bloqueoDinamico
    motivoSolicitado = "bloqueo_dinamico";
elseif periodica && modo ~= "solo_bloqueo"
    motivoSolicitado = "periodica";
else
    motivoSolicitado = "ninguno";
end

if solicitada && ~permitida
    motivo = "espera_reintento";
elseif solicitada
    motivo = motivoSolicitado;
else
    motivo = "ninguno";
end

%% Salida
info = detalleBloqueo;

info.debeReplanificar = debeReplanificar;
info.solicitada = solicitada;
info.permitida = permitida;

info.caminoAusente = caminoAusente;
info.periodica = periodica;
info.modo = modo;

info.pasoActual = pasoActual;
info.siguientePasoPermitido = siguientePasoPermitido;
info.pasosDeEspera = max(0,siguientePasoPermitido-pasoActual);

info.motivo = motivo;
info.motivoSolicitado = motivoSolicitado;

info.pasosPrediccion = cfg.prediccion.pasos;
info.horizontePrediccion = ...
    cfg.prediccion.pasos*cfg.sim.Ts;
info.obstaculosDinamicosConsiderados = ...
    considerar_dinamicos(cfg);
end

%% ========================================================================
% COMPROBACION DE LA TRAYECTORIA
% ========================================================================

function detalle = camino_bloqueado( ...
    camino,radioRobot,estaticos,dinamicos,limites,cfg)
%CAMINO_BLOQUEADO Comprueba las aristas restantes en orden de recorrido.

detalle = bloqueo_vacio();

distanciaEstatica = ...
    radioRobot + cfg.seguridad.margenEstatico;

tol = 1e-12*max(1,max(abs([camino(:); limites(:)])));

for s = 1:size(camino,1)-1
    a = camino(s,:);
    b = camino(s+1,:);

    %% Limites del mapa
    [distancia,indiceLimite] = ...
        distancia_a_limites(a,b,limites);

    libre = distancia-distanciaEstatica;
    detalle.distanciaLibreMinima = ...
        min(detalle.distanciaLibreMinima,libre);

    if libre <= tol
        detalle.bloqueo = true;
        detalle.bloqueoLimites = true;
        detalle.segmentoBloqueado = s;
        detalle.tipoObstaculo = "limite";
        detalle.indiceObstaculo = indiceLimite;
        detalle.idObstaculo = nombre_limite(indiceLimite);
        detalle.distanciaLibre = libre;
        return;
    end

    %% Obstaculos estaticos
    for i = 1:size(estaticos,1)
        distancia = distancia_segmento_rectangulo( ...
            a,b,estaticos(i,:),tol);

        libre = distancia-distanciaEstatica;
        detalle.distanciaLibreMinima = ...
            min(detalle.distanciaLibreMinima,libre);

        if libre <= tol
            detalle.bloqueo = true;
            detalle.bloqueoEstatico = true;
            detalle.segmentoBloqueado = s;
            detalle.tipoObstaculo = "estatico";
            detalle.indiceObstaculo = i;
            detalle.idObstaculo = "S"+i;
            detalle.distanciaLibre = libre;
            return;
        end
    end

    %% Obstaculos dinamicos predichos
    % Por defecto, los cruces dinamicos no invalidan la ruta global. La
    % reaccion frente a ellos corresponde al controlador local.
    if considerar_dinamicos(cfg)
        for i = 1:numel(dinamicos)
            posicionActual = dinamicos(i).pos;

            posicionFutura = posicionActual + ...
                cfg.prediccion.pasos*cfg.sim.Ts*dinamicos(i).vel;

            distancia = distancia_segmentos( ...
                a,b,posicionActual,posicionFutura);

            distanciaDinamica = ...
                radioRobot + dinamicos(i).radio + ...
                cfg.seguridad.margenDinamico;

            libre = distancia-distanciaDinamica;
            detalle.distanciaLibreMinima = ...
                min(detalle.distanciaLibreMinima,libre);

            if libre <= tol
                detalle.bloqueo = true;
                detalle.bloqueoDinamico = true;
                detalle.segmentoBloqueado = s;
                detalle.tipoObstaculo = "dinamico";
                detalle.indiceObstaculo = i;
                detalle.idObstaculo = id_dinamico(dinamicos,i);
                detalle.distanciaLibre = libre;
                detalle.posicionDinamicaActual = posicionActual;
                detalle.posicionDinamicaPredicha = posicionFutura;
                return;
            end
        end
    end
end
end

function detalle = bloqueo_vacio()
%BLOQUEO_VACIO Inicializa el diagnostico geometrico.

detalle = struct();

detalle.bloqueo = false;
detalle.bloqueoLimites = false;
detalle.bloqueoEstatico = false;
detalle.bloqueoDinamico = false;

detalle.segmentoBloqueado = NaN;
detalle.tipoObstaculo = "ninguno";
detalle.indiceObstaculo = NaN;
detalle.idObstaculo = "";

detalle.distanciaLibre = inf;
detalle.distanciaLibreMinima = inf;

detalle.posicionDinamicaActual = [NaN NaN];
detalle.posicionDinamicaPredicha = [NaN NaN];
end

%% ========================================================================
% GEOMETRIA AUXILIAR
% ========================================================================

function d = distancia_segmento_rectangulo(a,b,r,tol)
%DISTANCIA_SEGMENTO_RECTANGULO Distancia entre segmento y rectangulo.

if segmento_rectangulo(a,b,r,tol)
    d = 0;
    return;
end

xmin = r(1);
ymin = r(2);
xmax = xmin+r(3);
ymax = ymin+r(4);

esquinas = [
    xmin ymin;
    xmax ymin;
    xmax ymax;
    xmin ymax
];

lados = [
    1 2;
    2 3;
    3 4;
    4 1
];

d = inf;

for k = 1:4
    c = esquinas(lados(k,1),:);
    f = esquinas(lados(k,2),:);
    d = min(d,distancia_segmentos(a,b,c,f));
end
end

function [d,indice] = distancia_a_limites(a,b,limites)
%DISTANCIA_A_LIMITES Menor distancia del segmento al borde del mapa.

distancias = [
    min(a(1),b(1))-limites(1);
    limites(2)-max(a(1),b(1));
    min(a(2),b(2))-limites(3);
    limites(4)-max(a(2),b(2))
];

[d,indice] = min(distancias);
end

function nombre = nombre_limite(indice)
%NOMBRE_LIMITE Identificador del borde del mapa.

nombres = ["izquierdo","derecho","inferior","superior"];
nombre = nombres(indice);
end

function id = id_dinamico(dinamicos,indice)
%ID_DINAMICO Devuelve el id almacenado o genera D1, D2, ...

if isfield(dinamicos(indice),'id') && ...
        strlength(string(dinamicos(indice).id)) > 0
    id = string(dinamicos(indice).id);
else
    id = "D"+indice;
end
end

function tf = considerar_dinamicos(cfg)
%CONSIDERAR_DINAMICOS Controla si un cruce movil invalida la ruta global.

tf = false;

if isfield(cfg,'replan') && isstruct(cfg.replan) && ...
        isfield(cfg.replan,'considerarDinamicos')
    valor = cfg.replan.considerarDinamicos;

    if islogical(valor) && isscalar(valor)
        tf = logical(valor);
    elseif isnumeric(valor) && isreal(valor) && isscalar(valor) && ...
            isfinite(valor) && any(valor == [0 1])
        tf = logical(valor);
    else
        error('replanificacion:ConsiderarDinamicosNoValido', ...
            'cfg.replan.considerarDinamicos debe ser un logico escalar.');
    end
end
end

%% ========================================================================
% VALIDACION
% ========================================================================

function validar_entradas( ...
    camino,estadoRobot,pasoActual,robot, ...
    estaticos,dinamicos,limites,cfg,siguientePasoPermitido)
%VALIDAR_ENTRADAS Comprueba únicamente las condiciones necesarias.

if ~isempty(camino) && ( ...
        ~isnumeric(camino) || ~isreal(camino) || ...
        size(camino,2) < 2 || any(~isfinite(camino(:))))
    error('replanificacion:CaminoNoValido', ...
        'El camino debe ser una matriz numerica N x 2.');
end

if ~isnumeric(estadoRobot) || ~isreal(estadoRobot) || ...
        numel(estadoRobot) < 2 || any(~isfinite(estadoRobot(:)))
    error('replanificacion:EstadoNoValido', ...
        'estadoRobot debe contener al menos [x y].');
end

if ~es_entero_positivo(pasoActual) || ...
        ~es_entero_positivo(siguientePasoPermitido)
    error('replanificacion:PasoNoValido', ...
        'Los pasos de simulacion deben ser enteros positivos.');
end

if ~isstruct(robot) || ~isfield(robot,'geometria') || ...
        ~isfield(robot.geometria,'radio') || ...
        ~isscalar(robot.geometria.radio) || ...
        robot.geometria.radio <= 0
    error('replanificacion:RobotNoValido', ...
        'robot debe proceder de configuracion_robot.m.');
end

if ~isempty(estaticos) && ( ...
        ~isnumeric(estaticos) || size(estaticos,2) ~= 4 || ...
        any(~isfinite(estaticos(:))) || ...
        any(estaticos(:,3:4) <= 0,'all'))
    error('replanificacion:EstaticosNoValidos', ...
        'Los obstaculos estaticos deben tener formato N x 4.');
end

if ~isstruct(dinamicos)
    error('replanificacion:DinamicosNoValidos', ...
        'Los obstaculos dinamicos deben ser estructuras.');
end

for i = 1:numel(dinamicos)
    if ~all(isfield(dinamicos(i),{'pos','vel','radio'})) || ...
            numel(dinamicos(i).pos) ~= 2 || ...
            numel(dinamicos(i).vel) ~= 2 || ...
            ~isscalar(dinamicos(i).radio) || ...
            any(~isfinite([ ...
                dinamicos(i).pos(:); ...
                dinamicos(i).vel(:); ...
                dinamicos(i).radio])) || ...
            dinamicos(i).radio <= 0
        error('replanificacion:DinamicoNoValido', ...
            'El obstaculo dinamico %d no es valido.',i);
    end
end

if ~isnumeric(limites) || numel(limites) ~= 4 || ...
        any(~isfinite(limites(:))) || ...
        limites(1) >= limites(2) || limites(3) >= limites(4)
    error('replanificacion:LimitesNoValidos', ...
        'limites debe tener formato [xmin xmax ymin ymax].');
end

camposNecesarios = {
    'replan','modo';
    'replan','periodo';
    'sim','Ts';
    'seguridad','margenEstatico';
    'seguridad','margenDinamico';
    'prediccion','pasos';
    'prediccion','modelo'
};

for i = 1:size(camposNecesarios,1)
    grupo = camposNecesarios{i,1};
    campo = camposNecesarios{i,2};

    if ~isfield(cfg,grupo) || ...
            ~isstruct(cfg.(grupo)) || ...
            ~isfield(cfg.(grupo),campo)
        error('replanificacion:ConfiguracionIncompleta', ...
            'Falta cfg.%s.%s.',grupo,campo);
    end
end

if ~es_entero_positivo(cfg.replan.periodo)
    error('replanificacion:PeriodoNoValido', ...
        'cfg.replan.periodo debe ser un entero positivo.');
end

if ~isscalar(cfg.sim.Ts) || ~isfinite(cfg.sim.Ts) || cfg.sim.Ts <= 0
    error('replanificacion:TsNoValido', ...
        'cfg.sim.Ts debe ser positivo.');
end

if any([cfg.seguridad.margenEstatico, ...
        cfg.seguridad.margenDinamico] < 0)
    error('replanificacion:MargenNoValido', ...
        'Los margenes de seguridad no pueden ser negativos.');
end

if cfg.prediccion.pasos < 0 || ...
        cfg.prediccion.pasos ~= floor(cfg.prediccion.pasos)
    error('replanificacion:PrediccionNoValida', ...
        'cfg.prediccion.pasos debe ser un entero no negativo.');
end

if string(cfg.prediccion.modelo) ~= "velocidad_constante"
    error('replanificacion:ModeloNoSoportado', ...
        'Esta version utiliza prediccion a velocidad constante.');
end
end

function tf = es_entero_positivo(x)
%ES_ENTERO_POSITIVO Comprueba un escalar entero mayor o igual que uno.

tf = isnumeric(x) && isscalar(x) && isreal(x) && ...
    isfinite(x) && x >= 1 && x == floor(x);
end
