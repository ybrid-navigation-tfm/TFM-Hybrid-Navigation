function [control, info] = mpc( ...
    estadoRobot, objetivoLocal, meta, obstaculosEstaticos, ...
    obstaculosDinamicos, limites, robot, cfg, controlAnterior)
%MPC Calcula el control local mediante Control Predictivo Basado en Modelo.
%
%   control = MPC(estadoRobot,objetivoLocal,meta,obstaculosEstaticos, ...
%       obstaculosDinamicos,limites,robot,cfg)
%
%   [control,info] = MPC(...,controlAnterior)
%
%   Implementa el controlador local comun de las arquitecturas:
%
%       - RRT* + MPC
%       - PRM  + MPC
%
%   El planificador global proporciona un camino y
%   seguimiento_trayectoria.m selecciona un waypoint adelantado. Este
%   modulo evalua una rejilla discreta de controles [v,w], predice durante
%   Np pasos el modelo de uniciclo y los obstaculos dinamicos, y devuelve
%   el control de menor coste.
%
%   VERSION DISCRETA DE REFERENCIA
%   ------------------------------
%   Se conservan los parametros de la implementacion funcional:
%
%       cfg.mpc.nVelocidades
%       cfg.mpc.nGiros
%       cfg.mpc.Np
%
%   Cada pareja [v,w] se mantiene constante durante el horizonte. Se aplica
%   solamente el primer control y el problema se vuelve a resolver en el
%   ciclo siguiente. De esta forma se mantiene un coste computacional
%   acotado, reproducible y sin requerir Optimization Toolbox ni Model
%   Predictive Control Toolbox.
%
%   Entradas:
%       estadoRobot            [x y theta]
%       objetivoLocal          waypoint local [x y]
%       meta                   objetivo final [x y]
%       obstaculosEstaticos    matriz N x 4 [x y ancho alto]
%       obstaculosDinamicos    estructuras con pos, vel y radio
%       limites                [xmin xmax ymin ymax]
%       robot                  salida de configuracion_robot.m
%       cfg                    salida de parametros_generales.m
%       controlAnterior        control aplicado previamente [v w]
%
%   Si controlAnterior se omite o se proporciona [], se utiliza
%   robot.controlInicial.
%
%   Salidas:
%       control                control solicitado [v w]
%       info                   diagnostico de la optimizacion discreta
%
%   Modelo predictivo:
%
%       x(k+1)     = x(k) + Ts*v*cos(theta(k))
%       y(k+1)     = y(k) + Ts*v*sin(theta(k))
%       theta(k+1) = theta(k) + Ts*w
%
%   Prediccion de obstaculos dinamicos:
%
%       o_i(k+h) = o_i(k) + h*Ts*vel_i,   h = 1,...,Np
%
%   Funcion objetivo:
%
%       J = sum_h [
%               factorMeta*Qmeta*||p_h-meta||^2
%             + Qcamino*||p_h-objetivoLocal||^2
%             + JobsEstaticos(h)
%             + JobsDinamicos(h)]
%           + Np*RdeltaU*||u-controlAnterior||^2
%
%   Para una distancia libre d superior al umbral de seguridad:
%
%       Jobs = Qobstaculo/(d^2 + epsilonObstaculo)
%
%   Si d queda por debajo del umbral, se añade
%   cfg.mpc.penalizacionColision y el candidato se marca como no factible.
%   Cuando existe al menos un candidato factible, la seleccion se restringe
%   a ellos. Si todos incumplen algun umbral, se devuelve el candidato con
%   menos violaciones y menor coste, marcando info.exito=false.
%
%   Los umbrales efectivos nunca son menores que los margenes comunes de
%   seguridad usados por los planificadores y la replanificacion:
%
%       umbralEstatico = max(cfg.mpc.umbralEstatico, ...
%                            cfg.seguridad.margenEstatico)
%
%       umbralDinamico = max(cfg.mpc.umbralDinamico, ...
%                            cfg.seguridad.margenDinamico)
%
%   Los campos opcionales:
%
%       cfg.mpc.factorMeta
%       cfg.mpc.epsilonObstaculo
%
%   permiten centralizar dos constantes de la implementacion. Si faltan,
%   se utilizan respectivamente 0.25 y 1e-3, valores conservados de los
%   codigos funcionales de referencia.
%
%   Ejemplo:
%
%       cfg = parametros_generales("visual");
%       escenario = escenarios("media");
%       robot = configuracion_robot();
%
%       [u,infoMPC] = mpc( ...
%           escenario.inicio,[3 3],escenario.meta, ...
%           escenario.obstaculosEstaticos, ...
%           escenario.obstaculosDinamicos, ...
%           escenario.limites,robot,cfg,robot.controlInicial);
%
%       [estadoSiguiente,uAplicado] = modelo_robot( ...
%           escenario.inicio,u,cfg.sim.Ts,robot);
%
%   Esta funcion utiliza:
%       - wrap_to_pi_local.m
%
%   El tiempo de control debe medirse externamente mediante tic/toc para
%   que tiempos.m compare APF y MPC con el mismo criterio.

if nargin < 9 || isempty(controlAnterior)
    if isstruct(robot) && isfield(robot,'controlInicial')
        controlAnterior = robot.controlInicial;
    else
        controlAnterior = [0 0];
    end
end

%% Validacion y normalizacion
[estadoRobot,objetivoLocal,meta,estaticos,dinamicos,limites, ...
    controlAnterior,p] = validar_entradas( ...
        estadoRobot,objetivoLocal,meta,obstaculosEstaticos, ...
        obstaculosDinamicos,limites,robot,cfg,controlAnterior);

posicionRobot = estadoRobot(1:2);
distanciaMeta = norm(posicionRobot-meta);
distanciaObjetivo = norm(posicionRobot-objetivoLocal);
metaAlcanzada = distanciaMeta <= p.radioMeta+p.tolerancia;
objetivoCercano = distanciaObjetivo <= p.tolWaypoint+p.tolerancia;

%% Parada al alcanzar la meta final
if metaAlcanzada
    control = p.controlParada;
    info = informacion_parada( ...
        estadoRobot,objetivoLocal,meta,controlAnterior,control, ...
        distanciaObjetivo,distanciaMeta,objetivoCercano,p);
    return;
end

%% Rejilla discreta de controles
[vGrid,wGrid,controles] = crear_candidatos(p);
numeroCandidatos = size(controles,1);

%% Prediccion de los obstaculos dinamicos
prediccionDinamicos = predecir_dinamicos(dinamicos,p.Np,p.Ts);

%% Evaluacion vectorizada del horizonte
E = evaluar_candidatos( ...
    estadoRobot,objetivoLocal,meta,estaticos,dinamicos,limites, ...
    controlAnterior,controles,prediccionDinamicos,p);

%% Seleccion determinista
[indiceOptimo,hayFactible,criterioDesempate] = ...
    seleccionar_candidato(E,controles,controlAnterior);

control = controles(indiceOptimo,:);

if hayFactible
    motivo = "control_optimo_factible";
else
    motivo = "sin_candidato_factible_mejor_penalizado";
end

if any(~isfinite(control)) || ~isequal(size(control),[1 2])
    error('mpc:ControlNoFinito', ...
        'La evaluacion del MPC produjo un control no finito.');
end

estadosOptimos = reshape( ...
    E.estadosPredichos(indiceOptimo,:,:),p.Np,3);

trayectoriaPredicha = [estadoRobot; estadosOptimos];
secuenciaControl = repmat(control,p.Np,1);

[dMin,pasoCritico,tipoCritico,idCritico,indiceCritico] = ...
    localizar_critico(indiceOptimo,E,estaticos,dinamicos);

%% Informacion de diagnostico
info = struct();
info.exito = hayFactible;
info.controlValido = true;
info.motivo = motivo;
info.tipoMPC = "busqueda_discreta_control_constante";

info.estadoRobot = estadoRobot;
info.posicionRobot = posicionRobot;
info.objetivoLocal = objetivoLocal;
info.meta = meta;
info.distanciaObjetivoLocal = distanciaObjetivo;
info.distanciaMeta = distanciaMeta;
info.objetivoDentroTolerancia = objetivoCercano;
info.metaDentroTolerancia = metaAlcanzada;

info.controlAnterior = controlAnterior;
info.controlSolicitado = control;
info.secuenciaControlOptima = secuenciaControl;
info.trayectoriaPredicha = trayectoriaPredicha;
info.posicionesDinamicasPredichas = prediccionDinamicos;

info.valoresVelocidad = vGrid;
info.valoresGiro = wGrid;
info.numeroCandidatos = numeroCandidatos;
info.numeroCandidatosFactibles = nnz(E.factible);
info.fraccionCandidatosFactibles = nnz(E.factible)/numeroCandidatos;
info.numeroCandidatosConRiesgo = nnz(E.numeroViolaciones > 0);
info.numeroCandidatosConColisionFisica = nnz(E.colisionFisica);

info.indiceCandidatoOptimo = indiceOptimo;
info.candidatoOptimoFactible = E.factible(indiceOptimo);
info.criterioDesempate = criterioDesempate;

info.costeOptimo = E.costeTotal(indiceOptimo);
info.costeMedioPorPaso = E.costeTotal(indiceOptimo)/p.Np;
info.costesOptimos = struct( ...
    'meta',E.costeMeta(indiceOptimo), ...
    'camino',E.costeCamino(indiceOptimo), ...
    'variacionControl',E.costeDeltaU(indiceOptimo), ...
    'obstaculosEstaticos',E.costeEstaticos(indiceOptimo), ...
    'obstaculosDinamicos',E.costeDinamicos(indiceOptimo), ...
    'total',E.costeTotal(indiceOptimo));

info.distanciaMinimaPredicha = dMin;
info.distanciaMinimaEstaticaPredicha = min( ...
    E.distanciasEstaticas(indiceOptimo,:));
info.distanciaMinimaDinamicaPredicha = min( ...
    E.distanciasDinamicas(indiceOptimo,:));
info.pasoCritico = pasoCritico;
info.tiempoCritico = pasoCritico*p.Ts;
info.tipoCritico = tipoCritico;
info.idCritico = idCritico;
info.indiceCritico = indiceCritico;

info.prediceRiesgo = E.numeroViolaciones(indiceOptimo) > 0;
info.prediceColisionFisica = E.colisionFisica(indiceOptimo);
info.numeroViolacionesSeguridad = E.numeroViolaciones(indiceOptimo);
info.numeroViolacionesEstaticas = sum( ...
    E.violacionEstatica(indiceOptimo,:));
info.numeroViolacionesDinamicas = sum( ...
    E.violacionDinamica(indiceOptimo,:));

info.distanciasOptimas = struct();
info.distanciasOptimas.estaticasPorPaso = ...
    E.distanciasEstaticas(indiceOptimo,:).';
info.distanciasOptimas.dinamicasPorPaso = ...
    E.distanciasDinamicas(indiceOptimo,:).';
info.distanciasOptimas.globalPorPaso = min( ...
    info.distanciasOptimas.estaticasPorPaso, ...
    info.distanciasOptimas.dinamicasPorPaso);
info.distanciasOptimas.codigoEstaticoPorPaso = ...
    E.codigosEstaticos(indiceOptimo,:).';
info.distanciasOptimas.indiceDinamicoPorPaso = ...
    E.indicesDinamicos(indiceOptimo,:).';

% Resumen de todos los candidatos. Son matrices pequenas con la
% configuracion actual (5*9 = 45 candidatos).
info.candidatos = struct();
info.candidatos.controles = controles;
info.candidatos.factibles = E.factible;
info.candidatos.costesTotal = E.costeTotal;
info.candidatos.costesMeta = E.costeMeta;
info.candidatos.costesCamino = E.costeCamino;
info.candidatos.costesVariacionControl = E.costeDeltaU;
info.candidatos.costesEstaticos = E.costeEstaticos;
info.candidatos.costesDinamicos = E.costeDinamicos;
info.candidatos.distanciasMinimas = E.distanciaMinima;
info.candidatos.numeroViolaciones = E.numeroViolaciones;
info.candidatos.colisionFisica = E.colisionFisica;

info.horizontePasos = p.Np;
info.horizontePrediccion = p.Np*p.Ts;
info.numeroPrediccionesEstado = numeroCandidatos*p.Np;
info.parametros = p;

info.formulaModelo = ...
    "x+=Ts*v*cos(theta); y+=Ts*v*sin(theta); theta+=Ts*w";
info.formulaCosteMeta = ...
    "factorMeta*Qmeta*sum(||p_h-meta||^2)";
info.formulaCosteCamino = ...
    "Qcamino*sum(||p_h-objetivoLocal||^2)";
info.formulaCosteControl = ...
    "Np*RdeltaU*||u-controlAnterior||^2";
info.formulaCosteObstaculo = ...
    "Qobstaculo/(distanciaLibre^2+epsilon) o penalizacion";
end

%% ========================================================================
% CANDIDATOS Y PREDICCION
% ========================================================================

function [vGrid,wGrid,controles] = crear_candidatos(p)
%CREAR_CANDIDATOS Construye la rejilla admisible [v,w].

vGrid = linspace(p.vMin,p.vMax,p.nVelocidades);
wGrid = linspace(p.wMin,p.wMax,p.nGiros);
[V,W] = ndgrid(vGrid,wGrid);
controles = [V(:),W(:)];
end

function posiciones = predecir_dinamicos(dinamicos,Np,Ts)
%PREDECIR_DINAMICOS Predice posiciones mediante velocidad constante.

nDinamicos = numel(dinamicos);
posiciones = zeros(Np,nDinamicos,2);

for h = 1:Np
    for i = 1:nDinamicos
        posiciones(h,i,:) = reshape( ...
            dinamicos(i).pos+h*Ts*dinamicos(i).vel,1,1,2);
    end
end
end

%% ========================================================================
% EVALUACION DE LOS CONTROLES
% ========================================================================

function E = evaluar_candidatos( ...
    estado,objetivo,meta,estaticos,dinamicos,limites, ...
    controlAnterior,controles,prediccionDinamicos,p)
%EVALUAR_CANDIDATOS Propaga y puntua todos los controles discretos.

nCandidatos = size(controles,1);
Np = p.Np;
v = controles(:,1);
w = controles(:,2);

x = repmat(estado(1),nCandidatos,1);
y = repmat(estado(2),nCandidatos,1);
theta = repmat(estado(3),nCandidatos,1);

estadosPredichos = nan(nCandidatos,Np,3);
costeMeta = zeros(nCandidatos,1);
costeCamino = zeros(nCandidatos,1);
costeEstaticos = zeros(nCandidatos,1);
costeDinamicos = zeros(nCandidatos,1);

% El control es constante en el horizonte. Esta expresion reproduce la
% penalizacion acumulada de la implementacion funcional de referencia.
deltaU = controles-controlAnterior;
costeDeltaU = p.Np*p.RdeltaU.*sum(deltaU.^2,2);

distanciasEstaticas = inf(nCandidatos,Np);
distanciasDinamicas = inf(nCandidatos,Np);
codigosEstaticos = zeros(nCandidatos,Np);
indicesDinamicos = zeros(nCandidatos,Np);
violacionEstatica = false(nCandidatos,Np);
violacionDinamica = false(nCandidatos,Np);
colisionFisicaPorPaso = false(nCandidatos,Np);

for h = 1:Np
    %% Modelo del uniciclo
    x = x+p.Ts.*v.*cos(theta);
    y = y+p.Ts.*v.*sin(theta);
    theta = wrap_to_pi_local(theta+p.Ts.*w);

    estadosPredichos(:,h,1) = x;
    estadosPredichos(:,h,2) = y;
    estadosPredichos(:,h,3) = theta;

    %% Costes de seguimiento
    costeMeta = costeMeta+ ...
        p.factorMeta*p.Qmeta.*( ...
        (x-meta(1)).^2+(y-meta(2)).^2);

    costeCamino = costeCamino+ ...
        p.Qcamino.*( ...
        (x-objetivo(1)).^2+(y-objetivo(2)).^2);

    %% Obstaculos estaticos y limites
    [dEstatico,codigoEstatico] = distancia_estatica_minima( ...
        x,y,estaticos,limites,p.radioRobot);

    distanciasEstaticas(:,h) = dEstatico;
    codigosEstaticos(:,h) = codigoEstatico;

    riesgoEstatico = dEstatico <= p.umbralEstatico+p.tolerancia;
    violacionEstatica(:,h) = riesgoEstatico;

    costePaso = zeros(nCandidatos,1);
    costePaso(riesgoEstatico) = p.penalizacionColision;
    seguros = ~riesgoEstatico;
    costePaso(seguros) = p.Qobstaculo./( ...
        dEstatico(seguros).^2+p.epsilonObstaculo);
    costeEstaticos = costeEstaticos+costePaso;

    %% Obstaculos dinamicos
    dDinamicoMin = inf(nCandidatos,1);
    indiceDinamicoMin = zeros(nCandidatos,1);
    riesgoDinamicoPaso = false(nCandidatos,1);
    costePasoDinamico = zeros(nCandidatos,1);

    for i = 1:numel(dinamicos)
        pos = reshape(prediccionDinamicos(h,i,:),1,2);
        d = hypot(x-pos(1),y-pos(2))- ...
            p.radioRobot-dinamicos(i).radio;

        actualizar = d < dDinamicoMin;
        dDinamicoMin(actualizar) = d(actualizar);
        indiceDinamicoMin(actualizar) = i;

        riesgo = d <= p.umbralDinamico+p.tolerancia;
        riesgoDinamicoPaso = riesgoDinamicoPaso | riesgo;

        costeObs = zeros(nCandidatos,1);
        costeObs(riesgo) = p.penalizacionColision;
        seguros = ~riesgo;
        costeObs(seguros) = p.Qobstaculo./( ...
            d(seguros).^2+p.epsilonObstaculo);
        costePasoDinamico = costePasoDinamico+costeObs;
    end

    distanciasDinamicas(:,h) = dDinamicoMin;
    indicesDinamicos(:,h) = indiceDinamicoMin;
    violacionDinamica(:,h) = riesgoDinamicoPaso;
    costeDinamicos = costeDinamicos+costePasoDinamico;

    dGlobal = min(dEstatico,dDinamicoMin);
    colisionFisicaPorPaso(:,h) = dGlobal <= p.tolerancia;
end

costeTotal = costeMeta+costeCamino+costeDeltaU+ ...
    costeEstaticos+costeDinamicos;

if any(~isfinite(costeTotal))
    error('mpc:CosteNoFinito', ...
        'La evaluacion produjo uno o varios costes no finitos.');
end

factible = ~any(violacionEstatica,2) & ~any(violacionDinamica,2);
numeroViolaciones = sum(violacionEstatica,2)+sum(violacionDinamica,2);
distanciaMinima = min([distanciasEstaticas,distanciasDinamicas],[],2);
colisionFisica = any(colisionFisicaPorPaso,2);

E = struct();
E.estadosPredichos = estadosPredichos;
E.costeMeta = costeMeta;
E.costeCamino = costeCamino;
E.costeDeltaU = costeDeltaU;
E.costeEstaticos = costeEstaticos;
E.costeDinamicos = costeDinamicos;
E.costeTotal = costeTotal;
E.factible = factible;
E.numeroViolaciones = numeroViolaciones;
E.distanciaMinima = distanciaMinima;
E.colisionFisica = colisionFisica;
E.colisionFisicaPorPaso = colisionFisicaPorPaso;
E.distanciasEstaticas = distanciasEstaticas;
E.distanciasDinamicas = distanciasDinamicas;
E.codigosEstaticos = codigosEstaticos;
E.indicesDinamicos = indicesDinamicos;
E.violacionEstatica = violacionEstatica;
E.violacionDinamica = violacionDinamica;
end

function [dMin,codigo] = distancia_estatica_minima( ...
    x,y,estaticos,limites,radioRobot)
%DISTANCIA_ESTATICA_MINIMA Distancia libre a contorno y rectangulos.
%
%   Codigos:
%       1..4       limites izquierdo, derecho, inferior y superior
%       5..(4+N)   obstaculos S1..SN

n = numel(x);
clearanceLimites = [ ...
    x-limites(1)-radioRobot, ...
    limites(2)-x-radioRobot, ...
    y-limites(3)-radioRobot, ...
    limites(4)-y-radioRobot];

[dMin,codigo] = min(clearanceLimites,[],2);

for i = 1:size(estaticos,1)
    r = estaticos(i,:);
    xmin = r(1);
    ymin = r(2);
    xmax = xmin+r(3);
    ymax = ymin+r(4);

    dx = max(max(xmin-x,zeros(n,1)),x-xmax);
    dy = max(max(ymin-y,zeros(n,1)),y-ymax);
    d = hypot(dx,dy)-radioRobot;

    actualizar = d < dMin;
    dMin(actualizar) = d(actualizar);
    codigo(actualizar) = 4+i;
end
end

%% ========================================================================
% SELECCION DEL CANDIDATO
% ========================================================================

function [indice,hayFactible,criterio] = ...
    seleccionar_candidato(E,controles,controlAnterior)
%SELECCIONAR_CANDIDATO Aplica coste, continuidad y seguridad como desempate.

factibles = find(E.factible);
hayFactible = ~isempty(factibles);

if hayFactible
    candidatos = factibles;
else
    minimoViolaciones = min(E.numeroViolaciones);
    candidatos = find(E.numeroViolaciones == minimoViolaciones);
end

costeMin = min(E.costeTotal(candidatos));
tolCoste = 1e-12*max(1,abs(costeMin));
candidatos = candidatos( ...
    abs(E.costeTotal(candidatos)-costeMin) <= tolCoste);
criterio = "coste_minimo";

if numel(candidatos) > 1
    cambios = vecnorm(controles(candidatos,:)-controlAnterior,2,2);
    cambioMin = min(cambios);
    tolCambio = 1e-12*max(1,abs(cambioMin));
    candidatos = candidatos(abs(cambios-cambioMin) <= tolCambio);
    criterio = "coste_y_variacion_control";
end

if numel(candidatos) > 1
    dMax = max(E.distanciaMinima(candidatos));
    tolD = 1e-12*max(1,abs(dMax));
    candidatos = candidatos( ...
        abs(E.distanciaMinima(candidatos)-dMax) <= tolD);
    criterio = "coste_variacion_y_distancia";
end

indice = candidatos(1);
end

function [dMin,paso,tipo,id,indice] = ...
    localizar_critico(indiceCandidato,E,estaticos,dinamicos)
%LOCALIZAR_CRITICO Identifica la menor distancia de la trayectoria optima.

estaticas = E.distanciasEstaticas(indiceCandidato,:);
dinamicas = E.distanciasDinamicas(indiceCandidato,:);
[dMin,paso] = min(min(estaticas,dinamicas));

if estaticas(paso) <= dinamicas(paso)
    codigo = E.codigosEstaticos(indiceCandidato,paso);
    if codigo <= 4
        nombres = ["izquierdo","derecho","inferior","superior"];
        tipo = "limite";
        id = nombres(codigo);
        indice = codigo;
    else
        indice = codigo-4;
        tipo = "estatico";
        if indice >= 1 && indice <= size(estaticos,1)
            id = "S"+indice;
        else
            id = "";
            indice = NaN;
        end
    end
else
    indice = E.indicesDinamicos(indiceCandidato,paso);
    tipo = "dinamico";
    if indice >= 1 && indice <= numel(dinamicos)
        id = id_dinamico(dinamicos(indice),indice);
    else
        id = "";
        indice = NaN;
    end
end
end

%% ========================================================================
% SALIDA DE PARADA
% ========================================================================

function info = informacion_parada( ...
    estado,objetivo,meta,controlAnterior,control, ...
    distanciaObjetivo,distanciaMeta,objetivoCercano,p)
%INFORMACION_PARADA Devuelve el contrato completo sin optimizar.

info = struct();
info.exito = true;
info.controlValido = true;
info.motivo = "meta_dentro_tolerancia";
info.tipoMPC = "busqueda_discreta_control_constante";
info.estadoRobot = estado;
info.posicionRobot = estado(1:2);
info.objetivoLocal = objetivo;
info.meta = meta;
info.distanciaObjetivoLocal = distanciaObjetivo;
info.distanciaMeta = distanciaMeta;
info.objetivoDentroTolerancia = objetivoCercano;
info.metaDentroTolerancia = true;
info.controlAnterior = controlAnterior;
info.controlSolicitado = control;
info.secuenciaControlOptima = repmat(control,p.Np,1);
info.trayectoriaPredicha = repmat(estado,p.Np+1,1);
info.posicionesDinamicasPredichas = zeros(p.Np,0,2);
info.valoresVelocidad = linspace(p.vMin,p.vMax,p.nVelocidades);
info.valoresGiro = linspace(p.wMin,p.wMax,p.nGiros);
info.numeroCandidatos = p.nVelocidades*p.nGiros;
info.numeroCandidatosFactibles = NaN;
info.fraccionCandidatosFactibles = NaN;
info.numeroCandidatosConRiesgo = NaN;
info.numeroCandidatosConColisionFisica = NaN;
info.indiceCandidatoOptimo = NaN;
info.candidatoOptimoFactible = true;
info.criterioDesempate = "meta_alcanzada";
info.costeOptimo = 0;
info.costeMedioPorPaso = 0;
info.costesOptimos = struct( ...
    'meta',0,'camino',0,'variacionControl',0, ...
    'obstaculosEstaticos',0,'obstaculosDinamicos',0,'total',0);
info.distanciaMinimaPredicha = NaN;
info.distanciaMinimaEstaticaPredicha = NaN;
info.distanciaMinimaDinamicaPredicha = NaN;
info.pasoCritico = NaN;
info.tiempoCritico = NaN;
info.tipoCritico = "ninguno";
info.idCritico = "";
info.indiceCritico = NaN;
info.prediceRiesgo = false;
info.prediceColisionFisica = false;
info.numeroViolacionesSeguridad = 0;
info.numeroViolacionesEstaticas = 0;
info.numeroViolacionesDinamicas = 0;
info.distanciasOptimas = struct( ...
    'estaticasPorPaso',nan(p.Np,1), ...
    'dinamicasPorPaso',nan(p.Np,1), ...
    'globalPorPaso',nan(p.Np,1), ...
    'codigoEstaticoPorPaso',zeros(p.Np,1), ...
    'indiceDinamicoPorPaso',zeros(p.Np,1));
info.candidatos = struct( ...
    'controles',zeros(0,2), ...
    'factibles',false(0,1), ...
    'costesTotal',zeros(0,1), ...
    'costesMeta',zeros(0,1), ...
    'costesCamino',zeros(0,1), ...
    'costesVariacionControl',zeros(0,1), ...
    'costesEstaticos',zeros(0,1), ...
    'costesDinamicos',zeros(0,1), ...
    'distanciasMinimas',zeros(0,1), ...
    'numeroViolaciones',zeros(0,1), ...
    'colisionFisica',false(0,1));
info.horizontePasos = p.Np;
info.horizontePrediccion = p.Np*p.Ts;
info.numeroPrediccionesEstado = 0;
info.parametros = p;
info.formulaModelo = ...
    "x+=Ts*v*cos(theta); y+=Ts*v*sin(theta); theta+=Ts*w";
info.formulaCosteMeta = ...
    "factorMeta*Qmeta*sum(||p_h-meta||^2)";
info.formulaCosteCamino = ...
    "Qcamino*sum(||p_h-objetivoLocal||^2)";
info.formulaCosteControl = ...
    "Np*RdeltaU*||u-controlAnterior||^2";
info.formulaCosteObstaculo = ...
    "Qobstaculo/(distanciaLibre^2+epsilon) o penalizacion";
end

%% ========================================================================
% VALIDACION
% ========================================================================

function [estado,objetivo,meta,estaticos,dinamicos,limites, ...
    controlAnterior,p] = validar_entradas( ...
        estado,objetivo,meta,estaticos,dinamicos,limites, ...
        robot,cfg,controlAnterior)
%VALIDAR_ENTRADAS Comprueba el contrato del controlador MPC.

estado = validar_vector(estado,3,'mpc:EstadoRobotNoValido', ...
    'estadoRobot debe ser [x y theta].');
estado(3) = wrap_to_pi_local(estado(3));
objetivo = validar_vector(objetivo,2,'mpc:ObjetivoLocalNoValido', ...
    'objetivoLocal debe ser [x y].');
meta = validar_vector(meta,2,'mpc:MetaNoValida', ...
    'meta debe ser [x y].');
limites = validar_vector(limites,4,'mpc:LimitesNoValidos', ...
    'limites debe ser [xmin xmax ymin ymax].');

if limites(2) <= limites(1) || limites(4) <= limites(3)
    error('mpc:OrdenLimitesNoValido', ...
        'Debe cumplirse xmin < xmax e ymin < ymax.');
end

if ~punto_en_limites(estado(1:2),limites)
    error('mpc:RobotFueraDelMapa', ...
        'La posicion actual del robot queda fuera del mapa.');
end
if ~punto_en_limites(objetivo,limites)
    error('mpc:ObjetivoFueraDelMapa', ...
        'El objetivo local queda fuera del mapa.');
end
if ~punto_en_limites(meta,limites)
    error('mpc:MetaFueraDelMapa', ...
        'La meta queda fuera del mapa.');
end

if isempty(estaticos)
    estaticos = zeros(0,4);
elseif ~isnumeric(estaticos) || ~isreal(estaticos) || ...
        size(estaticos,2) ~= 4 || any(~isfinite(estaticos(:)))
    error('mpc:EstaticosNoValidos', ...
        'obstaculosEstaticos debe ser una matriz N x 4.');
else
    estaticos = double(estaticos);
end
if ~isempty(estaticos) && any(estaticos(:,3:4) <= 0,'all')
    error('mpc:DimensionesEstaticosNoValidas', ...
        'El ancho y el alto de los obstaculos deben ser positivos.');
end

if isempty(dinamicos)
    dinamicos = struct('id',{},'pos',{},'vel',{},'radio',{});
elseif ~isstruct(dinamicos)
    error('mpc:DinamicosNoValidos', ...
        'obstaculosDinamicos debe ser un vector de estructuras.');
else
    for i = 1:numel(dinamicos)
        if ~all(isfield(dinamicos(i),{'pos','vel','radio'}))
            error('mpc:CampoDinamicoAusente', ...
                'Faltan campos pos, vel o radio en el obstaculo %d.',i);
        end
        dinamicos(i).pos = validar_vector( ...
            dinamicos(i).pos,2,'mpc:PosicionDinamicaNoValida', ...
            'La posicion dinamica debe ser [x y].');
        dinamicos(i).vel = validar_vector( ...
            dinamicos(i).vel,2,'mpc:VelocidadDinamicaNoValida', ...
            'La velocidad dinamica debe ser [vx vy].');
        if ~es_positivo(dinamicos(i).radio)
            error('mpc:RadioDinamicoNoValido', ...
                'El radio del obstaculo %d debe ser positivo.',i);
        end
        dinamicos(i).radio = double(dinamicos(i).radio);
    end
end

if ~isstruct(robot) || ~isscalar(robot) || ...
        ~all(isfield(robot,{'tipo','geometria','limites', ...
                           'controlInicial','controlParada'}))
    error('mpc:RobotNoValido', ...
        'robot debe proceder de configuracion_robot.m.');
end
if string(robot.tipo) ~= "uniciclo"
    error('mpc:ModeloNoSoportado', ...
        'Este MPC genera controles [v w] para un uniciclo.');
end
if ~isstruct(robot.geometria) || ...
        ~isfield(robot.geometria,'radio') || ...
        ~es_positivo(robot.geometria.radio)
    error('mpc:RadioRobotNoValido', ...
        'robot.geometria.radio debe ser positivo.');
end
if ~isstruct(robot.limites) || ...
        ~all(isfield(robot.limites,{'vMin','vMax','wMin','wMax'}))
    error('mpc:LimitesRobotNoValidos', ...
        'Faltan limites cinematicos del robot.');
end

vMin = robot.limites.vMin;
vMax = robot.limites.vMax;
wMin = robot.limites.wMin;
wMax = robot.limites.wMax;
if ~all(arrayfun(@es_finito,[vMin vMax wMin wMax])) || ...
        vMin < 0 || vMax <= vMin || wMin >= 0 || wMax <= 0 || wMax <= wMin
    error('mpc:LimitesRobotNoValidos', ...
        'Los limites cinematicos no son coherentes.');
end

controlInicial = validar_vector(robot.controlInicial,2, ...
    'mpc:ControlInicialNoValido','robot.controlInicial debe ser [v w].');
controlParada = validar_vector(robot.controlParada,2, ...
    'mpc:ControlParadaNoValido','robot.controlParada debe ser [v w].');
controlAnterior = validar_vector(controlAnterior,2, ...
    'mpc:ControlAnteriorNoValido','controlAnterior debe ser [v w].');

inferior = [vMin wMin];
superior = [vMax wMax];
tolControl = 1e-12*max(1,max(abs([controlAnterior inferior superior])));
if any(controlAnterior < inferior-tolControl) || ...
        any(controlAnterior > superior+tolControl)
    error('mpc:ControlAnteriorFueraLimites', ...
        'controlAnterior queda fuera de los limites cinematicos.');
end
controlAnterior = min(max(controlAnterior,inferior),superior);

if ~isstruct(cfg) || ~isscalar(cfg) || ...
        ~all(isfield(cfg,{'sim','navegacion','seguridad','prediccion','mpc'}))
    error('mpc:ConfiguracionNoValida', ...
        'cfg debe proceder de parametros_generales.m.');
end

p = struct();
p.Ts = campo_positivo(cfg.sim,'Ts','cfg.sim.Ts');
p.radioMeta = campo_no_negativo( ...
    cfg.navegacion,'radioMeta','cfg.navegacion.radioMeta');
p.tolWaypoint = campo_no_negativo( ...
    cfg.navegacion,'tolWaypoint','cfg.navegacion.tolWaypoint');
p.margenEstatico = campo_no_negativo( ...
    cfg.seguridad,'margenEstatico','cfg.seguridad.margenEstatico');
p.margenDinamico = campo_no_negativo( ...
    cfg.seguridad,'margenDinamico','cfg.seguridad.margenDinamico');

if ~isfield(cfg.prediccion,'modelo') || ...
        lower(strtrim(string(cfg.prediccion.modelo))) ~= "velocidad_constante"
    error('mpc:ModeloPrediccionNoSoportado', ...
        'cfg.prediccion.modelo debe ser "velocidad_constante".');
end
p.modeloPrediccion = "velocidad_constante";

p.Np = campo_entero_positivo(cfg.mpc,'Np','cfg.mpc.Np');
p.nVelocidades = campo_entero_positivo( ...
    cfg.mpc,'nVelocidades','cfg.mpc.nVelocidades');
p.nGiros = campo_entero_positivo( ...
    cfg.mpc,'nGiros','cfg.mpc.nGiros');
if p.nVelocidades < 2 || p.nGiros < 3
    error('mpc:RejillaInsuficiente', ...
        'Se requieren al menos 2 velocidades y 3 giros.');
end

p.Qmeta = campo_no_negativo(cfg.mpc,'Qmeta','cfg.mpc.Qmeta');
p.Qcamino = campo_no_negativo(cfg.mpc,'Qcamino','cfg.mpc.Qcamino');
p.RdeltaU = campo_no_negativo(cfg.mpc,'RdeltaU','cfg.mpc.RdeltaU');
p.Qobstaculo = campo_no_negativo( ...
    cfg.mpc,'Qobstaculo','cfg.mpc.Qobstaculo');
p.penalizacionColision = campo_positivo( ...
    cfg.mpc,'penalizacionColision','cfg.mpc.penalizacionColision');
p.umbralEstaticoConfigurado = campo_no_negativo( ...
    cfg.mpc,'umbralEstatico','cfg.mpc.umbralEstatico');
p.umbralDinamicoConfigurado = campo_no_negativo( ...
    cfg.mpc,'umbralDinamico','cfg.mpc.umbralDinamico');

if p.Qmeta == 0 && p.Qcamino == 0
    error('mpc:PesosSeguimientoNulos', ...
        'Qmeta y Qcamino no pueden ser ambos cero.');
end

p.factorMeta = campo_opcional_no_negativo(cfg.mpc,'factorMeta',0.25);
p.epsilonObstaculo = campo_opcional_positivo( ...
    cfg.mpc,'epsilonObstaculo',1e-3);
p.umbralEstatico = max(p.umbralEstaticoConfigurado,p.margenEstatico);
p.umbralDinamico = max(p.umbralDinamicoConfigurado,p.margenDinamico);
p.radioRobot = double(robot.geometria.radio);
p.vMin = double(vMin);
p.vMax = double(vMax);
p.wMin = double(wMin);
p.wMax = double(wMax);
p.controlInicial = controlInicial;
p.controlParada = controlParada;

escala = [estado(:);objetivo(:);meta(:);limites(:);estaticos(:); ...
    p.radioRobot;p.umbralEstatico;p.umbralDinamico];
for i = 1:numel(dinamicos)
    escala = [escala;dinamicos(i).pos(:);dinamicos(i).vel(:); ...
        dinamicos(i).radio]; %#ok<AGROW>
end
p.tolerancia = 1e-12*max(1,max(abs(escala)));
end

%% ========================================================================
% VALIDADORES AUXILIARES
% ========================================================================

function v = validar_vector(v,n,id,mensaje)
if ~isnumeric(v) || ~isreal(v) || numel(v) ~= n || any(~isfinite(v(:)))
    error(id,'%s',mensaje);
end
v = reshape(double(v),1,n);
end

function tf = punto_en_limites(q,limites)
tf = q(1) >= limites(1) && q(1) <= limites(2) && ...
     q(2) >= limites(3) && q(2) <= limites(4);
end

function id = id_dinamico(obstaculo,indice)
if isfield(obstaculo,'id') && ...
        strlength(strtrim(string(obstaculo.id))) > 0
    id = strtrim(string(obstaculo.id));
else
    id = "D"+indice;
end
end

function valor = campo_positivo(s,campo,nombre)
if ~isstruct(s) || ~isfield(s,campo) || ~es_positivo(s.(campo))
    error('mpc:EscalarPositivoNoValido', ...
        '%s debe ser un escalar positivo.',nombre);
end
valor = double(s.(campo));
end

function valor = campo_no_negativo(s,campo,nombre)
if ~isstruct(s) || ~isfield(s,campo) || ~es_no_negativo(s.(campo))
    error('mpc:EscalarNoNegativoNoValido', ...
        '%s debe ser un escalar no negativo.',nombre);
end
valor = double(s.(campo));
end

function valor = campo_entero_positivo(s,campo,nombre)
valor = campo_positivo(s,campo,nombre);
if valor ~= floor(valor)
    error('mpc:EnteroPositivoNoValido', ...
        '%s debe ser entero.',nombre);
end
end

function valor = campo_opcional_no_negativo(s,campo,defecto)
if ~isfield(s,campo) || isempty(s.(campo))
    valor = defecto;
elseif es_no_negativo(s.(campo))
    valor = double(s.(campo));
else
    error('mpc:CampoOpcionalNoValido', ...
        'cfg.mpc.%s debe ser no negativo.',campo);
end
end

function valor = campo_opcional_positivo(s,campo,defecto)
if ~isfield(s,campo) || isempty(s.(campo))
    valor = defecto;
elseif es_positivo(s.(campo))
    valor = double(s.(campo));
else
    error('mpc:CampoOpcionalNoValido', ...
        'cfg.mpc.%s debe ser positivo.',campo);
end
end

function tf = es_positivo(x)
tf = isnumeric(x) && isreal(x) && isscalar(x) && isfinite(x) && x > 0;
end

function tf = es_no_negativo(x)
tf = isnumeric(x) && isreal(x) && isscalar(x) && isfinite(x) && x >= 0;
end

function tf = es_finito(x)
tf = isnumeric(x) && isreal(x) && isscalar(x) && isfinite(x);
end
