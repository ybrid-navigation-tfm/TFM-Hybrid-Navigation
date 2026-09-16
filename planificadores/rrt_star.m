function [camino, arbol, info] = rrt_star( ...
    estadoInicial, meta, limites, obstaculosEstaticos, ...
    obstaculosDinamicos, robot, cfg)
% RRT_STAR Calcula un camino global mediante RRT* convencional.
%
%   camino = RRT_STAR( ...
%       estadoInicial,meta,limites,obstaculosEstaticos, ...
%       obstaculosDinamicos,robot,cfg)
%
%   [camino,arbol,info] = RRT_STAR(...)
%
%   Implementa un planificador global RRT* bidimensional comun para:
%
%       - RRT* + APF
%       - RRT* + MPC
%
%   El crecimiento del arbol es independiente del controlador local. APF
%   no sesga el muestreo, no altera el coste de las aristas y no interviene
%   en el rewiring. Asi, ambas arquitecturas emplean exactamente el mismo
%   planificador global y pueden compararse de forma controlada.
%
%   Entradas:
%       estadoInicial
%           Estado actual [x y] o [x y theta]. Solo se usan x e y.
%
%       meta
%           Posicion objetivo [x y].
%
%       limites
%           Limites del mapa [xmin xmax ymin ymax].
%
%       obstaculosEstaticos
%           Matriz N x 4 con rectangulos [x y ancho alto].
%
%       obstaculosDinamicos
%           Vector de estructuras con los campos:
%
%               pos      posicion [x y]                       [m]
%               vel      velocidad [vx vy]                    [m/s]
%               radio    radio fisico                         [m]
%
%       robot
%           Estructura obtenida mediante configuracion_robot.m.
%
%       cfg
%           Estructura obtenida mediante parametros_generales.m. Se usan:
%
%               cfg.sim.Ts
%               cfg.navegacion.radioMeta
%               cfg.seguridad.margenEstatico
%               cfg.seguridad.margenDinamico
%               cfg.prediccion.pasos
%               cfg.prediccion.modelo
%               cfg.rrt.maxIter
%               cfg.rrt.paso
%               cfg.rrt.radioVecinos
%               cfg.rrt.sesgoMeta
%
%   Salidas:
%       camino
%           Camino K x 2 desde el robot hasta la meta. Si no se encuentra
%           una solucion completa, devuelve zeros(0,2). No se devuelve una
%           rama parcial como si fuera una planificacion valida.
%
%       arbol
%           Estructura matricial:
%
%               .tipo
%               .nodos                 N x 2
%               .padres                N x 1, raiz con padre 0
%               .costes                N x 1
%               .aristas               M x 2 [padre hijo]
%               .indiceRaiz
%               .indiceMeta
%               .indicesCandidatosMeta
%               .numeroNodos
%               .numeroAristas
%
%       info
%           Diagnostico de la llamada:
%
%               .exito
%               .motivo
%               .iteracionesSolicitadas
%               .iteracionesEjecutadas
%               .iteracionPrimeraSolucion
%               .numeroNodos
%               .numeroAristas
%               .numeroRewirings
%               .numeroCandidatosMeta
%               .numeroMuestrasMeta
%               .numeroMuestrasUniformes
%               .numeroRechazosDireccionNula
%               .numeroRechazosDuplicado
%               .numeroRechazosColision
%               .costeCamino
%               .longitudCamino
%               .distanciaFinalMeta
%               .horizontePrediccion
%               .parametros
%               .vistaPlanificador
%               .metaExactaLibre
%               .inicioDentroEnvolventePredictiva
%               .numeroAristasEscapePredictivo
%
%   Seguridad geometrica:
%       El planificador global evita siempre limites y obstaculos
%       estaticos. Por defecto no invalida nodos, aristas ni la meta por
%       obstaculos dinamicos; estos se delegan al controlador local.
%
%       Los rectangulos estaticos se comprueban con la separacion:
%
%           radioRobot + cfg.seguridad.margenEstatico
%
%       Para cada obstaculo dinamico se considera su segmento barrido:
%
%           pFutura = pActual + ...
%               cfg.prediccion.pasos*cfg.sim.Ts*velocidad
%
%       y se exige la separacion:
%
%           radioRobot + radioObstaculo + ...
%               cfg.seguridad.margenDinamico
%
%       Este criterio coincide con replanificacion.m.
%
%       Si la raiz esta fisicamente libre pero queda dentro de una
%       envolvente predictiva dinamica, se permite exclusivamente una
%       expansion de escape que aumente de forma monotona la separacion.
%       No se permite entrar desde fuera en esas envolventes. El muestreo,
%       la seleccion de padre y el rewiring de RRT* permanecen invariantes.
%
%   RRT* convencional:
%       1. comprueba primero la conexion recta trivial;
%       2. muestrea uniformemente con sesgo ocasional hacia la meta;
%       3. selecciona el nodo mas cercano y aplica steering;
%       4. elige el padre seguro de menor coste entre los vecinos;
%       5. reconecta vecinos cuando reduce su coste;
%       6. propaga el cambio de coste a todos sus descendientes;
%       7. continua hasta maxIter para mejorar la primera solucion;
%       8. selecciona al final la conexion a meta de menor coste.
%
%   Reproducibilidad:
%       Esta funcion no llama a rng. La semilla se fija en el main:
%
%           rng(cfg.semilla,'twister');
%
%   Ejemplo:
%
%       cfg = parametros_generales("visual");
%       escenario = escenarios("baja");
%       robot = configuracion_robot();
%
%       rng(cfg.semilla,'twister');
%
%       [camino,arbol,info] = rrt_star( ...
%           escenario.inicio, ...
%           escenario.meta, ...
%           escenario.limites, ...
%           escenario.obstaculosEstaticos, ...
%           escenario.obstaculosDinamicos, ...
%           robot, ...
%           cfg);
%
%       vistaPlanificador = info.vistaPlanificador;
%
%   Esta funcion utiliza:
%       - distancia_punto_segmento.m
%       - distancia_segmentos.m
%       - segmento_rectangulo.m
%
%   El tiempo se mide externamente con tic/toc para tiempos.m.

%% Validacion y preparacion
[inicio, meta, limites, estaticos, dinamicos, parametros] = ...
    validar_entradas( ...
        estadoInicial,meta,limites,obstaculosEstaticos, ...
        obstaculosDinamicos,robot,cfg);

geometria = preparar_geometria( ...
    inicio,meta,limites,estaticos,dinamicos,parametros);

%% Memoria preasignada
maximoNodos = parametros.maxIter+2;

nodos = nan(maximoNodos,2);
padres = zeros(maximoNodos,1);
costes = inf(maximoNodos,1);
candidatoMeta = false(maximoNodos,1);

nodos(1,:) = inicio;
padres(1) = 0;
costes(1) = 0;
numeroNodos = 1;

%% Contadores
estadistica = struct();
estadistica.iteracionesEjecutadas = 0;
estadistica.iteracionPrimeraSolucion = NaN;
estadistica.numeroRewirings = 0;
estadistica.numeroMuestrasMeta = 0;
estadistica.numeroMuestrasUniformes = 0;
estadistica.numeroRechazosDireccionNula = 0;
estadistica.numeroRechazosDuplicado = 0;
estadistica.numeroRechazosColision = 0;
estadistica.numeroAristasEscapePredictivo = 0;

camino = zeros(0,2);
indiceMeta = NaN;
motivo = "sin_camino_hasta_meta";

%% Inicio fisicamente admisible y meta predictivamente libre
[inicioAdmisible,tipoInicio,indiceInicio, ...
    inicioDentroEnvolventePredictiva] = ...
        punto_inicio_admisible(inicio,geometria);

geometria.inicioDentroEnvolventePredictiva = ...
    inicioDentroEnvolventePredictiva;

if ~inicioAdmisible
    motivo = motivo_punto_no_libre("inicio",tipoInicio,indiceInicio);
    [arbol,info] = construir_salidas( ...
        false,motivo,camino,nodos,padres,costes,candidatoMeta, ...
        numeroNodos,indiceMeta,estadistica,parametros,geometria);
    return;
end

[libreMeta,tipoMeta,indiceBloqueoMeta] = ...
    punto_libre(meta,geometria);

% La condicion experimental de llegada utiliza una REGION de meta con
% radio cfg.navegacion.radioMeta. Si el centro exacto esta ocupado
% temporalmente por un obstaculo dinamico, RRT* puede seguir buscando un
% punto seguro dentro de dicha region. Solo se aborta cuando la meta es
% incompatible con limites u obstaculos estaticos.
metaExactaLibre = libreMeta;
geometria.metaExactaLibre = metaExactaLibre;

if ~libreMeta && tipoMeta ~= "dinamico"
    motivo = motivo_punto_no_libre("meta",tipoMeta,indiceBloqueoMeta);
    [arbol,info] = construir_salidas( ...
        false,motivo,camino,nodos,padres,costes,candidatoMeta, ...
        numeroNodos,indiceMeta,estadistica,parametros,geometria);
    return;
end

%% El robot ya se encuentra en la meta
if norm(inicio-meta) <= geometria.toleranciaPosicion
    camino = inicio;
    indiceMeta = 1;
    candidatoMeta(1) = true;
    motivo = "inicio_en_meta";
    estadistica.iteracionPrimeraSolucion = 0;

    [arbol,info] = construir_salidas( ...
        true,motivo,camino,nodos,padres,costes,candidatoMeta, ...
        numeroNodos,indiceMeta,estadistica,parametros,geometria);
    return;
end

%% Solucion recta trivial y globalmente minima
if metaExactaLibre && arista_libre(inicio,meta,geometria)
    numeroNodos = 2;
    indiceMeta = 2;

    nodos(indiceMeta,:) = meta;
    padres(indiceMeta) = 1;
    costes(indiceMeta) = norm(meta-inicio);
    candidatoMeta(indiceMeta) = true;

    camino = nodos(1:2,:);
    motivo = "conexion_directa";
    estadistica.iteracionPrimeraSolucion = 0;

    [arbol,info] = construir_salidas( ...
        true,motivo,camino,nodos,padres,costes,candidatoMeta, ...
        numeroNodos,indiceMeta,estadistica,parametros,geometria);
    return;
end

%% ========================================================================
% CRECIMIENTO DEL RRT*
% ========================================================================

radioVecinos2 = parametros.radioVecinos^2;
toleranciaPosicion2 = geometria.toleranciaPosicion^2;

for iteracion = 1:parametros.maxIter
    estadistica.iteracionesEjecutadas = iteracion;

    %% Muestreo
    if rand < parametros.sesgoMeta
        muestra = muestrear_objetivo( ...
            meta,parametros.radioMeta,geometria,metaExactaLibre);
        estadistica.numeroMuestrasMeta = ...
            estadistica.numeroMuestrasMeta+1;
    else
        muestra = muestrear_uniforme(geometria.limitesSeguros);
        estadistica.numeroMuestrasUniformes = ...
            estadistica.numeroMuestrasUniformes+1;
    end

    %% Nodo mas cercano
    nodosActuales = nodos(1:numeroNodos,:);
    diferenciasMuestra = nodosActuales-muestra;
    distancias2Muestra = sum(diferenciasMuestra.^2,2);

    [distancia2Minima,indiceMasCercano] = min(distancias2Muestra);
    distanciaMasCercana = sqrt(distancia2Minima);

    if distanciaMasCercana <= geometria.toleranciaPosicion
        estadistica.numeroRechazosDireccionNula = ...
            estadistica.numeroRechazosDireccionNula+1;
        continue;
    end

    %% Steering
    nodoCercano = nodos(indiceMasCercano,:);
    avance = min(parametros.paso,distanciaMasCercana);
    direccion = (muestra-nodoCercano)/distanciaMasCercana;
    nodoNuevo = nodoCercano+avance*direccion;

    diferenciasNuevo = nodosActuales-nodoNuevo;
    distancias2Nuevo = sum(diferenciasNuevo.^2,2);

    if any(distancias2Nuevo <= toleranciaPosicion2)
        estadistica.numeroRechazosDuplicado = ...
            estadistica.numeroRechazosDuplicado+1;
        continue;
    end

    %% La expansion basica debe ser segura
    [expansionLibre,~] = arista_libre( ...
        nodoCercano,nodoNuevo,geometria,true);

    if ~expansionLibre
        estadistica.numeroRechazosColision = ...
            estadistica.numeroRechazosColision+1;
        continue;
    end

    %% Vecinos RRT*
    indicesVecinos = find(distancias2Nuevo <= radioVecinos2);

    if ~any(indicesVecinos == indiceMasCercano)
        indicesVecinos(end+1,1) = indiceMasCercano; %#ok<AGROW>
    end

    indicesVecinos = unique(indicesVecinos,'stable');
    distanciasVecinos = sqrt(distancias2Nuevo(indicesVecinos));
    costesCandidatos = costes(indicesVecinos)+distanciasVecinos;

    [~,ordenCandidatos] = sort(costesCandidatos,'ascend');

    %% Padre seguro de menor coste
    mejorPadre = 0;
    mejorCoste = inf;
    mejorPadreUsaEscape = false;

    for posicion = 1:numel(ordenCandidatos)
        posicionVecino = ordenCandidatos(posicion);
        indiceVecino = indicesVecinos(posicionVecino);

        [aristaPadreLibre,usaEscapeCandidato] = ...
            arista_libre( ...
                nodos(indiceVecino,:),nodoNuevo,geometria,true);

        if aristaPadreLibre
            mejorPadre = indiceVecino;
            mejorCoste = costesCandidatos(posicionVecino);
            mejorPadreUsaEscape = usaEscapeCandidato;
            break;
        end
    end

    if mejorPadre == 0
        estadistica.numeroRechazosColision = ...
            estadistica.numeroRechazosColision+1;
        continue;
    end

    %% Insercion
    numeroNodos = numeroNodos+1;
    indiceNuevo = numeroNodos;

    nodos(indiceNuevo,:) = nodoNuevo;
    padres(indiceNuevo) = mejorPadre;
    costes(indiceNuevo) = mejorCoste;

    if mejorPadreUsaEscape
        estadistica.numeroAristasEscapePredictivo = ...
            estadistica.numeroAristasEscapePredictivo+1;
    end

    %% Rewiring
    for posicion = 1:numel(indicesVecinos)
        indiceVecino = indicesVecinos(posicion);

        if indiceVecino == 1 || indiceVecino == mejorPadre
            continue;
        end

        distanciaRewire = sqrt(distancias2Nuevo(indiceVecino));
        costeRewire = mejorCoste+distanciaRewire;

        if costeRewire >= ...
                costes(indiceVecino)-geometria.toleranciaCoste
            continue;
        end

        if es_ancestro(indiceVecino,indiceNuevo,padres,numeroNodos)
            continue;
        end

        [aristaRewireLibre,usaEscapeRewire] = ...
            arista_libre( ...
                nodoNuevo,nodos(indiceVecino,:),geometria,true);

        if ~aristaRewireLibre
            continue;
        end

        padres(indiceVecino) = indiceNuevo;
        costes(indiceVecino) = costeRewire;

        costes = propagar_costes_descendientes( ...
            indiceVecino,nodos,padres,costes,numeroNodos);

        estadistica.numeroRewirings = ...
            estadistica.numeroRewirings+1;

        if usaEscapeRewire
            estadistica.numeroAristasEscapePredictivo = ...
                estadistica.numeroAristasEscapePredictivo+1;
        end
    end

    %% Primera solucion factible
    distanciaMeta = norm(nodoNuevo-meta);

    if distanciaMeta <= ...
            parametros.radioMeta+geometria.tolerancia

        [nodoMetaLibre,~,~] = punto_libre(nodoNuevo,geometria);

        % Los nodos utilizados como final deben quedar fuera de todas las
        % envolventes predictivas. Las excepciones de escape solo sirven
        % para abandonar una zona de riesgo, nunca para terminar dentro.
        if nodoMetaLibre
            candidatoMeta(indiceNuevo) = true;

            if isnan(estadistica.iteracionPrimeraSolucion)
                estadistica.iteracionPrimeraSolucion = iteracion;
            end
        end
    end
end

%% ========================================================================
% MEJOR CONEXION A META
% ========================================================================

% Se vuelve a comprobar toda la region de meta. Esto hace que la salida no
% dependa de unicamente haber marcado el candidato durante su insercion.
distanciasMeta = hypot( ...
    nodos(1:numeroNodos,1)-meta(1), ...
    nodos(1:numeroNodos,2)-meta(2));

indicesRegionMeta = find( ...
    distanciasMeta <= parametros.radioMeta+geometria.tolerancia);

candidatoMeta(1:numeroNodos) = false;

for posicion = 1:numel(indicesRegionMeta)
    indiceCandidato = indicesRegionMeta(posicion);
    [candidatoLibre,~,~] = ...
        punto_libre(nodos(indiceCandidato,:),geometria);

    if candidatoLibre
        candidatoMeta(indiceCandidato) = true;
    end
end

indicesCandidatos = find(candidatoMeta(1:numeroNodos));

if isempty(indicesCandidatos) && ~metaExactaLibre
    motivo = "sin_camino_hasta_region_meta";
end

if ~isempty(indicesCandidatos)
    % Se minimiza el coste acumulado con una pequena preferencia por
    % terminar mas cerca del centro de la region objetivo.
    costesHastaMeta = costes(indicesCandidatos)+ ...
        distanciasMeta(indicesCandidatos);

    [~,posicionMejor] = min(costesHastaMeta);
    mejorCandidato = indicesCandidatos(posicionMejor);
    distanciaFinal = distanciasMeta(mejorCandidato);

    % Si el centro exacto esta libre y la ultima conexion es segura, se
    % conserva el final exacto. En caso contrario, el camino termina en el
    % mejor nodo seguro dentro de la region de meta, lo cual es coherente
    % con el criterio de exito norm(pos-meta) <= radioMeta.
    if distanciaFinal <= geometria.toleranciaPosicion
        indiceMeta = mejorCandidato;
        nodos(indiceMeta,:) = meta;
        motivo = "camino_encontrado_meta_exacta";
    elseif metaExactaLibre && ...
            arista_libre(nodos(mejorCandidato,:),meta,geometria)
        numeroNodos = numeroNodos+1;
        indiceMeta = numeroNodos;

        nodos(indiceMeta,:) = meta;
        padres(indiceMeta) = mejorCandidato;
        costes(indiceMeta) = costes(mejorCandidato)+distanciaFinal;
        motivo = "camino_encontrado_meta_exacta";
    else
        indiceMeta = mejorCandidato;
        motivo = "camino_encontrado_region_meta";
    end

    camino = reconstruir_desde_padres( ...
        nodos,padres,indiceMeta,numeroNodos);

    if isnan(estadistica.iteracionPrimeraSolucion)
        estadistica.iteracionPrimeraSolucion = ...
            estadistica.iteracionesEjecutadas;
    end
end

exito = ~isempty(camino) && isfinite(indiceMeta);

[arbol,info] = construir_salidas( ...
    exito,motivo,camino,nodos,padres,costes,candidatoMeta, ...
    numeroNodos,indiceMeta,estadistica,parametros,geometria);
end

%% ========================================================================
% GEOMETRIA
% ========================================================================

function geometria = preparar_geometria( ...
    inicio,meta,limites,estaticos,dinamicos,parametros)
%PREPARAR_GEOMETRIA Precalcula margenes y barridos dinamicos.

geometria = struct();
geometria.inicio = inicio;
geometria.meta = meta;
geometria.limites = limites;
geometria.estaticos = estaticos;
geometria.metaExactaLibre = false;
geometria.inicioDentroEnvolventePredictiva = false;

geometria.separacionEstatica = ...
    parametros.radioRobot+parametros.margenEstatico;

if parametros.considerarDinamicos
    geometria.horizontePrediccion = ...
        parametros.pasosPrediccion*parametros.Ts;
else
    geometria.horizontePrediccion = 0;
end

geometria.limitesSeguros = [ ...
    limites(1)+geometria.separacionEstatica, ...
    limites(2)-geometria.separacionEstatica, ...
    limites(3)+geometria.separacionEstatica, ...
    limites(4)-geometria.separacionEstatica];

if parametros.considerarDinamicos
    numeroDinamicos = numel(dinamicos);
else
    % Los dinamicos no forman parte de la geometria global. La firma de la
    % funcion se conserva para que RRT*+APF y RRT*+MPC compartan modulo.
    numeroDinamicos = 0;
end

geometria.posicionesDinamicasActuales = zeros(numeroDinamicos,2);
geometria.posicionesDinamicasFuturas = zeros(numeroDinamicos,2);
geometria.separacionesDinamicas = zeros(numeroDinamicos,1);
geometria.separacionesDinamicasFisicas = ...
    zeros(numeroDinamicos,1);

for i = 1:numeroDinamicos
    posicionActual = dinamicos(i).pos;
    posicionFutura = posicionActual+ ...
        geometria.horizontePrediccion*dinamicos(i).vel;

    geometria.posicionesDinamicasActuales(i,:) = posicionActual;
    geometria.posicionesDinamicasFuturas(i,:) = posicionFutura;
    geometria.separacionesDinamicas(i) = ...
        parametros.radioRobot+dinamicos(i).radio+ ...
        parametros.margenDinamico;

    geometria.separacionesDinamicasFisicas(i) = ...
        parametros.radioRobot+dinamicos(i).radio;
end

valoresEscala = [ ...
    inicio(:);meta(:);limites(:);estaticos(:); ...
    geometria.posicionesDinamicasActuales(:); ...
    geometria.posicionesDinamicasFuturas(:)];

escala = max(1,max(abs(valoresEscala)));

geometria.tolerancia = 1e-12*escala;
geometria.toleranciaPosicion = 1e-10*escala;
geometria.toleranciaCoste = 1e-12*escala;
end

function muestra = muestrear_uniforme(limitesSeguros)
%MUESTREAR_UNIFORME Genera un punto uniforme en el dominio seguro.

muestra = [ ...
    limitesSeguros(1)+rand*(limitesSeguros(2)-limitesSeguros(1)), ...
    limitesSeguros(3)+rand*(limitesSeguros(4)-limitesSeguros(3))];
end

function muestra = muestrear_objetivo( ...
    meta,radioMeta,geometria,metaExactaLibre)
%MUESTREAR_OBJETIVO Sesgo al centro o a un punto libre de su region.

if metaExactaLibre
    muestra = meta;
    return;
end

L = geometria.limitesSeguros;
muestra = meta;

% Cuando el centro exacto esta ocupado por un dinamico, insistir siempre
% en ese mismo punto desperdicia el sesgo a meta. Se intenta un punto
% reproducible y aleatorio dentro de la region experimental de llegada.
for intento = 1:16
    angulo = 2*pi*rand;
    radio = radioMeta*sqrt(rand);
    candidato = meta+radio*[cos(angulo) sin(angulo)];

    dentroLimites = ...
        candidato(1) >= L(1) && candidato(1) <= L(2) && ...
        candidato(2) >= L(3) && candidato(2) <= L(4);

    if ~dentroLimites
        continue;
    end

    [libre,~,~] = punto_libre(candidato,geometria);

    if libre
        muestra = candidato;
        return;
    end

    muestra = candidato;
end
end

function [admisible,tipo,indice,dentroPredictiva] = ...
    punto_inicio_admisible(punto,geometria)
%PUNTO_INICIO_ADMISIBLE Distingue colision fisica y riesgo predictivo.

admisible = false;
tipo = "ninguno";
indice = NaN;
dentroPredictiva = false;
tol = geometria.tolerancia;
L = geometria.limitesSeguros;

if punto(1) < L(1)-tol || punto(1) > L(2)+tol || ...
        punto(2) < L(3)-tol || punto(2) > L(4)+tol
    tipo = "limite";
    return;
end

for i = 1:size(geometria.estaticos,1)
    distancia = distancia_punto_rectangulo_local( ...
        punto,geometria.estaticos(i,:));

    if distancia <= geometria.separacionEstatica+tol
        tipo = "estatico";
        indice = i;
        return;
    end
end

for i = 1:size(geometria.posicionesDinamicasActuales,1)
    distanciaActual = norm( ...
        punto-geometria.posicionesDinamicasActuales(i,:));

    if distanciaActual <= ...
            geometria.separacionesDinamicasFisicas(i)+tol
        tipo = "dinamico_fisico";
        indice = i;
        return;
    end

    distanciaBarrido = distancia_punto_segmento( ...
        punto, ...
        geometria.posicionesDinamicasActuales(i,:), ...
        geometria.posicionesDinamicasFuturas(i,:));

    if distanciaBarrido <= ...
            geometria.separacionesDinamicas(i)+tol
        dentroPredictiva = true;
    end
end

admisible = true;
end

function [libre,tipo,indice] = punto_libre(punto,geometria)
%PUNTO_LIBRE Comprueba limites, rectangulos y barridos dinamicos.

libre = false;
tipo = "ninguno";
indice = NaN;
tol = geometria.tolerancia;
L = geometria.limitesSeguros;

if punto(1) < L(1)-tol || punto(1) > L(2)+tol || ...
        punto(2) < L(3)-tol || punto(2) > L(4)+tol
    tipo = "limite";
    return;
end

for i = 1:size(geometria.estaticos,1)
    distancia = distancia_punto_rectangulo_local( ...
        punto,geometria.estaticos(i,:));

    if distancia <= geometria.separacionEstatica+tol
        tipo = "estatico";
        indice = i;
        return;
    end
end

for i = 1:size(geometria.posicionesDinamicasActuales,1)
    distancia = distancia_punto_segmento( ...
        punto, ...
        geometria.posicionesDinamicasActuales(i,:), ...
        geometria.posicionesDinamicasFuturas(i,:));

    if distancia <= geometria.separacionesDinamicas(i)+tol
        tipo = "dinamico";
        indice = i;
        return;
    end
end

libre = true;
end

function [libre,usoEscape] = arista_libre( ...
    a,b,geometria,permitirEscapePredictivo)
%ARISTA_LIBRE Comprueba la seguridad completa de un segmento dirigido.
%
% Si el extremo inicial se encuentra dentro de una envolvente predictiva
% dinamica, permitirEscapePredictivo autoriza solo un movimiento de salida:
% la distancia al barrido debe crecer y la arista no puede cortar el disco
% fisico actual del obstaculo. Una arista que comienza fuera nunca puede
% entrar en la envolvente.

if nargin < 4 || isempty(permitirEscapePredictivo)
    permitirEscapePredictivo = false;
end

libre = false;
usoEscape = false;
tol = geometria.tolerancia;
L = geometria.limitesSeguros;

if a(1) < L(1)-tol || a(1) > L(2)+tol || ...
        a(2) < L(3)-tol || a(2) > L(4)+tol || ...
        b(1) < L(1)-tol || b(1) > L(2)+tol || ...
        b(2) < L(3)-tol || b(2) > L(4)+tol
    return;
end

for i = 1:size(geometria.estaticos,1)
    rectangulo = geometria.estaticos(i,:);
    separacion = geometria.separacionEstatica;

    if cajas_separadas_segmento_rectangulo( ...
            a,b,rectangulo,separacion+tol)
        continue;
    end

    distancia = distancia_segmento_rectangulo( ...
        a,b,rectangulo,tol);

    if distancia <= separacion+tol
        return;
    end
end

for i = 1:size(geometria.posicionesDinamicasActuales,1)
    posicionActual = geometria.posicionesDinamicasActuales(i,:);
    posicionFutura = geometria.posicionesDinamicasFuturas(i,:);
    separacion = geometria.separacionesDinamicas(i);

    if cajas_separadas_segmentos( ...
            a,b,posicionActual,posicionFutura,separacion+tol)
        continue;
    end

    distancia = distancia_segmentos( ...
        a,b,posicionActual,posicionFutura);

    if distancia > separacion+tol
        continue;
    end

    if ~permitirEscapePredictivo || ...
            ~arista_escape_predictivo_valida( ...
                a,b,i,geometria)
        return;
    end

    usoEscape = true;
end

libre = true;
end

function valida = arista_escape_predictivo_valida( ...
    a,b,indiceDinamico,geometria)
%ARISTA_ESCAPE_PREDICTIVO_VALIDA Autoriza solo una salida progresiva.

valida = false;
tol = geometria.tolerancia;
p0 = geometria.posicionesDinamicasActuales(indiceDinamico,:);
p1 = geometria.posicionesDinamicasFuturas(indiceDinamico,:);
separacionPredictiva = ...
    geometria.separacionesDinamicas(indiceDinamico);
separacionFisica = ...
    geometria.separacionesDinamicasFisicas(indiceDinamico);

% La arista no puede atravesar la ocupacion fisica actual.
distanciaFisicaActual = distancia_punto_segmento(p0,a,b);

if distanciaFisicaActual <= separacionFisica+tol
    return;
end

libreA = distancia_punto_segmento(a,p0,p1)-separacionPredictiva;
libreB = distancia_punto_segmento(b,p0,p1)-separacionPredictiva;

% Solo se aplica a una arista cuyo origen ya esta dentro de la envolvente.
if libreA > tol
    return;
end

longitudArista = norm(b-a);
progresoMinimo = max(100*tol,0.03*longitudArista);
retrocesoPermitido = max(100*tol,0.01*longitudArista);

if libreB <= libreA+progresoMinimo
    return;
end

fracciones = linspace(0,1,7).';
puntos = a+fracciones.*(b-a);
libres = zeros(size(fracciones));

for j = 1:numel(fracciones)
    libres(j) = distancia_punto_segmento( ...
        puntos(j,:),p0,p1)-separacionPredictiva;
end

if min(libres) < libreA-retrocesoPermitido
    return;
end

valida = true;
end

function separadas = cajas_separadas_segmento_rectangulo( ...
    a,b,rectangulo,margen)
%CAJAS_SEPARADAS_SEGMENTO_RECTANGULO Filtro AABB rapido.

xminS = min(a(1),b(1));
xmaxS = max(a(1),b(1));
yminS = min(a(2),b(2));
ymaxS = max(a(2),b(2));

xminR = rectangulo(1);
yminR = rectangulo(2);
xmaxR = rectangulo(1)+rectangulo(3);
ymaxR = rectangulo(2)+rectangulo(4);

separadas = ...
    xmaxS+margen < xminR || xminS-margen > xmaxR || ...
    ymaxS+margen < yminR || yminS-margen > ymaxR;
end

function separadas = cajas_separadas_segmentos(a,b,c,d,margen)
%CAJAS_SEPARADAS_SEGMENTOS Filtro AABB para dos segmentos.

separadas = ...
    max(a(1),b(1))+margen < min(c(1),d(1)) || ...
    min(a(1),b(1))-margen > max(c(1),d(1)) || ...
    max(a(2),b(2))+margen < min(c(2),d(2)) || ...
    min(a(2),b(2))-margen > max(c(2),d(2));
end

function d = distancia_punto_rectangulo_local(punto,rectangulo)
%DISTANCIA_PUNTO_RECTANGULO_LOCAL Distancia de punto a rectangulo.

xmin = rectangulo(1);
ymin = rectangulo(2);
xmax = xmin+rectangulo(3);
ymax = ymin+rectangulo(4);

dx = max([xmin-punto(1),0,punto(1)-xmax]);
dy = max([ymin-punto(2),0,punto(2)-ymax]);
d = hypot(dx,dy);
end

function d = distancia_segmento_rectangulo(a,b,rectangulo,tol)
%DISTANCIA_SEGMENTO_RECTANGULO Distancia exacta segmento-rectangulo.

if segmento_rectangulo(a,b,rectangulo,tol)
    d = 0;
    return;
end

xmin = rectangulo(1);
ymin = rectangulo(2);
xmax = xmin+rectangulo(3);
ymax = ymin+rectangulo(4);

esquinas = [ ...
    xmin ymin; ...
    xmax ymin; ...
    xmax ymax; ...
    xmin ymax];

lados = [1 2;2 3;3 4;4 1];
d = inf;

for i = 1:4
    c = esquinas(lados(i,1),:);
    f = esquinas(lados(i,2),:);
    d = min(d,distancia_segmentos(a,b,c,f));
end
end

%% ========================================================================
% ARBOL Y SALIDAS
% ========================================================================

function tf = es_ancestro( ...
    posibleAncestro,indiceNodo,padres,numeroNodos)
%ES_ANCESTRO Evita ciclos durante el rewiring.

tf = false;
indiceActual = indiceNodo;
contador = 0;

while indiceActual > 0
    contador = contador+1;

    if contador > numeroNodos
        error('rrt_star:CicloDetectado', ...
            'La estructura de padres contiene un ciclo.');
    end

    indiceActual = padres(indiceActual);

    if indiceActual == posibleAncestro
        tf = true;
        return;
    end
end
end

function costes = propagar_costes_descendientes( ...
    indiceInicial,nodos,padres,costes,numeroNodos)
%PROPAGAR_COSTES_DESCENDIENTES Recalcula el subarbol reparentado.

cola = zeros(numeroNodos,1);
inicioCola = 1;
finCola = 1;
cola(1) = indiceInicial;

while inicioCola <= finCola
    indicePadre = cola(inicioCola);
    inicioCola = inicioCola+1;

    hijos = find(padres(1:numeroNodos) == indicePadre);

    for k = 1:numel(hijos)
        indiceHijo = hijos(k);

        costes(indiceHijo) = costes(indicePadre)+ ...
            norm(nodos(indiceHijo,:)-nodos(indicePadre,:));

        finCola = finCola+1;
        cola(finCola) = indiceHijo;
    end
end
end

function camino = reconstruir_desde_padres( ...
    nodos,padres,indiceFinal,numeroNodos)
%RECONSTRUIR_DESDE_PADRES Recupera la rama raiz-meta.

indices = zeros(numeroNodos,1);
numeroIndices = 0;
indiceActual = indiceFinal;

while indiceActual > 0
    numeroIndices = numeroIndices+1;

    if numeroIndices > numeroNodos
        error('rrt_star:CicloDuranteReconstruccion', ...
            'La cadena de padres contiene un ciclo.');
    end

    indices(numeroIndices) = indiceActual;
    indiceActual = padres(indiceActual);
end

indices = flipud(indices(1:numeroIndices));
camino = nodos(indices,:);
end

function [arbol,info] = construir_salidas( ...
    exito,motivo,camino,nodos,padres,costes,candidatos, ...
    numeroNodos,indiceMeta,estadistica,parametros,geometria)
%CONSTRUIR_SALIDAS Recorta el arbol y genera el diagnostico.

nodos = nodos(1:numeroNodos,:);
padres = padres(1:numeroNodos);
costes = costes(1:numeroNodos);
candidatos = candidatos(1:numeroNodos);

indicesHijos = find(padres > 0);
aristas = [padres(indicesHijos),indicesHijos];

arbol = struct();
arbol.tipo = "RRT*";
arbol.nodos = nodos;
arbol.padres = padres;
arbol.costes = costes;
arbol.aristas = aristas;
arbol.indiceRaiz = 1;
arbol.indiceMeta = indiceMeta;
arbol.indicesCandidatosMeta = find(candidatos);
arbol.numeroNodos = numeroNodos;
arbol.numeroAristas = size(aristas,1);

info = struct();
info.exito = logical(exito);
info.motivo = string(motivo);
info.iteracionesSolicitadas = parametros.maxIter;
info.iteracionesEjecutadas = estadistica.iteracionesEjecutadas;
info.iteracionPrimeraSolucion = ...
    estadistica.iteracionPrimeraSolucion;
info.numeroNodos = arbol.numeroNodos;
info.numeroAristas = arbol.numeroAristas;
info.numeroRewirings = estadistica.numeroRewirings;
info.numeroCandidatosMeta = numel(arbol.indicesCandidatosMeta);
info.numeroMuestrasMeta = estadistica.numeroMuestrasMeta;
info.numeroMuestrasUniformes = estadistica.numeroMuestrasUniformes;
info.numeroRechazosDireccionNula = ...
    estadistica.numeroRechazosDireccionNula;
info.numeroRechazosDuplicado = ...
    estadistica.numeroRechazosDuplicado;
info.numeroRechazosColision = ...
    estadistica.numeroRechazosColision;
info.numeroAristasEscapePredictivo = ...
    estadistica.numeroAristasEscapePredictivo;
info.inicioDentroEnvolventePredictiva = ...
    geometria.inicioDentroEnvolventePredictiva;
info.metaExactaLibre = geometria.metaExactaLibre;
info.muestreoMetaComoRegion = ~geometria.metaExactaLibre;
info.horizontePrediccion = geometria.horizontePrediccion;
info.modeloPrediccion = parametros.modeloPrediccion;
info.obstaculosDinamicosConsiderados = ...
    parametros.considerarDinamicos;

info.parametros = struct();
info.parametros.maxIter = parametros.maxIter;
info.parametros.paso = parametros.paso;
info.parametros.radioVecinos = parametros.radioVecinos;
info.parametros.sesgoMeta = parametros.sesgoMeta;
info.parametros.radioMeta = parametros.radioMeta;
info.parametros.radioRobot = parametros.radioRobot;
info.parametros.margenEstatico = parametros.margenEstatico;
info.parametros.margenDinamico = parametros.margenDinamico;
info.parametros.pasosPrediccion = parametros.pasosPrediccion;
info.parametros.Ts = parametros.Ts;
info.parametros.considerarDinamicos = ...
    parametros.considerarDinamicos;

if isempty(camino)
    info.costeCamino = NaN;
    info.longitudCamino = NaN;
    info.distanciaFinalMeta = NaN;
else
    desplazamientos = diff(camino,1,1);
    info.longitudCamino = sum(hypot( ...
        desplazamientos(:,1),desplazamientos(:,2)));

    if isfinite(indiceMeta)
        info.costeCamino = arbol.costes(indiceMeta);
    else
        info.costeCamino = info.longitudCamino;
    end

    info.distanciaFinalMeta = ...
        norm(camino(end,1:2)-geometria.meta);
end

info.puntoFinalCamino = [NaN NaN];
info.finalEnRegionMeta = false;

if ~isempty(camino)
    info.puntoFinalCamino = camino(end,1:2);
    info.finalEnRegionMeta = ...
        info.distanciaFinalMeta <= ...
            parametros.radioMeta+geometria.tolerancia;
end

info.vistaPlanificador = struct();
info.vistaPlanificador.actualizar = true;
info.vistaPlanificador.nodos = arbol.nodos;
info.vistaPlanificador.aristas = arbol.aristas;
end

function motivo = motivo_punto_no_libre(prefijo,tipo,indice)
%MOTIVO_PUNTO_NO_LIBRE Construye una causa estable.

switch tipo
    case "limite"
        motivo = prefijo+"_fuera_de_limites_seguros";
    case "estatico"
        motivo = prefijo+"_bloqueado_por_estatico_S"+string(indice);
    case "dinamico"
        motivo = prefijo+"_bloqueado_por_dinamico_D"+string(indice);
    case "dinamico_fisico"
        motivo = prefijo+"_en_colision_con_dinamico_D"+string(indice);
    otherwise
        motivo = prefijo+"_no_libre";
end
end

%% ========================================================================
% VALIDACION
% ========================================================================

function [inicio,meta,limites,estaticos,dinamicos,parametros] = ...
    validar_entradas( ...
        estadoInicial,meta,limites,estaticos,dinamicos,robot,cfg)
%VALIDAR_ENTRADAS Comprueba el contrato del planificador.

if ~isnumeric(estadoInicial) || ~isreal(estadoInicial) || ...
        numel(estadoInicial) < 2 || any(~isfinite(estadoInicial(:)))
    error('rrt_star:EstadoInicialNoValido', ...
        'estadoInicial debe contener al menos [x y].');
end

estadoInicial = reshape(double(estadoInicial),1,[]);
inicio = estadoInicial(1:2);

if ~isnumeric(meta) || ~isreal(meta) || numel(meta) ~= 2 || ...
        any(~isfinite(meta(:)))
    error('rrt_star:MetaNoValida', ...
        'meta debe ser un vector real y finito [x y].');
end

meta = reshape(double(meta),1,2);

if ~isnumeric(limites) || ~isreal(limites) || ...
        numel(limites) ~= 4 || any(~isfinite(limites(:)))
    error('rrt_star:LimitesNoValidos', ...
        'limites debe tener formato [xmin xmax ymin ymax].');
end

limites = reshape(double(limites),1,4);

if limites(2) <= limites(1) || limites(4) <= limites(3)
    error('rrt_star:OrdenLimitesNoValido', ...
        'Debe cumplirse xmin < xmax e ymin < ymax.');
end

if isempty(estaticos)
    estaticos = zeros(0,4);
elseif ~isnumeric(estaticos) || ~isreal(estaticos) || ...
        size(estaticos,2) ~= 4 || any(~isfinite(estaticos(:)))
    error('rrt_star:EstaticosNoValidos', ...
        'obstaculosEstaticos debe tener formato N x 4.');
else
    estaticos = double(estaticos);
end

if ~isempty(estaticos) && any(estaticos(:,3:4) <= 0,'all')
    error('rrt_star:DimensionesEstaticosNoValidas', ...
        'El ancho y el alto de los rectangulos deben ser positivos.');
end

if isempty(dinamicos)
    dinamicos = struct('id',{},'pos',{},'vel',{},'radio',{});
elseif ~isstruct(dinamicos)
    error('rrt_star:DinamicosNoValidos', ...
        'obstaculosDinamicos debe ser un vector de estructuras.');
else
    for i = 1:numel(dinamicos)
        campos = {'pos','vel','radio'};

        for j = 1:numel(campos)
            if ~isfield(dinamicos(i),campos{j})
                error('rrt_star:CampoDinamicoAusente', ...
                    'Falta el campo "%s" en el obstaculo %d.', ...
                    campos{j},i);
            end
        end

        if ~isnumeric(dinamicos(i).pos) || ...
                ~isreal(dinamicos(i).pos) || ...
                numel(dinamicos(i).pos) ~= 2 || ...
                any(~isfinite(dinamicos(i).pos(:)))
            error('rrt_star:PosicionDinamicaNoValida', ...
                'La posicion del obstaculo %d debe ser [x y].',i);
        end

        if ~isnumeric(dinamicos(i).vel) || ...
                ~isreal(dinamicos(i).vel) || ...
                numel(dinamicos(i).vel) ~= 2 || ...
                any(~isfinite(dinamicos(i).vel(:)))
            error('rrt_star:VelocidadDinamicaNoValida', ...
                'La velocidad del obstaculo %d debe ser [vx vy].',i);
        end

        if ~es_escalar_positivo(dinamicos(i).radio)
            error('rrt_star:RadioDinamicoNoValido', ...
                'El radio del obstaculo %d debe ser positivo.',i);
        end

        dinamicos(i).pos = reshape(double(dinamicos(i).pos),1,2);
        dinamicos(i).vel = reshape(double(dinamicos(i).vel),1,2);
        dinamicos(i).radio = double(dinamicos(i).radio);
    end
end

if ~isstruct(robot) || ~isscalar(robot) || ...
        ~isfield(robot,'geometria') || ...
        ~isstruct(robot.geometria) || ...
        ~isfield(robot.geometria,'radio') || ...
        ~es_escalar_positivo(robot.geometria.radio)
    error('rrt_star:RobotNoValido', ...
        'robot debe proceder de configuracion_robot.m.');
end

if ~isstruct(cfg) || ~isscalar(cfg)
    error('rrt_star:ConfiguracionNoValida', ...
        'cfg debe proceder de parametros_generales.m.');
end

camposCfg = {'sim','navegacion','seguridad','prediccion','rrt'};
for i = 1:numel(camposCfg)
    if ~isfield(cfg,camposCfg{i}) || ~isstruct(cfg.(camposCfg{i}))
        error('rrt_star:ConfiguracionIncompleta', ...
            'Falta la estructura cfg.%s.',camposCfg{i});
    end
end

parametros = struct();
parametros.Ts = obtener_positivo(cfg.sim,'Ts','cfg.sim.Ts');
parametros.radioMeta = obtener_positivo( ...
    cfg.navegacion,'radioMeta','cfg.navegacion.radioMeta');
parametros.margenEstatico = obtener_no_negativo( ...
    cfg.seguridad,'margenEstatico','cfg.seguridad.margenEstatico');
parametros.margenDinamico = obtener_no_negativo( ...
    cfg.seguridad,'margenDinamico','cfg.seguridad.margenDinamico');
parametros.pasosPrediccion = obtener_entero_no_negativo( ...
    cfg.prediccion,'pasos','cfg.prediccion.pasos');
parametros.maxIter = obtener_entero_positivo( ...
    cfg.rrt,'maxIter','cfg.rrt.maxIter');
parametros.paso = obtener_positivo( ...
    cfg.rrt,'paso','cfg.rrt.paso');
parametros.radioVecinos = obtener_positivo( ...
    cfg.rrt,'radioVecinos','cfg.rrt.radioVecinos');

if ~isfield(cfg.rrt,'sesgoMeta') || ...
        ~es_escalar_no_negativo(cfg.rrt.sesgoMeta) || ...
        cfg.rrt.sesgoMeta > 1
    error('rrt_star:SesgoMetaNoValido', ...
        'cfg.rrt.sesgoMeta debe pertenecer a [0,1].');
end
parametros.sesgoMeta = double(cfg.rrt.sesgoMeta);

parametros.considerarDinamicos = false;
if isfield(cfg.rrt,'considerarDinamicos')
    valorConsiderar = cfg.rrt.considerarDinamicos;

    if islogical(valorConsiderar) && isscalar(valorConsiderar)
        parametros.considerarDinamicos = logical(valorConsiderar);
    elseif isnumeric(valorConsiderar) && isreal(valorConsiderar) && ...
            isscalar(valorConsiderar) && isfinite(valorConsiderar) && ...
            any(valorConsiderar == [0 1])
        parametros.considerarDinamicos = logical(valorConsiderar);
    else
        error('rrt_star:ConsiderarDinamicosNoValido', ...
            'cfg.rrt.considerarDinamicos debe ser un logico escalar.');
    end
end

if ~isfield(cfg.prediccion,'modelo') || ...
        strlength(strtrim(string(cfg.prediccion.modelo))) == 0
    error('rrt_star:ModeloPrediccionAusente', ...
        'Falta cfg.prediccion.modelo.');
end

parametros.modeloPrediccion = ...
    lower(strtrim(string(cfg.prediccion.modelo)));

if parametros.modeloPrediccion ~= "velocidad_constante"
    error('rrt_star:ModeloPrediccionNoSoportado', ...
        'Solo se admite el modelo "velocidad_constante".');
end

parametros.radioRobot = double(robot.geometria.radio);
separacion = parametros.radioRobot+parametros.margenEstatico;

if limites(2)-limites(1) <= 2*separacion || ...
        limites(4)-limites(3) <= 2*separacion
    error('rrt_star:MapaSinEspacioUtil', ...
        'El radio y el margen eliminan la region navegable.');
end
end

function valor = obtener_positivo(estructura,campo,nombre)
%OBTENER_POSITIVO Recupera un escalar positivo.

if ~isfield(estructura,campo) || ...
        ~es_escalar_positivo(estructura.(campo))
    error('rrt_star:EscalarPositivoNoValido', ...
        '%s debe ser positivo.',nombre);
end
valor = double(estructura.(campo));
end

function valor = obtener_no_negativo(estructura,campo,nombre)
%OBTENER_NO_NEGATIVO Recupera un escalar no negativo.

if ~isfield(estructura,campo) || ...
        ~es_escalar_no_negativo(estructura.(campo))
    error('rrt_star:EscalarNoNegativoNoValido', ...
        '%s debe ser no negativo.',nombre);
end
valor = double(estructura.(campo));
end

function valor = obtener_entero_positivo(estructura,campo,nombre)
%OBTENER_ENTERO_POSITIVO Recupera un entero positivo.

valor = obtener_positivo(estructura,campo,nombre);
if valor ~= floor(valor)
    error('rrt_star:EnteroPositivoNoValido', ...
        '%s debe ser entero.',nombre);
end
end

function valor = obtener_entero_no_negativo(estructura,campo,nombre)
%OBTENER_ENTERO_NO_NEGATIVO Recupera un entero no negativo.

valor = obtener_no_negativo(estructura,campo,nombre);
if valor ~= floor(valor)
    error('rrt_star:EnteroNoNegativoNoValido', ...
        '%s debe ser entero.',nombre);
end
end

function tf = es_escalar_positivo(valor)
%ES_ESCALAR_POSITIVO Comprueba un escalar real positivo.

tf = isnumeric(valor) && isreal(valor) && isscalar(valor) && ...
    isfinite(valor) && valor > 0;
end

function tf = es_escalar_no_negativo(valor)
%ES_ESCALAR_NO_NEGATIVO Comprueba un escalar real no negativo.

tf = isnumeric(valor) && isreal(valor) && isscalar(valor) && ...
    isfinite(valor) && valor >= 0;
end
