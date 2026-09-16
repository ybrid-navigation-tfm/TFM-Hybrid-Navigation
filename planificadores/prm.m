function [roadmap, info] = prm( ...
    meta, limites, obstaculosEstaticos, robot, cfg, ...
    roadmapAnterior, focoExpansion)
% PRM Construye o amplia una roadmap probabilistica persistente.
%
%   roadmap = PRM( ...
%       meta,limites,obstaculosEstaticos,robot,cfg)
%
%   [roadmap,info] = PRM(...)
%
%   [roadmap,info] = PRM( ...
%       meta,limites,obstaculosEstaticos,robot,cfg, ...
%       roadmapAnterior,focoExpansion)
%
%   Implementa la fase de preprocesamiento de un Probabilistic Roadmap
%   (PRM) bidimensional. La roadmap se construye una sola vez a partir de
%   la geometria estatica del escenario y se reutiliza posteriormente en
%   las distintas consultas realizadas durante la navegacion.
%
%   Responsabilidad de este modulo
%   --------------------------------
%   PRM se encarga exclusivamente de:
%
%       1. muestrear configuraciones libres del robot;
%       2. conectar vecinos mediante aristas libres de obstaculos
%          estaticos;
%       3. almacenar pesos y cajas envolventes de las aristas;
%       4. conservar la meta como nodo persistente de la roadmap;
%       5. ampliar excepcionalmente una roadmap existente.
%
%   Los obstaculos dinamicos NO intervienen en esta construccion. Su efecto
%   se evaluara en consulta_prm.m, que desactivara temporalmente las aristas
%   bloqueadas, conectara la posicion actual del robot y buscara el camino
%   hasta la meta. Esta separacion evita reconstruir la roadmap completa en
%   cada ciclo de simulacion.
%
%   Construccion inicial
%   --------------------
%   Cuando roadmapAnterior se omite o se proporciona vacia, la funcion:
%
%       - inserta la meta como primer nodo;
%       - genera cfg.prm.nMuestras nodos aleatorios adicionales;
%       - conecta como maximo cfg.prm.maxVecinos por nodo dentro de
%         cfg.prm.radioConexion;
%       - descarta las aristas que no respetan el margen estatico.
%
%   Ampliacion excepcional
%   ----------------------
%   Cuando se proporciona roadmapAnterior, la funcion conserva todos sus
%   nodos y aristas y agrega cfg.prm.muestrasExpansion muestras nuevas.
%
%   focoExpansion puede ser la posicion actual del robot [x y] o
%   [x y theta]. Si es valido, una parte de las muestras se concentra en su
%   entorno para mejorar la conectividad local tras varios fallos de
%   consulta. Si se omite, la ampliacion utiliza muestreo uniforme.
%
%   La funcion impide superar cfg.prm.maxExpansiones.
%
%   Entradas:
%       meta
%           Posicion objetivo [x y]. La meta se almacena como nodo 1.
%
%       limites
%           Limites del mapa [xmin xmax ymin ymax].
%
%       obstaculosEstaticos
%           Matriz N x 4 con rectangulos [x y ancho alto].
%
%       robot
%           Estructura obtenida mediante configuracion_robot.m. Se usa:
%
%               robot.geometria.radio
%
%       cfg
%           Estructura obtenida mediante parametros_generales.m. Se usan:
%
%               cfg.seguridad.margenEstatico
%               cfg.prm.nMuestras
%               cfg.prm.radioConexion
%               cfg.prm.maxVecinos
%               cfg.prm.radioConsulta
%               cfg.prm.muestrasExpansion
%               cfg.prm.maxExpansiones
%
%       roadmapAnterior
%           Roadmap devuelta por una llamada anterior a PRM. Es opcional.
%
%       focoExpansion
%           Posicion opcional alrededor de la que se concentra parte del
%           muestreo durante una ampliacion.
%
%   Salida roadmap:
%       Estructura persistente con los campos principales:
%
%           .esquema                         "roadmap_prm_v1"
%           .tipo                            "PRM_persistente"
%           .nodos                           N x 2
%           .aristas                         M x 2 [i j]
%           .pesos                           M x 1
%           .cajasAristas                     M x 4
%           .indiceMeta                      1
%           .meta
%           .limites
%           .obstaculosEstaticos
%           .separacionEstatica
%           .radioConexion
%           .maxVecinos
%           .numeroNodos
%           .numeroAristas
%           .numeroConstrucciones
%           .numeroExpansiones
%           .muestrasSolicitadasAcumuladas
%           .muestrasAceptadasAcumuladas
%
%       Cada fila de cajasAristas contiene:
%
%           [xmin xmax ymin ymax]
%
%       para la arista correspondiente. consulta_prm.m utilizara estas
%       cajas para filtrar rapidamente las aristas que pueden quedar
%       afectadas por el barrido predicho de un obstaculo dinamico.
%
%   Salida info:
%       Diagnostico de la construccion o ampliacion:
%
%           .accion
%           .exito
%           .motivo
%           .numeroNodosAntes
%           .numeroNodosDespues
%           .numeroAristasAntes
%           .numeroAristasDespues
%           .muestrasSolicitadas
%           .muestrasAceptadas
%           .intentosMuestreo
%           .paresCandidatos
%           .aristasAceptadas
%           .aristasRechazadasColision
%           .numeroComponentes
%           .numeroNodosAislados
%           .tamanoComponenteMeta
%           .fraccionComponenteMeta
%           .metaAislada
%           .roadmapUtilizable
%           .parametros
%           .vistaPlanificador
%
%   Visualizacion
%   -------------
%   Este modulo NO envia todas las aristas de la roadmap a la figura. Una
%   PRM completa puede ser muy densa y producir una representacion poco
%   interpretable. info.vistaPlanificador se devuelve con
%   actualizar=false. consulta_prm.m generara posteriormente un unico arbol
%   grafico reducido para la consulta actual. actualizar_graficos.m borrara
%   la representacion anterior antes de dibujar ese nuevo arbol.
%
%   Reproducibilidad
%   ----------------
%   Esta funcion no llama a rng. La semilla debe fijarse en el main:
%
%       rng(cfg.semilla,'twister');
%
%   Ejemplo de construccion inicial:
%
%       cfg = parametros_generales("visual");
%       escenario = escenarios("media");
%       robot = configuracion_robot();
%
%       rng(cfg.semilla,'twister');
%
%       relojPRM = tic;
%
%       [roadmap,infoPRM] = prm( ...
%           escenario.meta, ...
%           escenario.limites, ...
%           escenario.obstaculosEstaticos, ...
%           robot, ...
%           cfg);
%
%       tiempoConstruccionPRM = toc(relojPRM);
%
%   Ejemplo de ampliacion excepcional:
%
%       [roadmap,infoExpansion] = prm( ...
%           escenario.meta, ...
%           escenario.limites, ...
%           escenario.obstaculosEstaticos, ...
%           robot, ...
%           cfg, ...
%           roadmap, ...
%           estadoRobot);
%
%   Esta funcion utiliza:
%       - circulo_rectangulo.m
%       - segmento_rectangulo.m
%       - distancia_segmentos.m
%
%   El tiempo de construccion o ampliacion se mide externamente mediante
%   tic/toc para que tiempos.m pueda separarlo del control y la
%   visualizacion.

if nargin < 6
    roadmapAnterior = [];
end

if nargin < 7
    focoExpansion = [];
end

%% Validacion de la configuracion comun
[meta, limites, estaticos, parametros] = validar_entradas_base( ...
    meta,limites,obstaculosEstaticos,robot,cfg);

geometria = preparar_geometria( ...
    meta,limites,estaticos,parametros);

%% Construccion inicial o ampliacion
if isempty(roadmapAnterior)
    [roadmap,info] = construir_roadmap( ...
        meta,limites,estaticos,parametros,geometria);
else
    focoExpansion = normalizar_foco(focoExpansion);

    roadmapAnterior = validar_roadmap_anterior( ...
        roadmapAnterior,meta,limites,estaticos,parametros,geometria);

    [roadmap,info] = ampliar_roadmap( ...
        roadmapAnterior,focoExpansion,parametros,geometria);
end
end

%% ========================================================================
% CONSTRUCCION INICIAL
% ========================================================================

function [roadmap,info] = construir_roadmap( ...
    meta,limites,estaticos,parametros,geometria)
%CONSTRUIR_ROADMAP Genera la roadmap estatica por primera vez.

numeroNodosAntes = 0;
numeroAristasAntes = 0;

%% Muestreo uniforme del espacio libre
[muestras,estadisticaMuestreo] = muestrear_nodos_libres( ...
    parametros.nMuestras, ...
    meta, ...
    [NaN NaN], ...
    parametros, ...
    geometria);

nodos = [meta; muestras];
numeroNodos = size(nodos,1);

%% Pares de vecinos candidatos
indicesFuente = (1:numeroNodos).';

[paresCandidatos,estadisticaVecinos] = generar_pares_vecinos( ...
    nodos,indicesFuente,parametros.radioConexion, ...
    parametros.maxVecinos,geometria.tolerancia);

%% Comprobacion geometrica de las aristas
[aristas,pesos,cajasAristas,estadisticaAristas] = ...
    validar_aristas_candidatas( ...
        nodos,paresCandidatos,geometria);

%% Estructura persistente
roadmap = estructura_roadmap_base( ...
    meta,limites,estaticos,parametros,geometria);

roadmap.nodos = nodos;
roadmap.aristas = aristas;
roadmap.pesos = pesos;
roadmap.cajasAristas = cajasAristas;

roadmap.numeroNodos = size(nodos,1);
roadmap.numeroAristas = size(aristas,1);
roadmap.numeroConstrucciones = 1;
roadmap.numeroExpansiones = 0;

roadmap.muestrasSolicitadasAcumuladas = ...
    parametros.nMuestras;
roadmap.muestrasAceptadasAcumuladas = ...
    estadisticaMuestreo.aceptadas;
roadmap.intentosMuestreoAcumulados = ...
    estadisticaMuestreo.intentos;

roadmap.accionUltima = "construccion";
roadmap.focoUltimaExpansion = [NaN NaN];

%% Diagnostico topologico
analisis = analizar_roadmap( ...
    roadmap.nodos,roadmap.aristas,roadmap.indiceMeta);

roadmap.diagnosticoTopologico = resumen_topologico(analisis);

%% Salida informativa
if estadisticaMuestreo.aceptadas == parametros.nMuestras
    motivo = "roadmap_construida";
else
    motivo = "roadmap_construida_con_muestras_parciales";
end

exitoOperacion = estadisticaMuestreo.aceptadas > 0;

info = construir_info( ...
    "construccion",exitoOperacion,motivo, ...
    numeroNodosAntes,numeroAristasAntes,roadmap, ...
    estadisticaMuestreo,estadisticaVecinos, ...
    estadisticaAristas,analisis,parametros);
end

%% ========================================================================
% AMPLIACION EXCEPCIONAL
% ========================================================================

function [roadmap,info] = ampliar_roadmap( ...
    roadmap,foco,parametros,geometria)
%AMPLIAR_ROADMAP Agrega muestras sin destruir la roadmap existente.

numeroNodosAntes = size(roadmap.nodos,1);
numeroAristasAntes = size(roadmap.aristas,1);

%% Proteccion frente a un numero excesivo de ampliaciones
if roadmap.numeroExpansiones >= parametros.maxExpansiones
    estadisticaMuestreo = estadistica_muestreo_vacia(0);
    estadisticaVecinos = estadistica_vecinos_vacia();
    estadisticaAristas = estadistica_aristas_vacia();

    analisis = analizar_roadmap( ...
        roadmap.nodos,roadmap.aristas,roadmap.indiceMeta);

    info = construir_info( ...
        "expansion",false,"limite_expansiones_alcanzado", ...
        numeroNodosAntes,numeroAristasAntes,roadmap, ...
        estadisticaMuestreo,estadisticaVecinos, ...
        estadisticaAristas,analisis,parametros);
    return;
end

%% Nuevas muestras, con sesgo local opcional
[muestrasNuevas,estadisticaMuestreo] = muestrear_nodos_libres( ...
    parametros.muestrasExpansion, ...
    roadmap.nodos, ...
    foco, ...
    parametros, ...
    geometria);

roadmap.numeroExpansiones = roadmap.numeroExpansiones+1;
roadmap.accionUltima = "expansion";
roadmap.focoUltimaExpansion = foco;

roadmap.muestrasSolicitadasAcumuladas = ...
    roadmap.muestrasSolicitadasAcumuladas+ ...
    parametros.muestrasExpansion;

roadmap.muestrasAceptadasAcumuladas = ...
    roadmap.muestrasAceptadasAcumuladas+ ...
    estadisticaMuestreo.aceptadas;

roadmap.intentosMuestreoAcumulados = ...
    roadmap.intentosMuestreoAcumulados+ ...
    estadisticaMuestreo.intentos;

%% Ninguna muestra aceptada
if isempty(muestrasNuevas)
    estadisticaVecinos = estadistica_vecinos_vacia();
    estadisticaAristas = estadistica_aristas_vacia();

    analisis = analizar_roadmap( ...
        roadmap.nodos,roadmap.aristas,roadmap.indiceMeta);

    roadmap.diagnosticoTopologico = resumen_topologico(analisis);

    info = construir_info( ...
        "expansion",false,"expansion_sin_muestras_aceptadas", ...
        numeroNodosAntes,numeroAristasAntes,roadmap, ...
        estadisticaMuestreo,estadisticaVecinos, ...
        estadisticaAristas,analisis,parametros);
    return;
end

%% Incorporacion de los nodos
roadmap.nodos = [roadmap.nodos; muestrasNuevas];

indicesNuevos = (numeroNodosAntes+1:size(roadmap.nodos,1)).';

%% Solo los nodos nuevos originan pares adicionales
[paresCandidatos,estadisticaVecinos] = generar_pares_vecinos( ...
    roadmap.nodos,indicesNuevos,parametros.radioConexion, ...
    parametros.maxVecinos,geometria.tolerancia);

% Proteccion frente a duplicados con aristas ya existentes.
if ~isempty(paresCandidatos) && ~isempty(roadmap.aristas)
    aristasExistentes = sort(roadmap.aristas,2);
    paresCandidatos = sort(paresCandidatos,2);

    yaExiste = ismember( ...
        paresCandidatos,aristasExistentes,'rows');

    paresCandidatos = paresCandidatos(~yaExiste,:);
    estadisticaVecinos.paresUnicos = size(paresCandidatos,1);
end

%% Validacion de las nuevas conexiones
[nuevasAristas,nuevosPesos,nuevasCajas,estadisticaAristas] = ...
    validar_aristas_candidatas( ...
        roadmap.nodos,paresCandidatos,geometria);

roadmap.aristas = [roadmap.aristas; nuevasAristas];
roadmap.pesos = [roadmap.pesos; nuevosPesos];
roadmap.cajasAristas = [roadmap.cajasAristas; nuevasCajas];

roadmap.numeroNodos = size(roadmap.nodos,1);
roadmap.numeroAristas = size(roadmap.aristas,1);

%% Diagnostico topologico actualizado
analisis = analizar_roadmap( ...
    roadmap.nodos,roadmap.aristas,roadmap.indiceMeta);

roadmap.diagnosticoTopologico = resumen_topologico(analisis);

if estadisticaAristas.aceptadas > 0
    motivo = "expansion_realizada";
    exitoOperacion = true;
else
    motivo = "expansion_sin_nuevas_aristas";
    exitoOperacion = false;
end

info = construir_info( ...
    "expansion",exitoOperacion,motivo, ...
    numeroNodosAntes,numeroAristasAntes,roadmap, ...
    estadisticaMuestreo,estadisticaVecinos, ...
    estadisticaAristas,analisis,parametros);
end

%% ========================================================================
% MUESTREO
% ========================================================================

function [muestras,estadistica] = muestrear_nodos_libres( ...
    numeroSolicitado,nodosExistentes,foco,parametros,geometria)
%MUESTREAR_NODOS_LIBRES Genera configuraciones validas del robot.

estadistica = estadistica_muestreo_vacia(numeroSolicitado);

if numeroSolicitado == 0
    muestras = zeros(0,2);
    return;
end

muestras = zeros(numeroSolicitado,2);
numeroAceptadas = 0;
numeroIntentos = 0;

maximoIntentos = max( ...
    parametros.factorMaximoIntentos*numeroSolicitado, ...
    numeroSolicitado);

limitesSeguros = geometria.limitesSeguros;
usaFoco = all(isfinite(foco));

while numeroAceptadas < numeroSolicitado && ...
        numeroIntentos < maximoIntentos

    numeroIntentos = numeroIntentos+1;

    %% Muestreo local o uniforme
    if usaFoco && rand < parametros.probabilidadMuestreoLocal
        candidato = foco+ ...
            parametros.radioMuestreoLocal*(2*rand(1,2)-1);

        candidato(1) = min(max( ...
            candidato(1),limitesSeguros(1)), ...
            limitesSeguros(2));

        candidato(2) = min(max( ...
            candidato(2),limitesSeguros(3)), ...
            limitesSeguros(4));

        estadistica.intentosLocales = ...
            estadistica.intentosLocales+1;
    else
        candidato = muestrear_uniforme(limitesSeguros);
        estadistica.intentosUniformes = ...
            estadistica.intentosUniformes+1;
    end

    %% Colision con la geometria estatica
    if ~punto_estatico_libre(candidato,geometria)
        estadistica.rechazosColision = ...
            estadistica.rechazosColision+1;
        continue;
    end

    %% Separacion minima entre muestras
    nodosComparacion = [ ...
        nodosExistentes; ...
        muestras(1:numeroAceptadas,:)];

    if ~isempty(nodosComparacion)
        diferencias = nodosComparacion-candidato;
        distancias2 = sum(diferencias.^2,2);

        if any(distancias2 <= parametros.separacionMinimaMuestras^2)
            estadistica.rechazosProximidad = ...
                estadistica.rechazosProximidad+1;
            continue;
        end
    end

    numeroAceptadas = numeroAceptadas+1;
    muestras(numeroAceptadas,:) = candidato;
end

muestras = muestras(1:numeroAceptadas,:);

estadistica.aceptadas = numeroAceptadas;
estadistica.intentos = numeroIntentos;
estadistica.maximoIntentos = maximoIntentos;
estadistica.completa = numeroAceptadas == numeroSolicitado;
end

function candidato = muestrear_uniforme(limitesSeguros)
%MUESTREAR_UNIFORME Genera un punto uniforme en el mapa seguro.

candidato = [ ...
    limitesSeguros(1)+rand*( ...
        limitesSeguros(2)-limitesSeguros(1)), ...
    limitesSeguros(3)+rand*( ...
        limitesSeguros(4)-limitesSeguros(3))];
end

%% ========================================================================
% VECINOS Y ARISTAS
% ========================================================================

function [pares,estadistica] = generar_pares_vecinos( ...
    nodos,indicesFuente,radioConexion,maxVecinos,tolerancia)
%GENERAR_PARES_VECINOS Busca conexiones por radio y numero maximo.

estadistica = estadistica_vecinos_vacia();

numeroNodos = size(nodos,1);

if numeroNodos < 2 || isempty(indicesFuente)
    pares = zeros(0,2);
    return;
end

indicesFuente = reshape(indicesFuente,[],1);

%% Matriz de distancias cuadradas sin requerir toolboxes
coordenadaX = nodos(:,1);
coordenadaY = nodos(:,2);

diferenciasX = coordenadaX-coordenadaX.';
diferenciasY = coordenadaY-coordenadaY.';

distancias2 = diferenciasX.^2+diferenciasY.^2;
distancias2(1:numeroNodos+1:end) = inf;

radio2 = (radioConexion+tolerancia)^2;

capacidad = numel(indicesFuente)*min( ...
    maxVecinos,max(0,numeroNodos-1));

buffer = zeros(capacidad,2);
numeroPares = 0;

for posicionFuente = 1:numel(indicesFuente)
    indice = indicesFuente(posicionFuente);

    [distanciasOrdenadas,orden] = sort( ...
        distancias2(indice,:),'ascend');

    dentroRadio = isfinite(distanciasOrdenadas) & ...
        distanciasOrdenadas <= radio2;

    orden = orden(dentroRadio);

    if numel(orden) > maxVecinos
        orden = orden(1:maxVecinos);
    end

    estadistica.vecinosSeleccionados = ...
        estadistica.vecinosSeleccionados+numel(orden);

    for vecino = reshape(orden,1,[])
        numeroPares = numeroPares+1;
        buffer(numeroPares,:) = sort([indice vecino]);
    end
end

pares = buffer(1:numeroPares,:);
estadistica.paresAntesDeUnicos = numeroPares;

if ~isempty(pares)
    pares = unique(pares,'rows','stable');
end

estadistica.paresUnicos = size(pares,1);
end

function [aristas,pesos,cajas,estadistica] = ...
    validar_aristas_candidatas(nodos,pares,geometria)
%VALIDAR_ARISTAS_CANDIDATAS Conserva conexiones estaticamente seguras.

estadistica = estadistica_aristas_vacia();
estadistica.candidatas = size(pares,1);

if isempty(pares)
    aristas = zeros(0,2);
    pesos = zeros(0,1);
    cajas = zeros(0,4);
    return;
end

valida = false(size(pares,1),1);

for i = 1:size(pares,1)
    a = nodos(pares(i,1),:);
    b = nodos(pares(i,2),:);

    valida(i) = arista_estatica_libre(a,b,geometria);
end

aristas = pares(valida,:);

if isempty(aristas)
    pesos = zeros(0,1);
    cajas = zeros(0,4);
else
    extremoA = nodos(aristas(:,1),:);
    extremoB = nodos(aristas(:,2),:);

    pesos = hypot( ...
        extremoA(:,1)-extremoB(:,1), ...
        extremoA(:,2)-extremoB(:,2));

    cajas = [ ...
        min(extremoA(:,1),extremoB(:,1)), ...
        max(extremoA(:,1),extremoB(:,1)), ...
        min(extremoA(:,2),extremoB(:,2)), ...
        max(extremoA(:,2),extremoB(:,2))];
end

estadistica.aceptadas = size(aristas,1);
estadistica.rechazadasColision = ...
    estadistica.candidatas-estadistica.aceptadas;
end

%% ========================================================================
% GEOMETRIA ESTATICA
% ========================================================================

function geometria = preparar_geometria( ...
    meta,limites,estaticos,parametros)
%PREPARAR_GEOMETRIA Precalcula margenes y limites seguros.

geometria = struct();
geometria.meta = meta;
geometria.limites = limites;
geometria.estaticos = estaticos;
geometria.separacionEstatica = ...
    parametros.radioRobot+parametros.margenEstatico;

geometria.limitesSeguros = [ ...
    limites(1)+geometria.separacionEstatica, ...
    limites(2)-geometria.separacionEstatica, ...
    limites(3)+geometria.separacionEstatica, ...
    limites(4)-geometria.separacionEstatica];

valoresEscala = [ ...
    meta(:); ...
    limites(:); ...
    estaticos(:); ...
    geometria.separacionEstatica];

geometria.tolerancia = ...
    1e-12*max(1,max(abs(valoresEscala)));
end

function libre = punto_estatico_libre(punto,geometria)
%PUNTO_ESTATICO_LIBRE Comprueba limites y obstaculos rectangulares.

tol = geometria.tolerancia;
limitesSeguros = geometria.limitesSeguros;

libre = ...
    punto(1) >= limitesSeguros(1)-tol && ...
    punto(1) <= limitesSeguros(2)+tol && ...
    punto(2) >= limitesSeguros(3)-tol && ...
    punto(2) <= limitesSeguros(4)+tol;

if ~libre
    return;
end

for i = 1:size(geometria.estaticos,1)
    hayContacto = circulo_rectangulo( ...
        punto,geometria.separacionEstatica, ...
        geometria.estaticos(i,:),tol);

    if hayContacto
        libre = false;
        return;
    end
end
end

function libre = arista_estatica_libre(a,b,geometria)
%ARISTA_ESTATICA_LIBRE Comprueba el segmento con el margen del robot.

if ~punto_estatico_libre(a,geometria) || ...
        ~punto_estatico_libre(b,geometria)
    libre = false;
    return;
end

libre = true;
separacion = geometria.separacionEstatica;
tol = geometria.tolerancia;

for i = 1:size(geometria.estaticos,1)
    rectangulo = geometria.estaticos(i,:);

    if cajas_separadas_segmento_rectangulo( ...
            a,b,rectangulo,separacion,tol)
        continue;
    end

    distancia = distancia_segmento_rectangulo( ...
        a,b,rectangulo,tol);

    if distancia <= separacion+tol
        libre = false;
        return;
    end
end
end

function separadas = cajas_separadas_segmento_rectangulo( ...
    a,b,rectangulo,margen,tol)
%CAJAS_SEPARADAS_SEGMENTO_RECTANGULO Filtro geometrico rapido.

xminSegmento = min(a(1),b(1))-margen;
xmaxSegmento = max(a(1),b(1))+margen;
yminSegmento = min(a(2),b(2))-margen;
ymaxSegmento = max(a(2),b(2))+margen;

xminRectangulo = rectangulo(1);
xmaxRectangulo = rectangulo(1)+rectangulo(3);
yminRectangulo = rectangulo(2);
ymaxRectangulo = rectangulo(2)+rectangulo(4);

separadas = ...
    xmaxSegmento < xminRectangulo-tol || ...
    xminSegmento > xmaxRectangulo+tol || ...
    ymaxSegmento < yminRectangulo-tol || ...
    yminSegmento > ymaxRectangulo+tol;
end

function distancia = distancia_segmento_rectangulo( ...
    a,b,rectangulo,tol)
%DISTANCIA_SEGMENTO_RECTANGULO Distancia de un segmento a un rectangulo.

if segmento_rectangulo(a,b,rectangulo,tol)
    distancia = 0;
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

lados = [ ...
    1 2; ...
    2 3; ...
    3 4; ...
    4 1];

distancia = inf;

for i = 1:4
    c = esquinas(lados(i,1),:);
    d = esquinas(lados(i,2),:);

    distancia = min( ...
        distancia,distancia_segmentos(a,b,c,d));
end
end

%% ========================================================================
% DIAGNOSTICO TOPOLOGICO
% ========================================================================

function analisis = analizar_roadmap(nodos,aristas,indiceMeta)
%ANALIZAR_ROADMAP Calcula componentes, grados y conectividad de la meta.

numeroNodos = size(nodos,1);

analisis = struct();
analisis.numeroNodos = numeroNodos;
analisis.numeroAristas = size(aristas,1);

if numeroNodos == 0
    analisis.numeroComponentes = 0;
    analisis.numeroNodosAislados = 0;
    analisis.gradoMedio = NaN;
    analisis.gradoMaximo = NaN;
    analisis.tamanoComponenteMeta = 0;
    analisis.fraccionComponenteMeta = NaN;
    analisis.metaAislada = true;
    analisis.etiquetaComponente = zeros(0,1);
    analisis.tamanosComponentes = zeros(0,1);
    return;
end

if isempty(aristas)
    adyacencia = sparse(numeroNodos,numeroNodos);
else
    adyacencia = sparse( ...
        [aristas(:,1); aristas(:,2)], ...
        [aristas(:,2); aristas(:,1)], ...
        true(2*size(aristas,1),1), ...
        numeroNodos,numeroNodos);
end

grados = full(sum(adyacencia,2));

etiqueta = zeros(numeroNodos,1);
numeroComponentes = 0;
tamanos = zeros(numeroNodos,1);
cola = zeros(numeroNodos,1);

for raiz = 1:numeroNodos
    if etiqueta(raiz) ~= 0
        continue;
    end

    numeroComponentes = numeroComponentes+1;

    cabeza = 1;
    final = 1;
    cola(final) = raiz;
    etiqueta(raiz) = numeroComponentes;
    tamano = 0;

    while cabeza <= final
        nodo = cola(cabeza);
        cabeza = cabeza+1;
        tamano = tamano+1;

        vecinos = find(adyacencia(nodo,:));

        for vecino = reshape(vecinos,1,[])
            if etiqueta(vecino) ~= 0
                continue;
            end

            final = final+1;
            cola(final) = vecino;
            etiqueta(vecino) = numeroComponentes;
        end
    end

    tamanos(numeroComponentes) = tamano;
end

tamanos = tamanos(1:numeroComponentes);
componenteMeta = etiqueta(indiceMeta);
tamanoComponenteMeta = tamanos(componenteMeta);

analisis.numeroComponentes = numeroComponentes;
analisis.numeroNodosAislados = nnz(grados == 0);
analisis.gradoMedio = mean(grados);
analisis.gradoMaximo = max(grados);
analisis.tamanoComponenteMeta = tamanoComponenteMeta;
analisis.fraccionComponenteMeta = ...
    tamanoComponenteMeta/numeroNodos;
analisis.metaAislada = grados(indiceMeta) == 0;
analisis.etiquetaComponente = etiqueta;
analisis.tamanosComponentes = tamanos;
end

function resumen = resumen_topologico(analisis)
%RESUMEN_TOPOLOGICO Conserva solamente los escalares principales.

resumen = struct();
resumen.numeroComponentes = analisis.numeroComponentes;
resumen.numeroNodosAislados = analisis.numeroNodosAislados;
resumen.gradoMedio = analisis.gradoMedio;
resumen.gradoMaximo = analisis.gradoMaximo;
resumen.tamanoComponenteMeta = analisis.tamanoComponenteMeta;
resumen.fraccionComponenteMeta = analisis.fraccionComponenteMeta;
resumen.metaAislada = analisis.metaAislada;
end

%% ========================================================================
% CONSTRUCCION DE SALIDAS
% ========================================================================

function roadmap = estructura_roadmap_base( ...
    meta,limites,estaticos,parametros,geometria)
%ESTRUCTURA_ROADMAP_BASE Define el contrato persistente de PRM.

roadmap = struct();

roadmap.esquema = "roadmap_prm_v1";
roadmap.tipo = "PRM_persistente";
roadmap.version = "1.0";

roadmap.nodos = zeros(0,2);
roadmap.aristas = zeros(0,2);
roadmap.pesos = zeros(0,1);
roadmap.cajasAristas = zeros(0,4);

roadmap.indiceMeta = 1;
roadmap.meta = meta;
roadmap.limites = limites;
roadmap.obstaculosEstaticos = estaticos;

roadmap.radioRobot = parametros.radioRobot;
roadmap.margenEstatico = parametros.margenEstatico;
roadmap.separacionEstatica = geometria.separacionEstatica;
roadmap.radioConexion = parametros.radioConexion;
roadmap.maxVecinos = parametros.maxVecinos;
roadmap.separacionMinimaMuestras = ...
    parametros.separacionMinimaMuestras;

roadmap.numeroNodos = 0;
roadmap.numeroAristas = 0;
roadmap.numeroConstrucciones = 0;
roadmap.numeroExpansiones = 0;

roadmap.muestrasSolicitadasAcumuladas = 0;
roadmap.muestrasAceptadasAcumuladas = 0;
roadmap.intentosMuestreoAcumulados = 0;

roadmap.semillaConfigurada = parametros.semillaConfigurada;
roadmap.versionConfiguracion = parametros.versionConfiguracion;
roadmap.accionUltima = "";
roadmap.focoUltimaExpansion = [NaN NaN];
roadmap.diagnosticoTopologico = struct();
end

function info = construir_info( ...
    accion,exito,motivo,numeroNodosAntes,numeroAristasAntes, ...
    roadmap,muestreo,vecinos,aristas,analisis,parametros)
%CONSTRUIR_INFO Reune el diagnostico de la operacion.

info = struct();

info.accion = accion;
info.exito = logical(exito);
info.motivo = motivo;

info.numeroNodosAntes = numeroNodosAntes;
info.numeroNodosDespues = size(roadmap.nodos,1);
info.numeroNodosAgregados = ...
    info.numeroNodosDespues-numeroNodosAntes;

info.numeroAristasAntes = numeroAristasAntes;
info.numeroAristasDespues = size(roadmap.aristas,1);
info.numeroAristasAgregadas = ...
    info.numeroAristasDespues-numeroAristasAntes;

info.muestrasSolicitadas = muestreo.solicitadas;
info.muestrasAceptadas = muestreo.aceptadas;
info.intentosMuestreo = muestreo.intentos;
info.maximoIntentosMuestreo = muestreo.maximoIntentos;
info.muestreoCompleto = muestreo.completa;
info.intentosLocales = muestreo.intentosLocales;
info.intentosUniformes = muestreo.intentosUniformes;
info.rechazosColision = muestreo.rechazosColision;
info.rechazosProximidad = muestreo.rechazosProximidad;

info.vecinosSeleccionados = vecinos.vecinosSeleccionados;
info.paresAntesDeUnicos = vecinos.paresAntesDeUnicos;
info.paresCandidatos = vecinos.paresUnicos;

info.aristasCandidatas = aristas.candidatas;
info.aristasAceptadas = aristas.aceptadas;
info.aristasRechazadasColision = ...
    aristas.rechazadasColision;

info.numeroComponentes = analisis.numeroComponentes;
info.numeroNodosAislados = analisis.numeroNodosAislados;
info.gradoMedio = analisis.gradoMedio;
info.gradoMaximo = analisis.gradoMaximo;
info.tamanoComponenteMeta = analisis.tamanoComponenteMeta;
info.fraccionComponenteMeta = analisis.fraccionComponenteMeta;
info.metaAislada = analisis.metaAislada;

info.roadmapUtilizable = ...
    size(roadmap.aristas,1) > 0 && ...
    ~analisis.metaAislada && ...
    analisis.tamanoComponenteMeta >= 2;

info.numeroConstrucciones = roadmap.numeroConstrucciones;
info.numeroExpansiones = roadmap.numeroExpansiones;
info.numeroNodosRoadmap = roadmap.numeroNodos;
info.numeroAristasRoadmap = roadmap.numeroAristas;

info.parametros = struct();
info.parametros.nMuestras = parametros.nMuestras;
info.parametros.radioConexion = parametros.radioConexion;
info.parametros.maxVecinos = parametros.maxVecinos;
info.parametros.muestrasExpansion = parametros.muestrasExpansion;
info.parametros.maxExpansiones = parametros.maxExpansiones;
info.parametros.separacionMinimaMuestras = ...
    parametros.separacionMinimaMuestras;
info.parametros.probabilidadMuestreoLocal = ...
    parametros.probabilidadMuestreoLocal;
info.parametros.radioMuestreoLocal = ...
    parametros.radioMuestreoLocal;
info.parametros.separacionEstatica = ...
    roadmap.separacionEstatica;

% La vista se generara en consulta_prm.m para no dibujar la roadmap densa.
info.vistaPlanificador = struct();
info.vistaPlanificador.actualizar = false;
info.vistaPlanificador.nodos = zeros(0,2);
info.vistaPlanificador.aristas = zeros(0,2);
end

%% ========================================================================
% VALIDACION
% ========================================================================

function [meta,limites,estaticos,parametros] = validar_entradas_base( ...
    meta,limites,estaticos,robot,cfg)
%VALIDAR_ENTRADAS_BASE Comprueba escenario, robot y parametros PRM.

%% Meta
if ~isnumeric(meta) || ~isreal(meta) || ...
        numel(meta) ~= 2 || any(~isfinite(meta(:)))
    error('prm:MetaNoValida', ...
        'meta debe ser un vector numerico real y finito [x y].');
end

meta = reshape(double(meta),1,2);

%% Limites
if ~isnumeric(limites) || ~isreal(limites) || ...
        numel(limites) ~= 4 || any(~isfinite(limites(:)))
    error('prm:LimitesNoValidos', ...
        'limites debe tener formato [xmin xmax ymin ymax].');
end

limites = reshape(double(limites),1,4);

if limites(2) <= limites(1) || limites(4) <= limites(3)
    error('prm:ExtensionMapaNoValida', ...
        'Los limites del mapa deben definir un area positiva.');
end

%% Obstaculos estaticos
if isempty(estaticos)
    estaticos = zeros(0,4);
elseif ~isnumeric(estaticos) || ~isreal(estaticos) || ...
        size(estaticos,2) ~= 4 || any(~isfinite(estaticos(:)))
    error('prm:EstaticosNoValidos', ...
        ['obstaculosEstaticos debe ser una matriz N x 4 con formato ' ...
         '[x y ancho alto].']);
else
    estaticos = double(estaticos);
end

if ~isempty(estaticos) && any(estaticos(:,3:4) <= 0,'all')
    error('prm:DimensionesEstaticosNoValidas', ...
        'El ancho y el alto de los obstaculos deben ser positivos.');
end

%% Robot
if ~isstruct(robot) || ~isscalar(robot) || ...
        ~isfield(robot,'geometria') || ...
        ~isstruct(robot.geometria) || ...
        ~isfield(robot.geometria,'radio')
    error('prm:RobotNoValido', ...
        ['robot debe ser la estructura obtenida mediante ' ...
         'configuracion_robot.m.']);
end

radioRobot = robot.geometria.radio;

if ~es_escalar_positivo(radioRobot)
    error('prm:RadioRobotNoValido', ...
        'robot.geometria.radio debe ser un escalar positivo.');
end

radioRobot = double(radioRobot);

%% Configuracion
if ~isstruct(cfg) || ~isscalar(cfg) || ...
        ~isfield(cfg,'seguridad') || ...
        ~isfield(cfg,'prm')
    error('prm:ConfiguracionNoValida', ...
        ['cfg debe ser la estructura obtenida mediante ' ...
         'parametros_generales.m.']);
end

margenEstatico = obtener_no_negativo( ...
    cfg.seguridad,'margenEstatico', ...
    'cfg.seguridad.margenEstatico');

nMuestras = obtener_entero_positivo( ...
    cfg.prm,'nMuestras','cfg.prm.nMuestras');

radioConexion = obtener_positivo( ...
    cfg.prm,'radioConexion','cfg.prm.radioConexion');

maxVecinos = obtener_entero_positivo( ...
    cfg.prm,'maxVecinos','cfg.prm.maxVecinos');

radioConsulta = obtener_positivo( ...
    cfg.prm,'radioConsulta','cfg.prm.radioConsulta');

muestrasExpansion = obtener_entero_positivo( ...
    cfg.prm,'muestrasExpansion','cfg.prm.muestrasExpansion');

maxExpansiones = obtener_entero_no_negativo( ...
    cfg.prm,'maxExpansiones','cfg.prm.maxExpansiones');

%% Parametros internos conservados de la implementacion funcional
parametros = struct();
parametros.radioRobot = radioRobot;
parametros.margenEstatico = margenEstatico;

parametros.nMuestras = nMuestras;
parametros.radioConexion = radioConexion;
parametros.maxVecinos = maxVecinos;
parametros.muestrasExpansion = muestrasExpansion;
parametros.maxExpansiones = maxExpansiones;

parametros.separacionMinimaMuestras = 0.20;
parametros.factorMaximoIntentos = 100;
parametros.probabilidadMuestreoLocal = 0.60;
parametros.radioMuestreoLocal = max( ...
    radioConexion,radioConsulta);

if isfield(cfg,'semilla') && ...
        isnumeric(cfg.semilla) && isscalar(cfg.semilla) && ...
        isreal(cfg.semilla) && isfinite(cfg.semilla)
    parametros.semillaConfigurada = double(cfg.semilla);
else
    parametros.semillaConfigurada = NaN;
end

if isfield(cfg,'version')
    parametros.versionConfiguracion = string(cfg.version);
else
    parametros.versionConfiguracion = "";
end

%% Coherencia del espacio seguro
separacionEstatica = radioRobot+margenEstatico;

limitesSeguros = [ ...
    limites(1)+separacionEstatica, ...
    limites(2)-separacionEstatica, ...
    limites(3)+separacionEstatica, ...
    limites(4)-separacionEstatica];

if limitesSeguros(2) <= limitesSeguros(1) || ...
        limitesSeguros(4) <= limitesSeguros(3)
    error('prm:MapaDemasiadoPequeno', ...
        ['El mapa no admite la huella del robot y el margen ' ...
         'estatico configurado.']);
end

%% Obstaculos dentro del mapa
escala = max(1,max(abs([limites estaticos(:).'])));
tol = 1e-12*escala;

if ~isempty(estaticos)
    extremosX = estaticos(:,1)+estaticos(:,3);
    extremosY = estaticos(:,2)+estaticos(:,4);

    fuera = ...
        estaticos(:,1) < limites(1)-tol | ...
        extremosX > limites(2)+tol | ...
        estaticos(:,2) < limites(3)-tol | ...
        extremosY > limites(4)+tol;

    if any(fuera)
        error('prm:EstaticoFueraDelMapa', ...
            'Todos los obstaculos estaticos deben quedar dentro del mapa.');
    end
end

%% Meta estatica valida
geometriaTemporal = preparar_geometria( ...
    meta,limites,estaticos,parametros);

if ~punto_estatico_libre(meta,geometriaTemporal)
    error('prm:MetaEstaticamenteBloqueada', ...
        ['La meta no es valida considerando el radio del robot y el ' ...
         'margen estatico.']);
end
end

function roadmap = validar_roadmap_anterior( ...
    roadmap,meta,limites,estaticos,parametros,geometria)
%VALIDAR_ROADMAP_ANTERIOR Verifica compatibilidad antes de ampliar.

if ~isstruct(roadmap) || ~isscalar(roadmap)
    error('prm:RoadmapAnteriorNoValida', ...
        'roadmapAnterior debe ser una estructura escalar.');
end

campos = { ...
    'esquema', ...
    'nodos', ...
    'aristas', ...
    'pesos', ...
    'cajasAristas', ...
    'indiceMeta', ...
    'meta', ...
    'limites', ...
    'obstaculosEstaticos', ...
    'radioRobot', ...
    'margenEstatico', ...
    'separacionEstatica', ...
    'radioConexion', ...
    'maxVecinos', ...
    'numeroConstrucciones', ...
    'numeroExpansiones', ...
    'muestrasSolicitadasAcumuladas', ...
    'muestrasAceptadasAcumuladas', ...
    'intentosMuestreoAcumulados'};

for i = 1:numel(campos)
    if ~isfield(roadmap,campos{i})
        error('prm:RoadmapAnteriorIncompleta', ...
            'Falta roadmapAnterior.%s.',campos{i});
    end
end

if string(roadmap.esquema) ~= "roadmap_prm_v1"
    error('prm:EsquemaRoadmapNoSoportado', ...
        'La roadmap anterior no utiliza el esquema roadmap_prm_v1.');
end

%% Matrices principales
if ~isnumeric(roadmap.nodos) || ~isreal(roadmap.nodos) || ...
        size(roadmap.nodos,2) ~= 2 || ...
        isempty(roadmap.nodos) || ...
        any(~isfinite(roadmap.nodos(:)))
    error('prm:NodosRoadmapNoValidos', ...
        'roadmapAnterior.nodos debe ser una matriz N x 2 finita.');
end

roadmap.nodos = double(roadmap.nodos);
numeroNodos = size(roadmap.nodos,1);

if isempty(roadmap.aristas)
    roadmap.aristas = zeros(0,2);
elseif ~isnumeric(roadmap.aristas) || ...
        ~isreal(roadmap.aristas) || ...
        size(roadmap.aristas,2) ~= 2 || ...
        any(~isfinite(roadmap.aristas(:))) || ...
        any(roadmap.aristas(:) ~= floor(roadmap.aristas(:))) || ...
        any(roadmap.aristas(:) < 1) || ...
        any(roadmap.aristas(:) > numeroNodos) || ...
        any(roadmap.aristas(:,1) == roadmap.aristas(:,2))
    error('prm:AristasRoadmapNoValidas', ...
        'roadmapAnterior.aristas contiene indices no validos.');
else
    roadmap.aristas = double(roadmap.aristas);
end

numeroAristas = size(roadmap.aristas,1);

if ~isnumeric(roadmap.pesos) || ~isreal(roadmap.pesos) || ...
        numel(roadmap.pesos) ~= numeroAristas || ...
        any(~isfinite(roadmap.pesos(:))) || ...
        any(roadmap.pesos(:) <= 0)
    if numeroAristas == 0 && isempty(roadmap.pesos)
        roadmap.pesos = zeros(0,1);
    else
        error('prm:PesosRoadmapNoValidos', ...
            'roadmapAnterior.pesos no es coherente con las aristas.');
    end
else
    roadmap.pesos = reshape(double(roadmap.pesos),[],1);
end

if isempty(roadmap.cajasAristas) && numeroAristas == 0
    roadmap.cajasAristas = zeros(0,4);
elseif ~isnumeric(roadmap.cajasAristas) || ...
        ~isreal(roadmap.cajasAristas) || ...
        ~isequal(size(roadmap.cajasAristas),[numeroAristas 4]) || ...
        any(~isfinite(roadmap.cajasAristas(:)))
    error('prm:CajasAristasNoValidas', ...
        ['roadmapAnterior.cajasAristas debe tener una fila por ' ...
         'arista y formato [xmin xmax ymin ymax].']);
else
    roadmap.cajasAristas = double(roadmap.cajasAristas);
end

%% Meta e indice
if ~isnumeric(roadmap.indiceMeta) || ...
        ~isscalar(roadmap.indiceMeta) || ...
        roadmap.indiceMeta ~= floor(roadmap.indiceMeta) || ...
        roadmap.indiceMeta < 1 || ...
        roadmap.indiceMeta > numeroNodos
    error('prm:IndiceMetaNoValido', ...
        'roadmapAnterior.indiceMeta no es valido.');
end

escala = max(1,max(abs([ ...
    roadmap.nodos(:); meta(:); limites(:); estaticos(:)])));

tol = 1e-10*escala;

if norm(roadmap.nodos(roadmap.indiceMeta,:)-meta) > tol || ...
        norm(reshape(double(roadmap.meta),1,2)-meta) > tol
    error('prm:MetaRoadmapIncompatible', ...
        'La meta de la roadmap anterior no coincide con la actual.');
end

%% Entorno y parametros invariantes
comparar_vector( ...
    roadmap.limites,limites,tol,'limites');

comparar_matriz( ...
    roadmap.obstaculosEstaticos,estaticos,tol, ...
    'obstaculosEstaticos');

comparar_escalar( ...
    roadmap.radioRobot,parametros.radioRobot,tol,'radioRobot');

comparar_escalar( ...
    roadmap.margenEstatico,parametros.margenEstatico,tol, ...
    'margenEstatico');

comparar_escalar( ...
    roadmap.separacionEstatica,geometria.separacionEstatica,tol, ...
    'separacionEstatica');

comparar_escalar( ...
    roadmap.radioConexion,parametros.radioConexion,tol, ...
    'radioConexion');

if roadmap.maxVecinos ~= parametros.maxVecinos
    error('prm:MaxVecinosIncompatible', ...
        ['cfg.prm.maxVecinos no coincide con el utilizado para ' ...
         'construir la roadmap.']);
end

%% Contadores
roadmap.numeroConstrucciones = validar_contador( ...
    roadmap.numeroConstrucciones,'numeroConstrucciones');

roadmap.numeroExpansiones = validar_contador( ...
    roadmap.numeroExpansiones,'numeroExpansiones');

roadmap.muestrasSolicitadasAcumuladas = validar_contador( ...
    roadmap.muestrasSolicitadasAcumuladas, ...
    'muestrasSolicitadasAcumuladas');

roadmap.muestrasAceptadasAcumuladas = validar_contador( ...
    roadmap.muestrasAceptadasAcumuladas, ...
    'muestrasAceptadasAcumuladas');

roadmap.intentosMuestreoAcumulados = validar_contador( ...
    roadmap.intentosMuestreoAcumulados, ...
    'intentosMuestreoAcumulados');

roadmap.numeroNodos = numeroNodos;
roadmap.numeroAristas = numeroAristas;
end

function foco = normalizar_foco(foco)
%NORMALIZAR_FOCO Convierte una posicion opcional a [x y].

if isempty(foco)
    foco = [NaN NaN];
    return;
end

if ~isnumeric(foco) || ~isreal(foco) || ...
        numel(foco) < 2 || any(~isfinite(foco(:)))
    error('prm:FocoExpansionNoValido', ...
        ['focoExpansion debe ser un vector real y finito con al menos ' ...
         'las componentes [x y].']);
end

foco = reshape(double(foco),1,[]);
foco = foco(1:2);
end

function comparar_vector(valor,referencia,tol,nombre)
%COMPARAR_VECTOR Comprueba igualdad numerica de vectores.

if ~isnumeric(valor) || numel(valor) ~= numel(referencia) || ...
        any(~isfinite(valor(:))) || ...
        max(abs(double(valor(:))-double(referencia(:)))) > tol
    error('prm:RoadmapIncompatible', ...
        'roadmapAnterior.%s no coincide con el escenario actual.',nombre);
end
end

function comparar_matriz(valor,referencia,tol,nombre)
%COMPARAR_MATRIZ Comprueba igualdad numerica y dimensional de matrices.

if ~isnumeric(valor) || ~isequal(size(valor),size(referencia)) || ...
        any(~isfinite(valor(:)))
    error('prm:RoadmapIncompatible', ...
        'roadmapAnterior.%s no coincide con el escenario actual.',nombre);
end

if ~isempty(valor) && ...
        max(abs(double(valor(:))-double(referencia(:)))) > tol
    error('prm:RoadmapIncompatible', ...
        'roadmapAnterior.%s no coincide con el escenario actual.',nombre);
end
end

function comparar_escalar(valor,referencia,tol,nombre)
%COMPARAR_ESCALAR Comprueba igualdad numerica de escalares.

if ~isnumeric(valor) || ~isscalar(valor) || ...
        ~isreal(valor) || ~isfinite(valor) || ...
        abs(double(valor)-double(referencia)) > tol
    error('prm:RoadmapIncompatible', ...
        ['roadmapAnterior.%s no coincide con la configuracion ' ...
         'actual.'],nombre);
end
end

function valor = validar_contador(valor,nombre)
%VALIDAR_CONTADOR Comprueba un entero no negativo.

if ~isnumeric(valor) || ~isscalar(valor) || ...
        ~isreal(valor) || ~isfinite(valor) || ...
        valor < 0 || valor ~= floor(valor)
    error('prm:ContadorRoadmapNoValido', ...
        'roadmapAnterior.%s debe ser un entero no negativo.',nombre);
end

valor = double(valor);
end

%% ========================================================================
% ESTRUCTURAS VACIAS Y HELPERS NUMERICOS
% ========================================================================

function estadistica = estadistica_muestreo_vacia(numeroSolicitado)
%ESTADISTICA_MUESTREO_VACIA Inicializa los contadores de muestreo.

estadistica = struct();
estadistica.solicitadas = numeroSolicitado;
estadistica.aceptadas = 0;
estadistica.intentos = 0;
estadistica.maximoIntentos = 0;
estadistica.completa = numeroSolicitado == 0;
estadistica.intentosLocales = 0;
estadistica.intentosUniformes = 0;
estadistica.rechazosColision = 0;
estadistica.rechazosProximidad = 0;
end

function estadistica = estadistica_vecinos_vacia()
%ESTADISTICA_VECINOS_VACIA Inicializa el diagnostico de conexiones.

estadistica = struct();
estadistica.vecinosSeleccionados = 0;
estadistica.paresAntesDeUnicos = 0;
estadistica.paresUnicos = 0;
end

function estadistica = estadistica_aristas_vacia()
%ESTADISTICA_ARISTAS_VACIA Inicializa el diagnostico geometrico.

estadistica = struct();
estadistica.candidatas = 0;
estadistica.aceptadas = 0;
estadistica.rechazadasColision = 0;
end

function valor = obtener_positivo(estructura,campo,nombre)
%OBTENER_POSITIVO Recupera un escalar estrictamente positivo.

if ~isfield(estructura,campo) || ...
        ~es_escalar_positivo(estructura.(campo))
    error('prm:ParametroNoValido', ...
        '%s debe ser un escalar numerico positivo.',nombre);
end

valor = double(estructura.(campo));
end

function valor = obtener_no_negativo(estructura,campo,nombre)
%OBTENER_NO_NEGATIVO Recupera un escalar no negativo.

if ~isfield(estructura,campo) || ...
        ~es_escalar_no_negativo(estructura.(campo))
    error('prm:ParametroNoValido', ...
        '%s debe ser un escalar numerico no negativo.',nombre);
end

valor = double(estructura.(campo));
end

function valor = obtener_entero_positivo(estructura,campo,nombre)
%OBTENER_ENTERO_POSITIVO Recupera un entero estrictamente positivo.

valor = obtener_positivo(estructura,campo,nombre);

if valor ~= floor(valor)
    error('prm:ParametroNoEntero', ...
        '%s debe ser un entero positivo.',nombre);
end
end

function valor = obtener_entero_no_negativo(estructura,campo,nombre)
%OBTENER_ENTERO_NO_NEGATIVO Recupera un entero no negativo.

valor = obtener_no_negativo(estructura,campo,nombre);

if valor ~= floor(valor)
    error('prm:ParametroNoEntero', ...
        '%s debe ser un entero no negativo.',nombre);
end
end

function tf = es_escalar_positivo(valor)
%ES_ESCALAR_POSITIVO Comprueba un escalar real, finito y positivo.

tf = isnumeric(valor) && isscalar(valor) && isreal(valor) && ...
    isfinite(valor) && valor > 0;
end

function tf = es_escalar_no_negativo(valor)
%ES_ESCALAR_NO_NEGATIVO Comprueba un escalar real, finito y no negativo.

tf = isnumeric(valor) && isscalar(valor) && isreal(valor) && ...
    isfinite(valor) && valor >= 0;
end
