function [camino, info] = consulta_prm( ...
    roadmap, estadoRobot, obstaculosDinamicos, robot, cfg)
% CONSULTA_PRM Busca un camino en una roadmap PRM persistente.
%
%   camino = CONSULTA_PRM( ...
%       roadmap,estadoRobot,obstaculosDinamicos,robot,cfg)
%
%   [camino,info] = CONSULTA_PRM(...)
%
%   Ejecuta la fase de consulta de un Probabilistic Roadmap (PRM)
%   previamente construido mediante prm.m.
%
%   La roadmap estatica NO se reconstruye ni se modifica. En cada llamada,
%   la funcion:
%
%       1. predice el barrido de los obstaculos dinamicos;
%       2. desactiva temporalmente las aristas PRM bloqueadas;
%       3. conecta la posicion actual del robot con nodos cercanos;
%       4. busca el camino de menor coste hasta la meta;
%       5. simplifica la secuencia de waypoints mediante visibilidad;
%       6. genera un unico arbol grafico reducido para la consulta actual.
%
%   Separacion entre construccion y consulta
%   -----------------------------------------
%   prm.m contiene exclusivamente la geometria persistente del entorno
%   estatico. CONSULTA_PRM incorpora la informacion dinamica sin destruir
%   esa roadmap. Por tanto, el funcionamiento ordinario es:
%
%       [roadmap,infoConstruccion] = prm(...);      % una sola vez
%
%       [camino,infoConsulta] = consulta_prm(...);  % cada replanteamiento
%
%   Solo despues de varios fallos consecutivos el main puede solicitar una
%   ampliacion excepcional mediante una nueva llamada a prm.m.
%
%   Entradas:
%       roadmap
%           Estructura obtenida mediante prm.m. Debe utilizar el esquema:
%
%               roadmap.esquema = "roadmap_prm_v1"
%
%           y contener nodos, aristas, pesos, cajas envolventes, meta,
%           limites y geometria estatica.
%
%       estadoRobot
%           Estado actual [x y] o [x y theta]. Solo se utilizan las dos
%           primeras componentes.
%
%       obstaculosDinamicos
%           Vector de estructuras con los campos:
%
%               id       identificador opcional
%               pos      posicion actual [x y]                [m]
%               vel      velocidad [vx vy]                    [m/s]
%               radio    radio fisico                         [m]
%
%       robot
%           Estructura obtenida mediante configuracion_robot.m. Se utiliza:
%
%               robot.geometria.radio
%
%       cfg
%           Estructura obtenida mediante parametros_generales.m. Se usan:
%
%               cfg.sim.Ts
%               cfg.seguridad.margenEstatico
%               cfg.seguridad.margenDinamico
%               cfg.prediccion.pasos
%               cfg.prediccion.modelo
%               cfg.prm.radioConsulta
%               cfg.prm.vecinosConsulta
%               cfg.visual.activa
%               cfg.visual.mostrarPlanificador
%
%   Salida camino:
%       Matriz K x 2 desde la posicion actual hasta la meta. Si no existe
%       una ruta dinamicamente valida, devuelve zeros(0,2).
%
%   Salida info:
%       Estructura de diagnostico con los campos principales:
%
%           .exito
%           .motivo
%           .inicio
%           .meta
%           .consultaDirecta
%           .analisisAristasRealizado
%           .numeroAristasRoadmap
%           .numeroAristasBloqueadas
%           .numeroAristasActivas
%           .fraccionAristasBloqueadas
%           .mascaraAristasBloqueadas
%           .indicesAristasBloqueadas
%           .aristasBloqueadasPorObstaculo
%           .numeroPruebasGeometricasDinamicas
%           .numeroIntentosConsulta
%           .intentosConsulta
%           .segundoIntentoUtilizado
%           .radioConsultaUsado
%           .vecinosConsultaUsados
%           .numeroConexionesTemporalesInicio
%           .gradoActivoInicio
%           .inicioCoincidenteConNodo
%           .indiceInicioGrafo
%           .indicesCaminoGrafo
%           .costeCaminoGrafo
%           .longitudCaminoBruto
%           .longitudCamino
%           .numeroWaypointsBruto
%           .numeroWaypointsFinal
%           .horizontePrediccion
%           .vistaPlanificador
%
%   Prediccion dinamica:
%       Se utiliza el mismo modelo que rrt_star.m y replanificacion.m:
%
%           pFutura = pActual + ...
%               cfg.prediccion.pasos*cfg.sim.Ts*velocidad
%
%       Una arista se considera bloqueada cuando la distancia entre dicha
%       arista y el segmento barrido por un obstaculo es menor o igual que:
%
%           radioRobot + radioObstaculo + ...
%               cfg.seguridad.margenDinamico
%
%   Filtro de aristas:
%       roadmap.cajasAristas se utiliza como filtro AABB. La distancia
%       exacta entre segmentos solo se calcula para las aristas cuyas cajas
%       pueden intersectar el barrido ampliado del obstaculo.
%
%   Conexion del robot:
%       Primero se utiliza cfg.prm.radioConsulta y
%       cfg.prm.vecinosConsulta. Si no se encuentra camino, se realiza un
%       segundo intento sin reconstruir la roadmap, con:
%
%           radio = 1.45*cfg.prm.radioConsulta
%           vecinos = 2*cfg.prm.vecinosConsulta
%
%       El segundo intento solo se ejecuta tras fallar el primero.
%
%   Simplificacion:
%       La ruta obtenida sobre el grafo se simplifica conectando cada
%       waypoint con el punto visible mas lejano. Cada atajo vuelve a
%       comprobar obstaculos estaticos y barridos dinamicos, por lo que no
%       se introduce ninguna arista insegura.
%
%   Visualizacion sin maraña de roadmaps
%   -------------------------------------
%   La roadmap completa se utiliza internamente para shortestpath, pero no
%   se envia completa a la figura. La salida:
%
%       info.vistaPlanificador
%
%   contiene un unico arbol de expansion del componente alcanzable desde
%   el robot. En modo visual, el campo .actualizar vale true en cada
%   consulta, incluso si la planificacion falla.
%
%   actualizar_graficos.m vaciara primero el objeto gris anterior y
%   dibujara exclusivamente este nuevo arbol. De este modo:
%
%       - nunca se acumulan varias consultas PRM;
%       - nunca se muestra una roadmap antigua como si siguiera vigente;
%       - la representacion conserva una sola arista por nodo visitado.
%
%   En modo batch no se construye el arbol grafico, evitando incluir coste
%   puramente visual en las ejecuciones estadisticas.
%
%   Ejemplo:
%
%       cfg = parametros_generales("visual");
%       escenario = escenarios("media");
%       robot = configuracion_robot();
%
%       rng(cfg.semilla,'twister');
%
%       [roadmap,infoConstruccion] = prm( ...
%           escenario.meta, ...
%           escenario.limites, ...
%           escenario.obstaculosEstaticos, ...
%           robot,cfg);
%
%       relojConsulta = tic;
%
%       [camino,infoConsulta] = consulta_prm( ...
%           roadmap, ...
%           escenario.inicio, ...
%           escenario.obstaculosDinamicos, ...
%           robot,cfg);
%
%       tiempoConsulta = toc(relojConsulta);
%
%       vistaPlanificador = infoConsulta.vistaPlanificador;
%
%   Esta funcion utiliza:
%       - distancia_punto_segmento.m
%       - distancia_segmentos.m
%       - segmento_rectangulo.m
%
%   No ejecuta drawnow, pause, tic, toc ni rng.

%% Validacion y preparacion
[roadmap, inicio, dinamicos, parametros] = validar_entradas( ...
    roadmap,estadoRobot,obstaculosDinamicos,robot,cfg);

geometria = preparar_geometria( ...
    roadmap,inicio,dinamicos,parametros);

camino = zeros(0,2);
info = crear_info_base(roadmap,inicio,dinamicos,parametros,geometria);

%% ========================================================================
% COMPROBACION DE INICIO Y META
% ========================================================================

[libreInicio,tipoInicio,indiceInicioBloqueado] = ...
    punto_libre(inicio,geometria);

if ~libreInicio
    info.motivo = motivo_punto_no_libre( ...
        "inicio",tipoInicio,indiceInicioBloqueado,geometria);
    info = sincronizar_vista(info);
    return;
end

[libreMeta,tipoMeta,indiceMetaBloqueada] = ...
    punto_libre(roadmap.meta,geometria);

if ~libreMeta
    info.motivo = motivo_punto_no_libre( ...
        "meta",tipoMeta,indiceMetaBloqueada,geometria);
    info = sincronizar_vista(info);
    return;
end

%% Robot exactamente en la meta
if norm(inicio-roadmap.meta) <= geometria.toleranciaPosicion
    camino = inicio;

    info.exito = true;
    info.motivo = "inicio_en_meta";
    info.consultaDirecta = true;
    info.costeCaminoGrafo = 0;
    info.longitudCaminoBruto = 0;
    info.longitudCamino = 0;
    info.numeroWaypointsBruto = 1;
    info.numeroWaypointsFinal = 1;

    info.vistaPlanificador = vista_camino_directo( ...
        inicio,inicio,parametros.generarVista);

    info = sincronizar_vista(info);
    return;
end

%% Conexion recta: solucion geometricamente minima
if arista_libre(inicio,roadmap.meta,geometria)
    camino = [inicio;roadmap.meta];
    longitudDirecta = norm(roadmap.meta-inicio);

    info.exito = true;
    info.motivo = "conexion_directa";
    info.consultaDirecta = true;
    info.costeCaminoGrafo = longitudDirecta;
    info.longitudCaminoBruto = longitudDirecta;
    info.longitudCamino = longitudDirecta;
    info.numeroWaypointsBruto = 2;
    info.numeroWaypointsFinal = 2;

    info.vistaPlanificador = vista_camino_directo( ...
        inicio,roadmap.meta,parametros.generarVista);

    info = sincronizar_vista(info);
    return;
end

%% ========================================================================
% INVALIDACION TEMPORAL DE ARISTAS DINAMICAS
% ========================================================================

[mascaraBloqueadas,estadisticaBloqueo] = ...
    detectar_aristas_bloqueadas(roadmap,geometria);

aristasActivas = roadmap.aristas(~mascaraBloqueadas,:);
pesosActivos = roadmap.pesos(~mascaraBloqueadas);

info.analisisAristasRealizado = true;
info.numeroAristasBloqueadas = nnz(mascaraBloqueadas);
info.numeroAristasActivas = size(aristasActivas,1);

if roadmap.numeroAristas > 0
    info.fraccionAristasBloqueadas = ...
        info.numeroAristasBloqueadas/roadmap.numeroAristas;
else
    info.fraccionAristasBloqueadas = NaN;
end

info.mascaraAristasBloqueadas = mascaraBloqueadas;
info.indicesAristasBloqueadas = find(mascaraBloqueadas);
info.aristasBloqueadasPorObstaculo = ...
    estadisticaBloqueo.bloqueadasPorObstaculo;
info.numeroCandidatosAABB = estadisticaBloqueo.candidatosAABB;
info.numeroPruebasGeometricasDinamicas = ...
    estadisticaBloqueo.pruebasExactas;

%% ========================================================================
% PRIMER INTENTO DE CONSULTA
% ========================================================================

resultadoPrimero = resolver_intento( ...
    roadmap,inicio,aristasActivas,pesosActivos,geometria, ...
    parametros.radioConsulta,parametros.vecinosConsulta,1);

resultadosIntento = resultadoPrimero;
resultadoFinal = resultadoPrimero;

%% Segundo intento ampliado, sin reconstruir la roadmap
if ~resultadoPrimero.exito
    radioSegundo = ...
        parametros.factorSegundoRadio*parametros.radioConsulta;

    vecinosSegundo = min( ...
        max(0,size(roadmap.nodos,1)-1), ...
        parametros.factorSegundoVecinos*parametros.vecinosConsulta);

    cambioReal = ...
        radioSegundo > parametros.radioConsulta+geometria.tolerancia || ...
        vecinosSegundo > parametros.vecinosConsulta;

    if cambioReal
        resultadoSegundo = resolver_intento( ...
            roadmap,inicio,aristasActivas,pesosActivos,geometria, ...
            radioSegundo,vecinosSegundo,2);

        resultadosIntento(2,1) = resultadoSegundo;
        resultadoFinal = resultadoSegundo;
        info.segundoIntentoUtilizado = true;
    end
end

%% Diagnostico de los intentos
info.numeroIntentosConsulta = numel(resultadosIntento);
info.intentosConsulta = extraer_diagnosticos(resultadosIntento);

info.radioConsultaUsado = resultadoFinal.radioConsulta;
info.vecinosConsultaUsados = resultadoFinal.maxConexiones;
info.numeroConexionesTemporalesInicio = ...
    resultadoFinal.numeroConexionesTemporales;
info.gradoActivoInicio = resultadoFinal.gradoActivoInicio;
info.inicioCoincidenteConNodo = ...
    resultadoFinal.inicioCoincidenteConNodo;
info.indiceInicioGrafo = resultadoFinal.indiceInicioGrafo;
info.numeroNodosGrafoConsulta = ...
    size(resultadoFinal.nodosGrafo,1);
info.numeroAristasGrafoConsulta = ...
    size(resultadoFinal.aristasGrafo,1);

%% La vista corresponde siempre al intento vigente mas reciente
info.vistaPlanificador = crear_vista_consulta( ...
    resultadoFinal,parametros.generarVista);

%% ========================================================================
% RESULTADO DE SHORTESTPATH
% ========================================================================

if ~resultadoFinal.exito
    if resultadoFinal.numeroConexionesTemporales == 0 && ...
            resultadoFinal.gradoActivoInicio == 0
        info.motivo = "sin_conexiones_inicio";
    else
        info.motivo = "sin_camino_hasta_meta";
    end

    info = sincronizar_vista(info);
    return;
end

indicesCamino = resultadoFinal.indicesCamino;
caminoBruto = resultadoFinal.nodosGrafo(indicesCamino,:);

% Conserva exactamente las coordenadas actuales y la meta almacenada.
caminoBruto(1,:) = inicio;
caminoBruto(end,:) = roadmap.meta;

if parametros.simplificarCamino
    camino = simplificar_camino(caminoBruto,geometria);
else
    camino = caminoBruto;
end

camino(1,:) = inicio;
camino(end,:) = roadmap.meta;

info.exito = true;
info.motivo = "camino_encontrado";
info.indicesCaminoGrafo = indicesCamino(:);
info.costeCaminoGrafo = resultadoFinal.costeCamino;
info.longitudCaminoBruto = longitud_camino(caminoBruto);
info.longitudCamino = longitud_camino(camino);
info.numeroWaypointsBruto = size(caminoBruto,1);
info.numeroWaypointsFinal = size(camino,1);
info.caminoBruto = caminoBruto;

info = sincronizar_vista(info);
end

%% ========================================================================
% RESOLUCION DE UN INTENTO DE CONEXION
% ========================================================================

function resultado = resolver_intento( ...
    roadmap,inicio,aristasActivas,pesosActivos,geometria, ...
    radioConsulta,maxConexiones,numeroIntento)
%RESOLVER_INTENTO Conecta el robot y ejecuta shortestpath.

resultado = estructura_resultado_intento();
resultado.numeroIntento = numeroIntento;
resultado.radioConsulta = radioConsulta;
resultado.maxConexiones = maxConexiones;

numeroNodosRoadmap = size(roadmap.nodos,1);

%% Coincidencia de la posicion actual con un nodo persistente
if numeroNodosRoadmap > 0
    diferencias = roadmap.nodos-inicio;
    distancias2 = sum(diferencias.^2,2);
    [distancia2Minima,indiceCoincidente] = min(distancias2);
else
    distancia2Minima = inf;
    indiceCoincidente = NaN;
end

if distancia2Minima <= geometria.toleranciaPosicion^2
    resultado.inicioCoincidenteConNodo = true;
    resultado.indiceInicioGrafo = indiceCoincidente;
    resultado.nodosGrafo = roadmap.nodos;
else
    resultado.inicioCoincidenteConNodo = false;
    resultado.indiceInicioGrafo = numeroNodosRoadmap+1;
    resultado.nodosGrafo = [roadmap.nodos;inicio];
end

%% Conexiones temporales del robot
[destinos,pesosConexion,estadisticaConexion] = ...
    conectar_inicio( ...
        inicio,roadmap.nodos,resultado.indiceInicioGrafo, ...
        resultado.inicioCoincidenteConNodo,radioConsulta, ...
        maxConexiones,geometria);

if isempty(destinos)
    aristasConsulta = zeros(0,2);
    pesosConsulta = zeros(0,1);
else
    aristasConsulta = [ ...
        repmat(resultado.indiceInicioGrafo,numel(destinos),1), ...
        destinos(:)];

    pesosConsulta = pesosConexion(:);
end

%% Evita duplicar aristas activas cuando el inicio coincide con un nodo
if ~isempty(aristasConsulta) && ~isempty(aristasActivas)
    paresActivos = sort(aristasActivas,2);
    paresConsulta = sort(aristasConsulta,2);

    duplicada = ismember(paresConsulta,paresActivos,'rows');

    aristasConsulta = aristasConsulta(~duplicada,:);
    pesosConsulta = pesosConsulta(~duplicada);

    estadisticaConexion.duplicadasActivas = nnz(duplicada);
else
    estadisticaConexion.duplicadasActivas = 0;
end

resultado.aristasConsulta = aristasConsulta;
resultado.pesosConsulta = pesosConsulta;
resultado.numeroConexionesTemporales = size(aristasConsulta,1);
resultado.estadisticaConexion = estadisticaConexion;

%% Grafo activo de esta consulta
resultado.aristasGrafo = [aristasActivas;aristasConsulta];
resultado.pesosGrafo = [pesosActivos;pesosConsulta];

resultado.gradoActivoInicio = nnz( ...
    resultado.aristasGrafo(:,1) == resultado.indiceInicioGrafo | ...
    resultado.aristasGrafo(:,2) == resultado.indiceInicioGrafo);

%% Un nodo de consulta nuevo necesita al menos una conexion
if ~resultado.inicioCoincidenteConNodo && ...
        resultado.numeroConexionesTemporales == 0
    resultado.motivo = "sin_conexiones_inicio";
    return;
end

if isempty(resultado.aristasGrafo)
    resultado.motivo = "grafo_activo_sin_aristas";
    return;
end

%% Busqueda de camino minimo
numeroNodosGrafo = size(resultado.nodosGrafo,1);

try
    grafo = graph( ...
        resultado.aristasGrafo(:,1), ...
        resultado.aristasGrafo(:,2), ...
        resultado.pesosGrafo, ...
        numeroNodosGrafo);

    [indicesCamino,costeCamino] = shortestpath( ...
        grafo, ...
        resultado.indiceInicioGrafo, ...
        roadmap.indiceMeta, ...
        'Method','positive');
catch errorOriginal
    error('consulta_prm:ErrorBusquedaGrafo', ...
        ['No se pudo construir o consultar el grafo PRM activo. ' ...
         'Mensaje original: %s'],errorOriginal.message);
end

resultado.indicesCamino = indicesCamino(:);
resultado.costeCamino = costeCamino;
resultado.exito = ...
    ~isempty(indicesCamino) && isfinite(costeCamino);

if resultado.exito
    resultado.motivo = "camino_encontrado";
else
    resultado.motivo = "sin_camino_hasta_meta";
end
end

function [destinos,pesos,estadistica] = conectar_inicio( ...
    inicio,nodosRoadmap,indiceInicioGrafo,inicioCoincidente, ...
    radioConsulta,maxConexiones,geometria)
%CONECTAR_INICIO Selecciona conexiones seguras por distancia.
%
% A diferencia de seleccionar primero un numero fijo de candidatos y
% descartarlos despues por colision, esta funcion recorre los nodos dentro
% del radio hasta aceptar maxConexiones conexiones seguras. Asi, una arista
% bloqueada no impide examinar el siguiente vecino disponible.

estadistica = estructura_estadistica_conexion();

destinos = zeros(0,1);
pesos = zeros(0,1);

if isempty(nodosRoadmap) || maxConexiones <= 0
    return;
end

diferencias = nodosRoadmap-inicio;
distancias2 = sum(diferencias.^2,2);

[distancias2Ordenadas,orden] = sort(distancias2,'ascend');

dentroRadio = ...
    distancias2Ordenadas <= ...
    (radioConsulta+geometria.tolerancia)^2;

orden = orden(dentroRadio);
distancias2Ordenadas = distancias2Ordenadas(dentroRadio);

estadistica.candidatosDentroRadio = numel(orden);

bufferDestinos = zeros(min(maxConexiones,numel(orden)),1);
bufferPesos = zeros(min(maxConexiones,numel(orden)),1);
numeroAceptadas = 0;

for posicion = 1:numel(orden)
    indiceNodo = orden(posicion);
    distancia = sqrt(distancias2Ordenadas(posicion));

    %% Evita conectar un nodo consigo mismo
    if inicioCoincidente && indiceNodo == indiceInicioGrafo
        estadistica.rechazosCoincidencia = ...
            estadistica.rechazosCoincidencia+1;
        continue;
    end

    if distancia <= geometria.toleranciaPosicion
        estadistica.rechazosCoincidencia = ...
            estadistica.rechazosCoincidencia+1;
        continue;
    end

    estadistica.candidatosEvaluados = ...
        estadistica.candidatosEvaluados+1;

    [libre,tipoBloqueo] = arista_libre_detallada( ...
        inicio,nodosRoadmap(indiceNodo,:),geometria);

    if ~libre
        if tipoBloqueo == "estatico" || tipoBloqueo == "limite"
            estadistica.rechazosEstaticos = ...
                estadistica.rechazosEstaticos+1;
        elseif tipoBloqueo == "dinamico"
            estadistica.rechazosDinamicos = ...
                estadistica.rechazosDinamicos+1;
        end
        continue;
    end

    numeroAceptadas = numeroAceptadas+1;
    bufferDestinos(numeroAceptadas) = indiceNodo;
    bufferPesos(numeroAceptadas) = distancia;

    if numeroAceptadas >= maxConexiones
        break;
    end
end

destinos = bufferDestinos(1:numeroAceptadas);
pesos = bufferPesos(1:numeroAceptadas);

estadistica.aceptadas = numeroAceptadas;
end

%% ========================================================================
% BLOQUEO DINAMICO DE LA ROADMAP
% ========================================================================

function [bloqueadas,estadistica] = ...
    detectar_aristas_bloqueadas(roadmap,geometria)
%DETECTAR_ARISTAS_BLOQUEADAS Invalida aristas mediante AABB y distancia.

numeroAristas = size(roadmap.aristas,1);
numeroDinamicos = size(geometria.posicionesDinamicasActuales,1);

bloqueadas = false(numeroAristas,1);

estadistica = struct();
estadistica.candidatosAABB = 0;
estadistica.pruebasExactas = 0;
estadistica.bloqueadasPorObstaculo = ...
    zeros(numeroDinamicos,1);

if numeroAristas == 0 || numeroDinamicos == 0
    return;
end

for i = 1:numeroDinamicos
    posicionActual = geometria.posicionesDinamicasActuales(i,:);
    posicionFutura = geometria.posicionesDinamicasFuturas(i,:);
    separacion = geometria.separacionesDinamicas(i);

    xminBarrido = min(posicionActual(1),posicionFutura(1))-separacion;
    xmaxBarrido = max(posicionActual(1),posicionFutura(1))+separacion;
    yminBarrido = min(posicionActual(2),posicionFutura(2))-separacion;
    ymaxBarrido = max(posicionActual(2),posicionFutura(2))+separacion;

    cajas = roadmap.cajasAristas;
    tol = geometria.tolerancia;

    candidata = ...
        cajas(:,1) <= xmaxBarrido+tol & ...
        cajas(:,2) >= xminBarrido-tol & ...
        cajas(:,3) <= ymaxBarrido+tol & ...
        cajas(:,4) >= yminBarrido-tol & ...
        ~bloqueadas;

    indicesCandidatos = find(candidata);

    estadistica.candidatosAABB = ...
        estadistica.candidatosAABB+numel(indicesCandidatos);

    nuevasBloqueadas = 0;

    for posicion = 1:numel(indicesCandidatos)
        indiceArista = indicesCandidatos(posicion);
        extremos = roadmap.aristas(indiceArista,:);

        a = roadmap.nodos(extremos(1),:);
        b = roadmap.nodos(extremos(2),:);

        estadistica.pruebasExactas = ...
            estadistica.pruebasExactas+1;

        distancia = distancia_segmentos( ...
            a,b,posicionActual,posicionFutura);

        if distancia <= separacion+tol
            bloqueadas(indiceArista) = true;
            nuevasBloqueadas = nuevasBloqueadas+1;
        end
    end

    estadistica.bloqueadasPorObstaculo(i) = nuevasBloqueadas;
end
end

%% ========================================================================
% SIMPLIFICACION DEL CAMINO
% ========================================================================

function caminoSimplificado = simplificar_camino(camino,geometria)
%SIMPLIFICAR_CAMINO Elimina waypoints visibles de forma redundante.

numeroPuntos = size(camino,1);

if numeroPuntos <= 2
    caminoSimplificado = camino;
    return;
end

buffer = zeros(numeroPuntos,2);
numeroConservados = 1;
buffer(1,:) = camino(1,:);

indiceActual = 1;

while indiceActual < numeroPuntos
    siguiente = indiceActual+1;

    % Se intenta conectar primero con el punto mas lejano.
    for candidato = numeroPuntos:-1:indiceActual+1
        if arista_libre( ...
                camino(indiceActual,:),camino(candidato,:),geometria)
            siguiente = candidato;
            break;
        end
    end

    numeroConservados = numeroConservados+1;
    buffer(numeroConservados,:) = camino(siguiente,:);
    indiceActual = siguiente;
end

caminoSimplificado = buffer(1:numeroConservados,:);
end

function longitud = longitud_camino(camino)
%LONGITUD_CAMINO Longitud euclidiana de una polilinea.

if size(camino,1) < 2
    longitud = 0;
    return;
end

desplazamientos = diff(camino,1,1);
longitud = sum(hypot( ...
    desplazamientos(:,1),desplazamientos(:,2)));
end

%% ========================================================================
% REPRESENTACION GRAFICA REDUCIDA
% ========================================================================

function vista = crear_vista_consulta(resultado,generarVista)
%CREAR_VISTA_CONSULTA Genera un arbol del componente alcanzable.

vista = vista_vacia(generarVista);

if ~generarVista || isempty(resultado.nodosGrafo) || ...
        ~isfinite(resultado.indiceInicioGrafo)
    return;
end

[indicesVisitados,aristasArbol] = arbol_expansion_grafico( ...
    resultado.aristasGrafo, ...
    size(resultado.nodosGrafo,1), ...
    resultado.indiceInicioGrafo);

if isempty(indicesVisitados)
    indicesVisitados = resultado.indiceInicioGrafo;
end

mapaIndices = zeros(size(resultado.nodosGrafo,1),1);
mapaIndices(indicesVisitados) = (1:numel(indicesVisitados)).';

vista.nodos = resultado.nodosGrafo(indicesVisitados,:);

if isempty(aristasArbol)
    vista.aristas = zeros(0,2);
else
    vista.aristas = [ ...
        mapaIndices(aristasArbol(:,1)), ...
        mapaIndices(aristasArbol(:,2))];
end

vista.numeroNodos = size(vista.nodos,1);
vista.numeroAristas = size(vista.aristas,1);
vista.tipo = "arbol_consulta_prm";
end

function [ordenVisitados,aristasArbol] = arbol_expansion_grafico( ...
    aristas,numeroNodos,indiceRaiz)
%ARBOL_EXPANSION_GRAFICO Construye un BFS con una arista por nodo.

ordenVisitados = zeros(0,1);
aristasArbol = zeros(0,2);

if numeroNodos < 1 || indiceRaiz < 1 || indiceRaiz > numeroNodos
    return;
end

if isempty(aristas)
    ordenVisitados = indiceRaiz;
    return;
end

adyacencia = sparse( ...
    [aristas(:,1);aristas(:,2)], ...
    [aristas(:,2);aristas(:,1)], ...
    true(2*size(aristas,1),1), ...
    numeroNodos,numeroNodos);

visitado = false(numeroNodos,1);
cola = zeros(numeroNodos,1);
orden = zeros(numeroNodos,1);
bufferAristas = zeros(max(0,numeroNodos-1),2);

cabeza = 1;
final = 1;
numeroVisitados = 1;
numeroAristas = 0;

cola(1) = indiceRaiz;
orden(1) = indiceRaiz;
visitado(indiceRaiz) = true;

while cabeza <= final
    nodo = cola(cabeza);
    cabeza = cabeza+1;

    vecinos = find(adyacencia(nodo,:));

    for vecino = reshape(vecinos,1,[])
        if visitado(vecino)
            continue;
        end

        visitado(vecino) = true;

        final = final+1;
        cola(final) = vecino;

        numeroVisitados = numeroVisitados+1;
        orden(numeroVisitados) = vecino;

        numeroAristas = numeroAristas+1;
        bufferAristas(numeroAristas,:) = [nodo vecino];
    end
end

ordenVisitados = orden(1:numeroVisitados);
aristasArbol = bufferAristas(1:numeroAristas,:);
end

function vista = vista_camino_directo(inicio,meta,generarVista)
%VISTA_CAMINO_DIRECTO Representa la consulta trivial actual.

vista = vista_vacia(generarVista);

if ~generarVista
    return;
end

if norm(inicio-meta) <= eps(max(1,max(abs([inicio meta]))))
    vista.nodos = inicio;
    vista.aristas = zeros(0,2);
else
    vista.nodos = [inicio;meta];
    vista.aristas = [1 2];
end

vista.numeroNodos = size(vista.nodos,1);
vista.numeroAristas = size(vista.aristas,1);
vista.tipo = "consulta_directa_prm";
end

function vista = vista_vacia(generarVista)
%VISTA_VACIA Inicializa la salida destinada a actualizar_graficos.m.

vista = struct();
vista.actualizar = logical(generarVista);
vista.nodos = zeros(0,2);
vista.aristas = zeros(0,2);
vista.numeroNodos = 0;
vista.numeroAristas = 0;
vista.tipo = "consulta_prm_vacia";
end

function info = sincronizar_vista(info)
%SINCRONIZAR_VISTA Conserva alias utiles para integracion y diagnostico.

info.nodosVisuales = info.vistaPlanificador.nodos;
info.aristasVisuales = info.vistaPlanificador.aristas;

% Alias compatibles con el codigo funcional de referencia.
info.displayNodes = info.vistaPlanificador.nodos;
info.displayEdges = info.vistaPlanificador.aristas;
end

%% ========================================================================
% GEOMETRIA DE LA CONSULTA
% ========================================================================

function geometria = preparar_geometria( ...
    roadmap,inicio,dinamicos,parametros)
%PREPARAR_GEOMETRIA Precalcula margenes y barridos dinamicos.

geometria = struct();
geometria.inicio = inicio;
geometria.meta = roadmap.meta;
geometria.limites = roadmap.limites;
geometria.estaticos = roadmap.obstaculosEstaticos;
geometria.separacionEstatica = roadmap.separacionEstatica;

geometria.limitesSeguros = [ ...
    roadmap.limites(1)+roadmap.separacionEstatica, ...
    roadmap.limites(2)-roadmap.separacionEstatica, ...
    roadmap.limites(3)+roadmap.separacionEstatica, ...
    roadmap.limites(4)-roadmap.separacionEstatica];

geometria.horizontePrediccion = ...
    parametros.pasosPrediccion*parametros.Ts;

numeroDinamicos = numel(dinamicos);

geometria.posicionesDinamicasActuales = zeros(numeroDinamicos,2);
geometria.posicionesDinamicasFuturas = zeros(numeroDinamicos,2);
geometria.separacionesDinamicas = zeros(numeroDinamicos,1);
geometria.idsDinamicos = strings(numeroDinamicos,1);

for i = 1:numeroDinamicos
    geometria.posicionesDinamicasActuales(i,:) = dinamicos(i).pos;
    geometria.posicionesDinamicasFuturas(i,:) = ...
        dinamicos(i).pos+ ...
        geometria.horizontePrediccion*dinamicos(i).vel;

    geometria.separacionesDinamicas(i) = ...
        parametros.radioRobot+dinamicos(i).radio+ ...
        parametros.margenDinamico;

    geometria.idsDinamicos(i) = id_dinamico(dinamicos,i);
end

valoresEscala = [ ...
    inicio(:); ...
    roadmap.meta(:); ...
    roadmap.limites(:); ...
    roadmap.obstaculosEstaticos(:); ...
    geometria.posicionesDinamicasActuales(:); ...
    geometria.posicionesDinamicasFuturas(:); ...
    roadmap.separacionEstatica; ...
    geometria.separacionesDinamicas(:)];

escala = max(1,max(abs(valoresEscala)));

geometria.tolerancia = 1e-12*escala;
geometria.toleranciaPosicion = 1e-10*escala;
end

function [libre,tipo,indice] = punto_libre(punto,geometria)
%PUNTO_LIBRE Comprueba limites, estaticos y barridos dinamicos.

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

function libre = arista_libre(a,b,geometria)
%ARISTA_LIBRE Comprueba seguridad estatica y dinamica.

[libre,~] = arista_libre_detallada(a,b,geometria);
end

function [libre,tipoBloqueo] = ...
    arista_libre_detallada(a,b,geometria)
%ARISTA_LIBRE_DETALLADA Identifica el tipo de bloqueo de un segmento.

libre = false;
tipoBloqueo = "ninguno";

tol = geometria.tolerancia;
L = geometria.limitesSeguros;

if a(1) < L(1)-tol || a(1) > L(2)+tol || ...
        a(2) < L(3)-tol || a(2) > L(4)+tol || ...
        b(1) < L(1)-tol || b(1) > L(2)+tol || ...
        b(2) < L(3)-tol || b(2) > L(4)+tol
    tipoBloqueo = "limite";
    return;
end

for i = 1:size(geometria.estaticos,1)
    rectangulo = geometria.estaticos(i,:);

    if cajas_separadas_segmento_rectangulo( ...
            a,b,rectangulo,geometria.separacionEstatica+tol)
        continue;
    end

    distancia = distancia_segmento_rectangulo( ...
        a,b,rectangulo,tol);

    if distancia <= geometria.separacionEstatica+tol
        tipoBloqueo = "estatico";
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

    if distancia <= separacion+tol
        tipoBloqueo = "dinamico";
        return;
    end
end

libre = true;
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

function distancia = distancia_punto_rectangulo_local(punto,rectangulo)
%DISTANCIA_PUNTO_RECTANGULO_LOCAL Distancia de punto a rectangulo.

xmin = rectangulo(1);
ymin = rectangulo(2);
xmax = xmin+rectangulo(3);
ymax = ymin+rectangulo(4);

dx = max([xmin-punto(1),0,punto(1)-xmax]);
dy = max([ymin-punto(2),0,punto(2)-ymax]);

distancia = hypot(dx,dy);
end

function distancia = distancia_segmento_rectangulo( ...
    a,b,rectangulo,tol)
%DISTANCIA_SEGMENTO_RECTANGULO Distancia exacta segmento-rectangulo.

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

lados = [1 2;2 3;3 4;4 1];
distancia = inf;

for i = 1:4
    c = esquinas(lados(i,1),:);
    d = esquinas(lados(i,2),:);

    distancia = min( ...
        distancia,distancia_segmentos(a,b,c,d));
end
end

function motivo = motivo_punto_no_libre( ...
    nombrePunto,tipo,indice,geometria)
%MOTIVO_PUNTO_NO_LIBRE Construye un diagnostico legible.

switch tipo
    case "limite"
        motivo = nombrePunto+"_fuera_de_limites_seguros";

    case "estatico"
        motivo = nombrePunto+"_bloqueado_por_estatico_S"+indice;

    case "dinamico"
        if isfinite(indice) && indice >= 1 && ...
                indice <= numel(geometria.idsDinamicos)
            id = geometria.idsDinamicos(indice);
        else
            id = "D"+indice;
        end

        motivo = nombrePunto+"_bloqueado_por_dinamico_"+id;

    otherwise
        motivo = nombrePunto+"_no_libre";
end
end

function id = id_dinamico(dinamicos,indice)
%ID_DINAMICO Recupera el identificador almacenado o genera D1, D2, ...

if isfield(dinamicos(indice),'id') && ...
        strlength(strtrim(string(dinamicos(indice).id))) > 0
    id = strtrim(string(dinamicos(indice).id));
else
    id = "D"+indice;
end
end

%% ========================================================================
% SALIDAS Y DIAGNOSTICO
% ========================================================================

function info = crear_info_base( ...
    roadmap,inicio,dinamicos,parametros,geometria)
%CREAR_INFO_BASE Define un contrato estable incluso en fallos tempranos.

info = struct();

info.exito = false;
info.motivo = "";

info.inicio = inicio;
info.meta = roadmap.meta;
info.consultaDirecta = false;

info.analisisAristasRealizado = false;
info.numeroNodosRoadmap = size(roadmap.nodos,1);
info.numeroAristasRoadmap = size(roadmap.aristas,1);
info.numeroAristasBloqueadas = NaN;
info.numeroAristasActivas = NaN;
info.fraccionAristasBloqueadas = NaN;
info.mascaraAristasBloqueadas = ...
    false(size(roadmap.aristas,1),1);
info.indicesAristasBloqueadas = zeros(0,1);
info.aristasBloqueadasPorObstaculo = ...
    zeros(numel(dinamicos),1);
info.numeroCandidatosAABB = 0;
info.numeroPruebasGeometricasDinamicas = 0;

info.numeroIntentosConsulta = 0;
info.intentosConsulta = repmat( ...
    estructura_diagnostico_intento(),0,1);
info.segundoIntentoUtilizado = false;

info.radioConsultaUsado = NaN;
info.vecinosConsultaUsados = NaN;
info.numeroConexionesTemporalesInicio = 0;
info.gradoActivoInicio = 0;
info.inicioCoincidenteConNodo = false;
info.indiceInicioGrafo = NaN;
info.numeroNodosGrafoConsulta = 0;
info.numeroAristasGrafoConsulta = 0;

info.indicesCaminoGrafo = zeros(0,1);
info.costeCaminoGrafo = inf;
info.longitudCaminoBruto = NaN;
info.longitudCamino = NaN;
info.numeroWaypointsBruto = 0;
info.numeroWaypointsFinal = 0;
info.caminoBruto = zeros(0,2);

info.horizontePrediccion = geometria.horizontePrediccion;
info.modeloPrediccion = parametros.modeloPrediccion;
info.numeroObstaculosDinamicos = numel(dinamicos);

info.parametros = struct();
info.parametros.radioConsulta = parametros.radioConsulta;
info.parametros.vecinosConsulta = parametros.vecinosConsulta;
info.parametros.factorSegundoRadio = ...
    parametros.factorSegundoRadio;
info.parametros.factorSegundoVecinos = ...
    parametros.factorSegundoVecinos;
info.parametros.radioRobot = parametros.radioRobot;
info.parametros.margenEstatico = parametros.margenEstatico;
info.parametros.margenDinamico = parametros.margenDinamico;
info.parametros.separacionEstatica = roadmap.separacionEstatica;
info.parametros.simplificarCamino = parametros.simplificarCamino;

info.vistaPlanificador = vista_vacia(parametros.generarVista);
info.nodosVisuales = zeros(0,2);
info.aristasVisuales = zeros(0,2);
info.displayNodes = zeros(0,2);
info.displayEdges = zeros(0,2);
end

function resultado = estructura_resultado_intento()
%ESTRUCTURA_RESULTADO_INTENTO Inicializa los datos internos de una prueba.

resultado = struct();
resultado.numeroIntento = 0;
resultado.exito = false;
resultado.motivo = "";
resultado.radioConsulta = NaN;
resultado.maxConexiones = 0;
resultado.inicioCoincidenteConNodo = false;
resultado.indiceInicioGrafo = NaN;
resultado.nodosGrafo = zeros(0,2);
resultado.aristasGrafo = zeros(0,2);
resultado.pesosGrafo = zeros(0,1);
resultado.aristasConsulta = zeros(0,2);
resultado.pesosConsulta = zeros(0,1);
resultado.numeroConexionesTemporales = 0;
resultado.gradoActivoInicio = 0;
resultado.indicesCamino = zeros(0,1);
resultado.costeCamino = inf;
resultado.estadisticaConexion = estructura_estadistica_conexion();
end

function estadistica = estructura_estadistica_conexion()
%ESTRUCTURA_ESTADISTICA_CONEXION Diagnostico de las aristas del inicio.

estadistica = struct();
estadistica.candidatosDentroRadio = 0;
estadistica.candidatosEvaluados = 0;
estadistica.aceptadas = 0;
estadistica.rechazosCoincidencia = 0;
estadistica.rechazosEstaticos = 0;
estadistica.rechazosDinamicos = 0;
estadistica.duplicadasActivas = 0;
end

function diagnosticos = extraer_diagnosticos(resultados)
%EXTRAER_DIAGNOSTICOS Elimina matrices grandes de la salida informativa.

plantilla = estructura_diagnostico_intento();
diagnosticos = repmat(plantilla,numel(resultados),1);

for i = 1:numel(resultados)
    r = resultados(i);
    e = r.estadisticaConexion;

    diagnosticos(i).numeroIntento = r.numeroIntento;
    diagnosticos(i).exito = r.exito;
    diagnosticos(i).motivo = r.motivo;
    diagnosticos(i).radioConsulta = r.radioConsulta;
    diagnosticos(i).maxConexiones = r.maxConexiones;
    diagnosticos(i).inicioCoincidenteConNodo = ...
        r.inicioCoincidenteConNodo;
    diagnosticos(i).indiceInicioGrafo = r.indiceInicioGrafo;
    diagnosticos(i).numeroConexionesTemporales = ...
        r.numeroConexionesTemporales;
    diagnosticos(i).gradoActivoInicio = r.gradoActivoInicio;
    diagnosticos(i).numeroNodosGrafo = size(r.nodosGrafo,1);
    diagnosticos(i).numeroAristasGrafo = size(r.aristasGrafo,1);
    diagnosticos(i).costeCamino = r.costeCamino;
    diagnosticos(i).numeroNodosCamino = numel(r.indicesCamino);

    diagnosticos(i).candidatosDentroRadio = ...
        e.candidatosDentroRadio;
    diagnosticos(i).candidatosEvaluados = ...
        e.candidatosEvaluados;
    diagnosticos(i).conexionesAceptadasAntesDuplicados = ...
        e.aceptadas;
    diagnosticos(i).rechazosCoincidencia = ...
        e.rechazosCoincidencia;
    diagnosticos(i).rechazosEstaticos = ...
        e.rechazosEstaticos;
    diagnosticos(i).rechazosDinamicos = ...
        e.rechazosDinamicos;
    diagnosticos(i).duplicadasActivas = ...
        e.duplicadasActivas;
end
end

function diagnostico = estructura_diagnostico_intento()
%ESTRUCTURA_DIAGNOSTICO_INTENTO Define campos escalares de cada intento.

diagnostico = struct();
diagnostico.numeroIntento = 0;
diagnostico.exito = false;
diagnostico.motivo = "";
diagnostico.radioConsulta = NaN;
diagnostico.maxConexiones = 0;
diagnostico.inicioCoincidenteConNodo = false;
diagnostico.indiceInicioGrafo = NaN;
diagnostico.numeroConexionesTemporales = 0;
diagnostico.gradoActivoInicio = 0;
diagnostico.numeroNodosGrafo = 0;
diagnostico.numeroAristasGrafo = 0;
diagnostico.costeCamino = inf;
diagnostico.numeroNodosCamino = 0;
diagnostico.candidatosDentroRadio = 0;
diagnostico.candidatosEvaluados = 0;
diagnostico.conexionesAceptadasAntesDuplicados = 0;
diagnostico.rechazosCoincidencia = 0;
diagnostico.rechazosEstaticos = 0;
diagnostico.rechazosDinamicos = 0;
diagnostico.duplicadasActivas = 0;
end

%% ========================================================================
% VALIDACION
% ========================================================================

function [roadmap,inicio,dinamicos,parametros] = validar_entradas( ...
    roadmap,estadoRobot,dinamicos,robot,cfg)
%VALIDAR_ENTRADAS Comprueba la roadmap y la configuracion de consulta.

%% Roadmap
if ~isstruct(roadmap) || ~isscalar(roadmap)
    error('consulta_prm:RoadmapNoValida', ...
        'roadmap debe ser la estructura obtenida mediante prm.m.');
end

camposRoadmap = { ...
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
    'numeroNodos', ...
    'numeroAristas'};

for i = 1:numel(camposRoadmap)
    if ~isfield(roadmap,camposRoadmap{i})
        error('consulta_prm:RoadmapIncompleta', ...
            'Falta roadmap.%s.',camposRoadmap{i});
    end
end

if string(roadmap.esquema) ~= "roadmap_prm_v1"
    error('consulta_prm:EsquemaNoSoportado', ...
        'La roadmap debe utilizar el esquema roadmap_prm_v1.');
end

if ~isnumeric(roadmap.nodos) || ~isreal(roadmap.nodos) || ...
        size(roadmap.nodos,2) ~= 2 || isempty(roadmap.nodos) || ...
        any(~isfinite(roadmap.nodos(:)))
    error('consulta_prm:NodosNoValidos', ...
        'roadmap.nodos debe ser una matriz N x 2 real y finita.');
end

roadmap.nodos = double(roadmap.nodos);
numeroNodos = size(roadmap.nodos,1);

if isempty(roadmap.aristas)
    roadmap.aristas = zeros(0,2);
elseif ~isnumeric(roadmap.aristas) || ~isreal(roadmap.aristas) || ...
        size(roadmap.aristas,2) ~= 2 || ...
        any(~isfinite(roadmap.aristas(:))) || ...
        any(roadmap.aristas(:) ~= floor(roadmap.aristas(:))) || ...
        any(roadmap.aristas(:) < 1) || ...
        any(roadmap.aristas(:) > numeroNodos) || ...
        any(roadmap.aristas(:,1) == roadmap.aristas(:,2))
    error('consulta_prm:AristasNoValidas', ...
        'roadmap.aristas contiene indices no validos.');
else
    roadmap.aristas = double(roadmap.aristas);
end

numeroAristas = size(roadmap.aristas,1);

if numeroAristas == 0 && isempty(roadmap.pesos)
    roadmap.pesos = zeros(0,1);
elseif ~isnumeric(roadmap.pesos) || ~isreal(roadmap.pesos) || ...
        numel(roadmap.pesos) ~= numeroAristas || ...
        any(~isfinite(roadmap.pesos(:))) || ...
        any(roadmap.pesos(:) <= 0)
    error('consulta_prm:PesosNoValidos', ...
        'roadmap.pesos debe contener un peso positivo por arista.');
else
    roadmap.pesos = reshape(double(roadmap.pesos),[],1);
end

if numeroAristas == 0 && isempty(roadmap.cajasAristas)
    roadmap.cajasAristas = zeros(0,4);
elseif ~isnumeric(roadmap.cajasAristas) || ...
        ~isreal(roadmap.cajasAristas) || ...
        ~isequal(size(roadmap.cajasAristas),[numeroAristas 4]) || ...
        any(~isfinite(roadmap.cajasAristas(:))) || ...
        any(roadmap.cajasAristas(:,2) < roadmap.cajasAristas(:,1)) || ...
        any(roadmap.cajasAristas(:,4) < roadmap.cajasAristas(:,3))
    error('consulta_prm:CajasAristasNoValidas', ...
        ['roadmap.cajasAristas debe tener formato M x 4 con ' ...
         '[xmin xmax ymin ymax].']);
else
    roadmap.cajasAristas = double(roadmap.cajasAristas);
end

if ~es_entero_positivo(roadmap.indiceMeta) || ...
        roadmap.indiceMeta > numeroNodos
    error('consulta_prm:IndiceMetaNoValido', ...
        'roadmap.indiceMeta no es un indice de nodo valido.');
end

roadmap.indiceMeta = double(roadmap.indiceMeta);

roadmap.meta = validar_vector(roadmap.meta,2,'roadmap.meta');
roadmap.limites = validar_vector(roadmap.limites,4,'roadmap.limites');

if roadmap.limites(2) <= roadmap.limites(1) || ...
        roadmap.limites(4) <= roadmap.limites(3)
    error('consulta_prm:LimitesNoValidos', ...
        'roadmap.limites debe definir un area positiva.');
end

if isempty(roadmap.obstaculosEstaticos)
    roadmap.obstaculosEstaticos = zeros(0,4);
elseif ~isnumeric(roadmap.obstaculosEstaticos) || ...
        ~isreal(roadmap.obstaculosEstaticos) || ...
        size(roadmap.obstaculosEstaticos,2) ~= 4 || ...
        any(~isfinite(roadmap.obstaculosEstaticos(:))) || ...
        any(roadmap.obstaculosEstaticos(:,3:4) <= 0,'all')
    error('consulta_prm:EstaticosNoValidos', ...
        ['roadmap.obstaculosEstaticos debe ser una matriz N x 4 ' ...
         'con dimensiones positivas.']);
else
    roadmap.obstaculosEstaticos = ...
        double(roadmap.obstaculosEstaticos);
end

if ~es_escalar_positivo(roadmap.radioRobot) || ...
        ~es_escalar_no_negativo(roadmap.margenEstatico) || ...
        ~es_escalar_positivo(roadmap.separacionEstatica)
    error('consulta_prm:GeometriaRoadmapNoValida', ...
        'Los radios y margenes almacenados en la roadmap no son validos.');
end

roadmap.radioRobot = double(roadmap.radioRobot);
roadmap.margenEstatico = double(roadmap.margenEstatico);
roadmap.separacionEstatica = double(roadmap.separacionEstatica);

if ~es_entero_no_negativo(roadmap.numeroNodos) || ...
        roadmap.numeroNodos ~= numeroNodos || ...
        ~es_entero_no_negativo(roadmap.numeroAristas) || ...
        roadmap.numeroAristas ~= numeroAristas
    error('consulta_prm:ContadoresRoadmapIncoherentes', ...
        ['roadmap.numeroNodos y roadmap.numeroAristas deben coincidir ' ...
         'con las matrices almacenadas.']);
end

%% Coherencia de la meta almacenada
escalaRoadmap = max(1,max(abs([ ...
    roadmap.nodos(:);roadmap.meta(:);roadmap.limites(:)])));

tolRoadmap = 1e-10*escalaRoadmap;

if norm( ...
        roadmap.nodos(roadmap.indiceMeta,:)-roadmap.meta) > tolRoadmap
    error('consulta_prm:MetaIncoherente', ...
        'El nodo roadmap.indiceMeta no coincide con roadmap.meta.');
end

%% Estado actual
if ~isnumeric(estadoRobot) || ~isreal(estadoRobot) || ...
        numel(estadoRobot) < 2 || any(~isfinite(estadoRobot(:)))
    error('consulta_prm:EstadoRobotNoValido', ...
        ['estadoRobot debe ser un vector numerico real y finito ' ...
         'con al menos [x y].']);
end

estadoRobot = reshape(double(estadoRobot),1,[]);
inicio = estadoRobot(1:2);

%% Robot
if ~isstruct(robot) || ~isscalar(robot) || ...
        ~isfield(robot,'geometria') || ...
        ~isstruct(robot.geometria) || ...
        ~isfield(robot.geometria,'radio') || ...
        ~es_escalar_positivo(robot.geometria.radio)
    error('consulta_prm:RobotNoValido', ...
        ['robot debe ser la estructura obtenida mediante ' ...
         'configuracion_robot.m.']);
end

radioRobot = double(robot.geometria.radio);

%% Configuracion
if ~isstruct(cfg) || ~isscalar(cfg) || ...
        ~isfield(cfg,'sim') || ...
        ~isfield(cfg,'seguridad') || ...
        ~isfield(cfg,'prediccion') || ...
        ~isfield(cfg,'prm')
    error('consulta_prm:ConfiguracionNoValida', ...
        ['cfg debe ser la estructura obtenida mediante ' ...
         'parametros_generales.m.']);
end

Ts = obtener_positivo(cfg.sim,'Ts','cfg.sim.Ts');

margenEstatico = obtener_no_negativo( ...
    cfg.seguridad,'margenEstatico', ...
    'cfg.seguridad.margenEstatico');

margenDinamico = obtener_no_negativo( ...
    cfg.seguridad,'margenDinamico', ...
    'cfg.seguridad.margenDinamico');

pasosPrediccion = obtener_entero_no_negativo( ...
    cfg.prediccion,'pasos','cfg.prediccion.pasos');

if ~isfield(cfg.prediccion,'modelo') || ...
        ~isscalar(string(cfg.prediccion.modelo))
    error('consulta_prm:ModeloPrediccionAusente', ...
        'Falta cfg.prediccion.modelo.');
end

modeloPrediccion = lower(strtrim(string(cfg.prediccion.modelo)));

if modeloPrediccion ~= "velocidad_constante"
    error('consulta_prm:ModeloPrediccionNoSoportado', ...
        ['Esta version implementa exclusivamente el modelo ' ...
         '"velocidad_constante".']);
end

radioConsulta = obtener_positivo( ...
    cfg.prm,'radioConsulta','cfg.prm.radioConsulta');

vecinosConsulta = obtener_entero_positivo( ...
    cfg.prm,'vecinosConsulta','cfg.prm.vecinosConsulta');

%% Coherencia entre roadmap, robot y configuracion
escalaCoherencia = max(1,max(abs([ ...
    roadmap.radioRobot,radioRobot, ...
    roadmap.margenEstatico,margenEstatico, ...
    roadmap.separacionEstatica, ...
    radioRobot+margenEstatico])));

tolCoherencia = 1e-10*escalaCoherencia;

if abs(roadmap.radioRobot-radioRobot) > tolCoherencia
    error('consulta_prm:RadioRobotIncompatible', ...
        ['La roadmap fue construida con un radio de robot diferente ' ...
         'al de la consulta actual.']);
end

if abs(roadmap.margenEstatico-margenEstatico) > tolCoherencia || ...
        abs(roadmap.separacionEstatica- ...
            (radioRobot+margenEstatico)) > tolCoherencia
    error('consulta_prm:MargenEstaticoIncompatible', ...
        ['La roadmap fue construida con un margen estatico diferente ' ...
         'al configurado en la consulta.']);
end

%% Obstaculos dinamicos
if isempty(dinamicos)
    dinamicos = struct('id',{},'pos',{},'vel',{},'radio',{});
elseif ~isstruct(dinamicos)
    error('consulta_prm:DinamicosNoValidos', ...
        'obstaculosDinamicos debe ser un vector de estructuras.');
else
    camposDinamicos = {'pos','vel','radio'};

    for i = 1:numel(dinamicos)
        for j = 1:numel(camposDinamicos)
            if ~isfield(dinamicos(i),camposDinamicos{j})
                error('consulta_prm:CampoDinamicoAusente', ...
                    'Falta el campo %s en el obstaculo dinamico %d.', ...
                    camposDinamicos{j},i);
            end
        end

        dinamicos(i).pos = validar_vector( ...
            dinamicos(i).pos,2, ...
            sprintf('obstaculosDinamicos(%d).pos',i));

        dinamicos(i).vel = validar_vector( ...
            dinamicos(i).vel,2, ...
            sprintf('obstaculosDinamicos(%d).vel',i));

        if ~es_escalar_positivo(dinamicos(i).radio)
            error('consulta_prm:RadioDinamicoNoValido', ...
                ['obstaculosDinamicos(%d).radio debe ser un ' ...
                 'escalar positivo.'],i);
        end

        dinamicos(i).radio = double(dinamicos(i).radio);
    end
end

%% Activacion de la preparacion grafica
activarVista = false;
mostrarPlanificador = false;

if isfield(cfg,'visual') && isstruct(cfg.visual)
    if isfield(cfg.visual,'activa') && ...
            islogical(cfg.visual.activa) && isscalar(cfg.visual.activa)
        activarVista = cfg.visual.activa;
    end

    if isfield(cfg.visual,'mostrarPlanificador') && ...
            islogical(cfg.visual.mostrarPlanificador) && ...
            isscalar(cfg.visual.mostrarPlanificador)
        mostrarPlanificador = cfg.visual.mostrarPlanificador;
    end
end

%% Parametros normalizados
parametros = struct();
parametros.Ts = Ts;
parametros.radioRobot = radioRobot;
parametros.margenEstatico = margenEstatico;
parametros.margenDinamico = margenDinamico;
parametros.pasosPrediccion = pasosPrediccion;
parametros.modeloPrediccion = modeloPrediccion;
parametros.radioConsulta = radioConsulta;
parametros.vecinosConsulta = vecinosConsulta;
parametros.factorSegundoRadio = 1.45;
parametros.factorSegundoVecinos = 2;
parametros.simplificarCamino = true;
parametros.generarVista = activarVista && mostrarPlanificador;
end

function valor = obtener_positivo(estructura,campo,nombre)
%OBTENER_POSITIVO Recupera un escalar real positivo.

if ~isfield(estructura,campo) || ...
        ~es_escalar_positivo(estructura.(campo))
    error('consulta_prm:ParametroPositivoNoValido', ...
        '%s debe ser un escalar real, finito y positivo.',nombre);
end

valor = double(estructura.(campo));
end

function valor = obtener_no_negativo(estructura,campo,nombre)
%OBTENER_NO_NEGATIVO Recupera un escalar real no negativo.

if ~isfield(estructura,campo) || ...
        ~es_escalar_no_negativo(estructura.(campo))
    error('consulta_prm:ParametroNoNegativoNoValido', ...
        '%s debe ser un escalar real, finito y no negativo.',nombre);
end

valor = double(estructura.(campo));
end

function valor = obtener_entero_positivo(estructura,campo,nombre)
%OBTENER_ENTERO_POSITIVO Recupera un entero estrictamente positivo.

if ~isfield(estructura,campo) || ...
        ~es_entero_positivo(estructura.(campo))
    error('consulta_prm:ParametroEnteroPositivoNoValido', ...
        '%s debe ser un entero positivo.',nombre);
end

valor = double(estructura.(campo));
end

function valor = obtener_entero_no_negativo(estructura,campo,nombre)
%OBTENER_ENTERO_NO_NEGATIVO Recupera un entero mayor o igual que cero.

if ~isfield(estructura,campo) || ...
        ~es_entero_no_negativo(estructura.(campo))
    error('consulta_prm:ParametroEnteroNoNegativoNoValido', ...
        '%s debe ser un entero no negativo.',nombre);
end

valor = double(estructura.(campo));
end

function vector = validar_vector(vector,dimension,nombre)
%VALIDAR_VECTOR Comprueba y convierte un vector a formato fila.

if ~isnumeric(vector) || ~isreal(vector) || ...
        numel(vector) ~= dimension || any(~isfinite(vector(:)))
    error('consulta_prm:VectorNoValido', ...
        '%s debe ser un vector real y finito de %d componentes.', ...
        nombre,dimension);
end

vector = reshape(double(vector),1,dimension);
end

function tf = es_escalar_positivo(valor)
%ES_ESCALAR_POSITIVO Comprueba un escalar real estrictamente positivo.

tf = isnumeric(valor) && isreal(valor) && isscalar(valor) && ...
    isfinite(valor) && valor > 0;
end

function tf = es_escalar_no_negativo(valor)
%ES_ESCALAR_NO_NEGATIVO Comprueba un escalar real mayor o igual que cero.

tf = isnumeric(valor) && isreal(valor) && isscalar(valor) && ...
    isfinite(valor) && valor >= 0;
end

function tf = es_entero_positivo(valor)
%ES_ENTERO_POSITIVO Comprueba un entero estrictamente positivo.

tf = isnumeric(valor) && isreal(valor) && isscalar(valor) && ...
    isfinite(valor) && valor == floor(valor) && valor > 0;
end

function tf = es_entero_no_negativo(valor)
%ES_ENTERO_NO_NEGATIVO Comprueba un entero mayor o igual que cero.

tf = isnumeric(valor) && isreal(valor) && isscalar(valor) && ...
    isfinite(valor) && valor == floor(valor) && valor >= 0;
end
