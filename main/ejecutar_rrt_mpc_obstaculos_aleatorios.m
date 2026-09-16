%% RRT* + MPC en un entorno dinamico con obstaculos aleatorios
% Ensambla los modulos del proyecto TFM_RobotNavigation para ejecutar una
% simulacion completa de la arquitectura RRT* + MPC.
%
% El planificador global es exactamente el mismo RRT* convencional utilizado
% en RRT* + APF. La unica sustitucion metodologica es el controlador local:
% MPC recibe el objetivo adelantado comun, predice el comportamiento del
% robot y de los obstaculos y genera el control [v w].
%
% La semilla del planificador se fija al inicio de cada ejecucion y no se
% modifica durante la simulacion. Los obstaculos moviles utilizan un
% RandStream independiente, de modo que sus posiciones y rebotes no alteran
% el generador pseudoaleatorio empleado por RRT*.
%
% El robot se integra mediante el mismo modelo de uniciclo [x y theta]. La
% visualizacion conserva la misma estructura eficiente: los objetos se crean
% una vez, se actualizan sin clf y el arbol RRT* anterior se sustituye
% completamente cuando existe una nueva planificacion.
clearvars;
clc;
close all;

% Habilita el registro de animaciones de figuras en el Live Editor.
% La configuracion es temporal y no modifica permanentemente las
% preferencias del usuario. Si una version antigua no expone estos
% ajustes, la simulacion continua sin interrumpirse.
try
    configuracionEditor = settings;
    configuracionEditor.matlab.editor.AllowFigureAnimation.TemporaryValue = 1;

    try
        configuracionEditor.matlab.editor.MinimumFramesForAnimation.TemporaryValue = 2;
    catch
        % Este ajuste puede no existir en versiones antiguas.
    end
catch
    configuracionEditor = [];
end

%% Opciones de la ejecucion
% Cambie solamente estas opciones para ejecutar otra demostracion.
modoEjecucion = "visual";           % "visual" o "batch"
idEscenario = "alta";               % "baja", "media" o "alta"

% Semilla fija del planificador. RRT* comienza todas las ejecuciones desde
% la misma secuencia pseudoaleatoria y esta configuracion no cambia durante
% la simulacion.
semillaPlanificador = 7;

% Generador independiente de los obstaculos dinamicos.
%
% "nueva":
%   Cada ejecucion crea una realizacion distinta de posiciones iniciales y
%   rebotes sin modificar el generador utilizado por RRT*.
%
% "reproducible":
%   Repite exactamente la realizacion determinada por
%   semillaObstaculosFija. Este modo resulta util para reproducir o analizar
%   una situacion dinamica concreta.
modoSemillaObstaculos = "nueva";    % "nueva" o "reproducible"
semillaObstaculosFija = 100000 + semillaPlanificador;

% Parametros del movimiento aleatorio. Se definen aqui para mantener esta
% version del main autocontenida sin modificar parametros_generales.m.
cfgAleatoriedad = struct();
cfgAleatoriedad.modoPosicion = "alrededor_referencia";
cfgAleatoriedad.radioPerturbacionPosicion = 2.0;   % [m]
cfgAleatoriedad.maxIntentosPorObstaculo = 5000;
cfgAleatoriedad.desviacionMaximaRebote = pi/3;     % +/-60 grados
cfgAleatoriedad.maxIntentosDireccion = 40;

% La pausa solo modifica la velocidad con la que se observa la animacion;
% no modifica el periodo fisico de muestreo de la simulacion.
factorVelocidadAnimacion = 1.0;  % 1 = tiempo simulado aproximado

% Exportacion y guardado son opcionales. Permanecen desactivados durante el
% desarrollo para no mezclar su coste con los tiempos de los algoritmos.
exportarAnimacion = false;
formatoAnimacion = "mp4";        % "mp4", "m4v", "avi" o "gif"
guardarResultado = false;

%% Localizacion de la raiz y configuracion del path
% En los Live Scripts, mfilename puede devolver el nombre de un helper temporal del Live Editor en lugar de la ruta real del archivo. Por ello, la fuente principal de la raiz es el proyecto MATLAB actualmente abierto.
raizProyecto = '';

try
    proyectoAbierto = matlab.project.rootProject;
catch
    proyectoAbierto = [];
end

if ~isempty(proyectoAbierto)
    raizProyecto = char(proyectoAbierto.RootFolder);
end

% Fallback para poder ejecutar el script aunque el proyecto no este abierto.
% Se prueba primero la ubicacion del documento activo y despues pwd.
if isempty(raizProyecto)
    carpetasIniciales = {pwd};

    try
        rutaActiva = matlab.desktop.editor.getActiveFilename;

        if ~isempty(rutaActiva)
            carpetasIniciales = [ ...
                {fileparts(rutaActiva)}, ...
                carpetasIniciales];
        end
    catch
        % MATLAB Online puede no exponer esta funcion en todos los contextos.
    end

    for iInicio = 1:numel(carpetasIniciales)
        carpetaCandidata = carpetasIniciales{iInicio};

        for nivel = 1:10
            estructuraValida = ...
                isfolder(fullfile(carpetaCandidata,'config')) && ...
                isfolder(fullfile(carpetaCandidata,'planificadores')) && ...
                isfolder(fullfile(carpetaCandidata,'controladores')) && ...
                isfolder(fullfile(carpetaCandidata,'simulacion')) && ...
                isfolder(fullfile(carpetaCandidata,'metricas')) && ...
                isfolder(fullfile(carpetaCandidata,'visualizacion')) && ...
                isfolder(fullfile(carpetaCandidata,'herramientas'));

            if estructuraValida
                raizProyecto = carpetaCandidata;
                break;
            end

            carpetaPadre = fileparts(carpetaCandidata);

            if isempty(carpetaPadre) || ...
                    strcmp(carpetaPadre,carpetaCandidata)
                break;
            end

            carpetaCandidata = carpetaPadre;
        end

        if ~isempty(raizProyecto)
            break;
        end
    end
end

if isempty(raizProyecto)
    error('ejecutar_rrt_mpc:RaizNoEncontrada', ...
        ['No se pudo identificar la raiz del proyecto. Abra el archivo ' ...
         'TFM_RobotNavigation.prj o establezca como carpeta actual la ' ...
         'raiz que contiene config, main, planificadores y controladores.']);
end

carpetasProyecto = [ ...
    "config", ...
    "planificadores", ...
    "controladores", ...
    "simulacion", ...
    "metricas", ...
    "visualizacion", ...
    "herramientas"];

for iCarpeta = 1:numel(carpetasProyecto)
    carpetaModulo = fullfile( ...
        raizProyecto,carpetasProyecto(iCarpeta));

    if ~isfolder(carpetaModulo)
        error('ejecutar_rrt_mpc:CarpetaAusente', ...
            'No existe la carpeta requerida: %s',char(carpetaModulo));
    end

    addpath(char(carpetaModulo),'-begin');
end

rehash path;
fprintf('Raiz del proyecto: %s\n',raizProyecto);


% Comprobacion temprana de las dependencias utilizadas por este main.
funcionesNecesarias = [ ...
    "parametros_generales", ...
    "escenarios", ...
    "configuracion_robot", ...
    "rrt_star", ...
    "seguimiento_trayectoria", ...
    "mpc", ...
    "inicializar_obstaculos_aleatorios", ...
    "actualizar_obstaculos_aleatorios", ...
    "crear_flujo_obstaculos", ...
    "replanificacion", ...
    "modelo_robot", ...
    "detectar_colisiones", ...
    "resultados", ...
    "inicializar_figura", ...
    "dibujar_entorno", ...
    "dibujar_robot", ...
    "actualizar_graficos", ...
    "exportar_animacion"];

for iFuncion = 1:numel(funcionesNecesarias)
    if isempty(which(char(funcionesNecesarias(iFuncion))))
        error('ejecutar_rrt_mpc:DependenciaAusente', ...
            'No se encontro la funcion %s.m en el path del proyecto.', ...
            funcionesNecesarias(iFuncion));
    end
end

%% Configuracion comun, escenario y robot
cfg = parametros_generales(modoEjecucion);
cfg.semilla = semillaPlanificador;
cfg.nombreArquitectura = "RRT* + MPC";
cfg.planificador = @rrt_star;
cfg.controlador = @mpc;

if ~isnumeric(factorVelocidadAnimacion) || ...
        ~isscalar(factorVelocidadAnimacion) || ...
        ~isfinite(factorVelocidadAnimacion) || ...
        factorVelocidadAnimacion <= 0
    error('ejecutar_rrt_mpc:FactorAnimacionNoValido', ...
        'factorVelocidadAnimacion debe ser un escalar positivo.');
end

if cfg.visual.activa
    cfg.visual.pausa = true;
    cfg.visual.tPausa = ...
        cfg.sim.Ts/factorVelocidadAnimacion;
end

escenario = escenarios(idEscenario);
robot = configuracion_robot();

% Los parametros se incorporan a cfg para que los dos modulos aleatorios
% utilicen exactamente la misma configuracion.
cfg.aleatoriedadObstaculos = cfgAleatoriedad;

% La semilla del planificador forma parte de la configuracion interna fija
% del sistema. Se establece una sola vez al comienzo de cada ejecucion,
% antes de cualquier llamada a RRT*.
rng(cfg.semilla,'twister');
estadoGeneradorPlanificadorAntes = rng;

% Los obstaculos moviles utilizan un RandStream propio. La funcion auxiliar
% crea ese flujo sin llamar a rng y, por tanto, sin modificar la secuencia
% pseudoaleatoria reservada para el planificador.
[flujoObstaculos,semillaObstaculos,infoFlujoObstaculos] = ...
    crear_flujo_obstaculos( ...
        modoSemillaObstaculos, ...
        semillaObstaculosFija);

% Comprobacion defensiva: la creacion del flujo de obstaculos no debe alterar
% el estado del generador global que utilizara RRT*.
estadoGeneradorPlanificadorDespues = rng;

if ~isequaln( ...
        estadoGeneradorPlanificadorAntes, ...
        estadoGeneradorPlanificadorDespues)
    error('ejecutar_rrt_mpc:GeneradorPlanificadorModificado', ...
        ['La creacion del flujo de obstaculos ha modificado el ' ...
         'generador pseudoaleatorio del planificador.']);
end

% Se conserva el modo ya validado y normalizado por la funcion auxiliar.
modoSemillaObstaculos = infoFlujoObstaculos.modo;

[obstaculosDinamicos,infoInicializacionObstaculos] = ...
    inicializar_obstaculos_aleatorios( ...
        escenario,robot,cfg,flujoObstaculos);

% La realizacion inicial pasa a formar parte del escenario utilizado por
% los modulos de metricas y por el archivo de resultados.
escenario.obstaculosDinamicos = obstaculosDinamicos;
escenario.movimientoDinamico = "aleatorio_con_rebote";

%% Inicializacion del estado y de los historiales
estadoRobot = escenario.inicio;
numeroRecuperacionesColision = 0;
enColisionDinamicaAnterior = false;
caminoGlobal = zeros(0,2);
caminoGlobalEsParcial = false;

arbolRRT = struct();
infoRRT = struct();
infoMPC = struct();

% El MPC penaliza los cambios respecto al control realmente aplicado en el
% paso anterior. Se actualiza despues de resolver colisiones y saturaciones.
controlAnterior = robot.controlInicial;
infoSeguimiento = struct();
infoReplanificacion = struct();
infoColision = struct();
infoMovimientoObstaculos = struct();

numeroRebotesAleatorios = 0;

siguientePasoPermitido = 1;
maxPasos = cfg.terminacion.maxPasos;
numeroDinamicos = numel(obstaculosDinamicos);

trayectoriaEjecutada = nan(maxPasos+1,3);
trayectoriaEjecutada(1,:) = estadoRobot;

controlesAplicados = nan(maxPasos,2);

historialObstaculos = ...
    nan(maxPasos+1,numeroDinamicos,2);

for iObstaculo = 1:numeroDinamicos
    historialObstaculos(1,iObstaculo,:) = reshape( ...
        obstaculosDinamicos(iObstaculo).pos,1,1,2);
end

% Los valores NaN indican que una fase no fue ejecutada en ese paso.
tiempoPlanificacion = nan(maxPasos,1);
tiempoControl = nan(maxPasos,1);
tiempoCiclo = nan(maxPasos,1);
tiempoVisualizacion = nan(maxPasos,1);

planificacionEjecutada = false(maxPasos,1);
controlEjecutado = false(maxPasos,1);

% Diagnosticos complementarios del controlador y del seguimiento.
mpcFactiblePorPaso = false(maxPasos,1);
riesgoPredichoPorPaso = false(maxPasos,1);
colisionPredichaPorPaso = false(maxPasos,1);
distanciaMinimaPredichaPorPaso = nan(maxPasos,1);
costeMPCPorPaso = nan(maxPasos,1);
numeroCandidatosFactiblesPorPaso = nan(maxPasos,1);
fraccionCandidatosFactiblesPorPaso = nan(maxPasos,1);
numeroViolacionesSeguridadPorPaso = nan(maxPasos,1);
distanciaAlCaminoPorPaso = nan(maxPasos,1);
motivoMPCPorPaso = strings(maxPasos,1);
motivoCiclo = strings(maxPasos,1);
motivoRRT = strings(maxPasos,1);
motivoReplanificacion = strings(maxPasos,1);
motivoSeguimiento = strings(maxPasos,1);
caminoParcialPorPaso = false(maxPasos,1);

numeroPlanificaciones = 0;
numeroReplanificaciones = 0;
numeroFallosPlanificacion = 0;
numeroEpisodiosSinCandidatoFactible = 0;
numeroEpisodiosRiesgoPredicho = 0;
numeroEpisodiosColisionPredicha = 0;
numeroRecuperacionesColision = 0;
numeroAproximacionesParciales = 0;

enFalloFactibilidadAnterior = false;
enRiesgoPredichoAnterior = false;
enColisionPredichaAnterior = false;
enColisionDinamicaAnterior = false;

pasosEjecutados = 0;
metaAlcanzada = ...
    norm(estadoRobot(1:2)-escenario.meta) <= ...
        cfg.navegacion.radioMeta;
colisionOcurrida = false;

vistaPlanificador = struct( ...
    'actualizar',false, ...
    'nodos',zeros(0,2), ...
    'aristas',zeros(0,2));

%% Preparacion opcional de la exportacion externa
% El video permanece desactivado por defecto. Esta preparacion no crea
% ningun fotograma y no interviene en la animacion interna del Live Editor.
exportacion = struct();
infoExportacion = struct();
archivoAnimacion = "";
exportacionIniciada = false;

if exportarAnimacion
    extension = lower(strtrim(string(formatoAnimacion)));

    if ~any(extension == ["mp4","m4v","avi","gif"])
        error('ejecutar_rrt_mpc:FormatoAnimacionNoValido', ...
            'El formato debe ser mp4, m4v, avi o gif.');
    end

    nombreAnimacion = "rrt_mpc_aleatorio_"+string(escenario.id)+ ...
        "_semilla_"+string(cfg.semilla)+ ...
        "_obst_"+string(semillaObstaculos)+"."+extension;

    archivoAnimacion = fullfile( ...
        raizProyecto,"salidas","animaciones",nombreAnimacion);

    opcionesExportacion = struct();
    opcionesExportacion.activa = true;
    opcionesExportacion.sobrescribir = true;
    opcionesExportacion.factorVelocidad = ...
        factorVelocidadAnimacion;
    opcionesExportacion.calidad = 90;
    opcionesExportacion.capturarSoloActualizaciones = true;
    opcionesExportacion.bloquearRedimensionado = true;
    opcionesExportacion.eliminarSiVacio = true;
    opcionesExportacion.mostrarMensajes = false;
else
    opcionesExportacion = struct();
end

%% Visualizacion y simulacion RRT* + MPC
% IMPORTANTE PARA EL LIVE EDITOR:
%   1. La figura se crea directamente en este script.
%   2. Desde esta llamada a figure hasta el final del bucle no hay
%      separadores de seccion "%%".
%   3. El unico drawnow efectivo de la animacion se ejecuta dentro del
%      bucle, despues de actualizar todos los objetos graficos.
if cfg.visual.activa
    figuraLive = figure('Color','w');
else
    figuraLive = [];
end

graficos = inicializar_figura( ...
    escenario, ...
    cfg, ...
    cfg.nombreArquitectura, ...
    figuraLive);

graficos = dibujar_entorno( ...
    graficos, ...
    escenario, ...
    obstaculosDinamicos);

graficos = dibujar_robot( ...
    graficos, ...
    estadoRobot, ...
    robot);

% La exportacion externa se abre despues de crear los objetos, pero no
% captura todavia ningun fotograma.
if exportarAnimacion && graficos.activa
    [exportacion,infoExportacion] = exportar_animacion( ...
        "iniciar",[],graficos,cfg, ...
        archivoAnimacion,opcionesExportacion);

    exportacionIniciada = true;
end

relojTotal = tic;

try
    for k = 1:maxPasos
        pasosEjecutados = k;
        relojCiclo = tic;

        textoEstado = "Navegacion";
        mpcFactibleActual = true;
        riesgoPredichoActual = false;
        colisionPredichaActual = false;
        controlSolicitado = robot.controlParada;
        motivoRRT(k) = "no_ejecutado";
        motivoMPCPorPaso(k) = "control_no_ejecutado";

        vistaPlanificador = struct( ...
            'actualizar',false, ...
            'nodos',zeros(0,2), ...
            'aristas',zeros(0,2));

% -------------------------------------------------------------------------
% Movimiento de los obstaculos dinamicos
% -------------------------------------------------------------------------
        [obstaculosDinamicos,infoMovimientoObstaculos] = ...
            actualizar_obstaculos_aleatorios( ...
                obstaculosDinamicos, ...
                escenario.obstaculosEstaticos, ...
                escenario.limites, ...
                cfg.sim.Ts, ...
                flujoObstaculos, ...
                cfg);

        numeroRebotesAleatorios = ...
            numeroRebotesAleatorios + ...
            infoMovimientoObstaculos.numeroRebotesTotales;

        for iObstaculo = 1:numeroDinamicos
            historialObstaculos(k+1,iObstaculo,:) = reshape( ...
                obstaculosDinamicos(iObstaculo).pos,1,1,2);
        end

% -------------------------------------------------------------------------
% Decision global de replanificacion
% -------------------------------------------------------------------------
        % RRT* y el criterio global reciben una lista dinamica vacia.
        % Los obstaculos moviles se delegan exclusivamente al MPC local.
        dinamicosParaPlanificacion = obstaculosDinamicos([]);

        [debeReplanificar,infoReplanificacion] = replanificacion( ...
            caminoGlobal, ...
            estadoRobot, ...
            k, ...
            robot, ...
            escenario.obstaculosEstaticos, ...
            dinamicosParaPlanificacion, ...
            escenario.limites, ...
            cfg, ...
            siguientePasoPermitido);

        motivoReplanificacion(k) = ...
            string(infoReplanificacion.motivo);

        if debeReplanificar
            caminoAnterior = caminoGlobal;
            eraPlanificacionInicial = numeroPlanificaciones == 0;

            relojPlanificacion = tic;

            [caminoCandidato,arbolCandidato,infoRRT] = rrt_star( ...
                estadoRobot, ...
                escenario.meta, ...
                escenario.limites, ...
                escenario.obstaculosEstaticos, ...
                dinamicosParaPlanificacion, ...
                robot, ...
                cfg);

            tiempoPlanificacion(k) = toc(relojPlanificacion);
            planificacionEjecutada(k) = true;
            motivoRRT(k) = string(infoRRT.motivo);
            numeroPlanificaciones = numeroPlanificaciones+1;

            if ~eraPlanificacionInicial
                numeroReplanificaciones = ...
                    numeroReplanificaciones+1;
            end

            % El arbol gris del ultimo intento se representa siempre,
            % tanto si el camino final existe como si el intento falla.
            arbolRRT = arbolCandidato;
            vistaPlanificador = infoRRT.vistaPlanificador;

            if infoRRT.exito
                caminoGlobal = caminoCandidato;
                caminoGlobalEsParcial = false;
                siguientePasoPermitido = k+1;

                if eraPlanificacionInicial
                    textoEstado = "Planificacion inicial";
                else
                    textoEstado = "Replanificacion";
                end
            else
                numeroFallosPlanificacion = ...
                    numeroFallosPlanificacion+1;

                siguientePasoPermitido = ...
                    k+cfg.replan.reintento;

                % Un fallo puntual nunca destruye una ruta completa previa.
                if size(caminoAnterior,1) >= 2
                    caminoGlobal = caminoAnterior;
                    textoEstado = ...
                        "Planificacion fallida: ruta anterior";
                else
                    caminoGlobal = zeros(0,2);
                    textoEstado = "Sin camino global";
                end
            end

        elseif infoReplanificacion.solicitada && ...
                ~infoReplanificacion.permitida
            textoEstado = "Esperando reintento";
        end

% -------------------------------------------------------------------------
% Objetivo local comun sobre la trayectoria global
% -------------------------------------------------------------------------
        [objetivoLocal,caminoGlobal,infoSeguimiento] = ...
            seguimiento_trayectoria( ...
                estadoRobot,caminoGlobal,escenario.meta,cfg,false);

        motivoSeguimiento(k) = string(infoSeguimiento.motivo);
        caminoGlobalEsParcial = false;
        caminoParcialPorPaso(k) = false;

        if isfield(infoSeguimiento,'distanciaAlCamino')
            distanciaAlCaminoPorPaso(k) = ...
                infoSeguimiento.distanciaAlCamino;
        end

% -------------------------------------------------------------------------
% Colision instantanea antes del control
% -------------------------------------------------------------------------
        [hayColisionAntesControl,infoColisionAntesControl] = ...
            detectar_colisiones( ...
                estadoRobot, ...
                robot, ...
                escenario.obstaculosEstaticos, ...
                obstaculosDinamicos, ...
                escenario.limites);

        colisionDinamicaAntesControl = ...
            infoColisionAntesControl.colisionDinamicos;

        colisionNoDinamicaAntesControl = ...
            infoColisionAntesControl.colisionLimites || ...
            infoColisionAntesControl.colisionEstaticos;

% -------------------------------------------------------------------------
% Control local MPC
% -------------------------------------------------------------------------
        if colisionDinamicaAntesControl
            % Durante el contacto fisico con un movil, el robot espera.
            % La ruta global se conserva y el MPC vuelve a evaluarse cuando
            % el obstaculo deja de solaparse con el robot.
            controlSolicitado = robot.controlParada;
            textoEstado = "Esperando separacion del dinamico";

        elseif colisionNoDinamicaAntesControl
            % Esta rama protege frente a un estado numericamente invalido.
            controlSolicitado = robot.controlParada;
            textoEstado = "Reorientacion junto a obstaculo fijo";

        elseif infoSeguimiento.caminoUtilizable && ...
                ~infoSeguimiento.metaAlcanzada

            relojControl = tic;

            [controlSolicitado,infoMPC] = mpc( ...
                estadoRobot, ...
                objetivoLocal, ...
                escenario.meta, ...
                escenario.obstaculosEstaticos, ...
                obstaculosDinamicos, ...
                escenario.limites, ...
                robot, ...
                cfg, ...
                controlAnterior);

            tiempoControl(k) = toc(relojControl);
            controlEjecutado(k) = true;
            motivoMPCPorPaso(k) = string(infoMPC.motivo);

            % infoMPC.exito indica que existe al menos un candidato que
            % respeta todos los umbrales durante el horizonte. Si es false,
            % mpc.m devuelve aun el candidato menos penalizado.
            mpcFactibleActual = logical(infoMPC.exito);
            riesgoPredichoActual = logical(infoMPC.prediceRiesgo);
            colisionPredichaActual = ...
                logical(infoMPC.prediceColisionFisica);

            mpcFactiblePorPaso(k) = mpcFactibleActual;
            riesgoPredichoPorPaso(k) = riesgoPredichoActual;
            colisionPredichaPorPaso(k) = colisionPredichaActual;
            distanciaMinimaPredichaPorPaso(k) = ...
                infoMPC.distanciaMinimaPredicha;
            costeMPCPorPaso(k) = infoMPC.costeOptimo;
            numeroCandidatosFactiblesPorPaso(k) = ...
                infoMPC.numeroCandidatosFactibles;
            fraccionCandidatosFactiblesPorPaso(k) = ...
                infoMPC.fraccionCandidatosFactibles;
            numeroViolacionesSeguridadPorPaso(k) = ...
                infoMPC.numeroViolacionesSeguridad;

            if colisionPredichaActual
                textoEstado = "MPC: colision predicha";
            elseif ~mpcFactibleActual
                textoEstado = "MPC sin candidato factible";
            elseif riesgoPredichoActual
                textoEstado = "MPC: margen comprometido";
            end
        else
            controlSolicitado = robot.controlParada;

            if infoSeguimiento.metaAlcanzada
                textoEstado = "Meta alcanzada";
            elseif isempty(caminoGlobal) && textoEstado == "Navegacion"
                textoEstado = "Sin camino global";
            end
        end

        % Conteo de episodios predictivos del MPC.
        falloFactibilidadActual = ...
            controlEjecutado(k) && ~mpcFactibleActual;

        if falloFactibilidadActual && ...
                ~enFalloFactibilidadAnterior
            numeroEpisodiosSinCandidatoFactible = ...
                numeroEpisodiosSinCandidatoFactible+1;
        end

        enFalloFactibilidadAnterior = falloFactibilidadActual;

        riesgoActualRegistrado = ...
            controlEjecutado(k) && riesgoPredichoActual;

        if riesgoActualRegistrado && ~enRiesgoPredichoAnterior
            numeroEpisodiosRiesgoPredicho = ...
                numeroEpisodiosRiesgoPredicho+1;
        end

        enRiesgoPredichoAnterior = riesgoActualRegistrado;

        colisionActualRegistrada = ...
            controlEjecutado(k) && colisionPredichaActual;

        if colisionActualRegistrada && ...
                ~enColisionPredichaAnterior
            numeroEpisodiosColisionPredicha = ...
                numeroEpisodiosColisionPredicha+1;
        end

        enColisionPredichaAnterior = colisionActualRegistrada;

% -------------------------------------------------------------------------
% Modelo cinematico, contacto y recuperacion
% -------------------------------------------------------------------------
        estadoAnterior = estadoRobot;

        if colisionDinamicaAntesControl
            % No se retrocede ni se borra la ruta. Los obstaculos se
            % actualizaran otra vez al comienzo del siguiente ciclo.
            estadoRobot = estadoAnterior;
            controlAplicado = robot.controlParada;
            infoColision = infoColisionAntesControl;

        else
            [estadoCandidato,controlCandidato,~] = modelo_robot( ...
                estadoAnterior,controlSolicitado,cfg.sim.Ts,robot);

            [~,infoColisionCandidata] = detectar_colisiones( ...
                estadoCandidato, ...
                robot, ...
                escenario.obstaculosEstaticos, ...
                obstaculosDinamicos, ...
                escenario.limites);

            colisionFijaCandidata = ...
                infoColisionCandidata.colisionLimites || ...
                infoColisionCandidata.colisionEstaticos;

            if colisionFijaCandidata
                % El paso de traslacion se rechaza, pero el robot gira en
                % el sitio hacia el objetivo local o, en su defecto, la meta.
                vectorRecuperacion = ...
                    objetivoLocal-estadoAnterior(1:2);

                if norm(vectorRecuperacion) <= 1e-12
                    vectorRecuperacion = ...
                        escenario.meta-estadoAnterior(1:2);
                end

                anguloRecuperacion = atan2( ...
                    vectorRecuperacion(2),vectorRecuperacion(1));

                errorRecuperacion = wrap_to_pi_local( ...
                    anguloRecuperacion-estadoAnterior(3));

                % Se conserva, cuando sea util, el giro solicitado por
                % MPC. Si el candidato era recto, se aplica un giro hacia
                % el objetivo local limitado por la cinematica del robot.
                giroRecuperacion = controlCandidato(2);

                if abs(giroRecuperacion) <= 1e-12
                    giroRecuperacion = min(max( ...
                        errorRecuperacion/cfg.sim.Ts, ...
                        robot.limites.wMin), ...
                        robot.limites.wMax);
                end

                [estadoRobot,controlAplicado,~] = modelo_robot( ...
                    estadoAnterior,[0 giroRecuperacion], ...
                    cfg.sim.Ts,robot);

                [~,infoColision] = detectar_colisiones( ...
                    estadoRobot, ...
                    robot, ...
                    escenario.obstaculosEstaticos, ...
                    obstaculosDinamicos, ...
                    escenario.limites);

                textoEstado = ...
                    "Paso fijo rechazado: reorientacion";
            else
                % El contacto dinamico se acepta y se registra. En el ciclo
                % siguiente el robot esperara hasta que el movil se aparte.
                estadoRobot = estadoCandidato;
                controlAplicado = controlCandidato;
                infoColision = infoColisionCandidata;

                if infoColision.colisionDinamicos
                    textoEstado = ...
                        "Contacto dinamico: espera en siguiente ciclo";
                end
            end
        end

        controlesAplicados(k,:) = controlAplicado;
        controlAnterior = controlAplicado;
        trayectoriaEjecutada(k+1,:) = estadoRobot;

        hayColision = infoColision.hayColision;
        colisionOcurrida = colisionOcurrida || hayColision;

        if infoColision.colisionDinamicos && ...
                ~enColisionDinamicaAnterior
            numeroRecuperacionesColision = ...
                numeroRecuperacionesColision+1;
        end

        enColisionDinamicaAnterior = ...
            infoColision.colisionDinamicos;

        metaAlcanzadaAhora = ...
            norm(estadoRobot(1:2)-escenario.meta) <= ...
                cfg.navegacion.radioMeta;

        metaAlcanzada = ...
            metaAlcanzada || metaAlcanzadaAhora;

        if metaAlcanzadaAhora
            textoEstado = "Meta alcanzada";
        end

        motivoCiclo(k) = textoEstado;

        % El cronometro del nucleo se detiene antes de cualquier operacion
        % grafica, captura de video o pausa intencionada.
        tiempoCiclo(k) = toc(relojCiclo);

% -------------------------------------------------------------------------
% Visualizacion y captura fuera del tiempo computacional
% -------------------------------------------------------------------------
        if graficos.activa
            relojVisualizacion = tic;

            graficos = actualizar_graficos( ...
                graficos, ...
                k, ...
                estadoRobot, ...
                trayectoriaEjecutada(1:k+1,:), ...
                caminoGlobal, ...
                obstaculosDinamicos, ...
                vistaPlanificador, ...
                textoEstado);

            % Este es el unico drawnow repetitivo de la animacion. Debe
            % permanecer directamente en el cuerpo del Live Script.
            drawnow;

            if exportarAnimacion && exportacionIniciada
                [exportacion,~] = exportar_animacion( ...
                    "capturar",exportacion,graficos);
            end

            tiempoVisualizacion(k) = ...
                toc(relojVisualizacion);

            if graficos.pausa && graficos.tPausa > 0
                pause(graficos.tPausa);
            end
        end

% -------------------------------------------------------------------------
% Condiciones configuradas de terminacion
% -------------------------------------------------------------------------
        if metaAlcanzadaAhora && ...
                cfg.terminacion.detenerEnMeta
            break;
        end

        if hayColision && ...
                cfg.terminacion.detenerEnColision
            break;
        end
    end

catch errorSimulacion
    % Evita dejar un archivo de video abierto o incompleto si la simulacion
    % se interrumpe por un error.
    if exportarAnimacion && exportacionIniciada && ...
            ~isempty(fieldnames(exportacion))
        try
            [exportacion,~] = exportar_animacion( ...
                "cancelar",exportacion);
        catch
            % Se conserva siempre el error original de la simulacion.
        end
    end

    rethrow(errorSimulacion);
end

%% Finalizacion de la exportacion y tiempo real de reloj
if exportarAnimacion && exportacionIniciada
    [exportacion,infoExportacion] = exportar_animacion( ...
        "finalizar",exportacion);
end

tiempoRelojTotal = toc(relojTotal);

%% Calculo normalizado de los resultados
registroTiempos = struct();
registroTiempos.planificacion = tiempoPlanificacion;
registroTiempos.control = tiempoControl;
registroTiempos.ciclo = tiempoCiclo;
registroTiempos.visualizacion = tiempoVisualizacion;
registroTiempos.planificacionEjecutada = planificacionEjecutada;
registroTiempos.controlEjecutado = controlEjecutado;
registroTiempos.inicializacion = 0;
registroTiempos.tiempoRelojTotal = tiempoRelojTotal;
registroTiempos.incluyeVisualizacion = false;
registroTiempos.incluyePausas = false;
registroTiempos.metaAlcanzada = metaAlcanzada;

registro = struct();
registro.arquitectura = cfg.nombreArquitectura;
registro.semilla = cfg.semilla;
registro.pasosEjecutados = pasosEjecutados;
registro.trayectoria = trayectoriaEjecutada;
registro.historialObstaculos = historialObstaculos;
registro.controles = controlesAplicados;
registro.tiempos = registroTiempos;
registro.numeroReplanificaciones = numeroReplanificaciones;
registro.numeroFallosPlanificacion = numeroFallosPlanificacion;
registro.caminoGlobalFinal = caminoGlobal;

[resultado,filaResultado] = resultados( ...
    registro,escenario,robot,cfg);

% Diagnosticos especificos de este main, adicionales a las metricas comunes.
resultado.contadores.numeroPlanificacionesMain = ...
    numeroPlanificaciones;
resultado.contadores.numeroEpisodiosSinCandidatoFactibleMPC = ...
    numeroEpisodiosSinCandidatoFactible;
resultado.contadores.numeroPasosSinCandidatoFactibleMPC = ...
    nnz(controlEjecutado(1:pasosEjecutados) & ...
        ~mpcFactiblePorPaso(1:pasosEjecutados));
resultado.contadores.numeroEpisodiosRiesgoPredichoMPC = ...
    numeroEpisodiosRiesgoPredicho;
resultado.contadores.numeroPasosRiesgoPredichoMPC = ...
    nnz(riesgoPredichoPorPaso(1:pasosEjecutados));
resultado.contadores.numeroEpisodiosColisionPredichaMPC = ...
    numeroEpisodiosColisionPredicha;
resultado.contadores.numeroPasosColisionPredichaMPC = ...
    nnz(colisionPredichaPorPaso(1:pasosEjecutados));
resultado.contadores.numeroRecuperacionesColision = ...
    numeroRecuperacionesColision;
resultado.contadores.numeroAproximacionesParciales = ...
    numeroAproximacionesParciales;

resultado.historiales.mpcEjecutado = ...
    controlEjecutado(1:pasosEjecutados);
resultado.historiales.mpcFactible = ...
    mpcFactiblePorPaso(1:pasosEjecutados);
resultado.historiales.riesgoPredichoMPC = ...
    riesgoPredichoPorPaso(1:pasosEjecutados);
resultado.historiales.colisionPredichaMPC = ...
    colisionPredichaPorPaso(1:pasosEjecutados);
resultado.historiales.distanciaMinimaPredichaMPC = ...
    distanciaMinimaPredichaPorPaso(1:pasosEjecutados);
resultado.historiales.costeMPC = ...
    costeMPCPorPaso(1:pasosEjecutados);
resultado.historiales.numeroCandidatosFactiblesMPC = ...
    numeroCandidatosFactiblesPorPaso(1:pasosEjecutados);
resultado.historiales.fraccionCandidatosFactiblesMPC = ...
    fraccionCandidatosFactiblesPorPaso(1:pasosEjecutados);
resultado.historiales.numeroViolacionesSeguridadMPC = ...
    numeroViolacionesSeguridadPorPaso(1:pasosEjecutados);
resultado.historiales.motivoMPC = ...
    motivoMPCPorPaso(1:pasosEjecutados);
resultado.historiales.distanciaAlCamino = ...
    distanciaAlCaminoPorPaso(1:pasosEjecutados);
resultado.historiales.motivoCiclo = ...
    motivoCiclo(1:pasosEjecutados);
resultado.historiales.motivoRRT = ...
    motivoRRT(1:pasosEjecutados);
resultado.historiales.motivoReplanificacion = ...
    motivoReplanificacion(1:pasosEjecutados);
resultado.historiales.motivoSeguimiento = ...
    motivoSeguimiento(1:pasosEjecutados);
resultado.historiales.caminoParcialPorPaso = ...
    caminoParcialPorPaso(1:pasosEjecutados);

mascaraMPC = controlEjecutado(1:pasosEjecutados);

resultado.metricas.mpc = struct();
resultado.metricas.mpc.numeroEvaluaciones = nnz(mascaraMPC);

if any(mascaraMPC)
    factiblesMPC = ...
        mpcFactiblePorPaso(1:pasosEjecutados);
    factiblesMPC = factiblesMPC(mascaraMPC);

    distanciasMPC = ...
        distanciaMinimaPredichaPorPaso(1:pasosEjecutados);
    distanciasMPC = distanciasMPC(mascaraMPC);

    costesMPC = costeMPCPorPaso(1:pasosEjecutados);
    costesMPC = costesMPC(mascaraMPC);

    candidatosFactiblesMPC = ...
        numeroCandidatosFactiblesPorPaso(1:pasosEjecutados);
    candidatosFactiblesMPC = ...
        candidatosFactiblesMPC(mascaraMPC);

    resultado.metricas.mpc.porcentajeEvaluacionesFactibles = ...
        100*nnz(factiblesMPC)/numel(factiblesMPC);
    resultado.metricas.mpc.distanciaMinimaPredichaMinima = ...
        min(distanciasMPC,[],'omitnan');
    resultado.metricas.mpc.costeMedio = ...
        mean(costesMPC,'omitnan');
    resultado.metricas.mpc.candidatosFactiblesMedios = ...
        mean(candidatosFactiblesMPC,'omitnan');
else
    resultado.metricas.mpc.porcentajeEvaluacionesFactibles = NaN;
    resultado.metricas.mpc.distanciaMinimaPredichaMinima = NaN;
    resultado.metricas.mpc.costeMedio = NaN;
    resultado.metricas.mpc.candidatosFactiblesMedios = NaN;
end

resultado.metricas.mpc.numeroCandidatosNominal = ...
    cfg.mpc.nVelocidades*cfg.mpc.nGiros;
resultado.metricas.mpc.horizontePasos = cfg.mpc.Np;
resultado.metricas.mpc.horizontePrediccion_s = ...
    cfg.mpc.Np*cfg.sim.Ts;
resultado.metricas.mpc.criterioFactibilidad = ...
    "respeta_umbral_estatico_y_dinamico_en_todo_el_horizonte";
resultado.metricas.mpc.criterioSeleccionSinFactible = ...
    "menos_violaciones_menor_coste_mayor_continuidad_y_seguridad";

resultado.detalles.ultimaPlanificacionRRT = infoRRT;
resultado.detalles.ultimoControlMPC = infoMPC;
resultado.detalles.ultimoSeguimiento = infoSeguimiento;
resultado.detalles.ultimaColisionEvaluada = infoColision;
resultado.detalles.arbolRRTFinal = arbolRRT;
resultado.detalles.controlAnteriorFinal = controlAnterior;

resultado.identificacion.modoMovimientoObstaculos = ...
    "aleatorio_con_rebote";
resultado.identificacion.semillaPlanificador = ...
    cfg.semilla;
resultado.identificacion.modoSemillaObstaculos = ...
    modoSemillaObstaculos;
resultado.identificacion.semillaObstaculos = ...
    semillaObstaculos;
resultado.identificacion.generadoresAleatoriosSeparados = true;

resultado.detalles.configuracionFlujoObstaculos = ...
    infoFlujoObstaculos;
resultado.detalles.inicializacionObstaculosAleatorios = ...
    infoInicializacionObstaculos;
resultado.detalles.ultimoMovimientoObstaculosAleatorios = ...
    infoMovimientoObstaculos;
resultado.contadores.numeroRebotesAleatoriosObstaculos = ...
    numeroRebotesAleatorios;

if exportarAnimacion && exportacionIniciada
    resultado.detalles.exportacion = infoExportacion;
end

%% Guardado opcional
archivoResultadoMAT = "";
archivoResultadoCSV = "";

if guardarResultado
    carpetaResultados = fullfile( ...
        raizProyecto,"salidas","resultados");

    if exist(char(carpetaResultados),'dir') ~= 7
        mkdir(char(carpetaResultados));
    end

    nombreBase = "rrt_mpc_aleatorio_"+string(escenario.id)+ ...
        "_semilla_"+string(cfg.semilla)+ ...
        "_obst_"+string(semillaObstaculos);

    archivoResultadoMAT = fullfile( ...
        carpetaResultados,nombreBase+".mat");

    archivoResultadoCSV = fullfile( ...
        carpetaResultados,nombreBase+".csv");

    save(char(archivoResultadoMAT), ...
        'resultado','filaResultado','registro', ...
        'cfg','escenario','robot','-v7.3');

    writetable(filaResultado,char(archivoResultadoCSV));
end

%% Resumen mostrado en el Live Editor
disp(" ");
disp("Resumen de la ejecucion RRT* + MPC con obstaculos aleatorios");
disp(filaResultado);

disp("Semilla fija del planificador: "+string(cfg.semilla));
disp("Semilla independiente de obstaculos: "+ ...
    string(semillaObstaculos));
disp("Modo del flujo de obstaculos: "+modoSemillaObstaculos);
disp("Rebotes aleatorios acumulados: "+string(numeroRebotesAleatorios));

disp("Episodios sin candidato factible MPC: "+ ...
    string(numeroEpisodiosSinCandidatoFactible));
disp("Pasos con riesgo predicho por MPC: "+ ...
    string(nnz(riesgoPredichoPorPaso(1:pasosEjecutados))));
disp("Pasos con colision fisica predicha por MPC: "+ ...
    string(nnz(colisionPredichaPorPaso(1:pasosEjecutados))));

if exportarAnimacion && exportacionIniciada
    disp("Animacion: "+string(infoExportacion.archivo));
end

if guardarResultado
    disp("Resultado MAT: "+string(archivoResultadoMAT));
    disp("Resultado CSV: "+string(archivoResultadoCSV));
end
