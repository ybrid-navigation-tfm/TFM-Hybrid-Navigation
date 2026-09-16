%[text] # RRT\* + APF en un entorno dinamico
%[text] Ensambla los modulos del proyecto TFM\_RobotNavigation para ejecutar una simulacion completa de la arquitectura RRT\* + APF. El planificador global es el mismo RRT\* convencional que se utilizara en RRT\* + MPC. APF actua exclusivamente como controlador local y el robot se integra mediante el modelo de uniciclo \[x y theta\] con control \[v w\]. La visualizacion sigue el formato eficiente del codigo PRM + MPC: los objetos graficos se crean una vez, se actualizan sin clf y el arbol RRT\* anterior se sustituye completamente cuando existe una nueva planificacion.
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
%%
%[text] ## Opciones de la ejecucion
%[text] Cambie solamente estas opciones para ejecutar otra demostracion.
modoEjecucion = "visual";       % "visual" o "batch"
idEscenario = "alta";          % "baja", "media" o "alta"
semilla = 7;

% La pausa solo modifica la velocidad con la que se observa la animacion;
% no modifica el periodo fisico de muestreo de la simulacion.
factorVelocidadAnimacion = 1.0;  % 1 = tiempo simulado aproximado

% Exportacion y guardado son opcionales. Permanecen desactivados durante el
% desarrollo para no mezclar su coste con los tiempos de los algoritmos.
exportarAnimacion = false;
formatoAnimacion = "mp4";        % "mp4", "m4v", "avi" o "gif"
guardarResultado = false;
%%
%[text] ## Localizacion de la raiz y configuracion del path
%[text] En los Live Scripts, mfilename puede devolver el nombre de un helper temporal del Live Editor en lugar de la ruta real del archivo. Por ello, la fuente principal de la raiz es el proyecto MATLAB actualmente abierto.
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
    error('ejecutar_rrt_apf:RaizNoEncontrada', ...
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
        error('ejecutar_rrt_apf:CarpetaAusente', ...
            'No existe la carpeta requerida: %s',char(carpetaModulo));
    end

    addpath(char(carpetaModulo),'-begin');
end

rehash path;
fprintf('Raiz del proyecto: %s\n',raizProyecto); %[output:179f30fb]


% Comprobacion temprana de las dependencias utilizadas por este main.
funcionesNecesarias = [ ...
    "parametros_generales", ...
    "escenarios", ...
    "configuracion_robot", ...
    "rrt_star", ...
    "seguimiento_trayectoria", ...
    "apf", ...
    "actualizar_obstaculos", ...
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
        error('ejecutar_rrt_apf:DependenciaAusente', ...
            'No se encontro la funcion %s.m en el path del proyecto.', ...
            funcionesNecesarias(iFuncion));
    end
end
%%
%[text] ## Configuracion comun, escenario y robot
cfg = parametros_generales(modoEjecucion);
cfg.semilla = semilla;
cfg.nombreArquitectura = "RRT* + APF";
cfg.planificador = @rrt_star;
cfg.controlador = @apf;

if ~isnumeric(factorVelocidadAnimacion) || ...
        ~isscalar(factorVelocidadAnimacion) || ...
        ~isfinite(factorVelocidadAnimacion) || ...
        factorVelocidadAnimacion <= 0
    error('ejecutar_rrt_apf:FactorAnimacionNoValido', ...
        'factorVelocidadAnimacion debe ser un escalar positivo.');
end

if cfg.visual.activa
    cfg.visual.pausa = true;
    cfg.visual.tPausa = ...
        cfg.sim.Ts/factorVelocidadAnimacion;
end

escenario = escenarios(idEscenario);
robot = configuracion_robot();

% La semilla se fija en el main para que RRT* + APF y RRT* + MPC puedan
% utilizar exactamente las mismas secuencias aleatorias en la comparacion.
rng(cfg.semilla,'twister');
%%
%[text] ## Inicializacion del estado y de los historiales
estadoRobot = escenario.inicio;
numeroRecuperacionesColision = 0;
enColisionDinamicaAnterior = false;
obstaculosDinamicos = escenario.obstaculosDinamicos;
caminoGlobal = zeros(0,2);
caminoGlobalEsParcial = false;

arbolRRT = struct();
infoRRT = struct();
infoAPF = struct();
infoSeguimiento = struct();
infoReplanificacion = struct();
infoColision = struct();

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
minimoLocalPorPaso = false(maxPasos,1);
distanciaAlCaminoPorPaso = nan(maxPasos,1);
motivoCiclo = strings(maxPasos,1);
motivoRRT = strings(maxPasos,1);
motivoReplanificacion = strings(maxPasos,1);
motivoSeguimiento = strings(maxPasos,1);
caminoParcialPorPaso = false(maxPasos,1);

numeroPlanificaciones = 0;
numeroReplanificaciones = 0;
numeroFallosPlanificacion = 0;
numeroEpisodiosMinimoLocal = 0;
numeroAproximacionesParciales = 0;
enMinimoLocalAnterior = false;

pasosEjecutados = 0;
metaAlcanzada = ...
    norm(estadoRobot(1:2)-escenario.meta) <= ...
        cfg.navegacion.radioMeta;
colisionOcurrida = false;

vistaPlanificador = struct( ...
    'actualizar',false, ...
    'nodos',zeros(0,2), ...
    'aristas',zeros(0,2));
%%
%[text] ## Preparacion opcional de la exportacion externa
%[text] El video permanece desactivado por defecto. Esta preparacion no crea ningun fotograma y no interviene en la animacion interna del Live Editor.
exportacion = struct();
infoExportacion = struct();
archivoAnimacion = "";
exportacionIniciada = false;

if exportarAnimacion
    extension = lower(strtrim(string(formatoAnimacion)));

    if ~any(extension == ["mp4","m4v","avi","gif"])
        error('ejecutar_rrt_apf:FormatoAnimacionNoValido', ...
            'El formato debe ser mp4, m4v, avi o gif.');
    end

    nombreAnimacion = "rrt_apf_"+string(escenario.id)+ ...
        "_semilla_"+string(cfg.semilla)+"."+extension;

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
%%
%[text] ## Visualizacion y simulacion RRT\* + APF
%[text] IMPORTANTE PARA EL LIVE EDITOR: 1. La figura se crea directamente en este script. 2. Desde esta llamada a figure hasta el final del bucle no hay separadores de seccion "%%". 3. El unico drawnow efectivo de la animacion se ejecuta dentro del bucle, despues de actualizar todos los objetos graficos.
if cfg.visual.activa
    figuraLive = figure('Color','w'); %[output:4dbd3040]
else
    figuraLive = [];
end

graficos = inicializar_figura( ... %[output:4dbd3040]
    escenario, ... %[output:4dbd3040]
    cfg, ... %[output:4dbd3040]
    cfg.nombreArquitectura, ... %[output:4dbd3040]
    figuraLive); %[output:4dbd3040]

graficos = dibujar_entorno( ... %[output:4dbd3040]
    graficos, ... %[output:4dbd3040]
    escenario, ... %[output:4dbd3040]
    obstaculosDinamicos); %[output:4dbd3040]

graficos = dibujar_robot( ... %[output:4dbd3040]
    graficos, ... %[output:4dbd3040]
    estadoRobot, ... %[output:4dbd3040]
    robot); %[output:4dbd3040]

% La exportacion externa se abre despues de crear los objetos, pero no
% captura todavia ningun fotograma.
if exportarAnimacion && graficos.activa
    [exportacion,infoExportacion] = exportar_animacion( ...
        "iniciar",[],graficos,cfg, ...
        archivoAnimacion,opcionesExportacion);

    exportacionIniciada = true;
end

relojTotal = tic;
try %[output:group:4fce169c]
    for k = 1:maxPasos
        pasosEjecutados = k;
        relojCiclo = tic;

        textoEstado = "Navegacion";
        minimoLocalActual = false;
        controlSolicitado = robot.controlParada;
        motivoRRT(k) = "no_ejecutado";

        vistaPlanificador = struct( ...
            'actualizar',false, ...
            'nodos',zeros(0,2), ...
            'aristas',zeros(0,2));

% -------------------------------------------------------------------------
% Movimiento de los obstaculos dinamicos
% -------------------------------------------------------------------------
        obstaculosDinamicos = actualizar_obstaculos( ...
            obstaculosDinamicos, ...
            escenario.obstaculosEstaticos, ...
            escenario.limites, ...
            cfg.sim.Ts);

        for iObstaculo = 1:numeroDinamicos
            historialObstaculos(k+1,iObstaculo,:) = reshape( ...
                obstaculosDinamicos(iObstaculo).pos,1,1,2);
        end

% -------------------------------------------------------------------------
% Decision global de replanificacion
% -------------------------------------------------------------------------
        % RRT* y el criterio global reciben una lista dinamica vacia.
        % Los obstaculos moviles se delegan exclusivamente al APF local.
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
% Control local APF
% -------------------------------------------------------------------------
        if colisionDinamicaAntesControl
            % El obstaculo movil puede ser mas rapido que el robot. Durante
            % el contacto el robot espera; la ruta global se conserva.
            controlSolicitado = robot.controlParada;
            textoEstado = "Esperando separacion del dinamico";

        elseif colisionNoDinamicaAntesControl
            % Esta rama solo protege frente a un estado numericamente
            % invalido. El movimiento normal no acepta pasos contra muros.
            controlSolicitado = robot.controlParada;
            textoEstado = "Reorientacion junto a obstaculo fijo";

        elseif infoSeguimiento.caminoUtilizable && ...
                ~infoSeguimiento.metaAlcanzada

            relojControl = tic;

            [controlSolicitado,infoAPF] = apf( ...
                estadoRobot, ...
                objetivoLocal, ...
                escenario.obstaculosEstaticos, ...
                obstaculosDinamicos, ...
                escenario.limites, ...
                robot, ...
                cfg);

            tiempoControl(k) = toc(relojControl);
            controlEjecutado(k) = true;

            minimoLocalActual = infoAPF.minimoLocal;
            minimoLocalPorPaso(k) = minimoLocalActual;

            if minimoLocalActual
                textoEstado = "Escape de minimo local APF";
            end
        else
            controlSolicitado = robot.controlParada;

            if infoSeguimiento.metaAlcanzada
                textoEstado = "Meta alcanzada";
            elseif isempty(caminoGlobal) && textoEstado == "Navegacion"
                textoEstado = "Sin camino global";
            end
        end

        if minimoLocalActual && ~enMinimoLocalAnterior
            numeroEpisodiosMinimoLocal = ...
                numeroEpisodiosMinimoLocal+1;
        end

        enMinimoLocalAnterior = minimoLocalActual;

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

                giroRecuperacion = min(max( ...
                    cfg.apf.kGiro*errorRecuperacion, ...
                    robot.limites.wMin), ...
                    robot.limites.wMax);

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

            graficos = actualizar_graficos( ... %[output:4dbd3040]
                graficos, ... %[output:4dbd3040]
                k, ... %[output:4dbd3040]
                estadoRobot, ... %[output:4dbd3040]
                trayectoriaEjecutada(1:k+1,:), ... %[output:4dbd3040]
                caminoGlobal, ... %[output:4dbd3040]
                obstaculosDinamicos, ... %[output:4dbd3040]
                vistaPlanificador, ... %[output:4dbd3040]
                textoEstado); %[output:4dbd3040]

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
end %[output:group:4fce169c]
%%
%[text] ## Finalizacion de la exportacion y tiempo real de reloj
if exportarAnimacion && exportacionIniciada
    [exportacion,infoExportacion] = exportar_animacion( ...
        "finalizar",exportacion);
end

tiempoRelojTotal = toc(relojTotal);
%%
%[text] ## Calculo normalizado de los resultados
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
resultado.contadores.numeroEpisodiosMinimoLocalAPF = ...
    numeroEpisodiosMinimoLocal;
resultado.contadores.numeroPasosMinimoLocalAPF = ...
    nnz(minimoLocalPorPaso(1:pasosEjecutados));
resultado.contadores.numeroRecuperacionesColision = ...
    numeroRecuperacionesColision;
resultado.contadores.numeroAproximacionesParciales = ...
    numeroAproximacionesParciales;

resultado.historiales.minimoLocalAPF = ...
    minimoLocalPorPaso(1:pasosEjecutados);
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

resultado.detalles.ultimaPlanificacionRRT = infoRRT;
resultado.detalles.ultimoControlAPF = infoAPF;
resultado.detalles.ultimoSeguimiento = infoSeguimiento;
resultado.detalles.ultimaColisionEvaluada = infoColision;
resultado.detalles.arbolRRTFinal = arbolRRT;

if exportarAnimacion && exportacionIniciada
    resultado.detalles.exportacion = infoExportacion;
end
%%
%[text] ## Guardado opcional
archivoResultadoMAT = "";
archivoResultadoCSV = "";

if guardarResultado
    carpetaResultados = fullfile( ...
        raizProyecto,"salidas","resultados");

    if exist(char(carpetaResultados),'dir') ~= 7
        mkdir(char(carpetaResultados));
    end

    nombreBase = "rrt_apf_"+string(escenario.id)+ ...
        "_semilla_"+string(cfg.semilla);

    archivoResultadoMAT = fullfile( ...
        carpetaResultados,nombreBase+".mat");

    archivoResultadoCSV = fullfile( ...
        carpetaResultados,nombreBase+".csv");

    save(char(archivoResultadoMAT), ...
        'resultado','filaResultado','registro', ...
        'cfg','escenario','robot','-v7.3');

    writetable(filaResultado,char(archivoResultadoCSV));
end
%%
%[text] ## Resumen mostrado en el Live Editor
disp(" "); %[output:850af005]
disp("Resumen de la ejecucion RRT* + APF"); %[output:03f97987]
disp(filaResultado); %[output:2c78bfc3]

disp("Episodios de minimo local APF: "+ ... %[output:group:3e6ebd00] %[output:7f808f90]
    string(numeroEpisodiosMinimoLocal)); %[output:group:3e6ebd00] %[output:7f808f90]

if exportarAnimacion && exportacionIniciada
    disp("Animacion: "+string(infoExportacion.archivo));
end

if guardarResultado
    disp("Resultado MAT: "+string(archivoResultadoMAT));
    disp("Resultado CSV: "+string(archivoResultadoCSV));
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":40}
%---
%[output:179f30fb]
%   data: {"dataType":"text","outputData":{"text":"Raiz del proyecto: \/MATLAB Drive\/TFM\n","truncated":false}}
%---
%[output:4dbd3040]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAi0AAAGiCAYAAAAr5\/biAAAQAElEQVR4Aey9DZhU5X3\/\/VtkRRdQYwRfQJnlIpqAaLRCbUlgoWAj\/FP75C8xEAOLuWqwwesxotY8uZTd5GpNVVpzRf9a\/FeWpAUTrNW2aqoEZmlsqBiNojYayw4KagCJQVhF3p7zudl7ODs7szszOy9nZr4wv7nf3z5n9pzv3Pd9zgw4rH8iIAIiIAIiIAIiUAEEBpj+iYAIiIAIiIAI9IOAipaKgERLqUirHREQAREQAREQgX4RkGjpFz4VFgEREIHoElDPRKDaCEi0VNsR1XhEQAREQAREoEoJSLRU6YHVsEQgugTUMxEQARHIj4BES37cVEoEREAEREAERKDEBCRaSgxczUWXgHomAiIgAiIQbQISLdE+PuqdCIiACIiACIhAFwGJli4Q0XXUMxEQAREQAREQAQhItEBBJgIiIAIiIAIiEHkCeYuWyI9MHRQBERABERABEagqAhItVXU4NRgREAEREIEKIqCu5khAoiVHYJWQffv27TZlyhQbPXp0WiONPH4s9913X9p8vjzpPi\/umjVres1PuRtuuIGshou5QBHfaIN2sblz59qePXu6tcZ4GTfp6Sw8Rvzp8vg40rtV3hWgTdrG8HdFZ3TCHM8\/\/3zbtGlTj7x+XLjhRMKUD8dVqz987HIZM8eJY+aPR7b1cBw4Hhj+\/nBN7UO6uhgT\/cRNl06cr4fjThhjPIwNl3AUDF5ww\/CXuk\/83cEEnjCjffj09bcPe8qks3KNhb7LehKQaOnJpOpj3nzzTZs9e7bxx5zNYG+\/\/XbzJ4Bs8pPnV7\/6lfHH\/v7779vpp59unEhyrYN6sjHGsXHjxmTWl19+2To6OpLhbDz5jDGbenvL85Of\/CSZDKenn346Gc7k8Sflhx9+OFMWxdcAAQTBjBkzbNu2bcUZbYXWumHDBsMqtPvqdhYEJFqygFTJWZYtW2abN29OGn\/QZ555piFcXnzxxW5Du\/jii424cP6bbrrJ5Vm\/fn1y9mL69Ok96iNTuK1\/+7d\/s3\/4h39wJ5Af\/OAH9s1vftMWLlxItozmL8h828lF4NBnxjNu3DjD+hIA4X4y1i984QuuT+ExEpGOB\/n7Ggdl+zIvtIYOHWpcfMif2j5xsvwJcJw4XitXrrQhQ4bY8OHDrb293X12+QznX7NKZiIwfvx4e+GFF5zhz5SvGPH8TX3nO9\/pUXX4uPN5wDgHkJFzIX\/\/fB6IDxvx5OHvs9RjoV1ZegISLem5VG1sQ0ODjRgxwo3v9ddfd25vb2PGjHHJfKPr7Ox0\/mzemG697LLLbOnSpc7wE5dN2Vzz+BmLWbNmGUb5XATA5z73OYq4b625jNEVyvMtLLQWLFhgiJe+Zog4KTM+hCfNXn311W4ZkHjCLB0g+Lwxu4UQJC2TkU4+X4bZMb7Fh\/On5iFv+FjSPtPvlCUeP3kIU5dPD8f5+n2fcTHyYPSJdn2+LrebE643XRmEbzg+nJ9++srwkw+jzxwbnxZ2w\/0jb7o+hvOQvnfv3nAVBfHTX\/6eEOeIdcQ17frK\/bjpI0Z+n8bxYIzY1772NbfMi5948lAPZbwxhtTjQH0+HZcylMWoh\/ow\/MRh1EFd5PdGPaRh4WPzyCOPuJlZny9cP3kzGbOl8MiU7uNpy4ubW265xYlZn+Zd+sZsJqLGf3HzaXLLS0Cipbz8S946yyZcHGnYCxL8mcwLAoQOgidTvtR4\/80FF+MbDG5qvv6GOQGxNMRFf9KkSYbhZ4yMNZv68x1jNnVnyuPbnDx5sp177rlZzRBlqot4TuycZPF7Q9y0tLT4YA8XdmERRAYuhFdeeWVyf026PORDMHFxxO+NssT7Cwfhr3\/9624mKRy3aNGiHkuT9B3zddF36uJi5+PCLhdEvgH7ekmjDOOhz4SzMS5OtOPz0mcuZLg+Djcbvql56M8999xD8ZIZfeDiHW6Q8aU7Vk899ZTLdtJJJ9mpp57q9p+FjwGJjCH8GaIe6iPNG2Vo14dTXY4Hx4W6wmnUQ33hOPzXX3+9m6HFj1F\/unykeePzwHiYacV8fDoXPnxumElJd06iv32JmnT1Kq40BCRaSsO5bK1wYvDfWHD9NzS+nWHhjnFSOe+889y3L\/JinDDIc9VVV7kpdvyFNC5K\/hsYbdMH6ufEQvtYbycsvhVzAvIn3sbGxj4FQCqTTGOkL\/SJPnijr\/SZPiYtRw8nRYQWxRCOLF0gXgj3NkPENPdjjz1m\/rgxxc1yB+Woj2+F9BmB6L8dvvXWW8llPfKFjXHDLlyOEzkX7BUrVrisHIfUPLRL4r333psUN4QxytO+z7N161YnWsJx7733nv3mN78he9IQmo8++qhbuvFlexOe9I9+Mk7qxmibvjKuZMV9eLx4hCmfJV9PuJg\/XmFOtEsez9fnIY406uFYUIa4QhoXWljBjPpp584773THggu3j6MPPt+qVat6CEV4kac9WDKjf319hhgj9ZDXj9HXT7sIB9JSLdfPkD8WHA\/81Nfb3wV\/j7fddhvZ7Fvf+pabtXSBNG+MgXHCbv78+WlymFsi53NE21jaTIosGwGJlrKhL1\/DXBT8On9fveCPlpMHJ8q+8pYj3V90JkyY4KZ5EQAILPrS24mOdG+cwDj5lmqM8OSkyMUFUUQ\/uIAQ7u1CTb50hpjhwoMhBpia50KRLm84zi8Pzpkzx7EjjYsfFzJcf4InntkH2sHPZwJDNKRuHvZLbYyL8ZA\/XRzxYWPWxO8boG4sXf2UCfeLcXpB6cWKHxd5e7NwPXxm+OyQn4sZnwn8GOOGLZaJb\/iYcix9OdjiL4VxLGDGZwt+cPFfUlKFIuNjnL5f2YyRsVMPx9WPkWPW2x6WMONsP0P+WHA8vJj3\/Uzn+r1z11xzjZu1TJfHx\/EZgQ+zMXzB8fHeRQA98MADLkjb9MEF9BYZAhItkTkUBe+IqxCBwkWIb0YuInjLdFLnRMfJl29unJiCrEXf58FJAQFFH2mbPtAu\/SUOY0MlcakWPiFyMuIkjTGTQt5MAsAzoW6Mky4nX8qEjb7QJ\/J4o6\/0OZwvV78XWpw8aYM+4xLmosPFJ5c6OdEyA0Q9\/iLVV3nKMEvQVz7SucCxfIAfY\/xnnHEG3m6Wmo\/EdHHEpxozTj4uU\/0+vS+XcTG+vvJlSmeszNz5dOrKla8vGx6XjyuHy+cK0ZGp7WzGSHnqyVRHb\/Gpn4P+HmPfFucAZn84X3kh5dNSXcbIFxnivTDCHzaWlDlvZFNfuJz8pSMg0VI61mVtiQu\/\/6Pm2ylr+Zk6xLeuu+++202zciG97rrrMi4xZKqjFPEICvqXqS1OsLkKgEx1FSqekyzT073Vx4mVE2xvecJpiEyMOC\/IEH2EM1kuFw04csHyddE3hIEPF8INC+lc6vfj9aISN1thyR4t9mql9p+xMqPg42GLEfbtpfJF6HBhJk\/YwuMKxxfTjwDmbwMWYettJpHxYfQr1zFSpi8r1meIcXIOwBg3M3x+HJznEJt8nuifFyQcJ44XcanG+YK+8rng85GarnD5CZRetJR\/zDXbA060fIMAAFOg\/o+ZcKox88B0K\/GcBDD8UTI\/Y4EYC5+c8TNW+sq3MIQC\/iiYP8lyHGBKX72xRMUJlXisr\/76C6J3OWljHFeET1\/l\/SxAmBH7h5ix4WTPSZtlN+phY6LnSN8w+srGZ9L7a+E9EdSNZaofUe375T\/HjJk+03fGkE1\/wsLN10M5v18GP5YNXy6CzM5w8WTWj3Lwgi3+YhniCpFF\/RwLmMEOI44vJzDhji76Q1w6688YqZs2aCu17vCxKvZnKLXt1DCcECSZlobI7zloaQga0TSJlmgel6L0ihMI68pUzkmNtWD8mYy7SLgIkh4+4RAuhnER4VsyF3FmhnprgxOwn7HweybC+f0JnIsIQiGcVk6\/F1pcdDke4b6wxs4JlTifD38m45sk+1eOPfZYl4VjyjdNDL+L7OUNsYd4ghHHmQsPdVLET5+z74ELYTiPX35D1CJuyd9f42LC0hZ98PWH97mk1u\/7xTgZL4af8TCu1PyZwqn10L4XHb6MF3fUTzsYfp+Oy7H0+1dgSD0whRvpxTLPDcHG5wdmtAVD+oBLmL7RR\/zprL9jZKxYuro9Y1iQJ9yv\/n6GmD3ifOGNv3XaoB98ceF8wnmFsBckLG36OOK9IXz9DKLn4dOi6NZqnyRaauzI80fuT+rp7v4I4+APm4fC+YsWJ+Nwejn9nJw4CdI3vuWm9oUTeC4CILV8McJhoZXupAhvvuHRNoKM\/PjDRh5\/THw8AogTtA9z0v7pT39qXMBZn2da3KeFXS5i4buRfBrLA3xOCCNKmDKnTsLeyNOXsPR5s3H5TGI+L342A\/twqku\/EN0cf5\/GeFevXp3cVOzje3NT66E+BDquLweLbPjCIzUft3z7egrp0m8u+Kl1wgx24Xj6RN\/Ccan+XMbIsQ+X57NBHJ\/NcLz309dSfIZ8e5lcL1oypfOMJp5HlSld8dEgINESjeNQ0F5wMeJOB759cDJKrZwTG2l+AyonNMLhbyW+DCcc8pFOOR\/v3b7a8vkK7TIu+kTf6GNq\/ZxAGQ956Hcu\/eyNR2o7uYTDfaCNdGWJp88cP\/LTd8K4Pj\/jZdzE4xL25Yhj3Ig26vDpvmyqG+ZEWQy24Xx95aGfqW1lGxduhzHSPobfp\/m6iA\/3jXEzPuIx+kBeXy7VJY085M1UD\/V9+ctfdk90xU8b1JMt39R8ixcvdrdxc0zgSF35mK83zMXHMZ5w\/eQhzhv5fJuMh3Fh+H08Lvl8GerL9BmCnc+HS14\/tvHj0z8Rl3Tykd8b9dAulunY+D5RljrI25uRh7y0QdlwXs8FNxzv\/Zn64NPlRoOAREs0joN6IQIiUCACzFCxz8LPDGZaDihQc6pGBESghAQkWkoIO7UpHsbEngTWeLHUp0qyoZB4DH9qeYVFQAR6EmDDJRtUSWHJiOUR\/DIREIHKJyDRUqZjyLdBHmfOb\/MwlcnGPvYxeHGCoGGakztKMPzElam7ajYLAn5qmmOFP4siNZ+FqXo+\/7iFgsGyB8sf1MtyENP+haq7GPWwTEJfcYtRv+oUgWoiINFSpqPJiZQTqj9REWZDpd8sxsa1kSNHGuvKnIQnTpxoxGXbXWZnss2rfNkRKDzT7Nqt5lxiWvijK6ZiWngC0alRoiU6x6JbTxAvqWvxxHXLlCEQj8ddCq4sbvF4vCAG1HiB6lI9cXdMxPQIh3gBP1diGnefrXiBmcJVVn4CEi3lPwauByz98ICt8DNHwrfFhv2uQMob36688YwNknHzMZW5ytIxENP0XNKxyjZOTMU0289KOfPxOfXnV\/yy8hGQaCkf+2TLfn8LD4byy0XJxCw9rIl788\/lGDVqlMkKx4CZL\/EsHE9YimlheYpp4Xn68+natWvd7etZnpKVrUgEKki0FIlAmatFsMyePdvYz5K6GTG8HBT2Z9vldevWmaxwDH74wx+KZ4E\/Tk+vLQAAEABJREFUU2JauM+n\/1sX08IyjcVi2Z5yla8EBCRaSgA5UxNesPCI7VTBkm45KF1cproVLwIiIAIiIAI9CFR4hERLmQ4gv3Nx3XXXuRmW1Cc30iV+O+eZZ54x9rpg+IkjTSYCIiACIiACtUhAoqVMR511Un4Xhh9n8xu8cPnhMwQNtznj50fkMPzElam7alYEREAEiklAdYtAVgQkWrLCVPhMCBD\/ACy\/gRY3\/GAyZmCIw\/AXvheqUQREQAREQAQqh4BES+UcK\/VUBESg1ATUngiIQKQISLRE6nCoMyIgAiIgAiIgApkISLRkIqN4EYguAfVMBERABGqSgERLTR52DVoEREAEREAEKo+AREvlHbPo9lg9EwEREAEREIEiEpBoKSJcVS0CIiACIiACIlA4ArUgWgpHSzWJgAiIgAiIgAiUjYBES9nQq2EREAEREAERqBQC0einREs0joN6IQIiIAIiIAIi0AcBiZY+AClZBERABEQgugTUs9oiINFSW8dboxUBERCBqiDw9ttv26hRo6yuri5pTU1Nxm+3hQdImPi6uqP55s+fH86S9Ifr\/Ou\/\/utkvPdQbvDgwfaLX\/zCR8ktMQGJlhIDV3MiIAK1QEBjLCaBf\/3Xf7UzzjjDECOHDx827K233rKOjg4bN26cIT5oH5cw\/vfff9\/le\/bZZ+2hhx5yZRE0pHkj7Y033nDBJ554oocAcgl6KysBiZay4lfjIiACIiACuRBAiCxatMimTJli99xzT7Lo6aefblu2bHGGn4Sbb77Zdu7caUuXLrUhQ4YQZb\/3e79nDz74oLW3t3crTyJipqGhwS677DKXvm7dOqJlESIg0RKhg6GuiECxCah+Eah0An425NJLL00KkXRjQtzE43GbMGGCnXPOOd2yXHTRRXbWWWfZK6+8kowP57\/uuusM8YKISWaQJxIEJFoicRjUCREQAREQgWwIeKExduzYXrOzXMQsC\/te\/CxLagFmZvwSUVgMIWoQO4iegwcPphZTuIwEJFrKCF9NewJyRUAERCA\/AogO9rbU1R3daMuel1xrY1aF2ZXp06e7GRzEDvtb9u3bl2tVyl9EAhItRYSrqkVABERABApLwM+w+BkXZlGYEWEz7ne\/+91kY2zUPeWUU9weF4RNMiGNJ7w05JeSLr\/8cpezs7PTuXqLBgGJll6Og5JEQAREQATKSKC1tUfjU6dOdZtw+7q7h824zMBs3LjRXn311W71hJeCED0+zObcoUOHuluo\/+RP\/sSV+fDDD52rt2gQkGiJxnFQL0RABERABMIEEgmzlhazRCIc65ZuFi9e7O7u+frXv55MY0mIu4WSEYGHmRdmW77whS8kb4PmGStf+tKXnPDx5VkaCrLbv\/zLv7jbopm1webNm2cHDhwgSZY\/gYKWlGgpKE5VFkUCiUTCmD5ua2uzti6Lx+NR7Kr6JAIi4AmsWHHEF48fcUPvn\/\/8542Ntvwd19Ud2cviZ0YQHqSTndmWl19+2RobG91zXerq6oxNtiz9UJZZFr80xN1EpFHOG\/m837ssF5Gvru5Iu3V1ddbU1KRnunhARXYlWooMWNWXhwBCpTWYWuZkhTGlvGDBAvM2NZhirqurcycz8pWnl2pVBEQgI4F4\/EhSe\/sRN+UdQcLdP8yIhM0LFp8dYYJACedZgSDqyuDroS78XdHOoa5YLOb8vFEuXI\/3Uz\/tkEdWXAISLcXlq9pLTACxgjBBqDxwxx32xYYGe3TKFNt5+eXd7PmZM+2msWPtD4L+tbS0SLwEHPQSgcgQCGZHLR4\/0p22NjPCR0J6r3ECEi01\/gGopuG3ds2srF292u6eMMGeu\/RSJ0wmDRvWY5hnBmIG0fL9iy4yBAzipiUQL8zA9MisCBEoHQG1BIF4nPejFo8f9ctX0wQkWiJw+Lkdb+7cubZmzZpuvbnvvvts9OjRzkgnX7cMCiQJIFhaAtGBEEGsfGnUqGRaXx4vYBAv8Xjczbr0VUbpIiACRSQQWr5xrbSnXyJyaXqrKQISLWU+3AiRq6++2jZs2NCtJwiYVatWufgXX3zRpbUEF2Xn0Vs3Aq3BDAtLQQgWrFtiDgHEC8JlxN69Ei6p3BQWgQIQSATLPGyGX9DYaFMDaw0sEZilWCKesFZrsQW23HCtrc1S8yTDU6eatbWZ\/tUGgQG1McxojnLTpk02adIk17kzzzzTuf7t9ddftxEjRrjfv2CD1+TJk91ueUSOzyPXjBNgSyDmmFnpj2DxLBEuLC0d2rHDtFTkqcgVgf4TSASChb+pP7+h1f7l2Jj9+vQm++v3Ay0SxLcF5vatBG7wsqm2zlpsibVZs+E2WocZCalmwb\/ly82amwOPXrVAQKKljEcZMfLII4\/YXXfd1aMXY8aMsW3bthm31yFU1q9fbwgXyvTIXKMRiUTCmGWZE4u5vSuFwoBwYa9LPB53oqhQ9aoeEahVAolAbCxYsMDefNds+P9eZ2fOXWfjm5fblL\/qsPFz\/s4WBGDaLGYJi9kKazZcC\/0j3GYpwqSlxawjEDPB37\/pX80QkGgp46HmDhcsXRf4\/Yu7777bZsyYYeedd55dddVVtnDhwnRZXZzf+4Lr6+wI\/qCr2R5++GE7tGuX3XjuufbGhx8W1M4cOtT+NDgZsnTnGW7dujU4R3bICvi5EtPCf56iyPS\/\/uu\/7Oe\/TNhJf\/Bd+7DzsL37eoe9tKHD3tn0M3t\/9x4bOixmV9ebnV2\/1v6yfp7V13f0sCfqx1tHfb11BH+XHddfbx3z5pXkb9GdYIO3adOmuf2FgVevMhKQaCkG\/ALUySbcRYsW2VNPPWWbN2+2n\/zkJ3bDDTdkrJk83rjIkhHxUq1WV1dnPBXzSyNH2lnHHVcU+78XXWT79+93T96E48igLVxZoxWKgZgWjqU\/JpFiWheML7Bf\/eRXNrghZp+MXWHjGhrt3MDefWia\/fLuz1riscX2\/o5E8LeGbQncxrQ2dv\/71hgsmTeuW2eNS5cW7DPouWVyOZdia9eudedi\/LLyEZBoKR\/7jC375aA5c+bY8OHDXb758+fbM888Y+yDcRE1\/pYIppvPGjy4x7LQCbfeah9fvbqHDbnmmrTEXP5Vq6zhiivSps8JvtW1t7enTVOkCIhAiEAi8Ld1Ges9UwN\/4xFLtJFoNjQIHhsYr5NPb7Jjhsac1Z\/RRJTFzKzZ2qzJ4hb+x8LREmsxa2oyi8Usn3+JRMISgeVTVmXKQyBdqxIt6agoLvIEWltbjb0nmTq6L\/hW9O7s2Ybt37TJBgVTuwgUn78+WFL62N\/9ndWPH++j0ro846WtrU0nu7R0FFmzBBLByNsC8+KkLvA3BkYYIy2kO6bYFNv1Vtw+CrJg7wbuwN9bYp9Y1GEXL+mwEX8w32Jm1mFTbbktsHU21Vqs1RAwuB1G5WbW1ha8ZfdCoHCemDp1qtXVHXn6NbMpdXVH\/aSTL7salSsKBCRaonAUUvrAZls23XLL8\/bt210qj4\/2074uQm+GoMgGwwdde18GjhljgyZPdjb0pptswAkn2OEPPui1ii91Pe8lHo\/3mk+JIlC1BBAofPz5weWpwSjrAkNDpBEnQcrRFyqEvbOBNS8P3oKU\/4wvsE2BS5UHPh5zkyYDP0pY4p8WWFMsFqQcfTGzgoDBPRob+NragrfMr3g87u78Q6DwKITTEwn3sEmejO2Nh0liLS0tyWUmymWuNZwifzkJSLSUk34vbS9cuNAmTJhgF198sdv8xY+DLVu2zBA0vRSrmSROMCwPZRrw\/j17bOezz9qeLVvs\/WANfH9Hh9XV19sxp5\/uiiBW9tx7rx16l+98LirjW2\/tZCykBBGoVAKJoOPxwBAlUwMXgYLbEviJD5xuL7QGhi5ZHqSsC+xwYB2BEcaCtHXB3+GHr7XZrlWNdmBTq40YFLfX\/63V2v+\/RovFYrY8sKDEkRf+lhazw0FF3NJ8JPbIe3v65dpEIuHuJmRmBaGCQOFBk9wJyJcPvuR44\/EIGD\/v8fzMmcazmSjXGszgHmlE71ElINESgSPDvhX2TXDHULg7d955p9v4xQbblStXSrCE4QT+3paHPtyxw\/YEJ7GdGzfaO+3t9tF775kNHGiHGxps3\/r19tuvfc25QTVZvbYE4ierjMrkCHAByca4rT+bfNWSx8Ep0lu\/qk0EpdsCQ6h4kUI4VaTEgjxYIEIMMbIuCCNOMMLENwVxaV5NTU3ubp+v\/O8m++3PW+yZpVMD8dJmLS0t1hEIGovHj5SKxcwIL1lyJNzcbEFBs6amI+G2tiNu6J3PB6KDmRXECkIFgRLKktHLeYQyWEvQF2ZoMmZWQtkJDCh7D9QBEciTwJudnRlLDvrYx2zg4ME2JBaz44YNswHH+u1\/GYsoocAEOPn3ZdxG2leeakqPx+MFptzP6uiOFyq4bSn1xYJwoBlyFShBqbQvN6MSzJwcDmZQMO50XII4SSSO5A9EgxMosdiRsH+PxY4IGdKJa2vj3ZkXLDwQEuGRrVhxhUNvlPOzLnzmQknyRoiAREuEDoa6kj0BTn695T72pJNs5KWX2ikXXWSnTZlihO3AAavrRehkqu+NvXttVNfelkx5FJ+ewCc\/+Un79Kc\/ndE+9alPZUz7dC\/lKjEtPaEyxXLNZ8kHwx\/uRqAPrMUCkRBYFjMoQa7+v1asMGtpMUPA9FYb6czCBLOnZEOw8NA6BAtLQcyaEJ+vUV5PxM6XXmnKSbSUhrNaKQKBp4MloGyq5U6hY0aMsMP799vBt9\/Opkgyj5\/NicViyTh5sidw3HHH2UmBgMxkQ4cO7TU9U7lKjLco\/GNCA6HCrAqzLL5PfLwDzWCIFIyVmSafWAIXMYJl01RTU1LccIPC5mAJmBmWbIpmkwfhQn3xeNztkcmmjPKUjoBES+lYq6UCEmhqarJsRcugz37WBpx8sh14\/fXkPpZsu8IsC4KF9rIto3wiEEkC3P3DfpV4V+8QKs2B3+9LQagQF0SV\/BWL5dZkLOYeQ9DS0uKe1YTQyK2C3nNTHxt1qZ\/ZnN5zK7WUBCRaSklbbRWMwJQpUwxB4WdCUivmuSz+IXP4eW7L7m9\/OzVbn+HbX3nFEC19ZlQGEYgyAQRLS6iDTYEfscLmWfxBsNJera2tdtbgwcadQcXoO6KF+otRt+rMn4BES\/7silBSVWZLoLm52YkJREW4DMKEB8qlGrc3h\/N5\/3vf+Ia9O2eOdf7oRz4q6SKImM1xGwWTsfKIQIURCAuWWNB3xAuCBX8QrMQXsx9tbW1uliXcfx4g6b+shN0hGZ6I3XDFFfbxVavMPWjy3HPDVTl\/sQSRq1xveRGQaMkLmwpFgQBiYlUiYYiLYvQHQcQsS1NTUzGqV50iUHwCiaAJRErgGCKFmRWWgayy\/yFamAXJJCqYWfVfXNI9EduPvn7iRONRCD6c6jLbkhqncHkJDMimeeURgSgSYLYFQbFo48aCd+\/BLVsMQbR8OWf5glevChZ8nR4AABAASURBVItE4MYbbzQeYuYt9fhdcskl9thjjyXzLF26tEg9iUC1CBY23fquIFaqRH+3t7f3+jMefsi4qU\/EJu5AZ6cN+KM\/smNOO42gHT50yLnp3hBH6eIVVx4CEi3l4a5WC0SAixJLONc++2yBajQ3c4MQ8qKoYBWroqIS4NbXGTNmGN\/CedDYc88955YQvTC54IIL7M\/+7M+Mn8YgnXzjx483yhW1Y+WqnGUhhAvtNwdvWOBUwysej7v9LNmMZf9LL9nBbdvcE7HrTjnF3nvlFfvdhx\/akM9\/3g7t22eHP\/rI6gZkvhQO6yUtm\/ZDeeQtAIHMR6oAlasKESg2AZZv+FbNrEghhAtLTRc8\/rhRL4Ko2P1X\/YUjcEpwQaqvr7ddu3a5ShcvXux+gwaXCJ44TZ5XgosWYcQKMy9VeZzbghFigWN+Wciq6x93+OQ0ooEDbV8gUBAtJ8+dawOHDbN9\/\/EfxvObeqtnuERLb3hKnibRUnLkarDQBJqamtx0P8LlsmDaOJ\/6ESvsYUGwUB9P6synHpUpP4ELL7zQ\/OxKuDenBUsB+\/fvt507d4ajq8+\/LRgSz2EJHMtGsFjl\/WOWLJ9eH\/uxj9lJM2faxy67zA5t326H\/ud\/zOrqTP8qh4BES+UcK\/W0FwJeaGwbPNhOeeghQ4D0kj2ZhFhh\/wrLQT8O1rlbWlqcAEpmkKdiCKxZsyYpSBAuTz75ZLeln5NPPtkQLTODixazcxh7YCpmgNl29OZQxubA3xRYlb1isZhbxs1pWAcO2DHBTMtZ3\/2uHXPSSfbRhg1ZFd\/ey36XrCpQpoISkGgpKE5VVk4CnMiYIUF4IFoQLxc+8YQTMOx7CRtCheUkZmYQLJ+YNcuJFe5IKucY1HaSQM6e559\/3mbPnu32tFCYpaJ58+ZZWJg0NDTYG2+84ZaN+LbOHhiWichfFcaS0DNdI4kFLptvA6faXvytZzum8BOxuVOIp2PjHn\/55Tbk2mut7vjj3cMnh950kw2aPDnbapWvTAQkWsoEXs0WjwDCgx9jQ7xcdeONTrQgTsJ2544d9nYsZqTzjZt9DbEgXLxeqeZSEUCEXH\/99clZl4kTJxqbcNnrwkzLSy+95LrC3haEzblpns\/hMlTaG5tuw8tCPIul0saQZX95uCTLwdlkDz8Ru\/Mf\/9F++7Wvmb8des\/3v2+HP\/jADu3aZe\/ffrvtW7++W5XMxP7i\/fe7xSlQXgISLeXlr9aLSADxgiFgMGZhwoZYIb2pqSn7XihnRRBg1uX++++3zmDJz3f4nXfeMUQKm3F9XNW4iWAkXrAEXms2s5hV7T\/u7GNwzJ7iphpPwfYPl8PPc1t48GRqvr7Cmervq5zSi0dAoqV4bFVzxAgwk+ItYl1TdwpAgNkyhKhfDhoxYoQTKXv27DFEzAsvvOBEzNixY11ruOGZFxdZqW8rgo7HA+P1\/wRvVbosFIws+WpqajKWeZMRgQdh4mdRwm6mJ2Izs7IrWEJk9oVbo4Mqur2ync3pVkiBohKQaCkq3pJVroZEoOYJsCzEPhW\/0Zb9LNu2bUtuxmVj7ve+9z0bPny427+EqFm1apUhdioaHvtYWrpGwOzKtV3+Knfmz5\/vHgBZrNkQ6sWqHGPFDU+ipeIOmTosAiKQiQDChQfHeSMczotwmTVrltuIWxXPaGFZiIfI+UHyAOcRPlDdLktEzLaw6b7QI2UvCxv1aYPZ2ULXr\/ryJ1Bc0ZJ\/v1RSBERABESgLwIIFoQL+ZqDt6bAaujFLBmzIQiMQg4bITRg2DBjz1sh61Vd\/Scg0dJ\/hqpBBERABEpPgD0sLA3RMstCzLLgryFjFoR9TOw9QWj0d+jMsFDPz4OKEETUH3jL\/lIHjhKQaDnKQj4REAERqAwCzK6E7xaqQcHiD1RTU5Pbl4TY6M+MC4IF8cNDJlkWol7fhtzoEJBoic6xUE9EQAREIDsCzLIgXMjdHLw1BVbyV3QaRGT4GRf\/QMlse4dYQfDwEx4IFurRslC29EqfT6Kl9MzVogjUDIFf\/vKXFo\/HM9ozzzyTMS3eS7lKTCvYQUes+FkWloVq4PbmbNg1NTUZz2HigZGIEC9e2POSWh6hQjz5eOgkYqWlpcWV15JQKq1ohSVaInA8eI7E3Llzjd9OCXeH8OjRow3jCZDbt28PJ8svApElwImfb6x92Q9\/+EPrK0+u6VHOD5d+H7R4qIamwI9wCRy9zODLLIkXL4gRRIn\/SQ+EDH5mVYgnHZHDZ4ZyYhh9AhItZT5GCJarr77aNqT8eNemTZts8eLFtmzZMtu8ebPNmTPHrrvuOiN\/mbus5kUgKwJ88+3LeMR+X3mqKZ2LalbwesvEg+RIR6xolgUSPQzOiBDEC4Yoab37bsPYYEvYPyWbfOTvUYkiIklgQCR7VSOdQphMmjTJjfbMM890rn97+umnjR9zmz59uotauHChrVy50oYMGeLCeqs0AuqvCBSAALMsGFUhWjD8sowEECQIX\/a9eCOcsYASIk1AoqWMhwcB8sgjj9hdd93VrRfMpqxfv97GjBnTLV4BERCBGifAc1k8As2yeBJya4iAREsZD3ZjY6NhmbowdOhQYy9LNntayOPN18m0aF+m9A63+S4bDlu3bs06bzb1KU+HiWn2n7+OnwV5nw6sPrBYYKMC6+hpYtqTSX\/+1vz5edq0aW5\/oQ\/LLQ8BiZbycM+qVfazrF692u1pmTBhQq97Wtj34o0\/UBpAvMiOCMNCcBg5cqQTmYWoS3UcOS5ieoRDn5+HuiDfVwLb32XLA7frS09qWTHNzCaVVTZhzqXY2rVr3bkYv6x8BCIqWsoHJEots\/mWH3ejT\/w4mP8GRVgmAiJQAwS4vbktGOfUwPAHjjWbGXcNmf6JQO0RkGiJ4DFnr8sZZ5zRo2cnnniinXrqqT3iyxGRSCSstbW1puxu7j7o55jb2trKcbjUZqURSAQdZv8KYmVB4CccOBYzsxp++m0wer1yIVCFeSVaInpQP\/e5z9mqVavMP5tlxYoVds4555ifeYlCt1taWuy73\/1uzRjLdf0ZL7w4jlE4dupDhAl4sdIS9DEsVgh3BHF6iUANE5BoiejB51bnW265xS6++GK3+eutt94yLnpR6+4nP\/lJ10f6We12\/vnn92usp512WtQOn\/oTJQIIlAVBhxAn+AOvMbNCeJ2ZVc\/dQsFg9BKB\/AhItOTHraClmD1pb283hEq4YsJ+c62e0RImI78IVBkBRApLQeHVQ\/auMLOCWEG8VNmQ8x0OS9PxeNxYasXwJxKJfKtTuQojINFSYQdM3RUBESgSgXJWywxL6nU3Xs4ORattRElra6tNnTrV3cGHu2DBAsOmdsVxJxBhhEy0eq\/eFJKAREshaaouERABEciVAOIESy2HiMFS43sJM+vwz\/\/8z8lZCC7gpTTERS\/dyzmJ+hArCJIH7rjDTk8k7NEpU+z5mTNt5+WXO8OPfbGhwX792GNOyJAfFjk3qAKRJyDREvlDpA7WOAENv9oJsPHWjzF1GSg17PNlcFlm5rd0rrnmGiu1MctRSKEQFis3jR1rz116qX3\/oots0rBhdmYgUKzrH36MPF7QIGCYgcG6ssmpEgISLVVyIGt1GDfeeKPx42fe+DG0MIulS5d2S3\/yySfdN7FwHvlFoGwEEkHL6WZZgmjLUbBY179Bgwb1a8N4vhvqjzvuuK4e9N9BsLS0tBhCBLGCm22tYQGDiGLWJduyyhd9AhIt0T9G0exhBHrFN7sZM2YYU8h8o3ruueeMH0dDqPjunXzyydbZ2Wm33XabWw+\/5JJLLFXY+LxyRaDkBBAt4UbDYTbihtNqxI9gYSkIoYLlO2xmZFg2GrF3r9sHk289KhctAhIt0Toe6k0OBE455RSrr6+3Xbt2uVKLFy92wgSXCAQKd2YhWnbs2EGUTASiRSAsUsI9Y5aFu4bCcTXgZ2aEGZYvjRrlZln6O2RmXe6eMCH5xaa\/9al8+QlUm2gpP1H1oOQELrzwQgvPrvgOjBgxwokaxM3f\/M3fuGUilpN8ulwRKDuB9jQ9QLDU4FNvmTFl9nROLFYQweLJIlzY64Igwny83MokINFSmcdNvQ4IrFmzxnbu3Bn4zBAuqftVECvMxLBsxPIRJ0WWkzgxukJ6E4EoEWgKOsOSEA+Swx8Ea+mFoDgUzIiy2bbQ42apCDHEeaDQdVdnfdEdlURLdI+NetYHgeeff95mz57tpn7JikCZN2+e+dmUO+64o9ty0SuvvOJmXs4991yyy0Sg\/ASmhLowP\/AzwxIL3Bp78YWCLxMsCxVr6F4McQt4sdpQvcUnINFSfMZqocgEONldf\/31yVmXiRMn2gUXXNCjVWZl9u\/f3yNeESJQNgLMrPjG0y0V+bQqdxEtZw0e3GNZ6IRbb7WPr16dNO8fcs013YgQ9mm4lOuWoSvAbAu3hXcF5VQgAYmWCjxo6nJPAsy63H\/\/\/e5OoZ6pR2L8ctGRkN5FICIE\/FJQW9CfRGA1+OKOIfaeZBr6vrVr7d1gVhXbv2mTDZo2zbwwGTR5sh37B39gh3btst2trXZw61ar\/9SnrOGKK3pUxzKRZlp6YKmoCImWijpc6myYALcu83wWvxzkN97u2bPHEDFszg2njx071omaf\/\/3fw9XI78I9JNAAYvX4NKQp4eg8P7e3A8eftgJlIFjxhiC5ZjTT7e6+no7uG2b7X\/pJTvw2mtmAwfawHPO6VGNX36Kx+M90hRRGQQGVEY31UsR6EmAZSGmlWfOnOnuDGI\/y7bgxEU8ubn1OZyOqHnooYeMDbuky0QgUgRqWLAgIlgeyuZ4IEwQKAgVBMvBt9+2w8Gy7zEjRlj9uefawLPPNjtwwA68+mra6s4KlqE4L6RNVGTkCUi0RP4QqYO9EUCgcEeAN8Lh\/IR9Gs9tYXYmnF7Nfo2tQgj4JaEaFi0cqd6Wh0jvYcFsyuGGBvvdP\/2TvX\/77S75hCVL7JjTTrMPHnnEOn\/0IxeX7m3Lli3pohVXAQQkWirgIKmLIiACVUzAi5YqHmKxhtYZzLLYuHE29Kab7HBnp9v3sv+\/\/9uOv\/xyY3NusdpVveUjINFSPvY12rKGLQIikCQQFiw1PtPyZiA6klyy8QRLQHV799rHZs2yuuOPP7KXJSjnloWCNLdMFIRTX28EZUaNGpUarXCFEJBoqZADpW6KgAhUOYEaFi2xWMye3pHdT22wb4X9K+xjGVRXZ8fk8EONXhjRnulfRRKQaOk6bHJEQAREQATKQ6CpqSnrhgd99rM24OST7cDrr9u+9evdnURsvK3\/9KePbMTlrqGBA5MzL+GKmWWJBQIpl\/bC5eUvPwGJlvIfg4ruwTvvvGO1Yjycrj9j\/fDDDyv6WKvzIlAsAlOmTLFViYT5mZDUdnguCw+Nw\/Dz3Jbd3\/62y8aGWzbeDjjhBGMjbv348Ub6nnvvdenht9tfecX9Enw4Tv6CEih6ZRItRUeCXHaGAAAQAElEQVRcnQ3wbQU7Lpia5WJcC7Zv3z7rzzhPOumk6vwwaFQi0E8Czc3NxvkEURGuCmHCA+VSLVWQIFzenTPHbcQlb2o6dSKIWIJasmQJQVmFEpBoqdADF4Vud3R0WC3Z2rVr+z1eHnYXhWOnPohA1AggJnqbbelXf4PCCCKEUVNTUxDSq1IJSLRU6pFTv0VABCqfQCw0hETIX4NeP9uyaOPGgo\/+wS1b3PKTntNUcLQlr1CipeTI1aAIiEC1E+CJq\/yeDg82bGxstLq6Omf4iSMtHo9XO4acx4eoYAmHWZGcC2cowLIQQghRpFmWDJAqKFqiJQIHi9\/KmTt3rq1ZsyZtbzZt2mSTJ0823LQZFCkCIhAJAvwYH6IEcfLAHXfY6YmEfbGhwe6eMMEZ\/onbt9tT99xjPl\/CEub+dTnOX6NviAqWUBEtWH8xIFgua293+2UQRP2tT+XLT0CipczHAMFy9dVX24YNG9L2hPTbbrvNfve736VNV6QIFJyAKsyZQCKRsNbWVuNnIxAqj06ZYs9deql9\/6KL7KaxY40f6sPwY6Q\/P3OmEzTm\/yW8p7ZdhAsCA9GC4MiXBuUvePxxGzBsmNuLlm89KhctAgOi1Z3a6g0zJ5MmTXKDPvPMM52b+oaY2bp1q5144ompSQqLgAhEgACChVkTZlYQIwiVbH6xmN\/aQcCMGLb3yCgC0YLwORKo7XeWcphx2TZ4sF34xBOGAMmWiJ9d+XFnp7W0tEiwZAuuQvJJtJTxQA0ZMsQeeeQRu+uuu9L2Yvv27fbAAw\/Yrbfemja9xiI1XBGIHIFEImEsBY3Yu9fNrGQjVlIHcWjw0SfBcpGVcDlCiBkXhMtVN97oRMspDz1kzLwgYNj3gjjxxkbba5991gkcZlcQO8zWcEfSkdr0Xi0EJFrKeCQ52WGZuvDwww+7vSynnnpqpizJ+NGjR5s3X2ct3Y5cirEy41WKdmqpjUpm+rOf\/czOPvts+9NYzL7\/+79vb3z4YV6WsHeso77D2QMXfN7+8i\/\/0n70ox+5GYJcPwu7d++2AQMG2P79+0tuAwcOtF27duXV70zjPHz4sM2bN8\/+4z\/+w66\/\/noLgNvf\/vrXdvl\/\/qdNfOqppH3jl7+0nwdnw8u\/9jV78MEHjccT8PtCmerNJT6o1r2mTZvmzrEuoLeyEZBo6S\/6IpVn6ej555+3K6+8MqsWNm\/ebN74g6QQ4kXW6L4JF4LDyJEjC1ZXIfpTDXVUMtNbbrnFTj\/2WPu\/F11kZx13XP7WcIw17g8+p4H96ekj7Ruf+IR96UtfSt5tlMtxPuGEE+zQoUNWX19fcjtw4ICdfPLJRfkb+cxnPmNLly41hOJHH31kr732mrMnn3zSucRx3iPPFVdcUdA+cC7FEEKcY\/HLykdgQPmaVsu9EVixYoXNnj3bWELqLZ\/SREAESk+gra3NNm\/c6Dba9rf1g4O3J6s4Zu9wt3GXZSYtEyWx9PDEgtktjCUk3B4ZFFFxBLLtsERLtqRKmI+9LBuDEyJ3FbHkc9lllxnT6LiZbosuYffUlAjUPAG+VCAssGLAYIMuwigejxejetUpAhVLQKIlgodu+PDh1t7enlzuefTRR41pdNzp06dHsMfqkgjUDgGEBMYtzOFRn3DrrcYP+qXakGuuCWczwuE8Q\/78693SCSCGzho82BBHhGXlIqB2o0ZAoiVqR0T9EQERiDQBvlAgKBAW6Tq6b+3a5A\/37d+0yQZNm2YIGp93wLBhZgcO2AcPPeTyfXDHap\/UzfWzLd0iFRCBGicg0RKBD4CfWck0izJ+\/Hhbv3694Uagu+qCCNQ0gXSzLJmAfPDww3Zo1y4bOGaMDZo82erPPdeOGTHCDu\/fbwfffjtTsW7x3FbdLcLMFBaBWiUg0VKrR17jFgERyIsAouWsYOkmm8L7X3rJDm7bZnX19XbM6afbgJNPtrrjj3c25Npr3XLSoE8cXfLdP+zlZLV++Yn2kpHyiECNE5BoqfEPgIYvAoUjUP01+VkPnmab02gHDnSCBeGCgDm4datbGmL5aMDvPuaqOjT46EPmXETwlq04CrLqJQI1QUCipSYOswYpAiJQbgIfvfee7V6+3N6dM8fe+8Y3XHcOvPqqWdcNQgcbjt76bKF\/W7ZsCYXkFYHaJiDRUtvHvyZGr0GKQKEI+GeC8Pj4bOs8fPCgHf7oI+t87TUb2NDQrdihl\/cmwweGvZL0e88be\/caT3b1YbkiUOsEJFpq\/ROg8YuACOREwAuXbAq5jbcjR9qhffvswJtv9ihSv2NcMi68n4VIL4xyaY9yMhGoZgISLWU7umpYBESgEgk0NTUZP9iXTd8Hffazdswpp1jnM8\/Yh\/F48hkt\/hbo+v\/6VLKa8JNxiWSWBZf2cGUiIAJmEi36FIiACIhADgSmTJliqxKJjCV4Lot\/eBz+3\/7jP9pr06fbccOG2Z577zU239aPH28fv2O11T072NVzaPAOO9TQfSMuv1xcrFmWG2+80fgFZW\/8IrLrSNcbYZ\/22GOP2SWXXNKVIkcEykugh2gpb3fUugiIgAhEm0Bzc7PrIKLCebredn\/72+6OoHdnz066v5k1yzZfeaUNHDzYiRay+ny7m1sIOts3Ku5c\/8bSEMLIt+XjC+EuWLDAZsyYYdwJNXXqVHvuuecMccSPDVI\/goYw8bfddhtR9md\/9md2wQUXOL\/eRKCcBCRayklfbYuACFQkgaampl5nW\/yg3nul5+ZanzZoS5P3Wup+FpafEA5LlixJ5imU55RguYpfgd61a5ercvHixYZ4wSXijjvuSIZ37NhhnZ2d1tDQYMOCmSLTv2wJKF+RCEi0FAmsqhUBEaheAiyfICxSZ1uyHfGAzmE2KHFEtBwavKObaGGWZdHGjVaMWZZw\/y688ELzsyvh+LAfoYJg4Udcn3zyyXCS\/CJQFgISLWXBrkZFQAQqmUAsFrOWlhZDXCBeMo1lyKhRbmnolIsu6pblmL3Dk+Hw0pAXLNRfjFkWC1pds2aN7dy5M\/CZIVwQIywZuYjQG3E33HCDIVjwh5LkFYGyEZBoKRt6NSwCIlDJBBAVTU1Ndu2zz1om4cLm25GXXprcz+LHe\/wrs703OcuCYLk9WE7aNniw2ySbzFBgz\/PPP2+zZ892e1qomqWiefPmGXtZCHtjNslvwM0kbHxeuSJQKgISLaUirXZEQAQyEajYeO6wGT1hgl3W3p5RuKQOjqUh\/3wWvzSEYKGOnweZqZOZlsBb1BezJ9dff31y1mXixIlpN9u+EggphM25555b1P6ochHIhoBESzaUlEcEREAEMhBAZLD\/BNHBrEuGbMno8NLQr4c9ZsyuXPD44zZg2DA3w1IKweI7w6zL\/fff7zbb+rhM7jvvvJMpSfEiUDICEi0lQ62GKo6AOiwCWRJgKQXxwkzJKQ895IRIpk264aWhhYkV9uPOTmN\/TEdHh7v1OMsm887m++qXg0aMGGHMpOzZs8cQMWzOZSykc5szMzDcQfTCCy\/k3aYKikChCEi0FIqk6hEBEahpAuxvQXggQJ4ZPtxt0kXAXPjEE4YxE\/P\/tm8xvzSUiCWsqaXJza6wP6ZU8FgWSiQSNnPmTNc2+1m2bdtmxNMHbn326X\/zN3\/jbnf+3ve+Z+xrIV0mAuUkINFSTvr5ta1SIiACESaAAGGmAgGDe9WNNxr2iVmzbPqsP0\/2PNYcM\/LGYrFkXKk8CBSezeKNcLhtwj5tVtBvCZYwHfnLSUCipZz01bYIiEDVEkCMNDU1OWGCOGFZpjnRfHS8U4565RMBEciOQOFES3btKZcIiIAI1CaBRDDseGC8mFxpwiMTARHIhYBESy60lFcEREAE8iWAaPFlQxMuPkquCEBA1jsBiZbe+ShVBERABApDoDVUjZaGQjDkFYHsCUi0ZM9KOUVABEQgPwLMsvilIZaFsPxqKlMpNSsC0SAg0RKN46BeiIAIVDMBL1gY43zeZCIgAvkQkGjJh1qBy\/BQp7lz5xo\/ZOar5kfKpkyZYqNHj3ZGOvl8ulwREAGzimDALMuKrp6yAVf7WbpgyBGB3AlItOTOrKAlECJXX321bdiwIVkvcdddd53NmTPHNm\/ebC+++KJLa2lpca7eREAEKohAPOgrFjimZSHTPxHoD4EB\/Smssv0jsGnTJps0aZKr5Mwzz3Qub0OGDLGVK1fawoULCRrhyZMn21tvvWUIGheptwgTyK9rPIWUh3rVkt18883uSazlHnNbW1t+B62vUsyy+A24zLIs6auA0kVABHojINHSG50ipyFGHnnkEbvrrruK3JKqrxQCXDz5TNSK\/fSnP7Vyj\/XBBx+0FSv8+k2BPylUi3ChWmZZEC74ZSIgAnkRkGjJC1thCjU2NhrWV23sb1m1apUx24LQSZff733B9XXyGPGwyd9h\/WGwdevWfpXvq23q54frzjrrLBs3blxN2JgxY8o+ztNOO832799f+GP7s+Dz9peB1QcWC+zWwDqKa7t377YBAwa48TCmUtrAgQNt165dhedYZGZ9\/V368+20adPc\/kIfllseAhIt5eGedassB1133XXGL7FeeeWVGcux98Ubf4RkRLzIGp0wLASHkSNHFqyudP2hfi4yXHQQL7VgXOjKPU7+VuhDumOSd1xd8Ln7SmD7u2xJ4HZ9Scm7zizKn3DCCXbo0CH3q82MqZR24MABO\/nkk4v6N1JMdpnq5vOBrV271u0xxC8rH4EB5Wvatyw3EwEEC5t0SV+2bJnb24JfJgIiEGECLActCPqHGzjG3UKY6Z8IiEB\/CUi09Jdgkcp7wXLGGWe4TbmZloWK1LyqFQERyJcA+1j83ULsYVmeb0UqJwI5EKiRrBItET3QLS0trmfedQG9iYAIRJtAa9C9I3+6ZgiWdaZ\/IiACBSQg0VJAmIWqio23GzdudM9uOe+889zmLzbYTpkyxUgrVDuqRwTyIXDjjTfaunXrkrZ8efqpBJ8vU3o+bUe6DHdNhwULWBAuke500TunBkSgoAQkWgqKM7\/Khg8fbu3t7TZ9+nRXgQ\/7jbXeJQ9pLpPeRKAMBHieyowZMyyRSNjUqVPtueees1gsZkuXLu3WmwsuuMAmTpzYLa6qA4lgdOxjCRyLmdmSwLjFOXD0EgERKBwBiZbCsVRNIlD1BE455RR3Zwq3tjLYxYsXO\/GCS9gbApy8Phw5t5AdQrBMDVXIplssFCWvCIhAYQhItBSGo2oRgZoicOGFF\/aYXfEALrnkEmtqarLOzk73vBAfX7VuPBgZwiVwrNmOzLIEjl4iIAKFJyDRUnimqlEE8iUQ+XJr1qyxnTt3un4iXJ588kn3GH4X0fX2x3\/8x84Xj8edW9VviJXwshD7WKp6wBqcCJSXgERLefmrdRGoKALPP\/+8zZ492+1poeM8vGzevHnGplvCzLJ88pOftF\/96ldJcUN81RqixQ+OWRbvlysCIlAUAhItRcFaZZVqOCKQ6J0yXQAAEABJREFUQoANuddff31SmLDpls23c+bMcTn\/\/d\/\/3blV\/9YeGuGUkF9eERCBohCQaCkKVlUqAtVPgFmX+++\/3+1dYbQXXXSRcXdbQ0ODffOb3zRmYJiJ4e6i1atXG6KGfFVlidBomkJ+eUVABIpCoJJFS1GAqFIREIHMBHjmCs9o8ctB\/CYWwoQnOCNgZs2a5e4m4nboH\/zgB24jLrdHs6SEyMlcs1JEQAREoG8CEi19M1IOERCBLgIsCyFCZs6c6R4ux2zKtm3bemzG7cpeW0541qW2Rq7RVjSByuq8REtlHS\/1VgTKTgDhwkyKN8LpOsWsDBtzM6WnK1NxcVNCPeY3h0JBeUVABApPQKKl8ExVowiIQK0QCN8xxCP8NdtSsCOvikQgHQGJlnRUFCcCIiAC2RJArPi8\/pktPixXBESgoAQkWgqKU5WJgAhUN4E0owv\/zhDP02sM8rQGppcIiEDBCUi05IB0+\/btNmXKFBs9enRORpkcmlFWERCBSiPAk3D9Lc8sETH7UhcMIvybREFQLxEQgf4RkGjJg9+yZcvM\/\/JyXy5582hCRWqYwDvvvOOeOMtdOtlapebjzqNy9\/29997r\/6ctFlSBcAnvcQmijJkXLRmZ\/olAoQhItBSKpOoRgX4S4CFs2HHHHdfPmlQ8FwInnXRSLtkz5\/XCpSPIEhYvzLwEUXqJgAj0n4BESw4Medpne3u7TZ8+3ZXatGmTnX\/++WmXilgSYjmJvJRxBWruTQPOlUBHR4fVkq1duzYS4+WBebkeq4z5ES\/hxPnhgPwiIAL9ISDRkic9ngB622232YwZM9IuFSFUEDl5Vq9iIiAClUqAmZW2rs4jYMKzLl3RckRABPIjUJOiJT9U3Ut1dnYa6\/Gf+9znuicoJAIiULsEECzhPSzcWVS7NDRyESg4AYmWPJHyo3D87kqexVVMBESgGgnwVFw23zI27iYqwyzLvn37jM3cpTaGLKspAmUZrERLntiHDBliV111lX3nO98x9q7kWY2KiYAIVAsBloS41ZnxsCzE3UT4S2yIll\/96ldWavvwww9LPFI1V4sEJFr6cdRPPfVU43bJiy++uMdmXL8Rtx\/Vq6gIiEClEAgvC3nBglvi\/i9ZssReffVVO3z4cFmsubm5xCNO05yiqpqAREueh9dvxL3mmmu0ETdPhiomAlVBAMHCU3D9YLhuszTkw3JFQAQKRkCiJU+UfiPumDFj8qxBxURABCqeAIIl\/NTblmBE6TffBgl6iYAI9JeAREueBAu5EZdZm7lz59qaNWu69ea+++5LLjvh75aogAiIQHkJIFi4UwiXnjDDIsECCZkIFI2AREueaNmI+81vftO+973v9WsjLoLl6quvtg0bNnTrCQ+uW7lypT366KPO8BPXLZMCItBfAiqfP4HwnULsXynTxtv8B6CSIlB5BAZUXpej0WPuGFq0aJG9\/PLLlu9GXETIpEmT3IDOPPNM5\/q3p59+2kaOHGmNjY02fvx4mzhxohHn0+WKgAiUkQC3NbMURBcQLOvwyERABIpNQKIlT8I87Zan3mb6wUTSyNNb9czWPPLII3bXXXf1yPb666\/bGWecYeTxicR5f5W7Gp4IRJcAy0EsC\/keMsOCcPFhuSIgAkUjINGSA1pmV7iVOXXvSW9VkJcy6fIwi4KlSyMuvMk37Cct1UaPHp3c\/+LrrIXfsPnZz37mZqMYc7Ft2rRpPdr6zGc+E4nfzqnUY71169bK4vezDuu4MrBtgdUH9sXARgUWod+MqjimEWKX7u\/In2v5++c868Nyy0NAoiUX7hHOG57x4Q+Prhb7Ih6F+llCSyQSVo4HW\/GMHn7KIQocKrUPHL+S9b2u0Rq3hCxYem3MxSj\/06D804HtD2xEYD8KLJc6SpC3pExLMJ6cjlER+sO5FOPHPTnP4peVj4BESx7s2TiL4s7GyJtHE65IeDko7HeJeutG4LTTTrNYLFZU42cbYqE2TjrppG59UCCCBBJBn1oD47ZknqWC660uiCeO9MDb64t62HirfSy9YlKiCGRDoD95JFpyoMceFfaqoLZzMcrk0IzLmm45KF2cy6w3ERCB7gQQGYgRRAlCg42z3XMcCZGPdPKR\/0js0XfSeTw\/Qod8pMSCNzbe4gZevURABEpHQKKldKxzaom7ip555hnjDiMMP3E5VaLMIlCLBBAaYZEBAwQGz1Fh0yyGAAk\/tZYyxDH7Qlk22uIiZvCT7uuRYIFEFZqGVAkEJFoiepS4zZkHzl122WWG4Scuot1Vt0Sg\/AQQFsyWIDTw0yPECmKkIwggVhAuGA+BQ3wQT3qQnHwxK8PsCq6PpB7KkR+\/j5crAiJQUgISLSXFnb4xv+w0ffr0bhkWLlyY\/F0j\/N0SFRABEThKAJHBzEhYgCAyECYIlKM5u\/sQIKQjRigbnn0hDfP1IHq6ly5JSI2IgAgcJSDRcpSFfCIgApVGgBkVZlfSLeEgMhAd2YyJfIgXRM7hoACGkMFyqScoqpcIiEDxCEi0FI+tahaBKiZQ5qF5scJSEDMkdAfhgR+hEZ4xIU0mAiJQFQQkWnI4jDxc7oYbbjB+LyiHYsoqAiJQSALsNUldCkKwMEvCbEkh21JdIiACkSIg0ZLj4di4caOdd955PX6ROcdqlL1IBFRtFRNgdoVlIAQLfoaKWPGzK\/iJk4mACFQtAYmWHA6t3zB70003GQ+N4\/H8zL7kUIWyioAI5EoAgcK+FZaC2HDry\/sNsppd8UTkikDVE5BoyeMQcyfPiy++aDwhlV94vu+++\/qoRckiIAJ5EUCkMLPCbIqvgBkVloK0QdYTkSsCNUNAoiXPQz1kyBBbuXKlLVu2zG6\/\/fbkjxX6R\/trFiZPsComAhBgdgWxwnIQfuIQK4iXjiCgjbYBBL1EoMYIBMOVaAkg5PNiWQhhwjIRy0Wpj\/Xn0f0sJ+VTt8qIQM0SQKD4pSA23AIiLFa0FAQRmQjULAGJljwOPXcQsSxE0Q0bNhjLRfhlIiACeRIIixVmU3w1CBaWgiRWPBG50SOgHpWQgERLDrD97MrDDz\/sloU0m5IDPGUVgUwEECwsBaWKFcIsBSFcMpVVvAiIQE0RkGjJ8XBPmDDB2ISb+sj9HKtRdhEQAcTK3QEG7grCH3gNgeLFimZXrF\/\/VFgEqpCAREsOB5U9KnfeeaexCTeHYsoqAiIQJoBA8ftWvh9K0C3MIRjyioAIpCMg0ZKOiuJEQASKQ6DNzNItBbFvRbcwF4e5ahWBKiIg0VJFB1NDEYHIEmB2BbGSegvztUGP2beiW5gDEHqJgAj0RUCipS9CSq8NAhplcQggVvxSULpbmBcVp1nVKgIiUJ0EJFqq87hqVCJQXgJhscLGWt+bWOBhKUibbAMQeomACORKQKIlV2Klza\/WRKDyCDCjwlJQqlghzFIQwqXyRqUei4AIRICAREsEDoK6IAJVQYDZFfasIFjwMygEihcrml2BiEwERKAfBPITLf1oUEVFQASqjAACxe9b4QcO\/fB0C7MnIVcERKBABCRaCgRS1YhATRJApDCzwmyKB8DsCvtWdAuzJyJXBNISUGTuBCRacmemEiIgAsyuIFZYDsIPEcQK4oV9K7qFGSIyERCBAhOQaCkwUFUnAlVNAIHil4LYcMtgw2JF+1YgUuGm7otAdAlItET32KhnIhAtAogUZleYTfE9Q7CwFCSx4onIFQERKCIBiZYiwu1v1ffdd5+NHj3a2dy5c23Pnj39rVLlRSB3An52BcGCnxpiwRvihaUg\/EGw2C\/VLwIiIAISLRH9DKxZs8ZWrVplGzZssBdffNH1sqWlxbl6E4GSEUCkIFbCHz38iBXNrpTsMKghERCBIwQkWo5wiNz766+\/biNGjLCGhgb3q9KTJ0+2t956S7MtkTtSVdyheDC2xsAQLoFjzKhoKcj0TwREoHwEJFrKx77XlseMGWPbtm2zzs5OJ1TWr19vCJchQ4b0Wk6JIlAQAggVZlh8ZX52RXcFeSJyRUAEykBAoqUM0LNpcvr06Xb33XfbjBkz7LzzzrOrrrrKFi5cmLGo3\/uC29jYaGTs6OiwaretW7dafX29HTp0yPbv319UO3DgQLf6YYxVHeOfBZ+bswOr77LrA3deYEX4PHH8qo5fETjlwkhMC\/tZ5W8cmzZtmttfiF9WPgISLeVj32vLbMJdtGiRPfXUU7Z582b7yU9+YjfccEPGMuTxxgmOjIiXareRI0c6ITFgwAAnXhAwxbKBAwd2a8O6\/lUV47pGa7wlsP1d9q3AXRpYY3GM41dV\/IrEKRdGYlrYz2rXn7mtXbvWnYt9WG55CJRYtJRnkJXWKncJsRw0Z84cGz58uOv+\/Pnz7ZlnnrFNmza5sN5EoCgE2MeCUTlLQdpsCwmZCIhARAhItETkQKgbIlBqAolEwtra2py1trZaW2ub2YKuXrDplsfwdwXliIAIRIhADXdFoiWCB5\/Ntmy65Zbn7du3ux6uWLHC\/LSvi9CbCORBwAuVqVOnGksOCxYssCXBMuQDd9xhm+\/YmKyxLdFmjVMbDTGTjJRHBERABMpMQKKlzAcgU\/MLFy60CRMm2MUXX+w2f3G787Jly9ztz5nKKF4EeiOAAJk6daoTKacnEnZ38Pnaefnl9tyllzq7peHPXfFDg3fYyVPa7IsNDdbS0uLEDWVdot5EIDMBpYhA0QlItBQdcf4N3HnnnW7jFxtsV65cKcGSP8qaLpkIBAqig9kUhMijU6bY9y+6yL40alSSy4DOYVa\/Y5wLH2zYbpOGDbObxo6152fOTIoXBI\/LoDcREAERKBMBiZYygVezIlAKAggWxAaCBaGCEDmzoaHXpg8MeyWZTl7KIF42b9zoZl2SiZXiUT9FQASqhoBES9UcSg1EBLoTQLCwZ+XQjh3G7AqzJ91zHA0ds\/fIXWrEHBx8ZB8Vfm+IF+oYsXevhIuHIlcERKDkBCRaSo5cDYqAI1D0NzZvMzuC2EB09NYgy0M+\/VDDDu\/t5lIH+2AQQ8zedEtUQAREQARKQECipQSQ1YQIlJpAPB53m2hZ2kFsFKp96kIEUT9WqHpVjwiIgAhkQ0CiJRtKtZRHY60KAmy8ZTkovNm2t4GFl4d6y0ca9c6JxYylJ8IyERABESgVAYmWUpFWOyJQIgLMgGDMshSrSepmmYh2itWG6hUBERCBVAKVIlpS+62wCORM4MYbb7R169Ylbfny9I98Jf6xxx6zSy65JOc2olCgvb3dzho82JgRCffnhFtvtY+vXt3DhlxzjYX3tAz+7te65SE9XA9+lomon30zhGUiIAIiUAoCEi2loKw2yk6ApYwZM2YYswNsIn3uuecsFixxLF26NNk3RApihfhkZAV6mP3obVlo39q19u7s2c72b9pkg6ZNs2NPmeBGevi0D+2Yi0834slDXtKHBMLGZQi9sUTU1tYWipFXBEQg2gQqv3cSLZV\/DDWCLAiccsopVk3Thg8AABAASURBVF9fb7t27XK5Fy9ebIgXXCIQNTfccIPL09nZSVRFGqIsHo\/3mGXJNJgPHn7YDgVM6nYceXbL4fffd2Jm97e\/7YqQZgcO2MCzz3bh8BuzLYRpE1cmAiIgAsUmINFSbMKqP1IELrzwQgvProQ7t2PHDuMpxJUsWsLjyca\/\/6WX7NDGHWZb6lx2nobrPF1vA885x2zgwK5Qd4clKGIkWqAg6w8BlRWBbAkMyDaj8olAJRNYs2aN7dy50w0B4fLkk092u\/uFfSxf\/vKXDeHiMlXomxcQXlD0NYwDwazSocTRXOGn4TZccYXVf+pTLvHAa685N\/zmZ1p8HEtFjY2N7uFz2brTgqWpbPP2la+1tdV3Ra4IiECVEpBoqdIDq2F1J\/D888\/b7NmzzV\/UWSqaN2+esTm3e87aCCFWdj77rG19\/HE7\/J\/HJge9f9jLzo9gOf5P\/9SYZWGJaN9\/\/IeLD7+9GQiecHjLli32zjvvhKNK5i9OuyXrvhoSARHIksCALPMpmwhUBQH2rlx\/\/fXJWZeJEyfaBRdcUBVjYxB+E\/Ebe\/cSTGt733zTiZU9iYQNHDzYBr5wUjIfomXQ5Ml23KxZhmBhPwubcVlGSmbqw0MfsrURI0ZYtnl7y9dHl5QsAiJQJQQGVMk4NAwRyJoAsy7333+\/Vereld4G6i\/sqbMg4TKH2FgbiJWTxo61s6691o757Yku+dDJu63+3HOtIVgmqzv+eEOwfPDII9b5ox+59NQ3hBHtNTU1pSYpLAIiIAJFISDRUhSsqjRqBNizwjNa\/HIQ3\/BZItqzZ48hYqLW3\/70BxHx9I4dGasY9LGPGYIFGzz+82ZvHDkNHDzmTRv02c\/agJNPdmX3rV+fUbCQ4cFgOQjRgl8mAiIgAqUgcORsVYqW1EaVE4j28FgWYj\/LzJkz3cPl2M+ybdu2bptxoz2C7Hs3ZcoUW5VIZCww9AtfsFHPPOMeIHfsH\/5hMt+BYa90u7WZ57P4h9Gd9Ld\/m8yHh5kc2kAgEZaJgAiIQCkISLSUgrLaiAQBhAvPZvFGOLVjzLqwYXfWrFnGHUap6ZUQbm5udt1kJsR5ut549goPjAvb7uaWrlSzg4O323vf+IZ7Tks4D37ikxkDDzM5zLLMnz8\/COklAiIgAqUhUPWipTQY1YoIRItAS0uLLdq40ZgR6a1n9TvGJZMPNWReUkpmCjzUefsrr1hTU5PbRBtE6SUCIiACJSEg0VISzGpEBEpLYMmSJU5QIFwK3TKCZcCwYUYbha5b9YmACESSQGQ6JdESmUOhjohAYQmw+ZhlnGuffTZjxeEfSmR5KGPGrgQEy88DP3WzPBR49RIBERCBkhGQaCkZajUkAqUlwPINd0yxYbY34eJ71dfyEIIFa25udktDvpxcESgbATVccwQkWmrukGvAtUSgqanJvHC5rL29zz0u6dj4PSw\/7uw09spoWSgdJcWJgAiUgoBESykoqw0RKCMBhEtHR4fVjxtnFzz+uDFbghDpq0vkIS9lECyIHwmWvqi5dL2JgAgUiYBES5HAFqJafuRv9OjRhvHsje3btxeiWtVRgwTYf4LoYKYEAYIQufCJJ4w9Lx4Ht0hjLCUxK0Me8lIG0UMdPq9cERABESgHAYmWclDPos1NmzbZ4sWLbdmyZbZ582abM2eOXXfddcYTXLMoriwi0JNAEMNMCQIEu+rGG427gIJo9+JOozt37LC3YzH7xKxZblmJfJRxGfQmAiIgAmUmINFS5gOQqfmnn37aZsyYYdOnT3dZFi5caCtXrrQhQ4a4sN5EoD8EmDVBjLB05Os5fPiwIVKYkeHuoHCazyNXBERABMpJQKKlnPQztM1syvr1623MmDEZclRVtAYjAiIgAiIgAlkRkGjJClN5Mg0dOtTYy5LNnhbyeGtsbHQd5ltztdvWrVuNHz48dOiQ7d+\/v6h24MCBbvU7yMFbRTPu7LCO+i7rCNx+2O7du23gwIHdGPV1TFKZ9pU\/Uzrt0n5FH4t+sA+Pm7+JcFj+\/n2ugz9x95o2bZrbX+gCeisbAYmWTOgjEM9+ltWrV7s9LRMmTOh1Twv7XrxxkqL7iJdqt5EjR7qL5IABA5x4QcAUy7gwhuu2rn8Vzbih0Rr3d1kgdvszlhNOOMEQIWFGfflTmfaVP1M67dJ+f\/pfLWX5m6iWsURhHF1\/5rZ27Vp3LvZhueUhINFSHu5Ztcrm2+HDh7u8\/DCd\/wblIvQmAoUg4H8MOlaIylSHCIiACHQnUOiQREuhiRagPjbbnnHGGT1qOvHEE+3UU0\/tEa8IERABERABEagFAhItET3Kn\/vc52zVqlXmn82yYsUKO+ecc8zPvES02+pWpRHQTEulHTH1tyAEVEmlEpBoieiR41bnW265xS6++GK3+eutt95yj1CPaHfVrUol4EVLpfZf\/RYBEagpAhItET7cCBe\/uVbPaInwgarUroUFS1OlDqK6+q3RiIAI9E5AoqV3PkoVgeolEBYt1TtKjUwERKCKCEi0VNHB1FBEICcC7aHcU0L+Hl5FiIAIiEA0CEi0ROM4qBciUHoC4ZkW3fJcev5qUQREIGcCEi05I1OBqBBQP\/pJwIsWBAvWz+pUXAREQASKTUCipdiEVb8IRJEAgiXe1TEJli4QckRABKJOQKKl4EdIFYpAhRHQnUMVdsDUXRGoXQISLbV77DXyWibATIsfvzbhehJyRUAEokIgQz8kWjKAUbQIVDWB8J1DVT1QDU4ERKCaCEi0VNPR1FhEIFsC4f0sWh7KlpryiYAIlJmAREuZD4CaF4GyEPDLQ9qEWxb8alQERCA\/AhIt+XFTKRGoXAIIFqxyR6CepxJQWARqhIBES40caA1TBJIEwoJFS0NJLPKIgAhEn4BES\/SPkXooAoUlEBYtxb1zqLD9Vm0iIAI1T0CipeY\/AgJQcwS21NyINWAREIEqISDRUiUHUsPIgYCyHiVQpo24N954o61bt85++MMfOnf58uXJPl1wwQW2evVqF0+eJ5980hYsWJBMl0cERKB2CUi01O6x18hrlUB4eagMDBAgM2bMsEQiYV\/5ylfsueees1gsZkuXLnW9ufLKK+2UU06xxx9\/3KZOnWqbNm2yOXPmSLg4OnoTgdomINESneOvnohAaQjEStNMplYQJPX19bZr1y6XZfHixU6c4BKBi1i54447CNo777xj5D\/33HNdWG8iIAK1S0CipXaPvUZeqwRGhQbuHzIXiiqV98ILL7Sbb765z+bGjh3bZx5lEAERqA0CfYuW2uCgUYpAZRFoDbrbGFhdYLiEA29Wr+ZQrjI8zn\/NmjW2c+dO14lx48ZZb3tW2PvC0tH+\/fvtpZdecmX0JgIiULsEJFpq99hr5JVKoC3oeEtgicB44RJGvJBGXF\/mn89Cfsr3lb+A6c8\/\/7zNnj3b7WmhWpZ+5s2bZwgUwt4Iz5w50wW3bdtm4c26LlJvIlBBBNTVwhCQaCkMR9UiAqUhgMDwN9KwNyU8a+LTspl1mR\/qrq8vFFUKLxtyb7vttuSsy8SJE407h2ibNDbr4u\/s7LRVq1bhlYmACNQ4AYmWGv8AaPgVRiAsSBAs3Cl8OBgDMy2B4174w\/lcZMobZf1sC\/ta+sqfUrxQwVdeecXuv\/9+Q5j4Oi+55BK7\/PLL3eZb4r\/3ve+5JSSfLreQBFSXCFQWAYmWyjpe6m0tE0BcsJwDA2ZZluDpMvzruvw4CBfy489kCB6flk1+n7efLss8PH+F5R+qGjFihBMoe\/bsMZaO\/viP\/9gaGhqMfSwPPfSQBAuQZCIgAo6ARIvDEO03nlMxefJk97yKaPdUvSsagURQc3g2JCw4giT3YuYkLFymBrG9CReETy75g+oK8WLph2e0sF+Fh8uxn4U9K8SzPHTWWWcZ7fi9LggczD\/HhTSZCIhAbRKQaIn4cefbJ+v+v\/vd7yLeU3WvqAQQHxiNBEs7iVjC2tra3APXuNhjra2t1treaolmFA4ZA+trvwpCJyyAEDqh4kENRXnRX57FwsPlcAnTEDMtbNIlLtV4fgt5ZCIgArVLQKIl4sd+w4YNtnXrVjvxxBMj3tNa6F6ZxoiI6BIfCUtYY1ujNTY22pJFi+zXjz2WtKfuucdaWlpcetzi5v5RFiHiAhnemoN4LHDcKzyj4yL0JgIiIALRICDREo3jkLYX27dvtwceeMBuvfXWtOmKrBECXYKF0T4w+A774tgGe3TKFHvu0kudi9\/bzssvt+dnzrQnxn7d4sF\/4x\/6pS\/hwmxLE5kDawsMsRM4eomACIhAlAhItETpaKT05eGHHzb2spx66qkpKd2DhEaPHm3e+BZOXEdHh1W7MQvF3ofXX3\/dnn322aIaDzcLt8E+jKJz\/pvgGD4dWH2H\/c8Jv7D\/Z+ob9qXgWJ85dKi98eGHae3wgAEuz8f+aLl1BOWcUccVQT29fSY+H6T7\/H8R+HvLmyZt9+7dNnDgQLeBlk202diBAwdyyp+pTtql\/Wr\/vGczPv4mssmnPNl9xvkbx6ZNm+bOsfhl5SMg0VI+9r22zOZb1vf58bheM3Ylbt682bxxMiIa8VLt9pnPfMa+9a1v2Q033FB0++pXv9qtDdptbm52SzVF4VwXLAMtDmx\/o406dogNu+Bf7azjjsvaRn6s006acYc1BuWd\/Tio69uBBUtLaft7fZA2IjDybw\/cTPkyxJ9wwgmGCEFEZmuIjWzz9paPdmk\/7bgy9Lda844cObJ4n8kaY8lnhHMptnbtWneOxS8rH4EiipbyDaoaWl6xYoV7auiQIUOqYThFHcOSJUusFLZo0aK07RRlcCzPTD1a875Rcds\/7OWjEVn6DjXssN1TWo7mZukntNx0NKHLF+tyWVLq8soRAREQgagQkGiJypEI9YO9LBs3brSrr77aTUdedtllbjMuLr\/bEsoqbzUSCARLYgFvRwa3Lxa3zrE\/PhLI4x2x00O4hARR2ipjaWMVKQIiUGoCaq8bAYmWbjiiERg+fLi1t7e7qUiWfB599FFjyhd3+vTp0eikelE8AivMYvEjquHQ4B2256J7+t1WD+ESD6rkt4pwA697BTrJfPhI86Z\/IiACIhAlAhItUToa6osItAYIulZzECy\/m8KjboO4ArwQLr+d+eeG66pDpDDjgnihXfwuIXgrXLNBZXpVIQENSQTKQkCipSzYc2t0\/Pjxtn79esPNraRyVxQBhENIsDDDwp6UQo6B+rgdutVorKtmxAvt4hLFLEsTHpkIiIAIRIuAREu0jod6U6sE0BAIh2D8zLCkbrw94dZb7eOrV\/ewIddcE5Q4+uqWb9Uqa7jiiqOJXb5Jw4YZz3tZ0LTArNmO\/kOsEO44GlVxPnVYBESgqglItFT14dXgKoJASLAkLGG\/HvZYxo23+9autXdnz3a2f9MmGzRtmiFULPjXEAiU+k99yg5u3Wq7W1vt0O7ddtysWTZo8uQgtfsL4dIWbzPjoXLIUk\/jAAAQAElEQVSHgzSECkY4COolAiIgAlEkINESxaOiPlUbgczjSREsK2yFvToqEBOZSyRTPnj4YTu0a5cNHDPGCZPOH\/3I3p0zx977xjds\/0sv2eHOTqurr7djTj89WcZ7EC3e71xmWZxHbyIgAiIQXQISLdE9NupZtRMICRYLREOiKWEtwf+zBg+2bP4hTA5u25ZWmNSfe67VNTS42Zb9L\/d8vosXLYlEIpumlEcEREAEIkFAoiUSh6FMnVCz5SOQIliMvSRdd+ycGYgNy\/Lf4YMHzQYOtAEnn5wswXLQ0JtucuE93\/++m3VxgTRvEi1poChKBEQgsgQGRLZn6pgIVCuBtmBgXZtuLZhhsZBgsRz\/7X\/\/fVfio\/fecy5v+9avt13z5hmzMCcsWWKpm3XJIxMBERCBSiQQRdFSiRzVZxHIjgAzLAtCWdMIljc7O0MZMnv3bNlihz76yA4HNvDQoR4ZD7z6qtmBAzbw7LN7pL2xd6+Li8ViztWbCBSCADN3bW1t1hZYa2urYfjj8XghqlcdImASLfoQiECpCCBY\/AwLbeLvWhIi2NTUhGNP79jh3N7eDgTC5sCwYXbcJz9ph\/fvt4Nvv50x+6Hf\/rZHGm0gWLAeiYooOgF\/cV+wYIFNnTrV6urqksaP9BHHBb\/oHSlAA4yFvtJn+s6YlixaZA\/ccYczwj6NfOQvQLOqok8C1ZlBoqU6j6tGVSgC7FPlS2JbUOEzgeXzog5mVxAplGdyA39IsBCNIVwQFPh7s50bN9qJX\/iC1Y8YYQc3bzaWhFgG4lku\/hboYy++2FXhZlyc7+gbbUiwHOVRKh8XbC7iXNy5sO+Lx+30RMLunjAhaV9saDA2T7e0tDghwwWfcqXqY7bt0CdECGNBoDCOR6dMsZ2XX27PXXpp0gg\/P3OmMS7ykT+qY8p27MpXPgISLeVjr5ajSiARdGxBYHWBNQY2NTDCXwlcH8esCfmCqIwv0slHHYgeMiJYmgPPksDSvObPn2+rEok0KUeiBk2b5h4wN277djvlq1+1zscft93f\/rZL3HPvvcazW+rHj3d5jjntNPvgkUeMW6Fdhq43lp8QLfwydleUnCITCF\/g165e7QQKF\/bvX3SRYV8aNcq83TR2rPmLP\/7NgUDlQo9AKHI3s66e8SA8ECH00Y\/F35WWWhGby32+5wMBgyhjTCwdpeZVWAR6IyDR0hsdpdUWAYQFAiUsMtIRSASRLYGRD0OYxIMw8Rj1EEca+YIk90KwrAt8GQRLkGLNzc049uCWLc71bwgT\/1A53P8ZN862TJxoHyzv\/jS4bvnmzOkhWKjv9ldesVgsZk1NTaZ\/xSfABZ7ZlfAFHoGSTcv+Qo\/b0tLilpKyKVfMPPF43BAcI\/budeKKvuXSHgIGUUY5uGC5lFfe2iYg0VLbx1+jhwBCA7HCbEqciC6LBW5zYOgC7NrA3xJY+EVZ4iiPSMGohzifj3oIdwQR+AOntxfCZVHw7bq3PCcF38aHBN\/Oe8uTLo1ZFmZyNMuSSqc4YQQLMxLMlvgLdT4tcYFnhsILhnzqKEQZP545sZgTLAiQfOtlTDBhtiVKs0j5jkflSkNAoqU0nNVKFAkgOPyMSFisIFSYEUFkIFYIY4uCQTBL4h97jxAJojK+YkEKeaiLckEwm9fyYPaEmZDL2tuzyZ51HgQLYoi6EUZZF1TGvAhwgWcW4dCOHf2+wNMBBALChXoRQsSV0ny7CBaWtArRNstJiJeWlhZDkBWiTtVR3QQkWqr7+Gp06QgkgkgvVloCv381Bx4EBkKlr5UTBAlCBAFDGerxRnniED3kIW9QdS4vhAv7Tq599tlcimXMi2BhhmXb4MG2bh2dy5hVCQUiwEWYGRYu8AiOQlRLPcxOUHdrKx\/iQtSaXR20hwBjPNmVyC4XogXxgsDLroRy1TIBiZZaPvq1NnYvVljKQWD48SMquI4jNvoSK75M2KUM4sQb4oe4cJ4c\/U1NTU5cIDT6K1y8YPlxZ6chhphpybE7yp4jAWYluAizd4ULco7Fe81OfVzomZ2gnV4zFyiRdljGod0CVdmtGoSYb6NbggIikEJAoiUFiILZEKiwPIiVtqDPXqwQDoKGWEG8MCPSZJH7FxYuFz7xhCE+cu0kZVhm8oKFOnOtQ\/lzJ7BixQo7K5jVKtZFnnqpn9mP3HuXewnaoT1EWO6lsyvBshNCL7vcylWrBCRaavXI18K4ESderLA5ljDjDosVZkeIi6ghMjo6Omz0hAl2weOPG3f+sGzUW3cRKuRhhoYylGVJiLp6K6e0whBgxoBZkNQLPM\/P4Tk6qcbzdXzLmfKc9Ld\/67Mk3S+NGmXMfiQjiuRhPLRDe6lNZOpveEyU4Qc8P\/Z3f+duxWf8J6UZD0KMvPF4HEcmAmkJVJVoSTtCRdYeAcRJWzBsZlbSiRWWgiIuVoLeJ18s5yA6WNphxoSZE2ZecBEx3hApGEKFtJ8HNVCGstQRBPUqAYFEIuFaYRnHeVLe9q1da9y2jvFcHZ69w8WfbOFb1ne3ttqhXbvcTzF8tGEDyd2sVBf5vsZDp3obE+mDv\/pVHGNMjPmYkSN7\/CYW+3WYzWGWymXWmwikISDRkgaKoiqUANcK9iamEyvNwZi8WGGmJQhW2qu5udmYdcGuuvFG+8SsWfbM8OGGkMEQKYOamty+FYQK+ShTaeOs9P62t7e7paFMoiU8vg8eftgJk4Fjxhi\/zh1OG\/TZz7pf797\/3\/+d9nk75OUiT3v4i2XUTzvZjIc+pI6p4Yor7JjTTnM\/4Ln\/pZfcwxARbDwMkfxhy7aNcBn5C04g0hVKtET68KhzWREIixX2qBCmIOKEMGKFTbaEia9wY9aE56z4WRTEiTfiECpNgXip8GFWbPeZmWDWIJsBcBE\/uG2b1dXX2zGnn54swnJK\/ac\/bYc\/+MB41H8yIcXDRZ72UqILHsx2PDScOqYBJ59sNnCgcecR6b0Z7cTj8d6yKK3GCQyo8fFr+JVMAHHiZ1YQJ4QZD+KEMGKFZSDCxMtEoAQEEBHMTOTUVHBRdxf3rkL148bZgBNOsEPvvut+V6orOq1De2kTChSZd\/3BmD7Yu9fqEC382ngwJvazYH45LLWLvXJLzaxwTRKQaKnJw17hg0acIFZ4+izihDBDQpwQlliBhqxCCBw+eND1tPPtt53L28BzzjFmJw689hrBirVDgVjZFwgvxmL797u9POxp4fexUjfrMsg3ApGDKxOBTAQGZEpQvAhEjgDiJCxWfAe9WOHWZc2seCpyS0OgRyuxWKxHXKaIA52d9tF779nhjz5ysyrkY2nomBEj3NIQPyxIXDmN8XBHWs59CATLMYFQ2ZdIuKK724884fnAq6+6zcUDzz7bxae+0V5qnMIi4AlItHgSEXO3b99uU6ZMsdGjRzubO3eu7dmzJ2K9LFF3OOdlI1ZK1B01IwJ9EchmxgDBwl\/0oE98wg7t22eD6upctSwT1R1\/vBMt7u4hF5v+jXaKvX9p1KhRRjvpe9AzNim6AsFSH9ig+nonygaedpptfeIJ+3Dnzp6FumK4Vb\/LK0cE0hKQaEmLpbyRiJPrrrvO5syZY5s3b7YXX3zRdailpcW5NfMmsZL\/oVbJshHgy0Y2F9+dGzfaiV\/4gtUHsyqHEonk3hU25LIx93AwC8Om1kwDYfaDdhAVmfIUIr65udlVQ1vO08ebv+vpwOuvuzEd7uiww++\/b8ePHWvH\/\/7v28BApLFclLr05cfTpE3kfRCu7WSJlgge\/yFDhtjKlStt4cKFrneEJ0+ebG+99VZtzLYkgmFrZiWAoFclEujrIj9o2jT3kLVxwWzqKV\/9qnU+\/ri7DdiPlZkWLuqHfvtbH5XW9bMfpVhOoY0Ht2xJ2w8i\/ZjYZIuf57bwzBnSEF577r7bBgwaZGf\/9Kc29I\/+yHb+\/d\/bnnvvJTlpjId2EH3JSHlEIIWAREsKEAWLSqD3yiVWeuej1IohwGxB6kWeizjPJ0m1D5ZzP\/7RoXExJw\/5j8b29PFQQdrBeqYWNoZb7PkdLGZDwjXTR\/qaaowhnA\/h8tuvfc1txP3NrFm274c\/DCc7P+NBtJRiPK5BvVUkAYmWCjhs7G9ZtWqVMdvCrEu6Lvu9L7iNjdxWY8kHkflneBTS\/bd\/+zc74YQT7Nhjj3X2mc98xl5++WXX5he\/+EUX59PCLmn0g7yUIe1r\/+tr1nFFh3WcHdhfBlZ\/xJ4+9mlbXL\/Yjt0WtPGXgXW19ffBtzTqwKV8OmPK\/JlnnnH9IW8hbOvWrQWtrxB9ilIdu3fvtoMHD9qmTZuyttdeey3rvL3VS7u0HxUen\/\/85+2hbdvskWB29I0PP7RC238FszDPvPeem41NHXMxPqfMfiAoWl95pd9jeWvAANs+dGi3euCUaTyp4yt12J9vpwUzZJxffVhueQhItMA9wub3t4wI1r2vvPLKjD1l74s3\/qjJiHgphh133HH29a9\/3S688ELbtWuX\/dM\/\/ZM9\/fTTdvvttxvt\/fjHP7aPPvrItgTTyaeffrr94R\/+octHHGnk+TA4ke96bpct27\/M\/u6xv7PGHzda4\/4uG9Fodd+qs0sGXmI75+w0ymG7grao69vf\/rbRh68GU+vEY+z\/qa+vt5\/\/\/OcuP21PnDjR9Yf2CmEjR44saH2F6FOU6uBC\/a1vfcuuvfbarO0rX\/lK1nl7q5d2aT8qPK6\/\/nqbNGmSLf\/1r+2s4O+lkFZ36JD91YsvGueEK664osdnslifUx5c+EgiYW++\/35Bx8R4vv3CC\/blL3\/Z0o2n3MeUcym2du1at8cQv6x8BCRayse+z5YRLFdffbXLt2zZMss0y+IylPCNvTU7d+40ZjPoExeLw4cPW9a\/GZIw+\/DmD+2VD16x5uC\/+X+xwMNMeYfZu\/\/r3SDQ\/UVbl156qb3xxhv27LPPdk9UqOwEmNZnGSEXW7RokeWSv7e8tF92CKEO0Fc2r7LsEYrut3dVImHbBg92P9fQ78pyqAC+GL9rxbhyKJoxK8tNizZutAHDhpV8PBk7pYS8CZSioERLKSjn0YYXLGeccYbblMsFO49qilrkBz\/4gc2fPz\/7NhJB1q4NtpPWTAoCR15bB261757zXduzaY9Zs+mfCFQFgaamJuM3oBAtqftb8h0gdWHNzc3W1NSUbzV5l2M8tMsPc\/ZXuHjBggCj3rw7pYI1RUCiJaKHu6WlxfXMuy4Qkbff+73fs8svv9z1BuFSV1dn\/\/qv\/+rC6d5O+\/A0O\/avjjXjhwyPDMtl2zlkp1kQ\/tbcb9l33vyOvcpDpyzzP4TcE088YWeddZZddNFFmTMqRQQiQoALPMsqzCZwoc+3W1zgESsY5wRmcfKtq7\/lEBijJ0wwZlzoTz71MR7Ke8ESi8XyqSbLMspWTQQkWiJ4NNl4uzGYMt2wYYOdd9557uFybABjMxxpUegyS0Hf\/e53k135kz\/5E\/vrv\/7rZNh7Rh4YaTe\/UE9rCgAAEABJREFUerMde1sgWphpCRJ+97HfWWvw\/+crf262xGzs2LHW2dlpa9asCVK7v7woqqurs6FDh7qNsHBhr0z3nAqJQDQJMCvChZ5lnQsD0Z3rDAWzNFzgfxz8jZRbsHjCjIe+IFoYE65P681FrCDeLnj8cZs2e7abiZJg6Y2Y0lIJSLSkEolAePjw4dbe3u42ffnNtbjEkRaBLrou\/MVf\/IWxl2XevHku\/H\/+z\/+xt\/3vpwQCZcjSIfb0W0\/bp9\/7tEu3mNlH3\/zILjvvMnvgrAeSsyXTp0+3hoYGYxaF2RQL\/aNu2vACiU15iJdQFnlFIPIEmHFhgzwXagQIF3ou9JkEDBd3L1aYpaEcQqGcMyypkOkLY7rqxhuNsTAmxoafvjM2XAyhQjpiJfiq4vavMAMVi8VSq1VYBHolINHSKx4lZkPgnnvuMWaBknkDwWILzIYuHZqMOvjZg2brzDb9703GLBKbadmvU1dX58QLMy3EZ1oi4m4l2kC40V6yYnlEoEIIcIHmQh2+0HOR52KOef8pDz1kXNzv3LHDPjFrlpuNoBzlozZU+hQWL\/XjxjkBg9BiPLiM4+1YzBA3CC\/Gz+xT1Mai\/lQGAYmWyjhOkeoly0B1dXWGS8fef\/99t2zDLMjH\/uVjZo1BbDyw4MUm27ZYm33w+AdmMXNLQAgUZk6YQfFGmPh0S0QW\/GMj8tKlS92MDLc8\/+IXvwhi9So8AdVYbAL+Qs9nnwt46913uwv6jK9\/3bkIFH9xx88sTbH71N\/6\/ZjoN+MKG2MkHnFTCWPpLwuVLy4BiZbi8q3K2lkWQmTcfPPNVldXZ8yY\/MHpf2BP\/uGTdtzC45JjPnjmQbti+BXWNqrNxbH0wxIQS0EsCbnIrjfCxJNOvq7obg4bgG+99Va3\/2Xx4sW18ZMG3QgoUG0EuNgz68AF3RthXdyr7UhrPIUiINFSKJJVUk+iM2Ftb7bZgl8ucNb6WqthqcNDuCS\/TXUctgc\/9eCRzbZkDGZUrMXsmDeOsae3PW3xeNw9Y4bZEvx79+41BIiF\/hEmnnTy+TAbfsnmzbfr8\/l48lGecj5OrgiIgAiIQHURkGipruPZr9EgTqb+fKoteGGBtW1tc9byWothjT9tNMRMjwbYvzI1iG0LjBeCZUngwQJHLxEQAREQAREoFAGJln6RrJ7CCBbESeIDVEjPcRGPmFkQzMAkU8mKYMElEsGyLvA0B6aXCIiACIiACBSYgERLgYFWYnXMoCBYsuk7MzDxd+NmXU+2tbBg6TC32TZ410sEREAEREAEsiOQQy6JlhxgVWNW9rAwg5Lt2GLbY9Z+bbtZix39hx\/BcjRGPhEQAREQAREoOAGJloIjrawKWfbJpseIleafNtu6b66zJau6NqywHNQSlO4KBj69REAERKBaCGgcESQg0RLBg1LKLrW\/G8yaZGiwZWWLYcvvWu7ECi7ixWVHsCBWMBehNxEQAREQAREoLgGJluLyjXztLA9l6iQzKhgzLEmxEmRundNqC\/55gZk23Jr+iUDJCahBEahhAhItNXzwcxl6YnjCECtT\/2qqtcxtsVgDUy251KC8IiACIiACItA\/AhIt\/eNX8aWnfHxKxjEgULC6f62zxr9vdGIlPj7u8vdWzmXQW60R0HhFQAREoOgEJFqKjjjaDTSf2Wyx49PPmiBQsNQRkL\/p402p0QqLgAiIgAiIQFEJSLQUFW9lVL7k7Nx20+aav6wU1LgIiIAIiEDVEJBoqZpDmf9AmG1pHpndrtqWs1uM\/Pm3ppIiIAIiIAIikB8BiZb8uPW3VOTKL\/\/0clt+\/vKM\/WJJCMGiWZaMiJQgAiIgAiJQZAISLUUGXEnVM4Ny+H8dduIFgcLsC4aY6fijDpNgqaSjqb6KgAiIQPUR6C5aqm98GlEeBBAvCJTlzL4ERjiPalREBERABERABApKQKKloDhVmQiIgAiIQK0T0PiLR0CipXhsVbMIiIAIiIAIiEABCUi0FBCmqhIBERCB6BJQz0Sg8glItFT+MdQIREAEREAERKAmCEi0RPgw33fffTZ69Ghn+CPcVXVNBPImoIIiIAIikC0BiZZsSZU436ZNm2zlypX26KOPOsNPXIm7oeZEQAREQAREIDIEJFoicyi6d+Tpp5+2kSNHWmNjo40fP94mTpxoxHXPpVDxCKhmERABERCBqBGQaInaEenqz+uvv25nnHGGDRkypCvGjLhkIAtPIpGwhKxgDLZt21awunRcEo6lmB7hkCjg36mYJtxnK1EgplmcapWlhAQkWkoIO9emxowZkyzi\/cmIFI\/f+4LL7AzJ06ZNM1nhGMybN088C\/yZEtPCfT7937qYFpZpXV0dp1P3t8\/51QX0VjYCEi1lQ1\/Yhjdv3mzeDh8+7CrHlR22QjEAaqHqUj1HjouYHuFQyM+DmBaH6dq1a905Fr6y8hEokGgp3wCqueXwclDY39eYOzo6XBZcWYcVigFQC1WX6jlyXMT0CIdCfh7EtDhMY7EYaGVlJiDRUuYDkKn5dMtB6eIylVe8CIiACIhAhRFQd\/skINHSJ6LyZJg0aZI988wzxm3OGH7iytMbtSoCIiACIiAC5Scg0VL+Y5C2B9zmPHfuXLvsssuc4ScubeY0kexvSROtqH4QENN+wMtQVEwzgOlHdBGY9qM31VFUTKNzHCVaonMsevRk4cKFbuMXfzD4e2RQhAiIgAiIgAjUEAGJlho62BqqCFQVAQ1GBESg5ghItNTcIdeARUAEREAERKAyCUi0VOZxU6+jS0A9EwEREAERKBIBiZYigVW1IiACIiACIiAChSUg0VJYntGtTT0TAREQAREQgQonINFS4Qcwtfs33HCD8fsY2Jo1a1KTFc6RwPbt223KlClJpnC97777cqxF2T0BPp+p\/MKMYU3Y55fbN4F0TPnb57Pq7fzzz3fPfOq7ttrNweeOz59nxmMm9uzZkwQSTicf4WSiPCUjUG7RUrKB1kJDnKg2btxoGzZssGXLltl3vvMd0x9W\/478b37zGzvhhBMcU249x3T7eX5Mubg+\/PDDPQrffvvtNmHCBHd7Py7hHpkUkZZAJqb87McXvvAFx5TP7AsvvGC5POcpbWNVHIk4ue6662zOnDmO2YsvvuhG29LS4lze+Fzy+YQnLmHiZaUlINFSWt5Fbe0nP\/mJO\/kPHz7cLr74YhsxYoT5P76iNlzFlSNahg4dag0NDVU8yuIODeHMN9NXX33Vxo0b160x0hDan\/vc51z8\/PnzjXzEuwi9pSUAn0xMKYBo0c9+QCI7GzJkiK1cudL8FxLCkydPtrfeessQNPCO9uc0u3FWQy6Jlmo4isEY+MPiDyz1RMXJK0jWK08C4pcnuJRi99xzj61atcoQgOEkROHhw4ft1FNPTUb\/7ne\/M+KTEfKkJZCJqT8XpC2kyLwI8HnU5zQvdAUvJNFScKTlrdCLFr4pnHHGGeXtTBW0jmhhue28885z+1pS17mrYIhFHwIzf+eee27Gdk488cSkaEG8EM6YWQmOQG9MOzs7bdu2bcbyhd+fkbqPyFVSgrdKbYKZFUQ2sy2cSxkHn0s+n\/hxCeOXlZaAREtpeau1CiLgv7H6vQF+qS28zl1Bw1FXa4QAswLvvfee29fG\/otHH33U7r33XmPPW40g6Ncw+bu\/7rrr3PL6lVde2a+6VLjwBCRaCs+0rDUyM0AH+MNjuQi\/LD8CfMNinfvOO+90FRC+6qqrjLVtvom5SL31m0B4OYgLLuF+V9prBdWdyIZbNt5Onz7dDZTwjBkzjD1vLkJvGQlw3rz66qtdOjcz8DfvAsEbn0s+n4HXLV8Sxi8rLQGJltLyLlpr\/HGlWw7yy0VFa7gGK2aDszbmFubAp5tmZ9qd+MK0oFo8AZ0LPIn0rhcsnEf5ssI51efk88jn0odxCROPX1Y6AhItpWNd9Ja4A4N1WGYB2IfBujZ7MYrecJU2AEfu0PDT6oS5jTy8zl3IoddiXezNOOecc2zFihVu+LiEiXcResuZAJ9XPrd8XilM+KmnnrJJkyYRlGUg4Jd9vRvOxueRzyWfT+JxCRNPWFY6AhItpWNd9JaYDub5AdzuzBTnLbfcYvqjyh877FavXu2ed8OGRrjCd+HChflXqpI9CNx0001uyQ3GLL0R7pFJEVkT4DzA3z6fV5hyLli6dKme09ILQQQenz2+7PFFD25YWPzxuSQP8biEe6lSSUUiINFSJLDlqpb9F2y+wzh59b8ftV0DwqW9vd09cAqm8K1tIvmPnul2pt1TRV+YMawJ599KbZXMxJS\/fT6v3gjXFpncRstnjs+e5+Vd4kijNlzCpOESJl5WWgISLaXlrdZEQAREQAREQATyJFCxoiXP8aqYCIiACIiACIhAhRKQaKnQA6dui4AIiIAIiEA\/CVRccYmWijtk6rAIiIAIiIAI1CYBiZbaPO4atQiIgAhEl4B6JgIZCEi0ZACjaBEQgd4J8PwPbv+84YYbemT0abipiZs2bbLzzz\/f\/ZZTurKp+VPD\/I4O7WLp6k\/Nr7AIiED1EJBoqZ5jqZGIQEkJcBstjzp\/+OGHu\/2uDaJk8eLFxnMsyJOuUzxNlN\/EyecWcm6Z5nkaZ555ZrqqixmnukVABMpMQKKlzAdAzYtAJRNAlPCDkjwpmAd08Sj02267zcaNG2f6sblKPrLquwhEk4BESzSPi3olAtkTKHNOZlTowu23327\/8A\/\/YPx8xF133WU8+Iz4voyZGX4agae2suSD+SeRsnxEGMPfV11KFwERqG4CEi3VfXw1OhEoOgGeDMpj41kmQrjgJy6XhvnF3F\/84hf24osvOuNHKXkMPb+nxRNIWUri93O0hyUXqsorAtVHQKKl+o5pVEakftQQAX6vhT0mGP58hn7VVVe52RlmaJh5QbRg1HXqqafaSSedhFcmAiJQwwQkWmr44GvoIlAoAsywHD582N577z3Dn2u9bMxFmORaTvlFQARqi0DtiZbaOr4arQgUnQC3ILN0c8899xj7Ulgm0jJO0bGrARGoSQISLTV52DVoESgMATbR3nvvvXbNNdfY+PHjzd9NxC3PpBWmFdUiAiIQNQLl6o9ES7nIq10RqHACmW5v5m4i9p9w6zN5KnyY6r4IiECECEi0ROhgqCsiUCkEECNXX3112tubuXPo7rvvtpdfftlaWloqZUjqZ1UQ0CCqnYBES7UfYY1PBIpAgDt8Vq5cae3t7YZISW2CpaIXXnjBsnniLXnXr1\/vlpd8PTz1lvpphzjaoC2WnwjLREAEapOAREttHneNWgREoIQE1JQIiEBhCEi0FIajahEBEciBAA+Tu+yyyyyfp9xytxLPb3nzzTdzaFFZRUAEqoGAREs1HEWNQQTyIlCeQiwHsXTEk26zWT5K7SVLR5TFtFyUSkdhEahuAhIt1X18NToREAEREAERqBoCEi1VcyirZyAaiQiIgAiIgAikIyDRko6K4kRABChqLPcAAAAhSURBVERABERABCJHQKIl60OijCIgAiIgAiIgAuUk8P8DAAD\/\/3+bDjMAAAAGSURBVAMAHvwwbCE42vYAAAAASUVORK5CYII=","height":418,"width":557}}
%---
%[output:850af005]
%   data: {"dataType":"text","outputData":{"text":" \n","truncated":false}}
%---
%[output:03f97987]
%   data: {"dataType":"text","outputData":{"text":"Resumen de la ejecucion RRT* + APF\n","truncated":false}}
%---
%[output:2c78bfc3]
%   data: {"dataType":"text","outputData":{"text":"    <strong>Arquitectura<\/strong>    <strong>Escenario<\/strong>    <strong>Semilla<\/strong>    <strong>Exito<\/strong>    <strong>MetaAlcanzada<\/strong>    <strong>Colision<\/strong>     <strong>MotivoTerminacion<\/strong>     <strong>PasosEjecutados<\/strong>    <strong>TiempoSimulado_s<\/strong>    <strong>TiempoHastaMeta_s<\/strong>    <strong>Longitud_m<\/strong>    <strong>SuavidadRMS_1_m<\/strong>    <strong>DistanciaMinima_m<\/strong>    <strong>DistanciaFinalMeta_m<\/strong>    <strong>Planificaciones<\/strong>    <strong>Replanificaciones<\/strong>    <strong>FallosPlanificacion<\/strong>    <strong>EpisodiosRiesgo<\/strong>    <strong>EpisodiosColision<\/strong>    <strong>TiempoPlanificacionTotal_s<\/strong>    <strong>TiempoPlanificacionMedio_s<\/strong>    <strong>TiempoControlTotal_s<\/strong>    <strong>TiempoControlMedio_s<\/strong>    <strong>TiempoControlMaximo_s<\/strong>    <strong>TiempoCicloMedio_s<\/strong>    <strong>TiempoCicloMaximo_s<\/strong>    <strong>TiempoComputoTotal_s<\/strong>    <strong>FactorTiempoReal<\/strong>    <strong>CiclosFueraPlazo<\/strong>    <strong>EsfuerzoControlNormalizado_s<\/strong>    <strong>VariacionControlNormalizada<\/strong>\n    <strong>____________<\/strong>    <strong>_________<\/strong>    <strong>_______<\/strong>    <strong>_____<\/strong>    <strong>_____________<\/strong>    <strong>________<\/strong>    <strong>___________________<\/strong>    <strong>_______________<\/strong>    <strong>________________<\/strong>    <strong>_________________<\/strong>    <strong>__________<\/strong>    <strong>_______________<\/strong>    <strong>_________________<\/strong>    <strong>____________________<\/strong>    <strong>_______________<\/strong>    <strong>_________________<\/strong>    <strong>___________________<\/strong>    <strong>_______________<\/strong>    <strong>_________________<\/strong>    <strong>__________________________<\/strong>    <strong>__________________________<\/strong>    <strong>____________________<\/strong>    <strong>____________________<\/strong>    <strong>_____________________<\/strong>    <strong>__________________<\/strong>    <strong>___________________<\/strong>    <strong>____________________<\/strong>    <strong>________________<\/strong>    <strong>________________<\/strong>    <strong>____________________________<\/strong>    <strong>___________________________<\/strong>\n\n    \"RRT* + APF\"     \"alta\"         7       false        true          true       \"meta_con_colision\"          477               71.55                71.55            34.163          4.4596             -0.89897                0.5305                 55                  54                     0                    1                   1                      3.0287                       0.055067                   0.68375                0.0015094                 0.09899                0.012601               0.77149                 6.0106                11.904                1                       76.394                         56.276           \n\n","truncated":false}}
%---
%[output:7f808f90]
%   data: {"dataType":"text","outputData":{"text":"Episodios de minimo local APF: 0\n","truncated":false}}
%---
