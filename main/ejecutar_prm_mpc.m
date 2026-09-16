%[text] # PRM + MPC en un entorno dinamico
%[text] Ensambla los modulos del proyecto TFM\_RobotNavigation para ejecutar una simulacion completa de la arquitectura PRM + MPC. El planificador global utiliza una roadmap PRM persistente: se construye una sola vez con la geometria estatica y se reutiliza en las consultas posteriores desde la posicion actual del robot. MPC recibe el objetivo adelantado comun, predice el comportamiento del robot y de los obstaculos y genera el control \[v w\]. El robot se integra mediante el mismo modelo de uniciclo \[x y theta\]. La visualizacion conserva la misma estructura: los objetos se crean una vez, se actualizan sin clf y el arbol grafico reducido de la consulta PRM anterior se sustituye completamente cuando existe una nueva consulta.
clearvars;
clc;
close all;
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
    error('ejecutar_prm_mpc:RaizNoEncontrada', ...
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
        error('ejecutar_prm_mpc:CarpetaAusente', ...
            'No existe la carpeta requerida: %s',char(carpetaModulo));
    end

    addpath(char(carpetaModulo),'-begin');
end

rehash path;
fprintf('Raiz del proyecto: %s\n',raizProyecto); %[output:67459425]


% Comprobacion temprana de las dependencias utilizadas por este main.
funcionesNecesarias = [ ...
    "parametros_generales", ...
    "escenarios", ...
    "configuracion_robot", ...
    "prm", ...
    "consulta_prm", ...
    "seguimiento_trayectoria", ...
    "mpc", ...
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
        error('ejecutar_prm_mpc:DependenciaAusente', ...
            'No se encontro la funcion %s.m en el path del proyecto.', ...
            funcionesNecesarias(iFuncion));
    end
end
%%
%[text] ## Configuracion comun, escenario y robot
cfg = parametros_generales(modoEjecucion);
cfg.semilla = semilla;
cfg.nombreArquitectura = "PRM + MPC";
cfg.planificador = @prm;
cfg.controlador = @mpc;

if ~isnumeric(factorVelocidadAnimacion) || ...
        ~isscalar(factorVelocidadAnimacion) || ...
        ~isfinite(factorVelocidadAnimacion) || ...
        factorVelocidadAnimacion <= 0
    error('ejecutar_prm_mpc:FactorAnimacionNoValido', ...
        'factorVelocidadAnimacion debe ser un escalar positivo.');
end

if cfg.visual.activa
    cfg.visual.pausa = true;
    cfg.visual.tPausa = ...
        cfg.sim.Ts/factorVelocidadAnimacion;
end

escenario = escenarios(idEscenario);
robot = configuracion_robot();

% La semilla se fija en el main para que la construccion de la roadmap
% PRM y el resto de arquitecturas sean reproducibles.
rng(cfg.semilla,'twister');
%%
%[text] ## Construccion inicial de la roadmap PRM persistente
%[text] La roadmap contiene exclusivamente la geometria estatica. Su coste se registra como inicializacion y no como tiempo del controlador.
relojTotal = tic;
relojConstruccionPRM = tic;

[roadmapPRM,infoConstruccionPRM] = prm( ...
    escenario.meta, ...
    escenario.limites, ...
    escenario.obstaculosEstaticos, ...
    robot, ...
    cfg);

tiempoConstruccionPRM = toc(relojConstruccionPRM);

if ~infoConstruccionPRM.roadmapUtilizable
    warning('ejecutar_prm_mpc:RoadmapInicialDebil', ...
        ['La roadmap inicial no contiene una componente utilizable ' ...
         'conectada con la meta. Las consultas y la ampliacion ' ...
         'excepcional intentaran recuperar la conectividad.']);
end
%%
%[text] ## Inicializacion del estado y de los historiales
estadoRobot = escenario.inicio;
numeroRecuperacionesColision = 0;
enColisionDinamicaAnterior = false;
obstaculosDinamicos = escenario.obstaculosDinamicos;
caminoGlobal = zeros(0,2);
caminoGlobalEsParcial = false;

infoConsultaPRM = struct();
infoExpansionPRM = struct();
infoMPC = struct();

% El MPC penaliza los cambios respecto al control realmente aplicado en el
% paso anterior. Se actualiza despues de resolver colisiones y saturaciones.
controlAnterior = robot.controlInicial;
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

% Diagnosticos complementarios del planificador, del controlador y del seguimiento.
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
motivoPRM = strings(maxPasos,1);
motivoExpansionPRM = strings(maxPasos,1);
motivoReplanificacion = strings(maxPasos,1);
motivoSeguimiento = strings(maxPasos,1);
caminoParcialPorPaso = false(maxPasos,1);

expansionPRMEjecutada = false(maxPasos,1);
numeroAristasBloqueadasPRM = nan(maxPasos,1);
fraccionAristasBloqueadasPRM = nan(maxPasos,1);
numeroConexionesInicioPRM = nan(maxPasos,1);
numeroNodosRoadmapPRM = nan(maxPasos,1);
numeroAristasRoadmapPRM = nan(maxPasos,1);

numeroPlanificaciones = 0;
numeroConsultasPRM = 0;
numeroReplanificaciones = 0;
numeroFallosPlanificacion = 0;
numeroEpisodiosSinCandidatoFactible = 0;
numeroEpisodiosRiesgoPredicho = 0;
numeroEpisodiosColisionPredicha = 0;
numeroRecuperacionesColision = 0;
numeroAproximacionesParciales = 0;

fallosConsultaConsecutivos = 0;
maximoFallosConsultaConsecutivos = 0;

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
%%
%[text] ## Inicializacion diferida de la visualizacion y la exportacion
%[text] La figura se crea dentro del primer ciclo de simulacion. De este modo no se genera un resultado independiente de iteracion cero y el Live Editor puede registrar los drawnow del bucle como fotogramas reproducibles.
graficos = struct();
graficos.activa = false;
graficosInicializados = false;

exportacion = struct();
infoExportacion = struct();
archivoAnimacion = "";
exportacionIniciada = false;

% La configuracion del archivo se prepara ahora, pero el escritor solo se
% abre despues de crear la figura durante la primera iteracion.
if exportarAnimacion
    extension = lower(strtrim(string(formatoAnimacion)));

    if ~any(extension == ["mp4","m4v","avi","gif"])
        error('ejecutar_prm_mpc:FormatoAnimacionNoValido', ...
            'El formato debe ser mp4, m4v, avi o gif.');
    end

    nombreAnimacion = "prm_mpc_"+string(escenario.id)+ ...
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
%[text] ## Simulacion PRM + MPC
try
    for k = 1:maxPasos
        pasosEjecutados = k;
        relojCiclo = tic;

        textoEstado = "Navegacion";
        mpcFactibleActual = true;
        riesgoPredichoActual = false;
        colisionPredichaActual = false;
        controlSolicitado = robot.controlParada;
        motivoPRM(k) = "no_ejecutado";
        motivoExpansionPRM(k) = "no_ejecutada";
        motivoMPCPorPaso(k) = "control_no_ejecutado";

        vistaPlanificador = struct( ...
            'actualizar',false, ...
            'nodos',zeros(0,2), ...
            'aristas',zeros(0,2));
%%
%[text] ## Movimiento de los obstaculos dinamicos
        obstaculosDinamicos = actualizar_obstaculos( ...
            obstaculosDinamicos, ...
            escenario.obstaculosEstaticos, ...
            escenario.limites, ...
            cfg.sim.Ts);

        for iObstaculo = 1:numeroDinamicos
            historialObstaculos(k+1,iObstaculo,:) = reshape( ...
                obstaculosDinamicos(iObstaculo).pos,1,1,2);
        end
%%
%[text] ## Decision global de replanificacion
%[text] Para mantener la misma filosofia que RRT\* + MPC, los obstaculos moviles se gestionan localmente mediante MPC. La roadmap y sus consultas globales utilizan solamente limites y obstaculos fijos.
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

            [caminoCandidato,infoConsultaPRM] = consulta_prm( ...
                roadmapPRM, ...
                estadoRobot, ...
                dinamicosParaPlanificacion, ...
                robot, ...
                cfg);

            numeroConsultasPRM = numeroConsultasPRM+1;

            if infoConsultaPRM.exito
                fallosConsultaConsecutivos = 0;
            else
                fallosConsultaConsecutivos = ...
                    fallosConsultaConsecutivos+1;

                maximoFallosConsultaConsecutivos = max( ...
                    maximoFallosConsultaConsecutivos, ...
                    fallosConsultaConsecutivos);
            end

            % Tras varios fallos consecutivos se amplia excepcionalmente la
            % roadmap existente. No se reconstruye desde cero ni se pierden
            % sus nodos y aristas anteriores.
            debeAmpliarRoadmap = ...
                ~infoConsultaPRM.exito && ...
                fallosConsultaConsecutivos >= ...
                    cfg.prm.ampliarTrasFallos && ...
                roadmapPRM.numeroExpansiones < ...
                    cfg.prm.maxExpansiones;

            if debeAmpliarRoadmap
                [roadmapPRM,infoExpansionPRM] = prm( ...
                    escenario.meta, ...
                    escenario.limites, ...
                    escenario.obstaculosEstaticos, ...
                    robot, ...
                    cfg, ...
                    roadmapPRM, ...
                    estadoRobot);

                expansionPRMEjecutada(k) = true;
                motivoExpansionPRM(k) = ...
                    string(infoExpansionPRM.motivo);

                % La consulta se repite inmediatamente sobre la roadmap ya
                % ampliada. El tiempo de ambas operaciones queda incluido
                % en el mismo ciclo de planificacion.
                [caminoCandidato,infoConsultaPRM] = consulta_prm( ...
                    roadmapPRM, ...
                    estadoRobot, ...
                    dinamicosParaPlanificacion, ...
                    robot, ...
                    cfg);

                numeroConsultasPRM = numeroConsultasPRM+1;
                fallosConsultaConsecutivos = 0;

                if ~infoConsultaPRM.exito
                    fallosConsultaConsecutivos = 1;
                end
            end

            tiempoPlanificacion(k) = toc(relojPlanificacion);
            planificacionEjecutada(k) = true;
            motivoPRM(k) = string(infoConsultaPRM.motivo);
            numeroPlanificaciones = numeroPlanificaciones+1;

            if ~eraPlanificacionInicial
                numeroReplanificaciones = ...
                    numeroReplanificaciones+1;
            end

            % consulta_prm devuelve un arbol grafico reducido del componente
            % alcanzable desde el robot. Se representa incluso si la
            % consulta falla, sustituyendo la vista anterior.
            vistaPlanificador = infoConsultaPRM.vistaPlanificador;

            numeroAristasBloqueadasPRM(k) = ...
                infoConsultaPRM.numeroAristasBloqueadas;
            fraccionAristasBloqueadasPRM(k) = ...
                infoConsultaPRM.fraccionAristasBloqueadas;
            numeroConexionesInicioPRM(k) = ...
                infoConsultaPRM.numeroConexionesTemporalesInicio;
            numeroNodosRoadmapPRM(k) = roadmapPRM.numeroNodos;
            numeroAristasRoadmapPRM(k) = roadmapPRM.numeroAristas;

            if infoConsultaPRM.exito
                caminoGlobal = caminoCandidato;
                caminoGlobalEsParcial = false;
                siguientePasoPermitido = k+1;

                if eraPlanificacionInicial
                    textoEstado = "Consulta PRM inicial";
                elseif expansionPRMEjecutada(k)
                    textoEstado = "Consulta tras expansion PRM";
                else
                    textoEstado = "Nueva consulta PRM";
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
                        "Consulta PRM fallida: ruta anterior";
                else
                    caminoGlobal = zeros(0,2);
                    textoEstado = "Sin camino global PRM";
                end
            end

        elseif infoReplanificacion.solicitada && ...
                ~infoReplanificacion.permitida
            textoEstado = "Esperando reintento";
        end
%%
%[text] ## Objetivo local comun sobre la trayectoria global
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
%%
%[text] ## Colision instantanea antes del control
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
%%
%[text] ## Control local MPC
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
%%
%[text] ## Modelo cinematico, contacto y recuperacion
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
%%
%[text] ## Visualizacion y captura fuera del tiempo computacional
        if cfg.visual.activa %[output:group:0f04cabb]
            relojVisualizacion = tic;

            % La figura se crea por primera vez dentro del bucle. No existe
            % una figura separada de inicializacion ni un fotograma cero.
            if ~graficosInicializados
                graficos = inicializar_figura( ... %[output:7aeddb6b]
                    escenario,cfg,cfg.nombreArquitectura); %[output:7aeddb6b]

                % Proteccion frente a versiones anteriores del modulo que
                % creaban la figura con Visible='off'.
                if isfield(graficos,'figura') && ...
                        isgraphics(graficos.figura,'figure')
                    set(graficos.figura,'Visible','on');
                end

                graficos = dibujar_entorno( ...
                    graficos,escenario,obstaculosDinamicos);

                graficos = dibujar_robot( ...
                    graficos,estadoRobot,robot);

                graficosInicializados = true;

                % La exportacion externa comienza una vez que existe una
                % figura valida. Todavia no se captura ningun fotograma.
                if exportarAnimacion
                    [exportacion,infoExportacion] = exportar_animacion( ...
                        "iniciar",[],graficos,cfg, ...
                        archivoAnimacion,opcionesExportacion);

                    exportacionIniciada = true;
                end
            end

            graficos = actualizar_graficos( ...
                graficos, ...
                k, ...
                estadoRobot, ...
                trayectoriaEjecutada(1:k+1,:), ...
                caminoGlobal, ...
                obstaculosDinamicos, ...
                vistaPlanificador, ...
                textoEstado);

            % Al ejecutarse dentro del bucle de un archivo .mlx con la
            % salida configurada como Inline, el Live Editor conserva los
            % estados y muestra Play, deslizador y velocidad al finalizar.
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
        end %[output:group:0f04cabb]
%%
%[text] ## Condiciones configuradas de terminacion
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
registroTiempos.inicializacion = tiempoConstruccionPRM;
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
resultado.contadores.numeroConsultasPRM = ...
    numeroConsultasPRM;
resultado.contadores.numeroConstruccionesPRM = ...
    roadmapPRM.numeroConstrucciones;
resultado.contadores.numeroExpansionesPRM = ...
    roadmapPRM.numeroExpansiones;
resultado.contadores.numeroNodosRoadmapFinal = ...
    roadmapPRM.numeroNodos;
resultado.contadores.numeroAristasRoadmapFinal = ...
    roadmapPRM.numeroAristas;
resultado.contadores.maximoFallosConsultaConsecutivosPRM = ...
    maximoFallosConsultaConsecutivos;

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
resultado.historiales.motivoPRM = ...
    motivoPRM(1:pasosEjecutados);
resultado.historiales.motivoExpansionPRM = ...
    motivoExpansionPRM(1:pasosEjecutados);
resultado.historiales.expansionPRMEjecutada = ...
    expansionPRMEjecutada(1:pasosEjecutados);
resultado.historiales.numeroAristasBloqueadasPRM = ...
    numeroAristasBloqueadasPRM(1:pasosEjecutados);
resultado.historiales.fraccionAristasBloqueadasPRM = ...
    fraccionAristasBloqueadasPRM(1:pasosEjecutados);
resultado.historiales.numeroConexionesInicioPRM = ...
    numeroConexionesInicioPRM(1:pasosEjecutados);
resultado.historiales.numeroNodosRoadmapPRM = ...
    numeroNodosRoadmapPRM(1:pasosEjecutados);
resultado.historiales.numeroAristasRoadmapPRM = ...
    numeroAristasRoadmapPRM(1:pasosEjecutados);
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

resultado.metricas.prm = struct();
resultado.metricas.prm.tiempoConstruccionInicial_s = ...
    tiempoConstruccionPRM;
resultado.metricas.prm.numeroNodosFinal = ...
    roadmapPRM.numeroNodos;
resultado.metricas.prm.numeroAristasFinal = ...
    roadmapPRM.numeroAristas;
resultado.metricas.prm.numeroConstrucciones = ...
    roadmapPRM.numeroConstrucciones;
resultado.metricas.prm.numeroExpansiones = ...
    roadmapPRM.numeroExpansiones;
resultado.metricas.prm.roadmapPersistente = true;
resultado.metricas.prm.dinamicosEnConsultaGlobal = false;

resultado.detalles.construccionInicialPRM = ...
    infoConstruccionPRM;
resultado.detalles.ultimaConsultaPRM = ...
    infoConsultaPRM;
resultado.detalles.ultimaExpansionPRM = ...
    infoExpansionPRM;
resultado.detalles.roadmapPRMFinal = ...
    roadmapPRM;
resultado.detalles.ultimoControlMPC = infoMPC;
resultado.detalles.ultimoSeguimiento = infoSeguimiento;
resultado.detalles.ultimaColisionEvaluada = infoColision;
resultado.detalles.controlAnteriorFinal = controlAnterior;

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

    nombreBase = "prm_mpc_"+string(escenario.id)+ ...
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
disp(" "); %[output:553528d1]
disp("Resumen de la ejecucion PRM + MPC"); %[output:114182f1]
disp(filaResultado); %[output:35a74053]

disp("Construcciones de roadmap PRM: "+ ... %[output:group:54a46443] %[output:7c924386]
    string(roadmapPRM.numeroConstrucciones)); %[output:group:54a46443] %[output:7c924386]
disp("Expansiones de roadmap PRM: "+ ... %[output:group:3d63be1f] %[output:2bd131ab]
    string(roadmapPRM.numeroExpansiones)); %[output:group:3d63be1f] %[output:2bd131ab]
disp("Consultas PRM realizadas: "+ ... %[output:group:5fe500fd] %[output:2ebd897d]
    string(numeroConsultasPRM)); %[output:group:5fe500fd] %[output:2ebd897d]
disp("Episodios sin candidato factible MPC: "+ ... %[output:group:0b37a5cb] %[output:36ccdca0]
    string(numeroEpisodiosSinCandidatoFactible)); %[output:group:0b37a5cb] %[output:36ccdca0]
disp("Pasos con riesgo predicho por MPC: "+ ... %[output:group:898bf2f8] %[output:27569d34]
    string(nnz(riesgoPredichoPorPaso(1:pasosEjecutados)))); %[output:group:898bf2f8] %[output:27569d34]
disp("Pasos con colision fisica predicha por MPC: "+ ... %[output:group:126e9f1c] %[output:7f38c900]
    string(nnz(colisionPredichaPorPaso(1:pasosEjecutados)))); %[output:group:126e9f1c] %[output:7f38c900]

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
%[output:67459425]
%   data: {"dataType":"text","outputData":{"text":"Raiz del proyecto: \/MATLAB Drive\/TFM\n","truncated":false}}
%---
%[output:7aeddb6b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAkgAAAG2CAIAAAALDO4MAAAAB3RJTUUH6ggFExAW3NOTngAAIABJREFUeJzs3X1UG\/edKPwvGMW2JGzHtURiDIy03psWG9tQ49JlY418bLexT6+93NKGHMeIbC7FT+02roGt85xUUnM2LxjapE5q6t1jSa7XZEsPG9\/ekC7RiQR5uOUE18QGc5peL\/rxmoAIpUaMnQjD88fPGg+SEHpDb3w\/xycZzYxGX43EfPV7naS5uTlACCGEEkVytANACCGEwgkTG0IIoYSCiQ0hhFBCwcSGEEIooWBiQwghlFAwsSGEEEoomNgQQgglFExsCCGEEgomNoQQQgkFExtCCKGEgokNIYRQQsHEhhBCKKFgYkMIIZRQMLFFTmVlpdIbs9lMdzCbzV532L59e3d3N92nvr7e84kAMDY2plKp6PqnnnrK4XCEHqfwdQGgu7t7+\/btdFNlZaU\/74hyOBxPPfWUjx2E6Bv0sYPwdVUq1djYmOcmGh5lNptDPCHRQk+426fgz56eJ0GI\/6r4Psle0U\/H6\/mkMXh9UfrFFn5YY2Nj3\/rWtxZ9X2EUyrv2E\/89d3sJ4d9sfX29cJPwT97t1An\/avz5DiAhTGzRV15e7vuPbWpq6siRI57f7N\/\/\/vf88o0bNwYHB8Mb2NTUVHt7O\/+wvb19amrKnycK35HZbN62bVtHR4fbDkEnm7Gxsc7OTro8ODjY1NTkY+f6+vry8vIgXgUtne7u7n379g0MDEQ7kDDT6XRu33MAqK+vr6mp4R\/W1NTwuc1sNgu\/nE1NTXxuczgc5eXl\/NEWugKghWBii7SioqI+lxs3bhQUFADAhQsX+Kt8amrqlStX+H06OjoyMjKmpqZMJpPboTo7O\/mfwMIk5wNf6vLzd2tbWxsNzOFwtLW1BfqOxsbGXnzxRQAoKCi4ceMG3aeoqAgAOjo6Ll265E8MbmgKz8jIePrpp4URLmc5OTnXr1+\/fv16Tk4OANTW1vb19dXW1kY7rhgil8tbW1v7+vr27t0b9oPT4qDnb6yxsbGGhgYAqK6u7uvrq66uBoCGhoaxsTGHw3HhwgVw\/fmcP38eAN577z2avTo6Ojo6OuilgP5Neb0CoIVgYosmqVS6e\/duABgeHuY4zus+crk8Pz\/fbWVqauqmTZsmJydHR0fBVYihK8MV26ZNm1JTU\/nAbDbbzZs3F30Jt3fU1NREk9Brr70mlUrpPjqdrqCgoLq6uqKiIojAaArPz8\/\/9re\/nZqaevPmTZvN5nXPyspK+mO5o6Nj27Zt9JeysM7WR30deNSguu0prJgV\/lDgawXNZjP\/QvSl+RpUvrTqdWc\/QxJWTy1aFcnXhm3fvv3GjRtuh3V7L8LqMuHp8hGY\/8xm86FDh6ampqampg4dOsSfCq\/1dfyr\/9u\/\/RtdoPv7\/hCFh+LrP71WRXrdU3g+Ozo6+BPuWe\/N77xv377BwUH6JyPcxP8Ioz\/mioqKMjIy6J8t\/wdVWloKAAUFBTR70b9o+iXft29fTk6OVCp95plnAGBkZAR\/w\/kJE1s08T\/o0tPTxWLxQvvQmrfNmzfzK9etW6dWq\/mqQvr3s2XLlq985Svhim3Tpk0FBQWDg4P0OkjrIQsKCnwnNuE7mpubo4W8\/Px8uVzO7yOVSi9fvhxcVuPPxje\/+U2FQrFlyxa3+tJFn15cXCyssxXW\/7jtefDgQWHNknDP+vp6enXmt5aXlwvzwdTUVHl5Of9C586d2717N\/+L3q206rZzU1OT13pat5BoYvCn5C2sDXN7LQDo7u4+cuSI8L3w1WVup6upqUlYqxZG\/E8QPgC3D+WFF16gYezevZvjOB8fotuhBgcHi4uLPRMS\/Yngtue+ffuE1X1TU1NPPfUUf8IHBwefe+45r6ll7dq1V65cefPNN\/15s3z28urWrVsLbfLx8xe5wcQWaU1NTfyPRJo5AOCZZ57hCzT0guW2D\/+jj\/f444+npqbSijj6x7B79263H4w8vo2avxyXl5d7bc0WeuKJJ8D145G+BF3j5zuSSCR0B2FKDhH\/E3jbtm186ZDW7XjuXFtbSyt\/aEVoRUUFLUHydaf0lHr9IUz35KuFhTVFfPLmj0Nf5dy5c8LLIt3K1yQnJSV1dHTwVbVuNah8VS09FK2JcguppqZGGDwNSViJ7ZVntG5fJJPJNDU1RevK+PDox+12Eq5cubLQF8x\/e\/fupcehh718+XJHR0dTU1NGRkZHRwd\/xvhKOYrf6vtD7O7ufu+998BV9UcP5bUhlj\/D58+f59\/41NTUyy+\/7ONz8Vo9kJOT09bWRiuB3dDT6Pmz9datW6Ojo1NTU+vWrUtLS3Pb5HA4RkZGwOMPh6+hQYvCxBZl9C\/Wd73\/+fPnW1tbhYUeAPibv\/mbLVu23Lx5s6enp62tLTU1tbCwMLyxbd++PSMjo7Ozs6enp7OzMyMjY\/v27Ys+y593FDS+HpKejcLCwtTUVL5YuaiKigra+ETroHx0PKGXJFoXBAB79+7t6+ujjVh8cqUXO3BVMbmVHb\/5zW8CgFgsTk9P52Pmk7FQamrq6dOn6S+bI0eOCFMLj7\/Y8T8jaL8DHzWxFI2Wr\/ICgOrq6oyMDH4H2iBXUVFRWVkp7ObDt6ryJyEnJ+fYsWM+Xis49J0ODg4WFBTwv43cTmZJSQn\/\/ffxIdJswf8K5NvVPKsH6BepqKiIflH56j6388n\/4qTftLC\/d7REUqIdwLJTVFTku1U\/NTX10qVLOTk5fA3S73\/\/e888IZFINm7c2NHRYTAYbt686fnTT4hel0FQ73T+\/PlFc49EIklPT+\/o6DAajfS6wxfCAnpHPmpXAsLXQzY1NbnlJK+nyJNntZtXC\/1kpjx\/htPsJawcS01Ndfs4Qiy2chw3PDwc9NN9fD3c+uZ5CmOB26tAvx4+PsSFSkhuvH6+aWlpbqnL80MMFD0+rULkq2ToevpytBAm\/M26efNmqVS6ceNG8Dgzvv\/GkRCW2GJXRUUF\/eHZ1NTktcKQlgnee++9qakpt3assBCLxbR4QbPI7t27fV8v3PClE2HvTXA1bwTRE4HWQXnd5PYSXjkcjpdffpn+oqf1Wm6VcsLIvV5ZKOHViq4JMev4bnSh+JIfrWTj8T0hFyK8gHpudeu2yldFwmInIYzo+RT2m6WExSw+A\/n+ED0\/Gq+8vjVa2gvrO3M3Ojo6OTnpNV\/yXyE\/f0sh3zCxxTTagRA82m+obdu28XVKNMmFHV8DE1xVJ62jc2t1p8N9FsrWC+FrxtyugLTBZtEBbSC4dtB6LeF4OE\/0+sK39AiHSdBUMTg4yHc94Nuigq4N5pvKLl26RCsD3S5w\/LWYb1D0MVZaKC0tbd26dcLO4rStji7TSy246txoVz0fJ+HcuXPBvUEf6KvwjV6+h6X7\/hD5j4Z+GfhOpJ6\/ouiLNjU10d43fOf7LVu2KBSKcL01+hfKx0N7YNGCF9\/1iX4ufI0xzXn8b9bu7m7+m79x40ZhsQ\/5gFWRMU0qlZ4+fZpWvJhMJrcaPzoSgO9M4c8B6YAn\/wOgf34dHR3B\/cHL5fIXXniBDjV1i7CgoODIkSP+H4q\/5u7evVv4581H2NbWttAB6asXFRXR01VTU7No776ioqKGhobBwcFDhw4JYy4oKJBKpceOHaupqXGrET127FhOTk5wo2jdzg9t+3E7VGlp6XvvvUfrhPmVwm5HXsnl8pKSEs9oKf7sea2N9HoSwoV2ksrIyDCZTAUFBW4xFBUV5eTkeJbC+e+81w8xJydn3759tPcmv1XYvsg7cuRIW1ub24sKGzvDgj\/5wnj49sJnnnmG\/sLjPxe+OZN+0zo6Ovgz7\/VdoIVgiS3W8S32\/K9LIfrDc+nqKPiCQtC\/Fvfu3Sus4KKqq6svX74c0AHpr13PUhFf4em1GwXfF4PiS8AAUFRUREt7Xp9I+x0Iq7mKior4mCsqKoRdBGkHv+AGMNCnX758WRiY1zbLnJyc9957T9jvw5+GUhot388lNTX1\/Pnz\/EGkUulrr73GP6yurqadLWnVrlwub2xs5LcWFRXxxwmFWycUsVh8+fJlt1Pto9XW94dYW1tL3wJFO1h61tbSMSfCt1NQUNDe3u67XjcIwpMPAMLhm3v37hWGKnzXUqn0\/Pnz\/Nvk293DG1sCS5qbm4t2DAg9QLvM+HnJjne0HwQAhHjZoh1Aonv5o+9l3759OOMJijqsikQojgknFcRecwhRWBUZGOHURMImEH79QvPuILQUhB0yX3jhhbD3jEUoHmGJLQD19fUNDQ0dHR1yudxsNh8\/fryxsZEu8+vr6+ufe+658+fPY\/8ltKhA+\/J4om2B4YoHocSAJTZ\/0U63fI+mgoKC9PT0Gzdu0I7C\/PqioqKpqSkfk0EolcrIBR3zPM8GnVdiOTSwuYn3LwZN0mFpYIv3UxFGeCqCgyU2f9FuVPxDvgqILvCjjsRicWpq6lJ0r0IIIeQPTGxBampqSk9PLygo4DhuzZo1fKM93z\/eh6SkpKUPMD4oFAo8GxSeCh6eCh49Fdh3PVCY2IJRX19\/7ty5S5cuSaVSP28kIaxSCOPUBgkAzwYPTwUPTwVPoVDQqwed7hX5AxNbwPisxlc23r59e3R0lD6k86t6TvjGfymVSqXNZsOfYJTNZsNLGIWngoengoeXi+Bg55HAVFZWNjQ0COcyoLPT8rOpchw3NTUV9jvIIIQQ8hMmtgDU19d3dnbSLv78SnonJ35q2qamptTUVPy9iRBC0YJVkf6idyJ2m4KWzvy2d+\/eW7du0fUZGRmNjY04iA0hhKIFE5u\/fI+EraioCHoOXIQQQmGEVZEIIYQSCiY2hBBCCQUTG0IIoYSCiQ0hhFBCwcSGEEIooWBiQwghlFAwsSGEEEoomNgQQgglFExsCCGEEgomNoQQQgkFExtCCKGEgokNIYTCw+FwsCyb5JKVlfXJJ58Id\/jjH\/8okUj4HUpLS92O8Mknn2RlZXluevXVV5OSkl599dUlfw8JARMbQgiFwR\/\/+Me0tDThVOkDAwObN2\/+4x\/\/SB\/+7ne\/27lzJ8dx\/A4XL150S35Xr14dGBgAgN\/+9rf8E1GgMLEhhFCoHA7HqVOnOI47evTo3Nzc3NzcyMhIZmYmx3G\/+MUvAOCTTz45fvw4ALzyyivCHQYGBi5evMgfpK6uTiwWHzp0iOM4s9kczbcUzzCxIYRQqD7++OPOzk6VSvXmm2\/SNY8++mhTU9OhQ4foGloUO3r06D\/90z8JdxCLxb\/85S9poY0eJD8\/\/7nnnhOLxe+++2603k68w8SGEEKhGhkZ4TjuiSeeEN5k+Ktf\/erbb79N1\/T29gJAdna28FmPPfZYfn4+\/9BsNtOD7Ny5Mz8\/v7OzM1LhJxpMbAghFB40ewHA7373Ox9dSLxyOBzvvvuuWCzeu3evVCqlFZtLHG\/CwjtoI4RQqDZu3CgWi\/v7+x0Oh7DQxqNlNT7zUbTuccOGDfwyx3E7d+6MTMwJDEtsCCEUIKvVbQWtVGxtbf3+978PAN\/61rf47iF0h507d2ZmZl68eJHvsv\/JJ58UFRVxHMey7KOPPvqLX\/wCi2jhgokNIYQCQQio1W7rpFIp7dB48eJFvhJy48aNAwMDNG89+uijb7zxBgD8+Mc\/Fm7NzMx85ZVXPvnkE6vVKhaLr169OufyyiuvuL0K\/1wKh7UtBBMbQggFghAAL4W2r371q6OjoyqVil+TmZk5MjJiMpnow29961tXr14Vi8X8DkePHu3v73\/00Udpn8n8\/PzHHnuM37p371664E8THRJKmpubi3YMy4tSqbTZbHjaKZvNplAooh1FTMBTwYv1U6HXg04HGg0YDEv9Uni5CA6W2BBCKBC0rGY0RjcK5AMmNoQQ8pvV+qAS0qM2EsUITGwIIeQ3wVSQ4Go8Q7EGE1swzGbzU0895XA4+DWVlZVKpVKpVKpUqrGxsSjGhhBaQsJSGtZGxipMbAEzm83l5eXCNfX19Z2dnR0dHX19fSUlJc8995ww5yGE4o9eD0lJnv+IlehBpwaLHnQA4HUf0OujHPyyhzOPBGBsbKy4uHhycnLfvn1TU1P8+lu3buXn58vlcgAoLCx85513OI7zOvsAQig+aLWgUrmNVyPAqMFCgAEAK7BGKLWAmgHyYA+GAY0GtNpIRoo8YYktMCdPnrx+\/Xpubq5w5ebNmzs7O2kNZHt7e2pqqnCoCkIoLrEs2GzAsvwKE2hoVqMIMHoQ5DCGAYsFs1oswBJbAORy+eHDhz3XV1RUbN68uaCgAACKioouX77suY9SqeSXRSKRzWZbujjjyNDQULRDiBV4KnixdSouXIA33oCzZwHgPfiyCOb95f5fWG0DEaSnQ1ERHD8Oc3MQ7j9thUJBrx59fX3hPXICw8QWBpWVlSMjIzdu3JBKpfX19SqVqrGxkdZM8vgvpVKpdDqdMT3+NLLwVPDwVPBi61TU1cGJE6BW\/y250w7zAtsHFxVMOhgMwoJdeOEA7SBgVWSoxsbGOjs7n3nmGdqoduTIkfT09KampmjHhRAKH4YBhtHCvF4hDJBSMAIhS5fVUHAwsSGE0GIIAauVAWKAMrqCAfKg5widPRLFDExsoZLL5fn5+RcuXKBd\/C9dujQ8PFxUVBTtuBBC4eMxFpsB8qA\/JE5BEmMwsYVBbW3txo0bt23bplQqz50798Ybb7g1sCGE4ptn6mKYB8s4BUmMwcQWjIqKisuXLwtHqtXW1vb19fX19V2\/fj0nJyeKsSGEwoyQ+4mNYYC9f1cahmXAYrm\/g9WKtZExBXtFIhQhhBC1xw0qEwPLsoalv4dL1NACGR2mZmXA6lpPB7qVld2fGVmjiVaAyA2W2BCKHELI3bt3ox1FmH366afRDmGJEQI6HdhswDD9\/fM30Wyn082bHBlFG5bYEIqoRx55ZN26dZ7rnU6nSCSKfDyhm5ycjHYIS0yrndei5nUHQoCQRXZDkYKJDSGEfPKWrtzXYUqLJVgViRBCKKFgYkMIIZRQMLEhhJC\/sFd\/XMA2NoQSQV1dXV5eHl2+du3aqVOn6HJubu7zzz+\/YcMGt\/UIJTAssSEU96qqqvisBgB5eXl1dXUwP6vR9VVVVdEJMeFkZUU7ArQwTGwIxb3s7GwAaG5uVqvVzc3NAJCZmZmbm7t3794NGzaMj4\/\/6Ec\/ouvpngglNqyKRCjuTUxMMAzDsuz169fPnDlz5swZuv7IkSMA8OGHH3Z1dXV1dfHrUdCwjS0uYIkNobjX09PjdDrFYvHp06ctFgs\/u9X69eudTueOHTssFotwPUKJDRMbQnHPYDDU1tZyHEcfMgzT2NiYm5sLACKRaOPGjfx6zG3hggOyYxkmNoQSQUtLy8GDB\/k2trVr1+7YsWNiYgIArl27plarL1686HQ65XL5\/v37ox0sQksLExtC8S03N7exsbGlpaWsrAwAzGbz+Pg43SScnnh4eNjpdEYnRIQiCzuPIBTfurq6BgYG8vLyjh49evToUbpyfHz8o48+AoBdu3bl5eVZXHcOI4S0tLRELVaEIgJLbAjFvVOnTl27do1\/OD4+\/tJLL9GekP\/yL\/\/Ct70RQmipDgUNe0XGBSyxIZQIFppSpKWlBYtoaLnBEhtCCAUMe0XGMkxsCCGEEgpWRSKEkL\/4NjZCiNVq7e\/vJ4QAAMMwAKBSqViWjVJo6AFMbAghFDC1Wg1AMiUSAMgQi9\/nuIHpaQBgGEaj0ZSWljJYWRk9mNgQQmhxhBCTyQSgpQ+vqLIKZTvd9hnkuAZCdC5arTbiYSIATGwIRRghZNWqVZ7rk5KS5ubmIh9P6O7evRvtEJYcIUStVs\/a7XxiK5TJPHfLEIurs7Ors7Nrent1Op3RaDQYDFg5GXmY2IJhNpsvXLhw\/vx5qVRK19TX19fU1NDl8+fP7927N3rRoRhFJ+BfaKvT6RSJRBEMJ5wSu9qNEKJQKDIlkrM7v32o1a+nVGdnlzDM8c5OtVptsVgwt0UYJraAmc3m8vLygoIC4Zpz585duXIlJyenu7v7+9\/\/flpaWk5OThSDRLGJnwHEk81mUygUkQwG+YPPateeeGKQE\/v\/xAyx+IpKdeLqVcxtkYeJLQBjY2PFxcWTk5P79u2bmpqiKx0Ox4ULF44dO0YzWU5OTltbW1TDRAiFB62BpFlNuD5TYvfzCGd37hyYnsbcFmE4ji0wJ0+evH79Or0hCGWz2YaGhgoLC6MYFUJoKej1+lm7\/YpKFcpBrqhUhTKZXq8PV1RoUZjYAiCXyw8fPuy5fu3ataOjo9u3b1cqldu3b+\/u7o58bAih8CKEGI3G6uzsDHEANZBeVWdnW61Wo9EYjrjQ4rAqMgwGBgYuXLjQ3t4ulUrNZvORI0cuXbrk1samVCr5ZZFIZLPZIh5mLBoaGop2CLECTwUvRk7F008\/nblmzd+lpQ24un1em1wlEtkAAFZMDATSFzQjNfVraWl6vV4VeOFPoVDQq0dfX1+gz122MLGFQWZm5muvvUZ7SBYUFGzZsqW9vd0tsfFfSqVS6XQ6sZsAD08FD08FL+qnghDS3t5enZ2dKRib8ZfnTju\/IwOA5HRF7u9+BwD3hoYmT550e+6an\/xElJPjtunpzMzjnZ39\/f2BtrTZbLY4HQcSRVgVGaq0tDQAGB0djXYgCKGwoRNl8YPVZjhusrd31nWnVn50w4pNmx7+1a9EW7fyTxR\/97uir3zF84BPZmVlSiQmk2kJg0YuWGILlVwuP3jw4Msvv0yHtXV0dNy8efP06dMRC6CsrIzE7U2ighu8xbIszumAllRra2umRMIntrt2+2Rv76zzC\/rwTmfnZ8U1K3fvljz7bPL69auLipw9PQAg2rp15Z49kOL9ulook8Xvn2p8wcQWBhUVFQCwbds2AEhNTfVsYFtSVqt1cnLS62QWsS+I6TYmJycBABMbWlJWq1XYZ0SalXXXbv\/M9XBmenro3Xelvb1pWVnS\/\/7fkx9+mK6X\/OM\/Jq9ZM\/PxxymPPeZ5zEKZ7LjVutSRI8DEFpyKigqazHysiaR169bF6dQPQZTY\/vSnPy1RMAjxCCHfmT9p1oadO\/\/PxBr+4cz09GRvr\/j6dck3v5m0erVo69aVjz++YtMmZ3f3rN3uNbGhiME2NoQQCswqmWzTgQPrsrPvP05KEm3f\/tDXvz47MXGnqcn3c7E2MgIwsSGEkBeDHLfQpkyJPUUsXpedvb6wMOmhh+Y4LuVv\/iZp9erk9evXaLUr9+wBb\/1KqDitXIkvWBWJEIot\/\/Ef\/7F+\/folfQnfE1IDAMuyny\/WHibaulW0YwcAzPz5z8neJvt3MzA9jVktMjCxIYRiy49\/\/GOnq2P9EtFoNIuOJ2sg5OzOeXdcI4P3L5grWfZLhvtPn52Y+PyDD2ivSEp67NjKPXs8h7i12+3Mli0hx44Wh4kNIRRzGIZZusJNR0fHovuoVCqj0TjIcb7n0\/I6QNurQY5rt9t1OA9yRGBiQ\/6qq6vLy8ujy9euXTt16hRdrqqqOnDgAL8bIaSsrCwK8SEUPhqNpqysrKa3V1hou\/uf\/wlQDADyDxs\/K\/7NQs91nDvnOHfObeXA9DQABDGlFgoCdh5BfqmqquKzGgDk5eXV1dXR5Wy+bxhCCUSj0TSErwdjTW8vy7J455rIwMSG\/EKzV3Nzs1qtbm5uBoDMzMzc3Nzc3FypVMpx3Msvv6xWq9VqNRbXUGIwGAwAcOLq1dAP9VZ\/f7vdjrMKRAwmNuSXiYkJAGBZdv\/+\/WfOnFGr1cXFxV1dXTKZTCwWi8Xi06dPWywWei1AKDHodLoGQtrt928rOsgt3vXR0yDH1fT2+tNdBYULJjbkl56eHqfT6ZnA0tPThVOHMAyDuQ0lDK1Wy7LsiatX+dwWqEGOO97ZmSyTYXEtkjCxIb8YDIba2lrONWSVYZjGxsbc3NwNGzaIRCJCiFqtfvnllzmOk8vl+\/fvj260CIWLxWJR5ue75bZMyZg\/z6VZbVgiMRgMOIItkjCxIX+1tLQcPHiQb2Nbu3btjh07aLUkbVez2+0cx4lEovT09GgHi1DYGAwGZX7+odbWgMptgxx3qLV1WCKxWCxYCRlhmNjQ4nJzcxsbG1taWmgCM5vN4+PjdFNdXZ3FYqE9JHfs2LF27Vqn0zk8PBzNcBEKK4ZhLBaLTqcbmParjW2Q405cvZrb3Jwsk1ksFiyrRR6OY0OL6+rqGhgYyMvLO3r06NGjR+nK8fHxjz76CABycnLy8vIsFgtd393d3dLSErVYEVoaWq3WagU6zVZNb+\/AdG+hTJYpkWSIxXRWyYHp6Xa7nf5jGEan02G7WrRgYkN+OXXqlHCA9vj4+EsvvdTV1dXV1QUAJSUltAuJcOA2QolqT3F+jfGXnusZhmEPHnxWpdJoNBEPCj2AiQ35a6GMZTAYsCckWlZKSzUGg4a4gGsOMKx1jBGY2BBCKBiYyWIWdh5BCCG\/4C1C4wUmNoQQQgkFExtCCKGEgm1siWBycpIsm1qSycnJaIeAljtsWYtxWGKLewzDrFu3LtpRRM4jjzwS7RAQQjENS2xxjx8ZHY9sNptCoYh2FAj5ZdlUi8Q9LLEhhBBKKJjYEEIoMNjGFuMwsQXDbDY\/9dRTDofDbf3Y2JhKpTKbzVGJyg0hxGg0lpWVqdVqhUKRlJSkUCgUCoVardbr9cunswlCaLnBxBYws9lcXl7udVNNTc3g4GCE4\/FECNHr9QqFoqys7P3GxkcJ+Y5YXJ2d\/R2x+DtisfPmTZ1OR5OcXq+PdrAIxQf8KRhHsPNIAMbGxoqLiycnJ\/ft2zc1NeW21Ww2j4yMZGRkRCU2nl6v1+l0mRJJdXZ2oUxWKPN+o412u\/2t\/n6dTmc0Gg0GA94vCiGUMLDEFpiTJ09ev349NzfXbf3Y2NiFCxeef\/75qERF0YKaTqerzs6+9sQTNLEttHOhTHZ2586uAwdm7Xa1Wm00GiMYKUJxDBvYYh+W2AIgl8sPHz7sdVNTU9Pu3btDzlXBAAAgAElEQVTlcnmEQ+IRQsrKyvo6O9\/Iz38yK8vPZ2WIxdeeeOLE1av0DqJ4rw2EUALAxBYG3d3dXV1ddXV1HMcttI9SqeSXRSKRzWYLYwDDw8M\/\/vGPB7u7zxUUPLp69cDduwE9vWrr1jsA5eXlq1ev3rVrVxgDW9TQ0FAkXy6W4angpaSkAIDT6Vy643McF+jf4PAwiET3l8P657sIhUJBrx59fX2Re9U4h4ktVA6H44033jh+\/LhUKvWR2PgvpVKpdDqd4R2VfPHixcGPPjq7c+fXHn44uCP8686dh6ann3zySZvNFuE7ceAAbR6eCmpmZgYARHwaWYLji8XiQM92fz\/wqTaSH5TNZpubm4vc6yUEbGMLlc1m6+joOHTokFKpLCgoGBwcLC8vr6+vj1gAhBCdTvdkVpaPFjV\/XFGpMiUSWieJEELxCxNbqHJycq5fv97X19fX19fR0ZGRkXH+\/PmKioqIBVBWVkb7QIZ+qOrsbKvVarVaQz8UQokKO4\/EPkxs8Y0QYrVaw5LVAIAW+0wmU1iOhhBCUYFtbMGoqKjwWiaTy+Wtra2RjMRkMmVKJG7dINf9\/OcrNm0Srrk3NDR58qTnDrMTE46zZ509PfymEoY5bjRqtVq85z1CKE5hiS2+Wa3WDLF40d1WbNr08K9+Jdq6FQDW\/OQnfNpLXr9eeuIEXU\/RHIm1kQi5oTOPMAAMATACGAFI9KJBPmGJLb75qIf8\/P33HefOAcDK3bslzz6bvH796qIiAFiRng4zM3fefvveJ59Inn02afXq5PXrhU\/MlEj6+\/sjEDxCcYQBMABoAIAAlM1fy0YnJLQQTGxxjE5kvGhnyM\/b2kRbtqzcsyf54YedPT1\/+d736PqVu3cDwOxnn33e1ibcP0MsximSEZpHD6zO23oCoAbQABgiGg7yDasiE9Dc7CwAOB0OR3\/\/+NWr41ev3u7qmnM6k8RivtZReuyY9MSJ2c8+E7a9UZkSCSY2hB7QA+juLxIAKwNgANABMK4djADqiEeFFoaJLQHdu3MHAO7a7eOdnQ5CHITMcBzMH+PpOHfus+JiAPhSQ4P4u9+NTqAIxT7r\/axGAPQACgATC6AB0ALYACyC3fBWGTEDE1scox0XBxeY7iQ5JWWVTCZlmA35+Wu\/+tWkhx6a4zjRli1famjgO5LM\/uUvkJKS8thjwicOTE8vfewIxQPyoDnNyvDFNgFWkNu8bEbRgYktAaVIJACwSiZ7RKXasHPnwwcPriooAICZP\/\/ZefPm7O3byevXr3z8cdHWrSvS0wFg1m4XPn2Q4\/AuNggBAFhdXR9Z6NfcX+c+EIalXUoAAMC45BEhf2DnkfjGMEy73e51On\/xgQPiAwf4h7MTE59\/8IGzp+fe8HDy+vUr9+xZuWcPAMzdueO8eZPfbZDjBqanVSpVBIJHKDh1dXV5eXl0+dq1a6dOnaLLZWVlJSUldJLJ5ubmM2fOhPQyBICfq0AL4GOEaqkrpbUKkhyKHiyxxTetVtvgR0ePe0NDf\/ne9+hA7Ns\/\/amzu5uun7tzZ\/pf\/1XYK7LdbmcYBktsKGZVVVXxWQ0A8vLy6urqAGD\/\/v3f\/va3+amTDxw4UFVVFdIrEQArAACwi3Xo57eSkF4QhQuW2OKbRqMpKyt7q79fWGjz7Ojo5vZPf7rQpgZCmC1bwhYfQuGWnZ0NrgJZVVXVgQMHMjMzc3Nzt2\/fLhaL6Y0JaZHukUceCemV+M4gWgDXAO1F+LMPWnqY2OIey7I1nZ3+31zUh3a7vd1ut2i1oR8KoSUyMTFBKxWuX79+5swZvr6xq6tLWPfodDp7BHPFBYMAAAATyPhrAqC\/nwhRFGFVZNzTarUD09M1vb0hHmeQ42p6e1mWxXpIFMt6enqcTqdYLD59+rTFYjEY5g2Nzs3NbWxszMnJaWhocNsUGCJIbPMt8htSh2Paog8TW9xjWVan09X09rbP79wYqAZChiUSLRbXUGwzGAy1tbX8TX0ZhmlsbMzNzaUPu7q6iouLGxoaSkpKQk1sFBv4c62uxjkUJZjYEoFWq2VZ9sTVq0Hntpre3preXo1Gg8U1FPtaWloOHjyoVqubm5sBYO3atTt27DAYDBaLhXYYGR4edjqdcrl8\/\/79Qb4G3wfS1UGYEGAANABMK4B+sZ79eOunqMLEliAsFosyPz+I3EZrIGt6e3U6HRbXUIyjNY0tLS30Vu9ms3l8fJxu6u3tBYBdu3bxHUk4jrMHXY1B3B8aCNjojMdGAB1AGUASgMI10z+KJdh5JHFYLBa1Wn3Iaq3Ozvbz1qODHHe8s3NYIsGshuJCV1fXwMBAXl7e0aNHjx49SleOj49\/9NFHMpmMZdkNGzb87Gc\/o+sHBga6urqCfCXiWmDuzxXJLLRbmdcNKJqwxJZQLBYLbW\/Le\/dd391J2u32E1ev5jY3D0skFosFsxqKF6dOnbp27Rr\/cHx8\/KWXXurq6mppaXn99df5tjfhwO2QmHCurPiDJbZEo9VqS0tLTSYTzXD8\/bUzJRIAoBWVdEw3wzBYUEPxaKGM1dLS0tLSEuYX03lbyQiWSZhfEIUOE1sCYhiGT28AYLVa+TtiMwzDMIxOo1GpVNhPBKEFEe\/rrACsARiNYK3VW\/9+ZglCQn7DxJawaHoDACyTIRQ6AqAGIAAWZn7aYgF0WF0ZW7CNDSGEFsOAmnGN2GY8tuJPxxiDiQ0hhDww85dtgeyPog0TG0II+cQE\/pQwTN2KgodtbAgh5IEIlhkghBBCAEiAkyKj6MASG0IIeSAPFvVGvVqtBlADlAGo1WoF380YxSZMbMEwm81PPfWUw+Hg11RWViqVSqVSqVKpxsbGohgbQiiM9KDXgS7lUc26b1nWl9jWl9huP8So1WrMbbEME1vAzGZzeXm5cE1lZeXIyMiNGzf6+vpKSkqee+45Yc5DCMUfBgCAANGBbvNXdZtztFseZVekMitSmdX7LVKGVat93pyGiUSMaCHYxhaAsbGx4uLiycnJffv2TU1N8Ss7OztfeOEFqVQKAEVFRe+8847NZsvJyYlqsMuR0WhsbW1dfL+YxHGcWCyOzGvxYxyRbyYwrU5lNn9VCwCpAJunyEAq88UXsKnE8qeXk4xGo0ajiXaMyAtMbIE5efLk4cOH6+vr29ra6Bq5XC68mI6Ojv71r3+NUnTLXWtr61tvvbVq1apoBxKMFStW3Lt3LwIvdPfu3UceeQQT2yJYACNYwbo6lfnkIXj0CwAAxyfWT\/+3ftV\/06Sy2tVfYvr7+6McJFoAJrYAyOXyw4cP+97HZDLt2rXLs7imVCr5ZZFIZLMtOi5mWRgaGgrj0TiOk0gkX\/7yl8N4zIiZmZlJSYnE3+Pw8DAAxPI3kJ4Hp9O5dMfnOG6RM5ADIAI5yJOThu9O2noA5AB\/\/kiffHf4ixv\/bB8wztwevn379ryDiARPH\/Vj6Jt\/FAoFvXr09fWF54jLACa2cKqsrOzs7GxsbPTcxH8plUql0+lUKBSRDS12hfFUiMXie\/fuiUSixXeNSZGMPJa\/gTMzM7CUZ2NmZkYsFi9yBn4EcAqegCd+M1y26V7\/RCo7CCB5wvL5n03Tf9Q57QQAcnJy5h1EmIi\/FrZmNpvNNjc3F55jLRvYeSRs+Kwml8ujHQtCKGQa0ICGAebj\/12WkwMbN8K6jUzGfu2O4xYAYBgGpxGPWVhiCwOHw0H7Sb7zzju0CwlCKO6VAhjBAhbFlOIPOkX632mSAabGyZ\/+YGQYxmKxMAxDXDSgiXa46AFMbGGg0+kA4Pz585jVEEocLAADDGFsYDOlmqxTVkKICECn06lUKr1ebzQaBfuywkn\/rVYrq2EjHTBywcQWqu7u7vfee29qamrbtm38yvPnz+\/duzeKUSGEwoABIMAAoyVarUULDBiNRr1er9PpMiWSEoYplMkKZTIAyGyVwPSD56nL1IyeMRgMWF0ZFZjYglFRUVFRUUGXc3Jyrl+\/Ht14UOyoq6vLy8ujy9euXXO71\/P+\/ft\/+MMfikSihoYGg8EQjQBRIIhguQz0rF6n0xXKZK+rVDSfLaTrwIHjnZ1qtVqj0eAHHXnYeQShsKmqquKzGgDk5eXV1dUJdygpKYnYKGwUKjI\/sVkh44y4Ojv7ymJZDQAyxOIrKtUb+flGozGWO6AmKkxsCIVNdnY2ADQ3N6vV6ubmZgDIzMzMzc2lW6uqqhgvN6lEscp0\/\/+EIXThmemq\/1dW7P8BnszK6jpwgBCCuS3CMLEhFDYTExMAwLLs\/v37z5w5o1ari4uLu7q6AKCsrGzfvn0jIyPj4+PRDhP5gQDo6P9JGZTpQU9XS69+f9Gnzkrs\/HKGWExz2yJzS6KwwsSGUNj09PQ4nU6xWHz69GmLxcI3ruTm5h44cMDpdL799tvRjRD5hQCU3V+0grXP3nnowIdO2U0ASJ6WrWnVBXQwWi1ptVr1en2Y40QLwMSGUNgYDIba2lqO4+hDhmEaGxtzc3OPHDmyYcMGq9V669at6EaIFkcATABWukjKoKw6OztDLHbkv0m3i+xbxL3fCeiQhTJZdXa2TqcjhIQ3WOQVJjaEwqmlpeXgwYN8G9vatWuLioro9JUHDhz42c9+tmHDBpFIdPTo0aqqqmgHizwQAP39SkhgoAzKMiWSJ7OyAGBWbL+tohtgdW+xyL4loANXZ2dnSiRYaIsMTGwIhUdubm5jY2NLS0tZWRkAmM1mbE6LMwRADWAEAAAGiJZYwVqdnc1vd8pu3sm+PxPsmlbdQrntntj7rYafzMoSjulGSwcTG0Lh0dXVNTAwQEtjFouFFs7++te\/NjU10TKcWq3+0Y9+ND4+7nQ6L168eObMmWiHjAQIQJmrfz8DYAErWPniGu+h\/\/w6sPeX1\/TrvtTYuO7nP3c7kmjLli81Nn6psVF67JhwPc2RmNsiABMbQmFz6tSpa9eu8Q\/Hx8dfeukl2isSxTQCoL7frkazGjBgMpkyvA46tMD93EYA1LBi06aHf\/UrEN6LgLn\/\/5W7d4u\/+13hUzMlkvi9F24cwZlHEAont6lG3HR1dRUXBzAQCkUCAVDPK6vRzEQI+c4CA7G\/+J\/\/5yHr3wEAWAGevpf86\/Vzm6fh6v2t9+Zsk8XV637+8xWbNqU89pjwiYuO7EZhgSU2hNAyRrxnNQDw0YNx7pG7fEcSuLQCrACiB4WEL9Z0AcDsX\/4CALN2u\/CJGWKx1WoNR9zIF0xsCKFlzCTIajb3u4NmSiQLPc8pu8kPAHC7ZY3zxo2Vu3enbN48d+eO8+ZNtydij\/8IwMSGEEIAgc9U\/HmWlY7ahn5I+sNKuvILIBOZfas1GlixwtHY+Hlbm9uzcFq1CFh2bWxjY2PFxcWDg4OL7pmRkYHNvAglOLLgFoZhBqanF9wMAACO\/Dcfbv7lvFWZsxlvvpm8cuWnr7468pOfMN\/+tnDjIMdhYouAZZfYqEXvl2Y2m1988cWIxYPC5e7du1jV49vk5OS6deuiHUXMYFwLxH0Ly7KDi7WHrdiVNpdzJ+nV1fyah57OgNVJn\/3ylxNnz64TjIGjBqanRYCW3DJNbCghMQzzyCOPRDuKWIdZbR5+lFq\/+xaGYXSEnN250\/NJK\/fsWblnz4PHrwq27RVBCnzpBz\/40g9+AADO7u7bP\/0p3TLIce12uw5vPbr0ll1ik8vlfAVjd3f3kSNHpqamhDtkZGQ0Njbu3bsXb4Edd7RarVarjXYUQbLZbHhzkyjQuOY71gHM\/+6oVCoAaLfbfffRvzc0NMPcWknY+4\/ZBfdst9v5w6IltewSG8\/hcLz88sv79u2rra2NdiwIoehhXUOzrfPSEsuyLMu+RYgwsU2ePOl5gDXTOn75s4XHKTYQQo8ZUrTID8u3VyTHccPDw9\/85jejHQhCKKr4gprHHdNKS0sbCGmfPxbNzcp+1p8Jkd\/q72+32+O3RiG+LN8Sm1gsTk9Pj3YUCCEv7t69e\/fu3Qi9GCsotCkAbA+2aDQak8l0orPz2hNPeH1qMieTdi5+69FBjqvp7cXiWsQs38QmlUqfeeaZF198cdu2bXK5PNrhIIQe+PTTTz\/99NPIvZ4FIAkAAAiAAkDzoBhnMBgUCsWh1tYr89vGkjnZKqJe3bv4BGmDHHe8szNZJuNvPIuW2vJNbACQlpY2OTlZUFAgXEk7j2CqQyhafv3rX6elpS3pS3gZTGYDoH13CIAOwAigAVABwzIWi0WtVp+4epX2kEzmZCL7lpXESw3krMS90nKQ4xoIGZZIDAYDjmCLmOWb2GjnkWPHjlVUVEQ7FoTQA7t27YpCB1EGwAZQ5qqTJA9uN8oCa2NshJC\/2iXK6Xz\/DznIcYdaW5NlMq1Wi5WQkbR8ExvtPLJ58+ZoB4IQig0MgMV1E22jayWhWxgGGFhgHpJZiT15et6QAFpQq+ntZRjGYrFgWS3Clm+vyFA6j5jN5qeeesrhcPBr6uvrlUqlUqlUqVRjY97vn4sQigMMgAHABqADYN2nRQYGjGDUg\/5fZdX3J4oE4BcGpqfb7faa3t7c5ubfcJxOp7PZbJjVIm\/5JjapVHr69OnXX3890DxkNpvLy8vd1jQ0NHR0dPT19ZWUlDz33HPCnIcQij8MgBbAAmADmBP8swFrY0EHw3YJbWMjQP5xupo+iQA51NpKU5rFYsHO\/dGyfKsix8bGjh8\/Pjg46H\/nETqB8uTk5L59+\/j5ShwOx4ULF0pKSuhTioqK3nnnHZvNlpOTE4F3gVDsEM7SmcDFFIZhtFmujMWAlbX+LTlIW+ZYlrVoLdicFnXLN7EJ59by38mTJw8fPlxfX9\/muhuFW1udWCxOTU1tb2\/HxIYSmRWAADAALBBCTCaT1Wp1u4UmwzAsyzIMk2gFF+KahQsANKDRakDv6nLCAma1WLB8E1sQ5HL54cOHPdevWbOG750slUo3btwY2bgihBAS9onzR0dH+\/vnzT6L14VYRwRdBwGAAQVRZEokhTLZG\/n5GWIxXT3IcQPT0+3vvGO0241Go0ajSZD0RgQTlGhcw92s3nZjIhIP8mbZJTZanfjCCy\/4c9uaMN6PTalU8ssikchms\/nYOTZ9+OGHTz\/9dHiPmZKSMjMzI1zz61\/\/eteuXeF9lbgwNDQU7RD8MAzwNMAwgOjBmg9EH0jUr7rtmJGa+ncATyqVn9y5887IyD\/\/8z8bjcZXXnnFnw83dk+F8O2nA\/wEwAYwDNDuOiHvAaQCNAF8CAAA6QDvh\/qaCoWCXj36+vpCPdaysewS21K4ffv26OgorXt0OBwjIyOeowj4L6VSqXQ6nfE4j3t\/f7\/T6fzyl7+8atWqcB1zdnY2Ofl+D6a7d+\/+6U9\/SktLi8eTExax\/sYJwNP3u78TIO9LGllgldP5ClA4Rq9\/nmX1+qTMVau+9vDD\/1OhON7Z+eSTT+p0On+KbrF4KsiDt39\/YAADAACtAE7XPu0A7fOfkhRq0c1ms83NzYV0iOVnmSY2t26NXmVkZPhzKDps4NatW7QIyHHc1NRUYWFhqCHGqnXr1oUxsTmdTpHo\/o\/\/yM0NiIKgdw1YBiBAjkoO\/kLFbJiehtZ8ABD3fmehxEZliMVXVKqa3l6dTgcA8VctKXj787IaAdC7VpJIB4UWsuwSW3B9Rnzg55wsKiqSy+VNTU2pqamx+HsToeAQANO8rKaSbKWTAjvFN52ymyL7luRp2cp+1nduA4Dq7OxMieS4TqdSqeKmMZXMe\/vAAlgEW02ufMYCGFzDuhmAVtdT9AA4Q2TELbvEthT27t1769YtOmyAjhaQSqXRDgqhkJH513QAPegvSM4Ip7q\/k90oat0CfhTaqCezstrtdrVabbHEQ7d4K0CZoCimm38zUvJg2q376zWuTYxrkxFAix1JIm35DtAORUVFxeXLl4XZq6Kioq+vr6+vr7W1FSdQRnGPABgB1PPq34yMUQc6OhEwzym7SefdSJ6WSa8ufgMXADi7c2ehTKbX68MZcNgRAD2AWtCopvPIaoJO\/54TlDw4ddaliA\/5gokNISRAAPQAivklFQ0QAzExphKGEd5OmnLkv0kXvE5471UJw3iOe4sVxHUGdK41LIBlflYDAJMrYzEemyj+LjfhbPpAfsHEhhASFNEU8+oeQQNgATAAAWK1Wp\/MyvJ86qzYfie7kS6vadX5k9uezMrKlEhMJlPogYcT8UhpDIBO0FWEpxdUQi7UhMYssB4tPWxjQ2hZIgAAYAVoBSAe1WUMAAtQCsDeX6HX6+kobOFe637+8xWbNt1\/oL5\/kDWtutsqHT8v8MrduyXPPjt3547j7FlnTw\/\/3Ors7ONGY0zce5MAWAUlMIqZd7vReYyCzKd5cIq8HJY\/FIqsZVdiGxsbq6ysxEmK0bJAXFdtI4ARoAygzFUsU7jqG40eF3Td\/VKa8JJNCPFaXHvA8mB\/vtwmPXZMeuJE0urVnrvToxmNxmDeV1gQQSF1\/lwq98+A16ymFzSt6RbYh9+TUi28D1oay7HE1tnZuW3btvPnz\/uefASh+EBc\/yUA\/a4FCKTPAuMqoqm8lz\/obGqFKu9X6M\/ff99x7hwArCzaLb33ffggGQDWtOruvTi0Ys+muTt3kkQir0+MKOL657WESjELl9Io4Wg2nc89jYIWONb\/KFF4LLvERsex1dfXl5eX+5jIH6GYQwDAVXkIgit1EBjXP5VfV14\/5wj9vK1NpNuy8sU99Jq+4oVNsyOj3Fd+I3n2Wa\/7Z0okra2tGo3Gv6D9QwSnpdW\/HM+417t6P6zJ76xGBKW6eBuJnhiWXWKjKioqjhw5Ul5eXlBQUF1dXVFREe2IEJqPAICgDYwEmMMY13\/pgkrwkAkyokyJZNF9ZkZHV743M3f0XlLDSgBIPpcmXl8K+5LA269Hfsbk4JFgTxHjKqEyfpSoiKDfP\/iR1YSzJGv8DgmFzzJNbAAglUovX75M7xpaU1PDr8diHIoasnBvDq8Y13\/pvyzXGjbskfmLGxkRz87O\/nzyizsdq98+BADJE2vgKwD\/NAdeGtpCYIUHN4vxjXH9l\/E7k\/HI\/IIa47Ou0nPnGOgZszwt38RGp\/kfHBzEEhuKJuJKZv8mmEvXEzP\/0szEYl+7GY5bsW5d0kMPzd25w6VcunvgP9e26pOnZQCQ9Opq6bofOra\/xveWBIBBjtvHMMG8klFQ18ejR2IBQFBCZYM5\/H1kfkGN8dbvn2edP\/KPnT\/zFoqsZZrYKisrm5qaMjIyOjo6sHCGIo14K5m5dbBgAqkrW2J07qt2u913x8jVu3aJDx4EgHt9fQAwK7b\/5Yn\/J3W68qF3vwYAyZPr1rTqnLKbjvw3Z8V2esO2LN89LRfCj39jADQL9nkJHgHQuyZ+BD8Kam6dKn3sjCJi2SU2vqCGvSJRpBFXPjN625oeQ5nME8MwCyW2lXv2rNyzh384OzHx+Qcf8A+\/eOJD0c+2Jz2bAu0pACCyb3m4+ZezEvtYlhGgmQmuxGadv2wFMLkqYzXBHO8BMj+lgc+CGnGfS\/N+9SMbWgwoZMsusQFAfn7+O++8g\/MUowghC+czRlAyywJQRDSugLAsazQa3SaK9HRvaGjy5En3tVlzs\/\/rtvN\/3BB1bqE1k8nTsq\/0VtmgmGllAALPBKwrtxGPDiNlrjJTaSC1tcQ1QNsqWMn4LHvpPVKaj51RZC27xCaXy2tra6MdBVoeCIAJwOhx8WW89S+P7Xuql5aWGo3Gt\/r7hYU2LznMw+dtbZ+3tQEAbAB4AsS931nZz9L0xgDzYGIqDQD4nRgMrgYt4m0rAdAB6Fy3kmEWOAh9rmc+Az+ylN7vHiUoGpZdYkMoEoi3lMb4MV4qVrEsy7JsTWfnIvOPLIbL\/g2X\/Zvaq\/ZvgI4l7P21xJUndII+IHQhS5CZGEEXR2HXDOL6b+v8LGUFULiqBxmP8W1W8IIB0PqszySBDGhDUYKJDaFwMwLo56c0TbzmMyGDwaBQKGp6e6uzs0M5ziDHvUJaV+la2VLWS4GJAIgWaIYUYnxuIvMPqPa+47ynaPyovSSBDGhD0YOJDaHwIR4jmdhESGkUwzA6nU6n0xXKZJ43r\/HTIMcdam1lGEar1QIAaF25gSxQK7gQEtzrCzCCWcQYP17O\/wFtKNowsSEUJp69CXwMe4pPWq3WarUeslqvqFRB5LZBjjve2Zksk1ksHoO8GEGS+\/8ANglmv6SI4L+eDxfCzN9fuF7jxzgBssDE\/wn3ySYYTGwIhYwso5\/zFotFrVYfslrfyM8PqL2NltVoVlukl3\/64iPQCSFWq7W1tZWfx5JhGIZhVCoVHXXn8YT5\/fjJ\/M\/L8+WIt7Ijk8ifbCLBxIZQaMiya3exWCx6vf64TtdAyBv5+YtO+TjIcQ2E1PT2MgyzeFbziRBiMpmMRiMhJFMiyRCL+Rks\/+\/Nm0a7HQAYhtFoNKWlpfNeiAEwAGi9deoh\/pX8NIn\/ySYMTGwIhYAIshqzjK59Wq1WpVKp1erc5uYShnkyK8trzSRNaW\/19yfLZDqd7n67WrD0er1Op6P3O319gbpQ+oo6F\/dXZFwVnsQ1fYl14VY9ZpG7+aCYhYkNoWCR+VltmbW7sCw7Nzen1+uNRmNDaysACItQA9PT7a7y057iYq1WG3pBTafTVWdnlzCMjzJihlhcnZ1dnZ1d09ur0+mMRqP3MiLj+gnCd10B1wLj2gHFLUxsCAWFLOusxtNqtVqtliYeALBarZ8AAMDfMsw+Hy1egSCEqNXqWbs9oFY9mgIPtbaq1WqDwbBIGIzHAopnmNgQChyZn9Vie9KQCOC774dY2eiJEFJWVjZrt19RqQK9f1uGWHxFpTre2VlWVmazLfsPaTlJjnYACMUbglktckwmU19n59mdO4O7K2mGWPxGfv6s3a5WLzpIGyUOTGwIBYIIbrvF4D23lpbVatXpdAv1TPFThlh8dudOq9VqNBrDFxqKaZjYwrzgylUAACAASURBVKOyslKpVCqVSpVKNTY2Fu1w0JLhx+oyy7ddLWL0en2hTOZ7Bi9Hfz\/57W\/v2u0+9imUyUoYpqysLNwBohiFiS0M6uvrOzs7Ozo6+vr6SkpKnnvuOYfDEe2g0BKwwoPZ6H1MG4\/CwWq1Wq3WReeldBACAI7+ft+70RvuYKFtmcDOI2Fw69at\/Px8eifuwsLCd955h+M4vN9boiEA\/C9+DQ5sWnImk4kOWROuXPfzn6\/YtEm4Rvrxxz1f\/jJdFm3dKj1xInn9egCAmZk7b7\/N\/fu\/83uWMExra6tGo1nqyFHUYYktDDZv3tzZ2UlrINvb21NTU8VBNXSjmGZyNa2xy2UUdnRZrVZ\/OvevfOyxbUNDqWo1AEj+8R\/vZzUASElZdfDgyt27+T0LZTI6ZcmShItiCSa2MKioqHjhhRcKCgqUSuWtW7cuX76MxbVEQ+ZXQqIlRgghhCzUZ+Tz99\/\/rLj4s+Jix9mzs9PTovT0h8vLRVu3JonFMDNz57e\/va3Xz05MJK1eLdqyhX8WTZNWqzUybwFFEVZFhkFlZeXIyMiNGzekUml9fb1KpWpsbKQ1kzylUskvi0SieBxVMzo6KhKJZmZmnE5nuI45MzMjXBaJRKOjo0GcnOHh4Q8\/\/HB4eHhoaIiu2bRpU3p6+j\/8wz+EJ9CnAUQAAFAEMLckXfz5yNHQ0NDIyIhIJEpJSRm4e1e46S+Dgyuczi9GRzm6vqVF9NBDkv37v7h794vbt8UDA8lS6V27fYYuT0zc\/fjju4IjZK5ZMzExEV9\/fQqFgl49+vr6oh1L3MDEFqqxsbHOzs4XXniBltKOHDnS1tbW1NRUUVEh3I3\/UiqVSqfTqVAoohBraPr7+51OZ0pKikgkCuNh+aPdu3fP6XSmpaX5f3LoFO8mk4n+DKdTOtFNv7HbAeDUqVN0StyQBg5bAdoBAIABqAv+MIuKx2\/FEvmv\/\/ovp9P5tYcfdlu\/LiNjxaZNn9tsjlWr6Jrkubl16en3Vq\/mWltFt26tPnwYjh2jm5zd3bfffx9cewIA3Ls3NTUVX+fZZrPNzc1FO4o4g4kNxSuj0ajX62ft9kKZzOtkS3Q+3EGOo3MGLj6v0kL0rgWshIw9D61dm5ScnPzQQwCQvH49pDy4piV75EUAwDa25QDb2EIll8vz8\/MvXLhAu\/hfunRpeHi4qKgo2nElMkKIXq8vKyv7OsAVlerszp1eexnQ+XDP7tzZdeBA+vS0Wq0OZiSTVTBwjQ0lahSwQY5bdJ+UtDRISZnjuOT16x\/6+tfn7txxnD37WXHxvaGhFZs2SV2lN14oczGjeIEltjCora2trKzctm0bAKSmpl66dMmtgQ2FET8lLp3E3Z+n0DkD3+rvP240EkK83L7ZByyuRYOf6Ue0datoxw4AmPnzn1c8+miSSDQ3vwE4WdD9ZJDjBqanVSpVWCNFsQgTW3jU1tbW1tZGO4rEF8qUuE9mZWWIxYesVrVa7W9uI1hciw5aadxut3sti6\/cs2flnj38w9mJic8\/+ICuT16\/XnrihPTECQCAmZmZjz\/mdxuYnl7iqFGswKpIFE9CnBK3UCa7olJZrVa9Xr\/43iC4BaUmiFdDIWFZtt3nRFnUvaGhv3zve86eHmdPj+Ps2dmJifsbPAZot9vtDMOEfhsdFPuwxIbiBp0Stzo7O5QpcWlPk+M6nV+3CjO5FrD6KuJKS0vLysqqs7OFP2ImT5708RRnT89fvvc9r5sGOa6mtxenHVkmsMSG4oY\/U+L6g84Wv3ihjbhKbCzWQ0aBRqNhGKamtzcsR6PFtdLS0rAcDcU4TGwoPvg5Ja6fShiGHtDnS7oW2LC8JgqYRqNpIMSfCknfaHGNZVmsh1wmsCoSxQdaXBNWQnrOh3tvaIhWVYm\/+93Vhw8LhzQBHa7705\/S5Sezsmp6e00mk68rHdZDRptWq7VarSc6O6898UTQB6FZLVkmC\/vdvVHMwhIbigN0hpFFm9ZWbNr08K9+Jdq61XPT3J07n88vnz2ZlbXITUzo7gyW2KLJYDAMTE8fam0N+ggNhPwBwGAw4Ai25QMTG4oDdLYIr4lNOB\/u3J07yevXry4q4v793z8rKaHr7\/z2tzAz88Uf\/vB5W5vwiZkSCfiYh4JfzYTrTaBgMAxjsVja7fYgchstq9E+I1gJuaxgYkPhVFdXZ3Gpq3OfV5FuraqqCvSwPhIb7\/O2ti\/+8AcAmBOL+fspi7ZuXblnz+zt23Sck9Aic70T1wIbaLAozFiWpbkt7913\/W9vG+S4Q62tv+E4nU6HlZDLDSY2FDZVVVV5eXn8w7y8PD635ebmNjY2CrcGpL+\/nxawfJudmICZmaRVq6ZXrJjhOABY+fjjyevXOz\/6yNnTE9hL8sUDbGCLASzL2mw2ZX7+odbWQ62tb\/m8XzYtqOU2NyfLZBaLBbPaMoSdR1DYZGdnA0Bzc\/OZM2eqqqoOHDiQmZmZm5srk8l++MMfisXi27dvr1mzZqnDSF65cnVaWorTSedb4qel8Kp\/oUukdYmiQ0GidZJWq7WsrOx4Z2dNby8tc\/Pl+EGOa7fbB6anac9+LKgtZ5jYUNhMTEzQmR2uX79+5syZM2fO0PX79+\/nOO7111\/\/xje+EVyhLSsry5\/5kO5P7n73rtjpBADRli3Ja9bc+\/RTH8W1rEXv0cwGFipaUrToRgih9ypyq0lmGIY9ePBZlQoHYi9zmNhQ2PT09OTk5IjF4tOnT58+fZrO6wgALS0tLS0tAPCNb3wjlOMPcpyPmbSE8+HSNSmPPQYpKfxDz6OBj8l2rQAQnp4jer3en1ulcBwnDmqeMABgGGZZlU7o+6VvmT+32OkR8TCxobAxGAzDw8O01hEAGIZpbGx86aWXurq6QjyyRqMpKyvzOiXuQvPhAr0d18zMg8kD56NFQO9XQ+Ja8LYxUFartaOjY5XwdpferFix4t69e0Ec\/+7du4888siySmxCmM+QJ0xsKJz4whltY1u7du2OHTtCT2wAwDDMQnO98\/gB2gAg2ro1yWcB6K3+foZhFrks+tzov1WrVu3YscP3Pk6nM7hbk+OdMxFyg4kNhUdubu7zzz+\/du3ahoYGg8FgNpt37dq1du3acB2fZVmj0Xh2505+TdDz4YLr5to6nc77ZuJaYAKMEiEUA7C7PwqPrq6ugYEBkUh09OhRi8Xys5\/9bMOGDX\/9618\/+uijsByfVrX57uftP9pxDu85iVBCwsSGwubUqVPXrl3jH46Pj4elgY2i\/S2Pd3aGfiicEhehxIZVkSicTp06FfTWRRkMBoVCcai19UpoJS2cEhehxIYlNhQ3+GkDQ7lHV01v7x8AtFot9qZDKFFhYkPxhGVZnU5HZ7YN9LnCKXH9HcBLAn0RhFD0YVUkijO0ClGn07Xb7f7XSdJukP5OicuEFiJCKKqwxIbij1ar5ad7X7Toxk+J+xuO46erWATjWiChBYoQigYssaG4ROcMNJlMtGaS3ly7UCbjbwIwMD1NZ8VtICSYKXEZAIKJDaG4hIkNxSs6YWBpaSmdD7fG485qdGIRg1Yb\/JS4JKQIEUJRgYkNxTe3+XCFU+KG1O+RBTACAADBJjeE4gwmtvCor6+vqamhy+fPn9+7d29041meQk1mXhFMbAjFGew8EgZms\/ncuXNXrlzp6+u7cuXKT3\/60+7u7mgHhULDd7ckUQwCIRQMLLGFyuFwXLhw4dixYzk5OQCQk5PT1tYW7aBQ+LQCaKIdA0IoEFhiC5XNZhsaGiosLIx2ICisNNEOACEULCyxhcHatWtHR0ePHDkyNTWVmpp66dIlWnoTUiqV\/LJIJLLZbJGNMQxGR0dFItHMzIzT6QzXMWdmZoTLIpFodHQ0Vk4OAzAM8G8APwnpME6nc8WKFYueNOGpCEKsnLRwGBoainYIMUShUNCrR19fX7RjiRuY2MJgYGDgwoUL7e3tUqnUbDYfOXLEM7fxX0qlUul0OhUKRTQiDUl\/f7\/T6UxJSQnufpgL4Y927949p9OZlpYWKyeHdXWMTAqp\/4hIJLp3754\/Jy2UExsrJy1MEuzthMJms83NzUU7ijiDVZFhkJmZ+dprr0mlUgAoKCjYsmVLe3t7tINCIeP7j1ijGARCKGCY2EKVlpYGAKOjo9EOBC2Z1mgHgBAKBCa2UMnl8oMHD7788ssOhwMAOjo6bt68iX1JEoHGtWCMXgwIocBhG1sYVFRUAMC2bdsAYKHOIygusa56SILDtBGKG5jYwqOiooKmN5RQSl2JzYoDABCKG1gViZAfsJkNofiBiQ2hhWlcNZDGaEaBEAoIJjaEfGJdC8boxYAQCgQmNoR84kezYW0kQnECExtCPmmwNhKhOIOJDaHFsK4FY\/RiQAj5DRMbQospdS1gbSRC8QATG0KLYQW1kSSKcSCE\/IKJDSE\/aF0L+mhGgRDyByY2hPygcS0YoxcDQsg\/mNgQ8o\/GtWCNXgwIIT9gYkPIP\/yANlM0o0AILQoTG0L+0WAXEoTiAyY2hPymcS1YoxcDQmgxmNgQ8hvfN7IsmlEghHzDxIZQIFjXgjV6MSCEfMIbjSIUCP7WoyZBkgtNXV1dXl6ezWZTKBTXrl07deoUXZ+bm\/v8889v2LABAMbHx1966aWurq7wvCRCCQ1LbAgFQhPmLiRVVVV5eXn8w7y8vLq6Opif1QBgw4YNP\/jBD8LweggtA5jYEAoQ61qwhuFg2dnZANDc3Pz00083NzcDQGZmZm5u7o4dO9auXTs+Pv6jH\/3o4sWLTqdTLpfv378\/DC+JUKLDxIZQgPguJOGYE3liYgIAWJb9+7\/\/+zNnzqjV6uLi4q6urg0bNohEooGBga6uruHhYafTyXGc3W4Pw0silOgwsSEUICactZE9PT1Op1MsFn\/ve9+zWCwGg4Gup0mOtrd94xvfEIvFNMmF+noILQOY2BAKHF9os4Z6JIPBUFtby3EcfcgwTGNjY25uLr8D7VpCCOE7lSCEfMPEhlAIwlEb2dLScvDgQb6Nbe3atTt27KCb+KxWVoZD5xDyFyY2hAKnEdRGhiA3N7exsbGlpYXmLbPZPD4+zm\/FrIZQcHAcG0JR09XVNTAwkJeXd\/To0ccff1yhUADA+Pj4Rx99VFZWlpOTAwAMw1gsFgDgOO71119vaWmJctAIxTwssYXT2NiYSqUym83RDgQtPda1QEI6zKlTp65du8Y\/5Adib926VSQShXRohJYrLLGFU01NzeDgYLSjuI8QAgBWq7W19X5DEMMwAKBSqViWjVpYCYNxLRDBclBorxCn0ynMZNhVBKGgYWILG7PZPDIykpGREe1AgBBiMpl0Oh3A\/9\/e3ce0dd57AP85wUswJjRJMRcIcOxVrQaFrgphTJmwncukdFPWLhpaYF2xc3fbpEmkVEGkrVRh0j+WS5N2WruFpFOwt050l4i7rOomXVg5ZkNlc7K2uOE2XWofCCE1vkmW4phkvPj+8cTnnmBeDPj1+PtR\/rCPjw8PJ8d8\/byc56HCjAwiKlCpiOjvRH1eLxFxHGcymerr61nUAQDICYItMsbGxk6fPv3iiy\/u27cvjsUQI60wI6OxuHhrdvbW7OxZ+1z2+9sF4fQrr1gsFpPJJN44Fabbt29Hrrw0NTU1PT0dwQPGjhDvAgDAPBBskdHZ2VlVVaXRaObbQafTiY+VSqXb7Y54Ga5cufLDH\/5w5vr1Z7\/ylR99+cts43BoDq1atUun26XT\/eKzz9789a95nj969GhFRcWix\/d4PEql8sKFCxEsc1pa2tTUlPhUqVR6PJ5onJy5XSH6L6K\/EF0Jbskn+hrRd4nyF3vv34lYw2Ea0YLlnZycXL169eTk5MLHk56HZYjdSYu+kZGReBchgWi1WvbXw+VyxbssSQPBFgFOp\/ODDz44fvy4eJttKPGi1Ol0k5OTbPxbBFmtVrPZXJiRcVavZw2PizpSUvLvWu1+h2PXrl09PT2LdrxptdqcnJwIlFXC4\/HMOibHcbFoIBWIzHPdXi0Q9RG9SsQRtc0\/fz\/bjYg4om8s8qOUSuX09HQ4I0FWMlok4ldUfEXj1xEEgXU5s+5nQRC4oETueHa73YFAIN6lSDIItpXy+XxvvPHG\/v371Wr1AsEWVTzPm83mWo57vbx8SW8sUKnO6vUHzp0zGo3hZFvEP\/xsrZbIHnMRAlFzyP1nXPAl6W5GIhPRnC21fPCBIXIFg6iZ1evM2udziUgQ\/n7hglXS8dzU1LTgkSA5INhWyu129\/f3d3V1iVuefvrpxsbGPXv2xKYAgiAYjcZlpJro9fLy4Vu3wsy2JCYQ2Ygski0ckYlIf+\/YfZtkEkj2oCfkOObg2+ujUlKIlHB6nSnY8WyxWKxWK+JNBhSo5EbQ2NhYTU3NSy+9VF1dPd8+Op0usm0LRqPR5XD87bHHVnicx+12ZUkJuxc4ZmJXYxPubXvkiEySKR9DNUsi0CDJNulxTPPU5+5lNBo\/\/PBDcZas+cwa7h8+1rAmpz62SF0V7DvfjNe7q6iosbh40f1ZvLUMDhoMhhh\/EOYT8T8XKQI3aCc3nud5nl92XU2qsbiYHW3lh0o4PJE2mEYckYXIvWCqEVGTJMz44HsFIqPkOPhan8AEQdBqtTNe71m9PpxUI6IClaqxuPisXs\/zvMz6LFMNgi2SNBqN3W5foLoWcc3NzfO1riwVO05zc\/PKD5VYmomMwcccUU\/YgWSQZJuRyEikDTZRsuNwkSskRBSrqxVmZPztscfCHEsl2pqd\/cG3vsWOEKXiQbShjy2JsVFeZ\/V66cb7Xntt9aZN0i3TIyP\/eO458al6794127YRUWBi4tYvfnGnt1d8qZbj9vM8Gy0W3aLHhnBvp5oprJbDexiIDMEqGh\/cyCHVEhqbNnrG6112+zwbVPU4z5vN5qXe6AmJADW2JGaz2cQhXgtYvWnT+pMnlQ8\/TJJUIyJFerrqBz9g25ldRUVEJJPWSIHIKEk1y9JTjemRHIQLNmNyKykZRBfP8y6HY4Xt81uzs9\/YssVqtcrk45BiEGxJjOf5+ZpZ7rz33rWamms1Nb7XXw9MTKzasCF9507lww8rv\/pV9uoXzc0z16+vWrdOWVIifWMtx4lzSyYxgcgoaTa0rKw\/rIkoQBQIo2cO4o1V1yLSPr+rqEiejfMpAE2RyU366Z3y+\/\/X4VD7fKuJJn2+215vWkYG9fYqS0rWbNu2av36VRs2KNLTZ65fv\/OnP01+\/PGNZ54JPWCBSvWfyf4VtfneOhaaDVMJz\/OFGRkRGU5FwcZ5nuflfBuMHCHYkpggCBWSGts\/Bgdve72BmRkiuu31fh6seG3avFnzjW8o0tOVjzyiUCoDExOZjY2K9HSampr47W\/9v\/mN9JiFGRnC4GAsf4tIEu7tVOMWmewK5Mdms82qqy3c66x8+GH1gQOrNmy4+9q9H4pdRUUtg4M2mw3BllzQFCkf95eX3xcc1rx67dq0jAz2eIbNUqhQsKes3kZElJaW\/sQTqu9\/P\/ZFjQohpFMNqZZi2HAq1lW8AGmvs7KkZNW6dQvsvDU722q1RrCQEAOoscnKfcXFynXriEiVm7vpsceIaMrvX\/e1rym+9KUZv3\/yo4++tGULTU6ywZDsm2zaQw9JjzB861ZSDomc1fxoQmdYKmL3qs\/Zu3bnvfd8J04Q0Zqqqowf\/Yj1Ok9+\/PGqDRsoLW3S6fziyJE5j7k1O7tdEOQzVDg1oMaW3C4vNjtlekXFmvJyIpr69NOZ69cDExPiSzM3bkS3cLEh3FtR45ZypxrIi91uLww2VMznTm\/vP99\/n4hWrV9PRKuys4lIWVq6saNjY3t7aAMGi0kWmZAsUGNLYgaD4b2OjjlfWrNtmzisn4jEASOTH364Zts29YED6gMHiIimpqYuXpS+sc\/r5e4dJ5m4hJC5Hy2ItJQmCEI4t2PPXL9OU1OK9PQ7mZks3u5KS0t\/4gkiknY8swMi2JILamxJTK\/XD9+6tWilbXpk5MYzz0x+\/DER+U6cmHQ6774QMnjkst\/f5\/UmxwywVlTUYEVm\/vnPmfXrA6tX09TUxJkz12pqJp1OSkub1Ti\/6OcLEhBqbEnMZDKZzeZ2QZBOhSedZGRO8\/UlEKuucVxCDwATiHgi21KmM4aUwXHce2HkEOtXm\/H7Pa++6nn11fuKi9moqxmvl4JNlKFHjnRhIYpQY0tuJpOpJUKj89nU5on7ARaCsz6aQ2a3QqoBEREVFRUN37q18D7iNAWBoaEHXnnlUZ9Pd+7cmqoqIkp78EEK6Xte9ICQgBBsyY01G0Yk2\/q83isZGQnXDikEWx21RBbJWqAcURtmt4J7sK9lczYertm2bWNHx8aOjnVNTas2bGC9zlN\/+Uvgxg1Ferr6wIGNHR2rN20KTEzcuXeCgiRoxoAQCLbkxnGcxWJpGRzs83pXcpzLfv9+h8NgMMT\/AywQ8URWInMwz6RVNCIyEfUQuYlMcSgdJDKDwcBx3KLf86S9zv947rnpkRG2PXRacAoGW3TKC9GCPrak19TUxPP8AYfj9fLy5c2Pd9nvf9xu5zgu1hOZC0RXiNgEKfZgpM2HIzIR1aOKBgsxGAxWq1U6pdaivc4L7MCGU\/UkWjMGLAbBJgdtbW1ms3l52cZSbVV2dtSXDBaIiIgPJplAxBMpiSbnfwtHxBEZiPREhmiWDeSivr7earW+PTS06Pwj4WgZHEQ7ZDJCsMkBq2yZzebHeb6xuDjM9YKJ6O2hof0OB8dxPT09EW5vEYL\/7JLHi+KC\/\/TBSANYCtacvj+MibUW1ef1tgsC1mNLRgg2mWDh1NzcbLFY3h4aaiwuXviDzTrV+rxei8USmQEjgiTJrOGUmIiIKohUiDGIpLa2Nq1W+7jdPmsN3iW57PcfOHeO4ziTyRS5okGMINhkpampqb6+3mw27+f5lsHBXUVFhRkZBSqVOM9Qn9fL\/rE5IdtaWlb0uRWCTYvCgt1jFFIV44LB5ibSLv\/nA4RiX\/KMRuOBc+eWt34N+9oXi\/Z5iA4Em9ywT7UgCDabzWKxzLnDtpqa+vr65fccCMFbpPn5CoEWRYgng8HAGueJaKnZxlLtSkZGW1sbxkMmKQSbPHEc19TU1NTUxOa4E2e6W1E3uEBkI7LO1VvGsaMjySBRsKYIs9nc5\/We1evDmUOSiN4eGmoZHGR1NaRa8kKwyRz7cK7oIyrMXz\/jiAxE9UgySEQmk8lgMBiNxkd\/\/\/vG4uJajpsv3i77\/cO3brH7QVltD6mW1BBsMD+BqDlkJAhHxBHV4\/5oSAIcx7ndbjaoqmVwsDAjgw2qYn3PbI4SNvqR7bzSXmdIDAg2CCGETDTMGJBnkJTEZnmbzcbzPC+ZNIvVzCwWi16vx\/1qsoFgi4yGhobOzk4iKigo6Ojo0Gg08S7Rsghz9aJxmPID5EDseKZgrzPaG+UKc0VGQENDw+jo6MDAgMvlqq2tPXjwoM\/ni3ehls4aXOFMCG7hghMNNyHVQFY4jkOqyRiCbaXGxsYcDsfu3bvVajUR7dy5c3x83O12x7tcSyEEl4MRgltMmGgYAJIVmiJXSqPR2O128anH47l582Ycy7M0ApFNsg41ERmImjDKEQCSGIItwmw2W0VFRWlpabwLEgb+3loaR9SEKlpU3L59+\/PPP194n0AgoFAolnfwtWvXLqtcAPKEYIukhoYGh8PR0dER+pJOpxMfK5XKSLVVer3empqaq1evElFubm5HR0d2djYRvfnmm8eOHQvd\/8SJE9u2bTt64Kj6v9X7aT8RkZIon2gnXTBeqKuru\/30ben+FRUVJ0+eVKlU0h80a4df\/epXyy7\/SHApLBmbnJycnp7+7LPPFt4tLS1tampqeT8iPz8\/yVq\/F5QKV0X4tFot++vhcrniXZakgWCLGDHV5hwSKV6UOp1ucnJSq43ADIk+n89sNg8PD7Onw8PDNTU1\/f39ubm5WVlZk5NzrAej0WgyvBnmbnPlZOXdTQaiNiKOrp+\/Pj09PetdfX19LS0tNpuN1QlCj5mWlrbC3yUipyKR\/fnPfw5nN7fbLftTET6cCpHb7Q4EAvEuRZLB4JEI8Pl8dXV1o6Oj7777biwH+l+8eNHhcOj1+vHx8fHxcb1ePzw8fO7cOSI6fPhwIBAIBAJHjx4loqNHj7KnO0p3rNm+pvJOJRENKYasnNX3jk864pEdje08OjpaWFjI8\/zVq1dzc3OHhobEjYWFhaOjo4FAQHpLEABAIkCwRQCba\/jUqVNsYGTM5OXl3X\/\/\/Xa7\/Wc\/+5lareZ5PhAI7NixY943NBNpKetGFhHd\/pfbhysO7xvbd\/HixdiVGAAg+hBsK+V0Oru6uvr7+8vKynRB3d3dMfjRubm5bK6E559\/XqFQZGRknD9\/fs49s25kUfP\/j378fO3n03+cfvS7j\/r9\/llFtdvtmZmZCoVCoVDk5eUNDw8\/++yzubm5Uf1FAAAiCH1sK1VaWvrRRx\/F66fbbLbi4uLnn3+eiPx+f3l5+e9+97tZlTaOuNLzpRTMr2ZqXmtZe7j48FPrn\/r5z3\/+hz\/8Yd++fXPWNVUqVW9v7+bNm6P\/ewAARAxqbEmPdaexPjYiOnPmjPTVrBtZPdSztXsrEc0Uzlg5q4UsrIbHKmQOh0PaGsn62M6dO6dSqULrcwAAiQ\/BlsTeeecdhUJRX19PRGq1+tChQ7P34GnPf+zh2OAQjga+O2AWzLN28fv9P\/3pT2dt3Lx589tvv01ER44cma95EwAgMSHYklh5eXlhYeEvf\/lL1iX2ne98h4i+973v3X25mch49+HN9Teph1678RpJRkgGAgFWM2PjHmcdfMeOHU899ZTf7z906FBSTn0JAKkKwZbEcnNz+\/v7CwsLxS13O9gEkg4VsZK19XDr1TVXeZ5XqVTV1dXi\/g899NCWLVvEmwRmOXr0aGFhIRt1GdVfBAAgghS49S\/G7RBFRwAACDpJREFUdDpddO+4FIiMkomyLERN0fpRK4e7kkU4FSKcClHU\/1zIFEZFJjT+Gm+7bBMmBMEvEJGpwFSUXmQqMM37BkGSahyRKaFTDQAgGhBsCUrwC82fNltHrNKNlk8tRGQbsfV8vSfkDffO088R9WARNQBIRQi2RCT4BeP7RmFCmPNV\/hqv\/aPW\/a+SSW8FIjMRH3xqIAoJPgCAFIHBI4nI\/JF5vlRjhAnB+L6RiO6OE9EGU40jsiDVACClocaWcPhrPH+NX3Q34X8E4bcC18L9\/yaOqA1rhAJAqkOwJZzmT5tDN3JjHOfh2AMiqv9jvcFpkLyMcSIAAHch2JJGz4tztTByRAaiJowTAQC4C31sCSecdkgiEjSC9d+s1HN3mVAAAGBQY0s4XDoXOnJE0AjNtc1EJOQI7ClfynPpnIkzxbp8AACJDcGWcAwbDbNuX2MsdZZZWzgVF\/3iAAAkGTRFJhz9Rn2YezY9iOEiAACzIdgSjqnAZNhoWHQ3w0ZDOLsBAKQaBFsianukbeHQ4tK5tkfaYlUcAIBkgmBLRJyKWyDbDBsNPV\/vQQcbAMCcMHgkQXEqrufrPYJfsI3Y+Gs8m93fsNFQX1CPFkgAgAUg2BIap+KaHmxqwpwiAABhQ1MkAADICoINAABkBcEGAACygmADAABZQbBFRmtrq06n0+l0er1+bGws3sUBAEhdCLYI6O7ubm9v7+\/vd7lctbW1Bw8e9Pl88S4UAECKQrCtlM\/nO336dG1trUajIaKdO3eOj4+73e54lwsAIEUh2FbK7\/dfuXLlgQceYE9VKlVmZmZfX198SwUAkLJwg3YErFu3Licnhz1Wq9V5eXmh++h0OvGxVquVPgUAmI\/458LlcsW7LEkDwRYj4kWp0+nQUCnSarU4GwxOhQinQsRORSAQiHdBkgyCLQK++OILj8dTWlpKRD6fb3R0VGyZnBMuU5FOp8PZYHAqRDgVIpyK5UEf20qpVKr8\/PxLly6xp36\/f3x8fOvWrfEtFQBAykKwrZRard69e3d7ezu7fa2zszMzM1Or1ca7XAAAKUqBem5EtLa2trS0EFFBQUFHRwcb+g8AALGHYAMAAFlBUyQAAMgKgg0AAGQFwQYAALKCYAMAAFlBsAEAgKwg2AAAQFYQbDGF9UgZn89XV1enC0rls9Ha2trQ0CA+lZ4Z6fZUMOtUOJ3ORx55RLxIUuRsNDQ0hH4oUvmqWB4EW+xgPVKR2+3OzMwcGBhwuVwul8tut6fmLe3iff0ii8WSl5fncrkGBgZGR0dbW1vjVbYYCz0VfX19e\/fudQUdO3YsXmWLmYaGhtHRUfa5kP6JSNmrYtkQbDGC9UilPB5PZmamWq2Od0HihlVH2tvbKysrpRv\/+te\/1tfXU3Cqtt7eXtl\/+5nzVBDRpUuXFp5MXGbGxsYcDsfu3bvZ50L8E5GaV8UKIdhiBOuRSl26dOmTTz5J8XbIkydP2u126ep9Ho9HoVCIa\/vl5OSMjIykwref0FPh8\/nGx8ePHDmSOu1vGo3GbrdXV1ezpx6P5+bNm5TCV8VKINhiJ5z1SFPEpUuX1q1bJza51NTUpFq2lZaWzqqgMPn5+SqVij3OycnJysqKbbniYM5T4ff7P\/nkk7q6OrH9LRWyTcpms1VUVLDFsFLwqlghrMcGcSDtL3nyySd7e3s7Ozv37NkTxyJBQmHVF\/ZYrVa\/8MIL+\/btczqd7A+97DU0NDgcjo6OjngXJFmhxhY7bD1S9pitRxrf8iSIFK+8znLlyhW\/388ei41RkFLVFDHVxBFVuCqWCsEWI1iPVDQ2NqbX67u7u9nTcNYcTxE5OTmBQED89uPxeDZt2pSaa\/t1d3dLO1\/ZORFb8uWKDesfHR199913xVTDVbEMCLYYwXqkIo1Gs2XLltOnT7ORXW+99RYRzdnhlGpKS0srKipsNhsFh9FWVVWl5tjRsrIyIurs7CQin8\/34x\/\/+Nvf\/rbs7wmxWCxEdOrUKel\/Oq6KZUCwxU51dXVtbW1lZaVOp2tvb\/\/JT36SslfnsWPH8vLyysrKcCpmsVgso6OjOp2urKwsLy8vZfsdNRpNR0dHe3t76pwKp9PZ1dXV39\/PPhcMa9jAVbFUWGgUAABkBTU2AACQFQQbAADICoINAABkBcEGAACygmADAABZQbABAICsINgAAEBWEGwAkcemDZu1Ig\/bGDpLvbiuujjNWDgH1+l0dXV1WJcLIBSCDSDy2MQZRCSuCu3z+Q4ePJifn8+mTZqlsrJyYGBAXItr0YPb7fbGxsbIlRdAVhBsAFGh0Wheeumlzs5OVg976623Lly48MILL2DyMIBoQ7ABREt1dfXOnTtffvnl7u7uEydOHD9+fNHlxJxOZ1VV1fHjx6XLizc0NKTOQtIAK4dgA4gi1mD49NNPf\/Ob3wyzpfHmzZvnz58fGBgYGBjIz8+vrKzcvn27y+U6e\/ZsV1dXmP1wAKkMK2gDRBFbo+fy5cvbt28P\/127d+9mLZZVVVUUXNMnJyfnvvvui1I5AeQENTaAKOru7u7q6tq0adPLL78sHSG5gKysLNmvqAkQVQg2gGhxOp2HDh3au3fvmTNnSDJCEgCiCsEGEBVs3eeSkpInn3xSHCHZ2toa73IByB+CDSAqZo3vZyMkT5w44XQ64100AJlDsAFEXmtra0tLy6zx\/RaLpaSkZP\/+\/WF2tgHA8igCgUC8ywCQ0lpbW3t7e0+dOrWke7eX9y6AVIAaGwAAyAqCDSD++vv7y8rKljQJMsZYAswHTZEAACArqLEBAICsINgAAEBWEGwAACAr\/wf3dpXBHx2ZDAAAAABJRU5ErkJggg==","height":418,"width":557}}
%---
%[output:553528d1]
%   data: {"dataType":"text","outputData":{"text":" \n","truncated":false}}
%---
%[output:114182f1]
%   data: {"dataType":"text","outputData":{"text":"Resumen de la ejecucion PRM + MPC\n","truncated":false}}
%---
%[output:35a74053]
%   data: {"dataType":"text","outputData":{"text":"    <strong>Arquitectura<\/strong>    <strong>Escenario<\/strong>    <strong>Semilla<\/strong>    <strong>Exito<\/strong>    <strong>MetaAlcanzada<\/strong>    <strong>Colision<\/strong>           <strong>MotivoTerminacion<\/strong>           <strong>PasosEjecutados<\/strong>    <strong>TiempoSimulado_s<\/strong>    <strong>TiempoHastaMeta_s<\/strong>    <strong>Longitud_m<\/strong>    <strong>SuavidadRMS_1_m<\/strong>    <strong>DistanciaMinima_m<\/strong>    <strong>DistanciaFinalMeta_m<\/strong>    <strong>Planificaciones<\/strong>    <strong>Replanificaciones<\/strong>    <strong>FallosPlanificacion<\/strong>    <strong>EpisodiosRiesgo<\/strong>    <strong>EpisodiosColision<\/strong>    <strong>TiempoPlanificacionTotal_s<\/strong>    <strong>TiempoPlanificacionMedio_s<\/strong>    <strong>TiempoControlTotal_s<\/strong>    <strong>TiempoControlMedio_s<\/strong>    <strong>TiempoControlMaximo_s<\/strong>    <strong>TiempoCicloMedio_s<\/strong>    <strong>TiempoCicloMaximo_s<\/strong>    <strong>TiempoComputoTotal_s<\/strong>    <strong>FactorTiempoReal<\/strong>    <strong>CiclosFueraPlazo<\/strong>    <strong>EsfuerzoControlNormalizado_s<\/strong>    <strong>VariacionControlNormalizada<\/strong>\n    <strong>____________<\/strong>    <strong>_________<\/strong>    <strong>_______<\/strong>    <strong>_____<\/strong>    <strong>_____________<\/strong>    <strong>________<\/strong>    <strong>_______________________________<\/strong>    <strong>_______________<\/strong>    <strong>________________<\/strong>    <strong>_________________<\/strong>    <strong>__________<\/strong>    <strong>_______________<\/strong>    <strong>_________________<\/strong>    <strong>____________________<\/strong>    <strong>_______________<\/strong>    <strong>_________________<\/strong>    <strong>___________________<\/strong>    <strong>_______________<\/strong>    <strong>_________________<\/strong>    <strong>__________________________<\/strong>    <strong>__________________________<\/strong>    <strong>____________________<\/strong>    <strong>____________________<\/strong>    <strong>_____________________<\/strong>    <strong>__________________<\/strong>    <strong>___________________<\/strong>    <strong>____________________<\/strong>    <strong>________________<\/strong>    <strong>________________<\/strong>    <strong>____________________________<\/strong>    <strong>___________________________<\/strong>\n\n    \"PRM + MPC\"      \"alta\"         7       false        false         true       \"tiempo_agotado_con_colisiones\"         1000                150                  NaN             50.67           2.5509             -0.63129                5.945                  111                 110                    9                    3                   3                     0.70705                      0.0063698                    2.3626                0.0024636                0.058126               0.0057846               0.24077                 5.9693                25.129                1                       74.081                          110.2           \n\n","truncated":false}}
%---
%[output:7c924386]
%   data: {"dataType":"text","outputData":{"text":"Construcciones de roadmap PRM: 1\n","truncated":false}}
%---
%[output:2bd131ab]
%   data: {"dataType":"text","outputData":{"text":"Expansiones de roadmap PRM: 0\n","truncated":false}}
%---
%[output:2ebd897d]
%   data: {"dataType":"text","outputData":{"text":"Consultas PRM realizadas: 111\n","truncated":false}}
%---
%[output:36ccdca0]
%   data: {"dataType":"text","outputData":{"text":"Episodios sin candidato factible MPC: 6\n","truncated":false}}
%---
%[output:27569d34]
%   data: {"dataType":"text","outputData":{"text":"Pasos con riesgo predicho por MPC: 37\n","truncated":false}}
%---
%[output:7f38c900]
%   data: {"dataType":"text","outputData":{"text":"Pasos con colision fisica predicha por MPC: 24\n","truncated":false}}
%---
