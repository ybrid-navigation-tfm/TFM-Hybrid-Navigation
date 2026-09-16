%[text] # RRT\* + MPC en un entorno dinamico
%[text] Ensambla los modulos del proyecto TFM\_RobotNavigation para ejecutar una simulacion completa de la arquitectura RRT\* + MPC. El planificador global es exactamente el mismo RRT\* convencional utilizado en RRT\* + APF. La unica sustitucion metodologica es el controlador local: MPC recibe el objetivo adelantado comun, predice el comportamiento del robot y de los obstaculos y genera el control \[v w\]. El robot se integra mediante el mismo modelo de uniciclo \[x y theta\]. La visualizacion conserva la misma estructura: los objetos se crean una vez, se actualizan sin clf y el arbol RRT\* anterior se sustituye completamente cuando existe una nueva planificacion.
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
fprintf('Raiz del proyecto: %s\n',raizProyecto); %[output:32ff8ad1]


% Comprobacion temprana de las dependencias utilizadas por este main.
funcionesNecesarias = [ ...
    "parametros_generales", ...
    "escenarios", ...
    "configuracion_robot", ...
    "rrt_star", ...
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
        error('ejecutar_rrt_mpc:DependenciaAusente', ...
            'No se encontro la funcion %s.m en el path del proyecto.', ...
            funcionesNecesarias(iFuncion));
    end
end
%%
%[text] ## Configuracion comun, escenario y robot
cfg = parametros_generales(modoEjecucion);
cfg.semilla = semilla;
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
        error('ejecutar_rrt_mpc:FormatoAnimacionNoValido', ...
            'El formato debe ser mp4, m4v, avi o gif.');
    end

    nombreAnimacion = "rrt_mpc_"+string(escenario.id)+ ...
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
%[text] ## Simulacion RRT\* + MPC
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
%[text] RRT\* y el criterio global reciben una lista dinamica vacia. Los obstaculos moviles se delegan exclusivamente al MPC local.
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
        if cfg.visual.activa %[output:group:5fa4f152]
            relojVisualizacion = tic;

            % La figura se crea por primera vez dentro del bucle. No existe
            % una figura separada de inicializacion ni un fotograma cero.
            if ~graficosInicializados
                graficos = inicializar_figura( ... %[output:26b7d27e]
                    escenario,cfg,cfg.nombreArquitectura); %[output:26b7d27e]

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
        end %[output:group:5fa4f152]
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

    nombreBase = "rrt_mpc_"+string(escenario.id)+ ...
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
disp(" "); %[output:8e24e642]
disp("Resumen de la ejecucion RRT* + MPC"); %[output:5c613be2]
disp(filaResultado); %[output:08bd6676]

disp("Episodios sin candidato factible MPC: "+ ... %[output:group:859b22b2] %[output:16814516]
    string(numeroEpisodiosSinCandidatoFactible)); %[output:group:859b22b2] %[output:16814516]
disp("Pasos con riesgo predicho por MPC: "+ ... %[output:group:83d11ba1] %[output:58a86ad2]
    string(nnz(riesgoPredichoPorPaso(1:pasosEjecutados)))); %[output:group:83d11ba1] %[output:58a86ad2]
disp("Pasos con colision fisica predicha por MPC: "+ ... %[output:group:3a0b2246] %[output:347b685f]
    string(nnz(colisionPredichaPorPaso(1:pasosEjecutados)))); %[output:group:3a0b2246] %[output:347b685f]

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
%[output:32ff8ad1]
%   data: {"dataType":"text","outputData":{"text":"Raiz del proyecto: \/MATLAB Drive\/TFM\n","truncated":false}}
%---
%[output:26b7d27e]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAkgAAAG2CAIAAAALDO4MAAAAB3RJTUUH6ggEExYLUTM\/pAAAIABJREFUeJzs3X1YG+eZMPobjGJbAuO4lnCMgZHWe5LFxjbEeOmysSQf7Cb29tjLKW3wcYzI9lDyxtnG5WPrnONKaq5NUgxJWqdr6t1jSakb0tCLxm8bZw\/WGwn68pa3ckxsbE7T9aIHBCQgSqgRY1xhc\/54rPGgL\/SFPuD+Xb58jWZGMzcj0D3PxzxP0tzcHCCEEEJLRXKsA0AIIYQiCRMbQgihJQUTG0IIoSUFExtCCKElBRMbQgihJQUTG0IIoSUFExtCCKElBRMbQgihJQUTG0IIoSUFExtCCKElBRMbQgihJQUTG0IIoSUFExtCCKElBRNbNBiNRpk327dv7+3tDXyf3t7e7du3e91NJpM1NzcDQG1t7eHDhx0OR5gx19bWegbgFkNtbe3Y2JhcLvcMRi6Xj42N8Q\/ocDgOHz7M38doNPo5u9t5\/UToeS66qba2llvT3NzMf5lA6AVf8GoAQHNzs0wm4z59z4sQ2mE9+TkyjcHrJ+sWHgAYjcaI\/K4GLpyfOkDc77nbRaA\/Pv9PlcP\/23e7qvy\/L8\/fc+QLJrZYmpqaOnLkiP+\/sUD2oX79619v3759ampq48aN58+fj9RXxtTUVFdXF\/eyq6trampqwXfZbLaysjLu79BoNG7btq27u5u\/T1VVVchBjo2NWSwW7lxtbW1+dq6trW1oaAjhLGjxNDc3V1VVxTqKyNNoNG6\/5wDQ3NzM\/w1saGjgcpvRaORfh7a2Ni63jY2NlZWV2Ww2+tLtbwr5gYktetLS0i5cuNDv0t3dnZWVNTU1ZTAYAtwnLy\/v6tWrdFN9fT0AFBUVXbt2ja759a9\/TbOOxWIpLS195513UlNTvUZC7xADv2\/t7Oyk6cfhcHR2dnrd5+zZs1zYFy5cSEtL4\/LN2NjYyy+\/7BZtaWkpAHR3d58\/fz6oy0hdu3bNZrNlZWU988wz\/AiXs+rq6v7+fu5zb2xs7O\/vb2xsjHVccYT+BV29ejUvLy\/iB6elK897rLGxsZaWFgCor6\/n\/nJbWlrGxsYcDse5c+cAoLS0tL+\/\/+zZswBw6dIl+ofZ1tZGf8m7u7vpV8GC93CIwsQWMxKJpLCwMPx9+DQaDQA89thjL774YkS+6Ddt2pSWljY8PMyyLABYrdYbN26kpaVt2rTJz7ukUumWLVsA4ObNm8D7+3zzzTe5XKvRaIqKiurr66urq0MI7N\/\/\/d8BoLCw8Gtf+1paWtqNGzesVqvnbrReiH4XtLW1cRVEbpW6blVDfG51rW578uuX+DVFXJWX0Wjk3s7VFdOXXGmVq6N7\/\/33A6mn9VU9tWBVJL\/ydnR01O2w\/J\/FT\/2zn2sVOK4M3d3dvW3bNnpMfmW1Zw389u3bf\/azn9Ew6A\/l50N0q\/fmLoLXqsgFP8Tu7m7uaL7qA3t7e\/fu3Wuz2eifDH8TdxNGb+ZKS0uzsrImJydHR0e5P6iKigoAKCoqKioqmpqaGh0d5W4iy8vLJRKJRCIpLy8H198U8g8TW8xwlWmbN28OZx9Oc3PzpUuXTpw48fLLLw8PD9MkF6ZNmzYVFRXZbLZr166Bqx6yqKjIf2KjN5g0bO7vs7CwUCKRcPukpqa+8847oWU17rI8+eSTNIm61Zf619vbe+TIEX6FKr9qyG1P+m3luSf96uTXL9lstr179\/K\/Maempqqqqri3nzlzZvfu3dwdt1tptbu7+zvf+Q73sqqqKpCQPE\/qS21tLXdqm81WVVXFvwJudWX8CnC3y9XQ0LAYhYaxsbEDBw5wlXhTU1MHDx7kZ\/epqamTJ0\/SMJ588kk\/H6LboWB+\/R5fgB\/i4cOHuaPZbDZfd43p6ekXLlz48Y9\/HMjPS7OXr61+stfIyAhWTiwIE1v00L9V7t6QJgzuPi7wfXyprq6mdSwSiaSjo8OzDoq7M6V1+vxz+SkfPPXUU+AqIdG\/N7rGTVVVFRc2Pb5b2IHk5gBxt8Dbtm1LTU3dvXs3uOp23Pak6ZO7U+7v7y8pKTEYDFNTU7Re6Nq1a0VFReDjq4TuSeuC3CqRuORN62Dpcaampl599VX+9w49KVelnJSU1N3dzZ3UrQbVLSTPn8jhcLz66qtc8DQkt9psr3p7ey9duuR5CorWlXHV4LQmmfvmdbsItLosTI2Njfy69Orq6oaGBpvNRi8Xd5Zz587xrw9Xle3\/Q6Q1BNyP41a\/xxfgh8idl8bstXogLy+vs7PTaw0njSozM1MoFLqtHx0dnZqaWrt2bUZGhtsmlmWHh4fB4w+Hqz5BfmBii6WzZ892dHTwyzGh7bOotm\/fnpWVZbFYrl+\/brFYsrKytm\/fvuC7SktLFy9srh6SHr+4uJg26dFi5YJo41N1dXVtba1nlxaOw+EYGRkBV10QuBqx6M9FYygtLS0pKQGA1NTUZ599Fjy+9Z588kkAEAqFmZmZXMxcMuYrKio6cuQIPdSJEyfS0tJobRV\/H+7LrqGhgd5D0NKGxWLx36eAFrW5Ww3uFHQrvRO6evVqRkaGXC4\/ePAgVxLiCsfcRSgpKQnkNiso3KWm1cXcvZHbxXz22We5qmw\/HyJNJHv37qVppqSkpL+\/32u7WoAfInde+psW2Z8dLQZMbNHD3ULS+z5w\/V0Fu0\/I6Pcydw\/L76hC\/7C9EolEmZmZNptNr9fbbLbMzEyRSOS5G7\/ziNc+C5FqG+C+arkvQe6LOMBrxfWu9l+l5uuWGXhfxPxNGRkZbt96aWlpbnfiYRZbR0dHJycnQ367Z6GBQ5vfaA2B1x0iWOD2xF3qwPn6EL1+NF6F\/CEGix7fs6S1efNmejrPO5jNmzdzN0Nufzh+PkTEwcQWA9XV1fSet62tzVdTfCD7RIdQKKTFC\/oNsnv37qD+rrjSiVupgjZvhPBsGa1o8rppwYILeHTRdKuU4\/P1zQIAqampGzdudNtEq5UC\/jncBVLFlJGRsXbtWvC4jViwcOzru5UyGo30w6U1e7Qqkm7ycxEiiDsLV8VK8YtZ\/ATj50P0+tF4tRgfYiDo3YnXfOnnXoqLc+PGjb56OyMOJrbYoH0CAeDMmTO+Wv4D2Sc6uBqYtLS04uLiYN9Ou4G5tbrTx32CTdtcVxT+kwOeDxj4wRV6aP0S7ZbmdU\/ui49r6+I\/JkG\/etra2mjzJNdve8uWLVKpNPCfiMMFzzWkeTa9cAmAa3zy\/xQ2h5YMPE9Bt9JvTK6ikv+ooteLEPHOI55n8Xyam8\/\/h0g\/Gq5Rjes86dmQHPEP0att27bxe+rTy0s\/XK7rE20lpY3HNOe5NR4H1Y8MpcQ6gGWKNnLQbl0Gg8Hrw0aB7BMa2uoQ+P70z6+7uzu0P3iJRHLy5Mmqqirat5u\/iWtYChD3FbZ7927+fSsXYWdnp68DtrW1tbW1HT9+nO4ZyNPBFRUVly5dstls\/FIdbbyRSqWdnZ1ux0lLSztx4kTIN9QNDQ38HnonT56USCT8SiraCES7PHBXkuss7kdeXt7evXvb2trcTkFxjZReC69eL0Kk0B+ktLTU61n4jWp83Mft9UMsLS1taWmx2WwHDx7kVtKe9G6dPo4cORLxD9ET7anf4EJXcm2W9AOlv5x0E9c6yP0g3DUJsB8ZwhJbzOTl5T333HPAu2EMbZ8o4G6oQ64GKSkp8az0q6+v9\/MUuVf0btez4Mjd3nrtsVZfX5+VlUWXBQLBm2++yb2sr6+nLY5eqzHz8vK6urr4YdfX19M7DNrfkmsKBYCioqKurq6Qn\/wtKip65513uDrAs2fPem34LCkp4VcVpqWlnT9\/PpCTNjY2ct+JWVlZZ8+e5Q6Sl5fX1NTEHfDChQt0T9pmmZeXd\/78eW7n+vr6iHy3HjlyhH9h8\/LyLl26xH0u4PsKAEBqaqqfD5H2heEH6Wu8goh\/iL5UV1fzz8J\/fLOkpITf0bS0tJS7hZVIJK2trdyPmZWV1draGsN+ZIlkDqH4U1NTs23btmvXrsU6kGg4c+aMVCotLy+fmpoK5zg1NTUROU446M9y6dKlWAWA0NzcHFZFIpTw+E8rY+cChLAqMjjcM86+RuXBEbhR9HF9+bKysvhVXggtT1hiC0Jzc3NLS0t3d7dEIjEajceOHaNV3kajkVvf3Nz84osvnj17Fu+aUYCqq6tDG1qME2xvIISWNiyxBYp2NOf6MhUVFWVmZl67do12EebWl5aWTk1NeR2Ql5LJZNELOu75uhqNjY2LNAR73Foavxh0EAA\/z\/sHYmlciojASxEaLLEFinag4l5yj1LSBe7hEqFQmJaWthgdqxBCCAUCE1uI2traMjMzi4qKWJZds2YN9yAt1zPej6SkpMUPMDFIpVK8GhReCg5eCg69FHNzc7EOJMFgYgtFc3PzmTNnzp8\/n5qaGuBI2\/wqhQgOarAE4NXg4KXg4KXgSKVS+u2BzaiBw8QWNC6rcZWNt27dGh0dpS\/pyKqew95wv5QymcxqteItGGW1WvErjMJLwcFLwcGvi9Bg55Hg1NbWtrS0XLp0ictqdPg+bhxVlmWnpqZCGFARIYRQRGBiC0Jzc7PFYnEb1YYO38cN3trW1paWlob3mwghFCtYFRkoOsuw2yCtdMy3kpKSmzdv0vV0PDd8iA0hhGIFE1ug6MiqvraG\/4wtQgihiMCqSIQQQksKJjaEEEJLCiY2hBBCSwomNoQQQksKJjaEEEJLCiY2hBBCSwomNoQQQksKJjaEEEJLCiY2hBBCSwomNoQQQksKJjaEEEJLCiY2hBCKDIfDoVAoklxycnI+++wz\/g4ff\/yxSCTidqioqHA7wmeffZaTk+O56Qc\/+EFSUtIPfvCDRf8ZlgRMbAghFAEff\/xxRkYGf6j0wcHBzZs3f\/zxx\/Tlr371q507d7Isy+3w9ttvuyW\/y5cvDw4OAsAvfvEL7o0oWJjYEEIoXA6Ho6amhmXZo0ePzs3Nzc3NjYyMZGdnsyz7ox\/9CAA+++yzY8eOAcBrr73G32FwcPDtt9\/mDtLU1CQUCg8ePMiyrNFojOWPlMgwsSGEULg+\/fRTi8Uil8t\/\/OMf0zWPPPJIW1vbwYMH6RpaFDt69Og\/\/dM\/8XcQCoX\/8i\/\/Qgtt9CCFhYUvvviiUCj88MMPY\/XjJDpMbAghFK6RkRGWZZ966in+JMOPP\/74+++\/T9f09fUBQG5uLv9djz76aGFhIffSaDTSg+zcubOwsNBisUQr\/KUGExtCCEUGzV4A8Ktf\/cpPFxKvHA7Hhx9+KBQKS0pKUlNTacXmIse7ZOEM2gghFK6NGzcKhcKBgQGHw8EvtHFoWY3LfBSte1y\/fj23zLLszp07oxPzEoYlNoQQCpLZ7LaCVip2dHQ8\/\/zzAPDVr36V6x5Cd9i5c2d2dvbbb7\/Nddn\/7LPPSktLWZZVKBSPPPLIj370IyyiRQomNoQQCgYhoFS6rUtNTaUdGt9++22uEnLjxo2Dg4M0bz3yyCNvvfUWAHz3u9\/lb83Ozn7ttdc+++wzs9ksFAovX7485\/Laa6+5nYV7L4WPtfmCiQ0hhIJBCICXQtvjjz8+Ojoql8u5NdnZ2SMjIwaDgb786le\/evnyZaFQyO1w9OjRgYGBRx55hPaZLCwsfPTRR7mtJSUldCGQJjrElzQ3NxfrGJYXmUxmtVrxslNWq1UqlcY6iriAl4IT75dCqwWNBlQq0OkW+1T4dREaLLEhhFAwaFlNr49tFMgPTGwIIRQws\/lBJaRHbSSKE5jYEEIoYLyhIMHVeIbiDSa2UBiNxsOHDzscDm5NbW2tTCaTyWRyuXxsbCyGsSGEFhG\/lIa1kfEKE1vQjEZjVVUVf01zc7PFYunu7u7v7y8vL3\/xxRf5OQ8hlHi0WkhK8vxHzEQLGiWYtKABAK\/7gFYb4+CXPRx5JAhjY2NlZWWTk5N79+6dmpri1t+8ebOwsFAikQBAcXHxBx98wLKs19EHEEKJQa0GudzteTUCjBJMBBgAMINCDxUmUDJAHuzBMKBSgVodzUiRJyyxBef48eNXr17Nz8\/nr9y8ebPFYqE1kF1dXWlpafxHVRBCCUmhAKsVFApuhQFUNKtRBBgt8HIYw4DJhFktHmCJLQgSieTQoUOe66urqzdv3lxUVAQApaWl77zzjuc+MpmMWxYIBFardfHiTCBDQ0OxDiFe4KXgxNelOHcO3noLTp8GgEvwmADm\/eX+B6y2ggAyM6G0FI4dg7k5iPSftlQqpd8e\/f39kT3yEoaJLQJqa2tHRkauXbuWmpra3Nwsl8tbW1tpzSSH+6WUyWROpzOunz+NLrwUHLwUnPi6FE1N8MILoFT+JbndBfMC2wtvS5lM0On4BbvIwge0Q4BVkeEaGxuzWCzPPvssbVQ7cuRIZmZmW1tbrONCCEUOwwDDqGFerxAGSAXogZDFy2ooNJjYEEJoIYSA2cwAsfJKbDqovN9zhI4eieIGJrZwSSSSwsLCc+fO0S7+58+fHx4eLi0tjXVcCKHIcT2LzQDhukE+6A+JQ5DEGUxsEdDY2Lhx48Zt27bJZLIzZ8689dZbbg1sCKHE5i11PeghiUOQxBkc3T\/acLhuvngfxz2K8FJw4u5SEAI0HoYBlUppVtM0p1OZVXrXg25WKzBMxM+MXxehwV6RCEUJIUTpMUHl0qBQKHSLP4dLzNACGX1MjWEY4lovV4DaCpWV90dGVqliFSByg1WRCEUPIWRmZibWUUTY559\/HusQFhkhoNF4lskGBlzZTqOZNzgyijUssSEUVRs2bFi7dq3neqfTKRAIoh9P+CYnJ2MdwiJTq\/kpjVt80BdSrQZCgJDFqI1EIcDEhhBCfs1PVzk5C++DYgurIhFCKBT49FrcwsSGEEJB8FIVieIMJjaEEAoCJrb4h21sCC0FTU1NBQUFdPnKlSs1NTV0OT8\/\/6WXXlq\/fr3behQybE2Lf1hiQyjh1dXVcVkNAAoKCpqammB+VqPr6+rqYhPiEoWFtviEiQ2hhJebmwsAFy9eVCqVFy9eBIDs7Oz8\/PySkpL169ePj49\/5zvfoevpnihMWGiLc1gViVDCm5iYYBhGoVBcvXr11KlTp06douuPHDkCAL\/73e96enp6enq49ShS8NG1+IQlNoQS3vXr151Op1AoPHHihMlk4ka3WrdundPp3LFjh8lk4q9HYcL+I3EOExtCCU+n0zU2NrIsS18yDNPa2pqfnw8AAoFg48aN3HrMbRGBpbQ4h4kNoaWgvb39wIEDXBtbenr6jh07JiYmAODKlStKpfLtt992Op0SiWTfvn2xDnbpGBiIdQTIG0xsCCW2\/Pz81tbW9vb2yspKADAajePj43QTf3ji4eFhp9MZmxCXHKyKjHPYeQShxNbT0zM4OFhQUHD06NGjR4\/SlePj45988gkA7Nq1q6CgwGQy0fWEkPb29pjFulR4Hy4SxQ0ssSGU8Gpqaq5cucK9HB8ff+WVV2hPyH\/913\/l2t4IIbRUhyIFS2zxCUtsCC0FvoYUaW9vxyJaxGFVZJzDEhtCCAUHE1ucw8SGEELBwe7+cQ6rIhFCKHSVlVoAwjAMAMjlcoVCEeOAECY2hBAKHCHEYDCYzWYAHQADAB+1vpcltH\/EsoPT0wDAMIxKpaqoqGCwWBc7WBWJEEILI4RotVqpVHru1KlHCMkW2en60zvLLsjlV556avxrX+vZv\/\/rQqFGo5FKpVqtNrYBL2dYYkMoqgghq1at8lyflJQ0NzcX\/XjCNzMzE+sQFh0hRKlU3rPb63Nz63NzAeCFy9OD0wAANlbM7ZYlFNIdGvr6NBqNXq\/X6XRYORl9mNhCYTQaz507d\/bs2dTUVLqmubm5oaGBLp89e7akpCR20aE4RQfg97XV6XQKBIIohhNJS7vajRAilUqzRaILcnmWUBjIW+pzc8sZ5pjFolQqTSYT5rYow8QWNKPRWFVVVVRUxF9z5syZCxcu5OXl9fb2Pv\/88xkZGXl5eTEMEsUnbgQQT1arVSqVRjMYFAguq1156in++izh\/arIwWmJ1zdmCYUX5PIXLl\/G3BZ9mNiCMDY2VlZWNjk5uXfv3qmpKbrS4XCcO3fuueeeo5ksLy+vs7MzpmEihCKD1kB6ZjUAyBaN0QV+VaSn0zt3Dk5PY26LMuw8Epzjx49fvXqVTghCWa3WoaGh4uLiGEaFEFoMWq32nt1+QS4P5yAX5PJisRj7kkQTJrYgSCSSQ4cOea5PT08fHR3dvn27TCbbvn17b29v9GNDCEUWIUSv19fn5nptV+NVRforsVH1ublms1mv10c2QuQLVkVGwODg4Llz57q6ulJTU41G45EjR86fP+\/WxiaTybhlgUBgtVqjHmY8GhoainUI8QIvBSdOLsUzzzyTvWbN32RkDHrr9ml33hIIrABguzPjdQe+rLS0v87I0Gq18uALf1KplH579Pf3B\/veZQsTWwRkZ2e\/+eabtIdkUVHRli1burq63BIb90spk8mcTid2E+DgpeDgpeDE\/FIQQrq6uupzc7N5z2asfeONFZs20eWHCTilAACDTmn2\/Oc31nzve4K8vLtDQ5PHj3Mrn8nOPmaxDAwMBNvSZrVaE\/Q5kBjCqshwZWRkAMDo6GisA0EIRQwhBACKxferGWdZdrKv7+7t29wO\/AccBFu3csvCb3xD8Fd\/5XnAp3NyskUig8GwKOGi+bDEFi6JRHLgwIFXX32VPtbW3d1948aNEydORC2AyspKkrBjjIf28JZCoVCr1YsRD0JUR0dHtkjEJbYZu32yry\/T6QSAL372s4lXXlmbm5u97v8ZnFgDAGNFTz98\/f8GAMHWrSv37IEU79+rxWJx4v6pJhZMbBFQXV0NANu2bQOAtLQ0zwa2RWU2mycnJ70OZhH\/QhhuY3JyEgAwsaFFZTab+X1GUnNyZuz3e4vcnZmZ7OtzDAzMTk8ArAGAwVvrHgYAANE\/\/EPymjWzn36a8uijnscsFouPmc2LHzvCxBaS6upqmsz8rImmtWvXJujQDyGU2H7\/+98vUjAIcQghXxfP6+64fufOFKEQAFY+\/HCKSDQ7Pf0IkBFgAGBgTLhz69aVTzyxYtMmZ2\/vPbvda2JDUYNtbAghFJCklBQAeGjt2k1PPbVp\/36ZeJrbJNi+\/aEvf\/nexMTttjb\/B8HayCjAxIYQQl7YWNbP1hShMEUkosvkP++m\/MVfJK1enbxu3Rq1euWePQCwYtOmh3\/yE36\/EipBK1cSC1ZFIoTiyy9\/+ct169Yt6in8D0gNAAqF4s5C7WE5DAABALD2TfvfkxqcnsasFh2Y2BBC8eW73\/2u0+lc1FOoVKoFnydrIeT0zp2e61fu2UPLZI\/qAcwAAHdHR299\/5+5HVKfe27lnj1uz7EBQJfdzmzZEmbkKBCY2BBCcYdhmMUr3HR3dy+4j1wu1+v1NpYNZJ6agWEBbF5gHxvLdtntGhwHOSowsaFANTU1FRQU0OUrV67U1NTQ5bq6uv3793O7EUIqKytjEB9CkaNSqSorKxv6+viFNrcSWOpMEUCN53sdZ844zpxxWzk4PQ0AIQyphUKAnUdQQOrq6risBgAFBQVNTU10OTc3N0ZBIbSIVCpVi98ejNmi+0+22VjvU7LxNfT1KRQKnLkmOjCxoYDQ7HXx4kWlUnnx4kUAyM7Ozs\/Pz8\/PT01NZVn21VdfVSqVSqUSi2toadDpdADwwuXLvnbIEt6fkm3BAf7fHRjosttxVIGowcSGAjIxMQEACoVi3759p06dUiqVZWVlPT09YrFYKBQKhcITJ06YTCb6XYDQ0qDRaFoI6XKNOeLmIddsw+B3ulEbyzb09QXSXQVFCiY2FJDr1687nU7PBJaZmckfOoRhGMxtaMlQq9UKheKFy5d95bYNGxaYsMbGsscslmSxGItr0YSJDQVEp9M1NjayrkdWGYZpbW3Nz89fv369QCAghCiVyldffZVlWYlEsm\/fvthGi1CkmEwmWWGhn9xGDU57aWajWW1YJNLpdPgEWzRhYkOBam9vP3DgANfGlp6evmPHDlotSdvV7HY7y7ICgSAzMzPWwSIUMTqdTlZYeLCjo6Gvz20TN4+2Z1WkjWUPdnQMi0QmkwkrIaMMExtaWH5+fmtra3t7O01gRqNxfHycbmpqajKZTLSH5I4dO9LT051O5\/DwcCzDRSiiGIYxmUwajaahr6\/gww\/fHRjgNnmtirSx7AuXL+dfvJgsFptMJiyrRR8+x4YW1tPTMzg4WFBQcPTo0aNHj9KV4+Pjn3zyCQDk5eUVFBSYTCa6vre3t729PWaxIrQ41Gp1RUWFUqk8ZrE09PU9nZNTLBY\/NHWLbu2yi7KE9i77\/X8Mw2g0GmxXixVMbCggNTU1\/Ae0x8fHX3nllZ6enp6eHgAoLy+nXUj4D24jtMQwDGO1WgkhBoNBo9EAwIYNFoDHAaCFQAvpYBhGceDAN+VylUoV00iXO0xsKFC+MpZOp8OekGj5YBhGrVar1WpCyA9\/OPnmmwAACoVCp7NirWOcwMSGEEKhYBgmJ2eS9yqGsaB5sPMIQgiFiEtmOHtoXMHEhhBCIcLEFp8wsSGEUIgWHHkExQS2sS0Fk5OTZNncMU5OTi68E0JRRwhgM1ucwBJbwmMYZu3atbGOIno2bNgQ6xAQum\/VqlVYaItDWGJLeNyT0YnIarVKpdJYR4FQiFatWsUtY4ktfmCJDSGEQseV2JZNa0ACwMSGEEIhwqrI+ISJLRRGo\/Hw4cMOh8Nt\/djYmFwuNxqNMYnKDSFEr9dXVlYqlUqpVJqUlCSVSqVSqVKp1Gq1y6ezCULRwRsbGcUYJragGY3Gqqoqr5saGhpsNluU4\/FECNFqtVKptLKy8qPW1kcI+bpQWJ+b+3Wh8OtCofPGDY1GQ5OcVquNdbAIJTZ8lC0OYeeRIIyNjZWVlU1OTu7du3eKNys8ZTQaR0ZGsrKyYhIbR6vVajSabJGoPje3WCwuFnufsb7Lbn93YECj0ej1ep1Oh\/NFIRSajAysioxyaryYAAAgAElEQVQ7WGILzvHjx69evZqfn++2fmxs7Ny5cy+99FJMoqJoQU2j0dTn5l556ima2HztXCwWn965s2f\/\/nt2u1Kp1Ov1UYwUoaWD6xiJJbb4gSW2IEgkkkOHDnnd1NbWtnv3bonEy\/Tw0UEIqays7LdY3iosfDonJ8B3ZQmFV5566oXLl+kMojjXBkLBclVFErPZDKCKYSSIg4ktAnp7e3t6epqamliW9bWPTCbjlgUCgdVqjWAAw8PD3\/3ud229vWeKih5ZvXpwJri6kbqtW28DVFVVrV69eteuXREMbEFDQ0PRPF08w0vBSUlJAQCn07l4x2dZNiJ\/g8PDw\/\/tvxkEAgN9KZVqS0tLjx07Fv6ROVKplH579Pf3R\/CwSxsmtnA5HI633nrr2LFjqampfhIb90spk8mcTmdkn0p+++23bZ98cnrnzr9++OHQjvBvO3cenJ5++umnrdZozymFD2hz8FJQs7OzAECnrl2k4wuFwvCvNiHk5MmTv73W\/9C2\/2vmD\/q7U2R0Cl5\/\/fUrV65EcNgEq9U6NzcXqaMtE9jGFi6r1drd3X3w4EGZTFZUVGSz2aqqqpqbm6MWACFEo9HQierDOc4FuTxbJKJ1kgihBWm12v\/ZS\/KeMwofVyenMQCQ+TeqXTUms9mMjdaxhYktXHl5eVevXu3v7+\/v7+\/u7s7Kyjp79mx1dXXUAqisrKR9IMM\/VH1urtlsNpvN4R8KoaWNPicqflz12Z\/+EgCcI2YAuPWQPJVRrPtfFAaDIbbhLXOY2BIbIcRsNkckqwEALfbh3yRCAZr7kvzPf4aHXC9nH2JGRiDzbyrw7jC2sI0tFNXV1V7LZBKJpKOjI5qRGAyGbJHIrRvk2jfeWLFpE3\/N3aGhyePHPXe4NzHhOH3aef06t6mcYY7p9Wq1Gie6R8gPOnbP7EoGANYD5FXN3Z4itjTmz38GgWsH\/COKFSyxJTaz2ZwlFC6424pNmx7+yU8EW7cCwJrvfY9Le8nr1qW+8AJdT9EcifebCHlHAPQAWlAQBQDc6jV8CeARAABYkcZMAaSlwfD\/MCgUCsxqMYQltsTmpx7yzkcfOc6cAYCVu3eLvvnN5HXrVpeWAsCKzEyYnb39\/vt3P\/tM9M1vJq1enbxuHf+N2SLRAA57h5AbPYABwPxghQY0mo81azbK4REFAEwBPPQQzHyqn\/iDueKELjZBIgDAxJbQaGXIgp0h73R2CrZsWblnT\/LDDzuvX\/\/iW9+i61fu3g0A9\/74xzudnfz9s4RCHCIZoQcIgAFA475aDWozmM2\/Um5+XLMiLWcEYNVUx6cf61UqFY51EFuY2JaguXv3AMDpcDgGBmbsdgC419Oz\/oknkoRCwdattEUt9bnnVu7Z49b2RmWLRJjYEHpAC6B3LTMAKoAcgAEAM5jMJi1oNR9r6MYvMczzGo1arY5FlOgBTGxL0N3bt1MAZuz2cYuFrlnDsjD\/GU\/HmTOOM2fWvvHGl1pabr\/\/Pvvzn8ciUoTint6V1RgAFQA\/Z+UAmEENajVdq5m\/FcUOdh5JYLR12uZjuJPklJRVYnEqw6wvLEx\/\/PGkhx6aY1nBli1famnhOpLc++ILSElJefRR\/hsHp6cXP3aEEgEB4EYsUHvkLdX8lxoAstgBoYBgYluCUkQiAFglFm+Qy9fv3PnwgQOriooAYPYPf3DeuHHv1q3kdetWPvGEYOvWFZmZAHDPbue\/3cayOIsNQgAA3HyFKh\/jG7utNC9eKCgIWBWZ2BiG6bLbvQ7nL9y\/X7h\/P\/fy3sTEnd\/8xnn9+t3h4eR161bu2bNyzx4AmLt923njBrebjWUHp6flcnkUgkcoNE1NTQUFBXT5ypUrNTU1dLmysrK8vJwOMnnx4sVTp06FdRrCq4T0Vcco5zW\/AUAHju8fF7DEltjUanVLAB097g4NffGtb9FuI7e+\/31nby9dP3f79vS\/\/Ru\/V2SX3c4wDJbYUNyqq6vjshoAFBQUNDU1AcC+ffu+9rWvcUMn79+\/v66uLqwzcSPwKAAYH\/uo5m8yh3VCFClYYktsKpWqsrLy3YEBfqHNs6Ojm1vf\/76vTS2EMFu2RCw+hCItNzcXXAWyurq6\/fv3Z2dn5+fnb9++XSgU0okJaZFuw4YNYZ1JAwB+i2uUmtcOh+IDJraEp1AoGiyWwCcX9aPLbu+y203YWRnFsYmJCVqpcPXq1VOnTnH1jT09Pfy6R6fTeZ03VlzQzK4FxndxjVJhYos7WBWZ8NRq9eD0dENfX5jHsbFsQ1+fQqHAekgUz65fv+50OoVC4YkTJ0wmk043b4yP\/Pz81tbWvLy8lpYWt03B4cZ8DeQ2jwn9PGgxYGJLeAqFQqPRNPT1dc3v3BisFkKGRSJ8thTFOZ1O19jYyE3qyzBMa2trfn4+fdnT01NWVtbS0lJeXh5WYjO7FpgAdlaEfh60GDCxLQVqtVqhULxw+XLIua2hr6+hr0+lUmFxDcW\/9vb2AwcOKJXKixcvAkB6evqOHTt0Op3JZKIdRoaHh51Op0Qi2bdvX4jnMANAAPWQFHYijjOY2JYIk8kkKywMIbfRGsiGvj4NDgWE4h6taWxvb6dTvRuNxvHxcbqpr68PAHbt2sV1JGFZ1h7arR5xLTDhh4xiADuPLB0mk0mpVB40m+tzcwOcetTGsscslmGRCLMaSgg9PT2Dg4MFBQVHjx49evQoXTk+Pv7JJ5+IxWKFQrF+\/frXX3+drh8cHOzp6YlGWCrsPxJfsMS2pJhMJtreVvDhh\/67k3TZ7S9cvpx\/8eKwSGQymTCroURRU1Nz5coV7uX4+Pgrr7zS09PT3t7+wx\/+kGt74z+4HTom3AOgmMAS21KjVqsrKioMBgPNcNz82tkiEQDQikr6TDfDMFhQQ4nIV8Zqb29vb2+PwAmIa4EJ+C0MAMGxIuMFJrYliGEYLr0BgNls5mbEZhiGYRiNSiWXy7GfCEKRR7CcF3uY2JYsmt4AAMtkCEWPGYeLjD1sY0MIIR9I8G8ZiHgQKGiY2BBCaD4mjPeSCMWAwoCJDSGE5mOC3F\/Ly2fmSAaCQoOJDSGEfDAHsI\/WNQ8ARRYlEBQUTGwIIeSBAYAAshSZn9W4lSimMLGFwmg0Hj582OFwcGtqa2tlMplMJpPL5WNjYzGMDSEUAQrXAvG7m\/+tKEYwsQXNaDRWVVXx19TW1o6MjFy7dq2\/v7+8vPzFF1\/k5zyEUOJhXAtmv7uRgFeiKMLn2IIwNjZWVlY2OTm5d+\/eqakpbqXFYjl58mRqaioAlJaWfvDBB1arNS8vL6bBLkd6vb6jo2Ph\/eISy7JCoTA65+KecUQ+cRP3dvh9Lo1Z\/EhQ8DCxBef48eOHDh1qbm7u7OykayQSCf\/LdHR09E9\/+lOMolvuOjo63n333VWrVsU6kFCsWLHi7t27UTjRzMzMhg0bMLEtQBXYuMaKRQ4DhQQTWxAkEsmhQ4f872MwGHbt2uVZXJPJZNyyQCCwWq2Rjy8BDQ0NRfBoLMuKRKLHHnssgseMmtnZ2ZSUaPw9Dg8PA0A8\/wbS6+B0Ohfv+CzLLnwFGIBhgJ8B\/ANApu\/dvg7wy\/lrRgEid3WlUin99ujv74\/YQZc6TGyRVFtba7FYWltbPTdxv5QymczpdEql0uiGFr8ieCmEQuHdu3cFAkGkDhhl0Yw8nn8DZ2dnYTGvxuzsrFAoXPgKKAD0AABwE+Bvfe\/2FMB789fcBojc1bVarXNzcxE73PKAnUcihstqEokk1rEghMIW4LzYqkUNAoUCE1sEOByOw4cPj4yMfPDBB5jVEFoiGNeC7w5JhBCz2UwUZN5aHC4y1jCxRYBGowGAs2fP0o6RCKGlgPG5xWw2V1ZWJiUlSaVSpVJpMBvctnITRaGYwDa2cPX29l66dGlqamrbtm3cyrNnz5aUlMQwKoRQuIiXdXq9XqvVEkKyRaJyhikWi4vF4s329WDhvc9MKs2VDMPodDqc9TAmMLGForq6urq6mi7n5eVdvXo1tvGg+NHU1FRQUECXr1y54jbX8759+7797W8LBIKWlhadTheLAFHwGAAAQgidmL5YLP6hXF4sFnPb7wnt\/N3LGWZH7v5jFotSqVSpVPhBRx9WRSIUMXV1dVxWA4CCgoKmpib+DuXl5VF7ChuFS+takAMhRKlUnjt1qj4398L8rAYAd0Xuo+hlCYUX5PK3Cgv1en08d0BdqjCxIRQxubm5AHDx4kWlUnnx4kUAyM7Ozs\/Pp1vr6uoYholheCgIxDWYFgOEIUql8p7dfkEur8\/N9dzXrcTGeTonp2f\/fkII5rYow8SGUMRMTEwAgEKh2Ldv36lTp5RKZVlZWU9PDwBUVlbu3bt3ZGRkfHw81mGiALiGHSEMqayspFkty0dpO5kVe10PAFlCIc1tSqVyMcJEXmFiQyhirl+\/7nQ6hULhiRMnTCYT17iSn5+\/f\/9+p9P5\/vvvxzZCFBDtg+KaQWHot1j8ZDUAWEX8JS1aLWk2m7VarZ\/dUARhYkMoYnQ6XWNjI8uy9CXDMK2trfn5+UeOHFm\/fr3ZbL5582ZsI0QL07umWGOAqIlGo6nPzfWT1QAgxT6vftKzZrJYLK7PzdVoNISQSIaKfMDEhlAktbe3HzhwgGtjS09PLy0tpcNX7t+\/\/\/XXX1+\/fr1AIDh69GhdXV2sg0UeCG\/sYxVUGiqzRaKnc3L8vCOZFQvsWxY8cH1ubrZIhIW26MDEhlBk5Ofnt7a2tre3V1ZWAoDRaMTmtARDeFlNA6SCmM1mr71F+IR9X3db49lJkno6J0ev14cXIgoIPseGUGT09PQMDg4WFBQcPXr06NGjdOX4+HhbW9vJkyfpy\/z8\/Jdeeik9PR2fY4tHBlfTmgJADWa92bO4tvaNN1Zs2vTgNfEy3nFqw\/OpiucB4M5HHznOnOHW1+fmNvT16fV6lUq1CNGjB7DEhlDE1NTUXLlyhXs5Pj7+yiuv0F6RKK4RAO2DpjVQAwAYDAb\/TWsAC8yvvXL3buE3vsFfky0SJe5cuAkES2wIRZLbUCNuenp6ysrKohYMCggBMPCymur+9KGEkK+Lvffjp0WxZFac3v1aMqwBhjf+FgN37n3kKDtDy3Ypjz7Kf2OxjwOiyMISG0JoeXPLaq6pxRfswSiwb0meWAMAoIC5L9+5vzbrnvPGDQC498UXAHDPPq+HZJZQiOMjRwEmNoTQMkZ4WU33IKtR2SKRr\/cls+JUy\/MAMCdm4chdbv0Xv2++09m5cvfulM2b527fpklu3gmxx\/\/iw8SGEFrGuAlnFPdrIANEsxoA3NtiB\/mDGa7t9tap1atXq1SwYoWjtfVOZ6fbG3FYtShYdm1sY2NjZWVlNpttwT2zsrKwmRehJc7sWqhw38IwzOD0tNc3rfh\/H0mx\/xUA3BPZZ0\/+54qUHID7hTbRf8nd8NpryStXfv6DH4x873vM177Gf6ONZTGxRcGyS2zUgvOlGY3Gl19+OWrxoEiZmZnBqh7\/Jicn165dG+so4o\/CY4VCYfPaHkYgpeGv6OKdvf9j5Y4ngEDSb1fSNRsbG5MEgj\/+y79MnD691uMZuMHpaUEkg0beYVUkWjoYhtmwYUOso4h3mNXmIQDgfbJshmFaPG+ShlbwhyZZ\/cuDyevWzTXf5rYnrV4NKSlf+sd\/3DI29hc3bqz53ve4TTaW7bLbcerRKFh2JTaJRMJVMPb29h45cmRqaoq\/Q1ZWVmtra0lJCU6BnXDUarVarV54v7hktVpxcpOYIV7WyeVyAOiy2\/l99JPfE3HjI4MOAODu0NC9c38SgGtULeI9TdJDcYdFi2rZJTaOw+F49dVX9+7d29jYGOtYEEIxogDQA4CXhKRQKBQKxbuEcIlN2Pf1pF+sAYB7Irsj58fOshsAILBvWWPXcO\/64vn\/4mt6thZC6DEj+xMgT8u3KpJl2eHh4SeffDLWgSCEYocrPpm9bKyoqGghhJa0Vg4oVvfdf7j+To7ZKb7fj59b6d+7AwNddnvi1igkluVbYhMKhZmZmbGOAiHkxczMzMzMTDTOpHINfKwFUHlsVKkMBsMLFsuN3B9w\/ftv57ayue\/R5QCH9rexbENfHxbXomb5JrbU1NRnn3325Zdf3rZtm0QiiXU4CKEHPv\/8888\/\/zxKJ1MAmF3DRXoUqHQ6nUFq8JrVgPc0mx82lj1msSSLxTjsddQs38QGABkZGZOTk0VFRfyVtPMIpjqEYuWnP\/1pRkbGop5i3sNkalc9pMYjsRFgDIzatdYtqwnsW2hx7Z7Iflc4dn95fgObjWVbCBkWiXQ6HT7BFjXLN7HRziPPPfdcdXV1rGNBCD2wa9euqHYQVQCoXF1I9LwKScIbRhJAC9pL9h9fcDXKJbNirnWNzX3Pc1Y2ALCx7MGOjmSxWK1WYyVkNC33ziObN2+OdSAIoVjjhh3hxhrSAkh5w0hqQG6Sd9ntBR9+2NDXBwCriJIrrt3JMSdPi+kyfTdtVMu\/eDFZLDaZTDgBW5Qt38QWTucRo9F4+PBhh8PBrWlubpbJZDKZTC6Xj415nz8XIRSnFK6+\/noAAqB8UFC7Pz2bGhQKhdVqfbaurqGv7\/iHLC2u3RPZHTt\/nMzefx5gcHq6y26nKe09ltVoNFarFWsgY2BuGbt27drf\/d3fjY6OBvWuS5cuSaXS8vLyqakpbs3u3bvpcc6cOcPf5InWsYQT9lLS398f6xDiBV4KTmwuhWpuDjz+qebmrO47Wk1WbgcNaLJFov9DXEhfmsAEAAzD0JQWflD4dRGa5dvGNjY2duzYMZvNFnjnETqA8uTk5N69e7nxShwOx7lz58rLy+lbSktLP\/jgA6vVmpeXF4WfAqH4wR+lM2GKKQSAAHR4PMfGAOi8jfdPgKlk7i+rABh4FurAfP\/tCoXCpDZhc1rMLd\/Exh9bK3DHjx8\/dOhQc3Nzp2s2Cre2OqFQmJaW1tXVhYkNLWXENVQHA4QQg8FgNpvdptBkGEahUDAME9WnkgkAAJgBBlzLxO\/OnluZedONuu+v5A0vqYP7HSblrryoAMxq8WD5JrYQSCSSQ4cOea5fs2YN1zs5NTV148aN0Y0rSgghER84f3R0dGBggL8GvxfiHQGofFC+IQyREmm2SFQsFr9VWJglFNL1NpYdnJ7u+uADvd2u1+tVKtXipjcCYAYweB9AJDheC2rgkdVMvE3cHXJO2GdHkbDsEhutTjx58mQg09ZEcD42mUzGLQsEAqvVGqkjR83vfve7Z555JrLHTElJmZ2d5a\/56U9\/umvXrsieJSEMDQ3FOoQADAM8AzAMIHiwpmPNhbTd\/+q2Y1Za2t8APC2TfXb79gcjI\/\/8z\/+s1+tfe+21QD7c4C7FMMAvAU67XgY1K0ym699fA\/xPgF8CAMAogOdf51uuUwgAMgF+CjDH2+2S67wTAFaAYYDvAgwDvAYQ9u+yVCql3x79\/f3hHmvZWHaJbTHcunVrdHSU1j06HI6RkRHPpwi4X0qZTOZ0OhNxHPeBgQGn0\/nYY4+tWrUqUse8d+9ecvL9rrkzMzO\/\/\/3vMzIyEvHiRES8\/+AE4Jn7RRYCpF9k2TNdBgBSp9QxaruTY\/b6puxVq\/764Yf\/T6n0mMXy9NNPazSaQIpugV4KM0Dl\/OpEBkABIHf1cmR8vNFzvRLACQAA35i\/ngAYAF7nvdHk8XaB6701AKd58TwNYApuYm5PtBNKWIdYfpZpYquqqlpwn6ysrEAORR8buHnzJi0Csiw7NTVVXFwcbojxau3atRFMbE6nUyC4f48dpbEBUWi0D3rAEyBHRQd+JGduTd9Y06EBgFTL807xDV+j2gNAllB4QS5v6OvTaDQAEJlqSe38TvkqAHmoWYTAg8lo+Ct5D2gDeBuahP9e7iWfIdzEhkKw7BJbaH1G\/ODGnCwtLZVIJG1tbWlpafF+641Q4Mi873cCRC7aeuWppwDAKbzhFN+gzymnWp6\/Jdf4OgZVn5ubLRId02jkcnm4jan8rKYA0PkunAWCuBYY10szgHb+epWPHiVmXhjE1adG5XokTg+gDi82FLxll9gWQ0lJyc2bN+ljA\/RpgdTU1FgHhVDYiHuRRQvac6JTNKtRt+SaL\/2iFVwDJ3KTufjydE5Ol92uVCpNpjC6xet5UWl85JugGFwLFQBaV06iGN8pDVxDJ4PrOW7F\/E16AAAwe5k3AC2q5TvySDiqq6vfeecdfvaqrq7u7+\/v7+\/v6OjAAZRRwiMAevcBOPSMXgOa0zt3uu3LFdTWdGgCmcPl9M6dxWKxVqsNPbZK17ImElmNuDIQAFQCaOZnNZ3fUxhcOys8qhy5md4iWUOEAoKJDSHEQ1zDJPI7ZaiA6IiBMZQzDDedNMcpvsEV1FIvLzyNCwCUM4znc2+B4hKiKhJZDXw8IaAAMAFY\/baQ6XmDSXpG4ueNaJFhYkMIufKZkjfyL6UCMAHogAAxm81P53h\/UOuWXENzW\/K0mHYn8e\/pnJxskchgMCy4p5c49QDgKkuFiQBoeeU\/cFU8mgLozUh4b1R5a0UjvGOi6MI2NoSWJcIbTUrvsZUBUABUPPhy12q19Cls\/l5r33hjxaZNDw4oBQAQ2Les6dBw9ZMrd+8WffObc7dvO06fdl6\/zr23Pjf3mF4f9NybXCoMs6xGPHo8MgAqgIrA8hABULqWVT6C4UKVe9uKFtOyK7GNjY3V1tbyB+ZHaMkiAGYAPYDeVTRRAigBkgCkAEqASo+sxgBoAKzuA3AQQnwV1x680TUYB81tAJD63HOpL7yQtHq15+70aHq93nOTP2bXgiq4981D5jcfgiv4ALsvEveBtbzvo3ftoAgpSBSG5Vhis1gs27ZtO3v2rP\/BRxBKAMT1P13o4L0kAR+EAWBchRWFt5MQQggplnsvetz56CPHmTNAC2f\/XpX05EoAENi3rCP6pD2iudu3kwRBDQfiG3kwJGPo9PPrHilVwBWGZl5ZjfE2Rgl41FKiqFt2iY0+x9bc3FxVVeVnIH+E4gsBAN7YvtzLEDCu\/xnXCB2KhU4e2Bihdzo7BVu2rDTtoV\/9SZdFc5l\/nvnfP1z1g6e87p8tEnV0dER1Ek7iI6sFUrFJ5tdeMn6zmtm1TxTHf0acZZfYqOrq6iNHjlRVVRUVFdXX11dXV8c6IoTmIwBmgI7gi18U4\/pHl3Ncy4rQI8oWiRbcZ3Z0dGXZ7Nx\/vZ30v6UBQNLIQ6tPl8Ia59x3vOzMjZgcNCakdxFeYYt\/qECa+ci8oZ9B5bsGkp\/VTN72QYtvmSY2AEhNTX3nnXeMRmNVVVVDQwO3HotxKDaGATpcmcwcwP6M63+6IOe9ZCIdW8DYkRHhvXv3drG3Xzy38mcl9x9r+2dBsnlNcuHacI\/OhPd2rcf9AeOj1MVHPApqqkAntcH+kLGyfBMbHebfZrNhiQ3FDOEVy7pcA+l6YubXHEKc9keYZdkVa9cmPfTQ3O3bd4b++x35fxf2fX11XxkAQFdKate3V+RuZHPf4\/a3sexehgnuHIyrX0awff2JRzcZzUL1hMTbwFo+Zh91T36eAyWjKFqmia22tratrS0rK6u7uxsLZyjaiGvmMDNvJb+DBcMbol4RtbB8omNfddnt\/jtGrt61S3jgAADcdc1lwea+B\/\/ritX\/9e9hIAkAVveVrRxQ3Mkxs7nv0Qnbcvz3tPQSiis\/VYb3HJvKb1YjHrO7MT7eQgAMHkNwYVaLtWWX2LiCGvaKRNFGvOUzigEAVzJTRS+iwDEM4yuxrdyzZ+WePdzLexMTd37zG+7l3b8cnvv\/ZkALST9YDQDJ0+LVfWWr+8rGGT0DfUywJbYKV2LTA+h5141xtSMqfLyR8H8Y30mReCQq8J2ryPyGNyZyg6Gg8Cy7xAYAhYWFH3zwAY5TjKKE+J7cmeElM+v9B5zjk0Kh0Ov1ngNFurk7NDR5\/Ljn+rn629N3zq5o33i\/ZhLgUaKyggoqg3ksGlwD+XM9GwkAeHsUz\/OYCt5Wtz4d9CBe7zkY3xNqaz2GaMFR\/OPGsktsEomksbEx1lGg5YF4DG8BrtYyRRiTh8VCRUWFXq9\/d2CAX2jzmsPc3OnsvNPZef9FLswwplVEyaU3IAAaAI0rGwGAHMB\/9aQKQOHKQ8Rbf1HumJr55ac515wyZH7rptnjCIz7wCvux3drUfOV\/FCMLLvEhlA0EB8pTR2nNY0LUigUCoWiwWJZYPyRhdwT2tnc977P\/vhRUKkY1YOkQniXSwCQGUAPTwXvvcRbftIA6F0XnPD2JL6PyQTQ\/GbgjX3sf2cUI5jYEIoo4m0QQoXv2\/\/EodPppFJpQ19ffW5uOMexsexrpEOjUYLad7sjCf7RPa+Ix+2FJ4ZXgGYWOhr\/MTUVZrU4hYkNocghS7k3AcMwGo1Go9EUi8Wek9cEyMayBzs6GIZRq9UArlIsvUTENb3ZfwB0RSZmLxjesxMQQDKjSMBPs6E4gIkNoQhx602gCaZPRIJQq9Vms\/mg2XxBLg8ht9lY9pjFkiwWm0zexuRgXKmC9qMhABCJcpt2fnGQ8Jo5mYXeS1xNcXreSgY79Mc7TGwIhY0so94EJpNJqVQeNJvfKiwMqr2NltVoVguolz\/D+98DIcRsNnd0dHDjWDIMwzCMXC6nT909oHDNu6Z3rTEDmF09VsBbhiOu\/80eIamwoJYAMLEhFB7CG0gJAhjPIvGZTCatVntMo2kh5K3CwgWHfLSxbAshDX19DMMEmtV8IIQYDAa9Xk8IyRaJsoRCbgTL\/7hxQ2+3AwDDMCqVqqKi4sGJGAAdgNqj+ZMAgLfp6DwxQT6WgGIKExtCYSDzhwdULf2sRqnVarlcrlQq8y9eLGeYp3NyvNZM0pT27sBAslis0Wjut6uFSqvVajQaOt\/pD33UhdIzalzmnZFxtecRv08L8PeHJdLxZ7nBxIZQqAjvkWpm2bW7KBSKubk5rVar1+tbOjoAgI8t2jAAACAASURBVF+EGpye7nKVn\/aUlanV6vALahqNpj43t5xh\/JQRs4TC+tzc+tzchr4+jUaj1+u9lBEZXo8V8JHemOX1aS4xmNgQCglZ1lmNo1ar1Wo1TTwAYDabPwMAgL9kmL1eW7yCRwhRKpX37PagWvVoCjzY0aFUKnU6nb8wmGX62S1hmNgQCh4JYBrl5YTrvh9mZaMnQkhlZeU9u\/2CXB7s\/G1ZQuEFufyYxVJZWWm1LvsPaTlJjnUACCUaMr9dDb8wF5PBYOi3WE7v3BnarKRZQuFbhYX37HalUrnw3mipwMSGUDAIQCVmtSgxm80ajcZXz5QAZQmFp3fuNJvNer0+cqGhuIaJLTJqa2tlMplMJpPL5WNjY7EOBy0aA29EJW8PGaMI0mq1xWKx\/xG8HAMD5Be\/mLHb\/exTLBaXM0xlZaWffdBSgoktApqbmy0WS3d3d39\/f3l5+YsvvuhwOGIdFFoEZt7otzrscbC4zGaz2WxecFxKByEA4BgY8L8bnXAHC23LBHYeiYCbN28WFhbSmbiLi4s\/+OADlmVxvrelhvCmAVPhg02LzmAw0EfW+CvXvvHGik2b+GtSP\/30+mOP0WXB1q2pL7yQvG4dAMDs7O3332d\/\/nNuz3KG6ejoUKlUix05ijkssUXA5s2bLRYLrYHs6upKS0sThtTQjeKawdW0plguT2HHltlsDqRz\/8pHH902NJSmVAKA6B\/+4X5WA4CUlFUHDqzcvZvbs1gspkOWLEq4KJ5gYouA6urqkydPFhUVyWSymzdvvvPOO1hcW2oIrxISs9riI4QQQnz1Gbnz0Ud\/LCv7Y1mZ4\/Tpe9PTgszMh6uqBFu3JgmFMDt7+xe\/uKXV3puYSFq9WrBlC\/cumibNZnN0fgQUQ1gVGQG1tbUjIyPXrl1LTU1tbm6Wy+Wtra20ZpIjk8m4ZYFAkIhP1YyOjgoEgtnZWafTGaljzs7O8pcFAsHo6GgIF2d4ePh3v\/vd8PDw0NAQXbNp06bMzMy\/\/\/u\/j0ygzwAIAACgFCBnUTpDcpGjoaGhkZERgUCQkpIyODPD3\/SFzbbC6fzz6ChL17e3Cx56SLRv359nZv5865ZwcDA5NXXGbp+lyxMTM59+OsM7QvaaNRMTE4n11yeVSum3R39\/f6xjSRiY2MI1NjZmsVhOnjxJS2lHjhzp7Oxsa2urrq7m78b9UspkMqfTKZVKvRwrvg0MDDidzpSUFIFAEMHDcke7e\/eu0+nMyMgI\/OLQId4NBgO9DadDOtFN79ntAFBTU0OHxA3rwWGza3owBqAp9MMsKBF\/KxbJf\/7nfzqdzr9++GG39WuzslZs2nTHanWsWkXXJM\/Nrc3MvLt6NdvRIbh5c\/WhQ\/Dcc3STs7f31kcfgWtPAIC7d6emphLrOlut1rm5uVhHkWAwsaFEpdfrtVrtPbu9WCz2OtgSHQ\/XxrJ0zMAFxlXyQ+tawErI+PNQenpScnLyQw8BQPK6dZDy4Dst2SMvAgC2sS0H2MYWLolEUlhYeO7cOdrF\/\/z588PDw6WlpbGOaykjhGi12srKyi8DXJDLT+\/c6bWXAR0P9\/TOnT3792dOTyuVylCeZCK8B9dU4USNgmZj2QX3ScnIgJSUOZZNXrfuoS9\/ee72bcfp038sK7s7NLRi06ZUV+mNE85YzChRYIktAhobG2tra7dt2wYAaWlp58+fd2tgQxHEDYlLB3EP5C10zMB3BwaO6fWEEO\/TN\/uCxbVYCDD9CLZuFezYAQCzf\/jDikceSRII5uY3ACfzup\/YWHZweloul0c0UhSPMLFFRmNjY2NjY6yjWPrCGRL36ZycLKHwoNmsVCoDzW3ENQslg8W1qKKVxl12u9ey+Mo9e1bu2cO9vDcxcec3v6Hrk9etS33hhdQXXgAAmJ2d\/fRTbrfB6elFjhrFC6yKRIkkzCFxi8XiC3K52WzWarUL7w2uSkjAx7FjQKFQdPkdKIu6OzT0xbe+5bx+3Xn9uuP06XsTE\/c3eDyg3WW3MwwT\/jQ6KP5hiQ0lDDokbn1ubjhD4tKeJsc0moCmCjO4FipCPiEKUUVFRWVlZX1uLv8mZvL4cT9vcV6\/\/sW3vuV1k41lG\/r6cNiRZQJLbChhBDIkbiDoaPELF9oIr9uIIsxzoqCpVCqGYRr6+iJyNFpcq6jAO5RlARMbSgwBDokboHKGoQf0txNxLSgick4UNJVK1UJIIBWS\/tHimkKhwHrIZQKrIlFioMU1fiWk53i4d4eGaFWV8BvfWH3oEP+RJqCP637\/+3T56Zychr4+g8Hg75uuw7WAd\/kxolarzWbzCxbLlaeeCvkgNKsli8URn90bxS0ssaEEQEcYWbBpbcWmTQ\/\/5CeCrVs9N83dvn1nfvns6ZycBSYxobszWGKLJZ1ONzg9fbCjY+FdfWgh5LcAOp0On2BbPjCxoQRAR4vwmtj44+HO3b6dvG7d6tJS9uc\/\/2N5OV1\/+xe\/gNnZP\/\/2t3c6O\/lvzBaJwM84FISX2FDsMAxjMpm67PYQchstq9E+I1gJuaxgYkOR1NTUZHJpanIfV5FuraurC\/awfhIb505n559\/+1sAmBMKufmUBVu3rtyz596tW\/Q5J74FxnonrgVFsMGiCFMoFDS3FXz4YeDtbTaWPdjR8R7LajQarIRcbjCxoYipq6srKCjgXhYUFHC5LT8\/v7W1lb81KAMDA7SA5d+9iQmYnU1atWp6xYpZlgWAlU88kbxunfOTT5zXrwd3Sq54sPCMYGjRKRQKq9UqKyw82NFxsKPjXb\/zZdOCWv7Fi8lisclkwqy2DGHnERQxubm5AHDx4sVTp07V1dXt378\/Ozs7Pz9fLBZ\/+9vfFgqFt27dWrNmzWKHkbxy5eqMjBSnk463xA1L4dWAr69Is2uBiXR8KCS0TtJsNldWVh6zWBr6+miZmyvH21i2y24fnJ6mPfuxoLacYWJDETMxMUFHdrh69eqpU6dOnTpF1+\/bt49l2R\/+8Idf+cpXQiu05eTkBDIe0v3B3WdmhE4nAAi2bEles+bu55\/7Ka7lLDhHsyK4UNGiokU3Qgidq8itJplhGMWBA9+Uy\/FB7GUOExuKmOvXr+fl5QmFwhMnTpw4cYKO6wgA7e3t7e3tAPCVr3wlnOPbWNbPSFr88XDpmpRHH4WUFO6l59HAz2C7ZgCITHFNq9UGMlUKy7LCkMYJAwCGYZZV6YT+vPRH5q4tdnpEHExsKGJ0Ot3w8DCtdQQAhmFaW1tfeeWVnp6eMI+sUqkqKyu9DonrazxcoNNxzc4+GDxwPloE9P5tSFwL3jYGy2w2d3d3r+JPd+nNihUr7t69G8LxZ2ZmNmzYsKwSGx\/mM+QJExuKJK5wRtvY0tPTd+zYEX5iAwCGYXyN9c7hHtAGAMHWrUl+C0DvDgwwDLPA16LfjYFbtWrVjh07\/O\/jdDpDm5ocZ85EyA0mNhQZ+fn5L730Unp6ektLi06nMxqNu3btSk9Pj9TxFQqFXq8\/vXMntybk8XDBNbm2RqPxvpmEFiNCKC5gd38UGT09PYODgwKB4OjRoyaT6fXXX1+\/fv2f\/vSnTz75JCLHp1Vt\/vt5B452nFt4zkkmImdDCEUVJjYUMTU1NVeuXOFejo+PR6SBjaL9LY9ZLOEfCofERWhpw6pIFEk1NTUhb12QTqeTSqUHOzouLFjS8guHxEVoacMSG0oY3LCB4czR1dDX91sAtVqNvekQWqowsaFEolAoNBoNHdk22Pfyh8Rd4AFexrVAgo4QIRRzWBWJEgytQtRoNF12e+B1krQbJA6Ji9BygCU2lHjUajU33PuCRTduSNz3WJYbrmIBjGuBhBcoQigWsMSGEhIdM9BgMNCaSTq5drFYzE0CMDg9TUfFbSEEh8RFaFnBxIYSFR0wsKKigo6H2+AxsxodWESnVocyJC7Dm2sUIZRQMLGhxOY2Hi5\/SFzs94jQ8oSJLTKam5sbGhro8tmzZ0tKSmIbz\/IUyWSmANADAADB8UcQSjDYeSQCjEbjmTNnLly40N\/ff+HChe9\/\/\/u9vb2xDgqFh+tuaY5hEAihUGCJLVwOh+PcuXPPPfdcXl4eAOTl5XV2dsY6KBQ5kRmcEiEUPVhiC5fVah0aGiouLo51ICiiFK4FErsYEEIhwRJbBKSnp4+Ojh45cmRqaiotLe38+fO09MYnk8m4ZYFAYLVaoxtjBIyOjgoEgtnZWafTGaljzs7O8pcFAsHo6Gi8XBwGYBjgZwDfC+swTqdzxYoVC140\/qUIQbxctEgYGhqKdQhxRCqV0m+P\/v7+WMeSMDCxRcDg4OC5c+e6urpSU1ONRuORI0c8cxv3SymTyZxOp1QqjUWkYRkYGHA6nSkpKaHNh+kLd7S7d+86nc6MjIx4uTgKV\/+RpLD6jwgEgrt37wZy0cK5sPFy0SJkif044bBarXNzc7GOIsFgVWQEZGdnv\/nmm6mpqQBQVFS0ZcuWrq6uWAeFwsa4FsyxiwEhFDxMbOHKyMgAgNHR0VgHgiItx7XQEcsoEELBwsQWLolEcuDAgVdffdXhcABAd3f3jRs3sC\/JUqByLZhjFwNCKHjYxhYB1dXVALBt2zYA8NV5BCUkBoC4\/jExjQQhFDAssUVGdXV1f39\/f3\/\/1atXMastHSrXgjl2MSCEgoSJDSHfsJkNoQSEiQ0h31SuGkh9LKNACAUFExtCfilcC\/rYxYAQCgYmNoT84kZDxtpIhBIEJjaE\/FJhbSRCCQYTG0ILUbgW9LGLASEUMExsCC0EayMRSiiY2BBaiIpXG0liGAdCKCCY2BAKgNq1oI1lFAihQGBiQygAKteCPnYxIIQCg4kNocCoXAvm2MWAEAoAJjaEAsN1ITHEMgqE0IIwsSEUGBV2IUEoMWBiQyhgKteCOXYxIIQWgokNoYBxfSMrYxkFQsg\/TGwIBUPhWjDHLgaEkF84gzZCwahwpTQDL8mFp6mpqaCgwGq1SqXSK1eu1NTU0PX5+fkvvfTS+vXrAWB8fPyVV17p6emJzCkRWtKwxIZQMFQRHhO5rq6uoKCAe1lQUNDU1ATzsxoArF+\/\/h\/\/8R8jc0qEljpMbAgFSeFa0EfgYLm5uQBw8eLFZ5555uLFiwCQnZ2dn5+\/Y8eO9PT08fHx73znO2+\/\/bbT6ZRIJPv27YvAKRFa6jCxIRSkCtdCJMZEnpiYAACFQvG3f\/u3p06dUiqVZWVlPT0969evFwgEg4ODPT09w8PDTqeTZVm73R6BUyK01GFiQyhIikg+0Hb9+nWn0ykUCr\/1rW+ZTCadTkfX0yRH29u+8pWvCIVCmuTCPR9CywAmNoSCp3AtmMM9kk6na2xsZFmWvmQYprW1NT8\/n9uBdi0hhHCdShBC\/mFiQyh4EZ2hrb29\/cCBA1wbW3p6+o4dO+gmLqtVVuKjcwgFChMbQsFTuRbMYR0mPz+\/tbW1vb2d5i2j0Tg+Ps5txayGUGjwOTaEQsIAkHDb2Hp6egYHBwsKCo4ePfrEE09IpVIAGB8f\/+STTyorK\/Py8gCAYRjT\/9\/e\/ce0dZ57AH+c4CUYE5qkmAsEcuxVrUYCXRXCmHKFTS6T2k1Zu2hogbXFzt1tk4ZIqYJIW6nCTv9YLk3abe0Wkk7B3jrRXSK0rOomFVaO2VBRIe2CE27TpfjwI07BN8lSHCeZAd8\/3vj0BPPbv4+\/H+UP+\/j48HJyzOPnfZ\/zvp2dROT1en\/+85+\/\/\/77ITcdQOaQsYXT+Pi4Xq\/v6OiIdUMg8gyBB0JIhzl48ODHH38sPhVvxN68ebNSqQzp0ADJChlbODU2No6MjMS6FXcJgkBEPM\/b7XcHgjiOIyK9Xm8wGGLWLAjCqkJ8Pp80kqFUBGDZENjCpqOjw+Vy5eXlxbohJAiCzWYzm81ElJ+WRkR5KhUR\/YOo2+0mIo7jjEZjTU0NC3UQEiFQ\/Q8A8QGBLTzGx8dPnTr10ksv7du3L4bNEENaflpafUHBtszMbZmZM\/YZ8XpbBOHUq6+azWaj0SjeOLVIt2\/fDl97aXJycmpqKowHXCaBiJYYn4TAgyW9CwAiD4EtPNra2srKyjQazVw76HQ68bFSqXQ6nWFvw+XLl5966qnpa9ee+8Y3fvL1r7ONw8FxaMWKXTrdLp3u159\/\/tbvfsfz\/JEjR0pKShY8\/tjYmFKpvHDhQhjbnJKSMjk5KT5VKpVjY2ORODmzuEz0EVEb0UeSjblEuUTfIqpd6O0+IiVRLpGfaN72+ny+lStX+ny++Y8nPQ\/LEKWTFhWjo6OxbkIc0Wq17K\/H4OBgrNuSMBDYwsDhcHzyySfHjh0Tb7MNJl6UOp3O5\/Ox+rcwslqtJpMpPy3tjF7POh4XdHjTpv\/Samt7e3ft2tXZ2bngwJtWq83KygpDWyXGxsZmHJPjuIh3kApENiLzHC8JRN1EbURGyQJswbt1ExERR7TQ\/6RSqZyamlpMJUgo1SJhv6JiKxK\/jiAIbMiZDT8LgsAFxPPAs9Pp9Pv9sW5FgkFgC5XH43nzzTdra2vVavU8gS2ieJ43mUxVHPdGcfGS3pinUp3R6\/f39ZWXly8mtoX9w8\/WagnvMRdgJbLcW8rIBf4JkvvSBCIzEU\/UOdtBLIEHhnA3DyJgxqgz65\/PJiJB+MeFC1bJwHNDw1zfZSCRILCFyul09vT0tLe3i1ueeeaZ+vr6PXv2RKcBgiCUl5cvI6qJ3iguHr55c5GxLYEJRCZJ6OKIDEQ1QcFJkORzPJE2qKdRCMzrz82d0kF8WMyoMwUGns1ms9VqRXiTAQS2UBUWFp47d449Hh8fr6ysfPnllysqKqLWANYDueyoxpzR6x+32y0Wi2wDm5VIOn2HkahhjroPjqiBSE9UTkREAlG5JG8TJH2P+OsX39h3vmm3u76goL6gYJ4981Sq+oKCKo5j4Y3neXZTPCQo3KCd2Hie53k+xKjG1BcUsKOFfqj4IhBZJFGNI+okal6omtEgSdR4IlMgUdNKdjCGs5kQXoIgaLXaabf7jF4\/f1QTsfB2Rq\/neV5mY5bJBoEtnDQajd1uj2a6ZrFY5updWSp2HIvFsvCuCUQgskjqRMxEzkUPjHGSRM1KpA2KjhCvWK6Wn5b28WOPLbKWSrQtM\/OT736XHSFCzYNIQ1dkAmNVXmf0eunG+15\/feWGDdItU6Oj\/3z+efGpeu\/eVdu3E5H\/1q2bv\/71na4u8aUqjqvleVYtFtmmR4dAVC65R8249M5DA5E5qH7SjE7IuMamjZ52uz9+7LHlHYEVVT3O8yaTaak3ekI8QMaWwGw2m1jiNY+VGzasPXFCuXkzSaIaESlSU1U\/\/jHbzuzauJGIZNIbyRNpJVGtYbnRqIGoM7C4qJGoE1Et3vE8P9jbG2L\/\/LbMzDe3brVarTL5OCQZBLYExvP8XN0sdz744Gpl5dXKSs8bb\/hv3Vqxbl3qzp3KzZuV3\/wme\/VLi2X62rUVa9YoN22SvrGK48S5JROYJVD6QYFuQ2MIRzMQdRI5iZpR3x\/vWLoWlv75XRs3yrBzPjmgKzKxST+9k17v\/\/X2qj2elUQ+j+e2252SlkZdXcpNm1Zt375i7doV69YpUlOnr12789e\/+s6fv\/7ss8EHzFOp\/iehv6IK9958bUaClVx4ng+9SFjEOud5npdttbBMIbAlMEEQSiQZ2z8HBm673f7paSK67XZ\/EUi8NmzZovn3f1ekpiofflihVPpv3Uqvr1ekptLk5K0\/\/MH7+99Lj5mfliYMDETztwgnIeRBNUhwNpttRq42\/6izcvNm9f79K9atu\/vavR+KXRs3Ng4M2Gw2BLbEgq5I+bi\/uPi+QFnzytWrU9LS2ONpNkuhQsGesryNiCglJfWJJ1Q\/+lH0mxoRlnsH1YyIakmHlVOxoeJ5SEedlZs2rVizZp6dt2VmWq3WMDYSogAZm6zcV1CgXLOGiFTZ2Rsee4yIJr3eNd\/6luJrX5v2en3nzn1t61by+VgxJPsmm\/LQQ9IjDN+8mXglkcK93Y8cUScm3U9GbBLIWUfX7nzwgef4cSJaVVaW9pOfsFFn3\/nzK9ato5QUn8Px5eHDsx5zW2ZmiyDIp1Q4OSBjS2wjC81OmVpSsqq4mIgmP\/ts+to1\/61b4kvT169HtnHRIRCVB92pxsWqNRBLdrs9P9BRMZc7XV3\/+vBDIlqxdi0RrcjMJCJlYeH61tb1LS3BHRgsTLKQCYkCGVsCMxgMH7S2zvrSqu3bxbJ+IhILRnx\/\/\/uq7dvV+\/er9+8nIpqcnLx4UfrGbrebu7dOMn4JQYmaEd2PSU0QhMXcjj197RpNTipSU++kp7PwdldKSuoTTxCRdOCZHRCBLbEgY0tger1++ObNBZO2qdHR688+6zt\/nog8x4\/7HI67LwQVj4x4vd1udwLMACsERtTMgS0c7jCDpZn+17+m1671r1xJk5O3Tp++WlnpczgoJWVG5\/yCny+IQ8jYEpjRaDSZTC2CIJ0KTzrJyKzmGksglq5xXFwXgAlENiLrvQtYGxHSgIiI47gPFhGH2LjatNc79tprY6+9dl9BAau6mna7KdBFGXzkcDcWIggZW2IzGo2NYarOZyt3xO8HWJBkaQIRYSoQmGnjxo3DN2\/Ov484TYF\/aOiBV199xOPR9fWtKisjopQHH6SgsecFDwhxCBlbYmtoaLBarY0DA4ucv3we3W735bS05rjqhxSIBCI7ES9ZR40xzraUGiQ39rVsxOsNHmmbddR58vx5\/65dKzZsEEed\/bdu3bl3goIE6MaAIMjYEhvHcWazuXFgoNvtDuU4I15vbW+vwWCI8QdYIOIDa6eVE2kDFY98YAeOyIjZrWB2BoOB47gF+zCko87\/fP75qdFRtj14WnAKBLbItBciBRlbwmtoaOB5fn9v7xvFxcubH2\/E633cbuc4LtoTmQtEHxHZieyB5EyYY0+OiCOqwRJosACDwWC1WqVTai046jzPDqycqjOuujFgEZCxyUFzc7Nu69b9fX3LyNtYVFuRmRnZJYOFwEKd1kA2piDSEj1FZCKyEvGzRTWOyEjUTOQMeSJjSA41NTVE9M7QUFiO1jgwgH7IRISMTQ5YsmUymR7n+fqCgsWPt70zNFTb28txXGdnZ5j7WwQinsguebwgLvBPT8ShpxGWg3Wn1y5iYq0FdbvdLYKA9dgSEQKbTLDgZLFYzGbzO0ND9QUF83+w2aBat9ttNpvDc+OaIKn1sC6mxUQckYboMYQxCKfm5matVvu43T5jDd4lGfF69\/f1cRxnNBrD1zSIEgQ2WWloaKipqTGZTLU83zgwsGvjxvy0tDyVSpxnqNvtZv\/YnJDNjY0hfW6FQFomzJuTcUQUCF0zsjEnkXb5Px8gGPuSV15evr+vb3nr17CvfRHvn4eIQWCTG\/apFgTBZrOZzeZZd9heWVlTU7P8kQOByDZbCf5XP0PSqWjAzI0QbQaDgXXOE9FSYxuLapfT0pqbm1EPmaAQ2OSJ47iGhoaGhgY2x504011Iw+BC0KwfX\/08dnQMj0G8YF0RJpOp2+0+o9cvZg5JInpnaKhxYIDlaohqiQuBTebYhzOkj6gwd37GERlwozTEKaPRaDAYysvLH\/nTn+oLCqo4bq7wNuL1Dt+8ye4HZdkeolpCQ2CDuQlElqBKEA63lEHC4DjO6XSyoqrGgYH8tDRWVMXGntkEx6z6ke0c6qgzxAcENggiEPGBLE3KgHgGCUnslrfZbDzP85JJs1hmZjab9Xo97leTDQS28Kirq2trayOivLy81tZWjUYT6xYtizDbKBoXmJiRi0GLAMJFHHimwKgz+hvlCjOPhEFdXZ3L5erv7x8cHKyqqjpw4IDH44l1o5bOGpiYUQhs4QKzfjQgqoGscByHqCZjCGyhGh8f7+3t3b17t1qtJqKdO3dOTEw4nc5Yt2spBKJyIpMkpBmJOomc6HgEgMSDrshQaTQau90uPh0bG7tx40YM27M0ApFNsg41ERmIGlDlCAAJDIEtzGw2W0lJSWFhYawbsgj8vVkaR9SAFC0ibt++\/cUXX8y\/j9\/vVygUyzv46tWrl9UuAHlCYAunurq63t7e1tbW4Jd0Op34WKlUhquv0u12V1ZWXrlyhYiys7NbW1szMzOJ6K233jp69Gjw\/sePH9++ffuR\/UfU76trqZaISEmUS7STLpRfqK6uvv3Mben+JSUlJ06cUKlU0h80Y4ff\/va3y27\/aGApLBnz+XxTU1Off\/75\/LulpKRMTk4u70fk5uYmWO\/3vJLhqlg8rVbL\/noMDg7Gui0JA4EtbMSoNmtJpHhR6nQ6n8+n1YZhhkSPx2MymYaHh9nT4eHhysrKnp6e7OzsjIwMn88X\/BaNRpPmTjN1mEp9pXc3GYiaiTi6dvba1NTUjHd1d3c3NjbabDaWEwQfMyUlJcTfJSynIp797W9\/W8xuTqdT9qdi8XAqRE6n0+\/3x7oVCQbFI2Hg8Xiqq6tdLtd7770XzUL\/ixcv9vb26vX6iYmJiYkJvV4\/PDzc19dHRIcOHfL7\/X6\/\/8iRI0R05MgR9nRH4Y5Vj64qvVNKREOKIStn9bzrkVY8sqOxnV0uV35+Ps\/zV65cyc7OHhoaEjfm5+e7XC6\/3y+9JQgAIB4gsIUBm2v45MmTrDAyanJycu6\/\/3673f7LX\/5SrVbzPO\/3+3fs2DHnGyxEWsq4nkFEt\/\/t9qGSQ\/vG9128eDF6LQYAiDwEtlA5HI729vaenp6ioiJdQEdHRxR+dHZ2Npsr4YUXXlAoFGlpaWfPnp11z4zrGWT5qvrxi9VfTP1l6pEfPOL1emc01W63p6enKxQKhUKRk5MzPDz83HPPZWdnR\/QXAQAII4yxhaqwsPDcuXOx+uk2m62goOCFF14gIq\/XW1xc\/Mc\/\/nFG0sYRV3i2kALxy0KW1ebVhwoOPb326V\/96ld\/\/vOf9+3bN2uuqVKpurq6tmzZEvnfAwAgbJCxJTw2nMbG2IjozWMoJAAACUtJREFU9OnT0lczrmd0Uue2jm1ENJ0\/beWsZjKzDI8lZL29vdLeSDbG1tfXp1KpgvM5AID4h8CWwN59912FQlFTU0NEarX64MGDM\/fgac9\/7+FYcQhH\/T\/oNwmmGbt4vd5f\/OIXMzZu2bLlnXfeIaLDhw\/P1b0JABCfENgSWHFxcX5+\/m9+8xs2JPb973+fiH74wx\/efdlCVH734Y21N6iTXr\/+OkkqJP1+P8vMWN3jjIPv2LHj6aef9nq9Bw8eTMipLwEgWSGwJbDs7Oyenp78\/Hxxy90BNoGkpSJWsjYdarqy6grP8yqVqqKiQtz\/oYce2rp1q3iTwAxHjhzJz89nVZcR\/UUAAMJIgVv\/okyn00X2jkuBqFwyUZaZqCFSPyp0uCtZhFMhwqkQRfzPhUyhKjKu8Vd524hNuCUIXoGIjHnGjakbjXnGOd8gSKIaR2SM66gGABAJCGxxSvAKls8s1lGrdKP5MzMR2UZtnd\/uDHrDvfP0c0SdWEQNAJIRAls8ErxC+Yflwi1h1lf5q7z2L1rnf0gmvRWITER84KmBKCjwAQAkCRSPxCPTOdNcUY0RbgnlH5YT0d06EW0gqnFEZkQ1AEhqyNjiDn+V56\/yC+4m\/K8g\/EHgGrmvNnFEzVgjFACSHQJb3LF8ZgneyI1z3BjHHhBRzV9qDA6D5GXUiQAA3IXAljA6X5qth5EjMhA1oE4EAOAujLHFncX0QxKRoBGs\/2mlzrvLhAIAAIOMLe5wqVxw5YigESxVFiISsgT2lC\/kuVTOyBmj3T4AgPiGwBZ3DOsNM25fY8zV5hlbOBUX+eYAACQYdEXGHf16\/SL3bHgQ5SIAADMhsMUdY57RsN6w4G6G9YbF7AYAkGwQ2OJR88PN8wctLpVrfrg5Ws0BAEgkCGzxiFNx88Q2w3pD57c7McAGADArFI\/EKU7FdX67U\/AKtlEbf5Vns\/sb1htq8mrQAwkAMA8EtrjGqbiGBxsaMKcIAMCioSsSAABkBYENAABkBYENAABkBYENAABkBYEtPJqamnQ6nU6n0+v14+PjsW4OAEDyQmALg46OjpaWlp6ensHBwaqqqgMHDng8nlg3CgAgSSGwhcrj8Zw6daqqqkqj0RDRzp07JyYmnE5nrNsFAJCkENhC5fV6L1++\/MADD7CnKpUqPT29u7s7tq0CAEhauEE7DNasWZOVlcUeq9XqnJyc4H10Op34WKvVSp8CAMxF\/HMxODgY67YkDAS2KBEvSp1Oh45KkVarxdlgcCpEOBUidir8fn+sG5JgENjC4MsvvxwbGyssLCQij8fjcrnEnslZ4TIV6XQ6nA0Gp0KEUyHCqVgejLGFSqVS5ebmXrp0iT31er0TExPbtm2LbasAAJIWAluo1Gr17t27W1pa2O1rbW1t6enpWq021u0CAEhSCuS5YdHU1NTY2EhEeXl5ra2trPQfAACiD4ENAABkBV2RAAAgKwhsAAAgKwhsAAAgKwhsAAAgKwhsAAAgKwhsAAAgKwhsUYX1SBmPx1NdXa0LSOaz0dTUVFdXJz6Vnhnp9mQw41Q4HI6HH35YvEiS5GzU1dUFfyiS+apYHgS26MF6pCKn05ment7f3z84ODg4OGi325Pzlnbxvn6R2WzOyckZHBzs7+93uVxNTU2xaluUBZ+K7u7uvXv3DgYcPXo0Vm2Lmrq6OpfLxT4X0j8RSXtVLBsCW5RgPVKpsbGx9PR0tVod64bEDEtHWlpaSktLpRs\/+uijmpoaCkzV1tXVJftvP7OeCiK6dOnS\/JOJy8z4+Hhvb+\/u3bvZ50L8E5GcV0WIENiiBOuRSl26dOnTTz9N8n7IEydO2O126ep9Y2NjCoVCXNsvKytrdHQ0Gb79BJ8Kj8czMTFx+PDh5Ol\/02g0dru9oqKCPR0bG7tx4wYl8VURCgS26FnMeqRJ4tKlS2vWrBG7XCorK5MtthUWFs5IUJjc3FyVSsUeZ2VlZWRkRLddMTDrqfB6vZ9++ml1dbXY\/5YMsU3KZrOVlJSwxbCS8KoIEdZjgxiQjpc8+eSTXV1dbW1te\/bsiWGTIK6w9IU9VqvVL7744r59+xwOB\/tDL3t1dXW9vb2tra2xbkiiQsYWPWw9UvaYrUca2\/bEiSRPXme4fPmy1+tlj8XOKEiqNEWMamJFFa6KpUJgixKsRyoaHx\/X6\/UdHR3s6WLWHE8SWVlZfr9f\/PYzNja2YcOG5Fzbr6OjQzr4ys6J2JMvV6ys3+Vyvffee2JUw1WxDAhsUYL1SEUajWbr1q2nTp1ilV1vv\/02Ec064JRsCgsLS0pKbDYbBcpoy8rKkrN2tKioiIja2tqIyOPx\/PSnP\/3e974n+3tCzGYzEZ08eVL6n46rYhkQ2KKnoqKiqqqqtLRUp9O1tLT87Gc\/S9qr8+jRozk5OUVFRTgVM5jNZpfLpdPpioqKcnJyknbcUaPRtLa2trS0JM+pcDgc7e3tPT097HPBsI4NXBVLhYVGAQBAVpCxAQCArCCwAQCArCCwAQCArCCwAQCArCCwAQCArCCwAQCArCCwAQCArCCwAYQfmzZsxoo8bGPwLPXiuuriNGOLObhOp6uursa6XADBENgAwo9NnEFE4qrQHo\/nwIEDubm5bNqkGUpLS\/v7+8W1uBY8uN1ur6+vD197AWQFgQ0gIjQazcsvv9zW1sbysLfffvvChQsvvvgiJg8DiDQENoBIqaio2Llz5yuvvNLR0XH8+PFjx44tuJyYw+EoKys7duyYdHnxurq65FlIGiB0CGwAEcQ6DJ955pnvfOc7i+xpvHHjxtmzZ\/v7+\/v7+3Nzc0tLSx999NHBwcEzZ860t7cvchwOIJlhBW2ACGJr9IyMjDz66KOLf9fu3btZj2VZWRkF1vTJysq67777ItROADlBxgYQQR0dHe3t7Rs2bHjllVekFZLzyMjIkP2KmgARhcAGECkOh+PgwYN79+49ffo0SSokASCiENgAIoKt+7xp06Ynn3xSrJBsamqKdbsA5A+BDSAiZtT3swrJ48ePOxyOWDcNQOYQ2ADCr6mpqbGxcUZ9v9ls3rRpU21t7SIH2wBgeRR+vz\/WbQBIak1NTV1dXSdPnlzSvdvLexdAMkDGBgAAsoLABhB7PT09RUVFS5oEGTWWAHNBVyQAAMgKMjYAAJAVBDYAAJAVBDYAAJCV\/wcVRieDhxFl0AAAAABJRU5ErkJggg==","height":438,"width":584}}
%---
%[output:8e24e642]
%   data: {"dataType":"text","outputData":{"text":" \n","truncated":false}}
%---
%[output:5c613be2]
%   data: {"dataType":"text","outputData":{"text":"Resumen de la ejecucion RRT* + MPC\n","truncated":false}}
%---
%[output:08bd6676]
%   data: {"dataType":"text","outputData":{"text":"    <strong>Arquitectura<\/strong>    <strong>Escenario<\/strong>    <strong>Semilla<\/strong>    <strong>Exito<\/strong>    <strong>MetaAlcanzada<\/strong>    <strong>Colision<\/strong>           <strong>MotivoTerminacion<\/strong>           <strong>PasosEjecutados<\/strong>    <strong>TiempoSimulado_s<\/strong>    <strong>TiempoHastaMeta_s<\/strong>    <strong>Longitud_m<\/strong>    <strong>SuavidadRMS_1_m<\/strong>    <strong>DistanciaMinima_m<\/strong>    <strong>DistanciaFinalMeta_m<\/strong>    <strong>Planificaciones<\/strong>    <strong>Replanificaciones<\/strong>    <strong>FallosPlanificacion<\/strong>    <strong>EpisodiosRiesgo<\/strong>    <strong>EpisodiosColision<\/strong>    <strong>TiempoPlanificacionTotal_s<\/strong>    <strong>TiempoPlanificacionMedio_s<\/strong>    <strong>TiempoControlTotal_s<\/strong>    <strong>TiempoControlMedio_s<\/strong>    <strong>TiempoControlMaximo_s<\/strong>    <strong>TiempoCicloMedio_s<\/strong>    <strong>TiempoCicloMaximo_s<\/strong>    <strong>TiempoComputoTotal_s<\/strong>    <strong>FactorTiempoReal<\/strong>    <strong>CiclosFueraPlazo<\/strong>    <strong>EsfuerzoControlNormalizado_s<\/strong>    <strong>VariacionControlNormalizada<\/strong>\n    <strong>____________<\/strong>    <strong>_________<\/strong>    <strong>_______<\/strong>    <strong>_____<\/strong>    <strong>_____________<\/strong>    <strong>________<\/strong>    <strong>_______________________________<\/strong>    <strong>_______________<\/strong>    <strong>________________<\/strong>    <strong>_________________<\/strong>    <strong>__________<\/strong>    <strong>_______________<\/strong>    <strong>_________________<\/strong>    <strong>____________________<\/strong>    <strong>_______________<\/strong>    <strong>_________________<\/strong>    <strong>___________________<\/strong>    <strong>_______________<\/strong>    <strong>_________________<\/strong>    <strong>__________________________<\/strong>    <strong>__________________________<\/strong>    <strong>____________________<\/strong>    <strong>____________________<\/strong>    <strong>_____________________<\/strong>    <strong>__________________<\/strong>    <strong>___________________<\/strong>    <strong>____________________<\/strong>    <strong>________________<\/strong>    <strong>________________<\/strong>    <strong>____________________________<\/strong>    <strong>___________________________<\/strong>\n\n    \"RRT* + MPC\"     \"alta\"         7       false        false         true       \"tiempo_agotado_con_colisiones\"         1000                150                  NaN             51.03           2.7811             -0.53122                4.2877                 115                 114                   10                    3                   1                      3.8355                       0.033352                    1.8315                0.0018746                0.015453               0.0086367               0.16144                 8.6367                17.368                1                       72.337                         115.11           \n\n","truncated":false}}
%---
%[output:16814516]
%   data: {"dataType":"text","outputData":{"text":"Episodios sin candidato factible MPC: 5\n","truncated":false}}
%---
%[output:58a86ad2]
%   data: {"dataType":"text","outputData":{"text":"Pasos con riesgo predicho por MPC: 41\n","truncated":false}}
%---
%[output:347b685f]
%   data: {"dataType":"text","outputData":{"text":"Pasos con colision fisica predicha por MPC: 17\n","truncated":false}}
%---
