%% EJECUTAR_ESTADISTICAS
% Muestreo estadistico reproducible de las tres arquitecturas:
%
%   - RRT* + APF
%   - RRT* + MPC
%   - PRM  + MPC
%
% Cada arquitectura se ejecuta 30 veces en cada nivel de dificultad:
%
%   3 arquitecturas x 3 dificultades x 30 repeticiones = 270 ejecuciones
%
% PRINCIPIO DE CONTROL Y EMPAREJAMIENTO:
% La semilla del planificador se mantiene fija y comun en las 270
% ejecuciones. La variabilidad entre repeticiones procede del entorno
% dinamico: cada repeticion utiliza una semilla de obstaculos diferente,
% pero esa misma realizacion se aplica a las tres arquitecturas.
% Los obstaculos utilizan un RandStream independiente, por lo que su
% evolucion no modifica el generador pseudoaleatorio de RRT* o PRM.
%
% IMPORTANTE:
% Este script NO modifica los mains aleatorios ya probados. Para conservar
% exactamente su logica, lee su codigo fuente, sustituye solamente las
% opciones de ejecucion (batch, escenario y semillas) y lo ejecuta en un
% workspace aislado mediante evalc.
%
% SALIDAS:
%   salidas/estadisticas/<idExperimento>/
%       resultados_brutos.csv
%       errores.csv
%       resumen_estadistico.csv
%       tasas_exito.csv
%       resultados_estadisticos.mat
%       figuras/*.png
%       figuras/*.fig
%
% Las figuras de las metricas continuas o de conteo son boxplots con:
%       - caja: cuartiles;
%       - linea central: mediana;
%       - bigotes y valores atipicos;
%       - marcador adicional: media;
%       - barra adicional: media +/- 1 desviacion tipica.
%
% La tasa de exito se representa mediante barras con IC 95 %, porque un
% boxplot de una variable binaria 0/1 no es informativo.

clearvars;
clc;
close all;

%% ========================================================================
% CONFIGURACION DEL EXPERIMENTO
% ========================================================================

idExperimento = "muestreo_30_v1";

dificultades = ["baja","media","alta"];

arquitecturas = struct( ...
    'nombre', { ...
        "RRT* + APF", ...
        "RRT* + MPC", ...
        "PRM + MPC"}, ...
    'archivo', { ...
        "ejecutar_rrt_apf_obstaculos_aleatorios.m", ...
        "ejecutar_rrt_mpc_obstaculos_aleatorios.m", ...
        "ejecutar_prm_mpc_obstaculos_aleatorios.m"});

nRepeticiones = 30;

% La semilla del planificador se obtiene de parametros_generales.m y
% permanece fija en todas las repeticiones. Las semillas de obstaculos
% cambian entre repeticiones y se comparten entre las tres arquitecturas.
semillasObstaculos = 100000 + (1:nRepeticiones);

% Guarda un checkpoint periodicamente para no perder un lote largo si la
% ejecucion se interrumpe.
guardarCheckpointCada = 5;   % [ejecuciones]

% Si existe un resultados_brutos.csv previo con el mismo idExperimento,
% se saltan automaticamente las combinaciones ya terminadas.
reanudarSiExiste = true;

% Las figuras se guardan siempre. Mantener false evita abrir decenas de
% ventanas durante el postprocesado.
mostrarFiguras = false;

% Tambien se genera una figura adicional de "llegada a meta" para separar
% ese concepto del criterio estricto de tasa_exito.m.
generarGraficoLlegadaMeta = true;

%% ========================================================================
% METRICAS PRINCIPALES
% ========================================================================

% soloMeta = true:
%   La estadistica se calcula solo sobre ejecuciones que alcanzaron la meta.
%   Es la opcion utilizada para longitud, suavidad e iteraciones hasta meta,
%   porque una ejecucion truncada en maxPasos no representa una trayectoria
%   completa hasta el objetivo.
%
% soloMeta = false:
%   Se incluyen todas las ejecuciones. Esto es apropiado para seguridad,
%   colisiones, riesgo y coste computacional.

metricas = struct( ...
    'campo', { ...
        "Longitud_m", ...
        "SuavidadRMS_1_m", ...
        "DistanciaMinima_m", ...
        "PasosHastaMeta", ...
        "EpisodiosColision", ...
        "EpisodiosRiesgo", ...
        "TiempoPlanificacionTotal_s", ...
        "TiempoControlMedio_s", ...
        "TiempoCicloMedio_s", ...
        "TiempoComputoTotal_s"}, ...
    'etiqueta', { ...
        "Longitud de la trayectoria", ...
        "Suavidad de la trayectoria (RMS de curvatura)", ...
        "Distancia minima de seguridad", ...
        "Iteraciones hasta la meta", ...
        "Numero de episodios de colision", ...
        "Numero de situaciones de riesgo", ...
        "Tiempo total de planificacion global", ...
        "Tiempo medio del controlador", ...
        "Tiempo medio de ciclo", ...
        "Tiempo computacional total"}, ...
    'unidad', { ...
        "m", ...
        "1/m", ...
        "m", ...
        "pasos", ...
        "episodios", ...
        "episodios", ...
        "s", ...
        "s", ...
        "s", ...
        "s"}, ...
    'slug', { ...
        "longitud", ...
        "suavidad", ...
        "distancia_seguridad", ...
        "iteraciones_hasta_meta", ...
        "colisiones", ...
        "riesgo", ...
        "tiempo_planificacion_total", ...
        "tiempo_control_medio", ...
        "tiempo_ciclo_medio", ...
        "tiempo_computo_total"}, ...
    'soloMeta', { ...
        true, ...
        true, ...
        false, ...
        true, ...
        false, ...
        false, ...
        false, ...
        false, ...
        false, ...
        false});

%% ========================================================================
% LOCALIZACION DEL PROYECTO Y DE LAS SALIDAS
% ========================================================================

raizProyecto = localizar_raiz_proyecto();

carpetaMain = fullfile(raizProyecto,"main");

for i = 1:numel(arquitecturas)
    rutaMain = fullfile(carpetaMain,arquitecturas(i).archivo);

    if ~isfile(rutaMain)
        error('ejecutar_estadisticas:MainAusente', ...
            'No existe el main requerido: %s',rutaMain);
    end
end

% Se incorporan los modulos del proyecto al path. No se usa genpath para
% evitar incluir carpetas de respaldo o documentacion accidentalmente.
carpetasProyecto = [ ...
    "config", ...
    "planificadores", ...
    "controladores", ...
    "simulacion", ...
    "metricas", ...
    "visualizacion", ...
    "herramientas"];

for i = 1:numel(carpetasProyecto)
    carpeta = fullfile(raizProyecto,carpetasProyecto(i));

    if ~isfolder(carpeta)
        error('ejecutar_estadisticas:CarpetaAusente', ...
            'No existe la carpeta requerida: %s',carpeta);
    end

    addpath(char(carpeta),'-begin');
end

rehash path;

cfgReferencia = parametros_generales("batch");

% Condicion de control del experimento: una unica semilla del planificador.
semillaPlanificadorFija = cfgReferencia.semilla;

carpetaEstadisticas = fullfile( ...
    raizProyecto,"salidas","estadisticas",idExperimento);

carpetaFiguras = fullfile(carpetaEstadisticas,"figuras");

if ~isfolder(carpetaFiguras)
    mkdir(carpetaFiguras);
end

archivoBruto = fullfile( ...
    carpetaEstadisticas,"resultados_brutos.csv");

archivoErrores = fullfile( ...
    carpetaEstadisticas,"errores.csv");

archivoResumen = fullfile( ...
    carpetaEstadisticas,"resumen_estadistico.csv");

archivoTasas = fullfile( ...
    carpetaEstadisticas,"tasas_exito.csv");

archivoMAT = fullfile( ...
    carpetaEstadisticas,"resultados_estadisticos.mat");

archivoCheckpoint = fullfile( ...
    carpetaEstadisticas,"checkpoint.mat");

%% ========================================================================
% CARGA OPCIONAL DE UN CHECKPOINT
% ========================================================================

tablaBruta = table();
tablaErrores = table();

if reanudarSiExiste && isfile(archivoBruto)
    tablaBruta = readtable(archivoBruto,'TextType','string');

    fprintf(['Se ha localizado un muestreo previo: %d ejecuciones ' ...
        'completadas.\n'],height(tablaBruta));
end

if reanudarSiExiste && isfile(archivoErrores)
    tablaErrores = readtable(archivoErrores,'TextType','string');
end

%% ========================================================================
% EJECUCION DE LAS 270 SIMULACIONES
% ========================================================================

numeroTotal = ...
    numel(dificultades)*nRepeticiones*numel(arquitecturas);

contadorLote = 0;
contadorNuevas = 0;

fprintf('\n============================================================\n');
fprintf(' MUESTREO ESTADISTICO TFM\n');
fprintf('============================================================\n');
fprintf('Experimento: %s\n',idExperimento);
fprintf('Ejecuciones nominales: %d\n',numeroTotal);
fprintf('Repeticiones por escenario y arquitectura: %d\n',nRepeticiones);
fprintf('Semilla fija del planificador: %d\n',semillaPlanificadorFija);
fprintf('Visualizacion de simulacion: desactivada (modo batch)\n');
fprintf('============================================================\n\n');

for iEscenario = 1:numel(dificultades)
    idEscenario = dificultades(iEscenario);

    for repeticion = 1:nRepeticiones
        semillaPlanificador = semillaPlanificadorFija;
        semillaObstaculos = semillasObstaculos(repeticion);

        for iArquitectura = 1:numel(arquitecturas)
            contadorLote = contadorLote+1;

            nombreArquitectura = arquitecturas(iArquitectura).nombre;
            rutaMain = fullfile( ...
                carpetaMain,arquitecturas(iArquitectura).archivo);

            if ejecucion_ya_completada( ...
                    tablaBruta,idEscenario,nombreArquitectura,repeticion)

                fprintf('[%3d/%3d] OMITIDA  %s | %s | rep %02d\n', ...
                    contadorLote,numeroTotal, ...
                    idEscenario,nombreArquitectura,repeticion);
                continue;
            end

            fprintf('[%3d/%3d] EJECUTANDO %s | %s | rep %02d\n', ...
                contadorLote,numeroTotal, ...
                idEscenario,nombreArquitectura,repeticion);

            try
                [filaNueva,resultadoEjecucion] = ejecutar_main_batch( ...
                    rutaMain, ...
                    idEscenario, ...
                    semillaPlanificador, ...
                    semillaObstaculos);

                if height(filaNueva) ~= 1
                    error('ejecutar_estadisticas:FilaNoUnitaria', ...
                        'Cada main debe producir exactamente una fila.');
                end

                % Identificadores adicionales para el emparejamiento.
                filaNueva.Repeticion = repeticion;
                filaNueva.SemillaPlanificador = semillaPlanificador;

                if isfield(resultadoEjecucion,'identificacion') && ...
                        isfield(resultadoEjecucion.identificacion, ...
                                'semillaObstaculos')
                    semillaObstaculosReal = ...
                        resultadoEjecucion.identificacion.semillaObstaculos;
                else
                    semillaObstaculosReal = semillaObstaculos;
                end

                filaNueva.SemillaObstaculos = semillaObstaculosReal;

                if isempty(tablaBruta)
                    tablaBruta = filaNueva;
                else
                    tablaBruta = [tablaBruta; filaNueva]; %#ok<AGROW>
                end

                contadorNuevas = contadorNuevas+1;

            catch errorEjecucion
                filaError = table( ...
                    string(idEscenario), ...
                    string(nombreArquitectura), ...
                    repeticion, ...
                    semillaPlanificador, ...
                    semillaObstaculos, ...
                    string(errorEjecucion.identifier), ...
                    string(errorEjecucion.message), ...
                    'VariableNames',{ ...
                        'Escenario', ...
                        'Arquitectura', ...
                        'Repeticion', ...
                        'SemillaPlanificador', ...
                        'SemillaObstaculos', ...
                        'IdentificadorError', ...
                        'MensajeError'});

                if isempty(tablaErrores)
                    tablaErrores = filaError;
                else
                    tablaErrores = [tablaErrores; filaError]; %#ok<AGROW>
                end

                warning('ejecutar_estadisticas:EjecucionFallida', ...
                    ['Fallo en %s / %s / repeticion %d:\n%s'], ...
                    idEscenario,nombreArquitectura,repeticion, ...
                    errorEjecucion.message);
            end

            if contadorNuevas > 0 && ...
                    mod(contadorNuevas,guardarCheckpointCada) == 0
                guardar_checkpoint( ...
                    tablaBruta,tablaErrores, ...
                    archivoBruto,archivoErrores,archivoCheckpoint);
            end
        end
    end
end

% Ultimo checkpoint.
guardar_checkpoint( ...
    tablaBruta,tablaErrores, ...
    archivoBruto,archivoErrores,archivoCheckpoint);

%% ========================================================================
% COMPROBACION DEL EMPAREJAMIENTO
% ========================================================================

validar_emparejamiento( ...
    tablaBruta,dificultades,arquitecturas,nRepeticiones, ...
    semillaPlanificadorFija);

%% ========================================================================
% VARIABLES DERIVADAS PARA EL ANALISIS
% ========================================================================

tablaAnalisis = tablaBruta;

tablaAnalisis.PasosHastaMeta = ...
    nan(height(tablaAnalisis),1);

mascaraMeta = logical(tablaAnalisis.MetaAlcanzada);

tablaAnalisis.PasosHastaMeta(mascaraMeta) = ...
    tablaAnalisis.PasosEjecutados(mascaraMeta);

%% ========================================================================
% TABLA DE ESTADISTICA DESCRIPTIVA
% ========================================================================

tablaResumen = table();

for iEscenario = 1:numel(dificultades)
    idEscenario = dificultades(iEscenario);

    for iArquitectura = 1:numel(arquitecturas)
        nombreArquitectura = arquitecturas(iArquitectura).nombre;

        grupo = tablaAnalisis( ...
            string(tablaAnalisis.Escenario) == idEscenario & ...
            string(tablaAnalisis.Arquitectura) == nombreArquitectura,:);

        for iMetrica = 1:numel(metricas)
            m = metricas(iMetrica);

            if m.soloMeta
                grupoMetrica = grupo(logical(grupo.MetaAlcanzada),:);
                poblacion = "ejecuciones_que_alcanzan_meta";
            else
                grupoMetrica = grupo;
                poblacion = "todas_las_ejecuciones";
            end

            valores = grupoMetrica.(char(m.campo));

            filaResumen = resumir_metrica( ...
                idEscenario, ...
                nombreArquitectura, ...
                m.etiqueta, ...
                m.unidad, ...
                poblacion, ...
                valores);

            if isempty(tablaResumen)
                tablaResumen = filaResumen;
            else
                tablaResumen = [tablaResumen; filaResumen]; %#ok<AGROW>
            end
        end
    end
end

writetable(tablaResumen,archivoResumen);

%% ========================================================================
% TASA DE EXITO Y TASA DE LLEGADA A META
% ========================================================================

% tasa_exito.m utiliza actualmente el criterio configurado en cfg:
%   meta alcanzada
%   Y ausencia de colision
%   Y ejecucion dentro del tiempo maximo
%
% Para no perder informacion se calcula ademas TasaMeta_pct, que responde
% solamente a la pregunta "¿alcanzo la meta?".

tablaTasas = table();

for iEscenario = 1:numel(dificultades)
    idEscenario = dificultades(iEscenario);

    for iArquitectura = 1:numel(arquitecturas)
        nombreArquitectura = arquitecturas(iArquitectura).nombre;

        grupo = tablaAnalisis( ...
            string(tablaAnalisis.Escenario) == idEscenario & ...
            string(tablaAnalisis.Arquitectura) == nombreArquitectura,:);

        if isempty(grupo)
            continue;
        end

        meta = logical(grupo.MetaAlcanzada);
        colision = logical(grupo.Colision);
        pasos = grupo.PasosEjecutados;

        [tasaExito,~,detalleExito] = tasa_exito( ...
            meta,colision,pasos,cfgReferencia);

        n = height(grupo);
        nMeta = nnz(meta);
        tasaMeta = 100*nMeta/n;
        icMeta = 100*intervalo_wilson_local(nMeta,n,0.95);

        filaTasa = table( ...
            string(idEscenario), ...
            string(nombreArquitectura), ...
            n, ...
            detalleExito.numeroExitos, ...
            tasaExito, ...
            detalleExito.intervaloConfianza95Porcentaje(1), ...
            detalleExito.intervaloConfianza95Porcentaje(2), ...
            nMeta, ...
            tasaMeta, ...
            icMeta(1), ...
            icMeta(2), ...
            nnz(colision), ...
            'VariableNames',{ ...
                'Escenario', ...
                'Arquitectura', ...
                'N', ...
                'ExitosCriterioEstricto', ...
                'TasaExito_pct', ...
                'IC95Exito_Inferior_pct', ...
                'IC95Exito_Superior_pct', ...
                'LlegadasMeta', ...
                'TasaMeta_pct', ...
                'IC95Meta_Inferior_pct', ...
                'IC95Meta_Superior_pct', ...
                'EjecucionesConAlgunaColision'});

        if isempty(tablaTasas)
            tablaTasas = filaTasa;
        else
            tablaTasas = [tablaTasas; filaTasa]; %#ok<AGROW>
        end
    end
end

writetable(tablaTasas,archivoTasas);

%% ========================================================================
% FIGURAS: BOXPLOTS + MEDIA +/- DESVIACION TIPICA
% ========================================================================

ordenArquitecturas = string({arquitecturas.nombre});

for iEscenario = 1:numel(dificultades)
    idEscenario = dificultades(iEscenario);

    tablaEscenario = tablaAnalisis( ...
        string(tablaAnalisis.Escenario) == idEscenario,:);

    for iMetrica = 1:numel(metricas)
        m = metricas(iMetrica);

        crear_boxplot_comparativo( ...
            tablaEscenario, ...
            idEscenario, ...
            ordenArquitecturas, ...
            m, ...
            carpetaFiguras, ...
            mostrarFiguras);
    end

    crear_grafico_tasa( ...
        tablaTasas, ...
        idEscenario, ...
        ordenArquitecturas, ...
        "TasaExito_pct", ...
        "IC95Exito_Inferior_pct", ...
        "IC95Exito_Superior_pct", ...
        "Tasa de exito (criterio configurado)", ...
        "tasa_exito_estricto", ...
        carpetaFiguras, ...
        mostrarFiguras);

    if generarGraficoLlegadaMeta
        crear_grafico_tasa( ...
            tablaTasas, ...
            idEscenario, ...
            ordenArquitecturas, ...
            "TasaMeta_pct", ...
            "IC95Meta_Inferior_pct", ...
            "IC95Meta_Superior_pct", ...
            "Tasa de llegada a la meta", ...
            "tasa_llegada_meta", ...
            carpetaFiguras, ...
            mostrarFiguras);
    end
end

%% ========================================================================
% GUARDADO FINAL
% ========================================================================

configExperimento = struct();
configExperimento.id = idExperimento;
configExperimento.dificultades = dificultades;
configExperimento.arquitecturas = ordenArquitecturas;
configExperimento.nRepeticiones = nRepeticiones;
configExperimento.semillaPlanificadorFija = semillaPlanificadorFija;
configExperimento.semillasObstaculos = semillasObstaculos;
configExperimento.fecha = datetime('now');
configExperimento.criterioExito = cfgReferencia.evaluacion.exito;
configExperimento.periodoMuestreo_s = cfgReferencia.sim.Ts;
configExperimento.maxPasos = cfgReferencia.terminacion.maxPasos;

save(archivoMAT, ...
    'tablaBruta', ...
    'tablaAnalisis', ...
    'tablaResumen', ...
    'tablaTasas', ...
    'tablaErrores', ...
    'configExperimento', ...
    '-v7.3');

fprintf('\n============================================================\n');
fprintf(' MUESTREO FINALIZADO\n');
fprintf('============================================================\n');
fprintf('Filas de resultados: %d\n',height(tablaBruta));
fprintf('Errores de ejecucion: %d\n',height(tablaErrores));
fprintf('Resultados brutos:\n  %s\n',archivoBruto);
fprintf('Resumen estadistico:\n  %s\n',archivoResumen);
fprintf('Tasas de exito:\n  %s\n',archivoTasas);
fprintf('Figuras:\n  %s\n',carpetaFiguras);
fprintf('MAT completo:\n  %s\n',archivoMAT);
fprintf('============================================================\n');

disp(" ");
disp("Tasas de exito y llegada a meta:");
disp(tablaTasas);

%% ========================================================================
% FUNCIONES LOCALES
% ========================================================================

function raiz = localizar_raiz_proyecto()
%LOCALIZAR_RAIZ_PROYECTO Localiza la raiz de TFM_RobotNavigation.

raiz = "";

try
    proyecto = matlab.project.rootProject;

    if ~isempty(proyecto)
        candidato = string(proyecto.RootFolder);

        if es_raiz_valida(candidato)
            raiz = candidato;
            return;
        end
    end
catch
end

candidatos = string(pwd);

try
    archivoActivo = matlab.desktop.editor.getActiveFilename;

    if ~isempty(archivoActivo)
        candidatos = [ ...
            string(fileparts(archivoActivo)); ...
            candidatos];
    end
catch
end

for i = 1:numel(candidatos)
    candidato = candidatos(i);

    for nivel = 1:12
        if es_raiz_valida(candidato)
            raiz = candidato;
            return;
        end

        padre = string(fileparts(char(candidato)));

        if strlength(padre) == 0 || padre == candidato
            break;
        end

        candidato = padre;
    end
end

error('ejecutar_estadisticas:RaizNoEncontrada', ...
    ['No se encontro la raiz del proyecto. Abra el proyecto MATLAB ' ...
     'o seleccione como carpeta actual la raiz que contiene main, ' ...
     'config, planificadores, controladores, simulacion y metricas.']);
end

function tf = es_raiz_valida(carpeta)

tf = ...
    strlength(carpeta) > 0 && ...
    isfolder(carpeta) && ...
    isfolder(fullfile(carpeta,"main")) && ...
    isfolder(fullfile(carpeta,"config")) && ...
    isfolder(fullfile(carpeta,"planificadores")) && ...
    isfolder(fullfile(carpeta,"controladores")) && ...
    isfolder(fullfile(carpeta,"simulacion")) && ...
    isfolder(fullfile(carpeta,"metricas"));
end

function tf = ejecucion_ya_completada( ...
    tabla,idEscenario,nombreArquitectura,repeticion)

if isempty(tabla)
    tf = false;
    return;
end

campos = string(tabla.Properties.VariableNames);

if ~all(ismember( ...
        ["Escenario","Arquitectura","Repeticion"],campos))
    tf = false;
    return;
end

tf = any( ...
    string(tabla.Escenario) == string(idEscenario) & ...
    string(tabla.Arquitectura) == string(nombreArquitectura) & ...
    tabla.Repeticion == repeticion);
end

function [filaResultado,resultado] = ejecutar_main_batch( ...
    rutaMain,idEscenario,semillaPlanificador,semillaObstaculosSolicitada)
%EJECUTAR_MAIN_BATCH Ejecuta un main probado sin modificar su archivo.
%
% Se sustituye solamente:
%   - visual -> batch
%   - escenario
%   - semilla del planificador
%   - semilla reproducible de obstaculos
%   - exportacion/guardado individual -> false
%
% clearvars/clc/close all se neutralizan para no destruir el workspace
% local que recibe filaResultado y resultado.

codigo = fileread(rutaMain);

codigo = reemplazar_patron_unico( ...
    codigo, ...
    '(?m)^\s*clearvars\s*;', ...
    '% clearvars omitido por ejecutar_estadisticas', ...
    "clearvars");

codigo = reemplazar_patron_unico( ...
    codigo, ...
    '(?m)^\s*clc\s*;', ...
    '% clc omitido por ejecutar_estadisticas', ...
    "clc");

codigo = reemplazar_patron_unico( ...
    codigo, ...
    '(?m)^\s*close\s+all\s*;', ...
    '% close all omitido por ejecutar_estadisticas', ...
    "close all");

codigo = reemplazar_patron_unico( ...
    codigo, ...
    '(?m)^\s*modoEjecucion\s*=\s*"[^"]+"\s*;', ...
    'modoEjecucion = "batch";', ...
    "modoEjecucion");

codigo = reemplazar_patron_unico( ...
    codigo, ...
    '(?m)^\s*idEscenario\s*=\s*"[^"]+"\s*;', ...
    sprintf('idEscenario = "%s";',char(idEscenario)), ...
    "idEscenario");

codigo = reemplazar_patron_unico( ...
    codigo, ...
    '(?m)^\s*semillaPlanificador\s*=\s*[0-9]+\s*;', ...
    sprintf('semillaPlanificador = %d;',semillaPlanificador), ...
    "semillaPlanificador");

codigo = reemplazar_patron_unico( ...
    codigo, ...
    '(?m)^\s*modoSemillaObstaculos\s*=\s*"[^"]+"\s*;', ...
    'modoSemillaObstaculos = "reproducible";', ...
    "modoSemillaObstaculos");

codigo = reemplazar_patron_unico( ...
    codigo, ...
    '(?m)^\s*semillaObstaculosFija\s*=.*?;', ...
    sprintf('semillaObstaculosFija = %d;', ...
        semillaObstaculosSolicitada), ...
    "semillaObstaculosFija");

codigo = reemplazar_patron_unico( ...
    codigo, ...
    '(?m)^\s*exportarAnimacion\s*=\s*(true|false)\s*;', ...
    'exportarAnimacion = false;', ...
    "exportarAnimacion");

codigo = reemplazar_patron_unico( ...
    codigo, ...
    '(?m)^\s*guardarResultado\s*=\s*(true|false)\s*;', ...
    'guardarResultado = false;', ...
    "guardarResultado");

% evalc evita imprimir 270 resúmenes individuales en la consola.
salidaCapturada = evalc(codigo); %#ok<NASGU>

if ~exist('filaResultado','var') || ~istable(filaResultado)
    error('ejecutar_estadisticas:SinFilaResultado', ...
        'El main %s no genero filaResultado.',rutaMain);
end

if ~exist('resultado','var') || ~isstruct(resultado)
    error('ejecutar_estadisticas:SinResultado', ...
        'El main %s no genero resultado.',rutaMain);
end
end

function texto = reemplazar_patron_unico( ...
    texto,patron,reemplazo,etiqueta)

coincidencias = regexp(texto,patron,'match');

if numel(coincidencias) ~= 1
    error('ejecutar_estadisticas:PatronNoUnico', ...
        ['No se pudo preparar el main. El patron "%s" aparecio %d ' ...
         'veces. Revise la cabecera del main.'], ...
        etiqueta,numel(coincidencias));
end

texto = regexprep(texto,patron,reemplazo,'once');
end

function guardar_checkpoint( ...
    tablaBruta,tablaErrores,archivoBruto,archivoErrores,archivoMAT)

if ~isempty(tablaBruta)
    writetable(tablaBruta,archivoBruto);
end

if ~isempty(tablaErrores)
    writetable(tablaErrores,archivoErrores);
end

save(archivoMAT,'tablaBruta','tablaErrores','-v7.3');
end

function validar_emparejamiento( ...
    tabla,dificultades,arquitecturas,nRepeticiones, ...
    semillaPlanificadorFija)

if isempty(tabla)
    error('ejecutar_estadisticas:SinResultados', ...
        'No se obtuvo ninguna ejecucion valida.');
end

for iEscenario = 1:numel(dificultades)
    idEscenario = dificultades(iEscenario);

    for repeticion = 1:nRepeticiones
        filas = tabla( ...
            string(tabla.Escenario) == idEscenario & ...
            tabla.Repeticion == repeticion,:);

        % Si hubo un error de codigo puede faltar una arquitectura. Se
        % advierte, pero no se confunde con un fallo del algoritmo.
        if height(filas) ~= numel(arquitecturas)
            warning('ejecutar_estadisticas:EmparejamientoIncompleto', ...
                ['Escenario %s, repeticion %d: hay %d de %d ' ...
                 'arquitecturas registradas.'], ...
                idEscenario,repeticion,height(filas), ...
                numel(arquitecturas));
            continue;
        end

        semillasEntorno = unique(filas.SemillaObstaculos);

        if numel(semillasEntorno) ~= 1
            error('ejecutar_estadisticas:SemillasEntornoNoEmparejadas', ...
                ['Las tres arquitecturas no utilizaron la misma semilla ' ...
                 'de obstaculos en %s, repeticion %d.'], ...
                idEscenario,repeticion);
        end

        % Las tres arquitecturas deben utilizar la misma semilla fija del
        % planificador durante todo el experimento.
        semillasPlanificadorGrupo = unique( ...
            filas.SemillaPlanificador);

        if numel(semillasPlanificadorGrupo) ~= 1 || ...
                semillasPlanificadorGrupo(1) ~= semillaPlanificadorFija
            error('ejecutar_estadisticas:SemillaPlanificadorNoFija', ...
                ['La semilla del planificador no es fija y comun en %s, ' ...
                 'repeticion %d. Se esperaba %d.'], ...
                idEscenario,repeticion,semillaPlanificadorFija);
        end
    end
end
end

function fila = resumir_metrica( ...
    escenario,arquitectura,metrica,unidad,poblacion,valores)

valores = double(valores(:));
valores = valores(isfinite(valores));

n = numel(valores);

if n == 0
    media = NaN;
    desviacion = NaN;
    mediana = NaN;
    minimo = NaN;
    maximo = NaN;
    ic = [NaN NaN];
    metodoIC = "sin_datos";

elseif n == 1
    media = valores(1);
    desviacion = NaN;
    mediana = valores(1);
    minimo = valores(1);
    maximo = valores(1);
    ic = [NaN NaN];
    metodoIC = "n_insuficiente";

else
    media = mean(valores);
    desviacion = std(valores,0);
    mediana = median(valores);
    minimo = min(valores);
    maximo = max(valores);

    [ic,metodoIC] = intervalo_media95(media,desviacion,n);
end

fila = table( ...
    string(escenario), ...
    string(arquitectura), ...
    string(metrica), ...
    string(unidad), ...
    string(poblacion), ...
    n, ...
    media, ...
    desviacion, ...
    mediana, ...
    minimo, ...
    maximo, ...
    ic(1), ...
    ic(2), ...
    string(metodoIC), ...
    'VariableNames',{ ...
        'Escenario', ...
        'Arquitectura', ...
        'Metrica', ...
        'Unidad', ...
        'Poblacion', ...
        'N', ...
        'Media', ...
        'DesvTipica', ...
        'Mediana', ...
        'Minimo', ...
        'Maximo', ...
        'IC95_Inferior', ...
        'IC95_Superior', ...
        'MetodoIC95'});
end

function [ic,metodo] = intervalo_media95(media,desviacion,n)

errorEstandar = desviacion/sqrt(n);

if exist('tinv','file') == 2
    critico = tinv(0.975,n-1);
    metodo = "t_student";
else
    % Fallback sin Statistics and Machine Learning Toolbox.
    critico = 1.95996398454005;
    metodo = "normal_1_96";
end

margen = critico*errorEstandar;
ic = [media-margen media+margen];
end

function crear_boxplot_comparativo( ...
    tablaEscenario,idEscenario,ordenArquitecturas,metrica, ...
    carpetaFiguras,mostrarFiguras)

datos = tablaEscenario;

if metrica.soloMeta
    datos = datos(logical(datos.MetaAlcanzada),:);
end

if isempty(datos) || ...
        ~ismember(metrica.campo,string(datos.Properties.VariableNames))
    return;
end

valores = double(datos.(char(metrica.campo)));
grupos = nan(height(datos),1);

for i = 1:numel(ordenArquitecturas)
    grupos(string(datos.Arquitectura) == ordenArquitecturas(i)) = i;
end

mascara = isfinite(valores) & isfinite(grupos);
valores = valores(mascara);
grupos = grupos(mascara);

if isempty(valores)
    return;
end

if mostrarFiguras
    visibilidad = "on";
else
    visibilidad = "off";
end

figura = figure( ...
    'Color','w', ...
    'Visible',visibilidad, ...
    'Name',char(metrica.etiqueta));

hold on;

if exist('boxchart','file') == 2
    boxchart(grupos,valores);
elseif exist('boxplot','file') == 2
    boxplot(valores,grupos, ...
        'Labels',cellstr(ordenArquitecturas));
else
    warning('ejecutar_estadisticas:SinBoxplot', ...
        ['No se encontro boxchart ni boxplot. No se generara la ' ...
         'figura %s.'],metrica.etiqueta);
    close(figura);
    return;
end

% Media +/- una desviacion tipica.
medias = nan(1,numel(ordenArquitecturas));
desviaciones = nan(1,numel(ordenArquitecturas));

for i = 1:numel(ordenArquitecturas)
    x = valores(grupos == i);

    if isempty(x)
        continue;
    end

    medias(i) = mean(x);

    if numel(x) >= 2
        desviaciones(i) = std(x,0);
    else
        desviaciones(i) = 0;
    end
end

hMedia = plot( ...
    1:numel(ordenArquitecturas), ...
    medias, ...
    'd', ...
    'LineStyle','none', ...
    'MarkerSize',7, ...
    'LineWidth',1.2);

hSD = errorbar( ...
    1:numel(ordenArquitecturas), ...
    medias, ...
    desviaciones, ...
    'LineStyle','none', ...
    'CapSize',8, ...
    'LineWidth',1.1);

xticks(1:numel(ordenArquitecturas));
xticklabels(ordenArquitecturas);
xtickangle(0);

xlabel("Arquitectura");

if strlength(metrica.unidad) > 0
    ylabel(metrica.etiqueta+" ["+metrica.unidad+"]");
else
    ylabel(metrica.etiqueta);
end

titulo = metrica.etiqueta+" - dificultad "+idEscenario;

if metrica.soloMeta
    titulo = titulo+" (solo ejecuciones que alcanzan la meta)";
end

title(titulo,'Interpreter','none');

grid on;
box on;

legend( ...
    [hMedia hSD], ...
    ["Media","Media +/- 1 desviacion tipica"], ...
    'Location','best');

nombreBase = ...
    "box_"+idEscenario+"_"+metrica.slug;

rutaPNG = fullfile(carpetaFiguras,nombreBase+".png");
rutaFIG = fullfile(carpetaFiguras,nombreBase+".fig");

exportgraphics(figura,rutaPNG,'Resolution',200);
savefig(figura,rutaFIG);

if ~mostrarFiguras
    close(figura);
end
end

function crear_grafico_tasa( ...
    tablaTasas,idEscenario,ordenArquitecturas, ...
    campoTasa,campoInferior,campoSuperior, ...
    tituloFigura,slug,carpetaFiguras,mostrarFiguras)

grupo = tablaTasas( ...
    string(tablaTasas.Escenario) == idEscenario,:);

if isempty(grupo)
    return;
end

tasas = nan(1,numel(ordenArquitecturas));
inferior = nan(1,numel(ordenArquitecturas));
superior = nan(1,numel(ordenArquitecturas));

for i = 1:numel(ordenArquitecturas)
    fila = grupo( ...
        string(grupo.Arquitectura) == ordenArquitecturas(i),:);

    if isempty(fila)
        continue;
    end

    valorTasa = fila.(char(campoTasa));
    valorInferior = fila.(char(campoInferior));
    valorSuperior = fila.(char(campoSuperior));

    tasas(i) = valorTasa(1);
    inferior(i) = valorInferior(1);
    superior(i) = valorSuperior(1);
end

if mostrarFiguras
    visibilidad = "on";
else
    visibilidad = "off";
end

figura = figure( ...
    'Color','w', ...
    'Visible',visibilidad, ...
    'Name',char(tituloFigura));

bar(1:numel(ordenArquitecturas),tasas);
hold on;

errorInferior = tasas-inferior;
errorSuperior = superior-tasas;

errorbar( ...
    1:numel(ordenArquitecturas), ...
    tasas, ...
    errorInferior, ...
    errorSuperior, ...
    'LineStyle','none', ...
    'CapSize',8, ...
    'LineWidth',1.1);

xticks(1:numel(ordenArquitecturas));
xticklabels(ordenArquitecturas);
ylabel("Porcentaje [%]");
xlabel("Arquitectura");
ylim([0 100]);
title(tituloFigura+" - dificultad "+idEscenario, ...
    'Interpreter','none');
grid on;
box on;

nombreBase = slug+"_"+idEscenario;

exportgraphics( ...
    figura, ...
    fullfile(carpetaFiguras,nombreBase+".png"), ...
    'Resolution',200);

savefig( ...
    figura, ...
    fullfile(carpetaFiguras,nombreBase+".fig"));

if ~mostrarFiguras
    close(figura);
end
end

function intervalo = intervalo_wilson_local(exitos,n,nivel)

if n <= 0
    intervalo = [NaN NaN];
    return;
end

% Para 95 %, z = 1.9599639845. Se deja el argumento nivel para que la
% llamada documente el nivel utilizado.
if abs(nivel-0.95) > 1e-12
    error('ejecutar_estadisticas:NivelWilsonNoSoportado', ...
        'Esta implementacion local utiliza IC de Wilson al 95 %.');
end

z = 1.95996398454005;
p = exitos/n;
z2 = z^2;

centro = (p+z2/(2*n))/(1+z2/n);

semiancho = ...
    z/(1+z2/n)*sqrt( ...
        p*(1-p)/n + z2/(4*n^2));

intervalo = [ ...
    max(0,centro-semiancho), ...
    min(1,centro+semiancho)];
end
