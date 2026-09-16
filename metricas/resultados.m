function [resultado, fila] = resultados(registro, escenario, robot, cfg)
% Calcula y reúne los resultados de una ejecución completa.
%
%   resultado = RESULTADOS(registro,escenario,robot,cfg)
%
%   [resultado,fila] = RESULTADOS( ...
%       registro,escenario,robot,cfg)
%
%   Este módulo constituye la interfaz común entre el bucle de simulación
%   y el posterior análisis estadístico.
%
%   El bucle principal registra los datos brutos de una ejecución:
%
%       - trayectoria ejecutada;
%       - posiciones de los obstáculos dinámicos;
%       - controles aplicados;
%       - tiempos medidos;
%       - número de pasos;
%       - número de replanificaciones.
%
%   RESULTADOS recibe esos datos al terminar la ejecución y:
%
%       1. recorta los historiales preasignados;
%       2. calcula las métricas mediante los módulos especializados;
%       3. determina llegada a meta, colisión y éxito;
%       4. reúne todo en una estructura normalizada;
%       5. genera una fila de tabla para acumular repeticiones.
%
%   Por tanto, RESULTADOS no registra datos dentro del bucle y tampoco
%   realiza todavía la comparación estadística entre APF y MPC.
%
%   Campos obligatorios de registro:
%
%       registro.pasosEjecutados
%           Número de actualizaciones del modelo realizadas.
%
%       registro.trayectoria
%           Historial de estados. Debe contener al menos
%           pasosEjecutados+1 filas:
%
%               [estado inicial;
%                estado después del paso 1;
%                ...
%                estado después del último paso]
%
%           Las columnas deben incluir, como mínimo, [x y].
%
%       registro.historialObstaculos
%           Historial N x D x 2 de posiciones de los D obstáculos
%           dinámicos. Debe estar sincronizado con registro.trayectoria.
%
%       registro.controles
%           Historial de controles aplicados, con formato
%           pasosEjecutados x 2:
%
%               [v w]
%
%       registro.tiempos
%           Estructura compatible con tiempos.m:
%
%               .planificacion
%               .control
%               .ciclo
%
%   Campos recomendados:
%
%       registro.arquitectura
%           Por ejemplo, "RRT* + APF" o "RRT* + MPC".
%           Si falta, se usa cfg.nombreArquitectura.
%
%       registro.semilla
%           Semilla de la ejecución. Si falta, se usa cfg.semilla.
%
%       registro.numeroReplanificaciones
%           Número de replanificaciones, sin contar la planificación
%           global inicial. Si falta, se registra NaN.
%
%       registro.numeroFallosPlanificacion
%           Número de llamadas al planificador que no produjeron una ruta.
%           Si falta, se registra NaN.
%
%   Salida resultado:
%
%       Estructura jerárquica con:
%
%           .identificacion
%           .terminacion
%           .metricas
%           .contadores
%           .detalles
%           .historiales
%           .validacion
%
%   Salida fila:
%
%       Tabla de una sola fila con las variables principales. Las filas de
%       las distintas semillas pueden concatenarse:
%
%           tablaResultados = [tablaResultados; fila];
%
%       El futuro ejecutar_estadisticas.mlx utilizará esa tabla para
%       calcular media, desviación estándar, mediana, mínimo, máximo,
%       intervalos de confianza y comparaciones pareadas.
%
%   Métricas calculadas:
%
%       - longitud total de la trayectoria;
%       - suavidad mediante curvatura RMS;
%       - distancia mínima de seguridad;
%       - episodios de riesgo y colisión;
%       - éxito individual;
%       - tiempos de planificación, control y ciclo;
%       - esfuerzo de control normalizado;
%       - variación normalizada del control;
%       - número de replanificaciones.
%
%   Esfuerzo de control normalizado:
%
%       J_u = Ts * sum_k [ ...
%           (v_k/v_max)^2 + (w_k/w_max)^2 ]
%
%   Esta normalización evita sumar directamente magnitudes con unidades
%   distintas. Un valor menor implica menor esfuerzo acumulado relativo.
%
%   Ejemplo:
%
%       registro = struct();
%       registro.arquitectura = "RRT* + MPC";
%       registro.semilla = cfg.semilla;
%       registro.pasosEjecutados = pasosEjecutados;
%       registro.trayectoria = trayectoriaEjecutada;
%       registro.historialObstaculos = historialDinamicos;
%       registro.controles = controlesAplicados;
%       registro.tiempos = registroTiempos;
%       registro.numeroReplanificaciones = numeroReplanificaciones;
%       registro.numeroFallosPlanificacion = numeroFallosPlanificacion;
%
%       [resultado,fila] = resultados( ...
%           registro,escenario,robot,cfg);
%
%   Esta función utiliza:
%       - longitud.m
%       - suavidad.m
%       - distancia_seguridad.m
%       - tasa_exito.m
%       - tiempos.m

%% Validación, identificación y recorte de historiales
datos = validar_y_preparar(registro,escenario,robot,cfg);

trayectoria = datos.trayectoria;
historialObstaculos = datos.historialObstaculos;
controles = datos.controles;
pasosEjecutados = datos.pasosEjecutados;

%% Llegada geométrica a la meta
distanciasMeta = hypot( ...
    trayectoria(:,1)-escenario.meta(1), ...
    trayectoria(:,2)-escenario.meta(2));

valoresEscalaMeta = [
    trayectoria(:,1:2);
    reshape(double(escenario.meta),1,2)
];

toleranciaMeta = ...
    1e-12*max(1,max(abs(valoresEscalaMeta(:))));

dentroMeta = ...
    distanciasMeta <= cfg.navegacion.radioMeta+toleranciaMeta;

metaAlcanzada = any(dentroMeta);

if metaAlcanzada
    indiceEstadoMeta = find(dentroMeta,1,'first');
    pasoMeta = indiceEstadoMeta-1;
    tiempoHastaMeta = pasoMeta*cfg.sim.Ts;
else
    indiceEstadoMeta = NaN;
    pasoMeta = NaN;
    tiempoHastaMeta = NaN;
end

distanciaFinalMeta = distanciasMeta(end);
distanciaMinimaMeta = min(distanciasMeta);

%% Métrica de longitud
[longitudTotal,longitudesSegmentos] = ...
    longitud(trayectoria);

%% Métrica de suavidad
[suavidadRMS,detalleSuavidad] = ...
    suavidad(trayectoria);

%% Métrica de seguridad
[distanciaMinima,detalleSeguridad] = ...
    distancia_seguridad( ...
        trayectoria, ...
        historialObstaculos, ...
        escenario, ...
        robot, ...
        cfg);

colisionOcurrida = ...
    detalleSeguridad.colisionOcurrida;

%% Éxito de la ejecución
[tasaIndividual,exito,detalleExito] = ...
    tasa_exito( ...
        metaAlcanzada, ...
        colisionOcurrida, ...
        pasosEjecutados, ...
        cfg);

exito = logical(exito(1));

%% Métricas temporales
registroTiempos = datos.registroTiempos;
registroTiempos.metaAlcanzada = metaAlcanzada;

[resumenTiempos,detalleTiempos] = ...
    tiempos( ...
        registroTiempos, ...
        pasosEjecutados, ...
        cfg);

% resultados.m conoce el primer instante exacto de entrada en la región
% de meta. Se conserva este valor como referencia experimental principal.
resumenTiempos.tiempoHastaMetaSimulado = tiempoHastaMeta;

%% Esfuerzo y regularidad del control
[resumenControl,detalleControl] = ...
    resumir_control( ...
        controles, ...
        cfg.sim.Ts, ...
        robot);

%% Motivo de terminación
motivoTerminacion = determinar_terminacion( ...
    metaAlcanzada, ...
    colisionOcurrida, ...
    pasosEjecutados, ...
    cfg);

%% Consistencia entre contadores
numeroPlanificaciones = ...
    resumenTiempos.numeroPlanificaciones;

if isnan(datos.numeroReplanificaciones)
    contadorPlanificacionCoherente = true;
else
    % La planificación inicial no se considera replanificación.
    contadorPlanificacionCoherente = ...
        numeroPlanificaciones >= datos.numeroReplanificaciones;
end

%% Estructura normalizada de resultados
resultado = struct();

resultado.esquema = "resultado_ejecucion_v1";

resultado.identificacion = struct();
resultado.identificacion.arquitectura = datos.arquitectura;
resultado.identificacion.escenario = string(escenario.id);
resultado.identificacion.nombreEscenario = string(escenario.nombre);
resultado.identificacion.semilla = datos.semilla;
resultado.identificacion.versionConfiguracion = string(cfg.version);
resultado.identificacion.modeloRobot = string(robot.tipo);
resultado.identificacion.periodoMuestreo = cfg.sim.Ts;

resultado.terminacion = struct();
resultado.terminacion.exito = exito;
resultado.terminacion.tasaIndividualPorcentaje = tasaIndividual;
resultado.terminacion.metaAlcanzada = metaAlcanzada;
resultado.terminacion.colisionOcurrida = colisionOcurrida;
resultado.terminacion.motivo = motivoTerminacion;
resultado.terminacion.pasosEjecutados = pasosEjecutados;
resultado.terminacion.numeroEstados = size(trayectoria,1);
resultado.terminacion.indiceEstadoMeta = indiceEstadoMeta;
resultado.terminacion.pasoMeta = pasoMeta;
resultado.terminacion.tiempoHastaMeta = tiempoHastaMeta;
resultado.terminacion.estadoInicial = trayectoria(1,:);
resultado.terminacion.estadoFinal = trayectoria(end,:);
resultado.terminacion.distanciaFinalMeta = distanciaFinalMeta;
resultado.terminacion.distanciaMinimaMeta = distanciaMinimaMeta;

resultado.metricas = struct();

resultado.metricas.longitud = struct();
resultado.metricas.longitud.total = longitudTotal;
resultado.metricas.longitud.unidad = "m";

resultado.metricas.suavidad = struct();
resultado.metricas.suavidad.curvaturaRMS = suavidadRMS;
resultado.metricas.suavidad.unidad = "1/m";
resultado.metricas.suavidad.criterio = ...
    "menor_valor_mayor_suavidad";

resultado.metricas.seguridad = struct();
resultado.metricas.seguridad.distanciaMinima = distanciaMinima;
resultado.metricas.seguridad.unidad = "m";
resultado.metricas.seguridad.numeroEpisodiosRiesgo = ...
    detalleSeguridad.numeroEpisodiosRiesgo;
resultado.metricas.seguridad.numeroEpisodiosColision = ...
    detalleSeguridad.numeroEpisodiosColision;
resultado.metricas.seguridad.penetracionMaxima = ...
    detalleSeguridad.penetracionMaxima;
resultado.metricas.seguridad.criterio = ...
    "mayor_valor_mayor_seguridad";

resultado.metricas.exito = struct();
resultado.metricas.exito.indicador = exito;
resultado.metricas.exito.valorPorcentajeIndividual = tasaIndividual;
resultado.metricas.exito.criterio = ...
    "meta_sin_colision_y_dentro_del_tiempo";

resultado.metricas.tiempos = resumenTiempos;
resultado.metricas.control = resumenControl;

resultado.contadores = struct();
resultado.contadores.numeroPlanificaciones = ...
    numeroPlanificaciones;
resultado.contadores.numeroReplanificaciones = ...
    datos.numeroReplanificaciones;
resultado.contadores.numeroFallosPlanificacion = ...
    datos.numeroFallosPlanificacion;
resultado.contadores.numeroControles = ...
    resumenTiempos.numeroControles;
resultado.contadores.numeroPasosRiesgo = ...
    detalleSeguridad.numeroPasosRiesgo;
resultado.contadores.numeroEpisodiosRiesgo = ...
    detalleSeguridad.numeroEpisodiosRiesgo;
resultado.contadores.numeroPasosColision = ...
    detalleSeguridad.numeroPasosColision;
resultado.contadores.numeroEpisodiosColision = ...
    detalleSeguridad.numeroEpisodiosColision;

resultado.detalles = struct();
resultado.detalles.longitudesSegmentos = ...
    longitudesSegmentos;
resultado.detalles.suavidad = ...
    detalleSuavidad;
resultado.detalles.seguridad = ...
    detalleSeguridad;
resultado.detalles.exito = ...
    detalleExito;
resultado.detalles.tiempos = ...
    detalleTiempos;
resultado.detalles.control = ...
    detalleControl;
resultado.detalles.distanciasMeta = ...
    distanciasMeta;

resultado.validacion = struct();
resultado.validacion.historialesSincronizados = true;
resultado.validacion.numeroEstadosCorrecto = ...
    size(trayectoria,1) == pasosEjecutados+1;
resultado.validacion.numeroControlesCorrecto = ...
    size(controles,1) == pasosEjecutados;
resultado.validacion.contadorPlanificacionCoherente = ...
    contadorPlanificacionCoherente;
resultado.validacion.metricasCalculadas = true;

%% Historiales opcionales para reproducibilidad
resultado.historiales = struct();

if cfg.metricas.guardarTrayectoria
    resultado.historiales.trayectoria = trayectoria;
else
    resultado.historiales.trayectoria = zeros(0,size(trayectoria,2));
end

if cfg.metricas.guardarControles
    resultado.historiales.controles = controles;
else
    resultado.historiales.controles = zeros(0,2);
end

if cfg.metricas.guardarObstaculos
    resultado.historiales.obstaculosDinamicos = ...
        historialObstaculos;
else
    resultado.historiales.obstaculosDinamicos = ...
        zeros(0,0,2);
end

if cfg.metricas.guardarTiemposCiclo
    resultado.historiales.tiempos = registroTiempos;
else
    resultado.historiales.tiempos = struct();
end

if isfield(registro,'caminoGlobalFinal')
    resultado.historiales.caminoGlobalFinal = ...
        registro.caminoGlobalFinal;
end

%% Fila plana para acumular las repeticiones
fila = crear_fila( ...
    datos, ...
    escenario, ...
    exito, ...
    metaAlcanzada, ...
    colisionOcurrida, ...
    motivoTerminacion, ...
    tiempoHastaMeta, ...
    longitudTotal, ...
    suavidadRMS, ...
    distanciaMinima, ...
    detalleSeguridad, ...
    resumenTiempos, ...
    resumenControl, ...
    distanciaFinalMeta);
end

%% ========================================================================
% RESUMEN DEL CONTROL
% ========================================================================

function [resumen,detalle] = resumir_control( ...
    controles,Ts,robot)
%RESUMIR_CONTROL Calcula esfuerzo y variación del control aplicado.

numeroControles = size(controles,1);

vEscala = max(abs([ ...
    robot.limites.vMin, ...
    robot.limites.vMax]));

wEscala = max(abs([ ...
    robot.limites.wMin, ...
    robot.limites.wMax]));

if vEscala <= 0 || wEscala <= 0
    error('resultados:EscalaControlNoValida', ...
        'Los límites del robot no permiten normalizar los controles.');
end

if numeroControles == 0
    controlesNormalizados = zeros(0,2);
    esfuerzoNormalizado = 0;
    variacionNormalizada = 0;
    velocidadLinealMedia = NaN;
    velocidadLinealRMS = NaN;
    velocidadAngularMediaAbsoluta = NaN;
    velocidadAngularRMS = NaN;
    porcentajePasosParado = NaN;
else
    controlesNormalizados = [
        controles(:,1)/vEscala, ...
        controles(:,2)/wEscala
    ];

    esfuerzoPorPaso = ...
        sum(controlesNormalizados.^2,2);

    esfuerzoNormalizado = ...
        Ts*sum(esfuerzoPorPaso);

    if numeroControles >= 2
        variaciones = diff(controlesNormalizados,1,1);
        variacionNormalizada = ...
            sum(hypot(variaciones(:,1),variaciones(:,2)));
    else
        variacionNormalizada = 0;
    end

    velocidadLinealMedia = mean(controles(:,1));
    velocidadLinealRMS = ...
        sqrt(mean(controles(:,1).^2));

    velocidadAngularMediaAbsoluta = ...
        mean(abs(controles(:,2)));

    velocidadAngularRMS = ...
        sqrt(mean(controles(:,2).^2));

    toleranciaParada = 1e-10*max(1,vEscala);

    porcentajePasosParado = ...
        100*nnz(abs(controles(:,1)) <= toleranciaParada) / ...
        numeroControles;
end

resumen = struct();
resumen.esfuerzoNormalizado = esfuerzoNormalizado;
resumen.variacionNormalizada = variacionNormalizada;
resumen.velocidadLinealMedia = velocidadLinealMedia;
resumen.velocidadLinealRMS = velocidadLinealRMS;
resumen.velocidadAngularMediaAbsoluta = ...
    velocidadAngularMediaAbsoluta;
resumen.velocidadAngularRMS = velocidadAngularRMS;
resumen.porcentajePasosParado = porcentajePasosParado;
resumen.numeroControles = numeroControles;
resumen.unidadEsfuerzo = "s";
resumen.criterioEsfuerzo = ...
    "menor_valor_menor_esfuerzo_relativo";

detalle = struct();
detalle.controlesNormalizados = controlesNormalizados;
detalle.vEscala = vEscala;
detalle.wEscala = wEscala;
detalle.formulaEsfuerzo = ...
    "Ts*sum((v/vMax)^2+(w/wMax)^2)";
detalle.formulaVariacion = ...
    "sum(norm(diff([v/vMax,w/wMax])))";
end

%% ========================================================================
% TERMINACIÓN Y TABLA
% ========================================================================

function motivo = determinar_terminacion( ...
    metaAlcanzada,colisionOcurrida,pasosEjecutados,cfg)
%DETERMINAR_TERMINACION Etiqueta el final de la ejecución.

if metaAlcanzada && colisionOcurrida
    motivo = "meta_con_colision";
elseif metaAlcanzada
    motivo = "meta";
elseif pasosEjecutados >= cfg.terminacion.maxPasos && ...
        colisionOcurrida
    motivo = "tiempo_agotado_con_colisiones";
elseif pasosEjecutados >= cfg.terminacion.maxPasos
    motivo = "tiempo_agotado";
elseif colisionOcurrida
    % La colision queda como antecedente, pero no se presenta como causa
    % terminal cuando cfg.terminacion.detenerEnColision es false.
    motivo = "interrumpida_con_colision";
else
    motivo = "interrumpida";
end
end

function fila = crear_fila( ...
    datos,escenario,exito,metaAlcanzada,colisionOcurrida, ...
    motivoTerminacion,tiempoHastaMeta,longitudTotal,suavidadRMS, ...
    distanciaMinima,detalleSeguridad,resumenTiempos, ...
    resumenControl,distanciaFinalMeta)
%CREAR_FILA Genera una observación tabular por ejecución.

Arquitectura = datos.arquitectura;
Escenario = string(escenario.id);
Semilla = datos.semilla;

Exito = exito;
MetaAlcanzada = metaAlcanzada;
Colision = colisionOcurrida;
MotivoTerminacion = motivoTerminacion;

PasosEjecutados = datos.pasosEjecutados;
TiempoSimulado_s = resumenTiempos.tiempoSimulado;
TiempoHastaMeta_s = tiempoHastaMeta;

Longitud_m = longitudTotal;
SuavidadRMS_1_m = suavidadRMS;
DistanciaMinima_m = distanciaMinima;
DistanciaFinalMeta_m = distanciaFinalMeta;

Planificaciones = resumenTiempos.numeroPlanificaciones;
Replanificaciones = datos.numeroReplanificaciones;
FallosPlanificacion = datos.numeroFallosPlanificacion;

EpisodiosRiesgo = ...
    detalleSeguridad.numeroEpisodiosRiesgo;

EpisodiosColision = ...
    detalleSeguridad.numeroEpisodiosColision;

TiempoPlanificacionTotal_s = ...
    resumenTiempos.tiempoPlanificacionTotal;

TiempoPlanificacionMedio_s = ...
    resumenTiempos.tiempoPlanificacionMedio;

TiempoControlTotal_s = ...
    resumenTiempos.tiempoControlTotal;

TiempoControlMedio_s = ...
    resumenTiempos.tiempoControlMedio;

TiempoControlMaximo_s = ...
    resumenTiempos.tiempoControlMaximo;

TiempoCicloMedio_s = ...
    resumenTiempos.tiempoCicloMedio;

TiempoCicloMaximo_s = ...
    resumenTiempos.tiempoCicloMaximo;

TiempoComputoTotal_s = ...
    resumenTiempos.tiempoComputoTotal;

FactorTiempoReal = ...
    resumenTiempos.factorTiempoReal;

CiclosFueraPlazo = ...
    resumenTiempos.numeroCiclosFueraPlazo;

EsfuerzoControlNormalizado_s = ...
    resumenControl.esfuerzoNormalizado;

VariacionControlNormalizada = ...
    resumenControl.variacionNormalizada;

fila = table( ...
    Arquitectura, ...
    Escenario, ...
    Semilla, ...
    Exito, ...
    MetaAlcanzada, ...
    Colision, ...
    MotivoTerminacion, ...
    PasosEjecutados, ...
    TiempoSimulado_s, ...
    TiempoHastaMeta_s, ...
    Longitud_m, ...
    SuavidadRMS_1_m, ...
    DistanciaMinima_m, ...
    DistanciaFinalMeta_m, ...
    Planificaciones, ...
    Replanificaciones, ...
    FallosPlanificacion, ...
    EpisodiosRiesgo, ...
    EpisodiosColision, ...
    TiempoPlanificacionTotal_s, ...
    TiempoPlanificacionMedio_s, ...
    TiempoControlTotal_s, ...
    TiempoControlMedio_s, ...
    TiempoControlMaximo_s, ...
    TiempoCicloMedio_s, ...
    TiempoCicloMaximo_s, ...
    TiempoComputoTotal_s, ...
    FactorTiempoReal, ...
    CiclosFueraPlazo, ...
    EsfuerzoControlNormalizado_s, ...
    VariacionControlNormalizada);
end

%% ========================================================================
% VALIDACIÓN Y PREPARACIÓN
% ========================================================================

function datos = validar_y_preparar( ...
    registro,escenario,robot,cfg)
%VALIDAR_Y_PREPARAR Comprueba y recorta los datos brutos.

if ~isstruct(registro) || ~isscalar(registro)
    error('resultados:RegistroNoValido', ...
        'registro debe ser una estructura escalar.');
end

camposObligatorios = { ...
    'pasosEjecutados', ...
    'trayectoria', ...
    'historialObstaculos', ...
    'controles', ...
    'tiempos'};

for i = 1:numel(camposObligatorios)
    if ~isfield(registro,camposObligatorios{i})
        error('resultados:CampoAusente', ...
            'Falta registro.%s.',camposObligatorios{i});
    end
end

%% Configuración, escenario y robot
validar_configuracion(escenario,robot,cfg);

%% Pasos
pasos = registro.pasosEjecutados;

if ~isnumeric(pasos) || ~isscalar(pasos) || ...
        ~isreal(pasos) || ~isfinite(pasos) || ...
        pasos < 0 || pasos ~= floor(pasos)
    error('resultados:PasosNoValidos', ...
        'registro.pasosEjecutados debe ser un entero no negativo.');
end

pasos = double(pasos);
numeroEstados = pasos+1;

%% Trayectoria
trayectoria = registro.trayectoria;

if ~isnumeric(trayectoria) || ~isreal(trayectoria) || ...
        ~ismatrix(trayectoria) || size(trayectoria,2) < 2 || ...
        size(trayectoria,1) < numeroEstados
    error('resultados:TrayectoriaNoValida', ...
        ['registro.trayectoria debe contener al menos ' ...
         'pasosEjecutados+1 filas y dos columnas [x y].']);
end

trayectoria = double(trayectoria(1:numeroEstados,:));

if any(~isfinite(trayectoria(:)))
    error('resultados:TrayectoriaNoFinita', ...
        'La parte utilizada de la trayectoria contiene NaN o Inf.');
end

%% Historial dinámico
historial = registro.historialObstaculos;
numeroDinamicos = numel(escenario.obstaculosDinamicos);

if numeroDinamicos == 0
    if isempty(historial)
        historial = zeros(numeroEstados,0,2);
    elseif ~isnumeric(historial) || ...
            size(historial,1) < numeroEstados || ...
            size(historial,2) ~= 0
        error('resultados:HistorialObstaculosNoValido', ...
            'El escenario no contiene obstáculos dinámicos.');
    else
        historial = historial(1:numeroEstados,:,:);
    end
else
    if ~isnumeric(historial) || ~isreal(historial) || ...
            size(historial,1) < numeroEstados || ...
            size(historial,2) ~= numeroDinamicos || ...
            size(historial,3) ~= 2
        error('resultados:HistorialObstaculosNoValido', ...
            ['registro.historialObstaculos debe tener dimensiones ' ...
             '(pasosEjecutados+1) x numeroDinamicos x 2.']);
    end

    historial = double(historial(1:numeroEstados,:,:));

    if any(~isfinite(historial(:)))
        error('resultados:HistorialObstaculosNoFinito', ...
            'El historial utilizado de obstáculos contiene NaN o Inf.');
    end
end

%% Controles aplicados
controles = registro.controles;

if pasos == 0
    if isempty(controles)
        controles = zeros(0,2);
    elseif ~isnumeric(controles) || size(controles,2) ~= 2
        error('resultados:ControlesNoValidos', ...
            'registro.controles debe tener formato N x 2 [v w].');
    else
        controles = zeros(0,2);
    end
else
    if ~isnumeric(controles) || ~isreal(controles) || ...
            size(controles,2) ~= 2 || ...
            size(controles,1) < pasos
        error('resultados:ControlesNoValidos', ...
            ['registro.controles debe contener al menos ' ...
             'pasosEjecutados filas con formato [v w].']);
    end

    controles = double(controles(1:pasos,:));

    if any(~isfinite(controles(:)))
        error('resultados:ControlesNoFinitos', ...
            'Los controles aplicados contienen NaN o Inf.');
    end
end

%% Tiempos
if ~isstruct(registro.tiempos) || ...
        ~isscalar(registro.tiempos)
    error('resultados:TiemposNoValidos', ...
        'registro.tiempos debe ser una estructura compatible con tiempos.m.');
end

%% Arquitectura
arquitectura = "";

if isfield(registro,'arquitectura')
    arquitectura = strtrim(string(registro.arquitectura));
end

if strlength(arquitectura) == 0 && ...
        isfield(cfg,'nombreArquitectura')
    arquitectura = strtrim(string(cfg.nombreArquitectura));
end

if ~isscalar(arquitectura) || strlength(arquitectura) == 0
    error('resultados:ArquitecturaAusente', ...
        ['Defina registro.arquitectura o ' ...
         'cfg.nombreArquitectura antes de calcular resultados.']);
end

%% Semilla
if isfield(registro,'semilla')
    semilla = registro.semilla;
else
    semilla = cfg.semilla;
end

if ~isnumeric(semilla) || ~isscalar(semilla) || ...
        ~isreal(semilla) || ~isfinite(semilla) || ...
        semilla < 0 || semilla ~= floor(semilla)
    error('resultados:SemillaNoValida', ...
        'La semilla debe ser un entero no negativo.');
end

semilla = double(semilla);

%% Contadores opcionales
numeroReplanificaciones = obtener_contador( ...
    registro,'numeroReplanificaciones');

numeroFallosPlanificacion = obtener_contador( ...
    registro,'numeroFallosPlanificacion');

%% Salida interna
datos = struct();
datos.pasosEjecutados = pasos;
datos.trayectoria = trayectoria;
datos.historialObstaculos = historial;
datos.controles = controles;
datos.registroTiempos = registro.tiempos;
datos.arquitectura = arquitectura;
datos.semilla = semilla;
datos.numeroReplanificaciones = ...
    numeroReplanificaciones;
datos.numeroFallosPlanificacion = ...
    numeroFallosPlanificacion;
end

function contador = obtener_contador(registro,nombre)
%OBTENER_CONTADOR Recupera un contador opcional o devuelve NaN.

if ~isfield(registro,nombre) || isempty(registro.(nombre))
    contador = NaN;
    return;
end

contador = registro.(nombre);

if ~isnumeric(contador) || ~isscalar(contador) || ...
        ~isreal(contador) || ~isfinite(contador) || ...
        contador < 0 || contador ~= floor(contador)
    error('resultados:ContadorNoValido', ...
        'registro.%s debe ser un entero no negativo.',nombre);
end

contador = double(contador);
end

function validar_configuracion(escenario,robot,cfg)
%VALIDAR_CONFIGURACION Comprueba las estructuras compartidas.

if ~isstruct(escenario) || ~isscalar(escenario) || ...
        ~all(isfield(escenario,{ ...
            'id','nombre','meta', ...
            'obstaculosEstaticos', ...
            'obstaculosDinamicos','limites'}))
    error('resultados:EscenarioNoValido', ...
        'escenario debe proceder de escenarios.m.');
end

if ~isnumeric(escenario.meta) || ...
        numel(escenario.meta) ~= 2 || ...
        any(~isfinite(escenario.meta(:)))
    error('resultados:MetaNoValida', ...
        'escenario.meta debe tener formato [x y].');
end

if ~isstruct(robot) || ~isscalar(robot) || ...
        ~isfield(robot,'tipo') || ...
        ~isfield(robot,'limites') || ...
        ~all(isfield(robot.limites,{ ...
            'vMin','vMax','wMin','wMax'}))
    error('resultados:RobotNoValido', ...
        'robot debe proceder de configuracion_robot.m.');
end

if ~isstruct(cfg) || ~isscalar(cfg)
    error('resultados:ConfiguracionNoValida', ...
        'cfg debe proceder de parametros_generales.m.');
end

rutas = {
    'version','';
    'semilla','';
    'sim','Ts';
    'navegacion','radioMeta';
    'terminacion','maxPasos';
    'metricas','guardarTrayectoria';
    'metricas','guardarControles';
    'metricas','guardarObstaculos';
    'metricas','guardarTiemposCiclo'
};

for i = 1:size(rutas,1)
    grupo = rutas{i,1};
    campo = rutas{i,2};

    if strlength(campo) == 0
        if ~isfield(cfg,grupo)
            error('resultados:ConfiguracionIncompleta', ...
                'Falta cfg.%s.',grupo);
        end
    else
        if ~isfield(cfg,grupo) || ...
                ~isstruct(cfg.(grupo)) || ...
                ~isfield(cfg.(grupo),campo)
            error('resultados:ConfiguracionIncompleta', ...
                'Falta cfg.%s.%s.',grupo,campo);
        end
    end
end

if ~isnumeric(cfg.sim.Ts) || ~isscalar(cfg.sim.Ts) || ...
        ~isfinite(cfg.sim.Ts) || cfg.sim.Ts <= 0
    error('resultados:TsNoValido', ...
        'cfg.sim.Ts debe ser positivo.');
end

if ~isnumeric(cfg.navegacion.radioMeta) || ...
        ~isscalar(cfg.navegacion.radioMeta) || ...
        ~isfinite(cfg.navegacion.radioMeta) || ...
        cfg.navegacion.radioMeta <= 0
    error('resultados:RadioMetaNoValido', ...
        'cfg.navegacion.radioMeta debe ser positivo.');
end
end
