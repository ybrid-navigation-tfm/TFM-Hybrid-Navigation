function [resumen, detalle] = tiempos(registro, pasosEjecutados, cfg)
% Calcula las metricas temporales de una ejecucion.
%
%   resumen = TIEMPOS(registro,pasosEjecutados,cfg)
%
%   [resumen,detalle] = TIEMPOS(registro,pasosEjecutados,cfg)
%
%   Resume los tiempos computacionales registrados durante una simulacion
%   y los diferencia del tiempo fisico simulado.
%
%   La funcion separa:
%
%       - tiempo de planificacion global;
%       - tiempo del controlador local APF o MPC;
%       - tiempo computacional de cada ciclo completo;
%       - tiempo total de computo;
%       - tiempo simulado de navegacion;
%       - tiempo real de reloj, si se proporciona.
%
%   Entrada registro:
%       Estructura con los campos obligatorios:
%
%       registro.planificacion
%           Vector de tiempos de planificacion por paso [s].
%           Puede contener:
%
%               - un valor positivo cuando se llamo al planificador;
%               - 0 o NaN cuando no se llamo.
%
%       registro.control
%           Vector de tiempos del controlador por paso [s].
%           Puede contener 0 o NaN cuando el controlador no se ejecuto.
%
%       registro.ciclo
%           Vector de tiempos computacionales del ciclo completo [s].
%           Debe medirse antes de drawnow, pause y cualquier animacion.
%
%   Campos opcionales:
%
%       registro.planificacionEjecutada
%           Mascara logica que identifica las llamadas al planificador.
%
%       registro.controlEjecutado
%           Mascara logica que identifica las llamadas al controlador.
%
%       registro.inicializacion
%           Tiempo previo al bucle principal [s], por ejemplo la
%           construccion inicial de una roadmap PRM. Valor por defecto: 0.
%
%       registro.visualizacion
%           Tiempo de visualizacion por paso [s]. Se informa por separado
%           y no se utiliza para comparar APF y MPC.
%
%       registro.tiempoRelojTotal
%           Tiempo real transcurrido medido alrededor de toda la ejecucion,
%           potencialmente incluyendo graficos y pausas [s].
%
%       registro.incluyeVisualizacion
%           true si registro.ciclo incluye operaciones graficas.
%
%       registro.incluyePausas
%           true si registro.ciclo incluye pause().
%
%       registro.metaAlcanzada
%           Indicador logico opcional. Si es true, el tiempo simulado
%           acumulado se informa tambien como tiempo hasta la meta.
%
%   Otras entradas:
%       pasosEjecutados
%           Numero entero de actualizaciones del modelo realizadas.
%           Permite recortar historiales preasignados.
%
%       cfg
%           Estructura obtenida mediante parametros_generales.m. Se utiliza
%           principalmente:
%
%               cfg.sim.Ts
%
%   Salida resumen:
%       Estructura plana adecuada para resultados.m:
%
%           .tiempoSimulado
%           .tiempoHastaMetaSimulado
%           .tiempoInicializacion
%           .tiempoPlanificacionTotal
%           .tiempoPlanificacionMedio
%           .tiempoPlanificacionMaximo
%           .numeroPlanificaciones
%           .tiempoControlTotal
%           .tiempoControlMedio
%           .tiempoControlMaximo
%           .numeroControles
%           .tiempoCicloMedio
%           .tiempoCicloMaximo
%           .tiempoCicloP95
%           .tiempoComputoCiclos
%           .tiempoComputoTotal
%           .tiempoRelojTotal
%           .numeroCiclosFueraPlazo
%           .porcentajeCiclosFueraPlazo
%           .factorTiempoReal
%           .cumpleTiempoRealMedio
%           .cumpleTiempoRealMaximo
%
%   Salida detalle:
%       Contiene los historiales recortados, las mascaras utilizadas y un
%       resumen estadistico de cada fase con:
%
%           numero, total, media, desviacionEstandar,
%           mediana, minimo, maximo y percentil95.
%
%   Definiciones:
%
%       Tiempo simulado:
%
%           t_sim = pasosEjecutados * cfg.sim.Ts
%
%       Tiempo medio de control:
%
%           media de las llamadas efectivamente realizadas al controlador
%
%       Tiempo maximo de ciclo:
%
%           max(registro.ciclo)
%
%       Factor de tiempo real:
%
%           factor = tiempoSimulado / tiempoComputoTotal
%
%       Un factor mayor que 1 indica que el nucleo computacional se ejecuto
%       mas deprisa que el tiempo fisico simulado.
%
%   Ejemplo de registro dentro del futuro bucle principal:
%
%       maxPasos = cfg.terminacion.maxPasos;
%
%       tPlan = nan(maxPasos,1);
%       tControl = nan(maxPasos,1);
%       tCiclo = nan(maxPasos,1);
%
%       for k = 1:maxPasos
%           relojCiclo = tic;
%
%           if debeReplanificar
%               relojPlan = tic;
%               camino = rrt_star(...);
%               tPlan(k) = toc(relojPlan);
%           end
%
%           if hayCamino
%               relojControl = tic;
%               control = mpc(...);
%               tControl(k) = toc(relojControl);
%           end
%
%           % Actualizacion del modelo, colisiones y metricas.
%           % ...
%
%           % Detener antes de drawnow y pause.
%           tCiclo(k) = toc(relojCiclo);
%
%           % Visualizacion fuera del cronometro computacional.
%           % drawnow;
%           % pause(cfg.visual.tPausa);
%       end
%
%       registro = struct();
%       registro.planificacion = tPlan;
%       registro.control = tControl;
%       registro.ciclo = tCiclo;
%
%       [resumenTiempo,detalleTiempo] = tiempos( ...
%           registro,pasosEjecutados,cfg);
%
%   Esta funcion no inicia ni detiene cronometros. Resume mediciones
%   obtenidas mediante tic y toc en el bucle de simulacion.

%% Validacion y normalizacion
[registro, pasosEjecutados, Ts, modo] = ...
    validar_entradas(registro,pasosEjecutados,cfg);

%% Historiales recortados
tiempoPlanificacion = ...
    registro.planificacion(1:pasosEjecutados);

tiempoControl = ...
    registro.control(1:pasosEjecutados);

tiempoCiclo = ...
    registro.ciclo(1:pasosEjecutados);

%% Mascaras de ejecucion de cada fase
mascaraPlanificacion = obtener_mascara( ...
    registro, ...
    'planificacionEjecutada', ...
    tiempoPlanificacion, ...
    pasosEjecutados);

mascaraControl = obtener_mascara( ...
    registro, ...
    'controlEjecutado', ...
    tiempoControl, ...
    pasosEjecutados);

valoresPlanificacion = ...
    tiempoPlanificacion(mascaraPlanificacion);

valoresControl = ...
    tiempoControl(mascaraControl);

%% Visualizacion opcional
if isfield(registro,'visualizacion') && ...
        ~isempty(registro.visualizacion)

    tiempoVisualizacion = ...
        registro.visualizacion(1:pasosEjecutados);

    mascaraVisualizacion = isfinite(tiempoVisualizacion);
    valoresVisualizacion = ...
        tiempoVisualizacion(mascaraVisualizacion);
else
    tiempoVisualizacion = nan(pasosEjecutados,1);
    mascaraVisualizacion = false(pasosEjecutados,1);
    valoresVisualizacion = zeros(0,1);
end

%% Resumen estadistico de las fases
estadisticaPlanificacion = ...
    resumir_vector(valoresPlanificacion);

estadisticaControl = ...
    resumir_vector(valoresControl);

estadisticaCiclo = ...
    resumir_vector(tiempoCiclo);

estadisticaVisualizacion = ...
    resumir_vector(valoresVisualizacion);

%% Tiempos simulados y computacionales
tiempoSimulado = pasosEjecutados*Ts;

tiempoInicializacion = ...
    obtener_escalar_opcional(registro,'inicializacion',0);

tiempoComputoCiclos = sum(tiempoCiclo);

tiempoComputoTotal = ...
    tiempoInicializacion + tiempoComputoCiclos;

tiempoRelojTotal = ...
    obtener_escalar_opcional( ...
        registro,'tiempoRelojTotal',NaN);

%% Tiempo hasta la meta
metaAlcanzada = obtener_logico_opcional( ...
    registro,'metaAlcanzada',NaN);

if isequal(metaAlcanzada,true)
    tiempoHastaMetaSimulado = tiempoSimulado;
else
    tiempoHastaMetaSimulado = NaN;
end

%% Cumplimiento del periodo de muestreo
escala = max(1,max(abs([tiempoCiclo(:); Ts])));
toleranciaTiempo = 1e-12*escala;

cicloFueraPlazo = ...
    tiempoCiclo > Ts+toleranciaTiempo;

numeroCiclosFueraPlazo = nnz(cicloFueraPlazo);

if pasosEjecutados > 0
    porcentajeCiclosFueraPlazo = ...
        100*numeroCiclosFueraPlazo/pasosEjecutados;
else
    porcentajeCiclosFueraPlazo = NaN;
end

cumpleTiempoRealMedio = ...
    pasosEjecutados > 0 && ...
    estadisticaCiclo.media <= Ts+toleranciaTiempo;

cumpleTiempoRealMaximo = ...
    pasosEjecutados > 0 && ...
    estadisticaCiclo.maximo <= Ts+toleranciaTiempo;

%% Factor de tiempo real
factorTiempoRealCiclos = cociente_tiempo( ...
    tiempoSimulado,tiempoComputoCiclos);

factorTiempoReal = cociente_tiempo( ...
    tiempoSimulado,tiempoComputoTotal);

factorTiempoRealReloj = cociente_tiempo( ...
    tiempoSimulado,tiempoRelojTotal);

%% Distribucion del tiempo computacional
tiempoPlanificacionTotal = ...
    estadisticaPlanificacion.total;

tiempoControlTotal = ...
    estadisticaControl.total;

tiempoRestoCiclosSinLimitar = ...
    tiempoComputoCiclos - ...
    tiempoPlanificacionTotal - ...
    tiempoControlTotal;

solapamientoCronometros = ...
    tiempoRestoCiclosSinLimitar < -toleranciaTiempo;

tiempoRestoCiclos = ...
    max(0,tiempoRestoCiclosSinLimitar);

if tiempoComputoCiclos > 0
    porcentajePlanificacion = ...
        100*tiempoPlanificacionTotal/tiempoComputoCiclos;

    porcentajeControl = ...
        100*tiempoControlTotal/tiempoComputoCiclos;

    porcentajeResto = ...
        100*tiempoRestoCiclos/tiempoComputoCiclos;
else
    porcentajePlanificacion = NaN;
    porcentajeControl = NaN;
    porcentajeResto = NaN;
end

%% Comparabilidad de la medicion
incluyeVisualizacion = obtener_logico_opcional( ...
    registro,'incluyeVisualizacion',false);

incluyePausas = obtener_logico_opcional( ...
    registro,'incluyePausas',false);

medicionComputacionalComparable = ...
    ~incluyeVisualizacion && ~incluyePausas;

if medicionComputacionalComparable
    advertenciaComparabilidad = "";
else
    advertenciaComparabilidad = [ ...
        "El tiempo de ciclo incluye visualizacion o pausas y no debe " ...
        "usarse para comparar el coste computacional de APF y MPC."];
end

%% Salida plana
resumen = struct();

resumen.tiempoSimulado = tiempoSimulado;
resumen.tiempoHastaMetaSimulado = ...
    tiempoHastaMetaSimulado;

resumen.tiempoInicializacion = ...
    tiempoInicializacion;

resumen.tiempoPlanificacionTotal = ...
    tiempoPlanificacionTotal;

resumen.tiempoPlanificacionMedio = ...
    estadisticaPlanificacion.media;

resumen.tiempoPlanificacionMaximo = ...
    estadisticaPlanificacion.maximo;

resumen.numeroPlanificaciones = ...
    estadisticaPlanificacion.numero;

resumen.tiempoControlTotal = ...
    tiempoControlTotal;

resumen.tiempoControlMedio = ...
    estadisticaControl.media;

resumen.tiempoControlMaximo = ...
    estadisticaControl.maximo;

resumen.numeroControles = ...
    estadisticaControl.numero;

resumen.tiempoCicloMedio = ...
    estadisticaCiclo.media;

resumen.tiempoCicloMaximo = ...
    estadisticaCiclo.maximo;

resumen.tiempoCicloP95 = ...
    estadisticaCiclo.percentil95;

resumen.tiempoComputoCiclos = ...
    tiempoComputoCiclos;

resumen.tiempoComputoTotal = ...
    tiempoComputoTotal;

resumen.tiempoRelojTotal = ...
    tiempoRelojTotal;

resumen.numeroCiclosFueraPlazo = ...
    numeroCiclosFueraPlazo;

resumen.porcentajeCiclosFueraPlazo = ...
    porcentajeCiclosFueraPlazo;

resumen.factorTiempoReal = ...
    factorTiempoReal;

resumen.factorTiempoRealCiclos = ...
    factorTiempoRealCiclos;

resumen.factorTiempoRealReloj = ...
    factorTiempoRealReloj;

resumen.cumpleTiempoRealMedio = ...
    cumpleTiempoRealMedio;

resumen.cumpleTiempoRealMaximo = ...
    cumpleTiempoRealMaximo;

%% Salida detallada
detalle = struct();

detalle.calculable = pasosEjecutados > 0;

if pasosEjecutados > 0
    detalle.motivo = "correcto";
else
    detalle.motivo = "sin_pasos_ejecutados";
end

detalle.modo = modo;
detalle.periodoMuestreo = Ts;
detalle.frecuenciaControlNominal = 1/Ts;
detalle.pasosEjecutados = pasosEjecutados;

detalle.tiempoSimulado = tiempoSimulado;
detalle.tiempoHastaMetaSimulado = ...
    tiempoHastaMetaSimulado;

detalle.metaAlcanzada = metaAlcanzada;

detalle.historialPlanificacion = ...
    tiempoPlanificacion;

detalle.historialControl = ...
    tiempoControl;

detalle.historialCiclo = ...
    tiempoCiclo;

detalle.historialVisualizacion = ...
    tiempoVisualizacion;

detalle.mascaraPlanificacion = ...
    mascaraPlanificacion;

detalle.mascaraControl = ...
    mascaraControl;

detalle.mascaraVisualizacion = ...
    mascaraVisualizacion;

detalle.estadisticaPlanificacion = ...
    estadisticaPlanificacion;

detalle.estadisticaControl = ...
    estadisticaControl;

detalle.estadisticaCiclo = ...
    estadisticaCiclo;

detalle.estadisticaVisualizacion = ...
    estadisticaVisualizacion;

detalle.tiempoInicializacion = ...
    tiempoInicializacion;

detalle.tiempoComputoCiclos = ...
    tiempoComputoCiclos;

detalle.tiempoComputoTotal = ...
    tiempoComputoTotal;

detalle.tiempoRelojTotal = ...
    tiempoRelojTotal;

detalle.tiempoRestoCiclos = ...
    tiempoRestoCiclos;

detalle.tiempoRestoCiclosSinLimitar = ...
    tiempoRestoCiclosSinLimitar;

detalle.porcentajePlanificacion = ...
    porcentajePlanificacion;

detalle.porcentajeControl = ...
    porcentajeControl;

detalle.porcentajeResto = ...
    porcentajeResto;

detalle.solapamientoCronometrosDetectado = ...
    solapamientoCronometros;

detalle.presupuestoCiclo = Ts;
detalle.toleranciaTiempo = toleranciaTiempo;

detalle.cicloFueraPlazo = cicloFueraPlazo;

detalle.numeroCiclosFueraPlazo = ...
    numeroCiclosFueraPlazo;

detalle.porcentajeCiclosFueraPlazo = ...
    porcentajeCiclosFueraPlazo;

detalle.cumpleTiempoRealMedio = ...
    cumpleTiempoRealMedio;

detalle.cumpleTiempoRealMaximo = ...
    cumpleTiempoRealMaximo;

detalle.margenCicloMedio = ...
    Ts-estadisticaCiclo.media;

detalle.margenCicloMaximo = ...
    Ts-estadisticaCiclo.maximo;

detalle.ocupacionMediaCiclo = ...
    100*estadisticaCiclo.media/Ts;

detalle.ocupacionMaximaCiclo = ...
    100*estadisticaCiclo.maximo/Ts;

detalle.factorTiempoReal = factorTiempoReal;
detalle.factorTiempoRealCiclos = ...
    factorTiempoRealCiclos;

detalle.factorTiempoRealReloj = ...
    factorTiempoRealReloj;

detalle.incluyeVisualizacion = ...
    incluyeVisualizacion;

detalle.incluyePausas = ...
    incluyePausas;

detalle.medicionComputacionalComparable = ...
    medicionComputacionalComparable;

detalle.advertenciaComparabilidad = ...
    advertenciaComparabilidad;

detalle.tiemposSimulados = ...
    (1:pasosEjecutados)'*Ts;

detalle.unidad = "s";
detalle.criterioCoste = ...
    "menor_tiempo_computacional_mejor";
end

%% ========================================================================
% FUNCIONES LOCALES
% ========================================================================

function estadistica = resumir_vector(valores)
%RESUMIR_VECTOR Calcula estadisticos sin toolboxes adicionales.

valores = double(valores(:));
valores = valores(isfinite(valores));

estadistica = struct();
estadistica.numero = numel(valores);

if isempty(valores)
    estadistica.total = 0;
    estadistica.media = NaN;
    estadistica.desviacionEstandar = NaN;
    estadistica.mediana = NaN;
    estadistica.minimo = NaN;
    estadistica.maximo = NaN;
    estadistica.percentil95 = NaN;
    return;
end

estadistica.total = sum(valores);
estadistica.media = mean(valores);
estadistica.desviacionEstandar = std(valores,0);
estadistica.mediana = median(valores);
estadistica.minimo = min(valores);
estadistica.maximo = max(valores);
estadistica.percentil95 = percentil_lineal(valores,0.95);
end

function valor = percentil_lineal(datos,probabilidad)
%PERCENTIL_LINEAL Percentil mediante interpolacion lineal.

datos = sort(double(datos(:)));
numeroDatos = numel(datos);

if numeroDatos == 0
    valor = NaN;
    return;
end

if numeroDatos == 1
    valor = datos(1);
    return;
end

posicion = 1+(numeroDatos-1)*probabilidad;
inferior = floor(posicion);
superior = ceil(posicion);

if inferior == superior
    valor = datos(inferior);
else
    fraccion = posicion-inferior;
    valor = datos(inferior) + ...
        fraccion*(datos(superior)-datos(inferior));
end
end

function mascara = obtener_mascara( ...
    registro,nombreCampo,tiempos,numeroPasos)
%OBTENER_MASCARA Identifica las llamadas efectivamente realizadas.

if isfield(registro,nombreCampo) && ...
        ~isempty(registro.(nombreCampo))

    mascara = validar_mascara( ...
        registro.(nombreCampo), ...
        nombreCampo, ...
        numeroPasos);
else
    % Compatible con historiales que usan NaN o cero cuando no hay llamada.
    mascara = isfinite(tiempos) & tiempos > 0;
end

if any(mascara & ~isfinite(tiempos))
    error('tiempos:TiempoAusenteEnLlamada', ...
        ['La mascara %s identifica una llamada cuyo tiempo ' ...
         'es NaN o Inf.'],nombreCampo);
end
end

function mascara = validar_mascara( ...
    mascara,nombreCampo,numeroPasos)
%VALIDAR_MASCARA Normaliza una mascara logica o binaria.

if islogical(mascara)
    mascara = mascara(:);
elseif isnumeric(mascara) && isreal(mascara) && ...
        all(isfinite(mascara(:))) && ...
        all(mascara(:) == 0 | mascara(:) == 1)
    mascara = logical(mascara(:));
else
    error('tiempos:MascaraNoValida', ...
        '%s debe ser un vector logico o binario.',nombreCampo);
end

if numel(mascara) < numeroPasos
    error('tiempos:MascaraCorta', ...
        '%s contiene menos elementos que pasosEjecutados.',nombreCampo);
end

mascara = mascara(1:numeroPasos);
end

function valor = obtener_escalar_opcional( ...
    registro,nombreCampo,valorDefecto)
%OBTENER_ESCALAR_OPCIONAL Recupera un tiempo escalar no negativo.

if ~isfield(registro,nombreCampo) || ...
        isempty(registro.(nombreCampo))
    valor = valorDefecto;
    return;
end

valor = registro.(nombreCampo);

if ~isnumeric(valor) || ~isscalar(valor) || ...
        ~isreal(valor) || ...
        (~isnan(valor) && (~isfinite(valor) || valor < 0))
    error('tiempos:EscalarNoValido', ...
        '%s debe ser un escalar no negativo o NaN.',nombreCampo);
end

valor = double(valor);
end

function valor = obtener_logico_opcional( ...
    registro,nombreCampo,valorDefecto)
%OBTENER_LOGICO_OPCIONAL Recupera un indicador escalar.

if ~isfield(registro,nombreCampo) || ...
        isempty(registro.(nombreCampo))
    valor = valorDefecto;
    return;
end

dato = registro.(nombreCampo);

if islogical(dato) && isscalar(dato)
    valor = dato;
elseif isnumeric(dato) && isscalar(dato) && ...
        isreal(dato) && isfinite(dato) && ...
        (dato == 0 || dato == 1)
    valor = logical(dato);
else
    error('tiempos:IndicadorNoValido', ...
        '%s debe ser un escalar logico o binario.',nombreCampo);
end
end

function cociente = cociente_tiempo(numerador,denominador)
%COCIENTE_TIEMPO Calcula un factor evitando divisiones ambiguas.

if isnan(denominador)
    cociente = NaN;
elseif denominador > 0
    cociente = numerador/denominador;
elseif numerador > 0
    cociente = Inf;
else
    cociente = NaN;
end
end

function [registro,pasos,Ts,modo] = ...
    validar_entradas(registro,pasos,cfg)
%VALIDAR_ENTRADAS Comprueba historiales y configuracion.

%% Registro
if ~isstruct(registro) || ~isscalar(registro)
    error('tiempos:RegistroNoValido', ...
        'registro debe ser una estructura escalar.');
end

camposObligatorios = { ...
    'planificacion', ...
    'control', ...
    'ciclo'};

for i = 1:numel(camposObligatorios)
    if ~isfield(registro,camposObligatorios{i})
        error('tiempos:CampoAusente', ...
            'Falta registro.%s.',camposObligatorios{i});
    end
end

%% Numero de pasos
if ~isnumeric(pasos) || ~isscalar(pasos) || ...
        ~isreal(pasos) || ~isfinite(pasos) || ...
        pasos < 0 || pasos ~= floor(pasos)
    error('tiempos:PasosNoValidos', ...
        'pasosEjecutados debe ser un entero no negativo.');
end

pasos = double(pasos);

%% Historiales obligatorios
registro.planificacion = validar_vector_tiempos( ...
    registro.planificacion, ...
    'registro.planificacion', ...
    pasos, ...
    true);

registro.control = validar_vector_tiempos( ...
    registro.control, ...
    'registro.control', ...
    pasos, ...
    true);

registro.ciclo = validar_vector_tiempos( ...
    registro.ciclo, ...
    'registro.ciclo', ...
    pasos, ...
    false);

%% Visualizacion opcional
if isfield(registro,'visualizacion') && ...
        ~isempty(registro.visualizacion)

    registro.visualizacion = validar_vector_tiempos( ...
        registro.visualizacion, ...
        'registro.visualizacion', ...
        pasos, ...
        true);
end

%% Configuracion
if ~isstruct(cfg) || ~isscalar(cfg) || ...
        ~isfield(cfg,'sim') || ...
        ~isstruct(cfg.sim) || ...
        ~isfield(cfg.sim,'Ts')
    error('tiempos:ConfiguracionNoValida', ...
        'cfg debe proceder de parametros_generales.m.');
end

Ts = cfg.sim.Ts;

if ~isnumeric(Ts) || ~isscalar(Ts) || ...
        ~isreal(Ts) || ~isfinite(Ts) || Ts <= 0
    error('tiempos:TsNoValido', ...
        'cfg.sim.Ts debe ser un escalar positivo.');
end

Ts = double(Ts);

if isfield(cfg,'modo')
    modo = string(cfg.modo);
else
    modo = "";
end
end

function vector = validar_vector_tiempos( ...
    vector,nombre,numeroPasos,permitirNaN)
%VALIDAR_VECTOR_TIEMPOS Comprueba una serie temporal no negativa.

if ~isnumeric(vector) || ~isreal(vector) || ...
        (~isempty(vector) && ~isvector(vector))
    error('tiempos:VectorNoValido', ...
        '%s debe ser un vector numerico real.',nombre);
end

vector = double(vector(:));

if numel(vector) < numeroPasos
    error('tiempos:HistorialCorto', ...
        '%s contiene menos elementos que pasosEjecutados.',nombre);
end

vectorEvaluado = vector(1:numeroPasos);

if permitirNaN
    if any(isinf(vectorEvaluado))
        error('tiempos:ValoresInfinitos', ...
            '%s contiene valores Inf.',nombre);
    end
else
    if any(~isfinite(vectorEvaluado))
        error('tiempos:ValoresNoFinitos', ...
            '%s debe contener un tiempo finito por cada paso.',nombre);
    end
end

valoresFinitos = vectorEvaluado(isfinite(vectorEvaluado));

if any(valoresFinitos < 0)
    error('tiempos:TiempoNegativo', ...
        '%s contiene tiempos negativos.',nombre);
end
end
