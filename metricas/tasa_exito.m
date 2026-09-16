function [tasaPorcentaje, exitoPorEjecucion, detalle] = tasa_exito( ...
    metaAlcanzada, colisionOcurrida, pasosEjecutados, cfg)
% Calcula el exito individual y la tasa de exito experimental.
%
%   tasaPorcentaje = TASA_EXITO( ...
%       metaAlcanzada,colisionOcurrida,pasosEjecutados,cfg)
%
%   [tasaPorcentaje,exitoPorEjecucion,detalle] = TASA_EXITO( ...
%       metaAlcanzada,colisionOcurrida,pasosEjecutados,cfg)
%
%   Evalua cada ejecucion mediante los criterios comunes definidos en:
%
%       cfg.evaluacion.exito
%
%   y calcula la tasa:
%
%       tasaExito = 100 * numeroExitos / numeroEjecuciones
%
%   Con la configuracion actual, una ejecucion se considera exitosa si:
%
%       1. el robot alcanza la meta;
%       2. no se produce ninguna colision;
%       3. la meta se alcanza dentro del tiempo maximo permitido.
%
%   Entradas:
%       metaAlcanzada
%           Escalar o vector logico. El elemento i vale true cuando el
%           robot alcanzo la region de meta en la ejecucion i.
%
%       colisionOcurrida
%           Escalar o vector logico. El elemento i vale true cuando se
%           detecto al menos una colision durante la ejecucion i.
%
%       pasosEjecutados
%           Escalar o vector de enteros no negativos. Indica el numero de
%           actualizaciones del modelo realizadas en cada ejecucion.
%
%       cfg
%           Estructura obtenida mediante parametros_generales.m. Se usan:
%
%               cfg.sim.Ts
%               cfg.terminacion.maxPasos
%               cfg.terminacion.tMaxSimulado
%               cfg.evaluacion.exito.requiereMeta
%               cfg.evaluacion.exito.requiereSinColision
%               cfg.evaluacion.exito.requiereDentroDelTiempo
%
%   Salidas:
%       tasaPorcentaje
%           Porcentaje de ejecuciones exitosas en el intervalo [0,100].
%           Si no se proporcionan ejecuciones, devuelve NaN.
%
%       exitoPorEjecucion
%           Vector logico N x 1 con el resultado de cada repeticion.
%
%       detalle
%           Estructura con:
%
%               .numeroEjecuciones
%               .numeroExitos
%               .numeroFallos
%               .tasaFraccion
%               .tasaPorcentaje
%               .intervaloConfianza95Fraccion
%               .intervaloConfianza95Porcentaje
%               .metaAlcanzada
%               .colisionOcurrida
%               .pasosEjecutados
%               .tiempoSimulado
%               .dentroDelTiempo
%               .exitoPorEjecucion
%               .motivoPorEjecucion
%               .numeroFallosMeta
%               .numeroFallosColision
%               .numeroFallosTiempo
%
%   El intervalo de confianza del 95 %% para la proporcion se calcula
%   mediante el intervalo de Wilson, sin requerir toolboxes estadisticos.
%
%   IMPORTANTE:
%   Los contadores de fallo por meta, colision y tiempo pueden solaparse.
%   Por ejemplo, una ejecucion puede no alcanzar la meta y ademas terminar
%   por colision. No deben sumarse para obtener numeroFallos.
%
%   Ejemplo de una unica ejecucion:
%
%       cfg = parametros_generales("batch");
%
%       [tasa,exito,info] = tasa_exito( ...
%           true,false,210,cfg);
%
%       % tasa = 100
%       % exito = true
%
%   Ejemplo de un lote de cinco repeticiones:
%
%       meta =      [true; true; false; true; false];
%       colision =  [false; true; false; false; false];
%       pasos =     [180; 90; 350; 220; 350];
%
%       [tasa,exitos,info] = tasa_exito( ...
%           meta,colision,pasos,cfg);
%
%   Esta funcion no determina geometricamente la llegada a la meta ni la
%   colision. El futuro bucle principal debe registrar esos indicadores
%   usando el criterio de meta y detectar_colisiones.m.

%% Validacion y normalizacion
[metaAlcanzada, colisionOcurrida, pasosEjecutados, cfg] = ...
    validar_entradas( ...
        metaAlcanzada,colisionOcurrida,pasosEjecutados,cfg);

numeroEjecuciones = numel(metaAlcanzada);

%% Caso sin observaciones
if numeroEjecuciones == 0
    tasaPorcentaje = NaN;
    exitoPorEjecucion = false(0,1);
    detalle = detalle_vacio(cfg);
    return;
end

%% Tiempo simulado de cada ejecucion
tiempoSimulado = pasosEjecutados*cfg.sim.Ts;

escalaTiempo = max(1,max(abs([ ...
    tiempoSimulado(:); ...
    cfg.terminacion.tMaxSimulado])));

toleranciaTiempo = 1e-12*escalaTiempo;

dentroDelTiempo = ...
    pasosEjecutados <= cfg.terminacion.maxPasos & ...
    tiempoSimulado <= ...
        cfg.terminacion.tMaxSimulado+toleranciaTiempo;

%% Criterios configurables de exito
requiereMeta = logical( ...
    cfg.evaluacion.exito.requiereMeta);

requiereSinColision = logical( ...
    cfg.evaluacion.exito.requiereSinColision);

requiereDentroDelTiempo = logical( ...
    cfg.evaluacion.exito.requiereDentroDelTiempo);

cumpleMeta = ...
    ~requiereMeta | metaAlcanzada;

cumpleColision = ...
    ~requiereSinColision | ~colisionOcurrida;

cumpleTiempo = ...
    ~requiereDentroDelTiempo | dentroDelTiempo;

exitoPorEjecucion = ...
    cumpleMeta & cumpleColision & cumpleTiempo;

%% Tasa de exito
numeroExitos = nnz(exitoPorEjecucion);
numeroFallos = numeroEjecuciones-numeroExitos;

tasaFraccion = numeroExitos/numeroEjecuciones;
tasaPorcentaje = 100*tasaFraccion;

%% Intervalo de confianza del 95 % para una proporcion
intervalo95Fraccion = intervalo_wilson( ...
    numeroExitos,numeroEjecuciones,0.95);

intervalo95Porcentaje = 100*intervalo95Fraccion;

%% Causas de incumplimiento
falloMeta = ...
    requiereMeta & ~metaAlcanzada;

falloColision = ...
    requiereSinColision & colisionOcurrida;

falloTiempo = ...
    requiereDentroDelTiempo & ~dentroDelTiempo;

motivoPorEjecucion = construir_motivos( ...
    exitoPorEjecucion,falloMeta,falloColision,falloTiempo);

%% Salida detallada
detalle = struct();

detalle.calculable = true;
detalle.motivo = "correcto";

detalle.numeroEjecuciones = numeroEjecuciones;
detalle.numeroExitos = numeroExitos;
detalle.numeroFallos = numeroFallos;

detalle.tasaFraccion = tasaFraccion;
detalle.tasaPorcentaje = tasaPorcentaje;

detalle.intervaloConfianza95Fraccion = ...
    intervalo95Fraccion;

detalle.intervaloConfianza95Porcentaje = ...
    intervalo95Porcentaje;

detalle.metaAlcanzada = metaAlcanzada;
detalle.colisionOcurrida = colisionOcurrida;
detalle.pasosEjecutados = pasosEjecutados;
detalle.tiempoSimulado = tiempoSimulado;
detalle.dentroDelTiempo = dentroDelTiempo;

detalle.cumpleMeta = cumpleMeta;
detalle.cumpleColision = cumpleColision;
detalle.cumpleTiempo = cumpleTiempo;
detalle.exitoPorEjecucion = exitoPorEjecucion;

detalle.falloMeta = falloMeta;
detalle.falloColision = falloColision;
detalle.falloTiempo = falloTiempo;
detalle.motivoPorEjecucion = motivoPorEjecucion;

detalle.numeroFallosMeta = nnz(falloMeta);
detalle.numeroFallosColision = nnz(falloColision);
detalle.numeroFallosTiempo = nnz(falloTiempo);

detalle.requiereMeta = requiereMeta;
detalle.requiereSinColision = requiereSinColision;
detalle.requiereDentroDelTiempo = requiereDentroDelTiempo;

detalle.periodoMuestreo = cfg.sim.Ts;
detalle.maxPasos = cfg.terminacion.maxPasos;
detalle.tMaxSimulado = cfg.terminacion.tMaxSimulado;
detalle.toleranciaTiempo = toleranciaTiempo;

detalle.formula = ...
    "100 * numeroExitos / numeroEjecuciones";

detalle.unidad = "%";
detalle.criterio = "mayor_valor_mayor_tasa_exito";
end

%% ========================================================================
% FUNCIONES LOCALES
% ========================================================================

function intervalo = intervalo_wilson(exitos,nivelN,confianza)
%INTERVALO_WILSON Intervalo de Wilson para una proporcion binomial.
%
%   Para una confianza del 95 %, se utiliza z = 1.95996398454005.

if nivelN <= 0
    intervalo = [NaN NaN];
    return;
end

if abs(confianza-0.95) > 1e-12
    error('tasa_exito:ConfianzaNoSoportada', ...
        'Esta version implementa el intervalo del 95 por ciento.');
end

z = 1.95996398454005;
p = exitos/nivelN;

denominador = 1+z^2/nivelN;

centro = ...
    (p+z^2/(2*nivelN))/denominador;

semiancho = z*sqrt( ...
    p*(1-p)/nivelN + z^2/(4*nivelN^2)) / ...
    denominador;

intervalo = [
    max(0,centro-semiancho), ...
    min(1,centro+semiancho)
];
end

function motivos = construir_motivos( ...
    exitos,falloMeta,falloColision,falloTiempo)
%CONSTRUIR_MOTIVOS Describe el resultado de cada ejecucion.

numeroEjecuciones = numel(exitos);
motivos = strings(numeroEjecuciones,1);

for i = 1:numeroEjecuciones
    if exitos(i)
        motivos(i) = "exito";
        continue;
    end

    causas = strings(0,1);

    if falloMeta(i)
        causas(end+1,1) = "meta_no_alcanzada"; %#ok<AGROW>
    end

    if falloColision(i)
        causas(end+1,1) = "colision"; %#ok<AGROW>
    end

    if falloTiempo(i)
        causas(end+1,1) = "fuera_de_tiempo"; %#ok<AGROW>
    end

    if isempty(causas)
        motivos(i) = "fallo_no_clasificado";
    else
        motivos(i) = strjoin(causas,"+");
    end
end
end

function detalle = detalle_vacio(cfg)
%DETALLE_VACIO Inicializa la salida cuando no hay repeticiones.

detalle = struct();

detalle.calculable = false;
detalle.motivo = "sin_ejecuciones";

detalle.numeroEjecuciones = 0;
detalle.numeroExitos = 0;
detalle.numeroFallos = 0;

detalle.tasaFraccion = NaN;
detalle.tasaPorcentaje = NaN;
detalle.intervaloConfianza95Fraccion = [NaN NaN];
detalle.intervaloConfianza95Porcentaje = [NaN NaN];

detalle.metaAlcanzada = false(0,1);
detalle.colisionOcurrida = false(0,1);
detalle.pasosEjecutados = zeros(0,1);
detalle.tiempoSimulado = zeros(0,1);
detalle.dentroDelTiempo = false(0,1);

detalle.cumpleMeta = false(0,1);
detalle.cumpleColision = false(0,1);
detalle.cumpleTiempo = false(0,1);
detalle.exitoPorEjecucion = false(0,1);

detalle.falloMeta = false(0,1);
detalle.falloColision = false(0,1);
detalle.falloTiempo = false(0,1);
detalle.motivoPorEjecucion = strings(0,1);

detalle.numeroFallosMeta = 0;
detalle.numeroFallosColision = 0;
detalle.numeroFallosTiempo = 0;

detalle.requiereMeta = logical( ...
    cfg.evaluacion.exito.requiereMeta);

detalle.requiereSinColision = logical( ...
    cfg.evaluacion.exito.requiereSinColision);

detalle.requiereDentroDelTiempo = logical( ...
    cfg.evaluacion.exito.requiereDentroDelTiempo);

detalle.periodoMuestreo = cfg.sim.Ts;
detalle.maxPasos = cfg.terminacion.maxPasos;
detalle.tMaxSimulado = cfg.terminacion.tMaxSimulado;
detalle.toleranciaTiempo = NaN;

detalle.formula = ...
    "100 * numeroExitos / numeroEjecuciones";

detalle.unidad = "%";
detalle.criterio = "mayor_valor_mayor_tasa_exito";
end

function [meta,colision,pasos,cfg] = validar_entradas( ...
    meta,colision,pasos,cfg)
%VALIDAR_ENTRADAS Comprueba indicadores, pasos y configuracion.

meta = validar_indicador(meta,"metaAlcanzada");
colision = validar_indicador( ...
    colision,"colisionOcurrida");

if ~isnumeric(pasos) || ~isreal(pasos) || ...
        any(~isfinite(pasos(:))) || ...
        any(pasos(:) < 0) || ...
        any(pasos(:) ~= floor(pasos(:)))
    error('tasa_exito:PasosNoValidos', ...
        ['pasosEjecutados debe contener enteros no negativos ' ...
         'y finitos.']);
end

pasos = reshape(double(pasos),[],1);

if numel(meta) ~= numel(colision) || ...
        numel(meta) ~= numel(pasos)
    error('tasa_exito:DimensionesIncompatibles', ...
        ['metaAlcanzada, colisionOcurrida y pasosEjecutados ' ...
         'deben contener una observacion por ejecucion.']);
end

if ~isstruct(cfg) || ~isscalar(cfg)
    error('tasa_exito:ConfiguracionNoValida', ...
        'cfg debe proceder de parametros_generales.m.');
end

rutasNecesarias = {
    'sim','Ts';
    'terminacion','maxPasos';
    'terminacion','tMaxSimulado';
    'evaluacion','exito'
};

for i = 1:size(rutasNecesarias,1)
    grupo = rutasNecesarias{i,1};
    campo = rutasNecesarias{i,2};

    if ~isfield(cfg,grupo) || ...
            ~isstruct(cfg.(grupo)) || ...
            ~isfield(cfg.(grupo),campo)
        error('tasa_exito:ConfiguracionIncompleta', ...
            'Falta cfg.%s.%s.',grupo,campo);
    end
end

if ~isnumeric(cfg.sim.Ts) || ...
        ~isscalar(cfg.sim.Ts) || ...
        ~isreal(cfg.sim.Ts) || ...
        ~isfinite(cfg.sim.Ts) || ...
        cfg.sim.Ts <= 0
    error('tasa_exito:TsNoValido', ...
        'cfg.sim.Ts debe ser positivo.');
end

if ~isnumeric(cfg.terminacion.maxPasos) || ...
        ~isscalar(cfg.terminacion.maxPasos) || ...
        ~isfinite(cfg.terminacion.maxPasos) || ...
        cfg.terminacion.maxPasos < 1 || ...
        cfg.terminacion.maxPasos ~= ...
            floor(cfg.terminacion.maxPasos)
    error('tasa_exito:MaxPasosNoValido', ...
        'cfg.terminacion.maxPasos debe ser un entero positivo.');
end

if ~isnumeric(cfg.terminacion.tMaxSimulado) || ...
        ~isscalar(cfg.terminacion.tMaxSimulado) || ...
        ~isreal(cfg.terminacion.tMaxSimulado) || ...
        ~isfinite(cfg.terminacion.tMaxSimulado) || ...
        cfg.terminacion.tMaxSimulado <= 0
    error('tasa_exito:TMaxNoValido', ...
        'cfg.terminacion.tMaxSimulado debe ser positivo.');
end

camposExito = { ...
    'requiereMeta', ...
    'requiereSinColision', ...
    'requiereDentroDelTiempo'};

for i = 1:numel(camposExito)
    campo = camposExito{i};

    if ~isfield(cfg.evaluacion.exito,campo)
        error('tasa_exito:CriterioAusente', ...
            'Falta cfg.evaluacion.exito.%s.',campo);
    end

    validar_logico_escalar( ...
        cfg.evaluacion.exito.(campo), ...
        "cfg.evaluacion.exito."+campo);
end
end

function indicador = validar_indicador(indicador,nombre)
%VALIDAR_INDICADOR Convierte valores logicos o binarios a columna.

if islogical(indicador)
    indicador = reshape(indicador,[],1);
    return;
end

if ~isnumeric(indicador) || ~isreal(indicador) || ...
        any(~isfinite(indicador(:))) || ...
        any(indicador(:) ~= 0 & indicador(:) ~= 1)
    error('tasa_exito:IndicadorNoValido', ...
        '%s debe contener exclusivamente valores logicos, 0 o 1.', ...
        nombre);
end

indicador = logical(reshape(indicador,[],1));
end

function validar_logico_escalar(valor,nombre)
%VALIDAR_LOGICO_ESCALAR Comprueba un criterio booleano.

esLogico = islogical(valor) && isscalar(valor);

esBinario = isnumeric(valor) && isreal(valor) && ...
    isscalar(valor) && isfinite(valor) && ...
    (valor == 0 || valor == 1);

if ~(esLogico || esBinario)
    error('tasa_exito:CriterioNoValido', ...
        '%s debe ser un escalar logico o binario.',nombre);
end
end
