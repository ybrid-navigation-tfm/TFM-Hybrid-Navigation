function graficos = inicializar_figura( ...
    escenario, cfg, nombreArquitectura, figuraExistente)
%INICIALIZAR_FIGURA Crea la figura comun de las simulaciones del TFM.
%
%   graficos = INICIALIZAR_FIGURA(escenario,cfg)
%
%   graficos = INICIALIZAR_FIGURA( ...
%       escenario,cfg,nombreArquitectura)
%
%   graficos = INICIALIZAR_FIGURA( ...
%       escenario,cfg,nombreArquitectura,figuraExistente)
%
%   Inicializa una figura bidimensional con la misma configuracion visual
%   para las arquitecturas:
%
%       - RRT* + APF
%       - RRT* + MPC
%       - PRM  + MPC
%
%   La funcion crea:
%
%       - la ventana de figura;
%       - los ejes con escala espacial 1:1;
%       - limites, rejilla, etiquetas y titulo;
%       - capas graficas separadas;
%       - objetos vacios para planificador, camino y trayectoria;
%       - una paleta de estilos comun.
%
%   No dibuja todavia los obstaculos ni el robot. Estas responsabilidades
%   pertenecen a:
%
%       dibujar_entorno.m
%       dibujar_robot.m
%
%   Posteriormente, actualizar_graficos.m modificara los objetos existentes
%   mediante XData, YData, UData y VData. De este modo se evita utilizar
%   clf y reconstruir toda la figura en cada paso, mejorando notablemente
%   la eficiencia de la animacion.
%
%   Entradas:
%       escenario
%           Estructura obtenida mediante escenarios.m. Se utilizan:
%
%               escenario.id
%               escenario.nombre       (si existe)
%               escenario.limites      [xmin xmax ymin ymax]
%
%       cfg
%           Estructura obtenida mediante parametros_generales.m. Se usan:
%
%               cfg.modo
%               cfg.visual.activa
%               cfg.visual.pausa
%               cfg.visual.tPausa
%               cfg.visual.actualizarCada
%               cfg.visual.mostrarPlanificador
%
%       nombreArquitectura
%           Texto opcional, por ejemplo "RRT* + MPC". Si se omite, se
%           utiliza cfg.nombreArquitectura. Si este tambien esta vacio,
%           se muestra "Navegacion de robot movil".
%
%       figuraExistente
%           Figura opcional creada directamente desde un Live Script.
%           Cuando se proporciona, el modulo la reutiliza en lugar de
%           crear una ventana nueva. Esto permite que el Live Editor
%           registre la animacion y muestre los controles de reproduccion.
%
%   Salida:
%       graficos
%           Estructura compartida por todos los modulos de visualizacion:
%
%               .activa
%               .figura
%               .ejes
%               .titulo
%               .capas
%               .objetos
%               .estilo
%               .limites
%               .nombreArquitectura
%               .nombreEscenario
%               .textoTituloBase
%
%   Modo batch:
%       Si cfg.visual.activa es false, no se crea ninguna figura. La
%       funcion devuelve una estructura valida con graficos.activa=false.
%       Los demas modulos visuales deberan comprobar esta bandera y salir
%       sin realizar operaciones graficas.
%
%   Ejemplo:
%
%       cfg = parametros_generales("visual");
%       escenario = escenarios("media");
%
%       graficos = inicializar_figura( ...
%           escenario,cfg,"RRT* + MPC");
%
%   En el futuro main, drawnow se ejecutara despues de actualizar todos los
%   objetos. Esto permite que el Live Editor conserve los fotogramas de la
%   simulacion sin incluir el tiempo de renderizado en las metricas
%   computacionales.

if nargin < 4
    figuraExistente = [];
end

if nargin < 3 || strlength(strtrim(string(nombreArquitectura))) == 0
    if isstruct(cfg) && isfield(cfg,'nombreArquitectura') && ...
            strlength(strtrim(string(cfg.nombreArquitectura))) > 0
        nombreArquitectura = string(cfg.nombreArquitectura);
    else
        nombreArquitectura = "Navegacion de robot movil";
    end
end

%% Validacion y normalizacion
[escenario, cfg, nombreArquitectura] = ...
    validar_entradas(escenario,cfg,nombreArquitectura);

figuraExistente = validar_figura_existente( ...
    figuraExistente,cfg.visual.activa);

limites = escenario.limites;
nombreEscenario = obtener_nombre_escenario(escenario);

%% Estilo comun a todas las arquitecturas
estilo = crear_estilo(cfg);

%% Estructura base, tambien valida en modo batch
graficos = estructura_base( ...
    escenario,cfg,nombreArquitectura,nombreEscenario,estilo);

if ~cfg.visual.activa
    return;
end

%% Creacion de la figura
nombreVentana = nombreArquitectura + ...
    " | " + nombreEscenario;

if isempty(figuraExistente)
    % Compatibilidad con scripts normales que no proporcionen una figura.
    figura = figure( ...
        'Color',estilo.colorFondo, ...
        'Name',char(nombreVentana), ...
        'NumberTitle','off');
else
    % En un Live Script se reutiliza la figura creada directamente por el
    % main. Asi el Live Editor conserva la asociacion entre la figura y el
    % bucle que genera sus fotogramas.
    figura = figuraExistente;
    clf(figura);

    set(figura, ...
        'Color',estilo.colorFondo, ...
        'Name',char(nombreVentana), ...
        'NumberTitle','off');
end

ejes = axes( ...
    'Parent',figura, ...
    'NextPlot','add', ...
    'Box','on', ...
    'Layer','top', ...
    'FontSize',estilo.tamanoFuenteEjes, ...
    'TickDir','out');

hold(ejes,'on');
grid(ejes,'on');
axis(ejes,'equal');

xlim(ejes,limites(1:2));
ylim(ejes,limites(3:4));

set(ejes, ...
    'XLimMode','manual', ...
    'YLimMode','manual', ...
    'DataAspectRatio',[1 1 1]);

xlabel(ejes,'X [m]', ...
    'Interpreter','none', ...
    'FontSize',estilo.tamanoFuenteEtiquetas);

ylabel(ejes,'Y [m]', ...
    'Interpreter','none', ...
    'FontSize',estilo.tamanoFuenteEtiquetas);

textoTituloBase = nombreArquitectura + ...
    " | " + nombreEscenario;

titulo = title(ejes, ...
    textoTituloBase + " | Inicializacion", ...
    'Interpreter','none', ...
    'FontWeight','bold', ...
    'FontSize',estilo.tamanoFuenteTitulo);

%% Capas graficas
% El orden de creacion establece un orden visual coherente. Los objetos se
% actualizan posteriormente sin borrar ni reconstruir la figura.
capas = struct();

capas.entorno = hggroup( ...
    'Parent',ejes, ...
    'Tag','capa_entorno');

capas.obstaculosDinamicos = hggroup( ...
    'Parent',ejes, ...
    'Tag','capa_obstaculos_dinamicos');

capas.planificador = hggroup( ...
    'Parent',ejes, ...
    'Tag','capa_planificador');

capas.camino = hggroup( ...
    'Parent',ejes, ...
    'Tag','capa_camino');

capas.trayectoria = hggroup( ...
    'Parent',ejes, ...
    'Tag','capa_trayectoria');

capas.robot = hggroup( ...
    'Parent',ejes, ...
    'Tag','capa_robot');

capas.anotaciones = hggroup( ...
    'Parent',ejes, ...
    'Tag','capa_anotaciones');

%% Objetos persistentes comunes
objetos = graficos.objetos;

objetos.planificador = line( ...
    'Parent',capas.planificador, ...
    'XData',nan, ...
    'YData',nan, ...
    'LineStyle','-', ...
    'LineWidth',estilo.anchoPlanificador, ...
    'Color',estilo.colorPlanificador, ...
    'Visible',logico_a_visibilidad( ...
        cfg.visual.mostrarPlanificador), ...
    'Tag','planificador');

objetos.caminoGlobal = line( ...
    'Parent',capas.camino, ...
    'XData',nan, ...
    'YData',nan, ...
    'LineStyle','-', ...
    'LineWidth',estilo.anchoCamino, ...
    'Color',estilo.colorCamino, ...
    'Tag','camino_global');

objetos.waypoints = line( ...
    'Parent',capas.camino, ...
    'XData',nan, ...
    'YData',nan, ...
    'LineStyle','none', ...
    'Marker','o', ...
    'MarkerSize',estilo.tamanoWaypoint, ...
    'MarkerEdgeColor',estilo.colorCamino, ...
    'MarkerFaceColor',estilo.colorCamino, ...
    'Tag','waypoints');

objetos.trayectoria = line( ...
    'Parent',capas.trayectoria, ...
    'XData',nan, ...
    'YData',nan, ...
    'LineStyle','-', ...
    'LineWidth',estilo.anchoTrayectoria, ...
    'Color',estilo.colorTrayectoria, ...
    'Tag','trayectoria_ejecutada');

%% Incorporacion de los manejadores a la salida
graficos.figura = figura;
graficos.ejes = ejes;
graficos.titulo = titulo;

graficos.capas = capas;
graficos.objetos = objetos;

graficos.textoTituloBase = textoTituloBase;
graficos.inicializada = true;

% El renderizado se difiere hasta el drawnow del bucle principal.
end

%% ========================================================================
% ESTRUCTURAS Y ESTILO
% ========================================================================

function graficos = estructura_base( ...
    escenario,cfg,nombreArquitectura,nombreEscenario,estilo)
%ESTRUCTURA_BASE Define un contrato estable para la visualizacion.

graficos = struct();

graficos.version = "1.0";
graficos.activa = logical(cfg.visual.activa);
graficos.inicializada = false;

graficos.figura = gobjects(0);
graficos.ejes = gobjects(0);
graficos.titulo = gobjects(0);

graficos.capas = struct();

objetos = struct();

objetos.bordeMapa = gobjects(0);
objetos.obstaculosEstaticos = gobjects(0);
objetos.etiquetasEstaticos = gobjects(0);

objetos.obstaculosDinamicos = gobjects(0);
objetos.flechasDinamicos = gobjects(0);
objetos.etiquetasDinamicos = gobjects(0);

objetos.inicio = gobjects(0);
objetos.meta = gobjects(0);
objetos.etiquetaInicio = gobjects(0);
objetos.etiquetaMeta = gobjects(0);

objetos.planificador = gobjects(0);
objetos.caminoGlobal = gobjects(0);
objetos.waypoints = gobjects(0);
objetos.trayectoria = gobjects(0);

objetos.robot = gobjects(0);
objetos.orientacionRobot = gobjects(0);

objetos.textoEstado = gobjects(0);

graficos.objetos = objetos;
graficos.estilo = estilo;

graficos.limites = escenario.limites;
graficos.idEscenario = string(escenario.id);
graficos.nombreEscenario = nombreEscenario;
graficos.nombreArquitectura = nombreArquitectura;

graficos.textoTituloBase = ...
    nombreArquitectura + " | " + nombreEscenario;

if isfield(cfg,'modo')
    graficos.modo = string(cfg.modo);
else
    graficos.modo = "";
end

graficos.pausa = logical(cfg.visual.pausa);
graficos.tPausa = double(cfg.visual.tPausa);
graficos.actualizarCada = double(cfg.visual.actualizarCada);
graficos.mostrarPlanificador = ...
    logical(cfg.visual.mostrarPlanificador);

graficos.pasoUltimaActualizacion = 0;
end

function estilo = crear_estilo(cfg)
%CREAR_ESTILO Centraliza la apariencia comun de las simulaciones.

estilo = struct();

estilo.colorFondo = [1.00 1.00 1.00];
estilo.colorBordeMapa = [0.00 0.00 0.00];

estilo.colorObstaculoEstatico = [0.25 0.25 0.25];
estilo.colorTextoEstatico = [1.00 1.00 1.00];

estilo.colorObstaculoDinamico = [0.90 0.10 0.10];
estilo.colorFlechaDinamica = [0.80 0.00 0.00];
estilo.colorTextoDinamico = [1.00 1.00 1.00];

estilo.colorPlanificador = [0.75 0.75 0.75];
estilo.colorCamino = [0.00 0.00 1.00];
estilo.colorTrayectoria = [1.00 0.00 1.00];

estilo.colorRobot = [0.10 0.50 1.00];
estilo.colorOrientacionRobot = [0.00 0.00 0.00];

estilo.colorInicio = [0.00 0.70 0.00];
estilo.colorMeta = [1.00 0.00 0.00];

estilo.anchoBordeMapa = 1.50;
estilo.anchoPlanificador = 0.50;
estilo.anchoCamino = 2.20;
estilo.anchoTrayectoria = 2.00;
estilo.anchoFlecha = 1.50;
estilo.anchoOrientacion = 1.60;

estilo.alphaObstaculoDinamico = 0.75;
estilo.alphaRobot = 0.80;

estilo.tamanoWaypoint = 4;
estilo.tamanoInicio = 10;
estilo.tamanoMeta = 14;

estilo.tamanoFuenteEjes = 10;
estilo.tamanoFuenteEtiquetas = 11;
estilo.tamanoFuenteTitulo = 12;
estilo.tamanoFuenteObjetos = 10;

estilo.numeroPuntosCirculo = 60;
estilo.anguloCirculo = linspace(0,2*pi, ...
    estilo.numeroPuntosCirculo);

% Las flechas mostraran el desplazamiento predicho durante el mismo
% horizonte temporal usado por la simulacion, en lugar de aplicar un
% factor grafico arbitrario.
if isfield(cfg,'prediccion') && ...
        isstruct(cfg.prediccion) && ...
        isfield(cfg.prediccion,'pasos') && ...
        isfield(cfg,'sim') && ...
        isfield(cfg.sim,'Ts')

    estilo.horizonteFlechaVelocidad = ...
        cfg.prediccion.pasos*cfg.sim.Ts;
else
    estilo.horizonteFlechaVelocidad = 1.0;
end
end

%% ========================================================================
% UTILIDADES
% ========================================================================

function nombre = obtener_nombre_escenario(escenario)
%OBTENER_NOMBRE_ESCENARIO Devuelve una etiqueta legible.

if isfield(escenario,'nombre') && ...
        strlength(strtrim(string(escenario.nombre))) > 0
    nombre = string(escenario.nombre);
else
    nombre = "Escenario " + string(escenario.id);
end
end

function valor = logico_a_visibilidad(indicador)
%LOGICO_A_VISIBILIDAD Convierte true/false a on/off.

if indicador
    valor = 'on';
else
    valor = 'off';
end
end

%% ========================================================================
% VALIDACION
% ========================================================================

function figura = validar_figura_existente(figura,visualActiva)
%VALIDAR_FIGURA_EXISTENTE Normaliza la figura creada por el Live Script.

if isempty(figura)
    figura = [];
    return;
end

if ~(islogical(visualActiva) && isscalar(visualActiva))
    error('inicializar_figura:IndicadorVisualNoValido', ...
        'cfg.visual.activa debe ser un escalar logico.');
end

if ~visualActiva
    error('inicializar_figura:FiguraEnModoBatch', ...
        ['No proporcione figuraExistente cuando ' ...
         'cfg.visual.activa es false.']);
end

if ~isscalar(figura) || ~isgraphics(figura,'figure')
    error('inicializar_figura:FiguraExistenteNoValida', ...
        ['figuraExistente debe ser una figura valida creada ' ...
         'directamente por el Live Script.']);
end
end

function [escenario,cfg,nombreArquitectura] = ...
    validar_entradas(escenario,cfg,nombreArquitectura)
%VALIDAR_ENTRADAS Comprueba el contrato minimo del modulo.

%% Escenario
if ~isstruct(escenario) || ~isscalar(escenario)
    error('inicializar_figura:EscenarioNoValido', ...
        'escenario debe ser la estructura obtenida mediante escenarios.m.');
end

camposEscenario = {'id','limites'};

for i = 1:numel(camposEscenario)
    if ~isfield(escenario,camposEscenario{i})
        error('inicializar_figura:EscenarioIncompleto', ...
            'Falta escenario.%s.',camposEscenario{i});
    end
end

idEscenario = string(escenario.id);

if ~isscalar(idEscenario) || ...
        strlength(strtrim(idEscenario)) == 0
    error('inicializar_figura:IdEscenarioNoValido', ...
        'escenario.id debe ser un texto escalar no vacio.');
end

escenario.id = idEscenario;

if ~isnumeric(escenario.limites) || ...
        ~isreal(escenario.limites) || ...
        numel(escenario.limites) ~= 4 || ...
        any(~isfinite(escenario.limites(:)))
    error('inicializar_figura:LimitesNoValidos', ...
        'escenario.limites debe tener formato [xmin xmax ymin ymax].');
end

escenario.limites = reshape( ...
    double(escenario.limites),1,4);

if escenario.limites(1) >= escenario.limites(2) || ...
        escenario.limites(3) >= escenario.limites(4)
    error('inicializar_figura:OrdenLimitesNoValido', ...
        'Debe cumplirse xmin < xmax e ymin < ymax.');
end

%% Configuracion
if ~isstruct(cfg) || ~isscalar(cfg) || ...
        ~isfield(cfg,'visual') || ...
        ~isstruct(cfg.visual)
    error('inicializar_figura:ConfiguracionNoValida', ...
        'cfg debe proceder de parametros_generales.m.');
end

camposVisuales = { ...
    'activa', ...
    'pausa', ...
    'tPausa', ...
    'actualizarCada', ...
    'mostrarPlanificador'};

for i = 1:numel(camposVisuales)
    if ~isfield(cfg.visual,camposVisuales{i})
        error('inicializar_figura:ConfiguracionVisualIncompleta', ...
            'Falta cfg.visual.%s.',camposVisuales{i});
    end
end

cfg.visual.activa = validar_logico( ...
    cfg.visual.activa,'cfg.visual.activa');

cfg.visual.pausa = validar_logico( ...
    cfg.visual.pausa,'cfg.visual.pausa');

cfg.visual.mostrarPlanificador = validar_logico( ...
    cfg.visual.mostrarPlanificador, ...
    'cfg.visual.mostrarPlanificador');

if ~isnumeric(cfg.visual.tPausa) || ...
        ~isscalar(cfg.visual.tPausa) || ...
        ~isreal(cfg.visual.tPausa) || ...
        ~isfinite(cfg.visual.tPausa) || ...
        cfg.visual.tPausa < 0
    error('inicializar_figura:PausaNoValida', ...
        'cfg.visual.tPausa debe ser un escalar no negativo.');
end

cfg.visual.tPausa = double(cfg.visual.tPausa);

if ~isnumeric(cfg.visual.actualizarCada) || ...
        ~isscalar(cfg.visual.actualizarCada) || ...
        ~isreal(cfg.visual.actualizarCada) || ...
        ~isfinite(cfg.visual.actualizarCada) || ...
        cfg.visual.actualizarCada < 1 || ...
        cfg.visual.actualizarCada ~= ...
            floor(cfg.visual.actualizarCada)
    error('inicializar_figura:FrecuenciaVisualNoValida', ...
        'cfg.visual.actualizarCada debe ser un entero positivo.');
end

cfg.visual.actualizarCada = ...
    double(cfg.visual.actualizarCada);

%% Nombre de la arquitectura
nombreArquitectura = ...
    string(nombreArquitectura);

if ~isscalar(nombreArquitectura) || ...
        strlength(strtrim(nombreArquitectura)) == 0
    error('inicializar_figura:NombreNoValido', ...
        'nombreArquitectura debe ser un texto escalar no vacio.');
end

nombreArquitectura = strtrim(nombreArquitectura);
end

function valor = validar_logico(valor,nombre)
%VALIDAR_LOGICO Convierte un escalar logico o binario.

if islogical(valor) && isscalar(valor)
    return;
end

if isnumeric(valor) && isscalar(valor) && ...
        isreal(valor) && isfinite(valor) && ...
        (valor == 0 || valor == 1)
    valor = logical(valor);
    return;
end

error('inicializar_figura:IndicadorNoValido', ...
    '%s debe ser un escalar logico o binario.',nombre);
end
