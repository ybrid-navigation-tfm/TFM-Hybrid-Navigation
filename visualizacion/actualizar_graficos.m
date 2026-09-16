function graficos = actualizar_graficos( ...
    graficos, pasoActual, estadoRobot, trayectoria, caminoGlobal, ...
    obstaculosDinamicos, vistaPlanificador, textoEstado)
% Actualiza los objetos graficos persistentes de la simulacion.

if nargin < 7 || isempty(vistaPlanificador)
    vistaPlanificador = vista_planificador_vacia();
end

if nargin < 8 || isempty(textoEstado)
    textoEstado = "";
end

%% Validacion y normalizacion
[graficos, pasoActual, estadoRobot, trayectoria, caminoGlobal, ...
    obstaculosDinamicos, vistaPlanificador, textoEstado] = ...
    validar_entradas( ...
        graficos, pasoActual, estadoRobot, trayectoria, caminoGlobal, ...
        obstaculosDinamicos, vistaPlanificador, textoEstado);

if ~graficos.activa
    return;
end

%% Indicadores de la llamada actual
graficos.actualizacionRealizada = false;
graficos.planificadorReemplazado = false;

if ~isfield(graficos,'numeroReemplazosPlanificador') || ...
        ~es_entero_no_negativo( ...
            graficos.numeroReemplazosPlanificador)
    graficos.numeroReemplazosPlanificador = 0;
end

%% Cadencia grafica
pasoProgramado = paso_visual_programado( ...
    pasoActual,graficos.actualizarCada);

% Una nueva representacion del planificador nunca debe perderse por la
% reduccion de frecuencia grafica.
actualizacionForzada = vistaPlanificador.actualizar;

if ~pasoProgramado && ~actualizacionForzada
    return;
end

objetos = graficos.objetos;
estilo = graficos.estilo;

%% ========================================================================
% TITULO
% ========================================================================

textoTitulo = graficos.textoTituloBase + ...
    " | Iteracion " + string(pasoActual);

set(graficos.titulo, ...
    'String',char(textoTitulo));


%% ========================================================================
% OBSTACULOS DINAMICOS
% ========================================================================

escalaFlecha = obtener_escala_flecha(graficos);
angulosCirculo = estilo.anguloCirculo;

for i = 1:numel(obstaculosDinamicos)
    dinamico = obstaculosDinamicos(i);

    [xCirculo,yCirculo] = coordenadas_circulo( ...
        dinamico.pos,dinamico.radio,angulosCirculo);

    set(objetos.obstaculosDinamicos(i), ...
        'XData',xCirculo, ...
        'YData',yCirculo);

    set(objetos.flechasDinamicos(i), ...
        'XData',dinamico.pos(1), ...
        'YData',dinamico.pos(2), ...
        'UData',escalaFlecha*dinamico.vel(1), ...
        'VData',escalaFlecha*dinamico.vel(2));

    identificador = obtener_id_dinamico( ...
        dinamico,i);

    set(objetos.etiquetasDinamicos(i), ...
        'Position',[dinamico.pos 0], ...
        'String',char(identificador));
end

%% ========================================================================
% PLANIFICADOR: BORRADO Y SUSTITUCION EXPLICITA
% ========================================================================

if vistaPlanificador.actualizar
    % Paso 1: vaciar siempre la representacion anterior.
    % No se llama a delete ni a clf; se conserva el mismo objeto Line.
    set(objetos.planificador, ...
        'XData',nan, ...
        'YData',nan);

    % Paso 2: convertir exclusivamente el nuevo arbol o subgrafo a un solo
    % conjunto de datos separado mediante NaN.
    [xPlanificador,yPlanificador] = datos_aristas( ...
        vistaPlanificador.nodos, ...
        vistaPlanificador.aristas);

    % Paso 3: escribir la nueva representacion. Si nodos o aristas estan
    % vacios, xPlanificador e yPlanificador son NaN y la capa queda limpia.
    set(objetos.planificador, ...
        'XData',xPlanificador, ...
        'YData',yPlanificador);

    graficos.planificadorReemplazado = true;
    graficos.numeroReemplazosPlanificador = ...
        graficos.numeroReemplazosPlanificador+1;
end

%% ========================================================================
% CAMINO GLOBAL Y WAYPOINTS
% ========================================================================

if isempty(caminoGlobal)
    set(objetos.caminoGlobal, ...
        'XData',nan, ...
        'YData',nan);

    set(objetos.waypoints, ...
        'XData',nan, ...
        'YData',nan);
else
    posicionesCamino = caminoGlobal(:,1:2);

    set(objetos.caminoGlobal, ...
        'XData',posicionesCamino(:,1), ...
        'YData',posicionesCamino(:,2));

    set(objetos.waypoints, ...
        'XData',posicionesCamino(:,1), ...
        'YData',posicionesCamino(:,2));
end

%% ========================================================================
% TRAYECTORIA EJECUTADA
% ========================================================================

if isempty(trayectoria)
    set(objetos.trayectoria, ...
        'XData',nan, ...
        'YData',nan);
else
    posicionesEjecutadas = trayectoria(:,1:2);

    set(objetos.trayectoria, ...
        'XData',posicionesEjecutadas(:,1), ...
        'YData',posicionesEjecutadas(:,2));
end

%% ========================================================================
% ROBOT
% ========================================================================

centroRobot = estadoRobot(1:2);
orientacionRobot = estadoRobot(3);
radioRobot = graficos.radioRobot;

[xRobot,yRobot] = coordenadas_circulo( ...
    centroRobot,radioRobot,angulosCirculo);

set(objetos.robot, ...
    'XData',xRobot, ...
    'YData',yRobot);

set(objetos.orientacionRobot, ...
    'XData',centroRobot(1), ...
    'YData',centroRobot(2), ...
    'UData',radioRobot*cos(orientacionRobot), ...
    'VData',radioRobot*sin(orientacionRobot));

%% Informacion de diagnostico
graficos.objetos = objetos;
graficos.pasoUltimaActualizacion = pasoActual;
graficos.actualizacionRealizada = true;
graficos.textoEstadoActual = textoEstado;
graficos.estadoRobotActual = estadoRobot;
graficos.numeroPuntosTrayectoriaMostrados = ...
    size(trayectoria,1);
graficos.numeroWaypointsMostrados = ...
    size(caminoGlobal,1);

% drawnow y pause se ejecutan deliberadamente en el main.
end

%% ========================================================================
% REPRESENTACION DEL PLANIFICADOR
% ========================================================================

function vista = vista_planificador_vacia()
%VISTA_PLANIFICADOR_VACIA Conserva el dibujo actual por defecto.

vista = struct();
vista.actualizar = false;
vista.nodos = zeros(0,2);
vista.aristas = zeros(0,2);
end

function [xData,yData] = datos_aristas(nodos,aristas)
%DATOS_ARISTAS Convierte M aristas en los datos de un unico objeto Line.
%
%   Cada arista se representa mediante:
%
%       x_i, x_j, NaN
%       y_i, y_j, NaN
%
%   La separacion mediante NaN impide que MATLAB una el extremo de una
%   arista con el inicio de la siguiente.

if isempty(nodos) || isempty(aristas)
    xData = nan;
    yData = nan;
    return;
end

extremosA = nodos(aristas(:,1),1:2);
extremosB = nodos(aristas(:,2),1:2);

xData = [ ...
    extremosA(:,1), ...
    extremosB(:,1), ...
    nan(size(aristas,1),1)].';

yData = [ ...
    extremosA(:,2), ...
    extremosB(:,2), ...
    nan(size(aristas,1),1)].';

xData = xData(:);
yData = yData(:);
end

%% ========================================================================
% UTILIDADES GRAFICAS
% ========================================================================

function [x,y] = coordenadas_circulo(centro,radio,angulos)
%COORDENADAS_CIRCULO Calcula el contorno de un circulo.

x = centro(1)+radio*cos(angulos);
y = centro(2)+radio*sin(angulos);
end

function identificador = obtener_id_dinamico(dinamico,indice)
%OBTENER_ID_DINAMICO Recupera el id almacenado o genera D1, D2, ...

if isfield(dinamico,'id') && ...
        strlength(strtrim(string(dinamico.id))) > 0
    identificador = strtrim(string(dinamico.id));
else
    identificador = "D"+indice;
end
end

function escala = obtener_escala_flecha(graficos)
%OBTENER_ESCALA_FLECHA Recupera la escala usada al dibujar el entorno.

if isfield(graficos,'escalaFlechaVelocidad') && ...
        es_escalar_positivo(graficos.escalaFlechaVelocidad)
    escala = double(graficos.escalaFlechaVelocidad);
    return;
end

if isfield(graficos,'estilo') && ...
        isstruct(graficos.estilo) && ...
        isfield(graficos.estilo,'horizonteFlechaVelocidad') && ...
        es_escalar_positivo( ...
            graficos.estilo.horizonteFlechaVelocidad)
    escala = double( ...
        graficos.estilo.horizonteFlechaVelocidad);
    return;
end

escala = 1.0;
end

function tf = paso_visual_programado(pasoActual,actualizarCada)
%PASO_VISUAL_PROGRAMADO Aplica la misma cadencia desde el primer paso.

if pasoActual == 0
    tf = true;
else
    tf = mod(pasoActual-1,actualizarCada) == 0;
end
end

%% ========================================================================
% VALIDACION
% ========================================================================

function [graficos,paso,estado,trayectoria,camino,dinamicos,vista,texto] = ...
    validar_entradas( ...
        graficos,paso,estado,trayectoria,camino,dinamicos,vista,texto)
%VALIDAR_ENTRADAS Comprueba el contrato de actualizacion grafica.

%% Estructura grafica minima, tambien en modo batch
if ~isstruct(graficos) || ~isscalar(graficos) || ...
        ~isfield(graficos,'activa')
    error('actualizar_graficos:GraficosNoValidos', ...
        ['graficos debe ser la estructura obtenida mediante ' ...
         'inicializar_figura.m.']);
end

graficos.activa = validar_logico( ...
    graficos.activa,'graficos.activa');

if ~graficos.activa
    return;
end

%% Contrato de la figura activa
camposGraficos = { ...
    'inicializada', ...
    'figura', ...
    'ejes', ...
    'titulo', ...
    'objetos', ...
    'estilo', ...
    'textoTituloBase', ...
    'actualizarCada', ...
    'radioRobot'};

for i = 1:numel(camposGraficos)
    if ~isfield(graficos,camposGraficos{i})
        error('actualizar_graficos:GraficosIncompletos', ...
            'Falta graficos.%s.',camposGraficos{i});
    end
end

if ~validar_logico( ...
        graficos.inicializada,'graficos.inicializada') || ...
        ~isgraphics(graficos.figura,'figure') || ...
        ~isgraphics(graficos.ejes,'axes') || ...
        ~isgraphics(graficos.titulo,'text')
    error('actualizar_graficos:FiguraNoInicializada', ...
        ['La figura no esta inicializada o ha sido cerrada. ' ...
         'Ejecute de nuevo inicializar_figura.m.']);
end

if ~isstruct(graficos.objetos) || ...
        ~isscalar(graficos.objetos) || ...
        ~isstruct(graficos.estilo) || ...
        ~isscalar(graficos.estilo)
    error('actualizar_graficos:EstructuraInternaNoValida', ...
        'graficos.objetos y graficos.estilo deben ser estructuras.');
end

textoTituloBase = string(graficos.textoTituloBase);

if ~isscalar(textoTituloBase) || ...
        strlength(strtrim(textoTituloBase)) == 0
    error('actualizar_graficos:TituloBaseNoValido', ...
        'graficos.textoTituloBase debe ser un texto escalar no vacio.');
end

graficos.textoTituloBase = ...
    strtrim(textoTituloBase);

if ~es_entero_positivo(graficos.actualizarCada)
    error('actualizar_graficos:CadenciaNoValida', ...
        'graficos.actualizarCada debe ser un entero positivo.');
end

graficos.actualizarCada = ...
    double(graficos.actualizarCada);

if ~es_escalar_positivo(graficos.radioRobot)
    error('actualizar_graficos:RadioRobotNoValido', ...
        ['graficos.radioRobot debe contener el radio definido por ' ...
         'dibujar_robot.m.']);
end

graficos.radioRobot = double(graficos.radioRobot);

%% Objetos graficos persistentes
camposObjetos = { ...
    'obstaculosDinamicos', ...
    'flechasDinamicos', ...
    'etiquetasDinamicos', ...
    'planificador', ...
    'caminoGlobal', ...
    'waypoints', ...
    'trayectoria', ...
    'robot', ...
    'orientacionRobot'};

for i = 1:numel(camposObjetos)
    campo = camposObjetos{i};

    if ~isfield(graficos.objetos,campo)
        error('actualizar_graficos:ObjetoAusente', ...
            'Falta graficos.objetos.%s.',campo);
    end
end

objetosUnicos = { ...
    'planificador', ...
    'caminoGlobal', ...
    'waypoints', ...
    'trayectoria', ...
    'robot', ...
    'orientacionRobot'};

for i = 1:numel(objetosUnicos)
    campo = objetosUnicos{i};
    manejador = graficos.objetos.(campo);

    if ~isscalar(manejador) || ~isgraphics(manejador)
        error('actualizar_graficos:ObjetoNoValido', ...
            ['graficos.objetos.%s no contiene un objeto grafico ' ...
             'persistente valido.'],campo);
    end
end

if ~isfield(graficos.estilo,'anguloCirculo') || ...
        ~isnumeric(graficos.estilo.anguloCirculo) || ...
        ~isreal(graficos.estilo.anguloCirculo) || ...
        numel(graficos.estilo.anguloCirculo) < 3 || ...
        any(~isfinite(graficos.estilo.anguloCirculo(:)))
    error('actualizar_graficos:AngulosCirculoNoValidos', ...
        ['graficos.estilo.anguloCirculo debe contener al menos ' ...
         'tres angulos finitos.']);
end

%% Paso actual
if ~es_entero_no_negativo(paso)
    error('actualizar_graficos:PasoNoValido', ...
        'pasoActual debe ser un entero no negativo.');
end

paso = double(paso);

%% Estado del robot
if ~isnumeric(estado) || ~isreal(estado) || ...
        numel(estado) ~= 3 || ...
        any(~isfinite(estado(:)))
    error('actualizar_graficos:EstadoRobotNoValido', ...
        'estadoRobot debe ser un vector real y finito [x y theta].');
end

estado = reshape(double(estado),1,3);
estado(3) = atan2(sin(estado(3)),cos(estado(3)));

%% Trayectoria y camino
trayectoria = validar_puntos( ...
    trayectoria,'trayectoria',true);

camino = validar_puntos( ...
    camino,'caminoGlobal',true);

%% Obstaculos dinamicos
dinamicos = validar_dinamicos(dinamicos);

numeroDinamicos = numel(dinamicos);

camposObjetosDinamicos = { ...
    'obstaculosDinamicos', ...
    'flechasDinamicos', ...
    'etiquetasDinamicos'};

for i = 1:numel(camposObjetosDinamicos)
    campo = camposObjetosDinamicos{i};
    manejadores = graficos.objetos.(campo);

    if numel(manejadores) ~= numeroDinamicos
        error('actualizar_graficos:NumeroDinamicosIncoherente', ...
            ['Se recibieron %d obstaculos dinamicos, pero ' ...
             'graficos.objetos.%s contiene %d manejadores. ' ...
             'Vuelva a llamar a dibujar_entorno.m al cambiar de ' ...
             'escenario.'], ...
            numeroDinamicos,campo,numel(manejadores));
    end

    if numeroDinamicos > 0 && ...
            any(~isgraphics(manejadores(:)))
        error('actualizar_graficos:ManejadoresDinamicosNoValidos', ...
            'Uno o varios manejadores de %s ya no son validos.',campo);
    end
end

%% Vista del planificador
vista = validar_vista_planificador(vista);

%% Texto de estado
texto = string(texto);

if ~isscalar(texto)
    error('actualizar_graficos:TextoEstadoNoValido', ...
        'textoEstado debe ser un texto escalar.');
end

texto = strtrim(texto);
end

function puntos = validar_puntos(puntos,nombre,permiteVacio)
%VALIDAR_PUNTOS Comprueba una trayectoria o camino N x M, M >= 2.

if isempty(puntos)
    if permiteVacio
        puntos = zeros(0,2);
        return;
    end
end

if ~isnumeric(puntos) || ~isreal(puntos) || ...
        ~ismatrix(puntos) || size(puntos,2) < 2 || ...
        any(~isfinite(puntos(:)))
    error('actualizar_graficos:PuntosNoValidos', ...
        ['%s debe ser una matriz numerica real y finita N x M ' ...
         'con M >= 2.'],nombre);
end

puntos = double(puntos);
end

function dinamicos = validar_dinamicos(dinamicos)
%VALIDAR_DINAMICOS Comprueba posiciones, velocidades y radios.

if isempty(dinamicos)
    dinamicos = struct( ...
        'id',{},'pos',{},'vel',{},'radio',{});
    return;
end

if ~isstruct(dinamicos)
    error('actualizar_graficos:DinamicosNoValidos', ...
        'obstaculosDinamicos debe ser un vector de estructuras.');
end

for i = 1:numel(dinamicos)
    campos = {'pos','vel','radio'};

    for j = 1:numel(campos)
        if ~isfield(dinamicos(i),campos{j})
            error('actualizar_graficos:CampoDinamicoAusente', ...
                'Falta el campo "%s" en el obstaculo dinamico %d.', ...
                campos{j},i);
        end
    end

    if ~isnumeric(dinamicos(i).pos) || ...
            ~isreal(dinamicos(i).pos) || ...
            numel(dinamicos(i).pos) ~= 2 || ...
            any(~isfinite(dinamicos(i).pos(:)))
        error('actualizar_graficos:PosicionDinamicaNoValida', ...
            'La posicion de D%d debe tener formato [x y].',i);
    end

    if ~isnumeric(dinamicos(i).vel) || ...
            ~isreal(dinamicos(i).vel) || ...
            numel(dinamicos(i).vel) ~= 2 || ...
            any(~isfinite(dinamicos(i).vel(:)))
        error('actualizar_graficos:VelocidadDinamicaNoValida', ...
            'La velocidad de D%d debe tener formato [vx vy].',i);
    end

    if ~es_escalar_positivo(dinamicos(i).radio)
        error('actualizar_graficos:RadioDinamicoNoValido', ...
            'El radio de D%d debe ser positivo.',i);
    end

    dinamicos(i).pos = reshape( ...
        double(dinamicos(i).pos),1,2);

    dinamicos(i).vel = reshape( ...
        double(dinamicos(i).vel),1,2);

    dinamicos(i).radio = ...
        double(dinamicos(i).radio);
end
end

function vista = validar_vista_planificador(vista)
%VALIDAR_VISTA_PLANIFICADOR Normaliza el subgrafo que se desea mostrar.

if isempty(vista)
    vista = vista_planificador_vacia();
    return;
end

if ~isstruct(vista) || ~isscalar(vista)
    error('actualizar_graficos:VistaPlanificadorNoValida', ...
        'vistaPlanificador debe ser una estructura escalar.');
end

if ~isfield(vista,'actualizar')
    error('actualizar_graficos:IndicadorPlanificadorAusente', ...
        'Falta vistaPlanificador.actualizar.');
end

vista.actualizar = validar_logico( ...
    vista.actualizar,'vistaPlanificador.actualizar');

% Cuando no existe una nueva planificacion, los datos no se utilizan. Se
% permiten estructuras abreviadas con solo el indicador actualizar=false.
if ~vista.actualizar
    if ~isfield(vista,'nodos')
        vista.nodos = zeros(0,2);
    end

    if ~isfield(vista,'aristas')
        vista.aristas = zeros(0,2);
    end

    return;
end

campos = {'nodos','aristas'};

for i = 1:numel(campos)
    if ~isfield(vista,campos{i})
        error('actualizar_graficos:VistaPlanificadorIncompleta', ...
            'Falta vistaPlanificador.%s.',campos{i});
    end
end

nodos = vista.nodos;
aristas = vista.aristas;

if isempty(nodos)
    nodos = zeros(0,2);
elseif ~isnumeric(nodos) || ~isreal(nodos) || ...
        ~ismatrix(nodos) || size(nodos,2) < 2 || ...
        any(~isfinite(nodos(:)))
    error('actualizar_graficos:NodosPlanificadorNoValidos', ...
        ['vistaPlanificador.nodos debe ser una matriz numerica ' ...
         'real y finita N x M con M >= 2.']);
else
    nodos = double(nodos(:,1:2));
end

if isempty(aristas)
    aristas = zeros(0,2);
elseif ~isnumeric(aristas) || ~isreal(aristas) || ...
        ~ismatrix(aristas) || size(aristas,2) ~= 2 || ...
        any(~isfinite(aristas(:))) || ...
        any(aristas(:) < 1) || ...
        any(aristas(:) ~= floor(aristas(:)))
    error('actualizar_graficos:AristasPlanificadorNoValidas', ...
        ['vistaPlanificador.aristas debe ser una matriz M x 2 ' ...
         'de indices enteros positivos.']);
else
    aristas = double(aristas);
end

if ~isempty(aristas) && isempty(nodos)
    error('actualizar_graficos:AristasSinNodos', ...
        'No pueden proporcionarse aristas sin nodos.');
end

if ~isempty(aristas) && ...
        max(aristas(:)) > size(nodos,1)
    error('actualizar_graficos:IndiceAristaFueraRango', ...
        ['Una arista referencia un nodo inexistente. El indice maximo ' ...
         'permitido es %d.'],size(nodos,1));
end

vista.nodos = nodos;
vista.aristas = aristas;
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

error('actualizar_graficos:IndicadorNoValido', ...
    '%s debe ser un escalar logico o binario.',nombre);
end

function tf = es_escalar_positivo(valor)
%ES_ESCALAR_POSITIVO Comprueba un escalar numerico real y positivo.

tf = isnumeric(valor) && isscalar(valor) && ...
    isreal(valor) && isfinite(valor) && valor > 0;
end

function tf = es_entero_positivo(valor)
%ES_ENTERO_POSITIVO Comprueba un entero estrictamente positivo.

tf = isnumeric(valor) && isscalar(valor) && ...
    isreal(valor) && isfinite(valor) && ...
    valor >= 1 && valor == floor(valor);
end

function tf = es_entero_no_negativo(valor)
%ES_ENTERO_NO_NEGATIVO Comprueba un entero mayor o igual que cero.

tf = isnumeric(valor) && isscalar(valor) && ...
    isreal(valor) && isfinite(valor) && ...
    valor >= 0 && valor == floor(valor);
end
