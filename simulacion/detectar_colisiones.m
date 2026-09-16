function [hayColision, detalle] = detectar_colisiones( ...
    estadoRobot, robot, obstaculosEstaticos, ...
    obstaculosDinamicos, limites, tolerancia)
% Detecta colisiones físicas del robot en el escenario.
%
%   hayColision = DETECTAR_COLISIONES( ...
%       estadoRobot, robot, obstaculosEstaticos, ...
%       obstaculosDinamicos, limites)
%
%   [hayColision,detalle] = DETECTAR_COLISIONES( ...
%       estadoRobot, robot, obstaculosEstaticos, ...
%       obstaculosDinamicos, limites, tolerancia)
%
%   Comprueba la colisión instantánea del robot con:
%
%       - los límites del mapa;
%       - los obstáculos estáticos rectangulares;
%       - los obstáculos dinámicos circulares.
%
%   Entradas:
%       estadoRobot
%           Estado actual del robot. Puede tener formato [x y] o
%           [x y theta]. Para la detección geométrica solo se utilizan
%           las dos primeras componentes.
%
%       robot
%           Estructura obtenida mediante configuracion_robot.m. Debe
%           contener:
%
%               robot.geometria.radio
%
%       obstaculosEstaticos
%           Matriz N x 4 de rectángulos:
%
%               [x y ancho alto]
%
%       obstaculosDinamicos
%           Vector de estructuras con, al menos, los campos:
%
%               pos    : centro [x y]
%               radio  : radio del obstáculo
%
%       limites
%           Límites del mapa:
%
%               [xmin xmax ymin ymax]
%
%       tolerancia
%           Tolerancia numérica opcional, no negativa.
%
%   Salidas:
%       hayColision
%           true si el robot toca o solapa algún elemento del escenario.
%
%       detalle
%           Estructura con información adicional:
%
%               .colisionLimites
%               .colisionEstaticos
%               .colisionDinamicos
%               .indicesEstaticos
%               .indicesDinamicos
%               .distanciaLibreLimites
%               .distanciasLibresEstaticos
%               .distanciasLibresDinamicos
%               .distanciaLibreMinima
%               .penetracionMaxima
%               .tipoMasCercano
%               .idMasCercano
%
%   La distancia libre se mide entre las superficies:
%
%       distancia libre > 0  -> separación
%       distancia libre = 0  -> contacto
%       distancia libre < 0  -> solapamiento o penetración
%
%   IMPORTANTE:
%   Esta función detecta colisiones FÍSICAS y no añade los márgenes de
%   seguridad de parametros_generales.m. Los márgenes se emplean durante
%   la planificación y el control. Para declarar una situación de riesgo
%   puede utilizarse posteriormente:
%
%       riesgo = detalle.distanciaLibreMinima <= ...
%           cfg.metricas.distanciaRiesgo;
%
%   Ejemplo:
%
%       cfg = parametros_generales("batch");
%       escenario = escenarios("media");
%       robot = configuracion_robot();
%
%       estado = escenario.inicio;
%
%       [colision,detalle] = detectar_colisiones( ...
%           estado, robot, ...
%           escenario.obstaculosEstaticos, ...
%           escenario.obstaculosDinamicos, ...
%           escenario.limites);
%
%       fprintf('Colisión: %d\n',colision);
%       fprintf('Distancia libre mínima: %.3f m\n', ...
%           detalle.distanciaLibreMinima);
%
%   Esta función utiliza:
%       - circulo_rectangulo.m
%       - circulo_circulo.m

if nargin < 6
    tolerancia = [];
end

%% Validación y normalización
[centroRobot, radioRobot, obstaculosEstaticos, ...
    obstaculosDinamicos, limites, tolerancia] = validar_entradas( ...
    estadoRobot, robot, obstaculosEstaticos, ...
    obstaculosDinamicos, limites, tolerancia, nargin);

nEstaticos = size(obstaculosEstaticos,1);
nDinamicos = numel(obstaculosDinamicos);

%% Colisión con los límites del mapa
xmin = limites(1);
xmax = limites(2);
ymin = limites(3);
ymax = limites(4);

% Orden: izquierda, derecha, inferior, superior.
distanciasLibresLimites = [
    centroRobot(1) - xmin - radioRobot;
    xmax - centroRobot(1) - radioRobot;
    centroRobot(2) - ymin - radioRobot;
    ymax - centroRobot(2) - radioRobot
];

[distanciaLibreLimites, indiceLimiteMasCercano] = ...
    min(distanciasLibresLimites);

colisionLimites = distanciaLibreLimites <= tolerancia;

%% Colisión con obstáculos estáticos
distanciasLibresEstaticos = inf(nEstaticos,1);
colisionesEstaticos = false(nEstaticos,1);

for i = 1:nEstaticos
    [colisionesEstaticos(i),~,distanciaCentroRectangulo] = ...
        circulo_rectangulo( ...
            centroRobot, radioRobot, ...
            obstaculosEstaticos(i,:), tolerancia);

    distanciasLibresEstaticos(i) = ...
        distanciaCentroRectangulo - radioRobot;
end

indicesEstaticos = find(colisionesEstaticos);
colisionEstaticos = any(colisionesEstaticos);

%% Colisión con obstáculos dinámicos
distanciasLibresDinamicos = inf(nDinamicos,1);
colisionesDinamicos = false(nDinamicos,1);

for i = 1:nDinamicos
    [colisionesDinamicos(i),~,distanciaCentros] = ...
        circulo_circulo( ...
            centroRobot, radioRobot, ...
            obstaculosDinamicos(i).pos, ...
            obstaculosDinamicos(i).radio, ...
            tolerancia);

    distanciasLibresDinamicos(i) = ...
        distanciaCentros - radioRobot - obstaculosDinamicos(i).radio;
end

indicesDinamicos = find(colisionesDinamicos);
colisionDinamicos = any(colisionesDinamicos);

%% Resultado global
hayColision = ...
    colisionLimites || colisionEstaticos || colisionDinamicos;

[distanciaLibreMinima, tipoMasCercano, ...
    indiceMasCercano, idMasCercano] = localizar_objeto_mas_cercano( ...
    distanciaLibreLimites, indiceLimiteMasCercano, ...
    distanciasLibresEstaticos, obstaculosDinamicos, ...
    distanciasLibresDinamicos);

%% Estructura de detalle
detalle = struct();

detalle.hayColision = hayColision;

detalle.colisionLimites = colisionLimites;
detalle.colisionEstaticos = colisionEstaticos;
detalle.colisionDinamicos = colisionDinamicos;

detalle.indicesEstaticos = indicesEstaticos;
detalle.indicesDinamicos = indicesDinamicos;

detalle.distanciasLibresLimites = distanciasLibresLimites;
detalle.distanciaLibreLimites = distanciaLibreLimites;
detalle.distanciasLibresEstaticos = distanciasLibresEstaticos;
detalle.distanciasLibresDinamicos = distanciasLibresDinamicos;

detalle.distanciaLibreMinima = distanciaLibreMinima;
detalle.penetracionMaxima = max(0,-distanciaLibreMinima);

detalle.tipoMasCercano = tipoMasCercano;
detalle.indiceMasCercano = indiceMasCercano;
detalle.idMasCercano = idMasCercano;
end

%% ========================================================================
% FUNCIONES LOCALES
% ========================================================================

function [centroRobot, radioRobot, estaticos, dinamicos, limites, tol] = ...
    validar_entradas(estadoRobot, robot, estaticos, dinamicos, ...
    limites, tolerancia, numeroEntradas)
%VALIDAR_ENTRADAS Comprueba y normaliza las entradas principales.

if ~isnumeric(estadoRobot) || ~isreal(estadoRobot) || ...
        numel(estadoRobot) < 2 || any(~isfinite(estadoRobot(:)))
    error('detectar_colisiones:EstadoRobotNoValido', ...
        ['El estado del robot debe ser un vector numérico real y finito ' ...
         'con al menos las componentes [x y].']);
end

estadoRobot = reshape(double(estadoRobot),1,[]);
centroRobot = estadoRobot(1:2);

if ~isstruct(robot) || ~isscalar(robot) || ...
        ~isfield(robot,'geometria') || ...
        ~isstruct(robot.geometria) || ...
        ~isfield(robot.geometria,'radio')
    error('detectar_colisiones:RobotNoValido', ...
        ['robot debe ser la estructura obtenida mediante ' ...
         'configuracion_robot.m.']);
end

radioRobot = robot.geometria.radio;

if ~isnumeric(radioRobot) || ~isscalar(radioRobot) || ...
        ~isreal(radioRobot) || ~isfinite(radioRobot) || radioRobot <= 0
    error('detectar_colisiones:RadioRobotNoValido', ...
        'robot.geometria.radio debe ser un escalar positivo.');
end

radioRobot = double(radioRobot);

if isempty(estaticos)
    estaticos = zeros(0,4);
elseif ~isnumeric(estaticos) || ~isreal(estaticos) || ...
        size(estaticos,2) ~= 4 || any(~isfinite(estaticos(:)))
    error('detectar_colisiones:EstaticosNoValidos', ...
        ['Los obstáculos estáticos deben ser una matriz N x 4 ' ...
         'con formato [x y ancho alto].']);
else
    estaticos = double(estaticos);
end

if ~isempty(estaticos) && any(estaticos(:,3:4) <= 0,'all')
    error('detectar_colisiones:DimensionesEstaticosNoValidas', ...
        'El ancho y el alto de los rectángulos deben ser positivos.');
end

if isempty(dinamicos)
    dinamicos = struct('id',{},'pos',{},'radio',{});
elseif ~isstruct(dinamicos)
    error('detectar_colisiones:DinamicosNoValidos', ...
        'Los obstáculos dinámicos deben proporcionarse como estructuras.');
else
    for i = 1:numel(dinamicos)
        if ~isfield(dinamicos(i),'pos') || ...
                ~isfield(dinamicos(i),'radio')
            error('detectar_colisiones:CampoDinamicoAusente', ...
                ['El obstáculo dinámico %d debe contener los campos ' ...
                 '"pos" y "radio".'],i);
        end

        if ~isnumeric(dinamicos(i).pos) || ...
                ~isreal(dinamicos(i).pos) || ...
                numel(dinamicos(i).pos) ~= 2 || ...
                any(~isfinite(dinamicos(i).pos(:)))
            error('detectar_colisiones:PosicionDinamicaNoValida', ...
                'La posición del obstáculo dinámico %d no es válida.',i);
        end

        if ~isnumeric(dinamicos(i).radio) || ...
                ~isscalar(dinamicos(i).radio) || ...
                ~isreal(dinamicos(i).radio) || ...
                ~isfinite(dinamicos(i).radio) || ...
                dinamicos(i).radio <= 0
            error('detectar_colisiones:RadioDinamicoNoValido', ...
                'El radio del obstáculo dinámico %d no es válido.',i);
        end

        dinamicos(i).pos = reshape(double(dinamicos(i).pos),1,2);
        dinamicos(i).radio = double(dinamicos(i).radio);
    end
end

if ~isnumeric(limites) || ~isreal(limites) || ...
        numel(limites) ~= 4 || any(~isfinite(limites(:)))
    error('detectar_colisiones:LimitesNoValidos', ...
        'Los límites deben tener formato [xmin xmax ymin ymax].');
end

limites = reshape(double(limites),1,4);

if limites(1) >= limites(2) || limites(3) >= limites(4)
    error('detectar_colisiones:OrdenLimitesNoValido', ...
        'Debe cumplirse xmin < xmax e ymin < ymax.');
end

if numeroEntradas < 6 || isempty(tolerancia)
    escala = max(1,max(abs([centroRobot limites radioRobot])));
    tol = 1e-12*escala;
elseif ~isnumeric(tolerancia) || ~isscalar(tolerancia) || ...
        ~isreal(tolerancia) || ~isfinite(tolerancia) || tolerancia < 0
    error('detectar_colisiones:ToleranciaNoValida', ...
        'La tolerancia debe ser un escalar real, finito y no negativo.');
else
    tol = double(tolerancia);
end
end

function [distanciaMinima, tipo, indice, identificador] = ...
    localizar_objeto_mas_cercano( ...
    distanciaLimites, indiceLimite, ...
    distanciasEstaticos, dinamicos, distanciasDinamicos)
%LOCALIZAR_OBJETO_MAS_CERCANO Identifica el menor espacio libre.

nombresLimites = ["izquierdo","derecho","inferior","superior"];

distanciaEstatica = inf;
indiceEstatico = NaN;

if ~isempty(distanciasEstaticos)
    [distanciaEstatica,indiceEstatico] = min(distanciasEstaticos);
end

distanciaDinamica = inf;
indiceDinamico = NaN;

if ~isempty(distanciasDinamicos)
    [distanciaDinamica,indiceDinamico] = min(distanciasDinamicos);
end

[distanciaMinima,caso] = min([ ...
    distanciaLimites, ...
    distanciaEstatica, ...
    distanciaDinamica]);

switch caso
    case 1
        tipo = "limite";
        indice = indiceLimite;
        identificador = nombresLimites(indiceLimite);

    case 2
        tipo = "estatico";
        indice = indiceEstatico;
        identificador = "S" + indiceEstatico;

    otherwise
        tipo = "dinamico";
        indice = indiceDinamico;

        if isfield(dinamicos(indiceDinamico),'id') && ...
                strlength(string(dinamicos(indiceDinamico).id)) > 0
            identificador = string(dinamicos(indiceDinamico).id);
        else
            identificador = "D" + indiceDinamico;
        end
end
end
