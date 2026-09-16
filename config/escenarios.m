function escenario = escenarios(idEscenario)
% Define los tres escenarios del entorno experimental.
%
%   escenario = ESCENARIOS("baja")
%   escenario = ESCENARIOS("media")
%   escenario = ESCENARIOS("alta")
%
%   Si no se especifica un nivel, se utiliza "media".
%
%   Los tres escenarios siguen la propuesta experimental:
%
%       Nivel    Estaticos   Dinamicos   Rapidez dinamicos
%       baja         2           2             0.2 m/s
%       media        4           4             0.4 m/s
%       alta         6           8             0.6 m/s
%
%   En cada escenario, la mitad de los obstaculos estaticos son
%   verticales y la otra mitad horizontales:
%
%       baja  -> 1 vertical  + 1 horizontal
%       media -> 2 verticales + 2 horizontales
%       alta  -> 3 verticales + 3 horizontales
%
%   Convenciones:
%       limites                = [xmin xmax ymin ymax]       [m]
%       inicio                 = [x y theta]                 [m,m,rad]
%       meta                   = [x y]                       [m]
%       obstaculosEstaticos    = [x y ancho alto]            [m]
%       obstaculosDinamicos(i) = struct(id,pos,vel,radio)    [m,m/s,m]
%
%   Las velocidades dinamicas se expresan en m/s. Por tanto, la futura
%   funcion actualizar_obstaculos.m debera integrar el movimiento como:
%
%       posicionSiguiente = posicionActual + cfg.sim.Ts * velocidad;

%% Seleccion del escenario
if nargin < 1 || strlength(strtrim(string(idEscenario))) == 0
    idEscenario = "media";
end

idEscenario = lower(strtrim(string(idEscenario)));

if ~isscalar(idEscenario) || ...
        ~ismember(idEscenario, ["baja", "media", "alta"])
    error('escenarios:IdNoValido', ...
        'El escenario debe ser "baja", "media" o "alta".');
end

%% Datos comunes
escenario = struct();
escenario.id = idEscenario;
escenario.limites = [0 20 0 20];
escenario.inicio = [1 1 0];
escenario.meta = [18 18];
escenario.movimientoDinamico = "lineal_con_rebote";

escenario.unidades.longitud = "m";
escenario.unidades.velocidad = "m/s";
escenario.unidades.angulo = "rad";

radioDinamico = 0.80;

%% Definicion de los tres escenarios
switch idEscenario
    % ================================================================
    % BAJA: 2 estaticos (1 V + 1 H), 2 dinamicos, 0.2 m/s
    % ================================================================
    case "baja"
        escenario.nombre = "Baja complejidad";
        escenario.nivel = "baja";
        escenario.descripcion = [ ...
            "Dos obstaculos estaticos: uno vertical y uno horizontal; " ...
            "dos obstaculos dinamicos."];
        escenario.velocidadObstaculos = 0.20;

        escenario.obstaculosEstaticos = [
             5.0   4.0   2.0   6.0;   % S1 vertical
            11.0  12.0   5.0   1.5    % S2 horizontal
        ];

        posiciones = [
             2.5  17.5;
            17.5   3.0
        ];

        direcciones = [
             1.0  -0.5;
            -1.0   0.6
        ];

    % ================================================================
    % MEDIA: 4 estaticos (2 V + 2 H), 4 dinamicos, 0.4 m/s
    % ================================================================
    case "media"
        escenario.nombre = "Complejidad media";
        escenario.nivel = "media";
        escenario.descripcion = [ ...
            "Cuatro obstaculos estaticos: dos verticales y dos " ...
            "horizontales; cuatro obstaculos dinamicos."];
        escenario.velocidadObstaculos = 0.40;

        escenario.obstaculosEstaticos = [
             4.0   4.0   2.0   6.0;   % S1 vertical
            15.0  10.0   2.0   5.0;   % S2 vertical
             8.0   2.5   5.0   1.3;   % S3 horizontal
             7.0  12.0   6.0   1.3    % S4 horizontal
        ];

        % Se conservan las posiciones iniciales del circuito de referencia.
        posiciones = [
             2.5  17.5;
            18.0   3.0;
             8.0   8.5;
            17.5  17.0
        ];

        % Direcciones del circuito de referencia; despues se normalizan.
        direcciones = [
             0.08  -0.04;
            -0.07   0.05;
             0.06   0.04;
            -0.08  -0.05
        ];

    % ================================================================
    % ALTA: 6 estaticos (3 V + 3 H), 8 dinamicos, 0.6 m/s
    % ================================================================
    case "alta"
        escenario.nombre = "Alta complejidad";
        escenario.nivel = "alta";
        escenario.descripcion = [ ...
            "Seis obstaculos estaticos: tres verticales y tres " ...
            "horizontales; ocho obstaculos dinamicos."];
        escenario.velocidadObstaculos = 0.60;

        escenario.obstaculosEstaticos = [
             3.5   4.0   1.8   6.0;   % S1 vertical
             9.3   2.0   1.8   5.0;   % S2 vertical
            15.0  10.5   1.8   5.0;   % S3 vertical
             6.5  10.8   5.5   1.3;   % S4 horizontal
             1.5  14.5   4.5   1.3;   % S5 horizontal
             9.5  16.3   4.0   1.3    % S6 horizontal
        ];

        posiciones = [
             2.2  17.5;
            18.0   3.0;
             7.4   8.3;
            17.8  17.0;
             2.5  11.8;
            13.2   8.0;
             7.0  16.8;
            18.0   8.0
        ];

        direcciones = [
             1.0  -0.5;
            -1.0   0.7;
             0.8   0.5;
            -0.8  -0.6;
             0.7   0.9;
            -0.9   0.4;
             0.5  -1.0;
            -0.6  -0.8
        ];
end

%% Construccion de los obstaculos dinamicos
escenario.obstaculosDinamicos = crear_dinamicos( ...
    posiciones, direcciones, escenario.velocidadObstaculos, ...
    radioDinamico);

%% Resumen del escenario
orientaciones = clasificar_estaticos(escenario.obstaculosEstaticos);

escenario.orientacionEstaticos = orientaciones;
escenario.nEstaticos = size(escenario.obstaculosEstaticos,1);
escenario.nVerticales = nnz(orientaciones == "vertical");
escenario.nHorizontales = nnz(orientaciones == "horizontal");
escenario.nDinamicos = numel(escenario.obstaculosDinamicos);

%% Validacion interna
validar_escenario(escenario);
end

%% ========================================================================
% FUNCIONES LOCALES
% ========================================================================

function dinamicos = crear_dinamicos(posiciones, direcciones, rapidez, radio)
%CREAR_DINAMICOS Crea velocidades con igual modulo y distinta direccion.

if size(posiciones,2) ~= 2 || ~isequal(size(posiciones),size(direcciones))
    error('escenarios:DatosDinamicosInvalidos', ...
        'Posiciones y direcciones deben ser matrices N x 2.');
end

normas = vecnorm(direcciones,2,2);
if any(normas <= eps)
    error('escenarios:DireccionNula', ...
        'Ningun obstaculo dinamico puede tener direccion nula.');
end

velocidades = rapidez .* (direcciones ./ normas);
n = size(posiciones,1);

dinamicos = repmat(struct( ...
    'id', "", ...
    'pos', [0 0], ...
    'vel', [0 0], ...
    'radio', radio), n, 1);

for i = 1:n
    dinamicos(i).id = "D" + i;
    dinamicos(i).pos = posiciones(i,:);
    dinamicos(i).vel = velocidades(i,:);
    dinamicos(i).radio = radio;
end
end

function orientaciones = clasificar_estaticos(obstaculos)
%CLASIFICAR_ESTATICOS Clasifica cada rectangulo por su lado mayor.

n = size(obstaculos,1);
orientaciones = strings(n,1);

for i = 1:n
    ancho = obstaculos(i,3);
    alto = obstaculos(i,4);

    if alto > ancho
        orientaciones(i) = "vertical";
    elseif ancho > alto
        orientaciones(i) = "horizontal";
    else
        error('escenarios:ObstaculoCuadrado', ...
            'Los obstaculos estaticos deben ser horizontales o verticales.');
    end
end
end

function validar_escenario(escenario)
%VALIDAR_ESCENARIO Comprueba cantidades, orientaciones y geometria inicial.

if escenario.nivel == "baja"
    esperado = [2 2 0.20 1 1];
elseif escenario.nivel == "media"
    esperado = [4 4 0.40 2 2];
else
    esperado = [6 8 0.60 3 3];
end

if escenario.nEstaticos ~= esperado(1) || ...
        escenario.nDinamicos ~= esperado(2) || ...
        abs(escenario.velocidadObstaculos-esperado(3)) > 1e-12 || ...
        escenario.nVerticales ~= esperado(4) || ...
        escenario.nHorizontales ~= esperado(5)
    error('escenarios:ComplejidadIncoherente', ...
        ['El escenario no cumple las cantidades, velocidades u ' ...
         'orientaciones definidas para su nivel.']);
end

limites = escenario.limites;
estaticos = escenario.obstaculosEstaticos;

% Obstaculos estaticos dentro del mapa y sin solapamiento.
for i = 1:size(estaticos,1)
    r = estaticos(i,:);
    if r(3) <= 0 || r(4) <= 0 || ...
            r(1) < limites(1) || r(2) < limites(3) || ...
            r(1)+r(3) > limites(2) || r(2)+r(4) > limites(4)
        error('escenarios:EstaticoFueraMapa', ...
            'El obstaculo estatico S%d no es valido.',i);
    end

    for j = i+1:size(estaticos,1)
        if rectangulos_solapados(r,estaticos(j,:))
            error('escenarios:EstaticosSolapados', ...
                'Los obstaculos S%d y S%d se solapan.',i,j);
        end
    end
end

% Inicio y meta libres.
if distancia_punto_rectangulo(escenario.inicio(1:2),estaticos) <= 0
    error('escenarios:InicioBloqueado', ...
        'La posicion inicial esta dentro de un obstaculo estatico.');
end

if distancia_punto_rectangulo(escenario.meta,estaticos) <= 0
    error('escenarios:MetaBloqueada', ...
        'La meta esta dentro de un obstaculo estatico.');
end

% Obstaculos dinamicos dentro del mapa, con rapidez correcta y sin
% solapamientos iniciales.
dinamicos = escenario.obstaculosDinamicos;
for i = 1:numel(dinamicos)
    p = dinamicos(i).pos;
    radio = dinamicos(i).radio;

    if p(1)-radio < limites(1) || p(1)+radio > limites(2) || ...
            p(2)-radio < limites(3) || p(2)+radio > limites(4)
        error('escenarios:DinamicoFueraMapa', ...
            'El obstaculo dinamico D%d queda fuera del mapa.',i);
    end

    if abs(norm(dinamicos(i).vel)-escenario.velocidadObstaculos) > 1e-10
        error('escenarios:RapidezIncorrecta', ...
            'La rapidez de D%d no coincide con el nivel.',i);
    end

    if distancia_punto_rectangulo(p,estaticos) <= radio
        error('escenarios:DinamicoSobreEstatico', ...
            'El obstaculo dinamico D%d comienza sobre un estatico.',i);
    end

    for j = i+1:numel(dinamicos)
        if norm(p-dinamicos(j).pos) <= radio+dinamicos(j).radio
            error('escenarios:DinamicosSolapados', ...
                'Los obstaculos dinamicos D%d y D%d se solapan.',i,j);
        end
    end
end
end

function tf = rectangulos_solapados(a,b)
%RECTANGULOS_SOLAPADOS Verdadero si sus interiores se intersectan.

tf = a(1) < b(1)+b(3) && a(1)+a(3) > b(1) && ...
     a(2) < b(2)+b(4) && a(2)+a(4) > b(2);
end

function dmin = distancia_punto_rectangulo(p,obstaculos)
%DISTANCIA_PUNTO_RECTANGULO Distancia minima a un conjunto de rectangulos.

dmin = inf;
for i = 1:size(obstaculos,1)
    r = obstaculos(i,:);
    xmin = r(1);
    ymin = r(2);
    xmax = r(1)+r(3);
    ymax = r(2)+r(4);

    dx = max([xmin-p(1), 0, p(1)-xmax]);
    dy = max([ymin-p(2), 0, p(2)-ymax]);
    dmin = min(dmin,hypot(dx,dy));
end
end
