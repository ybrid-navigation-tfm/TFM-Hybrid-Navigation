function obstaculos = actualizar_obstaculos( ...
    obstaculos, obstaculosEstaticos, limites, Ts)
% Actualiza los obstáculos dinámicos del escenario.
%
%   obstaculosActualizados = ACTUALIZAR_OBSTACULOS( ...
%       obstaculos, obstaculosEstaticos, limites, Ts)
%
%   Integra durante un paso temporal el movimiento de los obstáculos
%   dinámicos y resuelve los posibles rebotes contra:
%
%       - los límites del escenario;
%       - los obstáculos estáticos rectangulares;
%       - otros obstáculos dinámicos circulares.
%
%   Entradas:
%       obstaculos
%           Vector de estructuras con los campos:
%
%               id      : identificador del obstáculo
%               pos     : posición del centro [x y]          [m]
%               vel     : velocidad lineal [vx vy]           [m/s]
%               radio   : radio del obstáculo                 [m]
%
%       obstaculosEstaticos
%           Matriz N x 4 con rectángulos en formato:
%
%               [x y ancho alto]
%
%       limites
%           Límites del escenario:
%
%               [xmin xmax ymin ymax]
%
%       Ts
%           Periodo de muestreo de la simulación              [s]
%
%   Salida:
%       obstaculos
%           Vector de obstáculos dinámicos con posiciones y velocidades
%           actualizadas.
%
%   Modelo de movimiento:
%
%       pos(k+1) = pos(k) + Ts*vel(k)
%
%   Los rebotes contra superficies estáticas se calculan mediante la
%   reflexión de la velocidad respecto a la normal de contacto:
%
%       vNueva = v - 2*(v·n)*n
%
%   Para los contactos entre obstáculos dinámicos se considera una
%   colisión elástica entre cuerpos de igual masa. Las componentes
%   normales de sus velocidades se intercambian y las componentes
%   tangenciales se conservan.
%
%   Esta función utiliza:
%
%       circulo_rectangulo.m
%       circulo_circulo.m

%% Validación de entradas
validar_entradas(obstaculos, obstaculosEstaticos, limites, Ts);

if isempty(obstaculos)
    return;
end

nObstaculos = numel(obstaculos);

%% Conservación de las posiciones anteriores
posicionesAnteriores = zeros(nObstaculos,2);

for i = 1:nObstaculos
    posicionesAnteriores(i,:) = obstaculos(i).pos;
end

%% Movimiento y rebote contra límites y obstáculos estáticos
for i = 1:nObstaculos

    posicionAnterior = posicionesAnteriores(i,:);
    velocidad = obstaculos(i).vel;
    radio = obstaculos(i).radio;

    posicionPropuesta = posicionAnterior + Ts*velocidad;

    %% Rebote contra los límites del escenario
    [posicionPropuesta, velocidad] = resolver_rebote_limites( ...
        posicionAnterior, posicionPropuesta, velocidad, ...
        radio, limites, Ts);

    %% Rebote contra los obstáculos estáticos
    for j = 1:size(obstaculosEstaticos,1)

        rectangulo = obstaculosEstaticos(j,:);

        [hayColision, normal] = circulo_rectangulo( ...
            posicionPropuesta, radio, rectangulo);

        if hayColision && dot(velocidad,normal) < 0

            velocidad = reflejar_velocidad(velocidad,normal);

            posicionPropuesta = ...
                posicionAnterior + Ts*velocidad;
        end
    end

    obstaculos(i).pos = posicionPropuesta;
    obstaculos(i).vel = velocidad;
end

%% Rebote entre obstáculos dinámicos
for i = 1:nObstaculos-1
    for j = i+1:nObstaculos

        [hayColision, normal, distancia] = circulo_circulo( ...
            obstaculos(i).pos, obstaculos(i).radio, ...
            obstaculos(j).pos, obstaculos(j).radio);

        if ~hayColision
            continue;
        end

        velocidadRelativa = ...
            obstaculos(i).vel - obstaculos(j).vel;

        % Solo se resuelve el choque si ambos obstáculos se aproximan.
        if dot(velocidadRelativa,normal) >= 0
            continue;
        end

        velocidadI = obstaculos(i).vel;
        velocidadJ = obstaculos(j).vel;

        componenteNormalI = dot(velocidadI,normal)*normal;
        componenteNormalJ = dot(velocidadJ,normal)*normal;

        componenteTangencialI = velocidadI - componenteNormalI;
        componenteTangencialJ = velocidadJ - componenteNormalJ;

        % Colisión elástica entre cuerpos de igual masa.
        obstaculos(i).vel = ...
            componenteTangencialI + componenteNormalJ;

        obstaculos(j).vel = ...
            componenteTangencialJ + componenteNormalI;

        %% Corrección del posible solapamiento
        distanciaMinima = ...
            obstaculos(i).radio + obstaculos(j).radio;

        solapamiento = distanciaMinima - distancia;

        if solapamiento > 0
            correccion = 0.5*solapamiento*normal;

            obstaculos(i).pos = ...
                obstaculos(i).pos + correccion;

            obstaculos(j).pos = ...
                obstaculos(j).pos - correccion;
        end
    end
end
end

%% ========================================================================
% FUNCIONES LOCALES
% ========================================================================

function [posicion, velocidad] = resolver_rebote_limites( ...
    posicionAnterior, posicion, velocidad, radio, limites, Ts)
%RESOLVER_REBOTE_LIMITES Resuelve el rebote contra el contorno del mapa.

xmin = limites(1);
xmax = limites(2);
ymin = limites(3);
ymax = limites(4);

%% Límites verticales
if posicion(1)-radio <= xmin || posicion(1)+radio >= xmax
    velocidad(1) = -velocidad(1);
    posicion(1) = posicionAnterior(1) + Ts*velocidad(1);
end

%% Límites horizontales
if posicion(2)-radio <= ymin || posicion(2)+radio >= ymax
    velocidad(2) = -velocidad(2);
    posicion(2) = posicionAnterior(2) + Ts*velocidad(2);
end

%% Protección frente a pequeños errores numéricos
posicion(1) = min(max(posicion(1),xmin+radio),xmax-radio);
posicion(2) = min(max(posicion(2),ymin+radio),ymax-radio);
end

function velocidadReflejada = reflejar_velocidad(velocidad,normal)
%REFLEJAR_VELOCIDAD Refleja una velocidad respecto a una normal unitaria.

velocidadReflejada = ...
    velocidad - 2*dot(velocidad,normal)*normal;
end

function validar_entradas( ...
    obstaculos, obstaculosEstaticos, limites, Ts)
%VALIDAR_ENTRADAS Comprueba la consistencia de las entradas principales.

if ~isstruct(obstaculos)
    error('actualizar_obstaculos:ObstaculosNoValidos', ...
        'Los obstáculos dinámicos deben proporcionarse como estructuras.');
end

camposNecesarios = {'pos','vel','radio'};

for i = 1:numel(obstaculos)
    for j = 1:numel(camposNecesarios)
        if ~isfield(obstaculos(i),camposNecesarios{j})
            error('actualizar_obstaculos:CampoAusente', ...
                'Falta el campo "%s" en el obstáculo dinámico %d.', ...
                camposNecesarios{j},i);
        end
    end

    if ~isnumeric(obstaculos(i).pos) || ...
            numel(obstaculos(i).pos) ~= 2 || ...
            any(~isfinite(obstaculos(i).pos))
        error('actualizar_obstaculos:PosicionNoValida', ...
            'La posición del obstáculo %d debe tener formato [x y].',i);
    end

    if ~isnumeric(obstaculos(i).vel) || ...
            numel(obstaculos(i).vel) ~= 2 || ...
            any(~isfinite(obstaculos(i).vel))
        error('actualizar_obstaculos:VelocidadNoValida', ...
            'La velocidad del obstáculo %d debe tener formato [vx vy].',i);
    end

    if ~isnumeric(obstaculos(i).radio) || ...
            ~isscalar(obstaculos(i).radio) || ...
            ~isfinite(obstaculos(i).radio) || ...
            obstaculos(i).radio <= 0
        error('actualizar_obstaculos:RadioNoValido', ...
            'El radio del obstáculo %d debe ser positivo.',i);
    end
end

if ~isnumeric(obstaculosEstaticos) || ...
        size(obstaculosEstaticos,2) ~= 4 || ...
        any(~isfinite(obstaculosEstaticos(:)))
    error('actualizar_obstaculos:EstaticosNoValidos', ...
        ['Los obstáculos estáticos deben ser una matriz N x 4 ' ...
         'con formato [x y ancho alto].']);
end

if any(obstaculosEstaticos(:,3:4) <= 0,'all')
    error('actualizar_obstaculos:DimensionesNoValidas', ...
        'El ancho y el alto de los rectángulos deben ser positivos.');
end

if ~isnumeric(limites) || numel(limites) ~= 4 || ...
        any(~isfinite(limites))
    error('actualizar_obstaculos:LimitesNoValidos', ...
        'Los límites deben tener formato [xmin xmax ymin ymax].');
end

limites = reshape(limites,1,4);

if limites(1) >= limites(2) || limites(3) >= limites(4)
    error('actualizar_obstaculos:OrdenLimitesNoValido', ...
        'Debe cumplirse xmin < xmax e ymin < ymax.');
end

if ~isnumeric(Ts) || ~isscalar(Ts) || ...
        ~isreal(Ts) || ~isfinite(Ts) || Ts <= 0
    error('actualizar_obstaculos:TsNoValido', ...
        'Ts debe ser un escalar real, finito y positivo.');
end
end