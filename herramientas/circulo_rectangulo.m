function [colision, normal, distancia] = circulo_rectangulo(centro, radio, rectangulo, tolerancia)
% Detecta colision entre un circulo y un rectangulo 2D.
%
%   colision = CIRCULO_RECTANGULO(centro,radio,rectangulo)
%
%   [colision,normal,distancia] = ...
%       CIRCULO_RECTANGULO(centro,radio,rectangulo,tolerancia)
%
%   Comprueba si un circulo intersecta, toca o se encuentra dentro de un
%   rectangulo alineado con los ejes.
%
%   Entradas:
%       centro      : centro del circulo [x y]
%       radio       : radio del circulo
%       rectangulo  : [x y ancho alto]
%       tolerancia  : tolerancia numerica opcional
%
%   Salidas:
%       colision    : true si existe contacto o solapamiento
%       normal      : normal unitaria de contacto orientada desde el
%                     rectangulo hacia el centro del circulo
%       distancia   : distancia euclidiana desde el centro del circulo
%                     hasta el rectangulo; vale 0 si el centro esta dentro
%
%   Convencion del rectangulo:
%
%       x, y        : esquina inferior izquierda
%       ancho, alto : dimensiones positivas
%
%   La colision se determina mediante el punto del rectangulo mas cercano
%   al centro del circulo. Existe colision cuando:
%
%       distancia <= radio + tolerancia
%
%   Si el centro esta dentro del rectangulo, la normal se obtiene a partir
%   del lado mas cercano. Esto permite utilizar la funcion en el rebote de
%   obstaculos dinamicos.
%
%   Ejemplos:
%
%       r = [1 1 2 2];
%
%       % Circulo separado
%       tf = circulo_rectangulo([0 0],0.5,r)
%       % false
%
%       % Circulo tocando el lado izquierdo
%       [tf,n,d] = circulo_rectangulo([0.5 2],0.5,r)
%       % tf = true, n = [-1 0], d = 0.5
%
%       % Centro dentro del rectangulo
%       [tf,n,d] = circulo_rectangulo([2 2],0.25,r)
%       % tf = true, d = 0
%
%   Esta funcion se utiliza en:
%       - actualizar_obstaculos.m
%       - detectar_colisiones.m
%       - calculo de rebotes contra obstaculos estaticos

%% Validacion y normalizacion
centro = validar_punto(centro);
radio = validar_radio(radio);
rectangulo = validar_rectangulo(rectangulo);

if nargin < 4 || isempty(tolerancia)
    escala = max(1,max(abs([centro rectangulo radio])));
    tolerancia = 1e-12*escala;
elseif ~isnumeric(tolerancia) || ~isscalar(tolerancia) || ...
        ~isreal(tolerancia) || ~isfinite(tolerancia) || tolerancia < 0
    error('circulo_rectangulo:ToleranciaNoValida', ...
        'La tolerancia debe ser un escalar numerico real, finito y no negativo.');
end

%% Limites del rectangulo
xmin = rectangulo(1);
ymin = rectangulo(2);
xmax = xmin + rectangulo(3);
ymax = ymin + rectangulo(4);

%% Punto del rectangulo mas cercano al centro
puntoCercano = [
    min(max(centro(1),xmin),xmax), ...
    min(max(centro(2),ymin),ymax)
];

vectorContacto = centro - puntoCercano;
distancia = norm(vectorContacto);

%% Deteccion de colision
colision = distancia <= radio + tolerancia;

%% Normal de contacto
if distancia > tolerancia
    % Centro exterior: normal desde el rectangulo hacia el circulo.
    normal = vectorContacto/distancia;
else
    % Centro dentro o exactamente sobre el contorno. Se selecciona el lado
    % mas cercano y su normal exterior.
    distIzquierda = abs(centro(1)-xmin);
    distDerecha   = abs(xmax-centro(1));
    distInferior  = abs(centro(2)-ymin);
    distSuperior  = abs(ymax-centro(2));

    [~,indice] = min([ ...
        distIzquierda, ...
        distDerecha, ...
        distInferior, ...
        distSuperior]);

    switch indice
        case 1
            normal = [-1 0];
        case 2
            normal = [1 0];
        case 3
            normal = [0 -1];
        otherwise
            normal = [0 1];
    end
end

% Evita residuos numericos muy pequenos en la distancia.
if distancia < tolerancia
    distancia = 0;
end
end

%% ========================================================================
% FUNCIONES LOCALES
% ========================================================================

function p = validar_punto(p)
%VALIDAR_PUNTO Comprueba y convierte el centro a formato fila 1 x 2.

if ~isnumeric(p) || ~isreal(p) || numel(p) ~= 2 || ...
        any(~isfinite(p(:)))
    error('circulo_rectangulo:CentroNoValido', ...
        ['El centro debe ser un vector numerico real, finito ' ...
         'y de dos componentes.']);
end

p = reshape(double(p),1,2);
end

function r = validar_radio(r)
%VALIDAR_RADIO Comprueba que el radio sea valido.

if ~isnumeric(r) || ~isscalar(r) || ~isreal(r) || ...
        ~isfinite(r) || r < 0
    error('circulo_rectangulo:RadioNoValido', ...
        'El radio debe ser un escalar numerico real, finito y no negativo.');
end

r = double(r);
end

function r = validar_rectangulo(r)
%VALIDAR_RECTANGULO Comprueba el formato [x y ancho alto].

if ~isnumeric(r) || ~isreal(r) || numel(r) ~= 4 || ...
        any(~isfinite(r(:)))
    error('circulo_rectangulo:RectanguloNoValido', ...
        ['El rectangulo debe ser un vector numerico real y finito ' ...
         'con formato [x y ancho alto].']);
end

r = reshape(double(r),1,4);

if r(3) <= 0 || r(4) <= 0
    error('circulo_rectangulo:DimensionesNoValidas', ...
        'El ancho y el alto del rectangulo deben ser positivos.');
end
end
