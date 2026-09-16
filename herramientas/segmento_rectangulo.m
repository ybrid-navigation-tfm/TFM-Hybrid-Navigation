function tf = segmento_rectangulo(a, b, rectangulo, tolerancia)
% Detecta si un segmento intersecta un rectangulo 2D.
%
%   tf = SEGMENTO_RECTANGULO(a,b,rectangulo)
%   tf = SEGMENTO_RECTANGULO(a,b,rectangulo,tolerancia)
%
%   Determina si el segmento cerrado definido por los extremos a y b
%   intersecta, toca o atraviesa un rectangulo alineado con los ejes.
%
%   Entradas:
%       a, b        : extremos del segmento [x y]
%       rectangulo  : [x y ancho alto]
%       tolerancia  : tolerancia numerica opcional
%
%   Salida:
%       tf          : true si existe interseccion o contacto;
%                     false en caso contrario
%
%   Convencion del rectangulo:
%
%       x, y        : esquina inferior izquierda
%       ancho, alto : dimensiones positivas
%
%   Se considera interseccion cuando:
%       - alguno de los extremos esta dentro del rectangulo;
%       - el segmento cruza cualquiera de sus cuatro lados;
%       - el segmento toca un lado o una esquina;
%       - el segmento se solapa con uno de los lados.
%
%   Ejemplos:
%
%       r = [1 1 2 2];
%
%       tf = segmento_rectangulo([0 2],[4 2],r)
%       % true
%
%       tf = segmento_rectangulo([0 0],[0.5 0.5],r)
%       % false
%
%       tf = segmento_rectangulo([1 1],[3 1],r)
%       % true
%
%   Esta funcion utiliza:
%       - segmentos_intersectan.m

%% Validacion
a = validar_punto(a, "a");
b = validar_punto(b, "b");
rectangulo = validar_rectangulo(rectangulo);

if nargin < 4 || isempty(tolerancia)
    escala = max(1,max(abs([a b rectangulo])));
    tolerancia = 1e-12*escala;
elseif ~isnumeric(tolerancia) || ~isscalar(tolerancia) || ...
        ~isreal(tolerancia) || ~isfinite(tolerancia) || tolerancia < 0
    error('segmento_rectangulo:ToleranciaNoValida', ...
        'La tolerancia debe ser un escalar numerico real, finito y no negativo.');
end

%% Limites del rectangulo
xmin = rectangulo(1);
ymin = rectangulo(2);
xmax = xmin + rectangulo(3);
ymax = ymin + rectangulo(4);

%% Extremos dentro o sobre el rectangulo
if punto_en_rectangulo(a,xmin,xmax,ymin,ymax,tolerancia) || ...
        punto_en_rectangulo(b,xmin,xmax,ymin,ymax,tolerancia)
    tf = true;
    return;
end

%% Esquinas y lados
esquinas = [
    xmin ymin;
    xmax ymin;
    xmax ymax;
    xmin ymax
];

lados = [
    1 2;
    2 3;
    3 4;
    4 1
];

%% Interseccion con cualquiera de los cuatro lados
tf = false;

for i = 1:4
    c = esquinas(lados(i,1),:);
    d = esquinas(lados(i,2),:);

    if segmentos_intersectan(a,b,c,d)
        tf = true;
        return;
    end
end
end

%% ========================================================================
% FUNCIONES LOCALES
% ========================================================================

function p = validar_punto(p,nombre)
%VALIDAR_PUNTO Comprueba y convierte un punto a formato fila 1 x 2.

if ~isnumeric(p) || ~isreal(p) || numel(p) ~= 2 || ...
        any(~isfinite(p(:)))
    error('segmento_rectangulo:PuntoNoValido', ...
        ['El punto %s debe ser un vector numerico real, finito ' ...
         'y de dos componentes.'], nombre);
end

p = reshape(double(p),1,2);
end

function r = validar_rectangulo(r)
%VALIDAR_RECTANGULO Comprueba el formato [x y ancho alto].

if ~isnumeric(r) || ~isreal(r) || numel(r) ~= 4 || ...
        any(~isfinite(r(:)))
    error('segmento_rectangulo:RectanguloNoValido', ...
        ['El rectangulo debe ser un vector numerico real y finito ' ...
         'con formato [x y ancho alto].']);
end

r = reshape(double(r),1,4);

if r(3) <= 0 || r(4) <= 0
    error('segmento_rectangulo:DimensionesNoValidas', ...
        'El ancho y el alto del rectangulo deben ser positivos.');
end
end

function tf = punto_en_rectangulo(p,xmin,xmax,ymin,ymax,tol)
%PUNTO_EN_RECTANGULO Comprueba inclusion, incluyendo el contorno.

tf = p(1) >= xmin-tol && p(1) <= xmax+tol && ...
     p(2) >= ymin-tol && p(2) <= ymax+tol;
end
