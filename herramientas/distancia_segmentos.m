function d = distancia_segmentos(a, b, c, f)
% Distancia minima entre dos segmentos 2D.
%
%   d = DISTANCIA_SEGMENTOS(a,b,c,f)
%
%   Calcula la distancia euclidiana minima entre los segmentos:
%
%       segmento 1: a ---- b
%       segmento 2: c ---- f
%
%   Cada punto debe proporcionarse como un vector numerico de dos
%   componentes:
%
%       a = [x1 y1]
%       b = [x2 y2]
%       c = [x3 y3]
%       f = [x4 y4]
%
%   La salida d es un escalar no negativo expresado en las mismas unidades
%   que las coordenadas de entrada.
%
%   Si los segmentos se cruzan, se tocan o se solapan, la distancia es 0.
%   Tambien se contemplan segmentos degenerados, es decir, aquellos cuyos
%   extremos coinciden y que por tanto representan un unico punto.
%
%   Ejemplos:
%
%       % Segmentos paralelos separados una unidad
%       d = distancia_segmentos([0 0],[2 0],[0 1],[2 1])
%       % d = 1
%
%       % Segmentos que se cruzan
%       d = distancia_segmentos([0 0],[2 2],[0 2],[2 0])
%       % d = 0
%
%       % Un segmento degenerado convertido en punto
%       d = distancia_segmentos([1 1],[1 1],[0 0],[2 0])
%       % d = 1
%
%   Esta funcion se utiliza en las comprobaciones geometricas compartidas
%   por RRT*, PRM, deteccion de colisiones y prediccion de obstaculos
%   dinamicos.

%% Validacion de entradas
a = validar_punto(a, "a");
b = validar_punto(b, "b");
c = validar_punto(c, "c");
f = validar_punto(f, "f");

%% Interseccion o contacto entre segmentos
if segmentos_intersectan(a,b,c,f)
    d = 0;
    return;
end

%% Distancia minima entre extremos y segmento opuesto
d = min([ ...
    distancia_punto_segmento(a,c,f), ...
    distancia_punto_segmento(b,c,f), ...
    distancia_punto_segmento(c,a,b), ...
    distancia_punto_segmento(f,a,b)]);

% Proteccion frente a errores numericos muy pequenos.
if d < 1e-12
    d = 0;
end
end

%% ========================================================================
% FUNCIONES LOCALES
% ========================================================================

function p = validar_punto(p,nombre)
%VALIDAR_PUNTO Comprueba y normaliza un punto bidimensional.

if ~isnumeric(p) || ~isreal(p) || numel(p) ~= 2 || ...
        any(~isfinite(p(:)))
    error('distancia_segmentos:PuntoNoValido', ...
        'El punto %s debe ser un vector numerico real y finito de dos componentes.', ...
        nombre);
end

p = reshape(double(p),1,2);
end

function d = distancia_punto_segmento(p,a,b)
%DISTANCIA_PUNTO_SEGMENTO Distancia de un punto a un segmento cerrado.

ab = b-a;
den = dot(ab,ab);

% Segmento degenerado: a y b representan el mismo punto.
if den <= eps(max(1,max(abs([a b]))))
    d = norm(p-a);
    return;
end

t = dot(p-a,ab)/den;
t = min(max(t,0),1);

proyeccion = a+t*ab;
d = norm(p-proyeccion);
end

function tf = segmentos_intersectan(a,b,c,d)
%SEGMENTOS_INTERSECTAN Detecta cruce, contacto o solapamiento.

tol = 1e-12 * max(1,max(abs([a b c d])));

o1 = producto_cruzado_2d(b-a,c-a);
o2 = producto_cruzado_2d(b-a,d-a);
o3 = producto_cruzado_2d(d-c,a-c);
o4 = producto_cruzado_2d(d-c,b-c);

% Cruce propio.
tf = ((o1 > tol && o2 < -tol) || (o1 < -tol && o2 > tol)) && ...
     ((o3 > tol && o4 < -tol) || (o3 < -tol && o4 > tol));

if tf
    return;
end

% Contacto o solapamiento colineal.
tf = ...
    (abs(o1) <= tol && punto_en_segmento(c,a,b,tol)) || ...
    (abs(o2) <= tol && punto_en_segmento(d,a,b,tol)) || ...
    (abs(o3) <= tol && punto_en_segmento(a,c,d,tol)) || ...
    (abs(o4) <= tol && punto_en_segmento(b,c,d,tol));
end

function tf = punto_en_segmento(p,a,b,tol)
%PUNTO_EN_SEGMENTO Comprueba si p pertenece al rectangulo envolvente de ab.

tf = p(1) >= min(a(1),b(1))-tol && ...
     p(1) <= max(a(1),b(1))+tol && ...
     p(2) >= min(a(2),b(2))-tol && ...
     p(2) <= max(a(2),b(2))+tol;
end

function z = producto_cruzado_2d(a,b)
%PRODUCTO_CRUZADO_2D Componente z del producto vectorial de dos vectores 2D.

z = a(1)*b(2)-a(2)*b(1);
end
