function tf = segmentos_intersectan(a, b, c, d)
% Detecta interseccion entre dos segmentos 2D.
%
%   tf = SEGMENTOS_INTERSECTAN(a,b,c,d)
%
%   Determina si los segmentos cerrados:
%
%       segmento 1: a ---- b
%       segmento 2: c ---- d
%
%   se cruzan, se tocan o se solapan.
%
%   Entradas:
%       a, b : extremos del primer segmento [x y]
%       c, d : extremos del segundo segmento [x y]
%
%   Salida:
%       tf   : true si existe interseccion, contacto o solapamiento;
%              false en caso contrario
%
%   Tambien contempla:
%       - segmentos colineales;
%       - contacto en un extremo;
%       - solapamiento parcial o total;
%       - segmentos degenerados reducidos a un punto.
%
%   Ejemplos:
%
%       % Cruce
%       tf = segmentos_intersectan([0 0],[2 2],[0 2],[2 0])
%       % true
%
%       % Segmentos separados
%       tf = segmentos_intersectan([0 0],[1 0],[0 1],[1 1])
%       % false
%
%       % Contacto en un extremo
%       tf = segmentos_intersectan([0 0],[1 1],[1 1],[2 0])
%       % true
%
%       % Solapamiento colineal
%       tf = segmentos_intersectan([0 0],[3 0],[1 0],[2 0])
%       % true
%
%   Esta funcion utiliza:
%       - producto_cruzado_2d.m
%       - punto_en_segmento.m

%% Validacion y normalizacion
a = validar_punto(a, "a");
b = validar_punto(b, "b");
c = validar_punto(c, "c");
d = validar_punto(d, "d");

%% Tolerancia numerica adaptada a la escala de las coordenadas
escala = max(1,max(abs([a b c d])));
tol = 1e-12*escala;

%% Orientaciones relativas
o1 = producto_cruzado_2d(b-a,c-a);
o2 = producto_cruzado_2d(b-a,d-a);
o3 = producto_cruzado_2d(d-c,a-c);
o4 = producto_cruzado_2d(d-c,b-c);

%% Cruce propio
tf = ((o1 > tol && o2 < -tol) || (o1 < -tol && o2 > tol)) && ...
     ((o3 > tol && o4 < -tol) || (o3 < -tol && o4 > tol));

if tf
    return;
end

%% Casos colineales, contacto y solapamiento
tf = ...
    (abs(o1) <= tol && punto_en_segmento(c,a,b,tol)) || ...
    (abs(o2) <= tol && punto_en_segmento(d,a,b,tol)) || ...
    (abs(o3) <= tol && punto_en_segmento(a,c,d,tol)) || ...
    (abs(o4) <= tol && punto_en_segmento(b,c,d,tol));
end

%% ========================================================================
% FUNCION LOCAL
% ========================================================================

function p = validar_punto(p,nombre)
%VALIDAR_PUNTO Comprueba y convierte un punto a formato fila 1 x 2.

if ~isnumeric(p) || ~isreal(p) || numel(p) ~= 2 || ...
        any(~isfinite(p(:)))
    error('segmentos_intersectan:PuntoNoValido', ...
        ['El punto %s debe ser un vector numerico real, finito ' ...
         'y de dos componentes.'], nombre);
end

p = reshape(double(p),1,2);
end
