function z = producto_cruzado_2d(a, b)
% Calcula el producto cruzado escalar de dos vectores 2D.
%
%   z = PRODUCTO_CRUZADO_2D(a,b)
%
%   Calcula la componente z del producto vectorial entre dos vectores
%   bidimensionales:
%
%       a = [ax ay]
%       b = [bx by]
%
%   El resultado es:
%
%       z = ax*by - ay*bx
%
%   Aunque los vectores sean 2D, puede interpretarse que ambos pertenecen
%   al plano z = 0. Su producto vectorial es perpendicular al plano y solo
%   tiene componente z.
%
%   Interpretacion geometrica:
%
%       z > 0  -> b queda a la izquierda de a
%       z < 0  -> b queda a la derecha de a
%       z = 0  -> a y b son paralelos o colineales
%
%   Entradas:
%       a : vector numerico real de dos componentes [ax ay]
%       b : vector numerico real de dos componentes [bx by]
%
%   Salida:
%       z : escalar real
%
%   Ejemplos:
%
%       z = producto_cruzado_2d([1 0],[0 1])
%       % z = 1
%
%       z = producto_cruzado_2d([0 1],[1 0])
%       % z = -1
%
%       z = producto_cruzado_2d([1 1],[2 2])
%       % z = 0
%
%   Esta funcion se utiliza principalmente en:
%       - segmentos_intersectan.m
%       - comprobaciones de orientacion
%       - deteccion de colinealidad

%% Validacion y normalizacion
a = validar_vector_2d(a, "a");
b = validar_vector_2d(b, "b");

%% Producto cruzado 2D
z = a(1)*b(2) - a(2)*b(1);

% Proteccion frente a residuos numericos extremadamente pequenos.
escala = max(1,max(abs([a b])));
if abs(z) < 1e-15*escala^2
    z = 0;
end
end

%% ========================================================================
% FUNCION LOCAL
% ========================================================================

function v = validar_vector_2d(v,nombre)
%VALIDAR_VECTOR_2D Comprueba y convierte un vector a formato fila 1 x 2.

if ~isnumeric(v) || ~isreal(v) || numel(v) ~= 2 || ...
        any(~isfinite(v(:)))
    error('producto_cruzado_2d:VectorNoValido', ...
        ['El vector %s debe ser numerico, real, finito ' ...
         'y de dos componentes.'], nombre);
end

v = reshape(double(v),1,2);
end
