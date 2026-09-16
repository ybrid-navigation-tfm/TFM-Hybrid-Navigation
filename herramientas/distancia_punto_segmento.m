function d = distancia_punto_segmento(p, a, b)
% Distancia minima de un punto a un segmento 2D.
%
%   d = DISTANCIA_PUNTO_SEGMENTO(p,a,b)
%
%   Calcula la distancia euclidiana minima entre el punto p y el segmento
%   cerrado definido por los extremos a y b.
%
%   Entradas:
%       p : punto de consulta [x y]
%       a : primer extremo del segmento [x y]
%       b : segundo extremo del segmento [x y]
%
%   Salida:
%       d : distancia minima no negativa, en las mismas unidades que las
%           coordenadas de entrada
%
%   La proyeccion ortogonal del punto sobre la recta se limita al intervalo
%   del segmento. Por tanto:
%
%       - si la proyeccion cae entre a y b, se mide hasta la proyeccion;
%       - si cae fuera, se mide hasta el extremo mas cercano;
%       - si a y b coinciden, el segmento se trata como un unico punto.
%
%   Ejemplos:
%
%       % Proyeccion interior
%       d = distancia_punto_segmento([1 1],[0 0],[2 0])
%       % d = 1
%
%       % Proyeccion exterior: el extremo mas cercano es b
%       d = distancia_punto_segmento([3 1],[0 0],[2 0])
%       % d = sqrt(2)
%
%       % Segmento degenerado
%       d = distancia_punto_segmento([2 2],[1 1],[1 1])
%       % d = sqrt(2)
%
%   Esta funcion es una primitiva geometrica compartida por:
%       - distancia_segmentos.m
%       - segmento_rectangulo.m
%       - deteccion de colisiones
%       - RRT* y PRM

%% Validacion y normalizacion
p = validar_punto(p, "p");
a = validar_punto(a, "a");
b = validar_punto(b, "b");

%% Vector del segmento
ab = b - a;
den = dot(ab,ab);

%% Segmento degenerado
escala = max(1,max(abs([a b])));

if den <= eps(escala)
    d = norm(p-a);
    return;
end

%% Proyeccion normalizada sobre el segmento
t = dot(p-a,ab)/den;
t = min(max(t,0),1);

proyeccion = a + t*ab;
d = norm(p-proyeccion);

% Proteccion frente a residuos numericos muy pequenos.
if d < 1e-12
    d = 0;
end
end

%% ========================================================================
% FUNCION LOCAL
% ========================================================================

function p = validar_punto(p,nombre)
%VALIDAR_PUNTO Comprueba y convierte un punto a formato fila 1 x 2.

if ~isnumeric(p) || ~isreal(p) || numel(p) ~= 2 || ...
        any(~isfinite(p(:)))
    error('distancia_punto_segmento:PuntoNoValido', ...
        ['El punto %s debe ser un vector numerico real, finito ' ...
         'y de dos componentes.'], nombre);
end

p = reshape(double(p),1,2);
end
