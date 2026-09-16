function [colision, normal, distancia] = ...
    circulo_circulo(centro1, radio1, centro2, radio2, tolerancia)
% Detecta colisión entre dos círculos en 2D.
%
%   colision = CIRCULO_CIRCULO(c1,r1,c2,r2)
%
%   [colision,normal,distancia] = ...
%       CIRCULO_CIRCULO(c1,r1,c2,r2,tolerancia)
%
%   Comprueba si dos círculos se cortan, se tocan o se solapan.
%
%   Entradas:
%       centro1     : centro del primer círculo [x y]
%       radio1      : radio del primer círculo
%       centro2     : centro del segundo círculo [x y]
%       radio2      : radio del segundo círculo
%       tolerancia  : tolerancia numérica opcional
%
%   Salidas:
%       colision    : true si existe contacto o solapamiento
%       normal      : vector unitario desde el círculo 2 hacia el círculo 1
%       distancia   : distancia entre los centros
%
%   Existe colisión cuando:
%
%       distancia <= radio1 + radio2 + tolerancia
%
%   Esta función se utiliza en:
%       - actualizar_obstaculos.m
%       - detectar_colisiones.m
%       - rebote entre obstáculos dinámicos

%% Validación

centro1 = validar_punto(centro1);
centro2 = validar_punto(centro2);

radio1 = validar_radio(radio1);
radio2 = validar_radio(radio2);

if nargin < 5 || isempty(tolerancia)

    escala = max(1,max(abs([centro1 centro2 radio1 radio2])));
    tolerancia = 1e-12*escala;

elseif ~isnumeric(tolerancia) || ...
        ~isscalar(tolerancia) || ...
        ~isreal(tolerancia) || ...
        ~isfinite(tolerancia) || ...
        tolerancia < 0

    error('circulo_circulo:ToleranciaNoValida', ...
        'La tolerancia debe ser un escalar numérico real, finito y no negativo.');

end

%% Distancia entre centros

vector = centro1 - centro2;
distancia = norm(vector);

%% Colisión

colision = distancia <= radio1 + radio2 + tolerancia;

%% Normal de contacto

if distancia > tolerancia

    normal = vector/distancia;

else

    % Ambos centros coinciden exactamente.
    normal = [1 0];

end

if distancia < tolerancia
    distancia = 0;
end

end

%% ========================================================================
% FUNCIONES LOCALES
% ========================================================================

function p = validar_punto(p)

if ~isnumeric(p) || ...
        ~isreal(p) || ...
        numel(p) ~= 2 || ...
        any(~isfinite(p(:)))

    error('circulo_circulo:CentroNoValido', ...
        'El centro debe ser un vector numérico real, finito y de dos componentes.');

end

p = reshape(double(p),1,2);

end

function r = validar_radio(r)

if ~isnumeric(r) || ...
        ~isscalar(r) || ...
        ~isreal(r) || ...
        ~isfinite(r) || ...
        r < 0

    error('circulo_circulo:RadioNoValido', ...
        'El radio debe ser un escalar numérico real, finito y no negativo.');

end

r = double(r);

end