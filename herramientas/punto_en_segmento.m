function tf = punto_en_segmento(p, a, b, tolerancia)
% Comprueba si un punto pertenece a un segmento 2D.
%
%   tf = PUNTO_EN_SEGMENTO(p,a,b)
%   tf = PUNTO_EN_SEGMENTO(p,a,b,tolerancia)
%
%   Determina si el punto p pertenece al segmento cerrado definido por
%   los extremos a y b.
%
%   La función asume que previamente ya se ha comprobado que los puntos
%   son colineales. Su única misión es verificar que p queda dentro del
%   rectángulo envolvente del segmento.
%
%   Entradas:
%       p          : punto de consulta [x y]
%       a          : primer extremo [x y]
%       b          : segundo extremo [x y]
%       tolerancia : tolerancia numérica (opcional)
%
%   Salida:
%       tf         : true si p pertenece al segmento
%                    false en caso contrario
%
%   Ejemplos:
%
%       punto_en_segmento([1 0],[0 0],[2 0])
%       % true
%
%       punto_en_segmento([3 0],[0 0],[2 0])
%       % false
%
%       punto_en_segmento([2 2],[0 0],[4 4])
%       % true
%
%   Esta función se utiliza en:
%       - segmentos_intersectan.m
%       - segmento_rectangulo.m

%% Tolerancia

if nargin < 4 || isempty(tolerancia)
    escala = max(1,max(abs([a b p])));
    tolerancia = 1e-12*escala;
end

%% Validación

p = validar_punto(p,"p");
a = validar_punto(a,"a");
b = validar_punto(b,"b");

%% Comprobación

tf = ...
    p(1) >= min(a(1),b(1)) - tolerancia && ...
    p(1) <= max(a(1),b(1)) + tolerancia && ...
    p(2) >= min(a(2),b(2)) - tolerancia && ...
    p(2) <= max(a(2),b(2)) + tolerancia;

end

%% ========================================================================
% FUNCIONES LOCALES
% ========================================================================

function p = validar_punto(p,nombre)

if ~isnumeric(p) || ~isreal(p) || numel(p) ~= 2 || ...
        any(~isfinite(p(:)))
    error('punto_en_segmento:PuntoNoValido', ...
        'El punto %s debe ser un vector numérico real, finito y de dos componentes.', ...
        nombre);
end

p = reshape(double(p),1,2);

end