function [longitudTotal, longitudesSegmentos] = longitud(trayectoria)
% Calcula la longitud total de una trayectoria bidimensional.
%
%   longitudTotal = LONGITUD(trayectoria)
%
%   [longitudTotal,longitudesSegmentos] = LONGITUD(trayectoria)
%
%   Calcula la distancia total recorrida como la suma de las distancias
%   euclidianas entre posiciones consecutivas:
%
%       L = sum_{k=2}^{N} sqrt( ...
%           (x(k)-x(k-1))^2 + (y(k)-y(k-1))^2 )
%
%   Entrada:
%       trayectoria
%           Matriz N x M con M >= 2. Cada fila representa una muestra del
%           estado del robot. Para el calculo se utilizan exclusivamente
%           las dos primeras columnas:
%
%               trayectoria(:,1) = x
%               trayectoria(:,2) = y
%
%           Por tanto, se admiten tanto trayectorias N x 2 como historiales
%           de estado N x 3 con formato [x y theta].
%
%   Salidas:
%       longitudTotal
%           Longitud total recorrida. Se expresa en las mismas unidades
%           que las coordenadas; en este proyecto, metros.
%
%       longitudesSegmentos
%           Vector (N-1) x 1 con la longitud de cada desplazamiento entre
%           dos muestras consecutivas.
%
%   Casos particulares:
%       - trayectoria vacia       -> longitudTotal = 0
%       - una sola posicion       -> longitudTotal = 0
%       - posiciones repetidas    -> aportan longitud cero
%
%   IMPORTANTE:
%   Para la metrica experimental debe proporcionarse la trayectoria
%   EJECUTADA por el robot, no la trayectoria global planificada. De este
%   modo se contabilizan las desviaciones locales, giros y replanteamientos
%   producidos durante la navegacion.
%
%   Ejemplo:
%
%       trayectoria = [
%           0 0 0;
%           3 0 0;
%           3 4 pi/2
%       ];
%
%       [L,segmentos] = longitud(trayectoria);
%
%       % L = 7
%       % segmentos = [3; 4]
%
%   Esta funcion no requiere otras funciones del proyecto.

%% Validacion
if ~isnumeric(trayectoria) || ~isreal(trayectoria)
    error('longitud:TrayectoriaNoValida', ...
        'La trayectoria debe ser una matriz numerica real.');
end

if isempty(trayectoria)
    longitudTotal = 0;
    longitudesSegmentos = zeros(0,1);
    return;
end

if ~ismatrix(trayectoria) || size(trayectoria,2) < 2
    error('longitud:DimensionNoValida', ...
        ['La trayectoria debe tener formato N x M con M >= 2; ' ...
         'las dos primeras columnas deben ser [x y].']);
end

if any(~isfinite(trayectoria(:)))
    error('longitud:ValoresNoFinitos', ...
        ['La trayectoria contiene NaN o Inf. Recorte primero las filas ' ...
         'no utilizadas del historial preasignado.']);
end

trayectoria = double(trayectoria);

%% Trayectoria con una sola posicion
if size(trayectoria,1) < 2
    longitudTotal = 0;
    longitudesSegmentos = zeros(0,1);
    return;
end

%% Distancias euclidianas entre posiciones consecutivas
posiciones = trayectoria(:,1:2);
desplazamientos = diff(posiciones,1,1);

longitudesSegmentos = hypot( ...
    desplazamientos(:,1), ...
    desplazamientos(:,2));

longitudTotal = sum(longitudesSegmentos);
end
