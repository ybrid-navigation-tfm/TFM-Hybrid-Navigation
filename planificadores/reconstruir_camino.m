function [camino, indicesCamino, info] = reconstruir_camino( ...
    nodos, padres, indiceFinal, indiceInicial)
% Reconstruye un camino a partir de la cadena de padres de un arbol.
%
%   camino = RECONSTRUIR_CAMINO(nodos,padres,indiceFinal)
%
%   camino = RECONSTRUIR_CAMINO( ...
%       nodos,padres,indiceFinal,indiceInicial)
%
%   [camino,indicesCamino,info] = RECONSTRUIR_CAMINO(...)
%
%   Recupera, en orden de recorrido, la rama que une un nodo inicial con
%   un nodo final dentro de un arbol representado mediante un vector de
%   padres.
%
%   En el proyecto TFM_RobotNavigation se utiliza principalmente despues
%   de que RRT* haya seleccionado el mejor nodo conectado con la meta. El
%   planificador conserva para cada nodo el indice de su padre, pero el
%   controlador necesita una matriz ordenada de posiciones:
%
%       raiz -> nodo 1 -> nodo 2 -> ... -> meta
%
%   Esta funcion realiza exclusivamente esa conversion. No planifica, no
%   comprueba colisiones, no simplifica la trayectoria y no modifica el
%   arbol recibido.
%
%   Entradas:
%       nodos
%           Matriz N x D, con D >= 2. Cada fila contiene las coordenadas o
%           variables asociadas a un nodo del arbol. En la implementacion
%           actual de RRT* se utiliza una matriz N x 2 con posiciones:
%
%               nodos(i,:) = [x_i y_i]
%
%           La funcion conserva todas las columnas de nodos en la salida,
%           aunque la longitud geometrica se calcula solo con [x y].
%
%       padres
%           Vector de N indices enteros. El elemento:
%
%               padres(i)
%
%           identifica al padre del nodo i. El valor 0 indica que el nodo
%           no tiene padre y actua como raiz de su componente.
%
%       indiceFinal
%           Indice del nodo desde el que comienza el recorrido inverso de
%           la cadena de padres. Normalmente corresponde a la meta.
%
%       indiceInicial
%           Indice opcional del nodo en el que debe detenerse la
%           reconstruccion. Debe ser un ancestro de indiceFinal.
%
%           Si se omite, la funcion continua hasta alcanzar el primer nodo
%           cuyo padre sea 0. De esta forma detecta automaticamente la
%           raiz de la rama.
%
%   Salidas:
%       camino
%           Matriz K x D con los nodos ordenados desde el inicio hasta el
%           final. Si K = 1, el camino contiene un unico nodo.
%
%       indicesCamino
%           Vector K x 1 con los indices correspondientes dentro de nodos.
%
%       info
%           Estructura de diagnostico:
%
%               .exito
%               .motivo
%               .indiceInicial
%               .indiceFinal
%               .raizEspecificada
%               .cadenaCompletaHastaRaiz
%               .indicesCamino
%               .numeroNodosCamino
%               .numeroAristasCamino
%               .longitudCamino
%               .longitudesSegmentos
%               .dimensionNodo
%               .numeroNodosArbol
%
%   Validaciones estructurales:
%       La funcion detecta y comunica mediante errores identificables:
%
%       - indices de padres fuera del intervalo [0,N];
%       - referencias de un nodo a si mismo;
%       - ciclos dentro de la cadena de padres;
%       - un indice inicial que no sea ancestro del nodo final;
%       - nodos o indices no validos.
%
%   Complejidad:
%       Solo se recorren los nodos pertenecientes a la rama final. La
%       complejidad temporal es O(K) y la memoria adicional es O(N) para
%       detectar ciclos de forma segura.
%
%   Ejemplo basico:
%
%       nodos = [ ...
%           0 0; ...     % nodo 1, raiz
%           1 0; ...     % nodo 2
%           1 1; ...     % nodo 3
%           2 1];        % nodo 4, meta
%
%       padres = [0;1;2;3];
%
%       [camino,indices,info] = reconstruir_camino( ...
%           nodos,padres,4);
%
%       % indices = [1;2;3;4]
%       % camino  = [0 0; 1 0; 1 1; 2 1]
%       % info.longitudCamino = 3
%
%   Ejemplo de subcamino:
%
%       caminoLocal = reconstruir_camino( ...
%           nodos,padres,4,2);
%
%       % caminoLocal = [1 0; 1 1; 2 1]
%
%   Uso previsto desde rrt_star.m:
%
%       [camino,indicesCamino,infoCamino] = reconstruir_camino( ...
%           nodos(1:numeroNodos,:), ...
%           padres(1:numeroNodos), ...
%           indiceMeta, ...
%           1);
%
%   Esta funcion no requiere otros modulos del proyecto.

if nargin < 4
    indiceInicial = [];
end

%% Validacion y normalizacion
[nodos, padres, indiceFinal, indiceInicial, raizEspecificada] = ...
    validar_entradas(nodos,padres,indiceFinal,indiceInicial,nargin);

numeroNodosArbol = size(nodos,1);
dimensionNodo = size(nodos,2);

%% Memoria para la cadena recorrida en sentido final -> inicio
indicesInversos = zeros(numeroNodosArbol,1);
visitados = false(numeroNodosArbol,1);

numeroIndices = 0;
indiceActual = indiceFinal;

%% Recorrido de la cadena de padres
while true
    if visitados(indiceActual)
        error('reconstruir_camino:CicloDetectado', ...
            ['La cadena de padres contiene un ciclo. El nodo %d fue ' ...
             'visitado mas de una vez durante la reconstruccion.'], ...
            indiceActual);
    end

    visitados(indiceActual) = true;
    numeroIndices = numeroIndices+1;
    indicesInversos(numeroIndices) = indiceActual;

    %% Detencion en el nodo inicial solicitado
    if raizEspecificada && indiceActual == indiceInicial
        break;
    end

    padreActual = padres(indiceActual);

    %% Raiz detectada automaticamente
    if padreActual == 0
        if raizEspecificada
            error('reconstruir_camino:IndiceInicialNoAlcanzado', ...
                ['El nodo inicial solicitado (%d) no pertenece a la ' ...
                 'cadena de ancestros del nodo final (%d). La rama ' ...
                 'termina en el nodo raiz %d.'], ...
                indiceInicial,indiceFinal,indiceActual);
        end

        indiceInicial = indiceActual;
        break;
    end

    indiceActual = padreActual;
end

%% Orden natural inicio -> final
indicesCamino = flipud(indicesInversos(1:numeroIndices));
camino = nodos(indicesCamino,:);

%% Longitud geometrica de la polilinea
if numeroIndices < 2
    longitudesSegmentos = zeros(0,1);
    longitudCamino = 0;
else
    desplazamientos = diff(camino(:,1:2),1,1);

    longitudesSegmentos = hypot( ...
        desplazamientos(:,1), ...
        desplazamientos(:,2));

    longitudCamino = sum(longitudesSegmentos);
end

%% Informacion de diagnostico
cadenaCompletaHastaRaiz = padres(indiceInicial) == 0;

info = struct();
info.exito = true;

if cadenaCompletaHastaRaiz
    info.motivo = "camino_reconstruido_hasta_raiz";
else
    info.motivo = "subcamino_reconstruido";
end

info.indiceInicial = indiceInicial;
info.indiceFinal = indiceFinal;
info.raizEspecificada = raizEspecificada;
info.cadenaCompletaHastaRaiz = cadenaCompletaHastaRaiz;

info.indicesCamino = indicesCamino;
info.numeroNodosCamino = numeroIndices;
info.numeroAristasCamino = max(0,numeroIndices-1);

info.longitudCamino = longitudCamino;
info.longitudesSegmentos = longitudesSegmentos;
info.unidadLongitud = "unidad_de_las_coordenadas";

info.dimensionNodo = dimensionNodo;
info.numeroNodosArbol = numeroNodosArbol;
end

%% ========================================================================
% VALIDACION
% ========================================================================

function [nodos,padres,indiceFinal,indiceInicial,raizEspecificada] = ...
    validar_entradas(nodos,padres,indiceFinal,indiceInicial,numeroEntradas)
%VALIDAR_ENTRADAS Comprueba la coherencia del arbol y de los indices.

%% Matriz de nodos
if ~isnumeric(nodos) || ~isreal(nodos) || ~ismatrix(nodos) || ...
        isempty(nodos) || size(nodos,2) < 2 || ...
        any(~isfinite(nodos(:)))
    error('reconstruir_camino:NodosNoValidos', ...
        ['nodos debe ser una matriz numerica real y finita N x D, ' ...
         'con N >= 1 y D >= 2.']);
end

nodos = double(nodos);
numeroNodos = size(nodos,1);

%% Vector de padres
if ~isnumeric(padres) || ~isreal(padres) || ...
        ~isvector(padres) || numel(padres) ~= numeroNodos || ...
        any(~isfinite(padres(:)))
    error('reconstruir_camino:PadresNoValidos', ...
        ['padres debe ser un vector numerico real y finito con un ' ...
         'elemento por cada fila de nodos.']);
end

padres = reshape(double(padres),[],1);

if any(padres ~= floor(padres)) || ...
        any(padres < 0) || any(padres > numeroNodos)
    error('reconstruir_camino:IndicesPadresFueraRango', ...
        ['Cada padre debe ser un entero del intervalo [0,N], donde ' ...
         '0 identifica un nodo sin padre.']);
end

indicesNodos = (1:numeroNodos)';

if any(padres == indicesNodos)
    indiceProblematico = find(padres == indicesNodos,1,'first');

    error('reconstruir_camino:Autorreferencia', ...
        'El nodo %d aparece definido como su propio padre.', ...
        indiceProblematico);
end

%% Indice final
indiceFinal = validar_indice( ...
    indiceFinal,numeroNodos,'indiceFinal');

%% Indice inicial opcional
raizEspecificada = numeroEntradas >= 4 && ~isempty(indiceInicial);

if raizEspecificada
    indiceInicial = validar_indice( ...
        indiceInicial,numeroNodos,'indiceInicial');
else
    indiceInicial = NaN;
end
end

function indice = validar_indice(indice,numeroNodos,nombre)
%VALIDAR_INDICE Comprueba un indice escalar dentro del arbol.

if ~isnumeric(indice) || ~isreal(indice) || ~isscalar(indice) || ...
        ~isfinite(indice) || indice ~= floor(indice) || ...
        indice < 1 || indice > numeroNodos
    error('reconstruir_camino:IndiceNoValido', ...
        '%s debe ser un entero del intervalo [1,%d].', ...
        nombre,numeroNodos);
end

indice = double(indice);
end
