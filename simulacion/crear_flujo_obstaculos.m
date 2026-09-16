function [flujoObstaculos, semillaObstaculos, info] = ...
    crear_flujo_obstaculos(modo, semillaFija)
%CREAR_FLUJO_OBSTACULOS Crea un flujo aleatorio exclusivo para obstaculos.
%
%   flujoObstaculos = CREAR_FLUJO_OBSTACULOS()
%
%   [flujoObstaculos,semillaObstaculos] = ...
%       CREAR_FLUJO_OBSTACULOS(modo)
%
%   [flujoObstaculos,semillaObstaculos,info] = ...
%       CREAR_FLUJO_OBSTACULOS(modo,semillaFija)
%
%   Crea un objeto RandStream independiente del generador global utilizado
%   por los planificadores RRT* y PRM.
%
%   Esta funcion NO llama a rng y, por tanto, no modifica la semilla ni el
%   estado del generador pseudoaleatorio del planificador.
%
%   Modos disponibles:
%
%       "nueva"
%           Genera una realizacion nueva para los obstaculos mediante una
%           semilla basada en el tiempo actual.
%
%       "reproducible"
%           Utiliza la semilla numerica proporcionada en semillaFija.
%           Permite repetir exactamente una realizacion anterior de los
%           obstaculos.
%
%   Entradas:
%
%       modo
%           "nueva" o "reproducible".
%           Si se omite, se utiliza "nueva".
%
%       semillaFija
%           Entero entre 0 y 2^32-1.
%           Solo es obligatorio cuando modo = "reproducible".
%
%   Salidas:
%
%       flujoObstaculos
%           Objeto RandStream que debe pasarse tanto a
%           inicializar_obstaculos_aleatorios.m como a
%           actualizar_obstaculos_aleatorios.m durante la misma ejecucion.
%
%       semillaObstaculos
%           Semilla utilizada para crear el flujo. Se devuelve para poder
%           registrarla junto con los resultados de la simulacion.
%
%       info
%           Informacion basica sobre el flujo creado.
%
%   Ejemplos:
%
%       % Nueva realizacion de los obstaculos
%       [flujo,semilla] = crear_flujo_obstaculos("nueva");
%
%       % Reproducir una realizacion concreta
%       [flujo,semilla] = ...
%           crear_flujo_obstaculos("reproducible",100017);
%
%   IMPORTANTE:
%
%       La semilla fija del planificador debe seguir configurandose
%       independientemente en el main:
%
%           rng(cfg.semilla,'twister');
%
%       Esta funcion no debe recibir cfg.semilla, porque la aleatoriedad
%       de los obstaculos y la del planificador pertenecen a generadores
%       separados.

%% Valores por defecto
if nargin < 1 || isempty(modo)
    modo = "nueva";
end

if nargin < 2
    semillaFija = [];
end

% Validacion del modo
modo = string(modo);

if ~isscalar(modo)
    error('crear_flujo_obstaculos:ModoNoEscalar', ...
        'modo debe ser un texto escalar.');
end

modo = lower(strtrim(modo));

if strlength(modo) == 0
    modo = "nueva";
end

% Creacion del flujo independiente
switch modo
    case "nueva"
        % Seed='shuffle' se aplica exclusivamente a este objeto RandStream.
        % No se modifica el generador global empleado por RRT* o PRM.
        flujoObstaculos = RandStream( ...
            'mt19937ar', ...
            'Seed','shuffle');

        semillaObstaculos = ...
            double(flujoObstaculos.Seed);

    case "reproducible"
        semillaObstaculos = ...
            validar_semilla(semillaFija);

        flujoObstaculos = RandStream( ...
            'mt19937ar', ...
            'Seed',semillaObstaculos);

    otherwise
        error('crear_flujo_obstaculos:ModoNoValido', ...
            ['modo debe ser "nueva" o "reproducible". ' ...
             'Se recibio: %s.'],modo);
end

% Informacion de salida
info = struct();
info.exito = true;
info.modo = modo;
info.semilla = semillaObstaculos;
info.tipoGenerador = string(flujoObstaculos.Type);
info.reproducible = modo == "reproducible";
info.independienteDelGeneradorGlobal = true;
info.modificaSemillaPlanificador = false;
end


function semilla = validar_semilla(semilla)
%VALIDAR_SEMILLA Comprueba una semilla valida para RandStream.

limiteSuperior = double(intmax('uint32'));

if ~isnumeric(semilla) || ...
        ~isreal(semilla) || ...
        ~isscalar(semilla) || ...
        ~isfinite(semilla) || ...
        semilla < 0 || ...
        semilla > limiteSuperior || ...
        semilla ~= floor(semilla)

    error('crear_flujo_obstaculos:SemillaNoValida', ...
        ['semillaFija debe ser un entero comprendido entre 0 y ' ...
         '2^32-1.']);
end

semilla = double(semilla);
end