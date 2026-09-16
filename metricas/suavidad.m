function [curvaturaRMS, detalle] = suavidad(trayectoria)
% Calcula la suavidad geometrica de una trayectoria ejecutada.
%
%   curvaturaRMS = SUAVIDAD(trayectoria)
%
%   [curvaturaRMS,detalle] = SUAVIDAD(trayectoria)
%
%   Cuantifica los cambios de direccion de la trayectoria mediante la
%   curvatura cuadratica media ponderada por longitud de arco.
%
%   Para cada vertice interior se calculan:
%
%       psi_i = atan2(y_(i+1)-y_i, x_(i+1)-x_i)
%
%       DeltaPsi_i = wrapToPi(psi_(i+1)-psi_i)
%
%       DeltaS_i = 0.5*(l_i+l_(i+1))
%
%       kappa_i = DeltaPsi_i/DeltaS_i
%
%   donde l_i es la longitud del segmento i. La metrica principal es:
%
%       curvaturaRMS = sqrt( ...
%           sum(DeltaS_i*kappa_i^2) / sum(DeltaS_i) )
%
%   Interpretacion:
%
%       curvaturaRMS = 0       trayectoria recta
%       valor pequeno          trayectoria suave
%       valor grande           giros frecuentes o bruscos
%
%   La unidad es 1/m cuando las coordenadas se expresan en metros.
%   Por tanto, un valor MENOR representa una trayectoria MAS SUAVE.
%
%   Entrada:
%       trayectoria
%           Matriz N x M, con M >= 2. Las dos primeras columnas contienen
%           las posiciones [x y]. Se admiten directamente historiales
%           N x 3 con formato [x y theta]; theta no se utiliza para la
%           metrica geometrica.
%
%   Salidas:
%       curvaturaRMS
%           Indice principal de falta de suavidad [1/m].
%
%       detalle
%           Estructura con resultados auxiliares:
%
%               .calculable
%               .motivo
%               .curvaturaRMS
%               .curvaturaMediaAbsoluta
%               .curvaturaMaximaAbsoluta
%               .variacionAngularTotal
%               .giroNeto
%               .cambiosSentidoCurvatura
%               .longitudEvaluada
%               .numeroPuntosOriginales
%               .numeroPuntosUtilizados
%               .numeroPuntosEliminados
%               .numeroVerticesEvaluados
%               .curvaturas
%               .angulosGiro
%               .longitudesLocales
%               .unidad
%
%   Antes del calculo se eliminan posiciones consecutivas repetidas o
%   separadas unicamente por ruido numerico. Esto evita dividir por una
%   longitud de segmento nula.
%
%   Si quedan menos de tres posiciones distintas, la curvatura no puede
%   estimarse y la funcion devuelve NaN. En el analisis estadistico estas
%   observaciones deben tratarse de forma explicita, por ejemplo mediante
%   "omitnan", y nunca interpretarse como trayectorias perfectamente suaves.
%
%   IMPORTANTE:
%   Debe utilizarse la trayectoria realmente EJECUTADA por el robot, no el
%   camino global generado por RRT* o PRM.
%
%   Ejemplos:
%
%       % Trayectoria recta
%       trayectoriaRecta = [
%           0 0 0;
%           1 0 0;
%           2 0 0
%       ];
%
%       S = suavidad(trayectoriaRecta)
%       % S = 0
%
%       % Giro de 90 grados con segmentos de un metro
%       trayectoriaGiro = [
%           0 0;
%           1 0;
%           1 1
%       ];
%
%       S = suavidad(trayectoriaGiro)
%       % S = pi/2 aproximadamente
%
%   Esta funcion utiliza:
%       - wrap_to_pi_local.m

%% Validacion de la trayectoria
if ~isnumeric(trayectoria) || ~isreal(trayectoria)
    error('suavidad:TrayectoriaNoValida', ...
        'La trayectoria debe ser una matriz numerica real.');
end

if ~ismatrix(trayectoria) || ...
        (~isempty(trayectoria) && size(trayectoria,2) < 2)
    error('suavidad:DimensionNoValida', ...
        ['La trayectoria debe tener formato N x M con M >= 2; ' ...
         'las dos primeras columnas deben ser [x y].']);
end

if any(~isfinite(trayectoria(:)))
    error('suavidad:ValoresNoFinitos', ...
        ['La trayectoria contiene NaN o Inf. Recorte primero las filas ' ...
         'no utilizadas del historial preasignado.']);
end

trayectoria = double(trayectoria);
numeroPuntosOriginales = size(trayectoria,1);

%% Inicializacion de la salida auxiliar
detalle = estructura_detalle_vacia(numeroPuntosOriginales);

if isempty(trayectoria)
    curvaturaRMS = NaN;
    detalle.motivo = "trayectoria_vacia";
    return;
end

%% Eliminacion de posiciones consecutivas repetidas
posiciones = trayectoria(:,1:2);

escala = max(1,max(abs(posiciones(:))));
toleranciaPosicion = 1e-10*escala;

if size(posiciones,1) >= 2
    desplazamientosOriginales = diff(posiciones,1,1);

    longitudesOriginales = hypot( ...
        desplazamientosOriginales(:,1), ...
        desplazamientosOriginales(:,2));

    conservar = [true; longitudesOriginales > toleranciaPosicion];
    posiciones = posiciones(conservar,:);
end

numeroPuntosUtilizados = size(posiciones,1);

detalle.numeroPuntosUtilizados = numeroPuntosUtilizados;
detalle.numeroPuntosEliminados = ...
    numeroPuntosOriginales-numeroPuntosUtilizados;
detalle.toleranciaPosicion = toleranciaPosicion;

%% Se requieren al menos tres posiciones distintas
if numeroPuntosUtilizados < 3
    curvaturaRMS = NaN;
    detalle.motivo = "puntos_insuficientes";
    return;
end

%% Segmentos, longitudes y direcciones
segmentos = diff(posiciones,1,1);

longitudesSegmentos = hypot( ...
    segmentos(:,1), ...
    segmentos(:,2));

direcciones = atan2( ...
    segmentos(:,2), ...
    segmentos(:,1));

%% Cambio angular en cada vertice interior
angulosGiro = wrap_to_pi_local(diff(direcciones));

% Longitud de arco local asociada a cada vertice.
longitudesLocales = 0.5*( ...
    longitudesSegmentos(1:end-1) + ...
    longitudesSegmentos(2:end));

curvaturas = angulosGiro ./ longitudesLocales;

%% Metrica principal: curvatura RMS ponderada por longitud
pesoTotal = sum(longitudesLocales);

curvaturaRMS = sqrt( ...
    sum(longitudesLocales .* curvaturas.^2) / pesoTotal);

%% Medidas auxiliares
curvaturaMediaAbsoluta = ...
    sum(longitudesLocales .* abs(curvaturas)) / pesoTotal;

curvaturaMaximaAbsoluta = max(abs(curvaturas));
variacionAngularTotal = sum(abs(angulosGiro));
giroNeto = wrap_to_pi_local(sum(angulosGiro));

% Cambios de signo de curvatura, ignorando giros numericamente nulos.
toleranciaCurvatura = 1e-10*max(1,max(abs(curvaturas)));
signos = sign(curvaturas(abs(curvaturas) > toleranciaCurvatura));

if numel(signos) >= 2
    cambiosSentidoCurvatura = ...
        nnz(signos(1:end-1).*signos(2:end) < 0);
else
    cambiosSentidoCurvatura = 0;
end

%% Estructura de detalle
detalle.calculable = true;
detalle.motivo = "correcto";

detalle.curvaturaRMS = curvaturaRMS;
detalle.curvaturaMediaAbsoluta = curvaturaMediaAbsoluta;
detalle.curvaturaMaximaAbsoluta = curvaturaMaximaAbsoluta;

detalle.variacionAngularTotal = variacionAngularTotal;
detalle.giroNeto = giroNeto;
detalle.cambiosSentidoCurvatura = cambiosSentidoCurvatura;

detalle.longitudEvaluada = sum(longitudesSegmentos);
detalle.numeroVerticesEvaluados = numel(curvaturas);

detalle.curvaturas = curvaturas;
detalle.angulosGiro = angulosGiro;
detalle.longitudesLocales = longitudesLocales;

detalle.toleranciaCurvatura = toleranciaCurvatura;
end

%% ========================================================================
% FUNCION LOCAL
% ========================================================================

function detalle = estructura_detalle_vacia(numeroPuntos)
%ESTRUCTURA_DETALLE_VACIA Inicializa la informacion de diagnostico.

detalle = struct();

detalle.calculable = false;
detalle.motivo = "";

detalle.curvaturaRMS = NaN;
detalle.curvaturaMediaAbsoluta = NaN;
detalle.curvaturaMaximaAbsoluta = NaN;

detalle.variacionAngularTotal = NaN;
detalle.giroNeto = NaN;
detalle.cambiosSentidoCurvatura = NaN;

detalle.longitudEvaluada = NaN;

detalle.numeroPuntosOriginales = numeroPuntos;
detalle.numeroPuntosUtilizados = 0;
detalle.numeroPuntosEliminados = 0;
detalle.numeroVerticesEvaluados = 0;

detalle.curvaturas = zeros(0,1);
detalle.angulosGiro = zeros(0,1);
detalle.longitudesLocales = zeros(0,1);

detalle.toleranciaPosicion = NaN;
detalle.toleranciaCurvatura = NaN;

detalle.unidad = "1/m";
detalle.criterio = "menor_valor_mayor_suavidad";
end
