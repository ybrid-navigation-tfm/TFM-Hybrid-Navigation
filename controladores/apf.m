function [control, info] = apf( ...
    estadoRobot, objetivoLocal, obstaculosEstaticos, ...
    obstaculosDinamicos, limites, robot, cfg)
% APF Calcula el control local mediante Campos Potenciales Artificiales.
%
%   control = APF( ...
%       estadoRobot,objetivoLocal,obstaculosEstaticos, ...
%       obstaculosDinamicos,limites,robot,cfg)
%
%   [control,info] = APF(...)
%
%   Implementa el controlador local de la arquitectura RRT* + APF. El
%   planificador global RRT* proporciona una trayectoria y el modulo
%   seguimiento_trayectoria.m selecciona un objetivo local adelantado. APF
%   combina entonces:
%
%       - una fuerza atractiva hacia el objetivo local;
%       - fuerzas repulsivas debidas a obstaculos estaticos;
%       - fuerzas repulsivas debidas a obstaculos dinamicos predichos;
%       - fuerzas repulsivas debidas a los limites del mapa.
%
%   La fuerza total se transforma en un control para el modelo de uniciclo:
%
%       control = [v w]
%
%   donde v es la velocidad lineal y w la velocidad angular.
%
%   IMPORTANTE PARA LA COMPARACION EXPERIMENTAL:
%       Esta funcion actua exclusivamente como CONTROLADOR LOCAL. No sesga
%       el muestreo de RRT*, no modifica sus costes, no interviene en el
%       rewiring y no contiene ningun parametro apfBias. De este modo,
%       RRT* + APF y RRT* + MPC pueden utilizar exactamente el mismo
%       planificador global.
%
%   Entradas:
%       estadoRobot
%           Estado actual del robot:
%
%               [x y theta]
%
%           x, y  : posicion del centro                     [m]
%           theta : orientacion                             [rad]
%
%       objetivoLocal
%           Posicion objetivo seleccionada sobre el camino global:
%
%               [xObjetivo yObjetivo]
%
%           Normalmente procede de seguimiento_trayectoria.m y no tiene
%           por que coincidir con la meta final del escenario.
%
%       obstaculosEstaticos
%           Matriz N x 4 de rectangulos alineados con los ejes:
%
%               [x y ancho alto]
%
%       obstaculosDinamicos
%           Vector de estructuras con los campos:
%
%               id      identificador opcional
%               pos     posicion actual [x y]               [m]
%               vel     velocidad [vx vy]                   [m/s]
%               radio   radio fisico                        [m]
%
%       limites
%           Limites del mapa:
%
%               [xmin xmax ymin ymax]
%
%       robot
%           Estructura obtenida mediante configuracion_robot.m. Se usan:
%
%               robot.tipo
%               robot.geometria.radio
%               robot.limites.vMin
%               robot.limites.vMax
%               robot.limites.wMin
%               robot.limites.wMax
%               robot.controlParada
%
%       cfg
%           Estructura obtenida mediante parametros_generales.m. Se usan:
%
%               cfg.sim.Ts
%               cfg.navegacion.tolWaypoint
%               cfg.seguridad.margenEstatico
%               cfg.seguridad.margenDinamico
%               cfg.prediccion.pasos
%               cfg.prediccion.modelo
%               cfg.apf.kAtractivo
%               cfg.apf.kRepulsivo
%               cfg.apf.radioInfluencia
%               cfg.apf.kGiro
%               cfg.apf.distanciaFrenado
%               cfg.apf.anguloParada
%
%   Salidas:
%       control
%           Control solicitado al modelo de robot:
%
%               [v w]
%
%           La funcion limita ambos valores al intervalo configurado. El
%           modulo modelo_robot.m volvera a aplicar los limites como ultima
%           proteccion antes de integrar el estado.
%
%       info
%           Estructura de diagnostico con, entre otros, los campos:
%
%               .exito
%               .controlValido
%               .motivo
%               .controlSolicitado
%               .objetivoLocal
%               .distanciaObjetivo
%               .objetivoDentroTolerancia
%               .fuerzaAtractiva
%               .fuerzaRepulsiva
%               .fuerzaTotal
%               .potencialAtractivo
%               .potencialRepulsivo
%               .potencialTotal
%               .anguloDeseado
%               .errorAngular
%               .factorDistancia
%               .factorOrientacion
%               .minimoLocal
%               .distanciaLibreSeguridadMinima
%               .distanciaFisicaMinima
%               .estaticos
%               .dinamicos
%               .limites
%               .parametros
%
%   Formulacion del campo:
%       Fuerza atractiva:
%
%           Fatt(q) = katt * (qObjetivo - q)
%
%       Para una distancia libre rho menor que el radio de influencia rho0:
%
%           Frep(q) = krep * (1/rho - 1/rho0) * (1/rho^2) * n
%
%       donde n es la direccion unitaria desde el obstaculo hacia el robot.
%
%   Obstaculos dinamicos:
%       La posicion futura se aproxima mediante velocidad constante:
%
%           pFutura = pActual + ...
%               cfg.prediccion.pasos*cfg.sim.Ts*velocidad
%
%       Para no ignorar ni la posicion actual ni la futura, el obstaculo se
%       representa durante el horizonte mediante el segmento barrido entre
%       ambas posiciones. La fuerza se calcula respecto al punto de ese
%       segmento mas cercano al robot.
%
%   Conversion al modelo de uniciclo:
%
%       thetaDeseada = atan2(Fy,Fx)
%       errorTheta   = wrapToPi(thetaDeseada-theta)
%       w            = kGiro*errorTheta
%
%       La velocidad lineal se reduce:
%
%       - al aproximarse al objetivo local;
%       - cuando el error de orientacion aumenta;
%       - hasta anularse cuando |errorTheta| >= anguloParada.
%
%   Minimos locales:
%       APF puede producir una fuerza total casi nula lejos del objetivo
%       por cancelacion entre atraccion y repulsion. La funcion no incorpora
%       una maniobra artificial de escape, porque se desea evaluar el APF
%       convencional. En ese caso devuelve el control de parada y marca:
%
%           info.minimoLocal = true
%
%   Ejemplo:
%
%       cfg = parametros_generales("visual");
%       escenario = escenarios("media");
%       robot = configuracion_robot();
%
%       estado = escenario.inicio;
%       objetivoLocal = [3.0 3.0];
%
%       [u,infoAPF] = apf( ...
%           estado, ...
%           objetivoLocal, ...
%           escenario.obstaculosEstaticos, ...
%           escenario.obstaculosDinamicos, ...
%           escenario.limites, ...
%           robot, ...
%           cfg);
%
%       [estadoSiguiente,uAplicado] = modelo_robot( ...
%           estado,u,cfg.sim.Ts,robot);
%
%   Esta funcion utiliza:
%       - wrap_to_pi_local.m
%       - circulo_rectangulo.m
%
%   El tiempo de control debe medirse externamente mediante tic/toc para
%   que tiempos.m pueda comparar APF y MPC con el mismo criterio.

%% Validacion y normalizacion
[estadoRobot, objetivoLocal, estaticos, dinamicos, limites, ...
    parametros] = validar_entradas( ...
        estadoRobot,objetivoLocal,obstaculosEstaticos, ...
        obstaculosDinamicos,limites,robot,cfg);

posicionRobot = estadoRobot(1:2);
orientacionRobot = estadoRobot(3);

%% Escalas y tolerancias numericas
valoresEscala = [ ...
    posicionRobot(:); ...
    objetivoLocal(:); ...
    limites(:); ...
    estaticos(:)];

for i = 1:numel(dinamicos)
    valoresEscala = [ ...
        valoresEscala; ... %#ok<AGROW>
        dinamicos(i).pos(:); ...
        dinamicos(i).vel(:); ...
        dinamicos(i).radio]; %#ok<AGROW>
end

escala = max(1,max(abs(valoresEscala)));
tolerancia = 1e-12*escala;
toleranciaPosicion = 1e-10*escala;

% Regularizacion de la singularidad de la fuerza repulsiva. Solo afecta al
% calculo numerico cuando el robot entra en la envolvente de seguridad.
rhoRegularizacion = max(1e-3,0.02*parametros.radioInfluencia);

%% ========================================================================
% FUERZA ATRACTIVA
% ========================================================================

vectorObjetivo = objetivoLocal-posicionRobot;
distanciaObjetivo = norm(vectorObjetivo);

fuerzaAtractiva = ...
    parametros.kAtractivo*vectorObjetivo;

potencialAtractivo = ...
    0.5*parametros.kAtractivo*distanciaObjetivo^2;

%% ========================================================================
% FUERZAS REPULSIVAS: OBSTACULOS ESTATICOS
% ========================================================================

numeroEstaticos = size(estaticos,1);

detalleEstaticos = estructura_detalle_estaticos(numeroEstaticos);

for i = 1:numeroEstaticos
    rectangulo = estaticos(i,:);

    % Con radio cero se obtiene la distancia del centro del robot al
    % rectangulo y una normal exterior coherente incluso si el centro se
    % encuentra dentro del obstaculo.
    [~,normal,distanciaCentro] = circulo_rectangulo( ...
        posicionRobot,0,rectangulo);

    rho = distanciaCentro-parametros.separacionEstatica;

    [fuerza,potencial,rhoEvaluada,activa] = ...
        contribucion_repulsiva( ...
            normal,rho,parametros.kRepulsivo, ...
            parametros.radioInfluencia,rhoRegularizacion,tolerancia);

    detalleEstaticos.ids(i) = "S"+i;
    detalleEstaticos.distanciasCentro(i) = distanciaCentro;
    detalleEstaticos.distanciasFisicas(i) = ...
        distanciaCentro-parametros.radioRobot;
    detalleEstaticos.distanciasSeguridad(i) = rho;
    detalleEstaticos.distanciasEvaluadas(i) = rhoEvaluada;
    detalleEstaticos.normales(i,:) = normal;
    detalleEstaticos.fuerzas(i,:) = fuerza;
    detalleEstaticos.potenciales(i) = potencial;
    detalleEstaticos.activos(i) = activa;
    detalleEstaticos.dentroSeguridad(i) = rho <= 0;
end

fuerzaRepulsivaEstaticos = ...
    sum(detalleEstaticos.fuerzas,1);

potencialRepulsivoEstaticos = ...
    sum(detalleEstaticos.potenciales);

%% ========================================================================
% FUERZAS REPULSIVAS: OBSTACULOS DINAMICOS
% ========================================================================

numeroDinamicos = numel(dinamicos);
horizontePrediccion = ...
    parametros.pasosPrediccion*parametros.Ts;

detalleDinamicos = estructura_detalle_dinamicos(numeroDinamicos);

for i = 1:numeroDinamicos
    posicionActual = dinamicos(i).pos;
    posicionFutura = posicionActual+ ...
        horizontePrediccion*dinamicos(i).vel;

    [puntoEfectivo,fraccionSegmento,distanciaCentro,normal] = ...
        punto_mas_cercano_barrido( ...
            posicionRobot,posicionActual,posicionFutura, ...
            dinamicos(i).vel,orientacionRobot,tolerancia);

    separacionDinamica = ...
        parametros.radioRobot+dinamicos(i).radio+ ...
        parametros.margenDinamico;

    rho = distanciaCentro-separacionDinamica;

    [fuerza,potencial,rhoEvaluada,activa] = ...
        contribucion_repulsiva( ...
            normal,rho, ...
            parametros.kRepulsivo*parametros.factorRepulsionDinamica, ...
            parametros.radioInfluencia,rhoRegularizacion,tolerancia);

    detalleDinamicos.ids(i) = obtener_id_dinamico(dinamicos(i),i);
    detalleDinamicos.posicionesActuales(i,:) = posicionActual;
    detalleDinamicos.posicionesPredichas(i,:) = posicionFutura;
    detalleDinamicos.puntosEfectivos(i,:) = puntoEfectivo;
    detalleDinamicos.fraccionesBarrido(i) = fraccionSegmento;
    detalleDinamicos.distanciasCentro(i) = distanciaCentro;
    detalleDinamicos.distanciasFisicas(i) = ...
        distanciaCentro-parametros.radioRobot-dinamicos(i).radio;
    detalleDinamicos.distanciasSeguridad(i) = rho;
    detalleDinamicos.distanciasEvaluadas(i) = rhoEvaluada;
    detalleDinamicos.normales(i,:) = normal;
    detalleDinamicos.fuerzas(i,:) = fuerza;
    detalleDinamicos.potenciales(i) = potencial;
    detalleDinamicos.activos(i) = activa;
    detalleDinamicos.dentroSeguridad(i) = rho <= 0;
end

fuerzaRepulsivaDinamicos = ...
    sum(detalleDinamicos.fuerzas,1);

potencialRepulsivoDinamicos = ...
    sum(detalleDinamicos.potenciales);

%% ========================================================================
% FUERZAS REPULSIVAS: LIMITES DEL MAPA
% ========================================================================

% Orden: izquierdo, derecho, inferior y superior.
nombresLimites = ["izquierdo";"derecho";"inferior";"superior"];

normalesLimites = [ ...
     1  0; ...
    -1  0; ...
     0  1; ...
     0 -1];

distanciasCentroLimites = [ ...
    posicionRobot(1)-limites(1); ...
    limites(2)-posicionRobot(1); ...
    posicionRobot(2)-limites(3); ...
    limites(4)-posicionRobot(2)];

detalleLimites = estructura_detalle_limites();
detalleLimites.ids = nombresLimites;
detalleLimites.distanciasCentro = distanciasCentroLimites;
detalleLimites.distanciasFisicas = ...
    distanciasCentroLimites-parametros.radioRobot;

detalleLimites.distanciasSeguridad = ...
    distanciasCentroLimites-parametros.separacionEstatica;

detalleLimites.normales = normalesLimites;

for i = 1:4
    rho = detalleLimites.distanciasSeguridad(i);

    [fuerza,potencial,rhoEvaluada,activa] = ...
        contribucion_repulsiva( ...
            normalesLimites(i,:),rho,parametros.kRepulsivo, ...
            parametros.radioInfluencia,rhoRegularizacion,tolerancia);

    detalleLimites.distanciasEvaluadas(i) = rhoEvaluada;
    detalleLimites.fuerzas(i,:) = fuerza;
    detalleLimites.potenciales(i) = potencial;
    detalleLimites.activos(i) = activa;
    detalleLimites.dentroSeguridad(i) = rho <= 0;
end

fuerzaRepulsivaLimites = ...
    sum(detalleLimites.fuerzas,1);

potencialRepulsivoLimites = ...
    sum(detalleLimites.potenciales);

%% ========================================================================
% CAMPO TOTAL
% ========================================================================

fuerzaRepulsiva = ...
    fuerzaRepulsivaEstaticos+ ...
    fuerzaRepulsivaDinamicos+ ...
    fuerzaRepulsivaLimites;

fuerzaTotal = fuerzaAtractiva+fuerzaRepulsiva;

potencialRepulsivo = ...
    potencialRepulsivoEstaticos+ ...
    potencialRepulsivoDinamicos+ ...
    potencialRepulsivoLimites;

potencialTotal = potencialAtractivo+potencialRepulsivo;

normaFuerzaAtractiva = norm(fuerzaAtractiva);
normaFuerzaRepulsiva = norm(fuerzaRepulsiva);
normaFuerzaTotal = norm(fuerzaTotal);

toleranciaFuerza = 1e-10*max( ...
    1,normaFuerzaAtractiva+normaFuerzaRepulsiva);

%% Distancias minima fisica y respecto a la envolvente de seguridad
[distanciaFisicaMinima,tipoFisicoMinimo,idFisicoMinimo] = ...
    minimo_global( ...
        detalleEstaticos.distanciasFisicas,detalleEstaticos.ids, ...
        detalleDinamicos.distanciasFisicas,detalleDinamicos.ids, ...
        detalleLimites.distanciasFisicas,detalleLimites.ids);

[distanciaSeguridadMinima,tipoSeguridadMinimo,idSeguridadMinimo] = ...
    minimo_global( ...
        detalleEstaticos.distanciasSeguridad,detalleEstaticos.ids, ...
        detalleDinamicos.distanciasSeguridad,detalleDinamicos.ids, ...
        detalleLimites.distanciasSeguridad,detalleLimites.ids);

%% ========================================================================
% CONVERSION DE LA FUERZA EN CONTROL DE UNICICLO
% ========================================================================

objetivoCoincidente = ...
    distanciaObjetivo <= toleranciaPosicion;

objetivoDentroTolerancia = ...
    distanciaObjetivo <= parametros.toleranciaWaypoint+toleranciaPosicion;

minimoLocal = ...
    ~objetivoCoincidente && normaFuerzaTotal <= toleranciaFuerza;

anguloDeseado = orientacionRobot;
errorAngular = 0;
factorDistancia = 0;
factorOrientacion = 0;
velocidadLinealSinLimitar = 0;
velocidadAngularSinLimitar = 0;

if objetivoCoincidente
    control = parametros.controlParada;
    motivo = "objetivo_local_coincidente";
    exito = true;

else
    % Un minimo local no provoca una parada indefinida. En ese caso se
    % utiliza temporalmente la direccion atractiva pura para recuperar
    % progreso hacia el objetivo local.
    if minimoLocal
        fuerzaDireccion = fuerzaAtractiva;
        motivo = "escape_minimo_local_hacia_objetivo";
    else
        fuerzaDireccion = fuerzaTotal;
        motivo = "control_calculado";
    end

    anguloDeseado = atan2( ...
        fuerzaDireccion(2),fuerzaDireccion(1));

    errorAngular = wrap_to_pi_local( ...
        anguloDeseado-orientacionRobot);

    velocidadAngularSinLimitar = ...
        parametros.kGiro*errorAngular;

    factorDistancia = min( ...
        1,distanciaObjetivo/parametros.distanciaFrenado);

    factorOrientacion = factor_orientacion( ...
        abs(errorAngular),parametros.anguloParada);

    velocidadLinealSinLimitar = ...
        parametros.vMax*factorDistancia*factorOrientacion;

    if velocidadLinealSinLimitar <= tolerancia_control_previa(parametros)
        velocidadLineal = 0;
    else
        velocidadLineal = min(max( ...
            velocidadLinealSinLimitar,parametros.vMin), ...
            parametros.vMax);
    end

    velocidadAngular = min(max( ...
        velocidadAngularSinLimitar,parametros.wMin), ...
        parametros.wMax);

    control = [velocidadLineal velocidadAngular];
    exito = true;
end

%% Saturaciones aplicadas por el propio controlador
escalaControl = max(1,max(abs([ ...
    velocidadLinealSinLimitar, ...
    velocidadAngularSinLimitar, ...
    parametros.vMax,parametros.wMin,parametros.wMax])));

toleranciaControl = 1e-12*escalaControl;

saturacionVelocidadLineal = ...
    abs(control(1)-velocidadLinealSinLimitar) > toleranciaControl;

saturacionVelocidadAngular = ...
    abs(control(2)-velocidadAngularSinLimitar) > toleranciaControl;

controlValido = ...
    isnumeric(control) && isequal(size(control),[1 2]) && ...
    all(isfinite(control));

if ~controlValido
    error('apf:ControlNoFinito', ...
        'El calculo del APF produjo un control no finito.');
end

%% ========================================================================
% INFORMACION DE DIAGNOSTICO
% ========================================================================

info = struct();
info.exito = exito;
info.controlValido = controlValido;
info.motivo = motivo;

info.estadoRobot = estadoRobot;
info.posicionRobot = posicionRobot;
info.orientacionRobot = orientacionRobot;
info.objetivoLocal = objetivoLocal;
info.vectorObjetivo = vectorObjetivo;
info.distanciaObjetivo = distanciaObjetivo;
info.objetivoCoincidente = objetivoCoincidente;
info.objetivoDentroTolerancia = objetivoDentroTolerancia;

info.controlSolicitado = control;
info.velocidadLinealSinLimitar = velocidadLinealSinLimitar;
info.velocidadAngularSinLimitar = velocidadAngularSinLimitar;
info.saturacionVelocidadLineal = saturacionVelocidadLineal;
info.saturacionVelocidadAngular = saturacionVelocidadAngular;
info.controlSaturado = ...
    saturacionVelocidadLineal || saturacionVelocidadAngular;

info.fuerzaAtractiva = fuerzaAtractiva;
info.fuerzaRepulsivaEstaticos = fuerzaRepulsivaEstaticos;
info.fuerzaRepulsivaDinamicos = fuerzaRepulsivaDinamicos;
info.fuerzaRepulsivaLimites = fuerzaRepulsivaLimites;
info.fuerzaRepulsiva = fuerzaRepulsiva;
info.fuerzaTotal = fuerzaTotal;

info.normaFuerzaAtractiva = normaFuerzaAtractiva;
info.normaFuerzaRepulsiva = normaFuerzaRepulsiva;
info.normaFuerzaTotal = normaFuerzaTotal;
info.toleranciaFuerza = toleranciaFuerza;
info.minimoLocal = minimoLocal;

info.potencialAtractivo = potencialAtractivo;
info.potencialRepulsivoEstaticos = potencialRepulsivoEstaticos;
info.potencialRepulsivoDinamicos = potencialRepulsivoDinamicos;
info.potencialRepulsivoLimites = potencialRepulsivoLimites;
info.potencialRepulsivo = potencialRepulsivo;
info.potencialTotal = potencialTotal;

info.anguloDeseado = anguloDeseado;
info.errorAngular = errorAngular;
info.factorDistancia = factorDistancia;
info.factorOrientacion = factorOrientacion;

info.distanciaFisicaMinima = distanciaFisicaMinima;
info.tipoFisicoMasCercano = tipoFisicoMinimo;
info.idFisicoMasCercano = idFisicoMinimo;

info.distanciaLibreSeguridadMinima = distanciaSeguridadMinima;
info.tipoSeguridadMasCercano = tipoSeguridadMinimo;
info.idSeguridadMasCercano = idSeguridadMinimo;

info.dentroEnvolventeSeguridad = ...
    distanciaSeguridadMinima <= tolerancia;

info.colisionFisicaConservadora = ...
    distanciaFisicaMinima <= tolerancia;

info.estaticos = completar_resumen_estaticos(detalleEstaticos);
info.dinamicos = completar_resumen_dinamicos(detalleDinamicos);
info.limites = completar_resumen_limites(detalleLimites);

info.horizontePrediccion = horizontePrediccion;
info.rhoRegularizacion = rhoRegularizacion;
info.toleranciaNumerica = tolerancia;

info.parametros = parametros;
info.formulaAtractiva = ...
    "Fatt = kAtractivo*(objetivoLocal-posicionRobot)";
info.formulaRepulsiva = ...
    "Frep = kRepulsivo*(1/rho-1/rho0)*(1/rho^2)*normal";
info.formulaControlAngular = ...
    "w = kGiro*wrapToPi(thetaDeseada-theta)";
info.criterioMinimoLocal = ...
    "norm(Ftotal) <= toleranciaFuerza y objetivo no alcanzado";
end

%% ========================================================================
% CAMPO REPULSIVO
% ========================================================================

function [fuerza,potencial,rhoEvaluada,activa] = ...
    contribucion_repulsiva( ...
        normal,rho,kRepulsivo,radioInfluencia, ...
        rhoRegularizacion,tolerancia)
%CONTRIBUCION_REPULSIVA Evalua una contribucion del campo repulsivo.

normal = reshape(double(normal),1,2);
normaNormal = norm(normal);

if normaNormal <= tolerancia
    normal = [1 0];
else
    normal = normal/normaNormal;
end

activa = rho <= radioInfluencia+tolerancia;

if ~activa
    fuerza = [0 0];
    potencial = 0;
    rhoEvaluada = rho;
    return;
end

rhoEvaluada = max(rho,rhoRegularizacion);

inversoRho = 1/rhoEvaluada;
inversoInfluencia = 1/radioInfluencia;

diferenciaInversa = inversoRho-inversoInfluencia;

% Cuando rho se encuentra ligeramente por encima de rho0 debido a la
% tolerancia, la contribucion no debe cambiar de signo.
diferenciaInversa = max(0,diferenciaInversa);

potencial = ...
    0.5*kRepulsivo*diferenciaInversa^2;

magnitud = ...
    kRepulsivo*diferenciaInversa*inversoRho^2;

fuerza = magnitud*normal;
end

%% ========================================================================
% GEOMETRIA DE OBSTACULOS DINAMICOS
% ========================================================================

function [punto,fraccion,distancia,normal] = ...
    punto_mas_cercano_barrido( ...
        q,p0,p1,velocidad,orientacionRobot,tolerancia)
%PUNTO_MAS_CERCANO_BARRIDO Punto mas cercano de un segmento al robot.

segmento = p1-p0;
denominador = dot(segmento,segmento);

if denominador <= tolerancia^2
    fraccion = 0;
    punto = p0;
else
    fraccion = dot(q-p0,segmento)/denominador;
    fraccion = min(max(fraccion,0),1);
    punto = p0+fraccion*segmento;
end

vectorSalida = q-punto;
distancia = norm(vectorSalida);

if distancia > tolerancia
    normal = vectorSalida/distancia;
    return;
end

% Si el robot coincide exactamente con el segmento barrido no existe una
% normal geometrica unica. Se escoge una direccion determinista de escape.
normaVelocidad = norm(velocidad);

if normaVelocidad > tolerancia
    normal = -velocidad/normaVelocidad;
else
    normal = [-cos(orientacionRobot) -sin(orientacionRobot)];
end

if norm(normal) <= tolerancia
    normal = [1 0];
else
    normal = normal/norm(normal);
end
end

%% ========================================================================
% VELOCIDAD DEL UNICICLO
% ========================================================================

function tolerancia = tolerancia_control_previa(parametros)
%TOLERANCIACONTROL_PREVIA Umbral para distinguir parada y movimiento.

escala = max(1,max(abs([ ...
    parametros.vMin,parametros.vMax, ...
    parametros.wMin,parametros.wMax])));

tolerancia = 1e-12*escala;
end

function factor = factor_orientacion(errorAbsoluto,anguloParada)
%FACTOR_ORIENTACION Reduce suavemente la velocidad al aumentar el error.

if errorAbsoluto >= anguloParada
    factor = 0;
    return;
end

fraccion = errorAbsoluto/anguloParada;

% Coseno reescalado: 1 para error nulo y 0 en anguloParada.
factor = cos(0.5*pi*fraccion);
factor = min(max(factor,0),1);
end

%% ========================================================================
% MINIMOS Y RESUMENES
% ========================================================================

function [valorMinimo,tipo,id] = minimo_global( ...
    valoresEstaticos,idsEstaticos, ...
    valoresDinamicos,idsDinamicos, ...
    valoresLimites,idsLimites)
%MINIMO_GLOBAL Localiza el menor valor entre tres familias de obstaculos.

valores = [ ...
    valoresEstaticos(:); ...
    valoresDinamicos(:); ...
    valoresLimites(:)];

if isempty(valores)
    valorMinimo = inf;
    tipo = "ninguno";
    id = "";
    return;
end

[valorMinimo,indice] = min(valores);

nEstaticos = numel(valoresEstaticos);
nDinamicos = numel(valoresDinamicos);

if indice <= nEstaticos
    tipo = "estatico";
    id = idsEstaticos(indice);
elseif indice <= nEstaticos+nDinamicos
    tipo = "dinamico";
    id = idsDinamicos(indice-nEstaticos);
else
    tipo = "limite";
    id = idsLimites(indice-nEstaticos-nDinamicos);
end
end

function detalle = completar_resumen_estaticos(detalle)
%COMPLETAR_RESUMEN_ESTATICOS Incorpora contadores de actividad.

detalle.indicesActivos = find(detalle.activos);
detalle.numeroActivos = nnz(detalle.activos);
detalle.indicesDentroSeguridad = find(detalle.dentroSeguridad);
detalle.numeroDentroSeguridad = nnz(detalle.dentroSeguridad);
detalle.fuerzaTotal = sum(detalle.fuerzas,1);
detalle.potencialTotal = sum(detalle.potenciales);
end

function detalle = completar_resumen_dinamicos(detalle)
%COMPLETAR_RESUMEN_DINAMICOS Incorpora contadores de actividad.

detalle.indicesActivos = find(detalle.activos);
detalle.numeroActivos = nnz(detalle.activos);
detalle.indicesDentroSeguridad = find(detalle.dentroSeguridad);
detalle.numeroDentroSeguridad = nnz(detalle.dentroSeguridad);
detalle.fuerzaTotal = sum(detalle.fuerzas,1);
detalle.potencialTotal = sum(detalle.potenciales);
end

function detalle = completar_resumen_limites(detalle)
%COMPLETAR_RESUMEN_LIMITES Incorpora contadores de actividad.

detalle.indicesActivos = find(detalle.activos);
detalle.numeroActivos = nnz(detalle.activos);
detalle.indicesDentroSeguridad = find(detalle.dentroSeguridad);
detalle.numeroDentroSeguridad = nnz(detalle.dentroSeguridad);
detalle.fuerzaTotal = sum(detalle.fuerzas,1);
detalle.potencialTotal = sum(detalle.potenciales);
end

%% ========================================================================
% ESTRUCTURAS DE DETALLE
% ========================================================================

function detalle = estructura_detalle_estaticos(numero)
%ESTRUCTURA_DETALLE_ESTATICOS Inicializa el diagnostico estatico.

detalle = struct();
detalle.ids = strings(numero,1);
detalle.distanciasCentro = inf(numero,1);
detalle.distanciasFisicas = inf(numero,1);
detalle.distanciasSeguridad = inf(numero,1);
detalle.distanciasEvaluadas = inf(numero,1);
detalle.normales = zeros(numero,2);
detalle.fuerzas = zeros(numero,2);
detalle.potenciales = zeros(numero,1);
detalle.activos = false(numero,1);
detalle.dentroSeguridad = false(numero,1);
end

function detalle = estructura_detalle_dinamicos(numero)
%ESTRUCTURA_DETALLE_DINAMICOS Inicializa el diagnostico dinamico.

detalle = struct();
detalle.ids = strings(numero,1);
detalle.posicionesActuales = zeros(numero,2);
detalle.posicionesPredichas = zeros(numero,2);
detalle.puntosEfectivos = zeros(numero,2);
detalle.fraccionesBarrido = zeros(numero,1);
detalle.distanciasCentro = inf(numero,1);
detalle.distanciasFisicas = inf(numero,1);
detalle.distanciasSeguridad = inf(numero,1);
detalle.distanciasEvaluadas = inf(numero,1);
detalle.normales = zeros(numero,2);
detalle.fuerzas = zeros(numero,2);
detalle.potenciales = zeros(numero,1);
detalle.activos = false(numero,1);
detalle.dentroSeguridad = false(numero,1);
end

function detalle = estructura_detalle_limites()
%ESTRUCTURA_DETALLE_LIMITES Inicializa el diagnostico del contorno.

detalle = struct();
detalle.ids = strings(4,1);
detalle.distanciasCentro = inf(4,1);
detalle.distanciasFisicas = inf(4,1);
detalle.distanciasSeguridad = inf(4,1);
detalle.distanciasEvaluadas = inf(4,1);
detalle.normales = zeros(4,2);
detalle.fuerzas = zeros(4,2);
detalle.potenciales = zeros(4,1);
detalle.activos = false(4,1);
detalle.dentroSeguridad = false(4,1);
end

%% ========================================================================
% IDENTIFICADORES
% ========================================================================

function id = obtener_id_dinamico(dinamico,indice)
%OBTENER_ID_DINAMICO Recupera el id almacenado o genera D1, D2, ...

if isfield(dinamico,'id') && ...
        strlength(strtrim(string(dinamico.id))) > 0
    id = strtrim(string(dinamico.id));
else
    id = "D"+indice;
end
end

%% ========================================================================
% VALIDACION
% ========================================================================

function [estado,objetivo,estaticos,dinamicos,limites,parametros] = ...
    validar_entradas( ...
        estado,objetivo,estaticos,dinamicos,limites,robot,cfg)
%VALIDAR_ENTRADAS Comprueba el contrato del controlador APF.

%% Estado del robot
if ~isnumeric(estado) || ~isreal(estado) || numel(estado) ~= 3 || ...
        any(~isfinite(estado(:)))
    error('apf:EstadoRobotNoValido', ...
        'estadoRobot debe ser un vector real y finito [x y theta].');
end

estado = reshape(double(estado),1,3);
estado(3) = wrap_to_pi_local(estado(3));

%% Objetivo local
if ~isnumeric(objetivo) || ~isreal(objetivo) || ...
        numel(objetivo) ~= 2 || any(~isfinite(objetivo(:)))
    error('apf:ObjetivoLocalNoValido', ...
        'objetivoLocal debe ser un vector real y finito [x y].');
end

objetivo = reshape(double(objetivo),1,2);

%% Limites
if ~isnumeric(limites) || ~isreal(limites) || ...
        numel(limites) ~= 4 || any(~isfinite(limites(:)))
    error('apf:LimitesNoValidos', ...
        'limites debe tener formato [xmin xmax ymin ymax].');
end

limites = reshape(double(limites),1,4);

if limites(2) <= limites(1) || limites(4) <= limites(3)
    error('apf:OrdenLimitesNoValido', ...
        'Debe cumplirse xmin < xmax e ymin < ymax.');
end

if estado(1) < limites(1) || estado(1) > limites(2) || ...
        estado(2) < limites(3) || estado(2) > limites(4)
    error('apf:RobotFueraDelMapa', ...
        'La posicion actual del robot queda fuera de los limites.');
end

if objetivo(1) < limites(1) || objetivo(1) > limites(2) || ...
        objetivo(2) < limites(3) || objetivo(2) > limites(4)
    error('apf:ObjetivoFueraDelMapa', ...
        'El objetivo local queda fuera de los limites.');
end

%% Obstaculos estaticos
if isempty(estaticos)
    estaticos = zeros(0,4);
elseif ~isnumeric(estaticos) || ~isreal(estaticos) || ...
        size(estaticos,2) ~= 4 || any(~isfinite(estaticos(:)))
    error('apf:EstaticosNoValidos', ...
        ['obstaculosEstaticos debe ser una matriz N x 4 con formato ' ...
         '[x y ancho alto].']);
else
    estaticos = double(estaticos);
end

if ~isempty(estaticos) && any(estaticos(:,3:4) <= 0,'all')
    error('apf:DimensionesEstaticosNoValidas', ...
        'El ancho y el alto de los obstaculos deben ser positivos.');
end

%% Obstaculos dinamicos
if isempty(dinamicos)
    dinamicos = struct('id',{},'pos',{},'vel',{},'radio',{});
elseif ~isstruct(dinamicos)
    error('apf:DinamicosNoValidos', ...
        'obstaculosDinamicos debe ser un vector de estructuras.');
else
    camposDinamicos = {'pos','vel','radio'};

    for i = 1:numel(dinamicos)
        for j = 1:numel(camposDinamicos)
            if ~isfield(dinamicos(i),camposDinamicos{j})
                error('apf:CampoDinamicoAusente', ...
                    'Falta el campo %s en el obstaculo dinamico %d.', ...
                    camposDinamicos{j},i);
            end
        end

        if ~isnumeric(dinamicos(i).pos) || ...
                ~isreal(dinamicos(i).pos) || ...
                numel(dinamicos(i).pos) ~= 2 || ...
                any(~isfinite(dinamicos(i).pos(:)))
            error('apf:PosicionDinamicaNoValida', ...
                'La posicion del obstaculo %d debe tener formato [x y].',i);
        end

        if ~isnumeric(dinamicos(i).vel) || ...
                ~isreal(dinamicos(i).vel) || ...
                numel(dinamicos(i).vel) ~= 2 || ...
                any(~isfinite(dinamicos(i).vel(:)))
            error('apf:VelocidadDinamicaNoValida', ...
                'La velocidad del obstaculo %d debe tener formato [vx vy].',i);
        end

        if ~es_escalar_positivo(dinamicos(i).radio)
            error('apf:RadioDinamicoNoValido', ...
                'El radio del obstaculo dinamico %d debe ser positivo.',i);
        end

        dinamicos(i).pos = reshape(double(dinamicos(i).pos),1,2);
        dinamicos(i).vel = reshape(double(dinamicos(i).vel),1,2);
        dinamicos(i).radio = double(dinamicos(i).radio);
    end
end

%% Robot
if ~isstruct(robot) || ~isscalar(robot)
    error('apf:RobotNoValido', ...
        'robot debe proceder de configuracion_robot.m.');
end

camposRobot = {'tipo','geometria','limites','controlParada'};

for i = 1:numel(camposRobot)
    if ~isfield(robot,camposRobot{i})
        error('apf:ConfiguracionRobotIncompleta', ...
            'Falta robot.%s.',camposRobot{i});
    end
end

if string(robot.tipo) ~= "uniciclo"
    error('apf:ModeloNoSoportado', ...
        'APF genera controles [v w] para el modelo de uniciclo.');
end

if ~isstruct(robot.geometria) || ...
        ~isfield(robot.geometria,'radio') || ...
        ~es_escalar_positivo(robot.geometria.radio)
    error('apf:RadioRobotNoValido', ...
        'robot.geometria.radio debe ser positivo.');
end

camposLimitesRobot = {'vMin','vMax','wMin','wMax'};

if ~isstruct(robot.limites)
    error('apf:LimitesRobotNoValidos', ...
        'robot.limites debe ser una estructura.');
end

for i = 1:numel(camposLimitesRobot)
    if ~isfield(robot.limites,camposLimitesRobot{i}) || ...
            ~es_escalar_finito(robot.limites.(camposLimitesRobot{i}))
        error('apf:LimiteRobotAusente', ...
            'Falta un valor valido en robot.limites.%s.', ...
            camposLimitesRobot{i});
    end
end

if robot.limites.vMin < 0 || ...
        robot.limites.vMax <= robot.limites.vMin
    error('apf:LimitesLinealesNoValidos', ...
        'Los limites de velocidad lineal no son coherentes.');
end

if robot.limites.wMin >= 0 || ...
        robot.limites.wMax <= 0 || ...
        robot.limites.wMax <= robot.limites.wMin
    error('apf:LimitesAngularesNoValidos', ...
        'Los limites de velocidad angular no son coherentes.');
end

if ~isnumeric(robot.controlParada) || ...
        ~isreal(robot.controlParada) || ...
        numel(robot.controlParada) ~= 2 || ...
        any(~isfinite(robot.controlParada(:)))
    error('apf:ControlParadaNoValido', ...
        'robot.controlParada debe ser un vector [v w] finito.');
end

%% Configuracion general
if ~isstruct(cfg) || ~isscalar(cfg)
    error('apf:ConfiguracionNoValida', ...
        'cfg debe proceder de parametros_generales.m.');
end

camposCfg = {'sim','navegacion','seguridad','prediccion','apf'};

for i = 1:numel(camposCfg)
    if ~isfield(cfg,camposCfg{i}) || ~isstruct(cfg.(camposCfg{i}))
        error('apf:ConfiguracionIncompleta', ...
            'Falta la estructura cfg.%s.',camposCfg{i});
    end
end

parametros = struct();
parametros.Ts = obtener_positivo(cfg.sim,'Ts','cfg.sim.Ts');
parametros.toleranciaWaypoint = obtener_no_negativo( ...
    cfg.navegacion,'tolWaypoint','cfg.navegacion.tolWaypoint');
parametros.margenEstatico = obtener_no_negativo( ...
    cfg.seguridad,'margenEstatico','cfg.seguridad.margenEstatico');
parametros.margenDinamico = obtener_no_negativo( ...
    cfg.seguridad,'margenDinamico','cfg.seguridad.margenDinamico');
parametros.pasosPrediccion = obtener_entero_no_negativo( ...
    cfg.prediccion,'pasos','cfg.prediccion.pasos');

if ~isfield(cfg.prediccion,'modelo') || ...
        strlength(strtrim(string(cfg.prediccion.modelo))) == 0
    error('apf:ModeloPrediccionAusente', ...
        'Falta cfg.prediccion.modelo.');
end

parametros.modeloPrediccion = ...
    lower(strtrim(string(cfg.prediccion.modelo)));

if parametros.modeloPrediccion ~= "velocidad_constante"
    error('apf:ModeloPrediccionNoSoportado', ...
        'Solo se admite el modelo "velocidad_constante".');
end

parametros.kAtractivo = obtener_positivo( ...
    cfg.apf,'kAtractivo','cfg.apf.kAtractivo');
parametros.kRepulsivo = obtener_positivo( ...
    cfg.apf,'kRepulsivo','cfg.apf.kRepulsivo');
parametros.radioInfluencia = obtener_positivo( ...
    cfg.apf,'radioInfluencia','cfg.apf.radioInfluencia');
parametros.kGiro = obtener_positivo( ...
    cfg.apf,'kGiro','cfg.apf.kGiro');
parametros.distanciaFrenado = obtener_positivo( ...
    cfg.apf,'distanciaFrenado','cfg.apf.distanciaFrenado');
parametros.anguloParada = obtener_positivo( ...
    cfg.apf,'anguloParada','cfg.apf.anguloParada');

parametros.factorRepulsionDinamica = 1.0;

if isfield(cfg.apf,'factorRepulsionDinamica')
    valorFactorDinamico = cfg.apf.factorRepulsionDinamica;

    if ~isnumeric(valorFactorDinamico) || ...
            ~isscalar(valorFactorDinamico) || ...
            ~isreal(valorFactorDinamico) || ...
            ~isfinite(valorFactorDinamico) || ...
            valorFactorDinamico < 0 || valorFactorDinamico > 1
        error('apf:FactorRepulsionDinamicaNoValido', ...
            ['cfg.apf.factorRepulsionDinamica debe pertenecer ' ...
             'al intervalo [0,1].']);
    end

    parametros.factorRepulsionDinamica = ...
        double(valorFactorDinamico);
end

if parametros.anguloParada > pi
    error('apf:AnguloParadaNoValido', ...
        'cfg.apf.anguloParada debe pertenecer al intervalo (0,pi].');
end

parametros.radioRobot = double(robot.geometria.radio);
parametros.separacionEstatica = ...
    parametros.radioRobot+parametros.margenEstatico;

parametros.vMin = double(robot.limites.vMin);
parametros.vMax = double(robot.limites.vMax);
parametros.wMin = double(robot.limites.wMin);
parametros.wMax = double(robot.limites.wMax);
parametros.controlParada = ...
    reshape(double(robot.controlParada),1,2);

if limites(2)-limites(1) <= 2*parametros.separacionEstatica || ...
        limites(4)-limites(3) <= 2*parametros.separacionEstatica
    error('apf:MapaSinEspacioUtil', ...
        'El radio del robot y el margen eliminan la region navegable.');
end
end

function valor = obtener_positivo(estructura,campo,nombre)
%OBTENER_POSITIVO Recupera un escalar positivo.

if ~isfield(estructura,campo) || ...
        ~es_escalar_positivo(estructura.(campo))
    error('apf:EscalarPositivoNoValido', ...
        '%s debe ser un escalar positivo.',nombre);
end

valor = double(estructura.(campo));
end

function valor = obtener_no_negativo(estructura,campo,nombre)
%OBTENER_NO_NEGATIVO Recupera un escalar no negativo.

if ~isfield(estructura,campo) || ...
        ~es_escalar_no_negativo(estructura.(campo))
    error('apf:EscalarNoNegativoNoValido', ...
        '%s debe ser un escalar no negativo.',nombre);
end

valor = double(estructura.(campo));
end

function valor = obtener_entero_no_negativo(estructura,campo,nombre)
%OBTENER_ENTERO_NO_NEGATIVO Recupera un entero no negativo.

valor = obtener_no_negativo(estructura,campo,nombre);

if valor ~= floor(valor)
    error('apf:EnteroNoNegativoNoValido', ...
        '%s debe ser entero.',nombre);
end
end

function tf = es_escalar_positivo(valor)
%ES_ESCALAR_POSITIVO Comprueba un escalar real, finito y positivo.

tf = isnumeric(valor) && isreal(valor) && isscalar(valor) && ...
    isfinite(valor) && valor > 0;
end

function tf = es_escalar_no_negativo(valor)
%ES_ESCALAR_NO_NEGATIVO Comprueba un escalar real, finito y no negativo.

tf = isnumeric(valor) && isreal(valor) && isscalar(valor) && ...
    isfinite(valor) && valor >= 0;
end

function tf = es_escalar_finito(valor)
%ES_ESCALAR_FINITO Comprueba un escalar real y finito.

tf = isnumeric(valor) && isreal(valor) && isscalar(valor) && ...
    isfinite(valor);
end
