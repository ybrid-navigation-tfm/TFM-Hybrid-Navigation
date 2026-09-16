function cfg = parametros_generales(modo)
% Parametros Generales. Configuracion (cfg) comun del entorno experimental:
%
%   cfg = PARAMETROS_GENERALES("visual") activa la animacion del Live Editor.
%   cfg = PARAMETROS_GENERALES("batch") desactiva graficos y pausas.
%
%   El escenario (limites, inicio, meta y obstaculos) se define en
%   escenarios.m. El modelo y los limites fisicos del robot se definen en
%   configuracion_robot.m.

if nargin < 1 || strlength(string(modo)) == 0
    modo = "visual";
end

modo = lower(string(modo));
if ~any(modo == ["visual", "batch"])
    error('parametros_generales:ModoNoValido', ...
        'El modo debe ser "visual" o "batch".');
end

cfg = struct();
cfg.version = "0.1";
cfg.modo = modo;

%% Reproducibilidad
cfg.semilla = 7;

%% Simulacion comun
cfg.sim.Ts = 0.15;   % [s] periodo de muestreo comun

%% Criterios de navegacion
% Se utilizan durante la navegacion, pero no determinan por si solos
% si la ejecucion completa se considera exitosa.
cfg.navegacion.radioMeta = 0.60;       % [m]
cfg.navegacion.lookahead = 1.50;       % [m]
cfg.navegacion.tolWaypoint = 0.50;     % [m]

%% Criterios de terminacion de cada ejecucion
% Tiempo simulado, no tiempo real de calculo de MATLAB.
% Se limita cada ejecucion a 1000 actualizaciones del modelo.
cfg.terminacion.maxPasos = 1000;
cfg.terminacion.tMaxSimulado = ...
    cfg.terminacion.maxPasos*cfg.sim.Ts;   % 150 s simulados

cfg.terminacion.detenerEnMeta = true;

% Las colisiones se registran, pero no terminan la simulacion.
cfg.terminacion.detenerEnColision = false;

%% Definicion experimental de exito
cfg.evaluacion.exito.requiereMeta = true;
cfg.evaluacion.exito.requiereSinColision = true;
cfg.evaluacion.exito.requiereDentroDelTiempo = true;

%% Seguridad y prediccion de obstaculos
% Se usan los mismos margenes para todas las arquitecturas.
cfg.seguridad.margenEstatico = 0.15;   % [m]
cfg.seguridad.margenDinamico = 0.10;   % [m]

% Horizonte local moderado. RRT* no utiliza los dinamicos en esta
% arquitectura; esta prediccion se reserva para APF.
cfg.prediccion.pasos = 4;
cfg.prediccion.modelo = "velocidad_constante";

%% Replanificacion comun
% El mismo criterio se aplicara a RRT*+APF, RRT*+MPC y PRM+MPC.
cfg.replan.modo = "bloqueo_o_periodico";
cfg.replan.periodo = 10;           % [pasos]
cfg.replan.reintento = 1;          % reintento en el siguiente paso

% Los obstaculos dinamicos se gestionan localmente mediante APF. El main
% entrega al planificador y al criterio global una lista dinamica vacia.
cfg.replan.considerarDinamicos = false;

%% RRT* convencional
cfg.rrt.maxIter = 1600;
cfg.rrt.paso = 0.70;               % [m]
cfg.rrt.radioVecinos = 2.30;       % [m]
cfg.rrt.sesgoMeta = 0.12;

%% PRM persistente
cfg.prm.nMuestras = 340;
cfg.prm.radioConexion = 3.20;      % [m]
cfg.prm.maxVecinos = 18;
cfg.prm.radioConsulta = 4.20;      % [m]
cfg.prm.vecinosConsulta = 24;
cfg.prm.ampliarTrasFallos = 20;
cfg.prm.muestrasExpansion = 70;
cfg.prm.maxExpansiones = 1;

%% MPC discreto de referencia
% Primera version: conserva la busqueda discreta del codigo funcional.
cfg.mpc.Np = 12;
cfg.mpc.nVelocidades = 9;
cfg.mpc.nGiros = 12;
cfg.mpc.Qmeta = 10;
cfg.mpc.Qcamino = 3;
cfg.mpc.RdeltaU = 0.15;
cfg.mpc.Qobstaculo = 45;
cfg.mpc.penalizacionColision = 1e6;
cfg.mpc.umbralEstatico = 0.20;     % [m]
cfg.mpc.umbralDinamico = 0.25;    % [m]

%% APF convencional como controlador local
% No se incluye apfBias: el APF no modificara el crecimiento de RRT*.
% Ajuste orientado a mantener el avance. La proteccion final frente a
% paredes y obstaculos estaticos se realiza tambien en el main rechazando
% exclusivamente el paso candidato que invadiria su geometria.
cfg.apf.kAtractivo = 1.35;
cfg.apf.kRepulsivo = 1.00;
cfg.apf.radioInfluencia = 0.95;    % [m]
cfg.apf.kGiro = 2.00;
cfg.apf.distanciaFrenado = 0.60;   % [m]
cfg.apf.anguloParada = 2*pi/3;     % [rad]

% Los dinamicos pueden producir contacto y espera. Su repulsion se reduce
% para que no invalide continuamente el avance global.
cfg.apf.factorRepulsionDinamica = 0.55;

%% Visualizacion
cfg.visual.activa = modo == "visual";
cfg.visual.pausa = modo == "visual";
cfg.visual.tPausa = cfg.sim.Ts;
cfg.visual.actualizarCada = 1;
cfg.visual.mostrarPlanificador = true;

%% Registro de metricas
cfg.metricas.guardarTrayectoria = true;
cfg.metricas.guardarControles = true;
cfg.metricas.guardarObstaculos = true;
cfg.metricas.guardarTiemposCiclo = true;
cfg.metricas.guardarIndicadorExito = true;
cfg.metricas.calcularTasaExito = true;
cfg.metricas.distanciaRiesgo = 0.25;  % [m], valor inicial revisable

%% Experimentos estadisticos
cfg.batch.nRepeticiones = 30;
cfg.batch.usarParfor = false;

%% Modulos seleccionados por cada main
cfg.nombreArquitectura = "";
cfg.planificador = [];
cfg.controlador = [];
end
