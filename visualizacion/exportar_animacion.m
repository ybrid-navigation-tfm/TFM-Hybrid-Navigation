function [exportacion, info] = exportar_animacion( ...
    accion, exportacion, graficos, cfg, archivoSalida, opciones)
% Exporta la animacion de una simulacion a GIF, MP4 o AVI.
%
%   [exportacion,info] = EXPORTAR_ANIMACION( ...
%       "iniciar",[],graficos,cfg,archivoSalida)
%
%   [exportacion,info] = EXPORTAR_ANIMACION( ...
%       "capturar",exportacion,graficos)
%
%   [exportacion,info] = EXPORTAR_ANIMACION( ...
%       "finalizar",exportacion)
%
%   [exportacion,info] = EXPORTAR_ANIMACION( ...
%       "cancelar",exportacion)
%
%   Este modulo permite guardar externamente la animacion mostrada en la
%   figura comun del proyecto. No sustituye la reproduccion integrada del
%   Live Editor: ambas posibilidades pueden utilizarse simultaneamente.
%
%   El funcionamiento se divide en cuatro acciones:
%
%       "iniciar"
%           Prepara el archivo de salida y, para MP4 o AVI, abre un objeto
%           VideoWriter. No captura todavia ningun fotograma.
%
%       "capturar"
%           Captura el contenido completo de graficos.figura mediante
%           getframe y lo escribe inmediatamente en el archivo. Los
%           fotogramas no se acumulan en memoria.
%
%       "finalizar"
%           Cierra correctamente el archivo de video y devuelve un resumen
%           de la exportacion.
%
%       "cancelar"
%           Cierra el escritor, si existe, y elimina el archivo parcial.
%
%   Formatos admitidos:
%
%       .gif    GIF animado con repeticion configurable.
%       .mp4    Video MPEG-4 mediante VideoWriter.
%       .m4v    Video MPEG-4 mediante VideoWriter.
%       .avi    Video Motion JPEG AVI mediante VideoWriter.
%
%   El formato se determina mediante la extension de archivoSalida. Si no
%   se proporciona extension, se utiliza .mp4.
%
%   Entradas:
%       accion
%           Texto escalar: "iniciar", "capturar", "finalizar" o
%           "cancelar".
%
%       exportacion
%           Estado devuelto por la llamada anterior. En la accion
%           "iniciar" debe proporcionarse [] o una estructura que no
%           contenga una exportacion abierta.
%
%       graficos
%           Estructura creada por inicializar_figura.m y completada por
%           dibujar_entorno.m y dibujar_robot.m. Es necesaria para
%           "iniciar" y "capturar".
%
%       cfg
%           Estructura obtenida mediante parametros_generales.m. Solo es
%           necesaria durante "iniciar". Se utilizan:
%
%               cfg.sim.Ts
%               cfg.visual.actualizarCada
%
%           Cuando no se indica una frecuencia explicita, la velocidad de
%           reproduccion se calcula como:
%
%               fps = factorVelocidad / ...
%                     (cfg.sim.Ts*cfg.visual.actualizarCada)
%
%       archivoSalida
%           Ruta del archivo final. Solo se utiliza durante "iniciar".
%           Si se omite, se crea automaticamente un archivo MP4 dentro de:
%
%               <raiz del proyecto>/salidas/animaciones/
%
%       opciones
%           Estructura opcional utilizada durante "iniciar". Campos:
%
%               .activa
%                   true por defecto. Si es false, el modulo se convierte
%                   en una operacion nula.
%
%               .sobrescribir
%                   true por defecto. Permite sustituir un archivo previo.
%
%               .fps
%                   Frecuencia explicita de reproduccion. Si se deja vacia,
%                   se calcula a partir de cfg.sim.Ts.
%
%               .factorVelocidad
%                   Factor respecto al tiempo simulado. Valor 1 reproduce
%                   aproximadamente a velocidad real; 2 duplica la rapidez.
%
%               .calidad
%                   Calidad de MP4 o Motion JPEG AVI en [0,100].
%                   Valor por defecto: 90.
%
%               .bucleGif
%                   Numero de repeticiones del GIF. Inf produce un bucle
%                   continuo. Valor por defecto: Inf.
%
%               .capturarSoloActualizaciones
%                   Si es true, se omiten llamadas en las que
%                   graficos.actualizacionRealizada sea false. Valor por
%                   defecto: true.
%
%               .bloquearRedimensionado
%                   Si es true, desactiva temporalmente el cambio de tamano
%                   de la figura para conservar dimensiones constantes.
%                   Valor por defecto: true.
%
%               .eliminarSiVacio
%                   Elimina el archivo si se finaliza sin ningun fotograma.
%                   Valor por defecto: true.
%
%               .mostrarMensajes
%                   Muestra mensajes breves en la ventana de comandos.
%                   Valor por defecto: false.
%
%   Salida exportacion:
%       Estructura que conserva el estado entre llamadas:
%
%           .activa
%           .estado
%           .archivo
%           .formato
%           .fps
%           .retardoGif
%           .numeroFotogramas
%           .tamanoFotograma
%           .escritor
%           .mapaGif
%           .figura
%
%   Salida info:
%       Resumen de la accion realizada:
%
%           .accion
%           .realizada
%           .motivo
%           .archivo
%           .formato
%           .numeroFotogramas
%           .duracionEstimada
%           .archivoExiste
%           .tamanoArchivoBytes
%
%   Orden recomendado dentro del main:
%
%       %% Inicializacion
%       opcionesExportacion = struct();
%       opcionesExportacion.factorVelocidad = 1;
%
%       rutaVideo = fullfile( ...
%           "salidas","animaciones", ...
%           "rrt_mpc_media_semilla_7.mp4");
%
%       [exportacion,~] = exportar_animacion( ...
%           "iniciar",[],graficos,cfg,rutaVideo, ...
%           opcionesExportacion);
%
%       %% Dentro del bucle, fuera del cronometro computacional
%       graficos = actualizar_graficos(...);
%       drawnow;
%
%       [exportacion,~] = exportar_animacion( ...
%           "capturar",exportacion,graficos);
%
%       %% Al terminar la simulacion
%       [exportacion,infoExportacion] = exportar_animacion( ...
%           "finalizar",exportacion);
%
%   IMPORTANTE:
%       - La captura debe realizarse DESPUES de drawnow, para que el
%         fotograma contenga el estado grafico ya renderizado.
%       - La captura y la escritura deben permanecer fuera del cronometro
%         usado para comparar el coste computacional de APF y MPC.
%       - Este modulo no modifica el arbol ni la roadmap. El archivo recoge
%         exactamente lo que muestra actualizar_graficos.m; por tanto, si
%         dicho modulo ha sustituido el planificador anterior, en el video
%         solo aparece la representacion vigente.

if nargin < 2 || isempty(exportacion)
    exportacion = struct();
end

if nargin < 3
    graficos = [];
end

if nargin < 4
    cfg = [];
end

if nargin < 5
    archivoSalida = "";
end

if nargin < 6 || isempty(opciones)
    opciones = struct();
end

accion = validar_accion(accion);
info = crear_info(accion,exportacion);

switch accion
    case "iniciar"
        [exportacion,info] = iniciar_exportacion( ...
            exportacion,graficos,cfg,archivoSalida,opciones,info);

    case "capturar"
        [exportacion,info] = capturar_fotograma( ...
            exportacion,graficos,info);

    case "finalizar"
        [exportacion,info] = finalizar_exportacion( ...
            exportacion,info,false);

    case "cancelar"
        [exportacion,info] = finalizar_exportacion( ...
            exportacion,info,true);
end
end

%% ========================================================================
% INICIALIZACION
% ========================================================================

function [exportacion,info] = iniciar_exportacion( ...
    exportacionAnterior,graficos,cfg,archivoSalida,opciones,info)
%INICIAR_EXPORTACION Crea el estado y abre el escritor correspondiente.

if es_exportacion_abierta(exportacionAnterior)
    error('exportar_animacion:ExportacionYaAbierta', ...
        ['Ya existe una exportacion abierta. Finalicela o cancelela ' ...
         'antes de iniciar otra.']);
end

[graficos,cfg] = validar_inicio(graficos,cfg);
opciones = normalizar_opciones(opciones,cfg);

exportacion = estructura_exportacion_vacia();
exportacion.opciones = opciones;
exportacion.activa = opciones.activa && graficos.activa;
exportacion.figura = graficos.figura;

if ~exportacion.activa
    exportacion.estado = "inactivo";
    info = completar_info(info,exportacion);
    info.motivo = "exportacion_desactivada_o_modo_batch";
    return;
end

[archivo,formato] = preparar_archivo_salida( ...
    archivoSalida,graficos,opciones.sobrescribir);

exportacion.archivo = archivo;
exportacion.formato = formato;
exportacion.fps = opciones.fps;
exportacion.retardoGif = 1/opciones.fps;
exportacion.estado = "abierto";
exportacion.fechaInicio = datetime('now');

%% Conservacion de un tamano constante de la figura
if opciones.bloquearRedimensionado
    try
        exportacion.resizeOriginal = ...
            string(get(graficos.figura,'Resize'));

        set(graficos.figura,'Resize','off');
        exportacion.redimensionadoBloqueado = true;
    catch
        % La exportacion puede continuar aunque esta propiedad no se pueda
        % modificar en una version o entorno grafico concreto.
        exportacion.resizeOriginal = "";
        exportacion.redimensionadoBloqueado = false;
    end
end

%% Apertura del escritor
try
    switch formato
        case {"mp4","m4v"}
            escritor = VideoWriter(char(archivo),'MPEG-4');
            escritor.FrameRate = opciones.fps;
            escritor.Quality = opciones.calidad;
            open(escritor);
            exportacion.escritor = escritor;

        case "avi"
            escritor = VideoWriter( ...
                char(archivo),'Motion JPEG AVI');
            escritor.FrameRate = opciones.fps;
            escritor.Quality = opciones.calidad;
            open(escritor);
            exportacion.escritor = escritor;

        case "gif"
            % El GIF se escribe directamente mediante imwrite cuando se
            % recibe el primer fotograma.
            exportacion.escritor = [];

        otherwise
            error('exportar_animacion:FormatoNoSoportado', ...
                'Formato de exportacion no soportado: %s.',formato);
    end
catch errorOriginal
    restaurar_redimensionado(exportacion);

    if exist(char(archivo),'file') == 2
        eliminar_archivo_silencioso(archivo);
    end

    error('exportar_animacion:NoSePudoIniciar', ...
        ['No se pudo iniciar la exportacion en formato %s. ' ...
         'Mensaje original: %s'], ...
        formato,errorOriginal.message);
end

info.realizada = true;
info.motivo = "exportacion_iniciada";
info = completar_info(info,exportacion);

if opciones.mostrarMensajes
    fprintf('Exportacion iniciada: %s\n',char(archivo));
end
end

%% ========================================================================
% CAPTURA
% ========================================================================

function [exportacion,info] = capturar_fotograma( ...
    exportacion,graficos,info)
%CAPTURAR_FOTOGRAMA Captura la figura y escribe un unico fotograma.

exportacion = validar_exportacion(exportacion);

if ~exportacion.activa || exportacion.estado == "inactivo"
    info = completar_info(info,exportacion);
    info.motivo = "exportacion_inactiva";
    return;
end

if exportacion.estado ~= "abierto"
    error('exportar_animacion:ExportacionNoAbierta', ...
        ['La accion "capturar" requiere una exportacion abierta. ' ...
         'Estado actual: %s.'],exportacion.estado);
end

graficos = validar_graficos_captura(graficos,exportacion);

if exportacion.opciones.capturarSoloActualizaciones && ...
        isfield(graficos,'actualizacionRealizada') && ...
        ~logical(graficos.actualizacionRealizada)

    info = completar_info(info,exportacion);
    info.motivo = "sin_actualizacion_grafica";
    return;
end

%% Captura de la figura completa
try
    fotograma = getframe(graficos.figura);
catch errorOriginal
    error('exportar_animacion:CapturaFallida', ...
        ['No se pudo capturar la figura. Compruebe que permanece ' ...
         'abierta, visible y que drawnow se ejecuto antes de la captura. ' ...
         'Mensaje original: %s'],errorOriginal.message);
end

if ~isstruct(fotograma) || ...
        ~isfield(fotograma,'cdata') || ...
        isempty(fotograma.cdata)
    error('exportar_animacion:FotogramaVacio', ...
        'getframe no devolvio datos de imagen validos.');
end

alto = size(fotograma.cdata,1);
ancho = size(fotograma.cdata,2);

if exportacion.numeroFotogramas == 0
    exportacion.tamanoFotograma = [alto ancho];
else
    if ~isequal(exportacion.tamanoFotograma,[alto ancho])
        error('exportar_animacion:TamanoFotogramaVariable', ...
            ['El tamano del fotograma cambio de [%d %d] a [%d %d]. ' ...
             'No redimensione la figura durante la exportacion.'], ...
            exportacion.tamanoFotograma(1), ...
            exportacion.tamanoFotograma(2),alto,ancho);
    end
end

%% Escritura inmediata
try
    switch exportacion.formato
        case {"mp4","m4v","avi"}
            writeVideo(exportacion.escritor,fotograma);

        case "gif"
            exportacion = escribir_fotograma_gif( ...
                exportacion,fotograma.cdata);
    end
catch errorOriginal
    error('exportar_animacion:EscrituraFallida', ...
        ['No se pudo escribir el fotograma %d en %s. ' ...
         'Mensaje original: %s'], ...
        exportacion.numeroFotogramas+1, ...
        char(exportacion.archivo),errorOriginal.message);
end

exportacion.numeroFotogramas = ...
    exportacion.numeroFotogramas+1;

exportacion.fechaUltimaCaptura = datetime('now');

info.realizada = true;
info.motivo = "fotograma_escrito";
info = completar_info(info,exportacion);
end

function exportacion = escribir_fotograma_gif(exportacion,imagenRGB)
%ESCRIBIR_FOTOGRAMA_GIF Escribe el primer fotograma o anexa el siguiente.
%
% Se utiliza una paleta fija de 256 colores (8 niveles de rojo, 8 de verde
% y 4 de azul). De este modo no se requiere rgb2ind ni Image Processing
% Toolbox y todos los fotogramas comparten exactamente el mismo mapa.

archivo = char(exportacion.archivo);
retardo = exportacion.retardoGif;

[imagenIndexada,mapa] = cuantizar_gif_256(imagenRGB);

if exportacion.numeroFotogramas == 0
    imwrite( ...
        imagenIndexada,mapa,archivo,'gif', ...
        'LoopCount',exportacion.opciones.bucleGif, ...
        'DelayTime',retardo, ...
        'DisposalMethod','doNotSpecify');

    exportacion.mapaGif = mapa;
else
    imwrite( ...
        imagenIndexada,exportacion.mapaGif,archivo,'gif', ...
        'WriteMode','append', ...
        'DelayTime',retardo, ...
        'DisposalMethod','doNotSpecify');
end
end

function [imagenIndexada,mapa] = cuantizar_gif_256(imagenRGB)
%CUANTIZAR_GIF_256 Convierte una imagen RGB a una paleta fija 8x8x4.

if ~isnumeric(imagenRGB) || ~isreal(imagenRGB) || ...
        ndims(imagenRGB) ~= 3 || size(imagenRGB,3) ~= 3 || ...
        any(~isfinite(double(imagenRGB(:))))
    error('exportar_animacion:ImagenGifNoValida', ...
        'El fotograma GIF debe ser una imagen RGB numerica y finita.');
end

if isa(imagenRGB,'uint8')
    rgb = imagenRGB;
else
    rgbDouble = double(imagenRGB);

    if all(rgbDouble(:) >= 0) && all(rgbDouble(:) <= 1)
        rgb = uint8(round(255*rgbDouble));
    else
        rgb = uint8(round(min(max(rgbDouble,0),255)));
    end
end

nivelRojo  = floor(double(rgb(:,:,1))/32);  % 0 ... 7
nivelVerde = floor(double(rgb(:,:,2))/32);  % 0 ... 7
nivelAzul  = floor(double(rgb(:,:,3))/64);  % 0 ... 3

% En imagenes indexadas uint8, el valor 0 referencia la primera fila del
% mapa y el valor 255 la fila 256.
imagenIndexada = uint8( ...
    32*nivelRojo + 4*nivelVerde + nivelAzul);

mapa = zeros(256,3);
fila = 1;

for rojo = 0:7
    for verde = 0:7
        for azul = 0:3
            mapa(fila,:) = [ ...
                (rojo+0.5)/8, ...
                (verde+0.5)/8, ...
                (azul+0.5)/4];
            fila = fila+1;
        end
    end
end
end

%% ========================================================================
% FINALIZACION Y CANCELACION
% ========================================================================

function [exportacion,info] = finalizar_exportacion( ...
    exportacion,info,cancelar)
%FINALIZAR_EXPORTACION Cierra el escritor y consolida el archivo.

exportacion = validar_exportacion(exportacion);

if exportacion.estado == "inactivo"
    info = completar_info(info,exportacion);

    if cancelar
        info.motivo = "exportacion_inactiva_cancelada";
    else
        info.motivo = "exportacion_inactiva_finalizada";
    end
    return;
end

if any(exportacion.estado == ["finalizado","cancelado"])
    info = completar_info(info,exportacion);
    info.motivo = "exportacion_ya_cerrada";
    return;
end

if exportacion.estado ~= "abierto"
    error('exportar_animacion:EstadoNoValido', ...
        'Estado de exportacion no reconocido: %s.',exportacion.estado);
end

%% Cierre del escritor de video
if any(exportacion.formato == ["mp4","m4v","avi"]) && ...
        ~isempty(exportacion.escritor)
    try
        close(exportacion.escritor);
    catch errorOriginal
        restaurar_redimensionado(exportacion);

        error('exportar_animacion:CierreFallido', ...
            ['No se pudo cerrar correctamente el archivo %s. ' ...
             'Mensaje original: %s'], ...
            char(exportacion.archivo),errorOriginal.message);
    end
end

exportacion.escritor = [];
restaurar_redimensionado(exportacion);

if cancelar
    eliminar_archivo_silencioso(exportacion.archivo);
    exportacion.estado = "cancelado";
    exportacion.fechaFinalizacion = datetime('now');

    info.realizada = true;
    info.motivo = "exportacion_cancelada_y_archivo_eliminado";
else
    if exportacion.numeroFotogramas == 0 && ...
            exportacion.opciones.eliminarSiVacio
        eliminar_archivo_silencioso(exportacion.archivo);
    end

    exportacion.estado = "finalizado";
    exportacion.fechaFinalizacion = datetime('now');

    info.realizada = true;

    if exportacion.numeroFotogramas == 0
        info.motivo = "exportacion_finalizada_sin_fotogramas";
    else
        info.motivo = "exportacion_finalizada";
    end
end

info = completar_info(info,exportacion);

if exportacion.opciones.mostrarMensajes
    if cancelar
        fprintf('Exportacion cancelada.\n');
    elseif info.archivoExiste
        fprintf('Animacion exportada: %s\n', ...
            char(exportacion.archivo));
    end
end
end

function restaurar_redimensionado(exportacion)
%RESTAURAR_REDIMENSIONADO Restaura la propiedad Resize de la figura.

if ~isfield(exportacion,'redimensionadoBloqueado') || ...
        ~exportacion.redimensionadoBloqueado || ...
        ~isfield(exportacion,'figura') || ...
        ~isgraphics(exportacion.figura,'figure') || ...
        ~isfield(exportacion,'resizeOriginal') || ...
        strlength(exportacion.resizeOriginal) == 0
    return;
end

try
    set(exportacion.figura,'Resize', ...
        char(exportacion.resizeOriginal));
catch
    % No se interrumpe el cierre del archivo por un fallo cosmetico.
end
end

%% ========================================================================
% CONFIGURACION Y RUTA
% ========================================================================

function opciones = normalizar_opciones(opciones,cfg)
%NORMALIZAR_OPCIONES Aplica valores por defecto y valida las opciones.

if ~isstruct(opciones) || ~isscalar(opciones)
    error('exportar_animacion:OpcionesNoValidas', ...
        'opciones debe ser una estructura escalar.');
end

camposPermitidos = { ...
    'activa', ...
    'sobrescribir', ...
    'fps', ...
    'factorVelocidad', ...
    'calidad', ...
    'bucleGif', ...
    'capturarSoloActualizaciones', ...
    'bloquearRedimensionado', ...
    'eliminarSiVacio', ...
    'mostrarMensajes'};

camposRecibidos = fieldnames(opciones);

for i = 1:numel(camposRecibidos)
    if ~ismember(camposRecibidos{i},camposPermitidos)
        error('exportar_animacion:OpcionDesconocida', ...
            'Opcion no reconocida: opciones.%s.', ...
            camposRecibidos{i});
    end
end

defectos = struct();
defectos.activa = true;
defectos.sobrescribir = true;
defectos.fps = [];
defectos.factorVelocidad = 1.0;
defectos.calidad = 90;
defectos.bucleGif = Inf;
defectos.capturarSoloActualizaciones = true;
defectos.bloquearRedimensionado = true;
defectos.eliminarSiVacio = true;
defectos.mostrarMensajes = false;

for i = 1:numel(camposPermitidos)
    campo = camposPermitidos{i};

    if ~isfield(opciones,campo) || isempty(opciones.(campo))
        opciones.(campo) = defectos.(campo);
    end
end

opciones.activa = validar_logico( ...
    opciones.activa,'opciones.activa');

opciones.sobrescribir = validar_logico( ...
    opciones.sobrescribir,'opciones.sobrescribir');

opciones.capturarSoloActualizaciones = validar_logico( ...
    opciones.capturarSoloActualizaciones, ...
    'opciones.capturarSoloActualizaciones');

opciones.bloquearRedimensionado = validar_logico( ...
    opciones.bloquearRedimensionado, ...
    'opciones.bloquearRedimensionado');

opciones.eliminarSiVacio = validar_logico( ...
    opciones.eliminarSiVacio,'opciones.eliminarSiVacio');

opciones.mostrarMensajes = validar_logico( ...
    opciones.mostrarMensajes,'opciones.mostrarMensajes');

if ~es_escalar_positivo(opciones.factorVelocidad)
    error('exportar_animacion:FactorVelocidadNoValido', ...
        'opciones.factorVelocidad debe ser positivo.');
end

opciones.factorVelocidad = ...
    double(opciones.factorVelocidad);

if isempty(opciones.fps)
    actualizarCada = 1;

    if isfield(cfg,'visual') && ...
            isstruct(cfg.visual) && ...
            isfield(cfg.visual,'actualizarCada')
        actualizarCada = cfg.visual.actualizarCada;
    end

    if ~es_entero_positivo(actualizarCada)
        error('exportar_animacion:CadenciaNoValida', ...
            'cfg.visual.actualizarCada debe ser un entero positivo.');
    end

    periodoFotograma = ...
        cfg.sim.Ts*double(actualizarCada);

    opciones.fps = ...
        opciones.factorVelocidad/periodoFotograma;
end

if ~es_escalar_positivo(opciones.fps)
    error('exportar_animacion:FpsNoValido', ...
        'opciones.fps debe ser un escalar positivo.');
end

opciones.fps = double(opciones.fps);

if ~isnumeric(opciones.calidad) || ...
        ~isscalar(opciones.calidad) || ...
        ~isreal(opciones.calidad) || ...
        ~isfinite(opciones.calidad) || ...
        opciones.calidad < 0 || opciones.calidad > 100
    error('exportar_animacion:CalidadNoValida', ...
        'opciones.calidad debe pertenecer al intervalo [0,100].');
end

opciones.calidad = round(double(opciones.calidad));

if ~(isnumeric(opciones.bucleGif) && ...
        isscalar(opciones.bucleGif) && ...
        isreal(opciones.bucleGif) && ...
        (isinf(opciones.bucleGif) || ...
         (isfinite(opciones.bucleGif) && ...
          opciones.bucleGif >= 0 && ...
          opciones.bucleGif <= 65535 && ...
          opciones.bucleGif == floor(opciones.bucleGif))))
    error('exportar_animacion:BucleGifNoValido', ...
        ['opciones.bucleGif debe ser Inf o un entero ' ...
         'comprendido entre 0 y 65535.']);
end

opciones.bucleGif = double(opciones.bucleGif);

retardoGif = 1/opciones.fps;

if retardoGif < 0 || retardoGif > 655
    error('exportar_animacion:RetardoGifNoValido', ...
        ['La frecuencia indicada produce un DelayTime de GIF fuera ' ...
         'del intervalo permitido [0,655] s.']);
end
end

function [archivo,formato] = preparar_archivo_salida( ...
    archivoSalida,graficos,sobrescribir)
%PREPARAR_ARCHIVO_SALIDA Normaliza la ruta y crea la carpeta necesaria.

archivoSalida = string(archivoSalida);

if ~isscalar(archivoSalida)
    error('exportar_animacion:ArchivoNoValido', ...
        'archivoSalida debe ser un texto escalar.');
end

archivoSalida = strtrim(archivoSalida);

if strlength(archivoSalida) == 0
    nombreBase = nombre_exportacion_automatico(graficos);
    archivoSalida = fullfile( ...
        carpeta_salida_predeterminada(),nombreBase+".mp4");
end

[carpeta,nombre,extension] = ...
    fileparts(char(archivoSalida));

if isempty(nombre)
    error('exportar_animacion:NombreArchivoNoValido', ...
        'archivoSalida debe contener un nombre de archivo.');
end

if isempty(extension)
    extension = '.mp4';
end

formato = lower(erase(string(extension),"."));

if ~any(formato == ["gif","mp4","m4v","avi"])
    error('exportar_animacion:ExtensionNoSoportada', ...
        ['Extension no soportada: %s. Utilice .gif, .mp4, ' ...
         '.m4v o .avi.'],extension);
end

if isempty(carpeta)
    carpeta = carpeta_salida_predeterminada();
end

if exist(carpeta,'dir') ~= 7
    [creada,mensaje] = mkdir(carpeta);

    if ~creada
        error('exportar_animacion:CarpetaNoCreada', ...
            'No se pudo crear la carpeta "%s": %s', ...
            carpeta,mensaje);
    end
end

archivo = string(fullfile( ...
    carpeta,[nombre char(extension)]));

if exist(char(archivo),'file') == 2
    if ~sobrescribir
        error('exportar_animacion:ArchivoExistente', ...
            ['El archivo ya existe y opciones.sobrescribir es false: ' ...
             '%s'],char(archivo));
    end

    eliminar_archivo_silencioso(archivo);

    if exist(char(archivo),'file') == 2
        error('exportar_animacion:ArchivoNoEliminado', ...
            'No se pudo sustituir el archivo existente: %s', ...
            char(archivo));
    end
end
end


function carpeta = carpeta_salida_predeterminada()
%CARPETA_SALIDA_PREDETERMINADA Evita depender del directorio de trabajo.
%
% El modulo se encuentra en <raiz>/visualizacion. Por tanto, la carpeta
% superior se toma como raiz del proyecto y la salida se guarda en
% <raiz>/salidas/animaciones, independientemente de desde donde se ejecute
% el Live Script.

carpetaModulo = fileparts(mfilename('fullpath'));
raizProyecto = fileparts(carpetaModulo);
carpeta = fullfile(raizProyecto,'salidas','animaciones');
end

function nombre = nombre_exportacion_automatico(graficos)
%NOMBRE_EXPORTACION_AUTOMATICO Construye un nombre legible y seguro.

arquitectura = "animacion";
escenario = "escenario";

if isfield(graficos,'nombreArquitectura') && ...
        strlength(strtrim(string(graficos.nombreArquitectura))) > 0
    arquitectura = ...
        strtrim(string(graficos.nombreArquitectura));
end

if isfield(graficos,'idEscenario') && ...
        strlength(strtrim(string(graficos.idEscenario))) > 0
    escenario = strtrim(string(graficos.idEscenario));
elseif isfield(graficos,'nombreEscenario') && ...
        strlength(strtrim(string(graficos.nombreEscenario))) > 0
    escenario = strtrim(string(graficos.nombreEscenario));
end

nombre = arquitectura+"_"+escenario;
nombre = lower(nombre);
nombre = regexprep(nombre,'[^a-zA-Z0-9_-]+','_');
nombre = regexprep(nombre,'_+','_');
nombre = regexprep(nombre,'^_+|_+$','');

if strlength(nombre) == 0
    nombre = "animacion";
end
end

%% ========================================================================
% VALIDACION
% ========================================================================

function accion = validar_accion(accion)
%VALIDAR_ACCION Comprueba la accion solicitada.

accion = lower(strtrim(string(accion)));

if ~isscalar(accion) || ...
        ~any(accion == ["iniciar","capturar","finalizar","cancelar"])
    error('exportar_animacion:AccionNoValida', ...
        ['accion debe ser "iniciar", "capturar", ' ...
         '"finalizar" o "cancelar".']);
end
end

function [graficos,cfg] = validar_inicio(graficos,cfg)
%VALIDAR_INICIO Comprueba la figura y la configuracion temporal.

if ~isstruct(graficos) || ~isscalar(graficos) || ...
        ~isfield(graficos,'activa')
    error('exportar_animacion:GraficosNoValidos', ...
        ['graficos debe ser la estructura obtenida mediante ' ...
         'inicializar_figura.m.']);
end

graficos.activa = validar_logico( ...
    graficos.activa,'graficos.activa');

if graficos.activa
    if ~isfield(graficos,'figura') || ...
            ~isscalar(graficos.figura) || ...
            ~isgraphics(graficos.figura,'figure')
        error('exportar_animacion:FiguraNoValida', ...
            'graficos.figura no contiene una figura abierta valida.');
    end
end

if ~isstruct(cfg) || ~isscalar(cfg) || ...
        ~isfield(cfg,'sim') || ...
        ~isstruct(cfg.sim) || ...
        ~isfield(cfg.sim,'Ts') || ...
        ~es_escalar_positivo(cfg.sim.Ts)
    error('exportar_animacion:ConfiguracionNoValida', ...
        ['cfg debe proceder de parametros_generales.m y contener ' ...
         'cfg.sim.Ts como escalar positivo.']);
end

cfg.sim.Ts = double(cfg.sim.Ts);
end

function graficos = validar_graficos_captura(graficos,exportacion)
%VALIDAR_GRAFICOS_CAPTURA Comprueba que se captura la figura esperada.

if ~isstruct(graficos) || ~isscalar(graficos) || ...
        ~isfield(graficos,'activa')
    error('exportar_animacion:GraficosCapturaNoValidos', ...
        'graficos debe contener una visualizacion activa.');
end

graficos.activa = validar_logico( ...
    graficos.activa,'graficos.activa');

if ~graficos.activa
    error('exportar_animacion:GraficosCapturaNoActivos', ...
        'No se puede capturar una visualizacion desactivada.');
end

if ~isfield(graficos,'figura') || ...
        ~isscalar(graficos.figura) || ...
        ~isgraphics(graficos.figura,'figure')
    error('exportar_animacion:FiguraCerrada', ...
        'La figura que se desea capturar no existe o fue cerrada.');
end

if ~isfield(exportacion,'figura') || ...
        ~isgraphics(exportacion.figura,'figure') || ...
        ~isequal(graficos.figura,exportacion.figura)
    error('exportar_animacion:FiguraDistinta', ...
        ['La figura recibida no coincide con la utilizada al iniciar ' ...
         'la exportacion.']);
end

if isfield(graficos,'actualizacionRealizada')
    graficos.actualizacionRealizada = validar_logico( ...
        graficos.actualizacionRealizada, ...
        'graficos.actualizacionRealizada');
end
end

function exportacion = validar_exportacion(exportacion)
%VALIDAR_EXPORTACION Comprueba el estado minimo entre llamadas.

if ~isstruct(exportacion) || ~isscalar(exportacion)
    error('exportar_animacion:EstadoNoValido', ...
        'exportacion debe ser una estructura escalar.');
end

campos = { ...
    'activa', ...
    'estado', ...
    'archivo', ...
    'formato', ...
    'fps', ...
    'retardoGif', ...
    'numeroFotogramas', ...
    'tamanoFotograma', ...
    'escritor', ...
    'mapaGif', ...
    'figura', ...
    'opciones'};

for i = 1:numel(campos)
    if ~isfield(exportacion,campos{i})
        error('exportar_animacion:EstadoIncompleto', ...
            'Falta exportacion.%s.',campos{i});
    end
end

exportacion.activa = validar_logico( ...
    exportacion.activa,'exportacion.activa');

exportacion.estado = ...
    lower(strtrim(string(exportacion.estado)));

if ~isscalar(exportacion.estado) || ...
        ~any(exportacion.estado == [ ...
            "inactivo","abierto","finalizado","cancelado"])
    error('exportar_animacion:EstadoDesconocido', ...
        'exportacion.estado no es valido.');
end

if ~es_entero_no_negativo(exportacion.numeroFotogramas)
    error('exportar_animacion:ContadorNoValido', ...
        'exportacion.numeroFotogramas debe ser un entero no negativo.');
end

exportacion.numeroFotogramas = ...
    double(exportacion.numeroFotogramas);
end

function tf = es_exportacion_abierta(exportacion)
%ES_EXPORTACION_ABIERTA Detecta un escritor previo sin finalizar.

tf = isstruct(exportacion) && ...
    isscalar(exportacion) && ...
    isfield(exportacion,'estado') && ...
    lower(strtrim(string(exportacion.estado))) == "abierto";
end

function valor = validar_logico(valor,nombre)
%VALIDAR_LOGICO Convierte un escalar logico o binario.

if islogical(valor) && isscalar(valor)
    return;
end

if isnumeric(valor) && isscalar(valor) && ...
        isreal(valor) && isfinite(valor) && ...
        (valor == 0 || valor == 1)
    valor = logical(valor);
    return;
end

error('exportar_animacion:IndicadorNoValido', ...
    '%s debe ser un escalar logico o binario.',nombre);
end

function tf = es_escalar_positivo(valor)
%ES_ESCALAR_POSITIVO Comprueba un escalar numerico real y positivo.

tf = isnumeric(valor) && isscalar(valor) && ...
    isreal(valor) && isfinite(valor) && valor > 0;
end

function tf = es_entero_positivo(valor)
%ES_ENTERO_POSITIVO Comprueba un entero estrictamente positivo.

tf = isnumeric(valor) && isscalar(valor) && ...
    isreal(valor) && isfinite(valor) && ...
    valor >= 1 && valor == floor(valor);
end

function tf = es_entero_no_negativo(valor)
%ES_ENTERO_NO_NEGATIVO Comprueba un entero mayor o igual que cero.

tf = isnumeric(valor) && isscalar(valor) && ...
    isreal(valor) && isfinite(valor) && ...
    valor >= 0 && valor == floor(valor);
end

%% ========================================================================
% ESTRUCTURAS DE SALIDA
% ========================================================================

function exportacion = estructura_exportacion_vacia()
%ESTRUCTURA_EXPORTACION_VACIA Define un contrato estable entre llamadas.

exportacion = struct();
exportacion.version = "1.0";
exportacion.activa = false;
exportacion.estado = "inactivo";

exportacion.archivo = "";
exportacion.formato = "";
exportacion.fps = NaN;
exportacion.retardoGif = NaN;

exportacion.numeroFotogramas = 0;
exportacion.tamanoFotograma = [NaN NaN];

exportacion.escritor = [];
exportacion.mapaGif = zeros(0,3);
exportacion.figura = gobjects(0);

exportacion.opciones = struct();

exportacion.redimensionadoBloqueado = false;
exportacion.resizeOriginal = "";

exportacion.fechaInicio = NaT;
exportacion.fechaUltimaCaptura = NaT;
exportacion.fechaFinalizacion = NaT;
end

function info = crear_info(accion,exportacion)
%CREAR_INFO Inicializa el diagnostico de la llamada actual.

info = struct();
info.accion = accion;
info.realizada = false;
info.motivo = "";

info.archivo = "";
info.formato = "";
info.numeroFotogramas = 0;
info.duracionEstimada = 0;

info.archivoExiste = false;
info.tamanoArchivoBytes = 0;

if isstruct(exportacion) && isscalar(exportacion)
    info = completar_info(info,exportacion);
end
end

function info = completar_info(info,exportacion)
%COMPLETAR_INFO Incorpora los datos actualmente disponibles.

if isfield(exportacion,'archivo')
    info.archivo = string(exportacion.archivo);
end

if isfield(exportacion,'formato')
    info.formato = string(exportacion.formato);
end

if isfield(exportacion,'numeroFotogramas') && ...
        es_entero_no_negativo(exportacion.numeroFotogramas)
    info.numeroFotogramas = ...
        double(exportacion.numeroFotogramas);
end

if isfield(exportacion,'fps') && ...
        es_escalar_positivo(exportacion.fps)
    info.duracionEstimada = ...
        info.numeroFotogramas/double(exportacion.fps);
end

if strlength(info.archivo) > 0 && ...
        exist(char(info.archivo),'file') == 2
    datosArchivo = dir(char(info.archivo));

    info.archivoExiste = true;

    if ~isempty(datosArchivo)
        info.tamanoArchivoBytes = ...
            double(datosArchivo(1).bytes);
    end
else
    info.archivoExiste = false;
    info.tamanoArchivoBytes = 0;
end
end

%% ========================================================================
% ARCHIVOS
% ========================================================================

function eliminar_archivo_silencioso(archivo)
%ELIMINAR_ARCHIVO_SILENCIOSO Intenta eliminar un archivo sin ocultar fallos.

archivo = string(archivo);

if strlength(archivo) == 0 || ...
        exist(char(archivo),'file') ~= 2
    return;
end

try
    delete(char(archivo));
catch errorOriginal
    warning('exportar_animacion:ArchivoNoEliminado', ...
        'No se pudo eliminar el archivo "%s": %s', ...
        char(archivo),errorOriginal.message);
end
end
