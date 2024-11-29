function ev3() %#codegen
    % Inicialización de conexión a la Raspberry Pi y cámara
    r = raspi('10.50.123.165', 'Rasp', 'Roboticos');
    w = webcam(r);
    
    % Parámetros de procesamiento
    fps = 85; % Tasa de cuadros por segundo de la cámara Raspberry Pi
    distanceBetweenLines = 2.915; % Distancia entre líneas en metros
    
    % Definir las posiciones de las líneas para la detección de cruce
    linearoja1 = [0, 20];
    linearoja2 = [700, 20];
    lineaazul1 = [0, 90];
    lineaazul2 = [700, 90];
    
    % Crear el detector de primer plano con configuraciones adaptadas
    detector = vision.ForegroundDetector('NumGaussians', 3, 'LearningRate', 0.005, 'MinimumBackgroundRatio', 0.7);
    se = strel('square', 3); % Elemento estructurante para morfología

    % Inicialización de variables para el cálculo de velocidad y seguimiento de objetos
    %prevCenter = [0, 0];
    centroidHistory = zeros(100, 2);
    centroidHistoryIdx = 1;
    isCrossing = false;
    startFrame = 0;
    %endFrame = 0;
    frameCount = 0;
    
    % Bucle principal para capturar y procesar imágenes en tiempo real
    for k = 1:200
        img = snapshot(w);  % Capturar imagen desde la cámara Raspberry Pi
        frameCount = frameCount + 1;
        
        % Detectar el primer plano (sustracción de fondo)
        foregroundMask = step(detector, img);
        
        % Aplicar operaciones morfológicas para eliminar ruido pequeño
        cleanForeground = imopen(foregroundMask, se);
        cleanForeground = bwareaopen(cleanForeground, 75); % Filtrar áreas pequeñas

        % Obtener propiedades de las regiones detectadas
        stats = regionprops(cleanForeground, 'Centroid', 'BoundingBox');
        
        % Procesamiento de los centroides de los objetos detectados
        for i = 1:length(stats)
            bbox = stats(i).BoundingBox;
            centroid = stats(i).Centroid;
            
            % Expande el tamaño del bounding box basado en el centroide
            bbox(1) = max(centroid(1) - bbox(3) * 1.2 / 2, 1);  % Expandir ancho
            bbox(2) = max(centroid(2) - bbox(4) * 1.2 / 2, 1);  % Expandir altura
            bbox(3) = bbox(3) * 1.2;  % Aumentar ancho
            bbox(4) = bbox(4) * 1.2;  % Aumentar altura
            
            % Dibujar el bounding box y el centroide en la imagen
            img = insertShape(img, 'Rectangle', bbox, 'Color', [0, 255, 0], 'LineWidth', 2);
            img = insertMarker(img, centroid, 'o', 'Color', [255, 0, 0], 'Size', 10);
            
            % Guardar el historial del centroide
            if centroidHistoryIdx <= size(centroidHistory, 1)
                centroidHistory(centroidHistoryIdx, :) = centroid;
                centroidHistoryIdx = centroidHistoryIdx + 1;
            end
            
            % Detectar si el vehículo cruza la línea roja (inicio del cruce)
            if ~isCrossing && centroid(2) < linearoja1(2)
                isCrossing = true;
                startFrame = frameCount;
            end

            % Detectar si el vehículo cruza la línea azul (fin del cruce)
            if isCrossing && centroid(2) > lineaazul1(2)
                endFrame = frameCount;
                isCrossing = false;
                
                % Calcular el tiempo y la velocidad
                timeTaken = (endFrame - startFrame) / fps;
                if timeTaken > 0
                    speed = (distanceBetweenLines / timeTaken) * 3.6;
                    fprintf('Velocidad: %.2f km/h\n', speed);
                end
            end
        end
        
        % Dibujar las líneas de referencia
        img = insertShape(img, 'Line', [linearoja1, linearoja2], 'Color', [255, 0, 0], 'LineWidth', 3);
        img = insertShape(img, 'Line', [lineaazul1, lineaazul2], 'Color', [0, 0, 255], 'LineWidth', 3);
        
        % Preparar la máscara en formato uint8 para visualización
        cleanForeground_uint8 = uint8(cleanForeground) * 255;
        combinedImg = [img, cat(3, cleanForeground_uint8, cleanForeground_uint8, cleanForeground_uint8)];
        
        % Mostrar la imagen original junto con la detección de primer plano
        displayImage(r, combinedImg, 'Title', 'Original y Detección de Primer Plano con Seguimiento de Objetos');
    end
end