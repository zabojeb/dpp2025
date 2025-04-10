# Импорт необходимых библиотек:
# - cv2 (OpenCV) для обработки изображений.
# - numpy для работы с массивами и числовыми операциями.
# - os для работы с файловой системой.
# - math для математических вычислений (например, вычисления площади окружности).
# - requests для выполнения HTTP-запросов.
# - base64 для кодирования изображений в формат base64.
# - tempfile для работы с временными файлами и каталогами.
# - PIL (Pillow) для работы с изображениями.
# - BytesIO для работы с потоками байтов.
import cv2
import numpy as np
import os
import math
import requests
import base64
import tempfile
from PIL import Image
from io import BytesIO


def analyze_stones(image_path, output_dir, image_id, exclude_rects=None):
    """
    Функция анализирует изображение, обнаруживает камни бесконечности по заданным цветовым диапазонам,
    сохраняет найденные регионы с камнями и формирует список данных по каждому найденному объекту.

    Параметры:
    - image_path: путь к изображению для анализа.
    - output_dir: директория, в которую будут сохраняться обработанные изображения.
    - image_id: уникальный идентификатор изображения, используется при формировании имен файлов.
    - exclude_rects: список прямоугольных областей (x1, y1, x2, y2), которые исключаются из анализа.
      Если не указан, используется пустой список.
    """

    if exclude_rects is None:
        exclude_rects = []

    # Определение цветовых диапазонов для каждого типа камней.
    # Для каждого цвета задаются:
    # - name: наименование камня.
    # - description: описание способностей камня.
    # - color_bgr: цвет для отрисовки рамки (в формате BGR, используемом в OpenCV).
    # - hsv_min/hsv_max: минимальное и максимальное значение в пространстве HSV для обнаружения данного цвета.
    color_ranges = {
        'blue': {
            'name': 'Камень Пространства',
            'description': 'Позволяет управлять пространством, открывать порталы и перемещаться мгновенно.',
            'color_bgr': (255, 0, 0),
            'hsv_min': np.array([90, 80, 80]),
            'hsv_max': np.array([130, 255, 255])
        },
        'yellow': {
            'name': 'Камень Разума',
            'description': 'Даёт способность контролировать разум и усиливать интеллект.',
            'color_bgr': (0, 255, 255),
            'hsv_min': np.array([15, 80, 80]),
            'hsv_max': np.array([50, 255, 255])
        },
        'green': {
            'name': 'Камень Времени',
            'description': 'Дарует власть над временем: остановка, ускорение, замедление.',
            'color_bgr': (0, 255, 0),
            'hsv_min': np.array([60, 45, 45]),
            'hsv_max': np.array([150, 255, 255])
        },
        'red_1': {
            'name': 'Камень Реальности',
            'description': 'Способен искажать реальность по желанию владельца.',
            'color_bgr': (0, 0, 255),
            'hsv_min': np.array([0, 80, 80]),
            'hsv_max': np.array([10, 255, 255])
        },
        'red_2': {
            'name': 'Камень Реальности',
            'description': 'Способен искажать реальность по желанию владельца.',
            'color_bgr': (0, 0, 255),
            'hsv_min': np.array([170, 80, 80]),
            'hsv_max': np.array([180, 255, 255])
        }
    }

    # Группировка цветов для объединения масок. Для красного используется два диапазона.
    color_groups = {
        'blue': ['blue'],
        'yellow': ['yellow'],
        'green': ['green'],
        'red': ['red_1', 'red_2']
    }

    # Если выходная директория не существует, создаём её.
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Чтение исходного изображения.
    image = cv2.imread(image_path)
    if image is None:
        print(f"Не удалось открыть файл {image_path}")
        return [], None

    image_height, image_width = image.shape[:2]

    # Переводим изображение в цветовое пространство HSV для более точного выделения цветов.
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Создаём копию изображения для отрисовки обнаруженных объектов (камней).
    draw_img = image.copy()

    stones_data = []

    # Обрабатываем каждую группу цветов.
    for group_name, color_keys in color_groups.items():
        combined_mask = None  # Маска, объединяющая диапазоны для группы.
        # Для каждого ключа (цветового диапазона) в группе создаём маску.
        for ck in color_keys:
            info = color_ranges[ck]
            hsv_min, hsv_max = info['hsv_min'], info['hsv_max']

            mask = cv2.inRange(hsv, hsv_min, hsv_max)

            # Для повышения качества маски применяем морфологические операции:
            # сначала закрытие (MORPH_CLOSE) для устранения мелких дырок,
            # затем открытие (MORPH_OPEN) для удаления шумов.
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            # Объединяем маски для двух диапазонов красного цвета или оставляем одну маску для остальных.
            if combined_mask is None:
                combined_mask = mask
            else:
                combined_mask = cv2.bitwise_or(combined_mask, mask)

        # Если маска не создана, переходим к следующей группе.
        if combined_mask is None:
            continue

        # Исключаем из маски области, заданные в exclude_rects (например, для исключения фона или ненужных областей).
        for (ex_x1, ex_y1, ex_x2, ex_y2) in exclude_rects:
            combined_mask[ex_y1:ex_y2, ex_x1:ex_x2] = 0

        # Выбираем первичную информацию (имя, описание, цвет отрисовки) по первому ключу в группе.
        primary_info = color_ranges[color_keys[0]]

        # Находим контуры в объединённой маске.
        contours, _ = cv2.findContours(
            combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Обрабатываем каждый найденный контур.
        for cnt in contours:
            # Вычисляем площадь контура.
            area = cv2.contourArea(cnt)

            # Пропускаем контуры с малой площадью, чтобы избежать ложных срабатываний.
            if area < 100:
                continue

            # Вычисляем периметр контура.
            perimeter = cv2.arcLength(cnt, True)
            # Определяем ограничивающий прямоугольник вокруг контура.
            x, y, w, h = cv2.boundingRect(cnt)

            # Рассчитываем отступы относительно размеров прямоугольника,
            # чтобы добавить дополнительное пространство вокруг найденного объекта.
            padding_x = max(int(w * 0.25), 15)
            padding_y = max(int(h * 0.25), 15)

            # Определяем координаты области вырезки с учётом отступов и границ изображения.
            crop_x1 = max(0, x - padding_x)
            crop_y1 = max(0, y - padding_y)
            crop_x2 = min(image_width, x + w + padding_x)
            crop_y2 = min(image_height, y + h + padding_y)

            # Рисуем прямоугольник на копии изображения с использованием цвета из primary_info.
            cv2.rectangle(draw_img, (x, y), (x + w, y + h),
                          primary_info['color_bgr'], 3)

            # Извлекаем фрагмент изображения с найденным камнем.
            stone_crop = image[crop_y1:crop_y2, crop_x1:crop_x2]
            # Формируем имя файла для сохранения вырезанного изображения.
            stone_filename = f"{image_id}_{group_name}_{x}_{y}.png"
            stone_path = os.path.join(output_dir, stone_filename)
            # Сохраняем вырезанное изображение.
            cv2.imwrite(stone_path, stone_crop)

            # Вычисляем коэффициент формы (shape factor) как меру округлости:
            # (4*pi*area) / (perimeter^2). Если периметр равен нулю, задаём 0.
            if perimeter > 0:
                shape_factor = (4.0 * math.pi * area) / (perimeter * perimeter)
            else:
                shape_factor = 0.0

            # Вычисляем соотношение сторон прямоугольника.
            if h != 0:
                aspect_ratio = float(w) / float(h)
            else:
                aspect_ratio = 1.0

            # Вычисляем средние значения B, G, R в области интереса для анализа цвета.
            roi = image[y:y + h, x:x + w]
            mean_b = float(np.mean(roi[:, :, 0]))
            mean_g = float(np.mean(roi[:, :, 1]))
            mean_r = float(np.mean(roi[:, :, 2]))

            # Добавляем данные о найденном камне в список:
            stones_data.append({
                'name': primary_info['name'],
                'description': primary_info['description'],
                'color_bgr': primary_info['color_bgr'],
                'area': area,
                'perimeter': perimeter,
                'aspect_ratio': aspect_ratio,
                'shape_factor': shape_factor,
                'avg_color': (mean_b, mean_g, mean_r),
                'stone_img_path': stone_path,
                'contour': cnt
            })

    # Если найдено более одного камня, масштабируем силу (power) каждого на основе площади.
    if len(stones_data) > 1:
        min_area = min(s['area'] for s in stones_data)
        max_area = max(s['area'] for s in stones_data)
        for s in stones_data:
            # Если разница между максимальной и минимальной площадью незначительна,
            # устанавливаем фиксированную силу.
            if abs(max_area - min_area) < 1e-6:
                s['power'] = 3
            else:
                # Линейное масштабирование силы от 1 до 5.
                power = 1 + 4 * (s['area'] - min_area) / (max_area - min_area)
                s['power'] = int(round(power))
    elif len(stones_data) == 1:
        # Если найден только один камень, задаём ему максимальную силу.
        stones_data[0]['power'] = 5

    # Сохраняем изображение с отмеченными камнями.
    marked_filename = f"{image_id}_marked.png"
    marked_path = os.path.join(output_dir, marked_filename)
    cv2.imwrite(marked_path, draw_img)
    # Функция возвращает список данных по камням и путь к отмеченному изображению.
    return stones_data, marked_path


def generate_html_report(report_data, output_html):
    """
    Функция генерирует HTML-отчёт по результатам анализа камней.
    В отчёте отображается статистика, галерея изображений и подробный анализ каждого камня.

    Параметры:
    - report_data: список словарей с информацией по каждому изображению и обнаруженным камням.
    - output_html: путь для сохранения сгенерированного HTML-файла.
    """
    def image_to_data_url(file_path):
        """
        Вспомогательная функция для конвертации изображения в data URL (base64),
        чтобы встроить изображения непосредственно в HTML-страницу.
        """
        try:
            with open(file_path, "rb") as image_file:
                base64_data = base64.b64encode(
                    image_file.read()).decode("utf-8")
            return "data:image/png;base64," + base64_data
        except Exception as e:
            print(f"Error converting {file_path} to data URL: {e}")
            return ""

    # Определяем цвета для отрисовки камней в отчёте по их наименованиям.
    stone_colors = {
        'Камень Пространства': '#3498db',
        'Камень Разума': '#f1c40f',
        'Камень Времени': '#2ecc71',
        'Камень Реальности': '#e74c3c',
    }

    # Собираем все камни из всех изображений в единый список.
    all_stones = []
    for item in report_data:
        all_stones.extend(item['stones'])

    # Подсчитываем общие метрики: общее количество камней, суммарную силу и максимальное возможное значение силы.
    total_stones = len(all_stones)
    total_power = sum(s.get('power', 1) for s in all_stones)
    max_total = total_stones * 5  # Если все камни имеют силу 5
    power_percentage = (total_power / max_total * 100) if max_total > 0 else 0

    # Формирование базовой структуры HTML-страницы со встроенными стилями.
    html_content = """
    <!DOCTYPE html>
    <html lang="ru">
    <head>
        <meta charset="utf-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Анализ камней бесконечности</title>
        <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.1.1/css/all.min.css">
        <style>
            /* Стили отчёта: определяются корневые переменные, стили для контейнеров, карточек,
               галерей, а также адаптивность для мобильных устройств */
            :root {
                --primary-color: #6200ea;
                --secondary-color: #424242;
                --accent-color: #651fff;
                --background-color: #f5f5f7;
                --card-background: #ffffff;
                --text-color: #333333;
                --border-radius: 12px;
                --shadow: 0 4px 20px rgba(0,0,0,0.08);
                --transition: all 0.3s ease;
            }
            * { box-sizing: border-box; margin: 0; padding: 0; }
            body {
                font-family: 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, sans-serif;
                background: var(--background-color);
                color: var(--text-color);
                line-height: 1.6;
                padding-bottom: 40px;
            }
            .container {
                max-width: 1200px;
                margin: 0 auto;
                padding: 0 20px;
            }
            header {
                background: linear-gradient(135deg, var(--primary-color), var(--accent-color));
                color: white;
                padding: 30px 0;
                text-align: center;
                margin-bottom: 40px;
                box-shadow: var(--shadow);
                position: relative;
                overflow: hidden;
            }
            header::before {
                content: '';
                position: absolute;
                top: 0; left: 0; right: 0; bottom: 0;
                background: url('data:image/svg+xml;utf8,<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100" preserveAspectRatio="none"><path d="M0,0 L100,0 L100,100 Z" fill="rgba(255,255,255,0.1)"/></svg>');
                background-size: cover;
            }
            h1 { font-size: 2.5rem; font-weight: 700; margin: 0; position: relative; }
            h1::after {
                content: ''; display: block; width: 80px; height: 4px;
                background: white; margin: 10px auto 0; border-radius: 2px;
            }
            .subtitle { font-size: 1.1rem; opacity: 0.9; margin-top: 10px; }
            main { margin: 0 auto; }
            .dashboard {
                background: var(--card-background);
                border-radius: var(--border-radius);
                box-shadow: var(--shadow);
                padding: 25px;
                margin-bottom: 30px;
                display: flex;
                flex-wrap: wrap;
                gap: 20px;
                justify-content: space-between;
                align-items: center;
            }
            .stat-card {
                flex: 1;
                min-width: 200px;
                background: linear-gradient(to bottom right, rgba(255,255,255,0.7), rgba(255,255,255,0.3));
                backdrop-filter: blur(10px);
                border-radius: var(--border-radius);
                padding: 20px;
                text-align: center;
                border: 1px solid rgba(255,255,255,0.5);
                box-shadow: 0 4px 10px rgba(0,0,0,0.05);
                transition: var(--transition);
            }
            .stat-card:hover {
                transform: translateY(-5px);
                box-shadow: 0 10px 20px rgba(0,0,0,0.1);
            }
            .stat-card i { font-size: 2rem; margin-bottom: 10px; color: var(--primary-color); }
            .stat-value { font-size: 2rem; font-weight: bold; color: var(--primary-color); margin-bottom: 5px; }
            .stat-label { font-size: 0.9rem; color: var(--secondary-color); font-weight: 500; }
            .progress-container { flex: 100%; margin-top: 10px; }
            .progress-label {
                display: flex; justify-content: space-between; margin-bottom: 5px; font-weight: 600;
            }
            .progress-bar {
                height: 10px; background-color: #e0e0e0; border-radius: 5px; overflow: hidden;
            }
            .progress-bar-fill {
                height: 100%; background: linear-gradient(to right, var(--primary-color), var(--accent-color));
                border-radius: 5px; transition: width 1s ease-in-out; position: relative;
            }
            section { margin-bottom: 40px; }
            .section-title {
                font-size: 1.8rem; color: var(--secondary-color); margin-bottom: 20px;
                padding-bottom: 10px; border-bottom: 3px solid var(--primary-color); display: inline-block;
            }
            .stones-gallery {
                display: grid;
                grid-template-columns: repeat(auto-fill, minmax(300px, 1fr));
                gap: 25px; margin-bottom: 40px;
            }
            .image-container {
                background: var(--card-background);
                border-radius: var(--border-radius);
                overflow: hidden;
                box-shadow: var(--shadow);
                transition: var(--transition);
            }
            .image-container:hover {
                transform: translateY(-5px);
                box-shadow: 0 15px 30px rgba(0,0,0,0.1);
            }
            .image-container img {
                width: 100%; height: auto; display: block; transition: transform 0.5s ease;
            }
            .image-container:hover img { transform: scale(1.05); }
            .image-caption {
                padding: 15px; text-align: center; font-style: italic; color: var(--secondary-color);
                border-top: 1px solid #f0f0f0;
            }
            .stones-list {
                display: grid;
                grid-template-columns: repeat(auto-fill, minmax(350px, 1fr));
                gap: 25px;
            }
            .stone-card {
                background: var(--card-background);
                border-radius: var(--border-radius);
                overflow: hidden;
                box-shadow: var(--shadow);
                transition: var(--transition);
                display: flex;
                flex-direction: column;
            }
            .stone-card:hover {
                transform: translateY(-5px);
                box-shadow: 0 15px 30px rgba(0,0,0,0.15);
            }
            .stone-header {
                padding: 20px;
                display: flex;
                align-items: center;
                justify-content: space-between;
                border-bottom: 1px solid #f0f0f0;
            }
            .stone-title {
                font-size: 1.3rem;
                font-weight: 700;
                color: var(--secondary-color);
                margin: 0;
            }
            .stone-power {
                display: flex;
                align-items: center;
                font-weight: 600;
            }
            .stars { color: #ffc107; letter-spacing: 2px; margin-right: 8px; }
            .stone-banner { height: 8px; width: 100%; }
            .stone-image-container {
                padding: 20px;
                text-align: center;
                background: #f9f9f9;
            }
            .stone-image {
                max-width: 100%;
                height: auto;
                border-radius: 8px;
                box-shadow: 0 4px 8px rgba(0,0,0,0.1);
            }
            .stone-body { padding: 20px; flex-grow: 1; }
            .stone-description {
                margin-bottom: 15px;
                font-style: italic;
                color: #555;
                line-height: 1.5;
            }
            .stone-params {
                display: grid;
                grid-template-columns: 1fr 1fr;
                gap: 15px;
            }
            .param-group { margin-bottom: 10px; }
            .param-label { font-size: 0.85rem; color: #777; margin-bottom: 3px; }
            .param-value { font-weight: 600; color: var(--secondary-color); }
            .extra-params {
                background-color: #f8f9fa;
                border-radius: 8px;
                padding: 15px;
                margin-top: 15px;
            }
            .extra-params-title { font-size: 0.9rem; font-weight: 600; margin-bottom: 10px; color: #555; }
            footer {
                text-align: center;
                margin-top: 60px;
                padding: 20px;
                color: #777;
                font-size: 0.9rem;
            }
            footer p { margin: 5px 0; }
            @media (max-width: 768px) {
                .stones-list { grid-template-columns: 1fr; }
                .stones-gallery { grid-template-columns: 1fr; }
                .dashboard { flex-direction: column; }
                .stat-card { width: 100%; }
            }
        </style>
    </head>
    <body>
    <header>
        <div class="container">
            <h1>Анализ камней бесконечности</h1>
            <div class="subtitle">Исследование силы и характеристик камней бесконечности</div>
        </div>
    </header>
    <main class="container">
    """
    # Добавляем секцию с общей статистикой: количество камней, суммарная и средняя сила,
    # а также отображаем прогресс по силе с помощью полосы прогресса.
    html_content += f"""
    <section class="dashboard">
        <div class="stat-card">
            <i class="fas fa-gem"></i>
            <div class="stat-value">{total_stones}</div>
            <div class="stat-label">Всего камней</div>
        </div>
        <div class="stat-card">
            <i class="fas fa-bolt"></i>
            <div class="stat-value">{total_power}</div>
            <div class="stat-label">Общая сила</div>
        </div>
        <div class="stat-card">
            <i class="fas fa-star-half-alt"></i>
            <div class="stat-value">{round(total_power / total_stones, 1) if total_stones > 0 else 0.0}</div>
            <div class="stat-label">Средняя сила</div>
        </div>
        <div class="progress-container">
            <div class="progress-label">
                <span>Мощность камней</span>
                <span>{total_power} из {max_total}</span>
            </div>
            <div class="progress-bar">
                <div class="progress-bar-fill" style="width: {power_percentage}%;"></div>
            </div>
        </div>
    </section>
    """
    # Добавляем секцию с галереей изображений, где показаны исходное изображение с обведёнными камнями.
    html_content += """
    <section>
        <h2 class="section-title">Фото камней</h2>
        <div class="stones-gallery">
    """
    # Для каждого отчёта (каждого обработанного изображения) вставляем изображение с обведёнными камнями.
    for item in report_data:
        img_id = item['image_id']
        marked_src = image_to_data_url(item['marked_path'])
        html_content += f"""
        <div class="image-container">
            <img src="{marked_src}" alt="Камни бесконечности на изображении {img_id}">
            <div class="image-caption">Изображение {img_id} с обведёнными камнями</div>
        </div>
        """
    html_content += "</div></section>"
    # Добавляем секцию с подробным анализом каждого найденного камня.
    html_content += """
    <section>
        <h2 class="section-title">Детальный анализ камней</h2>
        <div class="stones-list">
    """
    # Для каждого камня создаём карточку с информацией: название, описание, параметры (площадь, периметр, коэффициент формы, соотношение сторон, средний цвет)
    for s in all_stones:
        name = s['name']
        desc = s['description']
        area = s['area']
        perimeter = s['perimeter']
        power = s.get('power', 1)
        # Формируем строку с "звёздочками", отражающими силу: закрашенные и незакрашенные звёзды.
        stars_str = "★" * power + "☆" * (5 - power)
        shape_factor = s.get('shape_factor', 0.0)
        aspect_ratio = s.get('aspect_ratio', 1.0)
        avg_color = s.get('avg_color', (0, 0, 0))
        shape_factor_str = f"{shape_factor:.3f}"
        aspect_ratio_str = f"{aspect_ratio:.2f}"
        avg_color_str = f"B={avg_color[0]:.1f}, G={avg_color[1]:.1f}, R={avg_color[2]:.1f}"
        stone_color = stone_colors.get(name, '#7d3c98')
        stone_src = image_to_data_url(s['stone_img_path'])
        html_content += f"""
        <div class="stone-card">
            <div class="stone-banner" style="background-color: {stone_color};"></div>
            <div class="stone-header">
                <h3 class="stone-title">{name}</h3>
                <div class="stone-power">
                    <span class="stars">{stars_str}</span>
                    <span>({power}/5)</span>
                </div>
            </div>
            <div class="stone-image-container">
                <img class="stone-image" src="{stone_src}" alt="{name}">
            </div>
            <div class="stone-body">
                <p class="stone-description">{desc}</p>
                <div class="stone-params">
                    <div class="param-group">
                        <div class="param-label">Площадь</div>
                        <div class="param-value">{area:.1f} px²</div>
                    </div>
                    <div class="param-group">
                        <div class="param-label">Периметр</div>
                        <div class="param-value">{perimeter:.1f} px</div>
                    </div>
                </div>
                <div class="extra-params">
                    <div class="extra-params-title">Дополнительные параметры</div>
                    <div class="stone-params">
                        <div class="param-group">
                            <div class="param-label">Коэффициент формы</div>
                            <div class="param-value">{shape_factor_str}</div>
                        </div>
                        <div class="param-group">
                            <div class="param-label">Соотношение сторон (W/H)</div>
                            <div class="param-value">{aspect_ratio_str}</div>
                        </div>
                        <div class="param-group">
                            <div class="param-label">Средний цвет (BGR)</div>
                            <div class="param-value">{avg_color_str}</div>
                        </div>
                    </div>
                </div>
            </div>
        </div>
        """
    html_content += """
        </div>
    </section>
    </main>
    <footer>
        <p>© 2025 Дежурный По Планете</p>
        <p>Система анализа камней бесконечности</p>
    </footer>
    </body>
    </html>
    """

    # При использовании скрипта отдельно от интерфейса можно сохранять HTML-отчёт в файл.
    # Но мы выводим сформированный HTML в консоль для отрисовки в Electron.
    print(html_content)


def main():
    """
    Основная функция:
    - Делает HTTP-запрос для получения изображения с камеры.
    - Сохраняет изображение во временную директорию.
    - Вызывает функцию анализа изображения для обнаружения и сохранения найденных камней.
    - Генерирует HTML-отчёт по полученным данным.
    """
    URL = "http://192.168.1.96:9999/snapshot?topic=/front_camera/image_raw"
    response = requests.get(URL)
    image = Image.open(BytesIO(response.content))

    base_dir = os.path.join(tempfile.gettempdir(), "analysis-reports")
    camera_stream_dir = os.path.join(base_dir, "camera-stream")
    os.makedirs(camera_stream_dir, exist_ok=True)

    image_save_path = os.path.join(camera_stream_dir, "stream.jpeg")
    image.save(image_save_path)

    # Можно анализировать несколько изображений
    input_images = [image_save_path]
    output_dir = base_dir
    os.makedirs(output_dir, exist_ok=True)

    # Задаём области, которые нужно исключить из анализа
    exclude_areas = [
        (0, 0, 1000, 75),
        (0, 0, 185, 1000),
        (0, 360, 1000, 1000),
        (457, 0, 1000, 1000),
        (415, 52, 456, 100),
        (293, 340, 325, 367)
    ]

    report_data = []

    # Для каждого изображения проводим анализ, генерируем данные по камням и сохраняем отмеченное изображение
    for idx, img_path in enumerate(input_images, start=1):
        image_id = f"img{idx}"
        stones_info, marked_path = analyze_stones(
            image_path=img_path,
            output_dir=output_dir,
            image_id=image_id,
            exclude_rects=exclude_areas
        )
        report_data.append({
            'image_id': image_id,
            'marked_path': marked_path,
            'stones': stones_info
        })

    # Задаём путь для HTML-отчёта и генерируем его
    output_html = os.path.join(output_dir, "report.html")
    generate_html_report(report_data, output_html)


# Точка входа в программу.
if __name__ == "__main__":
    main()
