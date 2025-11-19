
from pyproj import Transformer
import pandas as pd
from shapely.geometry import Point

def create_transformer(ref_lat, ref_lon):
    """
    Tworzy transformator dla odwzorowania TMerc wokół danego punktu.
    """
    return Transformer.from_crs(
        crs_from="EPSG:4326",
        crs_to=f"+proj=tmerc +lat_0={ref_lat} +lon_0={ref_lon} +k=1 +x_0=0 +y_0=0 +ellps=WGS84",
        always_xy=True
    )

def wgs84_to_local(lat, lon, transformer):
    x, y = transformer.transform(lon, lat)
    return x, y

def local_to_wgs84(x, y, transformer):
    lon, lat = transformer.transform(x, y, direction="INVERSE")
    return lat, lon

def estimate_building_height(row):
    """
    Szacuje wysokość budynku na podstawie 'height' lub 'building:levels'.
    """
    h = None
    if "height" in row.index and pd.notna(row["height"]):
        try:
            h = float(str(row["height"]).lower().replace("m", "").strip())
        except Exception:
            pass
    if h is None and "building:levels" in row.index and pd.notna(row["building:levels"]):
        try:
            h = float(row["building:levels"]) * 3.0
        except Exception:
            pass
    return float(h if h is not None else 10.0)

def get_building_height_at_point(point, buildings):
    """
    Zwraca wysokość budynku [m], jeżeli punkt (lat, lon) leży na budynku.
    W przeciwnym razie zwraca None.
    """
    if buildings is None:
        return None

    lat, lon = point
    p = Point(lon, lat)  # geometria w GDF jest (lon, lat)

    h_max = None

    for _, row in buildings.iterrows():
        geom = getattr(row, "geometry", None)
        if geom is None:
            continue

        # sprawdzamy czy rzut punktu jest w obrysie budynku
        try:
            if not geom.contains(p):
                continue
        except Exception:
            continue

        # szacujemy wysokość budynku
        try:
            h = float(estimate_building_height(row))
        except Exception:
            continue

        if h <= 0:
            continue

        if h_max is None or h > h_max:
            h_max = h

    return h_max
