import json
import pickle
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import osmnx as ox
import pandas as pd
from geopy.distance import geodesic
from shapely.geometry import LineString
from geo_utils import create_transformer, wgs84_to_local, estimate_building_height

# === USTAWIENIA ===
DATA_DIR = Path("data")
OUT_POINTS = Path("selected_points.json")
OUT_BBOX = Path("bbox_info.json")

COLORS = {
    "road": "#999999",
    "building": "#FFA500",
    "point_a": "green",
    "point_b": "red",
}

ox.settings.use_cache = True


def rotate_points(x, y, angle_rad):
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    xr = c * x - s * y
    yr = s * x + c * y
    return xr, yr


class MapABSelector:
    def __init__(self):
        self.G = None
        self.buildings = None
        self.points = []
        self.fig = None
        self.ax = None
        self.selection_mode = False

    def load_data(self):
        street_path = DATA_DIR / "warsaw_street_network.pkl"
        if not street_path.exists():
            print(f"✗ Brak pliku: {street_path} (uruchom 01_download_warsaw_data.py)")
            return False
        with open(street_path, "rb") as f:
            self.G = pickle.load(f)
        print(f"✓ Sieć drogowa: {len(self.G.nodes):,} węzłów")

        bldg_path = DATA_DIR / "warsaw_buildings.pkl"
        if bldg_path.exists():
            self.buildings = pd.read_pickle(bldg_path)
            print(f"✓ Budynki: {len(self.buildings):,}")
        else:
            print("⚠ Brak pliku z budynkami (rysunek profilu będzie bez zabudowy)")
        return True

    def create_map(self):
        self.fig, self.ax = ox.plot_graph(
            self.G,
            node_size=0,
            edge_linewidth=0.5,
            edge_color=COLORS["road"],
            bgcolor="white",
            show=False,
            close=False,
        )
        if self.buildings is not None:
            plot_df = self.buildings.copy()
            plot_df["estimated_height"] = plot_df.apply(estimate_building_height, axis=1)
            plot_df.plot(
                ax=self.ax,
                column="estimated_height",
                cmap="YlOrRd",
                alpha=0.6,
                edgecolor="darkgray",
                linewidth=0.3,
                legend=True,
                legend_kwds={"label": "Wysokość [m]", "shrink": 0.8},
            )

        self.ax.set_title(
            "TRYB ZOOM: Użyj narzędzi na pasku. Naciśnij SPACJĘ aby wejść w tryb zaznaczania.",
            color="blue",
            fontweight="bold",
            fontsize=11,
        )
        self.fig.canvas.mpl_connect("button_press_event", self.on_click)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)

    def on_key(self, event):
        if event.key == " ":
            self.selection_mode = not self.selection_mode
            if self.selection_mode:
                self.ax.set_title("TRYB ZAZNACZANIA: kliknij A, potem B", color="green", fontweight="bold")
            else:
                self.ax.set_title("TRYB ZOOM: naciśnij SPACJĘ aby zaznaczać", color="blue", fontweight="bold")
            self.fig.canvas.draw_idle()

    def on_click(self, event):
        if self.fig.canvas.toolbar.mode != "" or not self.selection_mode:
            return
        if event.xdata is None or event.ydata is None:
            return
        lon, lat = event.xdata, event.ydata
        self.points.append((lat, lon))

        color = COLORS["point_a"] if len(self.points) == 1 else COLORS["point_b"]
        label = "A" if len(self.points) == 1 else "B"
        self.ax.plot(lon, lat, "o", color=color, markersize=14, markeredgecolor="black", markeredgewidth=2, zorder=999)
        self.ax.text(lon, lat, f"  {label}", fontsize=13, fontweight="bold", zorder=1000)
        self.fig.canvas.draw_idle()

        if len(self.points) == 2:
            plt.close(self.fig)

    def show_yx_zx(self):
        if len(self.points) != 2:
            print("✗ Nie wybrano punktów A i B")
            return None

        a, b = self.points
        dist_ab = geodesic(a, b).meters
        print(f"📏 Odległość A–B: {dist_ab:.2f} m")

        margin_m = 100.0
        margin_deg = margin_m / 111000.0
        lats = [a[0], b[0]]
        lons = [a[1], b[1]]
        bbox = {
            "north": max(lats) + margin_deg,
            "south": min(lats) - margin_deg,
            "east": max(lons) + margin_deg,
            "west": min(lons) - margin_deg,
        }

        b_in = None
        if self.buildings is not None:
            try:
                b_in = self.buildings.cx[bbox["west"]:bbox["east"], bbox["south"]:bbox["north"]]
            except Exception:
                pass

        ref_lat = float(np.mean(lats))
        ref_lon = float(np.mean(lons))
        transformer = create_transformer(ref_lat, ref_lon)

        xa, ya = wgs84_to_local(a[0], a[1], transformer)
        xb, yb = wgs84_to_local(b[0], b[1], transformer)
        angle = -np.arctan2((yb - ya), (xb - xa))

        xa_r, ya_r = rotate_points(np.array([xa]), np.array([ya]), angle)
        xb_r, yb_r = rotate_points(np.array([xb]), np.array([yb]), angle)
        xa_r, ya_r = xa_r[0], ya_r[0]
        xb_r, yb_r = xb_r[0], yb_r[0]

        # Oblicz dystans od A do B
        dist_total = dist_ab

        # Konwersja współrzędnych budynków na dystans
        y_center = 0.5 * (ya_r + yb_r)
        y_span = 150.0
        y_min = y_center - y_span
        y_max = y_center + y_span

        fig, (ax_top, ax_bottom) = plt.subplots(2, 1, figsize=(14, 10), sharex=True)

        ax_top.set_aspect("equal", adjustable="datalim")
        ax_top.set_facecolor("#f9f9f9")

        if b_in is not None and len(b_in) > 0:
            for _, row in b_in.iterrows():
                try:
                    geom = row.geometry
                    if geom.geom_type == "Polygon":
                        coords = list(geom.exterior.coords)
                    elif geom.geom_type == "MultiPolygon":
                        coords = list(geom.geoms[0].exterior.coords)
                    else:
                        continue
                    lons_b, lats_b = zip(*coords)
                    bx, by = [], []
                    for latv, lonv in zip(lats_b, lons_b):
                        x, y = wgs84_to_local(latv, lonv, transformer)
                        bx.append(x)
                        by.append(y)
                    bx = np.array(bx)
                    by = np.array(by)
                    bxr, byr = rotate_points(bx, by, angle)

                    # Konwersja X na dystans (przesunięcie tak aby A było w 0)
                    bxr_dist = bxr - xa_r
                    ax_top.fill(bxr_dist, byr, facecolor="orange", edgecolor="darkgray", alpha=0.5, linewidth=0.5,
                                zorder=10)
                except Exception:
                    continue

        # Linia A-B jako dystans od 0 do dist_ab
        ax_top.plot([0, dist_total], [ya_r, yb_r], "b-", lw=3, label=f"Linia prosta ({dist_ab:.0f} m)", zorder=50)
        ax_top.plot(0, ya_r, "o", color=COLORS["point_a"], markersize=14, markeredgecolor="black", markeredgewidth=2)
        ax_top.plot(dist_total, yb_r, "o", color=COLORS["point_b"], markersize=14, markeredgecolor="black",
                    markeredgewidth=2)
        ax_top.text(0, ya_r, "  A", fontsize=13, fontweight="bold")
        ax_top.text(dist_total, yb_r, "  B", fontsize=13, fontweight="bold")
        ax_top.set_ylabel("Y [m]")
        ax_top.set_xlabel("Dystans [m]")
        ax_top.set_title("Widok z góry Y(dystans) – linia A–B")
        ax_top.grid(True, alpha=0.3, linestyle="--")
        ax_top.set_xlim(-50, dist_total + 50)
        ax_top.set_ylim(y_min, y_max)

        ax_bottom.set_facecolor("#f9f9f9")
        ax_bottom.plot([0, dist_total], [0, 0], "b-", lw=3, label="Poziom gruntu")

        max_h = 10.0
        if b_in is not None and len(b_in) > 0:
            route_line = LineString([(a[1], a[0]), (b[1], b[0])])
            buf = 10.0 / 111000.0
            rbuf = route_line.buffer(buf)
            for _, row in b_in.iterrows():
                geom = row.geometry
                if geom is None or not geom.intersects(rbuf):
                    continue
                try:
                    h = estimate_building_height(row)
                    max_h = max(max_h, h)

                    if geom.geom_type == "Polygon":
                        coords = list(geom.exterior.coords)
                    elif geom.geom_type == "MultiPolygon":
                        coords = list(geom.geoms[0].exterior.coords)
                    else:
                        continue

                    lons_b, lats_b = zip(*coords)
                    bx, by = [], []
                    for latv, lonv in zip(lats_b, lons_b):
                        x, y = wgs84_to_local(latv, lonv, transformer)
                        bx.append(x)
                        by.append(y)
                    bx = np.array(bx)
                    by = np.array(by)
                    bxr, byr = rotate_points(bx, by, angle)

                    if len(bxr) > 0:
                        # Konwersja na dystans
                        bxr_dist = bxr - xa_r
                        w = max(float(np.max(bxr_dist) - np.min(bxr_dist)), 5.0)
                        xc = 0.5 * (float(np.max(bxr_dist)) + float(np.min(bxr_dist)))
                        rect = patches.Rectangle((xc - w / 2, 0), w, h, facecolor="orange", edgecolor="darkgray",
                                                 alpha=0.6, linewidth=1, zorder=30)
                        ax_bottom.add_patch(rect)
                except Exception:
                    continue

        ax_bottom.scatter([0], [0], color=COLORS["point_a"], s=120, marker="o", edgecolor="black", linewidth=2,
                          zorder=50)
        ax_bottom.scatter([dist_total], [0], color=COLORS["point_b"], s=120, marker="o", edgecolor="black", linewidth=2,
                          zorder=50)
        ax_bottom.set_xlabel("Dystans [m]")
        ax_bottom.set_ylabel("Wysokość [m]")
        ax_bottom.set_title("Profil Z(dystans) – zabudowa pod linią A–B")
        ax_bottom.grid(True, alpha=0.3, linestyle="--")
        ax_bottom.set_ylim(bottom=-2, top=max_h * 1.15)
        ax_bottom.set_xlim(-50, dist_total + 50)

        fig.tight_layout()
        plt.show()

        with open(OUT_POINTS, "w", encoding="utf-8") as f:
            json.dump({"A": {"lat": a[0], "lon": a[1]}, "B": {"lat": b[0], "lon": b[1]}}, f, ensure_ascii=False,
                      indent=2)
        with open(OUT_BBOX, "w", encoding="utf-8") as f:
            json.dump({"distance_m": dist_ab, "bbox": bbox}, f, ensure_ascii=False, indent=2)

        print(f"✓ Zapisano punkty do {OUT_POINTS}")
        print(f"✓ Zapisano metadane do {OUT_BBOX}")
        return {"A": a, "B": b, "distance_m": dist_ab}

    def run(self):
        if not self.load_data():
            return
        self.create_map()
        plt.tight_layout()
        plt.show()
        return self.show_yx_zx()


if __name__ == "__main__":
    MapABSelector().run()