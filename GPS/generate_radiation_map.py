import sqlite3
import numpy as np
import folium
from scipy.interpolate import griddata
import matplotlib.pyplot as plt
import matplotlib.cm as cm

# 1. Pobranie danych
conn = sqlite3.connect('GPS/radiation_data.db')
cursor = conn.cursor()
cursor.execute('SELECT latitude, longitude, radiation FROM radiation')
rows = cursor.fetchall()
conn.close()

lats = np.array([row[0] for row in rows])
lons = np.array([row[1] for row in rows])
vals = np.array([row[2] for row in rows])

# 2. Interpolacja
grid_lat, grid_lon = np.mgrid[
    min(lats):max(lats):100j,
    min(lons):max(lons):100j
]
grid_vals = griddata((lats, lons), vals, (grid_lat, grid_lon), method='cubic')

# 3a. Obraz z legendą (do zapisu, nie na mapę)
fig1, ax1 = plt.subplots(figsize=(8, 6), dpi=150)
ax1.set_position([0, 0, 1, 1])  # usunięcie wszelkich marginesów
ax1.axis('off')
contour1 = ax1.contourf(grid_lon, grid_lat, grid_vals, levels=10, cmap=cm.jet)
ax1.scatter(lons, lats, c='black', s=10, marker='o')
cbar = plt.colorbar(contour1, ax=ax1, orientation='vertical', fraction=0.046)
cbar.set_label('Poziom radiacji')
fig1.savefig("map_with_legend.png", bbox_inches='tight', pad_inches=0)
plt.close(fig1)
print("[✓] Zapisano obraz z legendą: map_with_legend.png")

# 3b. Obraz bez legendy do mapy folium
fig2, ax2 = plt.subplots(figsize=(8, 6), dpi=150)
ax2.set_position([0, 0, 1, 1])  # dokładnie cała przestrzeń
ax2.axis('off')
ax2.contourf(grid_lon, grid_lat, grid_vals, levels=10, cmap=cm.jet)
ax2.scatter(lons, lats, c='black', s=10, marker='o')
fig2.savefig("map_no_legend.png", bbox_inches='tight', pad_inches=0)
plt.close(fig2)
print("[✓] Zapisano obraz bez legendy: map_no_legend.png")

# 4. Osadzenie obrazu na mapie folium
center_lat = np.mean(lats)
center_lon = np.mean(lons)
m = folium.Map(location=[center_lat, center_lon], zoom_start=15)

bounds = [[min(lats), min(lons)], [max(lats), max(lons)]]
overlay = folium.raster_layers.ImageOverlay(
    name='Radiation Overlay',
    image="map_no_legend.png",
    bounds=bounds,
    opacity=0.6,
    interactive=False,
    cross_origin=False,
)
overlay.add_to(m)

# (opcjonalnie) punkty z popupami
for lat, lon, val in zip(lats, lons, vals):
    folium.CircleMarker(
        location=[lat, lon],
        radius=3,
        popup=f"Radiation: {val}",
        color="black",
        fill=True,
        fill_color="white",
        fill_opacity=0.6
    ).add_to(m)

# Krok 5: Dodanie legendy z realnymi wartościami
min_val = round(np.nanmin(vals), 2)
max_val = round(np.nanmax(vals), 2)

legend_html = f"""
<div style="
    position: fixed; 
    bottom: 50px; left: 50px; width: 150px; height: 260px; 
    background-color: rgba(255, 255, 255, 0.85);
    border:2px solid grey; 
    z-index:9999; 
    font-size:14px;
    padding: 10px;
">
<b>Poziom radiacji</b><br>
<svg width="130" height="200">
  <defs>
    <linearGradient id="grad" x1="0" x2="0" y1="0" y2="1">
      <stop offset="0%" stop-color="darkred" />
      <stop offset="20%" stop-color="red" />
      <stop offset="40%" stop-color="orange" />
      <stop offset="60%" stop-color="yellow" />
      <stop offset="80%" stop-color="lightgreen" />
      <stop offset="100%" stop-color="blue" />
    </linearGradient>
  </defs>
  <rect x="0" y="0" width="20" height="200" fill="url(#grad)" />
  <text x="30" y="10" font-size="12">{max_val}</text>
  <text x="30" y="105" font-size="12">{(max_val + min_val)/2:.2f}</text>
  <text x="30" y="195" font-size="12">{min_val}</text>
</svg>
</div>
"""

m.get_root().html.add_child(folium.Element(legend_html))

m.save("radiation_map_with_overlay.html")
print("[✓] Mapa HTML zapisana jako: radiation_map_with_overlay.html")
