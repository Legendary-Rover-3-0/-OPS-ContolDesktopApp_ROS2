import sqlite3
import numpy as np
import folium
from scipy.interpolate import griddata
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import io
import base64

# Krok 1: Pobranie danych z bazy
conn = sqlite3.connect('GPS/radiation_data.db')
cursor = conn.cursor()
cursor.execute('SELECT latitude, longitude, radiation FROM radiation')
rows = cursor.fetchall()
conn.close()

# Ekstrakcja danych
lats = np.array([row[0] for row in rows])
lons = np.array([row[1] for row in rows])
vals = np.array([row[2] for row in rows])

# Krok 2: Interpolacja
grid_lat, grid_lon = np.mgrid[min(lats):max(lats):100j, min(lons):max(lons):100j]
grid_vals = griddata((lats, lons), vals, (grid_lat, grid_lon), method='cubic')

# Krok 3: Wygenerowanie konturów BEZ osi i legendy
fig, ax = plt.subplots(figsize=(8, 6))
ax.axis('off')  # wyłączenie osi
ax.contourf(grid_lon, grid_lat, grid_vals, levels=10, cmap=cm.jet)

# Zapis jako obraz do bufora
buf = io.BytesIO()
plt.savefig(buf, format='png', bbox_inches='tight', pad_inches=0)
buf.seek(0)
img_base64 = base64.b64encode(buf.read()).decode('utf-8')
buf.close()
plt.close()

# Krok 4: Osadzenie na mapie folium
center_lat = np.mean(lats)
center_lon = np.mean(lons)
m = folium.Map(location=[center_lat, center_lon], zoom_start=15)

image_overlay = folium.raster_layers.ImageOverlay(
    image=f"data:image/png;base64,{img_base64}",
    bounds=[[min(lats), min(lons)], [max(lats), max(lons)]],
    opacity=0.6,
    interactive=False,
    cross_origin=False,
)
image_overlay.add_to(m)

# Dodanie punktów pomiarowych
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
<b>Radiation Level [uSv/h]</b><br>
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

# Zapis mapy
m.save("radiation_contour_map.html")
print("Mapa zapisana jako radiation_contour_map.html")
