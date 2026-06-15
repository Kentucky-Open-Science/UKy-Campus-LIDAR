#!/usr/bin/env python
"""Refresh the cached LFUCG traffic-signal ground truth used by tools/smooth_roads.py.

Downloads the Lexington-Fayette Urban County Government (LFUCG) "Traffic Signal" open
data layer (point locations of every signalised intersection in Fayette County) as
GeoJSON and writes tools/lfucg_traffic_signals.geojson. smooth_roads.py projects these
points into scene coordinates and treats them as authoritative for which junctions get
traffic lights, so the cache is committed for reproducible offline builds.

Source: LFUCG Open Data Hub, "Traffic Signal" feature service (owner gis_lfucg).
        https://data-lfucg.hub.arcgis.com/  (data (c) LFUCG, open data)

Usage:  python tools/fetch_lfucg_signals.py
"""
import json, os, urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'lfucg_traffic_signals.geojson')
SERVICE = ('https://services1.arcgis.com/Mg7DLdfYcSWIaDnu/arcgis/rest/services/'
           'Traffic_Signal/FeatureServer/0/query')
QUERY = '?where=1%3D1&outFields=Intersection&returnGeometry=true&outSR=4326&f=geojson'


def main():
    url = SERVICE + QUERY
    req = urllib.request.Request(url, headers={'User-Agent': 'uky-campus-viewer/1.0'})
    data = json.load(urllib.request.urlopen(req, timeout=120))
    feats = data.get('features', [])
    if not feats:
        raise SystemExit('no features returned — service URL or schema may have changed')
    json.dump(data, open(OUT, 'w'))
    print(f'wrote {OUT}: {len(feats)} traffic signals')


if __name__ == '__main__':
    main()
