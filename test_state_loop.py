import urllib.request
import json
import time

urls = [
    "http://127.0.0.1:8081/scene/state",
    "http://127.0.0.1:8080/scene/state"
]

for url in urls:
    try:
        req = urllib.request.Request(url)
        with urllib.request.urlopen(req, timeout=3) as r:
            body = r.read().decode('utf-8')
            print(f"{url} -> status={r.status}, ok={json.loads(body).get('ok', True)}")
            data = json.loads(body)
            print(f"  workers_max_for_selected: {data.get('workers_max_for_selected')}")
            print(f"  workers: {data.get('workers')}")
            print(f"  active: {data.get('active_diaolan_path')}")
            print(f"  selected: {data.get('selected_diaolan_path')}")
    except Exception as e:
        print(f"{url} -> error {e}")

