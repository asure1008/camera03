import urllib.request
import time
import os

urls = [
    "http://127.0.0.1:8081/snapshot.jpg",
    "http://127.0.0.1:8080/onvif-snap.jpg"
]

os.makedirs("test_snaps", exist_ok=True)

for i in range(5):
    print(f"--- Capture {i+1} ---")
    for url in urls:
        name = url.split("/")[-1].split(".")[0]
        try:
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=3) as r:
                data = r.read()
                filename = f"test_snaps/{name}_{i+1}.jpg"
                with open(filename, "wb") as f:
                    f.write(data)
                print(f"{url} -> status={r.status}, size={len(data)}")
        except urllib.error.HTTPError as e:
            print(f"{url} -> error {e.code} {e.reason}")
        except Exception as e:
            print(f"{url} -> error {e}")
    time.sleep(1)
