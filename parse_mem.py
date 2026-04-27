import json
with open('_mem_test_history.json') as f:
    data = json.load(f)

for d in data:
    if d['time'] % 10 == 0 or d['time'] == 37:
        print(f"Time: {d['time']}s Phase: {d['phase']} ptz_stream: {d.get('ptz_stream.py', 0):.1f}MB")
