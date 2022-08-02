import yaml
import sys
import json


ymalFile = sys.argv[1] # (1) Load yaml file.
ns = sys.argv[2]       # (2) Target namespace.
# (3) Ignore params.
#   Example1: ignore "x" param          ->  x
#   Example2: ignore "x" and "y" params -> "x,y"
ignores = [] if len(sys.argv) <= 3 else sys.argv[3].split(",") 

f = open(ymalFile, "r")
data = yaml.load(f)

result = data.get(ns)

for ignore in ignores:
  result.pop(ignore)

result["header"] = {
  "seq": 0,
  "stamp": { "secs": 0, "nsecs": 0 },
  "frame_id": ''
}

print json.dumps(result).replace(" ", "")
