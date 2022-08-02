import yaml
import sys

ymalFile = sys.argv[1] # (1) Load yaml file.
ns = sys.argv[2]       # (2) Target namespace.
kn = sys.argv[3]       # (3) Target keyname.

f = open(ymalFile, "r")
data = yaml.load(f)
for key, value in data.get(ns).items():
    if key == kn:
        print(str(value))
        break
