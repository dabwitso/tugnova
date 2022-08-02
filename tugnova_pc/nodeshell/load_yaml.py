import yaml
import sys

ymalFile = sys.argv[1] # (1) Load yaml file.
ns = sys.argv[2]       # (2) Target namespace.
# (3) Ignore params.
#   Example1: ignore "x" param          ->  x
#   Example2: ignore "x" and "y" params -> "x,y"
ignores = [] if len(sys.argv) <= 3 else sys.argv[3].split(",") 

f = open(ymalFile, "r")
data = yaml.load(f)

result = ""
for key, value in data.get(ns).items():
  if key not in ignores:
    result += (key + ":=" + str(value) + " ")

print result
