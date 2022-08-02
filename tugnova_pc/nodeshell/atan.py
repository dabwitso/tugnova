import math
import sys
import yaml

distance = float(sys.argv[1])
ymalFile = sys.argv[2]

f = open(ymalFile, "r")
data = yaml.load(f)
wheel_base = float(data['vehicle_info']['wheel_base'])


print (round(math.atan(distance/wheel_base),2))
