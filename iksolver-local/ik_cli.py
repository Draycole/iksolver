import numpy as np
import math

L = 5 #this is the link length. you can adjust this yourself.

x = float(input("Enter X: "))
y = float(input("Enter Y: "))


t2 = np.arccos(1 - ((x**2 + y**2) / (2 * L**2)))
t1 = np.arccos(np.sqrt(x**2 + y**2) / (2 * L))

# alpha 1 (absolute angle of link 1)
a1 = math.degrees(t1 + math.atan2(y, x))

# compute joint 1 coordinates
x1 = L * math.cos(math.radians(a1))
y1 = L * math.sin(math.radians(a1))

# absolute orientation of link 2 (CCW)
theta2_abs = math.atan2(y - y1, x - x1)

a2 = -math.degrees(theta2_abs)

print("α₁ =", a1)
print("α₂ =", a2)

