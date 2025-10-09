import sympy as np

# Define symbols
rx, ry, width = np.symbols('rx ry width', real=True)

# rx, ry are in radians
A = np.sqrt(1 + np.tan(rx)**2 + np.tan(ry)**2)
B = np.sqrt(np.tan(rx)**2 + np.tan(ry)**2)

# 1) sin(theta/2) * cos(phi)
D1 = 2 * width * np.sqrt((A - 1) / (2 * A)) * (np.tan(ry) / B)

# 3) sin(theta/2) * cos(phi - 2/3 * pi)
D2 = 2 * width * np.sqrt((A - 1) / (2 * A)) * ((-np.tan(ry) + np.sqrt(3) * np.tan(rx)) / (2 * B))

# 3) sin(theta/2) * cos(phi - 4/3 * pi)
D3 = 2 * width * np.sqrt((A - 1) / (2 * A)) * ((-np.tan(ry) - np.sqrt(3) * np.tan(rx)) / (2 * B))

D1_simplified = np.simplify(D1)
D2_simplified = np.simplify(D2)
D3_simplified = np.simplify(D3)

print("Analytical distance:")
np.pretty_print(D1_simplified)
# np.pretty_print(D2_simplified)
# np.pretty_print(D3_simplified)