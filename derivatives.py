import sympy as sp

# Define the symbolic variables
rx, ry, lp, lm2, beta, gamma = sp.symbols('rx ry lp lm2 beta gamma', real=True)

# Define the expression
numerator = (
    sp.cos(2*rx)*sp.cos(2*ry)*(lp*sp.cos(beta) + lm2*sp.cos(gamma))
    + sp.cos(2*rx)**2*sp.sin(2*ry)*(lp*sp.sin(beta) + lm2*sp.sin(gamma))
)
denominator = sp.sqrt(sp.cos(2*rx)**2 + sp.sin(2*rx)**2 * sp.cos(2*ry)**2)

vm2z = numerator / denominator

# Derivatives
dvm2z_drx = sp.diff(vm2z, rx)
dvm2z_dry = sp.diff(vm2z, ry)

# Simplify results
dvm2z_drx_simplified = sp.simplify(dvm2z_drx)
dvm2z_dry_simplified = sp.simplify(dvm2z_dry)

# Display the results
print("∂vm2z/∂rx =")
sp.pprint(dvm2z_drx_simplified, use_unicode=True)

print("\n∂vm2z/∂ry =")
sp.pprint(dvm2z_dry_simplified, use_unicode=True)
