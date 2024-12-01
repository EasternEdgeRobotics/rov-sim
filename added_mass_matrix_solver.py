import capytaine as cpt
import numpy as np

cpt.set_logging('DEBUG')

# Based on example for Capytaine Docs (https://capytaine.github.io/stable/user_manual/cookbook.html#added-mass-of-a-rigid-body)

# ----------------------------------------------------------
# ----------==== INDICATE STL MOCDEL PATH HERE ====---------
# ----------------------------------------------------------

beaumont_mesh = cpt.load_mesh("models/Beaumont/chassis_assembly_for_added_mass_matrix_analysis.stl", file_format="stl")

# ----------------------------------------------------------
# ----------------------------------------------------------
# ----------------------------------------------------------


beaumont_chassis_rigid_body = cpt.FloatingBody(
        mesh=beaumont_mesh)

# Automatically add the six degrees of freedom of a rigid body
beaumont_chassis_rigid_body.add_all_rigid_body_dofs()

# Set up the problems: we will solve a radiation problem for each
# degree of freedom of the body, setting wave frequency to infinity as recommended 
# in https://gazebosim.org/api/sim/9/theory_hydrodynamics.html
problems = [
    cpt.RadiationProblem(body=beaumont_chassis_rigid_body, radiating_dof=dof, omega=np.inf)
    for dof in beaumont_chassis_rigid_body.dofs
]
# Water density, gravity and water depth have not been specified.
# Default values are used.

# Solve all radiation problems
solver = cpt.BEMSolver()
results = solver.solve_all(problems)

# Gather the computed added mass into a labelled array.
data = cpt.assemble_dataset(results)

added_mass = data['added_mass'].sel(radiating_dof='Surge', influenced_dof='Surge').values
print(f"Added mass in Surge: {added_mass}")

added_mass = data['added_mass'].sel(radiating_dof='Sway', influenced_dof='Sway').values
print(f"Added mass in Sway: {added_mass}")

added_mass = data['added_mass'].sel(radiating_dof='Heave', influenced_dof='Heave').values
print(f"Added mass in Heave: {added_mass}")

added_mass = data['added_mass'].sel(radiating_dof='Roll', influenced_dof='Roll').values
print(f"Added mass in Roll: {added_mass}")

added_mass = data['added_mass'].sel(radiating_dof='Pitch', influenced_dof='Pitch').values
print(f"Added mass in Pitch: {added_mass}")

added_mass = data['added_mass'].sel(radiating_dof='Yaw', influenced_dof='Yaw').values
print(f"Added mass in Yaw: {added_mass}")