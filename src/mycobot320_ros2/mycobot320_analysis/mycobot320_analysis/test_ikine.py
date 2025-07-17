import numpy as np
from mycobot320_analysis.utils import create_robot

robot = create_robot()

# Probar con una configuración conocida dentro del workspace
q_test = np.array([0.0, 0.4, 0.5, -0.9, 0.2, 0.1])

# Cinemática directa con TU modelo
pose = robot.fkine_all(q_test)[-1]
print("POSE generada por fkine:")
print(pose)

# Probar IK con 1 configuración
for conf in [[1,1,1], [1,1,-1], [1,-1,1], [-1,1,1]]:
    q_sol, status = robot.ikine_a(pose, conf)
    if status == 1:
        print(f"Config {conf} → IK ok: q = {np.round(q_sol, 3)}")
        print(f"FK con q_sol → pose:\n{robot.fkine_all(q_sol)[-1]}")
    else:
        print(f"Config {conf} → IK falló")
