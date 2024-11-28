import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from scipy.optimize import fsolve

# Parámetros del mecanismo en mm
motor_separation = 47.30     # Separación entre motores 
long_eje = 30                # Longitud de los ejes (pitch)
long_rod = 43                # Longitud de los rods (conexión entre ejes y plataformas)
platform_length = 60         # Longitud de la plataforma (eje Y)
platform_width = 47.30       # Ancho de la plataforma (eje X)

# Sistema de Coordenadas
# Origen en el punto medio entre los dos motores en la base
A1 = np.array([-motor_separation / 2, 0.0, 0.0])  # Posición del Motor 1
A2 = np.array([motor_separation / 2, 0.0, 0.0])   # Posición del Motor 2

def calculate_positions(alpha1_deg, alpha2_deg):
    """
    Calcula las posiciones de los componentes del mecanismo basado en los ángulos de los servos.
    
    Parámetros:
    - alpha1_deg: Ángulo del servo 1 en grados
    - alpha2_deg: Ángulo del servo 2 en grados
    
    Retorna:
    - B1: Posición del extremo del eje 1
    - B2: Posición del extremo del eje 2
    - C1: Posición de la conexión del rod 1 en la plataforma
    - C2: Posición de la conexión del rod 2 en la plataforma
    - platform_center: Posición del centro de la plataforma
    """
    # Convertir ángulos de servos a radianes
    alpha1 = np.radians(alpha1_deg)
    alpha2 = np.radians(alpha2_deg)
    
    # Posiciones de los extremos de los ejes
    B1 = A1 + long_eje * np.array([0, np.cos(alpha1), np.sin(alpha1)])
    B2 = A2 + long_eje * np.array([0, np.cos(alpha2), np.sin(alpha2)])
    
    # Relacionar ángulos de servos con pitch y roll
    theta = (alpha1 + alpha2) / 2    # Pitch en radianes
    phi = (alpha1 - alpha2) / 2      # Roll en radianes
    
    # Función para resolver las ecuaciones
    def equations(vars):
        x_c, y_c, z_c = vars
        
        # Matrices de rotación
        R_pitch = np.array([
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)]
        ])
        R_roll = np.array([
            [1, 0, 0],
            [0, np.cos(phi), -np.sin(phi)],
            [0, np.sin(phi), np.cos(phi)]
        ])
        R = R_roll @ R_pitch  # Rotación combinada
        
        # Vectores desde el centro de la plataforma a C1 y C2
        r_C1 = np.array([-platform_width / 2, 0, 0])
        r_C2 = np.array([platform_width / 2, 0, 0])
        
        # Posiciones de C1 y C2
        C1 = np.array([x_c, y_c, z_c]) + R @ r_C1
        C2 = np.array([x_c, y_c, z_c]) + R @ r_C2
        
        # Ecuaciones de distancia
        eq1 = np.linalg.norm(B1 - C1) - long_rod
        eq2 = np.linalg.norm(B2 - C2) - long_rod
        eq3 = y_c  # Mantener el centro de la plataforma en Y=0
        
        return [eq1, eq2, eq3]
    
    # Estimaciones iniciales
    x_c_guess = 0.0
    y_c_guess = 0.0
    z_c_guess = (B1[2] + B2[2]) / 2 + 10.0  # Estimación inicial
    
    # Resolver las ecuaciones
    solution, infodict, ier, mesg = fsolve(equations, [x_c_guess, y_c_guess, z_c_guess], full_output=True)
    
    if ier != 1:
        print("No se encontró una solución:", mesg)
    
    x_c, y_c, z_c = solution
    
    # Calcular las posiciones finales de C1 y C2
    R_pitch = np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])
    R_roll = np.array([
        [1, 0, 0],
        [0, np.cos(phi), -np.sin(phi)],
        [0, np.sin(phi), np.cos(phi)]
    ])
    R = R_roll @ R_pitch
    r_C1 = np.array([-platform_width / 2, 0, 0])
    r_C2 = np.array([platform_width / 2, 0, 0])
    C1 = np.array([x_c, y_c, z_c]) + R @ r_C1
    C2 = np.array([x_c, y_c, z_c]) + R @ r_C2
    
    # Posición del centro de la plataforma
    platform_center = np.array([x_c, y_c, z_c])
    
    return B1, B2, C1, C2, platform_center

def plot_mechanism(theta_deg, phi_deg, ax):
    """
    Visualiza el mecanismo en 3D basado en los ángulos deseados de pitch y roll.
    
    Parámetros:
    - theta_deg: Ángulo deseado de pitch en grados
    - phi_deg: Ángulo deseado de roll en grados
    - ax: Objeto de ejes de Matplotlib
    """
    ax.clear()
    
    # Convertir ángulos de pitch y roll a radianes
    theta = np.radians(theta_deg)
    phi = np.radians(phi_deg)
    
    # Calcular los ángulos de los servos a partir de theta y phi
    alpha1 = theta + phi
    alpha2 = theta - phi
    
    # Convertir ángulos de los servos a grados
    alpha1_deg = np.degrees(alpha1)
    alpha2_deg = np.degrees(alpha2)
    
    # Limitar los ángulos de los servos para evitar movimientos excesivos
    alpha1_deg = np.clip(alpha1_deg, -30, 30)
    alpha2_deg = np.clip(alpha2_deg, -30, 30)
    
    # Calcular posiciones
    B1, B2, C1, C2, platform_center = calculate_positions(alpha1_deg, alpha2_deg)
    
    # Plot de los motores
    ax.scatter(A1[0], A1[1], A1[2], color='red', s=50, label='Motor 1')
    ax.scatter(A2[0], A2[1], A2[2], color='green', s=50, label='Motor 2')
    
    # Plot de los extremos de los ejes
    ax.scatter(B1[0], B1[1], B1[2], color='orange', s=50, label='Eje 1')
    ax.scatter(B2[0], B2[1], B2[2], color='yellow', s=50, label='Eje 2')
    
    # Plot de las conexiones de los rods en la plataforma
    ax.scatter(C1[0], C1[1], C1[2], color='blue', s=50, label='C1')
    ax.scatter(C2[0], C2[1], C2[2], color='purple', s=50, label='C2')
    
    # Plot del centro de la plataforma
    ax.scatter(platform_center[0], platform_center[1], platform_center[2], color='black', s=50, label='Centro Plataforma')
    
    # Plot de los ejes
    ax.plot([A1[0], B1[0]], [A1[1], B1[1]], [A1[2], B1[2]], color='red')
    ax.plot([A2[0], B2[0]], [A2[1], B2[1]], [A2[2], B2[2]], color='green')
    
    # Plot de los rods
    ax.plot([B1[0], C1[0]], [B1[1], C1[1]], [B1[2], C1[2]], color='blue')
    ax.plot([B2[0], C2[0]], [B2[1], C2[1]], [B2[2], C2[2]], color='purple')
    
    # Plot de la plataforma
    platform_vertices = np.array([C1, C2, platform_center, C1])  # Triángulo para visualización simple
    ax.plot(platform_vertices[:,0], platform_vertices[:,1], platform_vertices[:,2], color='black')
    
    # Configuración de límites y etiquetas
    limit = motor_separation + long_eje + long_rod + platform_length
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit / 2, limit / 2)
    ax.set_zlim(0, limit)
    ax.set_xlabel('Eje X (mm)')
    ax.set_ylabel('Eje Y (mm)')
    ax.set_zlabel('Eje Z (mm)')
    
    # Mostrar los ángulos en el título
    ax.set_title(f'Pitch Deseado: {theta_deg:.1f}°, Roll Deseado: {phi_deg:.1f}°\n'
                 f'Ángulo Servo 1: {alpha1_deg:.1f}°, Ángulo Servo 2: {alpha2_deg:.1f}°')
    
    # Leyenda
    ax.legend(loc='upper right')
    
    # Habilitar la cuadrícula y ajustar la vista
    ax.grid(True)
    ax.view_init(elev=30, azim=45)
    plt.draw()

def main():
    # Crear la figura y los ejes en 3D
    fig = plt.figure(figsize=(10, 8))
    ax_main = fig.add_subplot(111, projection='3d')
    plt.subplots_adjust(left=0.1, bottom=0.25)
    
    # Inicializar el mecanismo con ángulos de pitch y roll en 0
    initial_theta = 0.0
    initial_phi = 0.0
    plot_mechanism(initial_theta, initial_phi, ax_main)
    
    # Crear sliders para Pitch y Roll
    ax_theta = plt.axes([0.1, 0.1, 0.8, 0.03])
    slider_theta = Slider(
        ax=ax_theta,
        label='Pitch Deseado (°)',
        valmin=-30,
        valmax=30,
        valinit=initial_theta,
        valstep=1
    )
    
    ax_phi = plt.axes([0.1, 0.05, 0.8, 0.03])
    slider_phi = Slider(
        ax=ax_phi,
        label='Roll Deseado (°)',
        valmin=-30,
        valmax=30,
        valinit=initial_phi,
        valstep=1
    )
    
    # Función de actualización cuando los sliders cambian
    def update(val):
        theta = slider_theta.val
        phi = slider_phi.val
        plot_mechanism(theta, phi, ax_main)
    
    # Conectar los sliders con la función de actualización
    slider_theta.on_changed(update)
    slider_phi.on_changed(update)
    
    plt.show()

if __name__ == "__main__":
    main()
