# Combined IMU acceleration message — linear + angular.
# Mirrors the layout of a 6D force/wrench vector (acc + ang_acc).
std_msgs/Header header
geometry_msgs/Vector3 linear     # ax, ay, az  (m/s^2 or sim units/s^2)
geometry_msgs/Vector3 angular    # alpha_x, alpha_y, alpha_z  (rad/s^2)
