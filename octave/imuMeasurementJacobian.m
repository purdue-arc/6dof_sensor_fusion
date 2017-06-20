syms x y z dx dy dz ax ay az q0 q1 q2 q3 wx wy wz
syms dt

syms G

state = [x, y, z, dx, dy, dz, ax, ay, az, q0, q1, q2, q3, wx, wy, wz]

syms imu_ax imu_ay imu_az imu_wx imu_wy imu_wz

measure = [imu_ax; imu_ay; imu_az; imu_wx; imu_wy; imu_wz]

h = [(1-2*q2^2 - 2*q3^2)*(ax) + 2*(q1*q2 + q0*q3)*ay + 2*(q1*q3 - q0*q2)*(az+G);
     2*(q1*q2 - q0*q3)*ax + (1-2*q1^2 - 2*q3^2)*ay + 2*(q2*q3 + q0*q1)*(az+G);
     2*(q1*q3 + q0*q2)*ax + 2*(q2*q3 - q0*q1)*ay + (1 - 2*q1^2 - 2*q2^2)*(az+G);
     wx;
     wy;
     wz];
 
 
 jaco = jacobian(h, state)
 
 ccode(jaco, 'file', 'imuMeasurementJacobian.c')