syms x y z dx dy dz ax ay az q0 q1 q2 q3 wx wy wz
syms dt

state = [x, y, z, dx, dy, dz, ax, ay, az, q0, q1, q2, q3, wx, wy, wz]

omega = sqrt(wx*wx*dt*dt + wy*wy*dt*dt +  wz*wz*dt*dt);

vx = wx/omega;
vy = wy/omega;
vz = wz/omega;

dq0 = cos(omega/2);
dq1 = vx*sin(omega/2);
dq2 = vy*sin(omega/2);
dq3 = vz*sin(omega/2);

trans = [x + dx*dt + 0.5*dt*dt*ax;
	y + dy*dt + 0.5*dt*dt*ay;
	z + dz*dt + 0.5*dt*dt*az;
	dx + ax*dt;
	dy + ay*dt;
	dz + az*dt;
	ax;
	ay;
	az;
  dq0*q0 - dq1*q1 - dq2*q2 - dq3*q3;
  dq0*q1 + dq1*q0 - dq2*q3 + dq3*q2;
  dq0*q2 + dq1*q3 + dq2*q0 - dq3*q1;
  dq0*q3 - dq1*q2 + dq2*q1 + dq3*q0;
  wx;
  wy;
  wz];
  
  jaco = jacobian(trans, state)
  
  ccode(jaco, 'file', 'stateTransitionJacobian.c')
  
 


