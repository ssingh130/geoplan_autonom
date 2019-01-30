
function Rd=desired_attitude_dot(bt,vt,dvd)

global m e3 Pr pt kt g e1

z = bt/((bt'*bt)^(1-(1/pt)));
dz = (((bt'*bt)^(1-1/pt))*...
    vt-(2-(2/pt))*((bt'*bt)^(-1/pt))*...
    (bt'*vt)*bt)/((bt'*bt)^(2-(2/pt)));

L=(kt*Pr*(vt+kt*z))/((vt+kt*z)'*Pr*(vt+kt*z))^(1-(1/pt));
u = (g*e3-dvd+kt*dz+kt*bt+L);

r3d=(m*u)/norm(m*u);

sd = e1;
r2d = cross(e1,r3d); 
r2d = r2d / norm(r2d);
r1d = cross(r2d,r3d);

Rd = [r1d r2d r3d];
