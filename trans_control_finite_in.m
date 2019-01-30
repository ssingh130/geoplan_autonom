function [f]=trans_control_finite_in(R,bt,nu,vd,dvd)
%(R,bt,vt,z,dz,dvd)
%(R,bt,nu,vd,dvd)

global kt e3 g m Pr pt

% parameter
vt=R*nu-vd;
z=bt/((bt'*bt)^(1-(1/pt)));

dz=(((bt'*bt)^(1-1/pt))*...
    vt-(2-(2/pt))*((bt'*bt)^(-1/pt))*...
    (bt'*vt)*bt)/((bt'*bt)^(2-(2/pt)));

L=(kt*Pr*(vt+kt*z))/((vt+kt*z)'*Pr*(vt+kt*z))^(1-(1/pt));

%f=e3'*R'*(m*(g*e3+dvd+kt*dz+kt*bt+((kt*Pr*(vt+kt*z))/((vt+kt*z)'*Pr*(vt+kt*z))^(1-1/p))));
f=e3'*R'*(m*(g*e3-dvd+kt*dz+kt*bt+L));
%f = e3'*R'*(m*g*e3+P*bt+L*(R*nu-vd)-m*dvd);
%f=m*g*e3+P*bt+L*(R*nu-vd)-m*dvd;
