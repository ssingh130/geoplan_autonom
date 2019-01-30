function [t,R,Rd,b,fm,tau,tau1,tau2,tau3,tau4,nu,Omd,dOmd,Om,Q,ydes,vdes,dvd,bt,vt] = LGVI_SE3_UAV(b0,R0,nu0,Om0,dvd0,t0,tf,h)

global J M m e3 g

b(:,1) = b0;
R(:,:,1) = R0;
nu(:,1) = nu0;
Om(:,1) = [0;0;0];
Delt = tf-t0;                 
n = fix(Delt/h);

t(1) = 0;
 
% bd(:,1)=[0.4*sin(pi*0);0.6*cos(pi*0);0.4*0];
% vd(:,1)=[0.4*pi*cos(pi*0);-0.6*pi*sin(pi*0);0.4];
% dvd(:,1)=[-0.4*pi^2*sin(pi*0);-0.6*pi^2*cos(pi*0);0];
bd(:,1)=[0.9999;0.01;0.002];
% bd(:,1)=[-1.2;0.01;0.002]; %for cosine traj
vd(:,1)=[0;0;0];
dvd(:,1)=[0;0;0];


bt(:,1) = b0-bd(:,1); % initial position error
vt(:,1) = R(:,:,1)*nu(:,1)-vd(:,1); % initial velocity error

Rd(:,:,1) = desired_attitude_dot(bt(:,1),vt(:,1),dvd(:,1)); % initial desired attitude

bd_1=[0.01;0.01;0.01];
vd_1=[0;0;0];
dvd_1=[0;0;0];

Omd(:,1) = Om0;
for j=1:15
    
    for k=1:n-1 
    t(k+1)=t(k)+h;    
    
%     bd(:,k+1)=[0.4*sin(pi*t(k+1));0.6*cos(pi*t(k+1));0.4*t(k+1)];
%     vd(:,k+1)=[0.4*pi*cos(pi*t(k+1));-0.6*pi*sin(pi*t(k+1));0.4];
%     dvd(:,k+1)=[-0.4*pi^2*sin(pi*t(k+1));-0.6*pi^2*cos(pi*t(k+1));0];

%     bd(:,k+1)=[cos(t(k+1))./(1+sin(t(k+1)).^2);cos(t(k+1)).*sin(t(k+1))./(1+sin(t(k+1)).^2);0.2*t(k+1)];
%     vd(:,k+1)=(1/h)*(bd(:,k+1)-bd(:,k));
%     dvd(:,k+1)=(1/h)*(vd(:,k+1)-vd(:,k));
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
   %Sine trajectroy
    bd(:,k+1)=[12.*sin(0.25*pi*t(k+1));12.*t(k+1);30];
    vd(:,k+1)=[12*0.25.*pi.*cos(0.25*pi*t(k+1));12;0];
    dvd(:,k+1)=[-12*0.25^2.*pi.^2.*sin(0.25*pi*t(k+1));0;0];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     Cosine Trajectory just to change the initial starting point
%      bd(:,k+1)=[1.2.*cos(0.25*pi*t(k+1));1.2.*t(k+1);30];
%      vd(:,k+1)=[-1.2*0.25.*pi.*sin(0.25*pi*t(k+1));1.2;0];
%      dvd(:,k+1)=[-1.2*0.25^2.*pi.^2.*cos(0.25*pi*t(k+1));0;0];

%     HAver sine function applied to the trajectory
%     bd(:,k+1)=;
%     vd(:,k+1)=[0.8.*pi.*cos(pi*t(k+1));0.8;0];
%     dvd(:,k+1)=[-0.8.*pi.^2.*sin(pi*t(k+1));0;0];

% a simple trajectory for checking the gains
% bd(:,k+1)=[t(k+1);2*t(k+1);t(k+1)];
% vd(:,k+1)=(1/h)*(bd(:,k+1)-bd(:,k));
% dvd(:,k+1)=(1/h)*(vd(:,k+1)-vd(:,k));

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fi = h*Om(:,k);
    F(:,:,k) = expmso3(fi);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    R(:,:,k+1) = R(:,:,k)*F(:,:,k);       % next attitude
    b(:,k+1) = h*R(:,:,k)*nu(:,k)+b(:,k); % the next position
    
    %gains for ref gov
    
    n = length(b0);
    L = size(bd,2);
    ydes(:,1) = bd(:,1);
    L1 = 0.98*eye(n);
    gam=0.08;
    p=9/7;
    ep = 1-1/p;
    
        for k=1:L-1
            Edk=ydes(:,k)-bd(:,k);
            edk = (Edk'*L1*Edk)^ep;
            Bcal=(edk-gam)/(edk+gam);
            Edkpl = Bcal*Edk;
            ydes(:,k+1)=bd(:,k+1)+Edkpl;
        end
    
%ref gov for velocity

        a=length(nu0);
        A1=size(vd,2); 
% length of the input reference to be governed

        vdes(:,1)=vd(:,1); % initial output reference from governor

% Governor gains
        Lv=0.98*eye(a); 
        gam1=0.06;
        p1=9/8;
        ep1 = 1-1/p1;
        
        for ks=1:A1-1
            Edk=vdes(:,ks)-vd(:,k);
            edk= (Edk'*Lv*Edk)^ep1;
            Bcal=(edk-gam1)/(edk+gam1);
            Edkp1=Bcal*Edk+0.03;
            vdes(:,ks+1)=vd(:,k+1)+Edkp1;
        end
    % The control torque:
    bt(:,k+1)=b(:,k+1) - ydes(:,k+1);
    
    fm(k)=trans_control_finite_in(R(:,:,k),bt(:,k),nu(:,k),vdes(:,ks),dvd(:,k)); % Gives the thrust magnitude. 
   
    nu(:,k+1)=M\(F(:,:,k)'*M*nu(:,k)+h*m*g*R(:,:,k+1)'*e3-h*fm(k)*e3);
   

    v(:,k+1) = R(:,:,k+1)*nu(:,k+1);
    vt(:,k+1) = v(:,k+1)-vdes(:,ks+1);
    
    %Rd(:,:,k+1)=desired_attitude_dot(fm(:,k),R(:,:,k))
    Rd(:,:,k+1) = desired_attitude_dot(bt(:,k+1),vt(:,k+1),dvd(:,k+1));
    
    Fd(:,:,k+1) = Rd(:,:,k)'*Rd(:,:,k+1);
    
    Omd(:,k+1) = (1/h)*(logmso3(Fd(:,:,k+1)));
    
    dOmd(:,k) = (1/h)*(Omd(:,k+1)-Omd(:,k));
    
    Q(:,:,k) = Rd(:,:,k)'*R(:,:,k);
    
    %om(:,k)=Om(:,k)-Q(:,:,k)'*Omd(:,k)
   
    [tau(:,k),tau1(:,k),tau2(:,k),tau3(:,k),tau4(:,k)] = attitude_control_tau(Om(:,k),Omd(:,k),dOmd(:,k),Q(:,:,k));
        
    Om(:,:,k+1) = J\((F(:,:,k)'*J*Om(:,:,k))+h*tau(:,k));
    end
    b_1 = v(:,2)*h*0.5; 
    v_1 = v(:,3)*0.5 - 2*v(:,2); 
    bt_1 = b_1 - bd_1; 
    vt_1 = v_1 - vd_1; 
    Rd_1 = desired_attitude_dot(bt_1,vt_1,dvd_1);
    Fd(:,:,1) = (Rd_1)'*Rd(:,:,1);
    Omd(:,1) = (1/h)*(logmso3(Fd(:,:,1)));
end

% this part was borrowed from Rakesh's IJC 2018 code which was given to me
% for tuning.