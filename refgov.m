function vdes=refgov(vd,nu0)
% "Offline" reference governor 


a=length(nu0);
A1=size(vd,2); 
% length of the input reference to be governed

vdes(:,1)=vd(:,1); % initial output reference from governor

% Governor gains
Lv=0.98*eye(a); 
gam=0.07;p=9/7;
ep=1-1/p;

for k=1:A1-1
    Edk=vdes(:,k)-vd(:,k);
    edk= (Edk'*Lv*Edk)^ep;
    Bcal=(edk-gam)/(edk+gam);
    Edkp1=Bcal*Edk;
    vdes(:,k+1)=vd(:,k+1)+Edkp1;
    
end