function Results_CUSE(t,bd,b,bt,vt,R,Q,fm,tau,Rd,Omd,dOmd)


figure
plot3(bd(1,:),bd(2,:),bd(3,:),'k','LineWidth',1.5,'LineStyle',':');
hold on
plot3(b(1,:),b(2,:),b(3,:),'m','LineWidth',1.5);
grid on
xlabel('$X\ \mathrm{(m)}$','interpreter','latex','Fontsize',16)
ylabel('$Y\ \mathrm{(m)}$','interpreter','latex','Fontsize',16)
zlabel('$Z\ \mathrm{(m)}$','interpreter','latex','Fontsize',16)
%axis([-l l -l l -l l])
%set(gcf,'Position',[100 500 500 220])
%axis equal
ax=0.15; % Body axes length
bx=0.25; % desired axes length
 for m=1:100:length(t)
    plot3([b(1,m) ax*R(1,1,m)+b(1,m)], [b(2,m) -ax*R(2,1,m)+b(2,m)], [b(3,m) -ax*R(3,1,m)+b(3,m)] ,'g','linewidth',3);
    plot3([b(1,m) ax*R(1,2,m)+b(1,m)], [b(2,m) -ax*R(2,2,m)+b(2,m)], [b(3,m) -ax*R(3,2,m)+b(3,m)] ,'b','linewidth',3);
    plot3([b(1,m) ax*R(1,3,m)+b(1,m)], [b(2,m) -ax*R(2,3,m)+b(2,m)], [b(3,m) -ax*R(3,3,m)+b(3,m)] ,'r','linewidth',3);
    hold on
 end
    locs = axis; % get current axis boundaries
        hold on;
        plot3([0 locs(2)], [0 0], [0 0],'g','LineWidth',2);
        plot3([0 0], [0 locs(4)], [0 0],'b','LineWidth',2);
        plot3([0 0], [0 0], [0 locs(6)],'r','LineWidth',2);
        plot3([locs(1) 0], [0 0], [0 0],'g--','LineWidth',2);
        plot3([0 0], [locs(3) 0], [0 0],'b--','LineWidth',2);
        plot3([0 0], [0 0], [locs(5) 0],'r--','LineWidth',2);
az = 139;
el = 24;
legend('$b^d$ desired trajectory','$b$ achieved trajectory','Orientation','vertical','Location','NorthEast');
set(legend, 'Box', 'off')
h = legend;
set(h, 'interpreter', 'latex','fontsize',14)
view(az, el);

figure
plot(t,bt(1,:),':',t,bt(2,:),'--',t,bt(3,:),'LineWidth',1.5);
xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
ylabel('$\tilde{b}$ (m)','interpreter','latex','fontsize',18);
legend('$\tilde{b}_{x}$','$\tilde{b}_{y}$','$\tilde{b}_{z}$','Orientation','horizontal','Location','SouthEast');
set(legend, 'Box', 'off')
h = legend;
set(gcf,'Position',[100 400 500 220])
set(h, 'interpreter', 'latex','fontsize',18)
%title('Control Torque','interpreter', 'latex','fontsize',18)
grid on

figure
plot(t,vt(1,:),':',t,vt(2,:),'-.',t,vt(3,:),'LineWidth',1.5);
xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
ylabel('$\tilde{v}$ (m/s)','interpreter','latex','fontsize',18);
legend('$\tilde{v}_{x}$','$\tilde{v}_{y}$','$\tilde{v}_{z}$','Orientation','horizontal','Location','NorthEast');
set(legend, 'Box', 'off')
h = legend;
set(gcf,'Position',[100 400 500 220])
set(h, 'interpreter', 'latex','fontsize',18)
%title('Control Torque','interpreter', 'latex','fontsize',18)
grid on


figure
plot(t(1:length(t)-1),fm,'LineWidth',2);
xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
ylabel('$f$ (N)','interpreter','latex','fontsize',18);
%legend('$\tilde{v}_{x}$','$\tilde{v}_{y}$','$\tilde{v}_{z}$','Orientation','horizontal','Location','NorthEast');
set(legend, 'Box', 'off')
h = legend;
set(gcf,'Position',[100 400 500 220])
set(h, 'interpreter', 'latex','fontsize',18)
%title('Control Torque','interpreter', 'latex','fontsize',18)
grid on

l=length(t);

for k = 1:l
nbd(k)=norm(bd(:,k));
nb(k)=norm(b(:,k));
nbt(k)=norm(bt(:,k));
nvt(k)=norm(vt(:,k));
end

figure
plot(t,nbt,'LineWidth',2);
xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
ylabel('$\|\tilde{b}\|$ (m)','interpreter','latex','fontsize',18);
%legend('$\tau_{x}$','$\tau_{y}$','$\tau_{z}$','Orientation','horizontal','Location','NorthEast');
set(legend, 'Box', 'off')
h = legend;
set(gcf,'Position',[100 400 500 220])
set(h, 'interpreter', 'latex','fontsize',18)
%title('Control Torque','interpreter', 'latex','fontsize',18)
grid on

figure
plot(t,nvt,'LineWidth',2);
xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
ylabel('$\|\tilde{v}\|$ (m/s)','interpreter','latex','fontsize',18);
%legend('$\tau_{x}$','$\tau_{y}$','$\tau_{z}$','Orientation','horizontal','Location','NorthEast');
set(legend, 'Box', 'off')
h = legend;
set(gcf,'Position',[100 400 500 220])
set(h, 'interpreter', 'latex','fontsize',18)
%title('Control Torque','interpreter', 'latex','fontsize',18)
grid on

n=length(t)-1;
for k = 1:n
Phi(k)= norm(logmso3(Q(:,:,k)));
ntau(k)=norm(tau(:,k));
end

figure
plot(t(1:n),Phi,'Linewidth',2);
xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
ylabel('$\|\Phi\|$ rad','interpreter','latex','fontsize',18);
%legend('$\tau_{x}$','$\tau_{y}$','$\tau_{z}$','Orientation','horizontal','Location','NorthEast');
set(legend, 'Box', 'off')
h = legend;
set(gcf,'Position',[100 400 500 220])
set(h, 'interpreter', 'latex','fontsize',18)
%title('Control Torque','interpreter', 'latex','fontsize',18)
grid on

figure
plot(t(1:length(t)-1),tau(1,:),':',t(1:length(t)-1),tau(2,:),'-.',t(1:length(t)-1),tau(3,:),'LineWidth',1.5);
xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
ylabel('$ \tau $ N-m','interpreter','latex','fontsize',18);
%legend('$\tilde{s}^d_{x}$','$\tilde{s}^d_{y}$','$\tilde{s}^d_{z}$','Orientation','horizontal','Location','NorthEast');
%set(legend, 'Box', 'off')
h = legend;
set(gcf,'Position',[100 400 500 220])
set(h, 'interpreter', 'latex','fontsize',18)
% title('Control Torque','interpreter', 'latex','fontsize',18)
grid on

figure
plot(t(1:length(t)),Omd(1,:),':',t(1:length(t)),Omd(2,:),'-.',t(1:length(t)),Omd(3,:),'LineWidth',1.5);
xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
ylabel('$ \Omega_d $','interpreter','latex','fontsize',18);
legend('$ \Omega_{d_x}$','$\Omega_{d_y}$','$\Omega_{d_z}$','Orientation','horizontal','Location','NorthEast');
set(legend, 'Box', 'off')
h = legend;
set(gcf,'Position',[100 400 500 220])
set(h, 'interpreter', 'latex','fontsize',18)
% title('Control Torque','interpreter', 'latex','fontsize',18)
grid on

figure
plot(t(1:length(t)-1),dOmd(1,:),':',t(1:length(t)-1),dOmd(2,:),'-.',t(1:length(t)-1),dOmd(3,:),'LineWidth',1.5);
xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
ylabel('$ \dot{\Omega}_d $','interpreter','latex','fontsize',18);
legend('$\dot{\Omega}_{d_x}$','$\dot{\Omega}_{d_y}$','$\dot{\Omega}_{d_z}$','Orientation','horizontal','Location','NorthEast');
set(legend, 'Box', 'off')
h = legend;
set(gcf,'Position',[100 400 500 220])
set(h, 'interpreter', 'latex','fontsize',18)
% title('Control Torque','interpreter', 'latex','fontsize',18)
grid on

figure
plot(t(1:length(t)-1),ntau,'LineWidth',2);
xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
ylabel('$\|\tau\| $ N-m','interpreter','latex','fontsize',18);
%legend('$\tilde{s}^d_{x}$','$\tilde{s}^d_{y}$','$\tilde{s}^d_{z}$','Orientation','horizontal','Location','NorthEast');
%set(legend, 'Box', 'off')
h = legend;
set(gcf,'Position',[100 400 500 220])
set(h, 'interpreter', 'latex','fontsize',18)
% title('Control Torque','interpreter', 'latex','fontsize',18)
grid on

