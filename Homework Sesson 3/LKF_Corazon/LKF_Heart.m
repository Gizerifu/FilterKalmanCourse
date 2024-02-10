clear
close all
%% Modelo corazón

Rc = 10; L = 1; Rp = 5; C = 2;

A = [[-Rc/L, -1/L]; [1/C, -1/Rp]];
B = [1/L; 0];
C = [0 1];
D = 0;

Ts=0.01;
%%Discretización de modelo
Ad=eye(2)+Ts*A;
Bd=Ts*B;
%%Inicialización de variables
t=0:Ts:100;
x=zeros([2,length(t)]);
y=zeros([1,length(t)]);
u=0;
x(:,1)=[6,3]';
w=5e-7;
v=10;
%%Inicialización de filtro de Kalman
Q = [w,w]*[w,w]';
R = v^2;
P = 1e2*eye(2);
alpha = 0.1;

x_pred=zeros([2,length(t)]);
x_est=zeros([2,length(t)]);
y_pred=zeros([1,length(t)]);

x_pred(:,1)=[6,2.1]';
%%Simulación del modelo
for k=2:length(t)

    x(:,k)=Ad*x(:,k-1)+Bd*u+w*randn([2,1]);
    y(k)=C*x(:,k)+v*randn;

    x_pred(:,k)=Ad*x_pred(:,k-1)+Bd*u;
    y_pred(k)=C*x_pred(:,k);

    P_pred=Ad*P*Ad'+Q;
    P_y=C*P_pred*C'+R;
    P_xy=P_pred*C';

    L=P_xy/P_y;
    x_est(:,k)=x_pred(:,k)+L*(y(k)-y_pred(k));
    P=P_pred-L*P_y*L';
end

subplot(2,1,1)
plot(t,x(1,:),'b')
%hold on
%plot(t,y,'r')
hold on
plot(t,x_est(1,:),'k')
legend('x1','x1 estimated')
subplot(2,1,2)
plot(t,x(2,:),'b')
hold on
plot(t,x_est(2,:),'k')
legend('x2','x2 estimated')
