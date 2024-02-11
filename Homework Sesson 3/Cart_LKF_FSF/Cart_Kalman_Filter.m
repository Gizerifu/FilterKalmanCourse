%% LKF Guel cortez 2024
% LKF for cart model


clearvars;
close all;

b=1; m=2; k_m=5; Ts=1e-3; L=30;
%Cart_generative_model;

A=[[0,1];[-k_m/m,-b/m]];
B=[0,1/m]';
C=[1,0];
D=0;

Ad=eye(2)+Ts*A;
Bd=Ts*B;
t=0:Ts:L;
x(:,1)=[6,3]';
w=5e-6;%3e-5;
v=1e-3;
Q=[w,w]*[w,w]';
R=v^2;
P=1e4*eye(2);
alpha=0.1;

x_pred=zeros([2,length(t)]);
x_est=zeros([2,length(t)]);
y_pred=zeros([1,length(t)]);

x_pred(:,1)=[0,0]';

u_c = zeros([2,length(t)]);
k_c = [5 1];
xDeseada = [50 0];
x_c = zeros([2,length(t)]);
y_c = zeros([2,length(t)]);

for k=2:length(t)

    u_c(k) = k_c(1)*(xDeseada(1)-x_est(1,k)) + k_c(2)*(xDeseada(2)-x_est(2,k)) ;
    x_c(:,k)=Ad*x_c(:,k-1)+Bd*u_c(k-1)+w*randn([2,1]);
    y_c(k)=C*x_c(:,k)+v*randn;

    x_pred(:,k)=Ad*x_pred(:,k-1)+Bd*u_c(k-1);
    y_pred(k)=C*x_pred(:,k);

    P_pred=Ad*P*Ad'+Q;
    P_y=C*P_pred*C'+R;
    P_xy=P_pred*C';

    L=P_xy/P_y;
    x_est(:,k)=x_pred(:,k)+L*(y_c(k)-y_pred(k));
    P=P_pred-L*P_y*L';
    
    % if k<100
    % Q=(1-alpha)*Q+alpha*L*(y_c(k)-y_pred(k))*(y_c(k)-y_pred(k))'*L';
    % end
end

subplot(3,1,1)
plot(t,x_c(1,:),'r')
hold on
plot(t,y_c,'k')
hold on
plot(t,x_est(1,:),'y')
legend('x1 ', 'y','x1 estimated')
subplot(3,1,2)
plot(t,x_c(2,:))
hold on
plot(t,x_est(2,:))
legend('x2 ','x2 estimate ')
subplot(3,1,3)
plot(t,y_c,t,u_c)
legend('Salida','Entrada')
