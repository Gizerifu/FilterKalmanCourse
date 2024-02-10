%% LKF Guel cortez 2024
% LKF for cart model


clearvars;
close all;

b=1; m=2; k_m=5; Ts=1e-3; L=30;
Cart_generative_model;

plot(t,x(1,:))
hold on;

Q=[w,w]*[w,w]';
R=v^2;
P=1e4*eye(2);
alpha=0.1;

x_pred=zeros([2,length(t)]);
x_est=zeros([2,length(t)]);
y_pred=zeros([1,length(t)]);

x_pred(:,1)=[6,2.1]';

u_c = zeros([2,length(t)]);
k_c = [10 1];
xDedesada = [6 0];
%x(:,1)=[2,3];

for k=2:length(t)

    x(:,k)=Ad*x(:,k-1)+Bd*u_c(k-1)+w*randn([2,1]);
    y(k)=C*x(:,k)+v*randn;

    x_pred(:,k)=Ad*x_pred(:,k-1)+Bd*u_c(k-1);
    y_pred(k)=C*x_pred(:,k);

    P_pred=Ad*P*Ad'+Q;
    P_y=C*P_pred*C'+R;
    P_xy=P_pred*C';

    L=P_xy/P_y;
    x_est(:,k)=x_pred(:,k)+L*(y(k)-y_pred(k));
    P=P_pred-L*P_y*L';
  
    %u_c = k_c(1)*(xDedesada(1)-x_est(1,k)) + k_c(2)*(xDedesada(2)-x_est(2,k)) 
    
    u_c(k) = (k_c(1)*(xDedesada(1)-y_pred(k)) + k_c(2)*(xDedesada(2)-y_pred(k)));
    % if k<100
    % Q=(1-alpha)*Q+alpha*L*(y(k)-y_pred(k))*(y(k)-y_pred(k))'*L';
    % end
end

plot(t,x(1,:))
figure
subplot(3,1,1)
plot(t,x(1,:),'b')
hold on
plot(t,y,'r')
hold on
plot(t,x_est(1,:),'k')
subplot(3,1,2)
plot(t,x(2,:),'b')
hold on
plot(t,x_est(2,:),'k')
subplot(3,1,3)
plot(t,y,t,u_c)
legend('Salida','Entrada')
