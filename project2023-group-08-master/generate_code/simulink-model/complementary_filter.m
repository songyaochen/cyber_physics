function [t_hat,phi_hat]=complementary_filter(gamma,fx,fy,fz,gx,gy,gz)
    h=0.01;
    n=size(fx,1);
    phi_hat=zeros(n+1,1);
    for i=2:1:n+1
        phi_hat(i)=(1-gamma)*atan2(fy(i-1),fz(i-1))+gamma*(phi_hat(i-1)+h*gx(i-1)*(pi/180));
    end

    t_hat=zeros(n+1,1);
    for i=2:1:n+1
        t_hat(i)=(1-gamma)*atan2(-fx(i-1),sqrt(fy(i-1)^2+fz(i-1)^2))+gamma*(t_hat(i-1)+h*gy(i-1)*(pi/180));
    end
end
