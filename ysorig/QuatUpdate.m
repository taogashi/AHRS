function y=QuatUpdate(q,w,dt)
sigma = [0 -w(1)*dt -w(2)*dt -w(3)*dt;w(1)*dt 0 w(3)*dt -w(2)*dt;w(2)*dt -w(3)*dt 0 w(1)*dt;w(3)*dt w(2)*dt -w(1)*dt 0];
y = expm(sigma/2)*q;%matlab 自带的矩阵指数计算函数
end