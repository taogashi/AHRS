clear;
close all;
%cal mag vector in n-cordinate
data=textread('data.txt');
acc=data(:,4:6);
mag=data(:,7:9);

%http://magnetic-declination.com/
%���صش�ƫ�� -6��33��

Gabs=sqrt(sum(acc.*acc,2));
roll=atan2(-acc(:,2),-acc(:,3));
pitch=asin(acc(:,1)./Gabs);
Hx=sum([cos(pitch),sin(pitch),cos(roll).*sin(pitch)].*mag,2);
Hy=sum([zeros(size(roll)),cos(roll),-sin(roll)].*mag,2);
yaw=zeros(size(roll));

index=find(Hy<2 & Hy>-2);
yaw(index)=pi;
indexIndex=find(Hx(:,index)>0);
yaw(index(indexIndex))=0;

index=find(Hy>=2 | Hy<=-2);
yaw(index)=atan(Hx(index)./Hy(index))+pi/2;
indexIndex=find(Hy(index)>0);
yaw(index(indexIndex))=yaw(index(indexIndex))+pi;
yaw=yaw-(6+33/60)/180*pi;
index=find(yaw<0);
yaw(index)=yaw(index)+2*pi;

Magn=zeros(size(mag));
for i=1:length(yaw)
    Magn(i,:) = angle2dcm(yaw(i), pitch(i), roll(i))'*mag(i,:)';
%     Magn(i,:)=(GetCbn([roll(i),pitch(i),yaw(i)])*mag(i,:)')';
end

MagReal=sum(Magn)/length(yaw);

%%
%load data
data=textread('1.txt');
gyrRate=data(1:end-1,1:3)/57.3;
acc=data(2:end,4:6);
mag=data(2:end,7:9);
time=data(:,10);
dT=time(2:end)-time(1:end-1);

%init quatanion
Gabs=sqrt(data(1,4:6)*data(1,4:6)');
roll=atan2(-data(1,5),-data(1,6));
pitch=asin(data(1,4)/Gabs);
Hx=[cos(pitch),sin(pitch),cos(roll)*sin(pitch)]*data(1,7:9)';
Hy=[0,cos(roll),-sin(roll)]*data(1,7:9)';

if(Hy<2 & Hy>-2)
    if Hx>0
        yaw=0;
    else
        yaw=pi;
    end
else
    if Hy<0
        yaw=atan(Hx/Hy)+pi/2;
    else
        yaw=atan(Hx/Hy)+3*pi/2;
    end
end

yaw=yaw-(6+33/60)/180*pi;
if yaw<0
    yaw=yaw+2*pi;
end

cnb = angle2dcm(yaw, pitch, roll);
q = cnb2quat(cnb);

% we always use column vector except library function
x = [q; 0; 0; 0];
P = diag([0.02 0.02 0.02 0.02 0.02 0.02]);
sigr = 0.05/57.3;
sigw = 0.007/57.3;
R = diag([4 4 4 20 20 20]);

w_k_1 = data(1,1:3)'/57.3;

%init recorder
angleRecorder=zeros(size(acc,1),3);
quatRecorder=zeros(size(acc,1),4);
biasRecorder = zeros(size(acc,1),3);
%%
for n = 1:size(acc,1)
    % record
    [angleRecorder(n,3),angleRecorder(n,2),angleRecorder(n,1)]=dcm2angle(quat2cnb(x(1:4)));
    quatRecorder(n,:)=x(1:4)';
    biasRecorder(n,:) = x(5:7)';
    
    dt = dT(n);
    w_k = gyrRate(n,:)'-x(5:7);
    w_avg = 0.5*(w_k_1+w_k);
    % predict
    x(1:4) = NormalizeV((expm(1/2*Omega(w_avg)*dt) + 1/48*(Omega(w_k)*Omega(w_k_1) - Omega(w_k_1)*Omega(w_k))*dt^2) * x(1:4));		% (131)
    if norm(w_avg) < 0.00001 %Too small, may induce numerical instability. Estimating instead.
        Theta = eye(3) - dt*skew(w_avg) + ((dt.^2)/2)*(skew(w_avg)*skew(w_avg));	% (193)
        Psi = -eye(3)*dt + ((dt^2)/2)*skew(w_avg) - ((dt^3)/6)*(skew(w_avg)*skew(w_avg)); % (201)
    else
        Theta = cos(norm(w_avg)*dt)*eye(3) - sin(norm(w_avg)*dt)*skew(w_avg/norm(w_avg)) + (1 -cos(norm(w_avg)*dt))*(w_avg/norm(w_avg))*(w_avg'/norm(w_avg));	% (192)
        Psi = -eye(3)*dt + (1/norm(w_avg).^2)*(1-cos(norm(w_avg)*dt))*skew(w_avg) - (1/norm(w_avg).^3)*(norm(w_avg)*dt - sin(norm(w_avg)*dt))*(skew(w_avg)*skew(w_avg));	%(197)
    end
    Phi = [Theta Psi; zeros(3) eye(3)];	%(187)
    
    if norm(w_avg) < 0.00001  %Too small, may induce numerical instability. Estimating instead.
        Q11 = (sigr.^2)*dt*eye(3) + (sigw.^2)*(eye(3)*dt^3/3 + (dt^5/60)*(skew(w_avg)*skew(w_avg)));	% (212)
        Q12 = -(sigw^2) * ( eye(3)*dt.^2/2 - (dt^3/6)*skew(w_avg) + (dt^4/24)*(skew(w_avg)*skew(w_avg)));	%(213)
    else
        Q11 = (sigr.^2)*dt*eye(3) + (sigw.^2)*( eye(3)*dt.^3/3 + (((norm(w_avg)*dt)^3/3 + 2*sin(norm(w_avg)*dt) - 2*norm(w_avg)*dt )/ (norm(w_avg)^5))*(skew(w_avg)*skew(w_avg)));	% (209)
        Q12 = -(sigw^2) * ( eye(3)*dt.^2/2 - ((norm(w_avg)*dt - sin(norm(w_avg)*dt))/(norm(w_avg)^3))*skew(w_avg) + (((norm(w_avg)*dt)^2/2 + cos(norm(w_avg)*dt) - 1)/(norm(w_avg)^4))*(skew(w_avg)*skew(w_avg)));	% (210)
    end
    Q22 = (sigw^2)*dt*eye(3);	% (211)
    Qd = [Q11 Q12; Q12' Q22];
    
    P = Phi*P*Phi' + Qd;
    
    % update
    cnb = quat2cnb(x(1:4));
    H = [skew(cnb*[0 0 -9.76]') zeros(3,3);
        skew(cnb*MagReal') zeros(3,3)];
    K = P*H'/(H*P*H'+R);
    obState=[acc(n,:),mag(n,:)]';%1*6
    Hq=[cnb*[0;0;-9.76];cnb*MagReal'];%6*1
    
    deltaX = K*(obState-Hq);
    dq = deltaX(1:3)/2;
    db = deltaX(4:6);
    
    if (dq'*dq) > 1
        dq_hat_avg_1 = (1/sqrt(1 + (dq'*dq))) * [dq ; 1];	%(239)
    else
        dq_hat_avg_1 = [dq ; (sqrt(1 - (dq'*dq)))];		%(238)
    end
    q = multiplyQuaternion(dq_hat_avg_1, x(1:4));	%(240)
    
    x(1:4) = q;
    x(5:7) = x(5:7) + db;
    w_k_1 = gyrRate(n,:)' - x(5:7);
    
    P=(eye(6)-K*H)*P;
end

index=find(angleRecorder(:,3)<0);
angleRecorder(index,3)=angleRecorder(index,3)+2*pi;

figure(1);hold off;
plot(quatRecorder(:,1))
hold on
plot(quatRecorder(:,2),'r')
plot(quatRecorder(:,3),'g')
plot(quatRecorder(:,4),'k')

figure(2);hold off;
subplot(2,1,1);hold off;
plot(angleRecorder(:,1)*57.3);
grid on;
hold on;
plot(angleRecorder(:,2)*57.3,'r');
plot(angleRecorder(:,3)*57.3,'g');
subplot(2,1,2);
hold off;
plot(biasRecorder(:,1));
hold on;
plot(biasRecorder(:,2),'r');
plot(biasRecorder(:,3),'g');
