clear;
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
    Magn(i,:)=(GetCbn([roll(i),pitch(i),yaw(i)])*mag(i,:)')';
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

q=myA2Q([roll,pitch,yaw]);

%%
x = [q,0,0,0]; % q, bias of gyro

%init recorder
angleRecorder=zeros(size(acc,1),3);
quatRecorder=zeros(size(acc,1),4);
biasRecorder = zeros(size(acc,1),3);
%set parameter
g=-9.75;
P=eye(7,7);
Q=0.0012*eye(3);
% R=diag([4 4 4 400 400 400]);
R = diag([4 4 4]);

for n=1:size(acc,1)
    [angleRecorder(n,3),angleRecorder(n,2),angleRecorder(n,1)]=quat2angle(x(1:4));
    quatRecorder(n,:)=x(1:4);
    biasRecorder(n,:) = x(5:7);
    
    A=GetA(x, gyrRate(n,:),dT(n));
    G=GetG(x, dT(n));
    x = INS_update(x, [gyrRate(n,:)-x(5:7),dT(n)]);
    P=A*P*A' + G*Q*G';
    
    H=GetH(x,MagReal,g);
    K = P*H'/(H*P*H'+R);
%     obState=[acc(n,:),mag(n,:)];%1*6
    obState = [acc(n,:)];
    
    Cnb=Quat2Cnb(x(1:4));
%     Hq=[Cnb*[0;0;g];Cnb*MagReal'];%6*1
    Hq = Cnb*[0;0;g];
    x=x+(K*(obState'-Hq))';
    P=(eye(7)-K*H)*P;
    x(1:4)=x(1:4)/sqrt(x(1:4)*x(1:4)');
end

index=find(angleRecorder(:,3)<0);
angleRecorder(index,3)=angleRecorder(index,3)+2*pi;

figure(1);
subplot(2,1,1);
hold off;
plot(quatRecorder(:,1));
hold on;
plot(quatRecorder(:,2),'r');
plot(quatRecorder(:,3),'g');
plot(quatRecorder(:,4),'k');
subplot(2,1,2);
hold off;
plot(biasRecorder(:,1));
hold on;
plot(biasRecorder(:,2),'r');
plot(biasRecorder(:,3),'g');

figure(2);hold off;
subplot(2,1,1);hold off;
plot(angleRecorder(:,1)*57.3);
grid on;
hold on;
plot(angleRecorder(:,2)*57.3,'r');
plot(angleRecorder(:,3)*57.3,'g');

subplot(2,1,2);hold off;
plot(acc(:,1));
hold on;
grid on;
plot(acc(:,2),'r');
plot(acc(:,3),'g');