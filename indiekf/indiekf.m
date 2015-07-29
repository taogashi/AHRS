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

x = [q, 0, 0, 0];

w_k_1 = data(1,1:3)/57.3;

%init recorder
angleRecorder=zeros(size(acc,1),3);
quatRecorder=zeros(size(acc,1),4);
biasRecorder = zeros(size(acc,1),3);

for n = 1:size(acc,1)
    % record
    [angleRecorder(n,3),angleRecorder(n,2),angleRecorder(n,1)]=quat2angle(x(1:4));
    quatRecorder(n,:)=x(1:4);
    biasRecorder(n,:) = x(5:7);
    
    % predict
    x = INS_update(x, [(gyrRate(n,:)-x(5:7)+w_k_1)/2,dT(n)]); 
    A=GetA(x, gyrRate(n,:),dT(n));
    G=GetG(x, dT(n));
    P=A*P*A' + G*Q*G';
    
    % update
    H=GetH(x,MagReal,g);
    K = P*H'/(H*P*H'+R);
    obState=[acc(n,:),mag(n,:)];%1*6
    Cnb=Quat2Cnb(x(1:4));
    Hq=[Cnb*[0;0;g];Cnb*MagReal'];%6*1
    x = x+(K*(obState'-Hq))';
    w_k_1 = gyrRate(n,:) - x(5:7);
    P=(eye(7)-K*H)*P;
    x(1:4)=x(1:4)/sqrt(x(1:4)*x(1:4)');
end
