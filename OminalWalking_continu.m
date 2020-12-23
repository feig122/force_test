function [sys,x0,str,ts]=OminalWalking_continu(t,x,u,flag)
switch flag
case 0
    [sys,x0,str,ts]=mdlInitializeSizes
case 3
    sys=mdlOutputs(t,x,u);
case {1, 2, 4, 9 }
    sys = [];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 36;
sizes.NumInputs      = 0;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 0;
sys=simsizes(sizes);
x0=[];
str=[];
ts=[];



function sys=mdlOutputs(t,x,u)
global Point;  %储存给每条腿的自由度
global time_pre;
global runcount;
global StateTime;
global StateInitTime;
global StateTime_Last;
global StateTriger;
global WalkingState;
global Time_Flight;
global Time_CentreMove;

global StartPX;
global StartPY;
global StartPZ;
global EndPX;
global EndPY;

global LF;
global RF;
global LH;
global RH;
global RM;
global LM;

global RM_F;
global LM_F;
RM_F=1;
LM_F=2;

global Vx;
global Vy;
global Va;
global Vx1;
global Vy1;
global Va1;

global Incre_X;
global Incre_Y;

global CorX;
global CorY;

%global Point_P;
global L;
global dX dY gaitCount yaw;
global lastdX lastdY

sys=zeros(36,1);
deltaYaw = zeros(6,1);
Time_Flight=1;
Time_CentreMove=1;
% A=0.1;  
% S=0.2;  
L=0.72;
H=0.1;  
% Vagive=0.04;
if t<=1  %Falling down 
    for i=1:6
        Point(i,3)=0;
        Point(i,1)=0;  
        Point(i,2)=0;  
    end
    runcount=1;
    StateTriger=1;  %状态标志
    Vx=0.000001;  
    Vy=0;
    Va=0.05;%原地转向

    Vx1=0.1;
    Vy1=0;
    Va1=0; %直线
    
    %originYaw = IMU(yaw)
elseif t>1    
     if StateTriger==1   %状态完成标志
            StateTriger=0;
            StateInitTime=t;    %时间变量更新
            StateTime_Last=0;  %每一步的持续时间
            
            if WalkingState~=LM_F   %LM=2，不等于
                WalkingState=WalkingState+1;
            else
                WalkingState=RM_F;   %RM_F=1，是第一个状态
                gaitCount = gaitCount +1; 
            end
            
            %--------FEI
            %--------read IMU
            %currentYaw = IMU(yaw);
            %yaw = currentYaw - originYaw;
            lastdX = zeros(6,1);
            lastdY = zeros(6,1);
            yaw = gaitCount*2 * Va; 
            dX = 0.1*cos(yaw);
            dY = 0.1*sin(yaw);
            
            switch WalkingState
                case RM_F %1
                    CorX(RM)=0;
                    CorY(RM)=-L;
                    CorX(LF)=sqrt(3)*L/2;
                    CorY(LF)=L/2;
                    CorX(LH)=-sqrt(3)*L/2;
                    CorY(LH)=L/2;
                case LM_F  %2
                    CorX(LM)=0;
                    CorY(LM)=L;
                    CorX(RF)=sqrt(3)*L/2;
                    CorY(RF)=-L/2;
                    CorX(RH)=-sqrt(3)*L/2;
                    CorY(RH)=-L/2;
            end
            %为什么角速度会改变?
            
            %选择新的落地点(以身体中心为原点，在身体坐标系下计算的增量PE)
            for LegPosition=1:6
                if LegPosition==LF  
                    StartPX(LegPosition)=Point(LegPosition,1)+sqrt(3)*L/2;%0.5;   %point是在单腿坐标系内的坐标
                    StartPY(LegPosition)=Point(LegPosition,2)+L/2;%0.5;
                    StartPZ(LegPosition)=Point(LegPosition,3);  
                    %[Ox,Oy,R,PE]=CircleCal(1.5*Time_CentreMove,Vx,Vy,Va,0+0.5,0+0.5);
                    [Ox,Oy,R,PE]=CircleCal(Time_CentreMove,Vx,Vy,Va,0+sqrt(3)*L/2,0+L/2);
                    EndPX(LegPosition)=PE(1)+dX;%----FEI
                    EndPY(LegPosition)=PE(2)+dY;%----FEI
                elseif LegPosition==LH  
                    StartPX(LegPosition)=Point(LegPosition,1)-sqrt(3)*L/2;
                    StartPY(LegPosition)=Point(LegPosition,2)+L/2;
                    StartPZ(LegPosition)=Point(LegPosition,3);                   
                    %[Ox,Oy,R,PE]=CircleCal(0.5*Time_CentreMove,Vx,Vy,Va,0-0.5,0+0.5);
                    [Ox,Oy,R,PE]=CircleCal(Time_CentreMove,Vx,Vy,Va,0-sqrt(3)*L/2,0+L/2);
                    EndPX(LegPosition)=PE(1)+dX;%----FEI
                    EndPY(LegPosition)=PE(2)+dY;%----FEI  
                elseif LegPosition==RF  
                    StartPX(LegPosition)=Point(LegPosition,1)+sqrt(3)*L/2;
                    StartPY(LegPosition)=Point(LegPosition,2)-L/2;
                    StartPZ(LegPosition)=Point(LegPosition,3);                     
                    %[Ox,Oy,R,PE]=CircleCal(1.5*Time_CentreMove,Vx,Vy,Va,0+0.5,0-0.5);
                    [Ox,Oy,R,PE]=CircleCal(Time_CentreMove,Vx,Vy,Va,0+sqrt(3)*L/2,0-L/2);
                    EndPX(LegPosition)=PE(1)+dX;%----FEI
                    EndPY(LegPosition)=PE(2)+dY;%----FEI  
                elseif LegPosition==RH  
                    StartPX(LegPosition)=Point(LegPosition,1)-sqrt(3)*L/2;
                    StartPY(LegPosition)=Point(LegPosition,2)-L/2;
                    StartPZ(LegPosition)=Point(LegPosition,3);                    
                    %[Ox,Oy,R,PE]=CircleCal(0.5*Time_CentreMove,Vx,Vy,Va,0-0.5,0-0.5);
                    [Ox,Oy,R,PE]=CircleCal(Time_CentreMove,Vx,Vy,Va,0-sqrt(3)*L/2,0-L/2);
                    EndPX(LegPosition)=PE(1)+dX;%----FEI
                    EndPY(LegPosition)=PE(2)+dY;%----FEI
                    deltaYaw(LegPosition) = atan2(PE(1),PE(2));
                elseif LegPosition==RM  
                    StartPX(LegPosition)=Point(LegPosition,1);
                    StartPY(LegPosition)=Point(LegPosition,2)-L;
                    StartPZ(LegPosition)=Point(LegPosition,3);                    
                    %[Ox,Oy,R,PE]=CircleCal(0.5*Time_CentreMove,Vx,Vy,Va,0-0.5,0-0.5);
                    [Ox,Oy,R,PE]=CircleCal(Time_CentreMove,Vx,Vy,Va,0,0-L);
                    EndPX(LegPosition)=PE(1)+dX;%----FEI
                    EndPY(LegPosition)=PE(2)+dY;%----FEI
                elseif LegPosition==LM  
                    StartPX(LegPosition)=Point(LegPosition,1);
                    StartPY(LegPosition)=Point(LegPosition,2)+L;
                    StartPZ(LegPosition)=Point(LegPosition,3);                    
                    %[Ox,Oy,R,PE]=CircleCal(0.5*Time_CentreMove,Vx,Vy,Va,0-0.5,0-0.5);
                    [Ox,Oy,R,PE]=CircleCal(Time_CentreMove,Vx,Vy,Va,0,0+L);
                    EndPX(LegPosition)=PE(1)+dX;%----FEI
                    EndPY(LegPosition)=PE(2)+dY;%----FEI
                end
            end                
    end
        StateTime=t-StateInitTime;  
            switch WalkingState
                case RM_F  
                    for LegID=1:6    %身体重心偏移
                        if LegID==LM||LegID==RF||LegID==RH 
                            Transit_k=0.5-0.5*cos(pi*StateTime/Time_CentreMove);
                            [Ox,Oy,R,PE]=CircleCal(StateTime-StateTime_Last,Vx,Vy,Va,CorX(LegID),CorY(LegID));
                            Incre_X(LegID)=PE(1)-CorX(LegID)+(dX*Transit_k-lastdX(LegID));%+(PE1(1)-CorX1(LegID));
                            Incre_Y(LegID)=PE(2)-CorY(LegID)+(dY*Transit_k-lastdY(LegID));%+(PE1(2)-CorY1(LegID));
                            lastdX(LegID) = dX*Transit_k;
                            lastdY(LegID) = dY*Transit_k;
                            CorX(LegID)=PE(1);%+PE1(1);
                            CorY(LegID)=PE(2);%+PE1(2);
                        end
                    end 
                    Point(LM,1)=Point(LM,1)-Incre_X(LM);
                    Point(LM,2)=Point(LM,2)-Incre_Y(LM);
                    Point(RF,1)=Point(RF,1)-Incre_X(RF);
                    Point(RF,2)=Point(RF,2)-Incre_Y(RF);
                    Point(RH,1)=Point(RH,1)-Incre_X(RH);
                    Point(RH,2)=Point(RH,2)-Incre_Y(RH);
                case LM_F
                    for LegID=1:6
                        if LegID==RM||LegID==LF||LegID==LH 
                            Transit_k=0.5-0.5*cos(pi*StateTime/Time_CentreMove);
                            [Ox,Oy,R,PE]=CircleCal(StateTime-StateTime_Last,Vx,Vy,Va,CorX(LegID),CorY(LegID));
                            Incre_X(LegID)=PE(1)-CorX(LegID)+(dX*Transit_k-lastdX(LegID));%+(PE1(1)-CorX1(LegID));
                            Incre_Y(LegID)=PE(2)-CorY(LegID)+(dY*Transit_k-lastdY(LegID));%+(PE1(2)-CorY1(LegID)); 
                            lastdX(LegID) = dX*Transit_k;
                            lastdY(LegID) = dY*Transit_k;
                            CorX(LegID)=PE(1);%+PE1(1);  
                            CorY(LegID)=PE(2);%+PE1(2);
                        end
                    end
                    Point(RM,1)=Point(RM,1)-Incre_X(RM);
                    Point(RM,2)=Point(RM,2)-Incre_Y(RM);
                    Point(LF,1)=Point(LF,1)-Incre_X(LF);
                    Point(LF,2)=Point(LF,2)-Incre_Y(LF);
                    Point(LH,1)=Point(LH,1)-Incre_X(LH);
                    Point(LH,2)=Point(LH,2)-Incre_Y(LH);
            end
            switch WalkingState    %摆动相轨迹（足端坐标系）
                case RM_F
                    Point(RM,1)=Transit(StateTime,Time_Flight,StartPX(RM),EndPX(RM));
                    Point(RM,2)=Transit(StateTime,Time_Flight,StartPY(RM)+L,EndPY(RM)+L);
                    Point(RM,3)=Lift(StateTime,Time_Flight,StartPZ(RM),-H);
                    Point(LF,1)=Transit(StateTime,Time_Flight,StartPX(LF)-sqrt(3)*L/2,EndPX(LF)-sqrt(3)*L/2);  %摆动相
                    Point(LF,2)=Transit(StateTime,Time_Flight,StartPY(LF)-L/2,EndPY(LF)-L/2);
                    Point(LF,3)=Lift(StateTime,Time_Flight,StartPZ(LF),-H);
                    Point(LH,1)=Transit(StateTime,Time_Flight,StartPX(LH)+sqrt(3)*L/2,EndPX(LH)+sqrt(3)*L/2);
                    Point(LH,2)=Transit(StateTime,Time_Flight,StartPY(LH)-L/2,EndPY(LH)-L/2);
                    Point(LH,3)=Lift(StateTime,Time_Flight,StartPZ(LH),-H);
                case LM_F
                    Point(LM,1)=Transit(StateTime,Time_Flight,StartPX(LM),EndPX(LM));
                    Point(LM,2)=Transit(StateTime,Time_Flight,StartPY(LM)-L,EndPY(LM)-L);
                    Point(LM,3)=Lift(StateTime,Time_Flight,StartPZ(LM),-H);
                    Point(RF,1)=Transit(StateTime,Time_Flight,StartPX(RF)-sqrt(3)*L/2,EndPX(RF)-sqrt(3)*L/2);
                    Point(RF,2)=Transit(StateTime,Time_Flight,StartPY(RF)+L/2,EndPY(RF)+L/2);
                    Point(RF,3)=Lift(StateTime,Time_Flight,StartPZ(RF),-H);
                    Point(RH,1)=Transit(StateTime,Time_Flight,StartPX(RH)+sqrt(3)*L/2,EndPX(RH)+sqrt(3)*L/2);
                    Point(RH,2)=Transit(StateTime,Time_Flight,StartPY(RH)+L/2,EndPY(RH)+L/2);
                    Point(RH,3)=Lift(StateTime,Time_Flight,StartPZ(RH),-H);
            end
            
            if StateTime>=Time_Flight   %摆动相时间约束
                StateTriger=1;
            end                  
        StateTime_Last=StateTime;  %增量式轨迹规划
end

    sys((1-1)*6+1)=Point(RF,1);  
    sys((1-1)*6+2)=Point(RF,2);
    sys((1-1)*6+3)=Point(RF,3);
    
    sys((2-1)*6+1)=Point(RM,1);   
    sys((2-1)*6+2)=Point(RM,2);
    sys((2-1)*6+3)=Point(RM,3);
    
    sys((3-1)*6+1)=Point(RH,1);   
    sys((3-1)*6+2)=Point(RH,2);
    sys((3-1)*6+3)=Point(RH,3);   
   
    sys((4-1)*6+1)=Point(LH,1);   
    sys((4-1)*6+2)=Point(LH,2);
    sys((4-1)*6+3)=Point(LH,3);   

    sys((5-1)*6+1)=Point(LM,1);   
    sys((5-1)*6+2)=Point(LM,2);
    sys((5-1)*6+3)=Point(LM,3);  
    
    sys((6-1)*6+1)=Point(LF,1);   
    sys((6-1)*6+2)=Point(LF,2);
    sys((6-1)*6+3)=Point(LF,3); 

for i=1:6   %旋转自由度均为0
    sys((i-1)*6+4)=0;
    sys((i-1)*6+5)=0;
    sys((i-1)*6+6)=0;   
end

time_pre=t;  %持续时间？？

for i=1:1:6
    for j=1:1:6
     Point_P(i,j)=sys((i-1)*6+j);
    end
end

