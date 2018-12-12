%% quadratic programing example
% for advanced robotics and fundamental of robot programming course
% Reza Ghabcheloo, 21.6.2018

% given set of way points generate a smooth trajectory (continuous acceleration) passing almost from all of
% them
% 1) we reduce the problem to that of finding Polynomial curves of order
% n_Curve 
% 2) to each segment of the desired path, we associate one polynomial
% 3) We build a cost that is quadratic so it is fast to calculate
% Cost includes a term for smoothing (square of jerk), and how close the
% trajectory is to the line segments connecting the way points
% 4) constraints are then added to make the velocities and acceleration continuous at the transision 
% points, or start with zero speed or end with zero speed. or start and end at way points; other
% constraints can be easily added as long as they can be added as arguments to quadprog(.)
% velocity and acceleration constraints are addressed when we convert the
% resulting trajectory to actual time trajectory

clear all
% way points example: the code is generic to handle any number of way
% points
load moroparams.mat
% way points: loaded from moroparams.mat
data.x = path(:,1)';%[-4 -3 -1 1 3 5]; 
data.y = path(:,2)';
% we assume 0<t<1 for each segment

n_Curve = 4; % order of Bezier curves or polynomial
NN = 50; % number of discretization per segment 
w = 1e-5; % relative wait of smoothness and tracking: larger w, more weight is given to smoothness

TPvec = @(t) PolyCurve(t,n_Curve,1); % Pos; anonymous function for bases functions, initial time t0: this will help us to concatinate the polynomials
TVvec = @(t) PolyCurve(t,n_Curve,2);
TAvec = @(t) PolyCurve(t,n_Curve,3);
TJvec = @(t) PolyCurve(t,n_Curve,4);

DesiredPath  = @(t,inx) [data.x(inx)*(1-t) + data.x(inx+1)*t;...
    data.y(inx)*(1-t) + data.y(inx+1)*t];% line connecting the way points

Nseg = length(data.x)-1; % number of segments

% Thus, a point on the traj can be calculated as follows
% t = 0.5;
% seg_inx = 1; % index of the path segment; no error checking: t must be in the interval 
% Pos = C(:,seg_inx:seg_inx+n_Curve)*TPvec(t);
% Vel = C(:,seg_inx:seg_inx+n_Curve)*TVvec(t);
% Acc = C(:,seg_inx:seg_inx+n_Curve)*TAvec(t);
% Jrk = C(:,seg_inx:seg_inx+n_Curve)*TJvec(t);
% DesiredPath(t,seg_inx);

% Step1: discretize the time
Tseg = linspace(0,1,NN);
DesiredPathWhole = zeros(2,NN*Nseg); % 2D paths (x,y)

% Step2: calculate Q and q of the quadratic cost: c*Q*c' + q*c
% if there is no coupled costs we can simply build and optimize x and y
% separately:
% Q_x is equal Q_y in this case, but q_x and q_y are differnt since they
% depend on actual trajectory

% next we calculate Q and q: (more details, refer to course slides)
Qsmooth_x = zeros((n_Curve+1)*Nseg,(n_Curve+1)*Nseg);
Qtracking_x = zeros((n_Curve+1)*Nseg,(n_Curve+1)*Nseg);
qtracking_x = zeros((n_Curve+1)*Nseg,1);
qtracking_y = zeros((n_Curve+1)*Nseg,1);

for seg_inx = 1:Nseg
    INX = (seg_inx-1)*(n_Curve+1)+1:seg_inx*(n_Curve+1);
    for del_t_inx= 1:NN
    t = Tseg(del_t_inx);
DesiredPathWhole(:,(seg_inx-1)*NN + del_t_inx)= DesiredPath(t,seg_inx);
% 
Qjrk = TJvec(t)*TJvec(t)';
Qpos = TPvec(t)*TPvec(t)';

 Qsmooth_x(INX,INX) = Qsmooth_x(INX,INX) + w*Qjrk;
 Qtracking_x(INX,INX) = Qtracking_x(INX,INX) + Qpos;
 qtracking_x(INX) = qtracking_x(INX) - 2*TPvec(t)*DesiredPathWhole(1,(seg_inx-1)*NN + del_t_inx);
 qtracking_y(INX) = qtracking_y(INX) - 2*TPvec(t)*DesiredPathWhole(2,(seg_inx-1)*NN + del_t_inx);
     end
end

%% equality constraints: Aeq*C = beq
% 
% in our case, where x and y constraints are the same, we calculate for x
% and copy to y axis constraints
n_constrants = 5+3*(Nseg-1); 
Aeq_x = zeros(n_constrants,(n_Curve+1)*Nseg);
beq_x = zeros(size(Aeq_x,1),1);
beq_y = zeros(size(Aeq_x,1),1);
c_n = 1; % constraint counter

% start from zero speed
seg_inx = 1;
INX = (seg_inx-1)*(n_Curve+1)+1:seg_inx*(n_Curve+1);
Aeq_x(c_n,INX) = TVvec(0)'; 
beq_x(c_n) = 0;
c_n = c_n+1;

% start from initial way point
Aeq_x(c_n,INX) = TPvec(0)'; 
beq_x(c_n) = data.x(1);
beq_y(c_n) = data.y(1);
c_n = c_n+1;

% stop at the end
seg_inx = Nseg;
INX = (seg_inx-1)*(n_Curve+1)+1:seg_inx*(n_Curve+1);
Aeq_x(c_n,INX) = TVvec(1)'; 
beq_x(c_n) = 0;
c_n = c_n+1;
% arive at the final way point
Aeq_x(c_n,INX) = TPvec(1)'; 
beq_x(c_n) = data.x(end);
beq_y(c_n) = data.y(end);
c_n = c_n+1;

% smooth transition at the connection points of the polynomials
for SEG_inx= 1:Nseg-1
seg_inx = SEG_inx; INX = (seg_inx-1)*(n_Curve+1)+1:seg_inx*(n_Curve+1);
Aeq_x(c_n,INX) = TPvec(1)'; % pos at the end of the poly
seg_inx = SEG_inx+1; INX = (seg_inx-1)*(n_Curve+1)+1:seg_inx*(n_Curve+1);
Aeq_x(c_n,INX) = -TPvec(0)'; % pos at the start of the poly 
c_n = c_n+1;

seg_inx = SEG_inx; INX = (seg_inx-1)*(n_Curve+1)+1:seg_inx*(n_Curve+1);
Aeq_x(c_n,INX) = TVvec(1)'; % speed at the end of the poly 
seg_inx = SEG_inx+1; INX = (seg_inx-1)*(n_Curve+1)+1:seg_inx*(n_Curve+1);
Aeq_x(c_n,INX) = -TVvec(0)'; % speed at the start of the poly 
c_n = c_n+1;

seg_inx = SEG_inx; INX = (seg_inx-1)*(n_Curve+1)+1:seg_inx*(n_Curve+1);
Aeq_x(c_n,INX) = TAvec(1)'; % acc at the end of the poly 
seg_inx = SEG_inx+1; INX = (seg_inx-1)*(n_Curve+1)+1:seg_inx*(n_Curve+1);
Aeq_x(c_n,INX) = -TAvec(0)'; % acc at the start of the poly 
c_n = c_n+1;
end

%%
Q_x = 2*(Qtracking_x + Qsmooth_x);
Q_y = Q_x;
Aeq_y = Aeq_x; % note that beq_x is not the same as beq_y
Copt_x = quadprog(Q_x,qtracking_x,[],[],Aeq_x,beq_x);
Copt_y = quadprog(Q_y,qtracking_y,[],[],Aeq_y,beq_y);
Cxy = [Copt_x'; Copt_y']; % coeficinets of poly

% % In case there are coupled costs, we need to build one big optimization as
% % follows:
% Aeq = blkdiag(Aeq_x,Aeq_y);
% beq = [beq_x; beq_y];
% Q= blkdiag(Q_x,Q_y);
% q = [qtracking_x; qtracking_y];
% 
% Copt = quadprog(Q,q,[],[],Aeq,beq);
% Cxy = [Copt(1:Nseg*(n_Curve+1))'; Copt(Nseg*(n_Curve+1)+1:end)'];


% % prepare for plots
POS = zeros(2,Nseg*NN);
VEL = zeros(2,Nseg*NN);
ACC = zeros(2,Nseg*NN);
JRK = zeros(2,Nseg*NN);
TAU = zeros(1,Nseg*NN);

for seg_inx = 1:Nseg
INX = (seg_inx-1)*(n_Curve+1)+1:seg_inx*(n_Curve+1); 
    for del_t_inx= 1:NN
    t = Tseg(del_t_inx);

TAU((seg_inx-1)*NN + del_t_inx)= (seg_inx-1)+t;
POS(:,(seg_inx-1)*NN + del_t_inx) = Cxy(:,INX)*TPvec(t);
VEL(:,(seg_inx-1)*NN + del_t_inx) = Cxy(:,INX)*TVvec(t);
ACC(:,(seg_inx-1)*NN + del_t_inx) = Cxy(:,INX)*TAvec(t);
JRK(:,(seg_inx-1)*NN + del_t_inx) = Cxy(:,INX)*TJvec(t);
    end
end

%%
% time scaling
% magnitude of Vel and Acc
VELmag = sqrt(VEL(1,:).^2+VEL(2,:).^2);
ACCmag = sqrt(ACC(1,:).^2 + ACC(2,:).^2);
% let assume 
Vmax = 4; % m/s
Amax = 10; % m/s/s

Vmag_max = max (VELmag);
Amag_max = max (ACCmag);

sc = max(Vmax/Vmag_max,Amax/Amag_max);

VEL_new = sc*VEL;
VELmag_new = sc*VELmag;

ACC_new = sc^2*ACC;
ACCmag_new = sc^2*ACCmag;

Time = 1/sc*TAU;

%%
% angular speed
CCurvature = zeros(1,Nseg*NN);
for seg_inx=1:Nseg
    INX = (seg_inx-1)*(n_Curve+1)+1:seg_inx*(n_Curve+1); 
    for del_t_inx= 1:NN
    t = Tseg(del_t_inx);
Coef = Cxy(:,INX);
CCurvature((seg_inx-1)*NN + del_t_inx)= Curvature(Coef, t, n_Curve);
    end
end
    
AngVeltime = CCurvature.*VELmag_new;

%%
figure(1)
plot(data.x,data.y,'*-',...
    DesiredPathWhole(1,:),DesiredPathWhole(2,:),'.-')
legend('way points','traj discretization ')

figure(2)
plot(data.x,data.y,'*-',...
    POS(1,:),POS(2,:),'.-')
legend('way points','smooth traj')

figure(3)
plot(Time,VELmag_new,'.-',TAU,VELmag,'.-'), legend('vel scaled', 'vel')

figure(4)
plot(Time,AngVeltime,'.-',Time, CCurvature), legend('angular speed', 'curvature')



%%
return

function out = PolyCurve(t,n,inx) 
% Polynomial arguments: time, order of the Poly, inx (1,2,3,4)=(pos, vel, acc, jerk)
  bc = zeros(n+1,1);
  bv = zeros(n+1,1);
  ba = zeros(n+1,1);
  bj = zeros(n+1,1);
     bc(1) = 1;
    for i = 1 : n
      bc(i+1) = t^i;
      if i>1
      bv(i+1)   = i*t^(i-1);
      elseif i==1
      bv(i+1)   = i;
      end
      
      if i>2
      ba(i+1)   = i*(i-1)*t^(i-2);
      elseif i==2
      ba(i+1)   = i*(i-1);
      end
      
      if i>3
      bj(i+1)   = i*(i-1)*(i-2)*t^(i-3);
      elseif i==3
      bj(i+1)   = i*(i-1)*(i-2);
      end
    end
    
    switch inx
        case 1
            out = bc;
        case 2
            out = bv;
        case 3
            out = ba;
        case 4
            out = bj;
        otherwise
            out = bc;
    end
end

function Cc = Curvature(Coef, t, n_Curve)

Pp = Coef*PolyCurve(t,n_Curve,2);
Ppp = Coef*PolyCurve(t,n_Curve,3);

nPp = sqrt(Pp'*Pp);
T   = Pp/nPp; % tangent vector
N = [-T(2); T(1)]; % normal vector
Cc = Ppp'*N/nPp^2; 

end

