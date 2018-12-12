function smoothPath = path2smoothPath(path)
% This is the algorithm for smoothing the path given in moroparams.m -file.
% We have implemented all the TODO-parts according to the instructions and
% removed the non-necessary lines.

        data.x = path(:,1)';
        data.y = path(:,2)';

        n_Curve = 4 ; % order of the used polynomials 
        NN = 40 ; % number of discretization per segment 
        w = 1e-3; % relative wait of smoothness and tracking: larger w, more weight is given to smoothness (try different values and see how they affect the path)

        % PolyCurve -function is implemented in a separate file!
        TPvec = @(t) PolyCurve(t,n_Curve,1); % TPvec=P(s) 
        TVvec = @(t) PolyCurve(t,n_Curve,2); %TVvec=V(s)
        TAvec = @(t) PolyCurve(t,n_Curve,3); %TAvec=A(s)
        TJvec = @(t) PolyCurve(t,n_Curve,4); %TJvec=J(jdelta_s)

        DesiredPath  = @(t,inx) [data.x(inx)*(1-t) + data.x(inx+1)*t;...
            data.y(inx)*(1-t) + data.y(inx+1)*t];% line connecting the way points

        [~, Nseg] = size(data.x); % number of segments
        Nseg = Nseg -1;

        Tseg = linspace(0,1,NN); % Descritize each segment to NN steps, starting from 0 and ending to 1. Hint: use linespace function

        DesiredPathWhole = zeros(2, Nseg*NN); % 2D paths (x,y) points for all the descritized points in all of the segments

        Qsmooth_x = zeros((n_Curve+1)*Nseg,(n_Curve+1)*Nseg);
        Qtracking_x = zeros((n_Curve+1)*Nseg,(n_Curve+1)*Nseg);
        qtracking_x = zeros((n_Curve+1)*Nseg,1);
        qtracking_y = zeros((n_Curve+1)*Nseg,1);

        for seg_inx = 1:Nseg
            INX = (seg_inx-1)*(n_Curve+1)+1:seg_inx*(n_Curve+1);
            for del_t_inx= 1:NN
                t = Tseg(del_t_inx);
                DesiredPathWhole(:,(seg_inx-1)*NN + del_t_inx)= DesiredPath(t,seg_inx); % DesiredPathWhole=P_d(j delta_s) slide 18
                % Q matrix of the jerk cost (slide 17)
                Qjrk = TJvec(t)*TJvec(t)';
                % Q matrix for the tracking error cost (slide 18)
                Qpos = TPvec(t)*TPvec(t)';

                Qsmooth_x(INX,INX) = Qsmooth_x(INX,INX) + w*Qjrk;
                Qtracking_x(INX,INX) = Qtracking_x(INX,INX) + Qpos;

                 % from slide 18 complete the missing as a function of (t). 
                 qtracking_x(INX) = qtracking_x(INX) - 2*(TPvec(t))*DesiredPathWhole(1,(seg_inx-1)*NN + del_t_inx);
                 qtracking_y(INX) = qtracking_y(INX) - 2*(TPvec(t))*DesiredPathWhole(2,(seg_inx-1)*NN + del_t_inx);
             end
        end

        % equality constraints: Aeq*C = beq

        n_constrants = 4+3*(Nseg-1); 
        Aeq_x = zeros(n_constrants,(n_Curve+1)*Nseg);
        beq_x = zeros(size(Aeq_x,1),1);
        beq_y = zeros(size(Aeq_x,1),1);
        c_n = 1; % constraint counter

        % start from zero speed (at t=0)
        % it means that you want your initial velocity to be equal to zero
        seg_inx = 1;
        INX = (seg_inx-1)*(n_Curve+1)+1:seg_inx*(n_Curve+1);
        Aeq_x(c_n,INX) = TVvec(0); % which output from section_2 (position,velocity, acceleration or Jerk)
        beq_x(c_n) = 0;      % the constraint, i.e. the desired value
        c_n = c_n+1;

        % start from initial way point (at t=0)
        % it means that the initial point is the first path point in moroparams.mat
        Aeq_x(c_n,INX) = TPvec(0); % which output from section_2 (position,velocity, acceleration or Jerk)
        beq_x(c_n) = data.x(1); % the constraint, i.e. the desired value
        beq_y(c_n) = data.y(1); % the constraint, i.e. the desired value
        c_n = c_n+1;

        % stop at the end (at t=1)
        % it means that you want your ending velocity to be equal to zero
        seg_inx = Nseg;
        INX = (seg_inx-1)*(n_Curve+1)+1:seg_inx*(n_Curve+1);
        Aeq_x(c_n,INX) = TVvec(1);  % which output from section_2 (position,velocity, acceleration or Jerk)
        beq_x(c_n) = 0;      % the constraint, i.e. the desired value
        c_n = c_n+1;
        % arive at the final way point
        % it means that the end point is the end path point in moroparams.mat
        Aeq_x(c_n,INX) = TPvec(1); % which output from section_2 (position,velocity, acceleration or Jerk)
        beq_x(c_n) = data.x(15);              % the constraint, i.e. the desired value
        beq_y(c_n) = data.y(15);              % the constraint, i.e. the desired value
        c_n = c_n+1;

        % smooth transition at the connection points of the polynomials
        % for each segment the beginning of the next segment should be constrained
        % to be the same as the end of the previous segment
        for SEG_inx= 1:Nseg-1
            seg_inx = SEG_inx; 
            INX = (seg_inx-1)*(n_Curve+1)+1:seg_inx*(n_Curve+1);
            Aeq_x(c_n,INX) = TPvec(1); % pos at the end of the poly
            seg_inx = SEG_inx+1; 
            INX = (seg_inx-1)*(n_Curve+1)+1:seg_inx*(n_Curve+1);
            Aeq_x(c_n,INX) = -TPvec(0); % pos at the start of the poly 
            c_n = c_n+1;

            seg_inx = SEG_inx; INX = (seg_inx-1)*(n_Curve+1)+1:seg_inx*(n_Curve+1);
            Aeq_x(c_n,INX) = TVvec(1); % speed at the end of the poly 
            seg_inx = SEG_inx+1; INX = (seg_inx-1)*(n_Curve+1)+1:seg_inx*(n_Curve+1);
            Aeq_x(c_n,INX) = -TVvec(0); % speed at the start of the poly 
            c_n = c_n+1;

            seg_inx = SEG_inx; INX = (seg_inx-1)*(n_Curve+1)+1:seg_inx*(n_Curve+1);
            Aeq_x(c_n,INX) = TAvec(1); % acc at the end of the poly 
            seg_inx = SEG_inx+1; 
            INX = (seg_inx-1)*(n_Curve+1)+1:seg_inx*(n_Curve+1);
            Aeq_x(c_n,INX) = -TAvec(0); % acc at the start of the poly 
            c_n = c_n+1;
        end

        % minimize the problem using the optimization package
        Q_x = 2*(Qtracking_x + Qsmooth_x);
        Q_y = Q_x;
        Aeq_y = Aeq_x; % note that beq_x is not the same as beq_y
        Copt_x = quadprog(Q_x,qtracking_x,[],[],Aeq_x,beq_x);
        Copt_y = quadprog(Q_y,qtracking_y,[],[],Aeq_y,beq_y);
        Cxy = [Copt_x'; Copt_y']; % coeficinets of poly


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
        % time scaling
        % magnitude of Vel and Acc
        VELmag = sqrt(VEL(1,:).^2+VEL(2,:).^2);
        ACCmag = sqrt(ACC(1,:).^2+ACC(2,:).^2);
        % let assume 
        Vmax = .5; % m/s
        Amax = 1; % m/s/s

        Vmag_max = max (VELmag);
        Amag_max = max (ACCmag);

        % sc is the same as c in the slides (a scaling factor)
        sc = max(Vmax/Vmag_max,Amax/Amag_max);

        VEL_new = sc.*VEL;

        Time = 1/sc*TAU;

        smoothPath.qd = POS;
        smoothPath.qd_dot = VEL_new;
        smoothPath.time = Time;