function [out] = PolyCurve(t,n,inx)
%CURVATURE Summary of this function goes here
%   Detailed explanation goes here
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


