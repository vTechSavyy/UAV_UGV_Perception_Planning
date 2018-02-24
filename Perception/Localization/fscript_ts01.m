function [fscript,dfscript_dx,dfscript_dvtil] = ...
                                fscript_ts01(t,x,u,vtil,idervflag)
%
%  Copyright (c) 2002 Mark L. Psiaki.  All rights reserved.  
%
% 
%  This function gives a dummy test case for the nonlinear numercial
%  integration function c2dnonlinear.m.  It is a linear case.
%  Equivalent outputs to those of c2dnonlinear.m should be derivable as
%
%             sysmodel_ct = ss(A,D,eye(4),zeros(4,3));
%             sysmodel_dt = c2d(sysmodel_ct,(tkp1-tk),'zoh');
%             [dfprinted_dxk,dfprinted_dvk] = ssdata(sysmodel_dt);
%             fprinted = dfprinted_dxk*xk + dfprinted_dvk*vk;
%
%  or using the old call format of c2d:
%
%             [dfprinted_dxk,dfprinted_dvk] = c2d(A,D,(tkp1-tk));
%             fprinted = dfprinted_dxk*xk + dfprinted_dvk*vk;
%
%  The differential equation in question is the following linear time-
%  invariant differential equation:
%
%      xdot = A*x + D*vtil
%
%
%  Inputs:
%
%    t               The time at which xdot is to be known.
%
%    x               The 4x1 state vector at time t.
%
%    u               The 0x1 control vector at time t.
%
%    vtil            The 3x1 process noise disturbance vector at time t.
%
%    idervflag       A flag that tells whether (idervflag = 1) or not
%                    (idervflag = 0) the partial derivatives 
%                    dfscript_dx and dfscript_dvtil must be calculated.
%                    If idervflag = 0, then these outputs will be
%                    empty arrays.
%  
%  Outputs:
%
%    fscript         The time derivative of x at time t as determined
%                    by the differential equation.
%
%    dfscript_dx     The partial derivative of fscript with respect to
%                    x.  This is a Jacobian matrix.  It is evaluated and
%                    output only if idervflag = 1.  Otherwise, an
%                    empty array is output.
%
%    dfscript_dvtil  The partial derivative of fscript with respect to
%                    vtil.  This is a Jacobian matrix.  It is evaluated and
%                    output only if idervflag = 1.  Otherwise, an
%                    empty array is output.
%

%
%  Set up the linear system matrices.
%
   A = ...
     [-0.43256481152822, -1.14647135068146,  0.32729236140865, ...
                    -0.58831654301419;...
      -1.66558437823810,  1.19091546564300,  0.17463914282092, ...
                     2.18318581819710;...
       0.12533230647483,  1.18916420165210, -0.18670857768144, ...
                    -0.13639588308660;...
       0.28767642035855, -0.03763327659332,  0.72579054829330, ...
                     0.11393131352081];
   D = ...
     [ 1.06676821135919,  0.29441081639264, -0.69177570170229;...
       0.05928146052361, -1.33618185793780,  0.85799667282826;...
      -0.09564840548367,  0.71432455181895,  1.25400142160253;...
      -0.83234946365002,  1.62356206444627, -1.59372957644748];
%
%  Calculate the outputs.
%
   fscript = A*x + D*vtil;
   
   
   if idervflag == 1
      dfscript_dx = A;
      dfscript_dvtil = D;
   else
      dfscript_dx = [];
      dfscript_dvtil = [];
   end