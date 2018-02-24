function [fprinted,dfprinted_dxk,dfprinted_dvk] = ...
    c2dnonlinear_verified(xk,uk,vk,tk,tkp1,nRK,fscriptname,idervflag)
%
%  Copyright (c) 2002 Mark L. Psiaki.  All rights reserved.
%
%
%  This function derives a nonlinear discrete-time dynamics function
%  for use in a nonlinear difference equation via 4th-order
%  Runge-Kutta numerical integration of a nonlinear differential
%  equation.  If the nonlinear differential equation takes the
%  form:
%
%               xdot = fscript{t,x(t),uk,vk}
%
%  and if the initial condition is x(tk) = xk, then the solution
%  gets integrated forward from time tk to time tkp1 using nRK
%  4th-order Runge-Kutta numerical integration steps in order to
%  compute fprinted(k,xk,uk,vk) = x(tkp1).  This function can
%  be used in a nonlinear dynamics model of the form:
%
%        xkp1 = fprinted(k,xk,uk,vk)
%
%  which is the form defined in AOE 6984 lecture for use in a nonlinear
%  extended Kalman filter.
%
%  This function also computes the first partial derivative of
%  fprinted(k,xk,uk,vk) with respect to xk, dfprinted_dxk, and with
%  respect to vk, dfprinted_dvk.
%
%
%  Inputs:
%
%    xk             The state vector at time tk, which is the initial
%                   time of the sample interval.
%
%    uk             The control vector, which is held constant
%                   during the sample interval from time tk to time
%                   tkp1.
%
%    vk             The discrete-time process noise disturbance vector,
%                   which is held constant during the sample interval
%                   from time tk to time tkp1.
%
%    tk             The start time of the numerical integration
%                   sample interval.
%
%    tkp1           The end time of the numerical integration
%                   sample interval.
%
%    nRK            The number of Runge-Kutta numerical integration
%                   steps to take during the sample interval.
%
%    fscriptname    The name of the Matlab .m-file that contains the
%                   function which defines fscript{t,x(t),uk,vk}.
%                   This must be a character string.  For example, if
%                   the continuous-time differential equation model is
%                   contained in the file rocketmodel.m with the function
%                   name rocketmodel, then on input to the present
%                   function fscriptname must equal 'rocketmodel',
%                   and the first line of the file rocketmodel.m
%                   must be:
%
%                   function [fscript,dfscript_dx,dfscript_dvtil] = ...
%                               rocketmodel(t,x,u,vtil,idervflag)
%
%                   The function must be written so that fscript
%                   defines xdot as a function of t, x, u, and vtil
%                   and so that dfscript_dx and dfscript_dvtil are the
%                   matrix partial derivatives of fscript with respect
%                   to x and vtil if idervflag = 1.  If idervflag = 0, then
%                   these outputs must be empty arrays.
%
%    idervflag      A flag that tells whether (idervflag = 1) or not
%                   (idervflag = 0) the partial derivatives
%                   dfprinted_dxk and dfprinted_dvk must be calculated.
%                   If idervflag = 0, then these outputs will be
%                   empty arrays.
%
%  Outputs:
%
%    fprinted       The discrete-time dynamics vector function evaluated
%                   at k, xk, uk, and vk.
%
%    dfprinted_dxk  The partial derivative of fprinted with respect to
%                   xk.  This is a Jacobian matrix.  It is evaluated and
%                   output only if idervflag = 1.  Otherwise, an
%                   empty array is output.
%
%    dfprinted_dvk  The partial derivative of fprinted with respect to
%                   vk.  This is a Jacobian matrix.  It is evaluated and
%                   output only if idervflag = 1.  Otherwise, an
%                   empty array is output.
%

%
%  Prepare for the Runge-Kutta numerical integration by setting up
%  the initial conditions and the time step.

%% Initialize variables:
x = xk;
if idervflag == 1
    nx = size(xk,1);
    nv = size(vk,1);
    F =  eye(nx,nx);      % Partial derivative of fprinted w.r.t x_k
    Gamma = zeros(nx,nv); % Partial derivative of fprinted w.r.t v_k
end
t = tk;

delt = (tkp1 - tk)/nRK;

%  This loop does one 4th-order Runge-Kutta numerical integration step
%  per iteration.  Integrate the state.  If partial derivatives are
%  to be calculated, then the partial derivative matrices simultaneously
%  with the state.

%% Perform Runge Kutta Numerical Integration:

for jj = 1:nRK
    
    if idervflag == 1
        [fscript,dfscript_dx,dfscript_dvtil] = ...
            feval(fscriptname,t,x,uk,vk,1);
        
        
        dFa = (dfscript_dx*F)*delt;
        
        
        dGammaa = (dfscript_dx*Gamma + dfscript_dvtil)*delt;
        
    else
        fscript = feval(fscriptname,t,x,uk,vk,0);
    end
    
    dxa = fscript*delt;
    
    
    if idervflag == 1
        [fscript,dfscript_dx,dfscript_dvtil] = ...
            feval(fscriptname,(t + 0.5*delt),(x + 0.5*dxa),...
            uk,vk,1);
        
        
        dFb = dfscript_dx*(F+0.5*dFa)*delt;
        
        
        dGammab = (dfscript_dx*(Gamma +0.5*dGammaa) + dfscript_dvtil)*delt;
        
        
        
    else
        fscript = feval(fscriptname,(t + 0.5*delt),(x + 0.5*dxa),...
            uk,vk,0);
    end
    
    dxb = fscript*delt;
    
    
    if idervflag == 1
        [fscript,dfscript_dx,dfscript_dvtil] = ...
            feval(fscriptname,(t + 0.5*delt),(x + 0.5*dxb),...
            uk,vk,1);
        
        
        dFc = dfscript_dx*(F+ 0.5*dFb)*delt;
        
        
        dGammac = (dfscript_dx*(Gamma +0.5*dGammab) + dfscript_dvtil)*delt;
        
    else
        fscript = feval(fscriptname,(t + 0.5*delt),(x + 0.5*dxb),...
            uk,vk,0);
    end
    dxc = fscript*delt;
    
    
    if idervflag == 1
        [fscript,dfscript_dx,dfscript_dvtil] = ...
            feval(fscriptname,(t + delt),(x + dxc),...
            uk,vk,1);
        
        dFd = dfscript_dx*(F + dFc)*delt;
        
        
        dGammad = (dfscript_dx*(Gamma +dGammac) + dfscript_dvtil)*delt;
        
        
    else
        fscript = feval(fscriptname,(t + delt),(x + dxc),...
            uk,vk,0);
    end
    dxd = fscript*delt;
    
    
    x = x + (dxa + 2*(dxb + dxc) + dxd)*(1/6);
    if idervflag == 1
        F = F + (dFa + 2*(dFb + dFc) + dFd)*(1/6);
        Gamma = Gamma + ...
            (dGammaa + 2*(dGammab + dGammac) + dGammad)*(1/6);
    end
    
    t = t + delt;
    
end

%%  Assign the results to the appropriate outputs.

fprinted = x;
if idervflag == 1
    dfprinted_dxk = F;
    dfprinted_dvk = Gamma;
else
    dfprinted_dxk = [];
    dfprinted_dvk = [];
end