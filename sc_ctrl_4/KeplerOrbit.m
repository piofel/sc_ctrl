classdef KeplerOrbit < GeneralMath & CelestialSystem
    %   Class for storing Kepler Orbit data and methods
    %   Anomaly angle should be formatted separately
    
    properties (SetAccess = private, GetAccess = public)
        coe % Set of Classical Orbital Elements (COE object)
        p % Semilatus rectum
        h % Massless angular momentum
        b % Semi-minor
        n % Mean angular motion
    end
    
    properties (Constant = true, GetAccess = private)
        tol = 10^-4; % Kepler's Equation solution tolerance, quite accurate tolerance: 10^-2
    end
    
    methods
        function KepOrb = KeplerOrbit(coeObj) % Constructor
            KepOrb.coe = coeObj; % coeObj is a COE object
            a = KepOrb.coe.a;
            e = KepOrb.coe.e;
            KepOrb.p = a * (1 - e^2); % Semilatus rectum
            mu_s = KepOrb.GrPrm; 
            p_s = KepOrb.p; % '_s' suffix means separate variable
            KepOrb.h = sqrt(mu_s * p_s);
            KepOrb.b = a * sqrt(1 - e^2);
            KepOrb.n = sqrt(mu_s/a^3);
        end
        function [M] = calcMeanAnomaly(KepOrb, t)
            M0 = KepOrb.coe.M0;
            t0 = KepOrb.coe.t0;
            n_s = KepOrb.n;
            M = M0 + n_s*(t - t0); % Mean anomaly
            M = KepOrb.formatAngle(M);
        end
        function [E] = solveKeplerEqn(KepOrb, t)
            e = KepOrb.coe.e;
            tol_s = KepOrb.tol;
            Me = KepOrb.calcMeanAnomaly(t);
            breakflag = 0;
            E1 = Me;
            E = 0; % Added to override error in embedded function in Simulink
            while breakflag == 0
                %Fixed-point iterative version of Kepler's Equation
                E = Me + e*sin(E1);   
                %Break loop if tolerance is achieved
                if abs(E - E1) < tol_s
                    breakflag = 1;
                end    
                E1 = E;
            end
            E = KepOrb.formatAngle(E);
        end
        function [TA] = calcTrueAnomaly(KepOrb, t)
            e = KepOrb.coe.e;
            E = KepOrb.solveKeplerEqn(t);
            TA = 2 * atan(sqrt((1+e)/(1-e)) * tan(E/2)); % True anomaly
            TA = KepOrb.formatAngle(TA);
        end
        function [dTA] = calcTrueAnomalyRate(KepOrb, t)
            % Calculates true anomaly first derivative
            mu = KepOrb.GrPrm;
            p_s = KepOrb.p;
            r = KepOrb.calcRadialDistance(t);
            dTA = sqrt(mu * p_s / r^4);
        end
        function [r] = calcRadialDistance(KepOrb, t)
            % Calculates the distance between the Earth centre and orbiting body
            p_s = KepOrb.p;
            e = KepOrb.coe.e;
            TA = KepOrb.calcTrueAnomaly(t);
            r = p_s/(1 + e*cos(TA));
        end
        function [vr] = calcRadialVelocity(KepOrb, t)
            % Calculates the radial velocity of the orbiting body
            mu = KepOrb.GrPrm;
            h_s = KepOrb.h;
            e = KepOrb.coe.e;
            TA = KepOrb.calcTrueAnomaly(t);
            vr = mu * e * sin(TA) / h_s;
        end
        function [theta] = calcArgumentOfLatitude(KepOrb, t)
            w = KepOrb.coe.w;
            f = KepOrb.calcTrueAnomaly(t);
            theta = w + f;
        end
    end % methods
end % classdef
